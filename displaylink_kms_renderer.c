#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <limits.h>

/* DRM/KMS */
#include <xf86drm.h>
#include <xf86drmMode.h>
#include <gbm.h>
#include <drm_fourcc.h>

/* EGL */
#include <EGL/egl.h>
#include <EGL/eglext.h>

/* OpenGL ES 2.0 */
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>

#include "breezy_imu.h"
#include "breezy_overlay.h"
#include "common.h"
#include "display_renderer.h"
#include "server.h"
#include "display_placement.h"
#include "usb_gadget.h"
#include "link_services.h"

/* Global stop flag — declared extern in kms_common.h */
volatile sig_atomic_t stop_requested = 0;

static void on_signal(int signo)
{
	(void)signo;
	stop_requested = 1;
}

static void install_signal_handlers(void)
{
	struct sigaction action;

	memset(&action, 0, sizeof(action));
	action.sa_handler = on_signal;
	sigemptyset(&action.sa_mask);
	sigaction(SIGINT, &action, NULL);
	sigaction(SIGTERM, &action, NULL);
	sigaction(SIGHUP, &action, NULL);
	sigaction(SIGQUIT, &action, NULL);
}

/* ================================================================
 * DRM/KMS Compositor
 * ================================================================ */

#define KMS_ARC_SPAN   1.047197551f   /* 60° fallback arc span when JS placement unavailable */
#define KMS_CAMERA_FOV 60.0f          /* placeholder vertical FOV (degrees) when device inactive */
#define KMS_NEAR       1.0f           /* clip planes in pixel units, matching KWin/GNOME */
#define KMS_FAR        10000.0f
#define KMS_MATH_PI    3.14159265358979323846f
#define MAX_DRM_DEVICES 64

struct drm_fb {
	struct gbm_bo *bo;
	uint32_t fb_id;
};

struct kms_display_tex {
	struct gbm_bo *bo;
	uint32_t bo_stride;
	EGLImageKHR egl_image;
	GLuint gl_tex;
	GLuint fallback_tex;
	uint32_t width;
	uint32_t height;
	bool initialized;
	bool has_first_frame;
};

struct kms_state {
	int drm_fd;
	uint32_t crtc_id;
	uint32_t connector_id;
	drmModeModeInfo mode;
	struct gbm_device *gbm_dev;
	struct gbm_surface *gbm_surface;
	EGLDisplay egl_display;
	EGLContext egl_context;
	EGLSurface egl_surface;
	PFNEGLCREATEIMAGEKHRPROC eglCreateImageKHR;
	PFNEGLDESTROYIMAGEKHRPROC eglDestroyImageKHR;
	PFNGLEGLIMAGETARGETTEXTURE2DOESPROC glEGLImageTargetTexture2DOES;
	bool dma_buf_supported;
	struct display_renderer dr;
	struct kms_display_tex displays[MAX_USBIP_DEVICES];
	size_t display_count;
	struct dp_placement placements[MAX_USBIP_DEVICES];
	bool placements_computed;

	/* Device state — updated by kms_poll_device_config() every ~1 s. */
	bool     async_flip;               /* DRM_CAP_ASYNC_PAGE_FLIP supported */
	bool     device_active;            /* glasses connected, data valid, version OK */
	float    device_complete_dist_px;  /* completeScreenDistancePixels — arc radius in pixel units */
	float    device_lens_dist_px;      /* lensDistancePixels — pivot-to-eye offset along -Z */
	float    device_camera_fov;        /* vertical FOV in degrees for es_perspective */
	uint64_t last_config_poll;         /* realtime_ms() of last poll */

	/* Size scale — 1.0 fills the device FOV exactly.  Placeholder for future config. */
	float    display_size;             /* distanceAdjustedSize equivalent */
	float    device_size_adj_w_px;     /* device_width  * display_size */
	float    device_size_adj_h_px;     /* device_height * display_size */

	/* Pre-computed curved-surface vertex meshes, rebuilt on config change. */
	struct display_mesh meshes[MAX_USBIP_DEVICES];
	bool     meshes_computed;

	/* Overlay: connection-status messages above the display group. */
	struct breezy_overlay overlay;
};

/* ----------------------------------------------------------------
 * Device config polling
 * ---------------------------------------------------------------- */

static uint64_t kms_realtime_ms(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	return (uint64_t)ts.tv_sec * 1000u + (uint64_t)(ts.tv_nsec / 1000000L);
}

/*
 * Poll the IMU module for device config at most once per second.
 * Derives display quad half-extents from the device's physical FOV so the
 * virtual display exactly fills the camera FOV at arc radius.
 * Mirrors the KWin plugin's diagonalToCrossFOVs + enable/disable logic.
 */
static void kms_poll_device_config(struct kms_state *kms,
                                    const struct server_runtime *server)
{
	const struct server_options *opts = &server->opts;

	uint64_t now = kms_realtime_ms();
	if (now - kms->last_config_poll < 1000u)
		return;
	kms->last_config_poll = now;

	struct breezy_imu_device_config cfg;
	struct breezy_imu_pose pose;
	bool have_config = breezy_imu_get_config(&cfg);
	bool have_pose   = breezy_imu_try_get_pose(&pose);

	bool active = have_config && have_pose
	              && cfg.enabled
	              && cfg.version == BREEZY_IMU_EXPECTED_VERSION
	              && cfg.diagonal_fov_deg != 0.0f;

	/* Count active display slots for the overlay regardless of glasses state. */
	{
		size_t active_slots = 0u;
		for (size_t i = 0u; i < server->opts.device_count; i++) {
			const struct device_runtime *dev = &server->devices[i];
			bool slot_active = dev->is_gadget_device
					       ? (dev->udl.gadget_fd >= 0)
					       : dev->imported;
			if (slot_active)
				active_slots++;
		}
		breezy_overlay_update(&kms->overlay, active,
				      active_slots, server->opts.device_count);
	}

	if (active == kms->device_active && kms->device_complete_dist_px != 0.0f)
		return;

	kms->device_active    = active;
	kms->meshes_computed  = false;

	if (!active)
		return;

	uint32_t dev_w = (cfg.display_width  > 0) ? cfg.display_width
	                                           : (uint32_t)kms->mode.hdisplay;
	uint32_t dev_h = (cfg.display_height > 0) ? cfg.display_height
	                                           : (uint32_t)kms->mode.vdisplay;
	float diag_rad   = cfg.diagonal_fov_deg * (KMS_MATH_PI / 180.0f);
	float lens_ratio = cfg.lens_distance_ratio;

	struct dp_fov_details fov_det;
	if (dp_build_fov_details(dev_w, dev_h, diag_rad, lens_ratio, 1.0f, &fov_det) != 0) {
		kms->device_active = false;
		return;
	}

	kms->device_camera_fov        = fov_det.vertical_radians * (180.0f / KMS_MATH_PI);
	kms->device_complete_dist_px  = fov_det.complete_dist_px;
	kms->device_lens_dist_px      = fov_det.lens_distance_px;
	kms->device_size_adj_w_px     = (float)dev_w * kms->display_size;
	kms->device_size_adj_h_px     = (float)dev_h * kms->display_size;

	{
		struct dp_monitor_info monitors[MAX_USBIP_DEVICES];
		size_t i;
		for (i = 0u; i < opts->device_count; i++) {
			monitors[i].x      = opts->devices[i].x;
			monitors[i].y      = opts->devices[i].y;
			monitors[i].width  = opts->devices[i].decode_width;
			monitors[i].height = opts->devices[i].decode_height;
		}
		if (dp_compute_placements(monitors, opts->device_count,
		                          dev_w, dev_h, diag_rad, lens_ratio,
		                          fov_det.complete_dist_px, kms->placements) == 0) {
			kms->placements_computed = true;

			/* Build a curved vertex mesh for each configured display. */
			for (i = 0u; i < opts->device_count && i < MAX_USBIP_DEVICES; i++) {
				display_renderer_build_mesh(
				    fov_det.complete_dist_px,
				    fov_det.horizontal_radians, fov_det.vertical_radians,
				    dev_w, dev_h,
				    kms->device_size_adj_w_px, kms->device_size_adj_h_px,
				    kms->placements[i].cnx,
				    kms->placements[i].cny,
				    kms->placements[i].cnz,
				    monitors[i].width, monitors[i].height,
				    /*horizontal_wrap=*/1,
				    &kms->meshes[i]);
			}
			kms->meshes_computed = true;
		}
	}
}

/* ----------------------------------------------------------------
 * DRM framebuffer helper
 * ---------------------------------------------------------------- */

static void drm_fb_destroy_callback(struct gbm_bo *bo, void *data)
{
	int drm_fd = gbm_device_get_fd(gbm_bo_get_device(bo));
	struct drm_fb *fb = data;

	if (fb->fb_id)
		drmModeRmFB(drm_fd, fb->fb_id);
	free(fb);
}

static struct drm_fb *drm_fb_get_from_bo(struct gbm_bo *bo)
{
	int drm_fd = gbm_device_get_fd(gbm_bo_get_device(bo));
	struct drm_fb *fb = gbm_bo_get_user_data(bo);
	uint32_t handles[4] = {0};
	uint32_t strides[4] = {0};
	uint32_t offsets[4] = {0};
	int ret;

	if (fb)
		return fb;

	fb = calloc(1, sizeof(*fb));
	if (!fb)
		return NULL;
	fb->bo = bo;

	handles[0] = gbm_bo_get_handle(bo).u32;
	strides[0] = gbm_bo_get_stride(bo);

	ret = drmModeAddFB2(drm_fd,
	                    gbm_bo_get_width(bo), gbm_bo_get_height(bo),
	                    gbm_bo_get_format(bo),
	                    handles, strides, offsets, &fb->fb_id, 0);
	if (ret) {
		ret = drmModeAddFB(drm_fd,
		                   gbm_bo_get_width(bo), gbm_bo_get_height(bo),
		                   24, (uint8_t)gbm_bo_get_bpp(bo),
		                   strides[0], handles[0], &fb->fb_id);
	}
	if (ret) {
		fprintf(stderr, "Failed to create DRM FB: %s\n", strerror(errno));
		free(fb);
		return NULL;
	}

	gbm_bo_set_user_data(bo, fb, drm_fb_destroy_callback);
	return fb;
}

static void kms_page_flip_handler(int fd, unsigned int frame,
                                   unsigned int sec, unsigned int usec,
                                   void *data)
{
	(void)fd; (void)frame; (void)sec; (void)usec;
	int *waiting = data;
	*waiting = 0;
}

/* ----------------------------------------------------------------
 * DRM initialisation
 * ---------------------------------------------------------------- */

static int kms_find_drm_device(drmModeRes **resources_out)
{
	drmDevicePtr devices[MAX_DRM_DEVICES] = {NULL};
	int num_devices = drmGetDevices2(0, devices, MAX_DRM_DEVICES);
	int fd = -1;

	if (num_devices < 0) {
		fprintf(stderr, "drmGetDevices2 failed: %s\n", strerror(-num_devices));
		return -1;
	}

	for (int i = 0; i < num_devices && fd < 0; i++) {
		if (!(devices[i]->available_nodes & (1 << DRM_NODE_PRIMARY)))
			continue;
		fd = open(devices[i]->nodes[DRM_NODE_PRIMARY], O_RDWR);
		if (fd < 0)
			continue;
		*resources_out = drmModeGetResources(fd);
		if (!*resources_out) {
			close(fd);
			fd = -1;
		}
	}
	drmFreeDevices(devices, num_devices);
	if (fd < 0)
		fprintf(stderr, "No KMS-capable DRM device found\n");
	return fd;
}

static int kms_init_drm(struct kms_state *kms, const char *device)
{
	drmModeRes *resources = NULL;
	drmModeConnector *connector = NULL;
	drmModeEncoder *encoder = NULL;
	int i;

	if (device) {
		kms->drm_fd = open(device, O_RDWR);
		if (kms->drm_fd >= 0)
			resources = drmModeGetResources(kms->drm_fd);
	} else {
		kms->drm_fd = kms_find_drm_device(&resources);
	}

	if (kms->drm_fd < 0 || !resources) {
		fprintf(stderr, "Failed to open/query DRM device\n");
		return -1;
	}

	for (i = 0; i < resources->count_connectors; i++) {
		connector = drmModeGetConnector(kms->drm_fd, resources->connectors[i]);
		if (connector && connector->connection == DRM_MODE_CONNECTED && connector->count_modes > 0)
			break;
		drmModeFreeConnector(connector);
		connector = NULL;
	}
	if (!connector) {
		fprintf(stderr, "No connected DRM connector found\n");
		drmModeFreeResources(resources);
		return -1;
	}

	kms->mode = connector->modes[0];
	{
		int best_area = 0;
		for (i = 0; i < connector->count_modes; i++) {
			drmModeModeInfo *m = &connector->modes[i];
			if (m->type & DRM_MODE_TYPE_PREFERRED) {
				kms->mode = *m;
				best_area = INT_MAX;
				break;
			}
			int area = m->hdisplay * m->vdisplay;
			if (area > best_area) {
				kms->mode = *m;
				best_area = area;
			}
		}
	}
	kms->connector_id = connector->connector_id;

	for (i = 0; i < resources->count_encoders; i++) {
		encoder = drmModeGetEncoder(kms->drm_fd, resources->encoders[i]);
		if (encoder && encoder->encoder_id == connector->encoder_id)
			break;
		drmModeFreeEncoder(encoder);
		encoder = NULL;
	}

	if (encoder) {
		kms->crtc_id = encoder->crtc_id;
		drmModeFreeEncoder(encoder);
	} else {
		for (i = 0; i < connector->count_encoders && !kms->crtc_id; i++) {
			int j;

			encoder = drmModeGetEncoder(kms->drm_fd, connector->encoders[i]);
			if (!encoder)
				continue;
			for (j = 0; j < resources->count_crtcs; j++) {
				if (encoder->possible_crtcs & (1u << j)) {
					kms->crtc_id = resources->crtcs[j];
					break;
				}
			}
			drmModeFreeEncoder(encoder);
		}
	}

	drmModeFreeConnector(connector);
	drmModeFreeResources(resources);

	if (!kms->crtc_id) {
		fprintf(stderr, "No usable CRTC found\n");
		return -1;
	}

	printf("KMS: mode %ux%u@%u connector=%u crtc=%u\n",
	       kms->mode.hdisplay, kms->mode.vdisplay, kms->mode.vrefresh,
	       kms->connector_id, kms->crtc_id);
	return 0;
}

/* ----------------------------------------------------------------
 * GBM + EGL initialisation
 * ---------------------------------------------------------------- */

static bool kms_has_ext(const char *list, const char *ext)
{
	size_t len;

	if (!list || !ext)
		return false;
	len = strlen(ext);
	while (list) {
		const char *p = strstr(list, ext);
		if (!p)
			return false;
		if (p[len] == ' ' || p[len] == '\0')
			return true;
		list = p + len;
	}
	return false;
}

static int kms_init_gbm_egl(struct kms_state *kms)
{
	EGLint major, minor;
	const char *client_exts, *dpy_exts, *gl_exts;
	EGLint count = 0, matched = 0;
	EGLConfig *configs = NULL;
	EGLConfig chosen_config = NULL;
	int i;

	static const EGLint context_attribs[] = {
		EGL_CONTEXT_CLIENT_VERSION, 2,
		EGL_NONE
	};
	static const EGLint config_attribs[] = {
		EGL_SURFACE_TYPE,    EGL_WINDOW_BIT,
		EGL_RED_SIZE,        1,
		EGL_GREEN_SIZE,      1,
		EGL_BLUE_SIZE,       1,
		EGL_ALPHA_SIZE,      0,
		EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
		EGL_NONE
	};

	kms->gbm_dev = gbm_create_device(kms->drm_fd);
	if (!kms->gbm_dev) {
		fprintf(stderr, "Failed to create GBM device\n");
		return -1;
	}

	kms->gbm_surface = gbm_surface_create(kms->gbm_dev,
	                                       kms->mode.hdisplay,
	                                       kms->mode.vdisplay,
	                                       GBM_FORMAT_XRGB8888,
	                                       GBM_BO_USE_SCANOUT | GBM_BO_USE_RENDERING);
	if (!kms->gbm_surface) {
		fprintf(stderr, "Failed to create GBM surface\n");
		return -1;
	}

	client_exts = eglQueryString(EGL_NO_DISPLAY, EGL_EXTENSIONS);
	if (kms_has_ext(client_exts, "EGL_EXT_platform_base")) {
		PFNEGLGETPLATFORMDISPLAYEXTPROC get_dpy =
			(PFNEGLGETPLATFORMDISPLAYEXTPROC)eglGetProcAddress("eglGetPlatformDisplayEXT");
		if (get_dpy)
			kms->egl_display = get_dpy(EGL_PLATFORM_GBM_KHR, kms->gbm_dev, NULL);
	}
	if (kms->egl_display == EGL_NO_DISPLAY)
		kms->egl_display = eglGetDisplay((EGLNativeDisplayType)kms->gbm_dev);
	if (kms->egl_display == EGL_NO_DISPLAY) {
		fprintf(stderr, "Failed to get EGL display\n");
		return -1;
	}

	if (!eglInitialize(kms->egl_display, &major, &minor)) {
		fprintf(stderr, "Failed to initialize EGL\n");
		return -1;
	}
	printf("EGL %d.%d on %s\n", major, minor,
	       eglQueryString(kms->egl_display, EGL_VENDOR));

	dpy_exts = eglQueryString(kms->egl_display, EGL_EXTENSIONS);
	if (kms_has_ext(dpy_exts, "EGL_KHR_image_base")) {
		kms->eglCreateImageKHR =
			(PFNEGLCREATEIMAGEKHRPROC)eglGetProcAddress("eglCreateImageKHR");
		kms->eglDestroyImageKHR =
			(PFNEGLDESTROYIMAGEKHRPROC)eglGetProcAddress("eglDestroyImageKHR");
	}
	kms->dma_buf_supported = kms->eglCreateImageKHR &&
	                          kms_has_ext(dpy_exts, "EGL_EXT_image_dma_buf_import");

	if (!eglBindAPI(EGL_OPENGL_ES_API)) {
		fprintf(stderr, "Failed to bind OpenGL ES API\n");
		return -1;
	}

	eglGetConfigs(kms->egl_display, NULL, 0, &count);
	configs = calloc((size_t)count, sizeof(*configs));
	if (!configs)
		return -1;
	eglChooseConfig(kms->egl_display, config_attribs, configs, count, &matched);
	chosen_config = configs[0];
	for (i = 0; i < matched; i++) {
		EGLint visual;
		eglGetConfigAttrib(kms->egl_display, configs[i], EGL_NATIVE_VISUAL_ID, &visual);
		if ((uint32_t)visual == GBM_FORMAT_XRGB8888) {
			chosen_config = configs[i];
			break;
		}
	}
	free(configs);

	kms->egl_context = eglCreateContext(kms->egl_display, chosen_config,
	                                     EGL_NO_CONTEXT, context_attribs);
	if (kms->egl_context == EGL_NO_CONTEXT) {
		fprintf(stderr, "Failed to create EGL context\n");
		return -1;
	}

	kms->egl_surface = eglCreateWindowSurface(kms->egl_display, chosen_config,
	                                           (EGLNativeWindowType)kms->gbm_surface, NULL);
	if (kms->egl_surface == EGL_NO_SURFACE) {
		fprintf(stderr, "Failed to create EGL surface\n");
		return -1;
	}

	if (!eglMakeCurrent(kms->egl_display, kms->egl_surface, kms->egl_surface, kms->egl_context)) {
		fprintf(stderr, "Failed to make EGL context current\n");
		return -1;
	}
	eglSwapInterval(kms->egl_display, 0);

	gl_exts = (const char *)glGetString(GL_EXTENSIONS);
	if (kms_has_ext(gl_exts, "GL_OES_EGL_image")) {
		kms->glEGLImageTargetTexture2DOES =
			(PFNGLEGLIMAGETARGETTEXTURE2DOESPROC)eglGetProcAddress("glEGLImageTargetTexture2DOES");
	}
	if (!kms->glEGLImageTargetTexture2DOES)
		kms->dma_buf_supported = false;

	printf("DMA-buf path: %s\n", kms->dma_buf_supported ? "enabled" : "disabled (fallback)");
	return 0;
}

/* ----------------------------------------------------------------
 * KMS initialisation
 * ---------------------------------------------------------------- */

static int kms_init(struct kms_state *kms, const char *drm_device,
                     const struct server_options *opts)
{
	(void)opts;
	memset(kms, 0, sizeof(*kms));
	kms->drm_fd = -1;
	kms->egl_display = EGL_NO_DISPLAY;
	kms->egl_context = EGL_NO_CONTEXT;
	kms->egl_surface = EGL_NO_SURFACE;

	if (kms_init_drm(kms, drm_device) != 0)
		return -1;
	if (kms_init_gbm_egl(kms) != 0)
		return -1;
	if (display_renderer_init(&kms->dr) != 0)
		return -1;

	return 0;
}

/* ----------------------------------------------------------------
 * Per-display DMA-buf texture initialisation
 * ---------------------------------------------------------------- */

static int kms_init_display_tex(struct kms_state *kms, size_t idx,
                                 struct udl_runtime *udl)
{
	struct kms_display_tex *disp = &kms->displays[idx];
	uint32_t w = udl->width;
	uint32_t h = udl->height;

	disp->width = w;
	disp->height = h;

	if (!w || !h) {
		disp->initialized = true;
		return 0;
	}

	if (kms->dma_buf_supported && kms->eglCreateImageKHR && kms->glEGLImageTargetTexture2DOES) {
		disp->bo = gbm_bo_create(kms->gbm_dev, w, h, GBM_FORMAT_RGB565, GBM_BO_USE_LINEAR);
		if (!disp->bo) {
			fprintf(stderr, "kms: GBM BO alloc failed for display %zu, using fallback\n", idx);
			goto fallback;
		}

		int fd = gbm_bo_get_fd(disp->bo);
		if (fd < 0) {
			gbm_bo_destroy(disp->bo);
			disp->bo = NULL;
			goto fallback;
		}
		disp->bo_stride = gbm_bo_get_stride(disp->bo);

		EGLint attrs[] = {
			EGL_WIDTH,                       (EGLint)w,
			EGL_HEIGHT,                      (EGLint)h,
			EGL_LINUX_DRM_FOURCC_EXT,        DRM_FORMAT_RGB565,
			EGL_DMA_BUF_PLANE0_FD_EXT,       fd,
			EGL_DMA_BUF_PLANE0_OFFSET_EXT,   0,
			EGL_DMA_BUF_PLANE0_PITCH_EXT,    (EGLint)disp->bo_stride,
			EGL_NONE
		};
		disp->egl_image = kms->eglCreateImageKHR(kms->egl_display, EGL_NO_CONTEXT,
		                                          EGL_LINUX_DMA_BUF_EXT, NULL, attrs);
		close(fd);

		if (disp->egl_image == EGL_NO_IMAGE_KHR) {
			fprintf(stderr, "kms: EGLImage create failed for display %zu, using fallback\n", idx);
			gbm_bo_destroy(disp->bo);
			disp->bo = NULL;
			goto fallback;
		}

		glGenTextures(1, &disp->gl_tex);
		glBindTexture(GL_TEXTURE_2D, disp->gl_tex);
		kms->glEGLImageTargetTexture2DOES(GL_TEXTURE_2D, disp->egl_image);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glBindTexture(GL_TEXTURE_2D, 0);

		fprintf(stderr, "kms: display %zu DMA-buf texture %ux%u stride=%u\n",
		        idx, w, h, disp->bo_stride);
		disp->initialized = true;
		return 0;
	}

fallback:
	glGenTextures(1, &disp->fallback_tex);
	glBindTexture(GL_TEXTURE_2D, disp->fallback_tex);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, (GLsizei)w, (GLsizei)h,
	             0, GL_RGB, GL_UNSIGNED_SHORT_5_6_5, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glBindTexture(GL_TEXTURE_2D, 0);

	fprintf(stderr, "kms: display %zu fallback GL texture %ux%u\n", idx, w, h);
	disp->initialized = true;
	return 0;
}

static int kms_init_display_textures(struct kms_state *kms, struct server_runtime *server)
{
	size_t i;

	kms->display_count = server->opts.device_count;
	if (kms->display_count == 0)
		kms->display_count = 1u;

	for (i = 0u; i < kms->display_count && i < MAX_USBIP_DEVICES; i++) {
		if (i < server->opts.device_count && server->devices[i].udl.enabled) {
			if (kms_init_display_tex(kms, i, &server->devices[i].udl) != 0)
				return -1;
		} else {
			kms->displays[i].initialized = true;
		}
	}
	return 0;
}

/* ----------------------------------------------------------------
 * Texture update from UDL decode buffer
 * ---------------------------------------------------------------- */

static void kms_update_display_textures(struct kms_state *kms,
                                         struct server_runtime *server)
{
	size_t i;

	for (i = 0u; i < kms->display_count; i++) {
		struct kms_display_tex *disp = &kms->displays[i];
		struct udl_runtime *udl;
		struct udl_sink_damage damage;
		uint32_t x1;
		uint32_t y1;
		uint32_t x2;
		uint32_t y2;
		uint32_t dirty_width;
		uint32_t dirty_height;

		if (i >= server->opts.device_count)
			continue;
		udl = &server->devices[i].udl;

		if (!udl->enabled || !udl->framebuffer_xrgb8888 || !disp->initialized)
			continue;

		if (!disp->has_first_frame) {
			if (!udl->have_decoder_snapshot)
				continue;
			disp->has_first_frame = true;
		}

		if (!atomic_exchange(&udl->frame_dirty, false))
			continue;
		if (!udl_runtime_consume_damage(udl, &damage))
			continue;

		x1 = damage.x1 < disp->width ? damage.x1 : disp->width;
		y1 = damage.y1 < disp->height ? damage.y1 : disp->height;
		x2 = damage.x2 < disp->width ? damage.x2 : disp->width;
		y2 = damage.y2 < disp->height ? damage.y2 : disp->height;
		if (x1 >= x2 || y1 >= y2)
			continue;
		dirty_width = x2 - x1;
		dirty_height = y2 - y1;

		if (kms->dma_buf_supported && disp->bo && disp->gl_tex) {
			void *map_data = NULL;
			uint32_t stride;
			void *mapped = gbm_bo_map(disp->bo, x1, y1,
			                          dirty_width, dirty_height,
			                          GBM_BO_TRANSFER_WRITE, &stride, &map_data);
			if (mapped) {
				uint32_t row;

				for (row = 0u; row < dirty_height; row++) {
					memcpy((uint8_t *)mapped + (size_t)row * stride,
					       udl->framebuffer_rgb565 +
					       (size_t)(y1 + row) * udl->backing_width + x1,
					       (size_t)dirty_width * 2u);
				}
				gbm_bo_unmap(disp->bo, map_data);
				glBindTexture(GL_TEXTURE_2D, disp->gl_tex);
				kms->glEGLImageTargetTexture2DOES(GL_TEXTURE_2D, disp->egl_image);
				glBindTexture(GL_TEXTURE_2D, 0);
			}
		} else if (disp->fallback_tex) {
			glBindTexture(GL_TEXTURE_2D, disp->fallback_tex);
			glPixelStorei(GL_UNPACK_ALIGNMENT, 2);
			for (uint32_t row = 0u; row < dirty_height; row++) {
				glTexSubImage2D(GL_TEXTURE_2D, 0,
				                (GLint)x1, (GLint)(y1 + row),
				                (GLsizei)dirty_width, 1,
				                GL_RGB, GL_UNSIGNED_SHORT_5_6_5,
				                udl->framebuffer_rgb565 +
				                (size_t)(y1 + row) * udl->backing_width + x1);
			}
			glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
			glBindTexture(GL_TEXTURE_2D, 0);
		}
	}
}

/* ----------------------------------------------------------------
 * Frame rendering
 * ---------------------------------------------------------------- */

static void kms_render_frame(struct kms_state *kms,
                              const struct server_runtime *server)
{
	float screen_aspect = (float)kms->mode.hdisplay / (float)kms->mode.vdisplay;
	ESMatrix proj, model, mvp;

	glViewport(0, 0, (GLsizei)kms->mode.hdisplay, (GLsizei)kms->mode.vdisplay);
	glClearColor(0.04f, 0.04f, 0.08f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);

	enum breezy_overlay_state ovstate = kms->overlay.state;

	/* ---- Overlay-only states (no display quads) ---- */
	if (ovstate != BREEZY_OVERLAY_NORMAL) {
		/*
		 * Centre a text quad in the viewport using a fixed FOV camera.
		 * The quad is sized to the texture's native pixel dimensions so
		 * the bitmap font renders at its intended scale, then placed on
		 * the arc at arc_dist_px so KMS_CAMERA_FOV frames the mode height.
		 */
		float fov_half_tan = tanf(KMS_CAMERA_FOV * 0.5f * (KMS_MATH_PI / 180.0f));
		float arc_dist_px  = (float)kms->mode.vdisplay * 0.5f / fov_half_tan;

		es_perspective(&proj, KMS_CAMERA_FOV, screen_aspect, KMS_NEAR, KMS_FAR);
		es_display_model(&model, 0.0f, 0.0f, 0.0f, -arc_dist_px);
		es_multiply(&mvp, &model, &proj);

		if (kms->overlay.tex.tex) {
			float w_half = (float)kms->overlay.tex.tex_w * 0.5f;
			float h_half = (float)kms->overlay.tex.tex_h * 0.5f;
			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			display_renderer_draw_quad_alpha(&kms->dr, &mvp,
							 kms->overlay.tex.tex,
							 w_half, h_half);
			glDisable(GL_BLEND);
		}
		return;
	}

	/*
	 * Glasses active with at least one imported display.
	 * Use device-derived FOV and head-tracking view rotation.
	 */
	es_perspective(&proj, kms->device_camera_fov, screen_aspect, KMS_NEAR, KMS_FAR);

	ESMatrix view, proj_view;
	struct breezy_imu_pose pose;
	if (breezy_imu_try_get_pose(&pose)) {
		es_view_from_quat(&view, pose.quat_w, pose.quat_x, pose.quat_y, pose.quat_z);
	} else {
		es_load_identity(&view);
	}
	/*
	 * Lens pivot offset: the rotation origin is behind the eye by lensDistancePixels.
	 * Rotating the lens vector back through R_conj always yields (0, 0, +lens_dist)
	 * in eye space, so it is a constant Z shift on the view matrix's translation column,
	 * independent of head orientation.  Mirrors CameraController.qml's lensVector logic.
	 */
	view.m[3][2] = kms->device_lens_dist_px;
	es_multiply(&proj_view, &view, &proj);

	/*
	 * Only render slots that have an active USB/IP import.
	 * Build a compact index array so the fallback arc spacing and the
	 * overlay Y position are based on the visible count only.
	 */
	size_t visible_idx[MAX_USBIP_DEVICES];
	size_t visible_count = 0u;
	for (size_t i = 0u; i < kms->display_count; i++) {
		if (i >= server->opts.device_count)
			continue;
		const struct device_runtime *dev = &server->devices[i];
		bool active_slot = dev->is_gadget_device
				       ? (dev->udl.gadget_fd >= 0)
				       : dev->imported;
		if (active_slot)
			visible_idx[visible_count++] = i;
	}

	/* Track extents of all rendered quads for overlay placement. */
	float top_y_px  = 0.0f;
	float min_x_px  = 0.0f;
	float max_x_px  = 0.0f;
	bool  first_ext = true;
	float arc_r_px  = kms->device_complete_dist_px;

	for (size_t vi = 0u; vi < visible_count; vi++) {
		size_t i = visible_idx[vi];
		struct kms_display_tex *disp = &kms->displays[i];
		GLuint tex;

		if (disp->initialized && disp->has_first_frame) {
			tex = disp->gl_tex       ? disp->gl_tex
			    : disp->fallback_tex ? disp->fallback_tex
			                        : kms->dr.placeholder_tex;
		} else {
			tex = kms->dr.placeholder_tex;
		}

		if (kms->meshes_computed) {
			/*
			 * Mesh vertices are pre-computed in GL world space relative to the
			 * rotation origin (centerNoRotate).  Apply Y-rotation only — no
			 * translation — so the arc curvature baked into the vertices is
			 * preserved.  Flat displays produce a 1-segment mesh (4 vertices)
			 * with no separate code path needed.
			 */
			const struct display_mesh *mesh = &kms->meshes[i];
			es_display_model(&model, kms->placements[i].angle, 0.0f, 0.0f, 0.0f);
			es_multiply(&mvp, &model, &proj_view);
			display_renderer_draw_mesh(&kms->dr, &mvp, tex,
			                          (const float *)mesh->verts, mesh->vert_count);

			float w_half  = (float)server->opts.devices[i].decode_width  * 0.5f;
			float h_half  = (float)server->opts.devices[i].decode_height * 0.5f;
			float left_x  = kms->placements[i].cnx - w_half;
			float right_x = kms->placements[i].cnx + w_half;
			float quad_top = kms->placements[i].cny + h_half;
			fprintf(stderr, "kms_overlay: slot=%zu cnx=%.0f w=%.0f left=%.0f right=%.0f\n",
				i, kms->placements[i].cnx, w_half*2.0f, left_x, right_x);
			if (quad_top > top_y_px)
				top_y_px = quad_top;
			if (first_ext || left_x  < min_x_px) min_x_px = left_x;
			if (first_ext || right_x > max_x_px) max_x_px = right_x;
			first_ext = false;
		} else {
			/* No mesh yet — evenly space displays on a fallback arc. */
			float ang    = (visible_count <= 1u) ? 0.0f
					: KMS_ARC_SPAN * ((float)vi / (float)(visible_count - 1u) - 0.5f);
			float w_half = (float)disp->width  * 0.5f;
			float h_half = (float)disp->height * 0.5f;
			es_display_model(&model, ang, arc_r_px * sinf(ang), 0.0f, -arc_r_px * cosf(ang));
			es_multiply(&mvp, &model, &proj_view);
			display_renderer_draw_quad(&kms->dr, &mvp, tex, w_half, h_half);

			if (h_half > top_y_px) top_y_px = h_half;
		}
	}

	/* Midpoint of the combined left-edge to right-edge span. */
	float center_x_px = first_ext ? 0.0f : (min_x_px + max_x_px) * 0.5f;
	{
		static size_t last_visible = SIZE_MAX;
		static float  last_cx = 0.0f;
		if (visible_count != last_visible || center_x_px != last_cx) {
			fprintf(stderr, "kms_overlay: visible=%zu meshes=%d "
				"min_x=%.0f max_x=%.0f center_x=%.0f top_y=%.0f\n",
				visible_count, (int)kms->meshes_computed,
				min_x_px, max_x_px, center_x_px, top_y_px);
			last_visible = visible_count;
			last_cx = center_x_px;
		}
	}

	/* ---- Overlay info strip above the display group ---- */
	if (kms->overlay.tex.tex) {
		float ot_w_half = (float)kms->overlay.tex.tex_w * 0.5f;
		float ot_h_half = (float)kms->overlay.tex.tex_h * 0.5f;
		float gap_px    = ot_h_half;                       /* one text-height gap */
		float ot_y      = top_y_px + gap_px + ot_h_half;

		if (kms->meshes_computed) {
			/* Place at arc radius, horizontally centred over the display group. */
			es_display_model(&model, 0.0f, center_x_px, ot_y, -arc_r_px);
		} else {
			es_display_model(&model, 0.0f, 0.0f, ot_y, -arc_r_px);
		}
		es_multiply(&mvp, &model, &proj_view);

		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		display_renderer_draw_quad_alpha(&kms->dr, &mvp,
						 kms->overlay.tex.tex,
						 ot_w_half, ot_h_half);
		glDisable(GL_BLEND);
	}
}

/* ----------------------------------------------------------------
 * KMS page-flip render loop
 * ---------------------------------------------------------------- */

static int kms_run(struct kms_state *kms, struct server_runtime *server)
{
	drmEventContext evctx = {
		.version = 2,
		.page_flip_handler = kms_page_flip_handler,
	};
	struct gbm_bo *bo;
	struct drm_fb *fb;
	fd_set fds;

	if (drmSetMaster(kms->drm_fd) < 0)
		fprintf(stderr, "kms: drmSetMaster: %s (may still work)\n", strerror(errno));

	{
		uint64_t cap = 0;
		kms->async_flip = (drmGetCap(kms->drm_fd, DRM_CAP_ASYNC_PAGE_FLIP, &cap) == 0 && cap != 0);
		printf("kms: async page flip: %s\n", kms->async_flip ? "enabled" : "disabled");
	}

	kms->device_active            = false;
	kms->device_complete_dist_px  = 0.0f;
	kms->device_lens_dist_px      = 0.0f;
	kms->device_camera_fov        = KMS_CAMERA_FOV;
	kms->last_config_poll         = 0;
	kms->display_size             = 1.0f;
	kms->device_size_adj_w_px     = 0.0f;
	kms->device_size_adj_h_px     = 0.0f;
	kms->meshes_computed          = false;
	/* Prime the overlay texture before the first frame. */
	breezy_overlay_update(&kms->overlay, false, 0u, server->opts.device_count);
	kms_update_display_textures(kms, server);
	kms_render_frame(kms, server);
	if (!eglSwapBuffers(kms->egl_display, kms->egl_surface)) {
		fprintf(stderr, "kms: initial eglSwapBuffers failed: 0x%x\n", eglGetError());
		return -1;
	}
	bo = gbm_surface_lock_front_buffer(kms->gbm_surface);
	if (!bo) {
		fprintf(stderr, "kms: initial gbm_surface_lock_front_buffer returned NULL\n");
		return -1;
	}
	fb = drm_fb_get_from_bo(bo);
	if (!fb) {
		fprintf(stderr, "kms: failed to get initial framebuffer\n");
		return -1;
	}
	if (drmModeSetCrtc(kms->drm_fd, kms->crtc_id, fb->fb_id, 0, 0,
	                   &kms->connector_id, 1, &kms->mode) != 0) {
		perror("drmModeSetCrtc");
		return -1;
	}

	while (!stop_requested) {
		int waiting_for_flip = 1;
		struct gbm_bo *next_bo;
		struct drm_fb *next_fb;
		int ret;

		kms_poll_device_config(kms, server);
		kms_update_display_textures(kms, server);
		kms_render_frame(kms, server);
		if (!eglSwapBuffers(kms->egl_display, kms->egl_surface)) {
			fprintf(stderr, "kms: eglSwapBuffers failed: 0x%x\n", eglGetError());
			break;
		}

		next_bo = gbm_surface_lock_front_buffer(kms->gbm_surface);
		if (!next_bo) {
			fprintf(stderr, "kms: gbm_surface_lock_front_buffer returned NULL\n");
			break;
		}
		next_fb = drm_fb_get_from_bo(next_bo);
		if (!next_fb) {
			fprintf(stderr, "kms: failed to get next framebuffer\n");
			gbm_surface_release_buffer(kms->gbm_surface, next_bo);
			break;
		}

		if (kms->async_flip) {
			ret = drmModePageFlip(kms->drm_fd, kms->crtc_id, next_fb->fb_id,
			                      DRM_MODE_PAGE_FLIP_ASYNC, NULL);
			if (ret && errno == EBUSY) {
				/* Display engine still scanning: drop this frame, try next iteration. */
				gbm_surface_release_buffer(kms->gbm_surface, next_bo);
				continue;
			}
		} else {
			ret = drmModePageFlip(kms->drm_fd, kms->crtc_id, next_fb->fb_id,
			                      DRM_MODE_PAGE_FLIP_EVENT, &waiting_for_flip);
		}
		if (ret) {
			fprintf(stderr, "kms: drmModePageFlip: %s\n", strerror(errno));
			gbm_surface_release_buffer(kms->gbm_surface, next_bo);
			break;
		}

		if (!kms->async_flip) {
			while (waiting_for_flip && !stop_requested) {
				struct timeval tv = {0, 1000};

				FD_ZERO(&fds);
				FD_SET(kms->drm_fd, &fds);
				ret = select(kms->drm_fd + 1, &fds, NULL, NULL, &tv);
				if (ret < 0 && errno != EINTR) {
					perror("select");
					waiting_for_flip = 0;
				} else if (ret > 0 && FD_ISSET(kms->drm_fd, &fds)) {
					drmHandleEvent(kms->drm_fd, &evctx);
				}
			}
		}

		gbm_surface_release_buffer(kms->gbm_surface, bo);
		bo = next_bo;
	}

	return 0;
}

/* ----------------------------------------------------------------
 * KMS state teardown
 * ---------------------------------------------------------------- */

static void kms_destroy(struct kms_state *kms)
{
	size_t i;

	if (!kms)
		return;

	for (i = 0u; i < MAX_USBIP_DEVICES; i++) {
		struct kms_display_tex *disp = &kms->displays[i];

		if (disp->gl_tex)
			glDeleteTextures(1, &disp->gl_tex);
		if (disp->egl_image && kms->eglDestroyImageKHR)
			kms->eglDestroyImageKHR(kms->egl_display, disp->egl_image);
		if (disp->bo)
			gbm_bo_destroy(disp->bo);
		if (disp->fallback_tex)
			glDeleteTextures(1, &disp->fallback_tex);
	}

	display_renderer_destroy(&kms->dr);
	breezy_overlay_destroy(&kms->overlay);

	if (kms->egl_display != EGL_NO_DISPLAY) {
		eglMakeCurrent(kms->egl_display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
		if (kms->egl_surface != EGL_NO_SURFACE)
			eglDestroySurface(kms->egl_display, kms->egl_surface);
		if (kms->egl_context != EGL_NO_CONTEXT)
			eglDestroyContext(kms->egl_display, kms->egl_context);
		eglTerminate(kms->egl_display);
	}

	if (kms->gbm_surface)
		gbm_surface_destroy(kms->gbm_surface);
	if (kms->gbm_dev)
		gbm_device_destroy(kms->gbm_dev);
	if (kms->drm_fd >= 0)
		close(kms->drm_fd);
}

/* ----------------------------------------------------------------
 * Background USB/IP server thread
 * ---------------------------------------------------------------- */

static void *server_thread_main(void *arg)
{
	struct server_runtime *server = arg;

	(void)run_server(server);
	return NULL;
}

/* ----------------------------------------------------------------
 * main()
 * ---------------------------------------------------------------- */

int main(int argc, char **argv)
{
	struct runtime_cli_args cli;
	struct server_runtime server;
	struct kms_state kms;
	struct usb_gadget_config gadget_cfg;
	struct usb_gadget_state gadget_state;
	struct link_services_state link_state;
	struct link_services_config link_cfg;
	bool link_cfg_valid = false;
	pthread_t srv_thread;
	bool srv_thread_created = false;
	int exit_code = EXIT_FAILURE;
	const char *drm_device = NULL;

	memset(&server, 0, sizeof(server));
	server.listen_fd = -1;
	memset(&kms, 0, sizeof(kms));
	kms.drm_fd = -1;
	memset(&gadget_state, 0, sizeof(gadget_state));
	memset(&link_state, 0, sizeof(link_state));

	if (parse_runtime_cli_args(argc, argv, &cli) != 0) {
		usage(argv[0]);
		return EXIT_FAILURE;
	}
	if (load_server_options_from_config(cli.config_path, &cli, &server.opts) != 0)
		return EXIT_FAILURE;

	install_signal_handlers();

	if (dp_init() != 0) {
		fprintf(stderr, "Failed to initialise display placement JS runtime\n");
		return EXIT_FAILURE;
	}

	if (breezy_imu_init() != 0)
		fprintf(stderr, "Warning: IMU watcher failed to start; head-tracking disabled\n");

	usb_gadget_config_defaults(&gadget_cfg);
	if (usb_gadget_setup(&gadget_cfg, &gadget_state) != 0)
		fprintf(stderr, "Warning: OTG gadget setup failed; USB/IP will only be reachable over existing interfaces\n");

	/*
	 * Make the USB/IP server easy to reach over the OTG ethernet link with no
	 * manual host setup: hand the host an address (DHCP) and advertise the
	 * gadget as breezy.local (mDNS).  Best-effort — warns if dnsmasq/avahi are
	 * absent but never blocks startup.
	 */
	memset(&link_cfg, 0, sizeof(link_cfg));
	if (gadget_state.active) {
		link_services_config_defaults(&link_cfg);
		if (gadget_state.netdev_name[0])
			snprintf(link_cfg.iface, sizeof(link_cfg.iface), "%s",
				 gadget_state.netdev_name);
		/* Match the host's link MTU to the gadget's via DHCP option 26 so the
		 * jumbo NCM segment size actually reduces the host-side packet count. */
		link_cfg.mtu = gadget_cfg.ncm_mtu;
		/*
		 * Pin the link mDNS name (breezy.local) to the gadget's own
		 * address on this link so it tracks the configured CIDR rather
		 * than the default. Strip the "/prefix" suffix.
		 */
		if (gadget_cfg.ncm_ip_cidr[0]) {
			char *slash;
			snprintf(link_cfg.link_ip, sizeof(link_cfg.link_ip), "%s",
				 gadget_cfg.ncm_ip_cidr);
			slash = strchr(link_cfg.link_ip, '/');
			if (slash)
				*slash = '\0';
		}
		link_services_start(&link_cfg, &link_state);
		link_cfg_valid = true;
	}

	if (server_runtime_init_devices(&server) != 0) {
		fprintf(stderr, "Failed to initialise USB/IP device runtimes\n");
		goto out;
	}
	atomic_init(&server.viewer_stop_requested, false);

	if (kms_init(&kms, drm_device, &server.opts) != 0) {
		fprintf(stderr, "Failed to initialise KMS compositor\n");
		goto out;
	}

	breezy_overlay_set_addresses(&kms.overlay,
				     link_cfg_valid ? &link_cfg : NULL,
				     gadget_state.netdev_name[0]
					 ? gadget_state.netdev_name : NULL);

	if (kms_init_display_textures(&kms, &server) != 0) {
		fprintf(stderr, "Failed to initialise display textures\n");
		goto out;
	}

	if (pthread_create(&srv_thread, NULL, server_thread_main, &server) != 0) {
		perror("pthread_create server");
		goto out;
	}
	srv_thread_created = true;

	printf("DisplayLink KMS compositor running with %zu display slot(s)\n",
	       kms.display_count);

	(void)kms_run(&kms, &server);

	exit_code = EXIT_SUCCESS;

	out:
	stop_requested = 1;
	if (srv_thread_created) {
		if (server.listen_fd >= 0)
			shutdown(server.listen_fd, SHUT_RDWR);
		pthread_join(srv_thread, NULL);
	}
	kms_destroy(&kms);
	server_runtime_destroy(&server);
	link_services_stop(&link_state);
	usb_gadget_teardown(&gadget_state);
	breezy_imu_destroy();
	dp_destroy();
	return exit_code;
}
