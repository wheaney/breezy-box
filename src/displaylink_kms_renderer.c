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
#include <dirent.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/stat.h>
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
#include "breezy_state.h"
#include "overlay_text.h"
#include "breezy_settings.h"
#include "breezy_driver_control.h"
#include "common.h"
#include "display_renderer.h"
#include "server.h"
#include "display_placement.h"
#include "smooth_follow.h"
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
#define KMS_NEAR       10.0f          /* clip planes in pixel units, matching KWin's single-scene camera */
#define KMS_FAR        10000.0f       /* near=10 (not 1) keeps 16-bit depth precise enough to sort displays */
#define KMS_MATH_PI    3.14159265358979323846f
#define KMS_FOCUS_CHECK_MS 500u       /* gaze focus poll cadence (KWin uses a 500ms Timer) */
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

/*
 * Snapshot of every input that affects monitor placement / mesh geometry.
 * Compared with memcmp each poll; placements are only rebuilt when it changes.
 * Zeroed before filling so padding never produces a spurious mismatch.
 */
struct kms_cfg_sig {
	uint32_t dev_w, dev_h;
	float    diag_rad, lens_ratio, dist_default, dist_adj, monitor_spacing;
	float    viewport_offset_x, viewport_offset_y;
	bool     curved, headset_center;
	char     scheme[32];
	size_t   mon_count;
	/* RAW monitor config (unscaled). Layout identity for focus reset is based on
	 * this, so display-size (which only changes dist_adj) never resets focus. */
	struct dp_monitor_info mons[MAX_USBIP_DEVICES];
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
	/* Fixed device-FOV projection params (distance-independent), mirroring
	 * KWin CameraController: fovHalfVerticalTangent = heightUnitDistance/2,
	 * device_aspect = device_width/device_height. */
	float    device_fov_half_v_tan;
	float    device_aspect;
	uint64_t last_config_poll;         /* realtime_ms() of last poll */

	/* GSettings-driven display settings, updated via change signal. */
	breezy_settings_handle        *settings_handle;
	struct breezy_display_settings settings;

	/* Derived size values, recomputed whenever device config or settings change. */
	float    display_size;             /* distanceAdjustedSize: (dist_default - lens_ratio) * display_size */
	float    device_size_adj_w_px;     /* device_width  * display_size */
	float    device_size_adj_h_px;     /* device_height * display_size */

	/* Pre-computed curved-surface vertex meshes, rebuilt on config change. */
	struct display_mesh meshes[MAX_USBIP_DEVICES];
	bool     meshes_computed;

	/* Zoom-on-focus state.  display_distance_default is the resting ("all
	 * displays" / farther) multiplier used when placements were computed;
	 * focused_distance is the nearer multiplier a focused display eases to. */
	float    zoom_all_distance;        /* = display_distance_default */
	float    zoom_focused_distance;    /* nearer toggle endpoint */
	bool     zoom_on_focus;            /* focused < all */
	int      focused_monitor_index;    /* gaze-focused slot, -1 = none */
	uint64_t last_focus_check_ms;      /* throttle for dp_find_focused_monitor */
	/* Per-monitor eased distance multiplier + active transition bookkeeping. */
	float    monitor_distance[MAX_USBIP_DEVICES];
	float    zoom_ease_start[MAX_USBIP_DEVICES];
	float    zoom_ease_target[MAX_USBIP_DEVICES];
	uint64_t zoom_ease_begin_ms[MAX_USBIP_DEVICES];
	bool     zoom_ease_active[MAX_USBIP_DEVICES];
	bool     zoom_ease_seq_delay[MAX_USBIP_DEVICES];

	/* Smooth follow — renderer-agnostic state machine lives in smooth_follow.c;
	 * this file only feeds it IMU data and applies its outputs to the camera and
	 * the focused display's model matrix. */
	struct smooth_follow_state sf;

	/* Inputs captured at placement time so the periodic focus check can call
	 * dp_find_focused_monitor() without recomputing them every frame. */
	struct dp_monitor_info focus_monitors[MAX_USBIP_DEVICES];
	size_t   focus_slot_map[MAX_USBIP_DEVICES]; /* focus_slot_map[compact_i] = slot index */
	size_t   focus_monitor_count;
	char     focus_scheme[32];
	uint32_t focus_dev_w, focus_dev_h;
	float    focus_diag_rad, focus_lens_ratio, focus_dist_default;
	bool     focus_curved;

	/* Signature of the last layout-relevant config, so the 1 Hz poll only
	 * rebuilds placements/meshes (and resets focus) on a genuine change. */
	struct kms_cfg_sig last_cfg_sig;
	bool     has_cfg_sig;

	/* Overlay: GL texture for connection-status messages; text from breezy_state. */
	struct overlay_text overlay_tex;

	/* MSAA framebuffer — active when disable_anti_aliasing is false. */
	struct display_renderer_msaa msaa;

	/* Back-pointer to the long-lived app state (not owned by the renderer). */
	struct breezy_state *state;
};

/* ----------------------------------------------------------------
 * Device config polling and settings wiring
 * ---------------------------------------------------------------- */

static uint64_t kms_realtime_ms(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	return (uint64_t)ts.tv_sec * 1000u + (uint64_t)(ts.tv_nsec / 1000000L);
}

/*
 * Resolve "automatic" or "flat+curved" wrapping scheme to a concrete value.
 * Mirrors VirtualDisplaysActor._actual_wrap_scheme().
 */
static const char *kms_resolve_wrapping_scheme(
    const char *scheme, bool curved,
    const struct dp_monitor_info *monitors, size_t n,
    uint32_t dev_w, uint32_t dev_h)
{
	bool use_auto = (strcmp(scheme, "automatic") == 0) ||
	                (strcmp(scheme, "flat") == 0 && curved);
	if (!use_auto)
		return scheme; /* "horizontal", "vertical", or "flat" */

	if (n == 0 || dev_w == 0 || dev_h == 0)
		return "horizontal";

	int32_t minx = INT32_MAX, maxx = INT32_MIN;
	int32_t miny = INT32_MAX, maxy = INT32_MIN;
	for (size_t i = 0; i < n; i++) {
		int32_t x2 = monitors[i].x + (int32_t)monitors[i].width;
		int32_t y2 = monitors[i].y + (int32_t)monitors[i].height;
		if (monitors[i].x < minx) minx = monitors[i].x;
		if (x2 > maxx)            maxx = x2;
		if (monitors[i].y < miny) miny = monitors[i].y;
		if (y2 > maxy)            maxy = y2;
	}
	float span_x = (float)(maxx - minx) / (float)dev_w;
	float span_y = (float)(maxy - miny) / (float)dev_h;
	return (span_x >= span_y) ? "horizontal" : "vertical";
}

/*
 * Apply size-adjustment and viewport centering to raw monitor positions.
 * Mirrors VirtualDisplaysActor._handle_display_size_distance_change() +
 * _update_monitor_placements() coordinate preparation.
 *
 * The resulting coords are ready to pass directly to dp_compute_placements().
 */
/*
 * devs[i].{x,y,width,height} are the raw (pre-size-adjust) pixel positions of
 * each connected monitor.  n is the count of entries in devs[].  out[] receives
 * the size-adjusted, viewport-centred, offset-shifted positions ready for
 * dp_compute_placements().
 */
static void kms_adjust_monitor_positions(
    const struct dp_monitor_info *devs,
    size_t n,
    uint32_t dev_w, uint32_t dev_h,
    float dist_adj_size,
    bool headset_as_center,
    double viewport_offset_x, double viewport_offset_y,
    struct dp_monitor_info *out)
{
	float size_ox = (1.0f - dist_adj_size) / 2.0f * (float)dev_w;
	float size_oy = (1.0f - dist_adj_size) / 2.0f * (float)dev_h;
	float adj_dev_w = (float)dev_w * dist_adj_size;
	float adj_dev_h = (float)dev_h * dist_adj_size;

	/* Mirror dp_compute_placements auto-x: if all raw x=0 and n>1, assign
	 * monitors left-to-right cumulatively so centering and offset operate on
	 * the correct spread layout rather than stacking all monitors at x=0. */
	bool all_x_zero = (n > 1u);
	for (size_t i = 0; i < n && all_x_zero; i++) {
		if (devs[i].x != 0)
			all_x_zero = false;
	}
	int32_t raw_x[MAX_USBIP_DEVICES];
	if (all_x_zero) {
		raw_x[0] = 0;
		for (size_t i = 1; i < n; i++)
			raw_x[i] = raw_x[i - 1] + (int32_t)devs[i - 1].width;
	} else {
		for (size_t i = 0; i < n; i++)
			raw_x[i] = devs[i].x;
	}

	/* Size-adjusted positions. */
	float ax[MAX_USBIP_DEVICES], ay[MAX_USBIP_DEVICES];
	for (size_t i = 0; i < n; i++) {
		ax[i] = (float)raw_x[i] * dist_adj_size + size_ox;
		ay[i] = (float)devs[i].y * dist_adj_size + size_oy;
	}

	/* Viewport reference origin — mirrors the viewportXBegin/YBegin logic. */
	float vpx, vpy;
	if (headset_as_center || n == 0) {
		/* Device display always sits at raw origin (0,0), so after size-adjust: size_ox/oy. */
		vpx = size_ox;
		vpy = size_oy;
	} else {
		float minx = ax[0], maxx = ax[0] + (float)devs[0].width  * dist_adj_size;
		float miny = ay[0], maxy = ay[0] + (float)devs[0].height * dist_adj_size;
		for (size_t i = 1; i < n; i++) {
			float x2 = ax[i] + (float)devs[i].width  * dist_adj_size;
			float y2 = ay[i] + (float)devs[i].height * dist_adj_size;
			if (ax[i] < minx) minx = ax[i];
			if (x2 > maxx)    maxx = x2;
			if (ay[i] < miny) miny = ay[i];
			if (y2 > maxy)    maxy = y2;
		}
		vpx = (minx + maxx) / 2.0f - adj_dev_w / 2.0f;
		vpy = (miny + maxy) / 2.0f - adj_dev_h / 2.0f;
	}

	for (size_t i = 0; i < n; i++) {
		out[i].x      = (int32_t)(ax[i] - vpx - (float)viewport_offset_x * adj_dev_w);
		out[i].y      = (int32_t)(ay[i] - vpy + (float)viewport_offset_y * adj_dev_h);
		out[i].width  = (uint32_t)((float)devs[i].width  * dist_adj_size);
		out[i].height = (uint32_t)((float)devs[i].height * dist_adj_size);
	}

	/* In the auto-x case monitors are adjacent by definition.  Enforce cumulative
	 * x so that out[i].x == out[i-1].x + out[i-1].width exactly — monitorWrap
	 * relies on this: it caches 'end + spacing' at nextMonitorPixel = beginPx + width,
	 * and the next monitor's beginPixel must hit that exact key.  Independent
	 * float→int truncation of each position can leave a 1-pixel gap that causes
	 * monitorWrap to pick the wrong cache entry and misapply spacing. */
	if (all_x_zero) {
		for (size_t i = 1; i < n; i++)
			out[i].x = out[i - 1].x + (int32_t)out[i - 1].width;
	}
}

/* Begin a per-monitor distance ease (defined below; used by the poll to retarget
 * a focused display when the focused-distance setting changes). */
static void kms_begin_zoom_ease(struct kms_state *kms, size_t i,
                                float target_distance,
                                bool needs_sequence_delay, uint64_t now_ms);

/*
 * Poll device config at most once per second, or immediately when settings_dirty
 * is set (a GSettings change just arrived).  Recomputes placements and meshes
 * whenever either changes.  The rebuild below is guarded by a layout signature,
 * so running this every frame on a settings change only does real work when the
 * new settings actually move the geometry.  Mirrors the KWin plugin's
 * diagonalToCrossFOVs + enable/disable logic.
 */
static void kms_poll_device_config(struct kms_state *kms,
                                    const struct server_runtime *server,
                                    bool settings_dirty)
{
	const struct server_options *opts = &server->opts;

	uint64_t now = kms_realtime_ms();
	if (!settings_dirty && now - kms->last_config_poll < 1000u)
		return;
	kms->last_config_poll = now;

	struct breezy_imu_device_config cfg;
	struct breezy_imu_pose pose;
	bool have_config = breezy_imu_get_config(&cfg);
	bool have_pose   = breezy_imu_try_get_pose(&pose);

	bool xr_driver_up = have_config || have_pose;
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
		bool changed = breezy_state_update(kms->state, xr_driver_up, active,
						   active_slots, server->opts.device_count,
						   server->opts.verbose);
		if (changed || !kms->overlay_tex.initialized) {
			char msg[BS_MSG_MAX];
			breezy_state_format_message(kms->state, msg, sizeof msg);
			overlay_text_update(&kms->overlay_tex, msg);
		}
	}

	kms->device_active = active;

	if (!active) {
		kms->meshes_computed = false;
		return;
	}

	uint32_t dev_w = (cfg.display_width  > 0) ? cfg.display_width
	                                           : (uint32_t)kms->mode.hdisplay;
	uint32_t dev_h = (cfg.display_height > 0) ? cfg.display_height
	                                           : (uint32_t)kms->mode.vdisplay;
	float diag_rad   = cfg.diagonal_fov_deg * (KMS_MATH_PI / 180.0f);
	float lens_ratio = cfg.lens_distance_ratio;

	/* Derive display-distance and size from settings. */
	const struct breezy_display_settings *s = &kms->settings;
	float dist_default = (float)breezy_settings_display_distance_default(s);
	float dist_adj     = breezy_settings_distance_adjusted_size(s, lens_ratio);

	/* Build the connected-only monitor list and slot map.
	 * Layout (centering, viewport offset, wrapping) is based solely on
	 * connected monitors so that disconnected slots don't skew the center. */
	struct dp_monitor_info conn_devs[MAX_USBIP_DEVICES];
	size_t conn_slot[MAX_USBIP_DEVICES];  /* conn_slot[compact_i] = opts slot index */
	size_t conn_count = 0u;
	for (size_t i = 0u; i < opts->device_count; i++) {
		const struct device_runtime *dev = &server->devices[i];
		bool connected = dev->is_gadget_device
		                     ? (dev->udl.gadget_fd >= 0)
		                     : dev->imported;
		if (!connected)
			continue;
		conn_devs[conn_count].x      = opts->devices[i].x;
		conn_devs[conn_count].y      = opts->devices[i].y;
		conn_devs[conn_count].width  = opts->devices[i].decode_width;
		conn_devs[conn_count].height = opts->devices[i].decode_height;
		conn_slot[conn_count]        = i;
		conn_count++;
	}

	/* Resolve wrapping scheme from connected monitors only. */
	const char *resolved_scheme = kms_resolve_wrapping_scheme(
	    s->monitor_wrapping_scheme, s->curved_display,
	    conn_devs, conn_count, dev_w, dev_h);

	float monitor_spacing = (float)s->monitor_spacing / 1000.0f;

	/* Size-adjust and viewport-centre the connected monitor positions. */
	struct dp_monitor_info adj_monitors[MAX_USBIP_DEVICES];
	kms_adjust_monitor_positions(conn_devs, conn_count, dev_w, dev_h, dist_adj,
	                              s->headset_display_as_viewport_center,
	                              s->viewport_offset_x, s->viewport_offset_y,
	                              adj_monitors);

	/*
	 * Zoom-on-focus distances — recomputed every poll, because the zoom switch
	 * only flips display_distance between the focused/all toggle endpoints; it
	 * does not change the layout.  Mirrors the UI enable rule (zoom on iff
	 * display_distance < the "all displays" distance) and GNOME's
	 * display_distance vs display_distance_default split:
	 *   all      = display_distance_default = max(display_distance, start, end)
	 *   focused  = display_distance          (the nearer value when zoom is on)
	 *   enabled  = display_distance < all
	 * zoom_all_distance must equal the distance placements were computed at so
	 * the per-monitor scale (monitor_distance / zoom_all_distance) is 1.0 when
	 * unfocused.
	 */
	kms->zoom_all_distance     = dist_default;
	kms->zoom_focused_distance = (float)s->display_distance;
	kms->zoom_on_focus         = kms->zoom_focused_distance < dist_default;

	/*
	 * If a display is focused (or easing) and the focused-distance setting
	 * changed, retarget it to the new value instead of waiting for it to lose
	 * and regain focus.  Mirrors the references rebinding monitorDistance when
	 * focusedDisplayDistance changes.
	 */
	if (kms->zoom_on_focus && kms->focused_monitor_index >= 0) {
		size_t fi = (size_t)kms->focused_monitor_index;
		float want = kms->zoom_focused_distance;
		float cur_target = kms->zoom_ease_active[fi] ? kms->zoom_ease_target[fi]
		                                             : kms->monitor_distance[fi];
		if (cur_target != want)
			kms_begin_zoom_ease(kms, fi, want, false, kms_realtime_ms());
	}

	/*
	 * Rebuild placements/meshes only when geometry-relevant inputs change.  The
	 * focus index and eased distances are preserved across rebuilds unless the
	 * monitor *layout* itself changes (see layout_changed below) — otherwise a
	 * benign setting like display-size would snap a focused display back out.
	 */
	struct kms_cfg_sig sig;
	memset(&sig, 0, sizeof sig);
	sig.dev_w             = dev_w;
	sig.dev_h             = dev_h;
	sig.diag_rad          = diag_rad;
	sig.lens_ratio        = lens_ratio;
	sig.dist_default      = dist_default;
	sig.dist_adj          = dist_adj;
	sig.monitor_spacing   = monitor_spacing;
	sig.viewport_offset_x = (float)s->viewport_offset_x;
	sig.viewport_offset_y = (float)s->viewport_offset_y;
	sig.curved            = s->curved_display;
	sig.headset_center    = s->headset_display_as_viewport_center;
	/* sig is fully zeroed above, so a bounded copy keeps it NUL-terminated. */
	size_t sch_len = strlen(resolved_scheme);
	if (sch_len >= sizeof(sig.scheme))
		sch_len = sizeof(sig.scheme) - 1;
	memcpy(sig.scheme, resolved_scheme, sch_len);
	sig.mon_count = conn_count;
	for (size_t i = 0u; i < conn_count && i < MAX_USBIP_DEVICES; i++)
		sig.mons[i] = conn_devs[i];

	if (kms->has_cfg_sig && kms->placements_computed && kms->meshes_computed &&
	    memcmp(&sig, &kms->last_cfg_sig, sizeof sig) == 0)
		return;  /* nothing geometry-relevant changed; keep focus + eased distances */

	/*
	 * A layout change (different monitor count or raw rectangles) invalidates the
	 * focus index and per-monitor distances; a pure geometry-param change
	 * (display-size, distance, FOV, spacing, viewport offset) does not — keep the
	 * focused display zoomed.  Compares RAW monitors so display-size, which only
	 * scales dist_adj, never counts as a layout change.
	 */
	bool layout_changed = !kms->has_cfg_sig ||
	    kms->last_cfg_sig.mon_count != sig.mon_count ||
	    memcmp(kms->last_cfg_sig.mons, sig.mons,
	           sig.mon_count * sizeof(sig.mons[0])) != 0;

	kms->last_cfg_sig    = sig;
	kms->has_cfg_sig     = true;
	kms->meshes_computed = false;

	struct dp_fov_details fov_det;
	if (dp_build_fov_details(dev_w, dev_h, diag_rad, lens_ratio, dist_default,
	                          resolved_scheme, s->curved_display, &fov_det) != 0) {
		kms->device_active = false;
		return;
	}

	/*
	 * Projection is built from the FIXED device FOV (independent of display
	 * distance), exactly like KWin CameraController.buildPerspectiveMatrix():
	 *   fovHalfVerticalTangent = diagonalToCrossFOVs(...).heightUnitDistance / 2
	 *   aspect                 = device_width / device_height
	 * Using the raw heightUnitDistance here — NOT defaultDistanceVerticalRadians —
	 * is what makes the "all displays" distance actually recede the monitors
	 * instead of the camera zooming in to compensate.
	 */
	kms->device_aspect = (float)dev_w / (float)dev_h;
	{
		struct dp_fov_lengths fov_len;
		if (dp_diagonal_to_cross_fovs(diag_rad, kms->device_aspect, &fov_len) != 0) {
			kms->device_active = false;
			return;
		}
		kms->device_fov_half_v_tan = fov_len.height_unit * 0.5f;
	}
	kms->device_complete_dist_px = fov_det.complete_dist_px;
	kms->device_lens_dist_px     = fov_det.lens_distance_px;
	kms->display_size            = dist_adj;
	kms->device_size_adj_w_px    = (float)dev_w * dist_adj;
	kms->device_size_adj_h_px    = (float)dev_h * dist_adj;

	/* Override sizeAdjusted* so dp_generate_mesh_vertices uses the correct
	 * scaled viewport reference (same override done for dp_compute_placements). */
	fov_det.size_adj_width  = kms->device_size_adj_w_px;
	fov_det.size_adj_height = kms->device_size_adj_h_px;

	/* dp_compute_placements writes placements[originalIndex] where originalIndex
	 * is the compact array index (0..conn_count-1).  We use a scratch array and
	 * remap to the real slot indices afterwards. */
	struct dp_placement compact_placements[MAX_USBIP_DEVICES];
	if (dp_compute_placements(adj_monitors, conn_count,
	                           dev_w, dev_h, diag_rad, lens_ratio,
	                           dist_default,
	                           kms->device_size_adj_w_px, kms->device_size_adj_h_px,
	                           fov_det.complete_dist_px,
	                           resolved_scheme, s->curved_display, monitor_spacing,
	                           compact_placements) == 0) {
		for (size_t ci = 0u; ci < conn_count; ci++)
			kms->placements[conn_slot[ci]] = compact_placements[ci];
		kms->placements_computed = true;

		/* Capture focus-check inputs for the periodic gaze test. */
		kms->focus_monitor_count = conn_count;
		for (size_t mi = 0u; mi < conn_count && mi < MAX_USBIP_DEVICES; mi++) {
			kms->focus_monitors[mi] = adj_monitors[mi];
			kms->focus_slot_map[mi] = conn_slot[mi];
		}
		strncpy(kms->focus_scheme, resolved_scheme, sizeof(kms->focus_scheme) - 1);
		kms->focus_scheme[sizeof(kms->focus_scheme) - 1] = '\0';
		kms->focus_dev_w        = dev_w;
		kms->focus_dev_h        = dev_h;
		kms->focus_diag_rad     = diag_rad;
		kms->focus_lens_ratio   = lens_ratio;
		kms->focus_dist_default = dist_default;
		kms->focus_curved       = s->curved_display;

		/*
		 * Only a real layout change invalidates focus — reset it and snap
		 * distances to resting.  For a pure geometry-param change (e.g.
		 * display-size) keep the focused display and its eased distance so it
		 * stays zoomed in with the rebuilt geometry instead of re-easing.
		 */
		if (layout_changed) {
			kms->focused_monitor_index = -1;
			kms->last_focus_check_ms   = 0;
			/* Drop any in-flight smooth-follow centring; the slot index no
			 * longer maps to the same display after a layout change. */
			smooth_follow_reset_focus(&kms->sf);
			for (size_t mi = 0u; mi < MAX_USBIP_DEVICES; mi++) {
				kms->monitor_distance[mi] = kms->zoom_all_distance;
				kms->zoom_ease_active[mi] = false;
			}
		}

		/* Pass the resolved scheme string directly so generateMeshVertices() in the
		 * shared JS derives horizontalWrap and verticalWrap independently — matching
		 * CurvableDisplayMesh.qml exactly and correctly handling 'flat' (both false). */
		int curved = s->curved_display ? 1 : 0;

		for (size_t ci = 0u; ci < conn_count; ci++) {
			size_t i = conn_slot[ci];
			int vc = dp_generate_mesh_vertices(
			    &fov_det,
			    (float)adj_monitors[ci].width, (float)adj_monitors[ci].height,
			    resolved_scheme, curved,
			    kms->placements[i].cnx,
			    kms->placements[i].cny,
			    kms->placements[i].cnz,
			    (float *)kms->meshes[i].verts,
			    DISPLAY_MESH_MAX_VERTS);
			kms->meshes[i].vert_count = (vc > 0) ? vc : 0;
		}
		kms->meshes_computed = true;
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

	{
		const char *type_name = drmModeGetConnectorTypeName(connector->connector_type);
		printf("kms: display connected: %s-%u %ux%u@%uHz\n",
		       type_name ? type_name : "Unknown",
		       connector->connector_type_id,
		       kms->mode.hdisplay, kms->mode.vdisplay, kms->mode.vrefresh);
	}

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
		/* Depth buffer so overlapping display quads sort by distance (the
		 * nearer/zoomed display wins) instead of by draw order. */
		EGL_DEPTH_SIZE,      16,
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
 * Renderer initialisation / teardown
 *
 * renderer_init/renderer_destroy cover only the render-cycle resources:
 * DRM fd, GBM, EGL, GL programs, display textures, and the overlay GL
 * texture.  The long-lived breezy_state (addresses, connectivity state)
 * is unaffected and rides through reconnect cycles unchanged.
 * ---------------------------------------------------------------- */

static int renderer_init(struct kms_state *kms, const char *drm_device)
{
	kms->drm_fd      = -1;
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

static void renderer_destroy(struct kms_state *kms)
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
	memset(kms->displays, 0, sizeof(kms->displays));
	kms->display_count = 0;

	display_renderer_destroy(&kms->dr);
	display_renderer_msaa_destroy(&kms->msaa);
	overlay_text_destroy(&kms->overlay_tex);

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

	kms->drm_fd      = -1;
	kms->egl_display = EGL_NO_DISPLAY;
	kms->egl_context = EGL_NO_CONTEXT;
	kms->egl_surface = EGL_NO_SURFACE;
	kms->gbm_surface = NULL;
	kms->gbm_dev     = NULL;

	breezy_settings_stop(kms->settings_handle);
	kms->settings_handle = NULL;
}

/* ----------------------------------------------------------------
 * Per-display DMA-buf texture initialisation
 * ---------------------------------------------------------------- */

static int kms_init_display_tex(struct kms_state *kms, size_t idx,
                                 struct udl_runtime *udl, bool verbose)
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

		if (verbose)
			printf("kms: display %zu DMA-buf texture %ux%u stride=%u\n",
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

	if (verbose)
		printf("kms: display %zu fallback GL texture %ux%u\n", idx, w, h);
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
			if (kms_init_display_tex(kms, i, &server->devices[i].udl,
						 server->opts.verbose) != 0)
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
 * Zoom-on-focus: periodic gaze focus check + per-frame distance easing
 * ---------------------------------------------------------------- */

/*
 * Begin a distance transition for a monitor toward target_distance.
 * needs_sequence_delay defers a focus-gaining ease-in until the previously
 * focused display has eased out (the 150ms + 50ms lead-in), matching the
 * sequenced animation in the references.
 */
static void kms_begin_zoom_ease(struct kms_state *kms, size_t i,
                                float target_distance,
                                bool needs_sequence_delay, uint64_t now_ms)
{
	if (i >= MAX_USBIP_DEVICES)
		return;
	kms->zoom_ease_start[i]     = kms->monitor_distance[i];
	kms->zoom_ease_target[i]    = target_distance;
	kms->zoom_ease_begin_ms[i]  = now_ms;
	kms->zoom_ease_seq_delay[i] = needs_sequence_delay;
	kms->zoom_ease_active[i]    = (kms->monitor_distance[i] != target_distance) || needs_sequence_delay;
}

/*
 * Resolve which display slot the given orientation/position is looking at, via
 * dp_find_focused_monitor().  Builds the compact (consecutive-index) placement
 * array the shared finder expects and remaps the result back to a device slot.
 * Returns -1 for "none" (or when placements aren't ready).
 */
static int kms_detect_focus_slot(struct kms_state *kms,
                                 float qw, float qx, float qy, float qz,
                                 float px, float py, float pz,
                                 bool smooth_follow_enabled)
{
	if (!kms->placements_computed || kms->focus_monitor_count == 0u)
		return -1;

	float focused_ratio = (kms->zoom_all_distance > 0.0f)
	                    ? kms->zoom_focused_distance / kms->zoom_all_distance
	                    : 1.0f;

	struct dp_placement compact[MAX_USBIP_DEVICES];
	int current_compact = -1;
	for (size_t mi = 0u; mi < kms->focus_monitor_count; mi++) {
		compact[mi] = kms->placements[kms->focus_slot_map[mi]];
		if ((int)kms->focus_slot_map[mi] == kms->focused_monitor_index)
			current_compact = (int)mi;
	}

	int nc = dp_find_focused_monitor(
	    compact, kms->focus_monitors, kms->focus_monitor_count,
	    qw, qx, qy, qz, px, py, pz,
	    current_compact, focused_ratio, smooth_follow_enabled,
	    kms->focus_dev_w, kms->focus_dev_h, kms->focus_diag_rad,
	    kms->focus_lens_ratio, kms->focus_dist_default,
	    kms->device_size_adj_w_px, kms->device_size_adj_h_px,
	    kms->focus_scheme, kms->focus_curved);

	return (nc >= 0 && (size_t)nc < kms->focus_monitor_count)
	     ? (int)kms->focus_slot_map[(size_t)nc]
	     : -1;
}

/*
 * Adapter so the reusable smooth_follow state machine can resolve the centred
 * display through this renderer's placement bookkeeping (smooth_follow_focus_fn).
 */
static int kms_sf_focus_cb(void *ctx, const float origin_quat[4], const float pose_pos[3])
{
	struct kms_state *kms = ctx;
	float px = pose_pos ? pose_pos[0] : 0.0f;
	float py = pose_pos ? pose_pos[1] : 0.0f;
	float pz = pose_pos ? pose_pos[2] : 0.0f;
	return kms_detect_focus_slot(kms,
	    origin_quat[0], origin_quat[1], origin_quat[2], origin_quat[3],
	    px, py, pz, true);
}

/*
 * Feed the latest IMU sample into the renderer-agnostic smooth-follow state
 * machine.  All behaviour (easing, slerp-back window, centring math) lives in
 * smooth_follow.c; here we only marshal the pose into its expected shape.
 */
static void kms_update_smooth_follow(struct kms_state *kms, uint64_t now_ms)
{
	struct breezy_imu_pose pose;
	bool have = breezy_imu_try_get_pose(&pose);

	float pose_quat[4]   = { pose.quat_w,    pose.quat_x,    pose.quat_y,    pose.quat_z };
	float origin_quat[4] = { pose.sf_quat_w, pose.sf_quat_x, pose.sf_quat_y, pose.sf_quat_z };
	float pose_pos[3]    = { pose.pos_x, pose.pos_y, pose.pos_z };

	bool enabled = have && pose.sf_enabled && !kms->settings.all_displays_follow_mode;
	smooth_follow_update(&kms->sf, have, enabled,
	                     pose_quat, origin_quat, pose_pos, now_ms,
	                     kms_sf_focus_cb, kms);
}

/*
 * Run the periodic gaze focus check (throttled to KMS_FOCUS_CHECK_MS).  When the
 * focused monitor changes, kick off the matching ease-out / ease-in transitions.
 */
static void kms_update_focus(struct kms_state *kms, uint64_t now_ms)
{
	if (!kms->placements_computed)
		return;
	if (now_ms - kms->last_focus_check_ms < KMS_FOCUS_CHECK_MS)
		return;
	kms->last_focus_check_ms = now_ms;

	/*
	 * Zoom-on-focus disabled (e.g. the user just turned it off): ease any
	 * still-focused display back out to the resting distance, drop focus, and
	 * stop checking.  Without this the last focused display would stay zoomed.
	 */
	if (!kms->zoom_on_focus) {
		if (kms->focused_monitor_index >= 0) {
			kms_begin_zoom_ease(kms, (size_t)kms->focused_monitor_index,
			                    kms->zoom_all_distance, false, now_ms);
			kms->focused_monitor_index = -1;
		}
		return;
	}

	int new_focus;

	/*
	 * When smooth follow is engaged, zoom-on-focus locks onto the followed
	 * (centred) display rather than re-deriving focus from gaze.  The followed
	 * display is rendered centred-in-front while gaze focus is evaluated against
	 * the displays' arc positions, so the two would otherwise disagree.  This
	 * also covers enabling order: follow may have picked its slot before
	 * zoom-on-focus turned on, and the first check here adopts it.
	 */
	int follow_slot = smooth_follow_focus_slot(&kms->sf);
	if (follow_slot >= 0) {
		new_focus = follow_slot;
	} else {
		struct breezy_imu_pose pose;
		if (!breezy_imu_try_get_pose(&pose))
			return;

		/*
		 * While smooth follow is engaged the head pose is the (near-identity)
		 * residual relative to the origin, so gaze focus must be evaluated
		 * against the origin orientation instead — mirroring BreezyDesktop.qml
		 * updateFocus().
		 */
		float fq_w = kms->sf.enabled ? pose.sf_quat_w : pose.quat_w;
		float fq_x = kms->sf.enabled ? pose.sf_quat_x : pose.quat_x;
		float fq_y = kms->sf.enabled ? pose.sf_quat_y : pose.quat_y;
		float fq_z = kms->sf.enabled ? pose.sf_quat_z : pose.quat_z;

		new_focus = kms_detect_focus_slot(kms, fq_w, fq_x, fq_y, fq_z,
		                                  pose.pos_x, pose.pos_y, pose.pos_z,
		                                  kms->sf.enabled);
	}

	if (new_focus == kms->focused_monitor_index)
		return;

	int old_focus = kms->focused_monitor_index;

	/* Ease the outgoing display back out to the resting distance. */
	if (old_focus >= 0 && (size_t)old_focus < MAX_USBIP_DEVICES)
		kms_begin_zoom_ease(kms, (size_t)old_focus, kms->zoom_all_distance, false, now_ms);

	/* Ease the incoming display in; if another display had focus, wait for its
	 * ease-out + pause first (the doubled-timeline / sequenced behaviour). */
	if (new_focus >= 0 && (size_t)new_focus < MAX_USBIP_DEVICES)
		kms_begin_zoom_ease(kms, (size_t)new_focus, kms->zoom_focused_distance,
		                    old_focus >= 0, now_ms);

	kms->focused_monitor_index = new_focus;
}

/* Advance all active distance eases toward their targets for this frame. */
static void kms_step_zoom_eases(struct kms_state *kms, uint64_t now_ms)
{
	for (size_t i = 0u; i < MAX_USBIP_DEVICES; i++) {
		if (!kms->zoom_ease_active[i])
			continue;
		bool gaining = kms->zoom_ease_target[i] < kms->zoom_ease_start[i];
		float dist = kms->monitor_distance[i];
		bool done = true;
		if (dp_ease_distance(kms->zoom_ease_start[i], kms->zoom_ease_target[i],
		                     (float)(now_ms - kms->zoom_ease_begin_ms[i]),
		                     gaining, kms->zoom_ease_seq_delay[i],
		                     &dist, &done) != 0) {
			/* On error, snap to target so we never stall mid-transition. */
			dist = kms->zoom_ease_target[i];
			done = true;
		}
		kms->monitor_distance[i] = dist;
		if (done) {
			kms->monitor_distance[i] = kms->zoom_ease_target[i];
			kms->zoom_ease_active[i] = false;
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
	glClearDepthf(1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	enum breezy_state_mode ovstate = kms->state->mode;

	/* ---- Overlay-only states (no display quads) ---- */
	if (ovstate != BREEZY_STATE_NORMAL) {
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

		if (kms->overlay_tex.tex) {
			float w_half = (float)kms->overlay_tex.tex_w * 0.5f;
			float h_half = (float)kms->overlay_tex.tex_h * 0.5f;
			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			display_renderer_draw_quad_alpha(&kms->dr, &mvp,
							 kms->overlay_tex.tex,
							 w_half, h_half);
			glDisable(GL_BLEND);
		}
		return;
	}

	/*
	 * Glasses active with at least one imported display.
	 * Projection uses the fixed device FOV (half-vertical-tangent + device aspect),
	 * matching KWin CameraController exactly and independent of display distance.
	 * The framebuffer/mode aspect is intentionally NOT used here — the reference
	 * always projects through the device's own aspect ratio.
	 */
	es_perspective_unit(&proj, kms->device_fov_half_v_tan, kms->device_aspect,
	                    KMS_NEAR, KMS_FAR);

	/*
	 * Smooth follow: advance the state machine, then drive the whole camera from
	 * the smooth-follow origin while follow is engaged (or still slerping back
	 * after a disable) so every display follows the head.  Mirrors KWin's
	 * CameraController, which picks smoothFollowOrigin over poseOrientations
	 * whenever smoothFollowEnabled || smoothFollowDisabling.
	 */
	uint64_t now_ms = kms_realtime_ms();
	kms_update_smooth_follow(kms, now_ms);

	ESMatrix view, proj_view;
	const float *cq = smooth_follow_camera_quat(&kms->sf);
	if (cq) {
		es_view_from_quat(&view, cq[0], cq[1], cq[2], cq[3]);
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
	 * Zoom-on-focus: poll gaze focus (self-throttled to KMS_FOCUS_CHECK_MS) and
	 * advance the per-monitor distance eases before placing the display meshes.
	 */
	kms_update_focus(kms, now_ms);
	kms_step_zoom_eases(kms, now_ms);

	/*
	 * Display meshes are opaque: depth-test so a nearer (e.g. focused/zoomed-in)
	 * display occludes a farther one regardless of draw order.  The blended
	 * overlay strip below re-disables depth before compositing.
	 */
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glDepthMask(GL_TRUE);

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

	/*
	 * The smooth-follow centred display must always render above the others, so
	 * its draw is deferred until after the loop (see below) and replayed over a
	 * cleared depth buffer.
	 */
	bool          have_follow_draw = false;
	ESMatrix      follow_mvp;
	GLuint        follow_tex = 0;
	const float  *follow_verts = NULL;
	int           follow_vert_count = 0;

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
			const struct display_mesh *mesh = &kms->meshes[i];
			float ang = kms->placements[i].angle;
			float s   = (kms->zoom_all_distance > 0.0f)
			          ? kms->monitor_distance[i] / kms->zoom_all_distance
			          : 1.0f;
			float ca  = cosf(ang);
			float sa  = sinf(ang);
			float cnx = kms->placements[i].cnx;
			float cny = kms->placements[i].cny;
			float cnz = kms->placements[i].cnz;

			bool centring = smooth_follow_is_centring(&kms->sf, (int)i);
			if (centring) {
				/*
				 * Smooth-follow focused display: hand the placement to the
				 * reusable module, which locks it centred in front of the
				 * (residual) head pose, easing between its arc position and
				 * centred-in-front.  The camera already runs on the origin.
				 */
				smooth_follow_focused_model(&kms->sf, cnx, cny, cnz, ang, s, &model);
			} else {
				/*
				 * Mesh vertices are pre-computed in GL world space relative to the
				 * rotation origin (centerNoRotate).  Apply Y-rotation only — the
				 * arc curvature baked into the vertices is preserved.  Flat displays
				 * produce a 1-segment mesh (4 vertices) with no separate path.
				 *
				 * Zoom-on-focus: scale the monitor's centre position by
				 * s = monitor_distance / zoom_all_distance (matching KWin's
				 * centerNoRotate.times(monitorDistance/allDisplaysDistance)).  The
				 * mesh already bakes centerNoRotate, so we add the delta (s-1)·R·cn
				 * as a post-rotation world translation, moving the centre without
				 * rescaling the display's own size or curvature.
				 */
				float rcn_x = ca * cnx + sa * cnz;
				float rcn_z = -sa * cnx + ca * cnz;
				float tdx = (s - 1.0f) * rcn_x;
				float tdy = (s - 1.0f) * cny;
				float tdz = (s - 1.0f) * rcn_z;
				es_display_model(&model, ang, tdx, tdy, tdz);
			}
			es_multiply(&mvp, &model, &proj_view);
			if (centring) {
				/* Defer; replayed last so it occludes every other display. */
				follow_mvp        = mvp;
				follow_tex        = tex;
				follow_verts      = (const float *)mesh->verts;
				follow_vert_count = mesh->vert_count;
				have_follow_draw  = true;
			} else {
				display_renderer_draw_mesh(&kms->dr, &mvp, tex,
				                          (const float *)mesh->verts, mesh->vert_count);
			}

			float w_half  = (float)server->opts.devices[i].decode_width  * 0.5f;
			float h_half  = (float)server->opts.devices[i].decode_height * 0.5f;
			float left_x  = kms->placements[i].cnx - w_half;
			float right_x = kms->placements[i].cnx + w_half;
			float quad_top = kms->placements[i].cny + h_half;
			if (server->opts.verbose)
				printf("kms_overlay: slot=%zu cnx=%.0f w=%.0f left=%.0f right=%.0f\n",
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

	/*
	 * Replay the deferred smooth-follow display last.  Clearing the depth buffer
	 * first lets it occlude every other display regardless of its world distance,
	 * while keeping the normal depth test so its own curved mesh self-sorts.
	 */
	if (have_follow_draw) {
		glClear(GL_DEPTH_BUFFER_BIT);
		display_renderer_draw_mesh(&kms->dr, &follow_mvp, follow_tex,
		                          follow_verts, follow_vert_count);
	}

	/* Midpoint of the combined left-edge to right-edge span. */
	float center_x_px = first_ext ? 0.0f : (min_x_px + max_x_px) * 0.5f;
	{
		static size_t last_visible = SIZE_MAX;
		static float  last_cx = 0.0f;
		if (visible_count != last_visible || center_x_px != last_cx) {
			if (server->opts.verbose)
				printf("kms_overlay: visible=%zu meshes=%d "
				       "min_x=%.0f max_x=%.0f center_x=%.0f top_y=%.0f\n",
				       visible_count, (int)kms->meshes_computed,
				       min_x_px, max_x_px, center_x_px, top_y_px);
			last_visible = visible_count;
			last_cx = center_x_px;
		}
	}

	/* ---- Overlay info strip above the display group ---- */
	if (kms->overlay_tex.tex) {
		float ot_w_half = (float)kms->overlay_tex.tex_w * 0.5f;
		float ot_h_half = (float)kms->overlay_tex.tex_h * 0.5f;
		float gap_px    = ot_h_half;                       /* one text-height gap */
		float ot_y      = top_y_px + gap_px + ot_h_half;

		if (kms->meshes_computed) {
			/* Place at arc radius, horizontally centred over the display group. */
			es_display_model(&model, 0.0f, center_x_px, ot_y, -arc_r_px);
		} else {
			es_display_model(&model, 0.0f, 0.0f, ot_y, -arc_r_px);
		}
		es_multiply(&mvp, &model, &proj_view);

		/* HUD label: composite over the scene without depth occlusion. */
		glDisable(GL_DEPTH_TEST);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		display_renderer_draw_quad_alpha(&kms->dr, &mvp,
						 kms->overlay_tex.tex,
						 ot_w_half, ot_h_half);
		glDisable(GL_BLEND);
	}

	/* Leave depth test disabled for the next frame's overlay-only path. */
	glDisable(GL_DEPTH_TEST);
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
	kms->device_fov_half_v_tan    = tanf(KMS_CAMERA_FOV * 0.5f * (KMS_MATH_PI / 180.0f));
	kms->device_aspect            = (float)kms->mode.hdisplay / (float)kms->mode.vdisplay;
	kms->last_config_poll         = 0;
	kms->display_size             = 1.0f;
	kms->device_size_adj_w_px     = 0.0f;
	kms->device_size_adj_h_px     = 0.0f;
	kms->meshes_computed          = false;
	kms->zoom_all_distance        = 1.0f;
	kms->zoom_focused_distance    = 1.0f;
	kms->zoom_on_focus            = false;
	kms->focused_monitor_index    = -1;
	kms->last_focus_check_ms      = 0;
	smooth_follow_init(&kms->sf);
	for (size_t mi = 0u; mi < MAX_USBIP_DEVICES; mi++) {
		kms->monitor_distance[mi] = 1.0f;
		kms->zoom_ease_active[mi] = false;
	}
	kms->settings_handle = breezy_settings_start(&kms->settings);
	if (!kms->settings_handle) {
		fprintf(stderr, "kms: failed to start settings watcher\n");
		return -1;
	}
	breezy_driver_control_update(&kms->settings);

	if (!kms->settings.disable_anti_aliasing)
		display_renderer_msaa_init(&kms->msaa,
					   (GLsizei)kms->mode.hdisplay,
					   (GLsizei)kms->mode.vdisplay, 0);
	/* Prime the overlay texture before the first frame. */
	{
		char msg[BS_MSG_MAX];
		breezy_state_format_message(kms->state, msg, sizeof msg);
		overlay_text_update(&kms->overlay_tex, msg);
	}
	kms_update_display_textures(kms, server);
	display_renderer_msaa_bind(&kms->msaa);
	kms_render_frame(kms, server);
	display_renderer_msaa_resolve_and_unbind(&kms->msaa);
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

	uint64_t last_frame_ms = 0;

	while (!stop_requested) {
		int waiting_for_flip = 1;
		struct gbm_bo *next_bo;
		struct drm_fb *next_fb;
		int ret;

		bool prev_disable_aa = kms->settings.disable_anti_aliasing;
		bool settings_dirty =
		    breezy_settings_consume_if_changed(kms->settings_handle, &kms->settings);
		if (settings_dirty) {
			breezy_driver_control_update(&kms->settings);
			if (kms->settings.disable_anti_aliasing != prev_disable_aa) {
				display_renderer_msaa_destroy(&kms->msaa);
				if (!kms->settings.disable_anti_aliasing)
					display_renderer_msaa_init(&kms->msaa,
								   (GLsizei)kms->mode.hdisplay,
								   (GLsizei)kms->mode.vdisplay, 0);
			}
		}
		kms_poll_device_config(kms, server, settings_dirty);

		/* Honour framerate_cap: skip this iteration if not enough time has passed. */
		if (kms->settings.framerate_cap > 0.0) {
			uint64_t now_ms   = kms_realtime_ms();
			uint64_t min_gap  = (uint64_t)(1000.0 / kms->settings.framerate_cap);
			if (now_ms - last_frame_ms < min_gap)
				continue;
			last_frame_ms = now_ms;
		}

		kms_update_display_textures(kms, server);
		display_renderer_msaa_bind(&kms->msaa);
		kms_render_frame(kms, server);
		display_renderer_msaa_resolve_and_unbind(&kms->msaa);
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
 * Background USB/IP server thread
 * ---------------------------------------------------------------- */

static void *server_thread_main(void *arg)
{
	struct server_runtime *server = arg;

	(void)run_server(server);
	return NULL;
}

/* ----------------------------------------------------------------
 * Wired Ethernet direct-link support
 * ---------------------------------------------------------------- */

/* IP addressing for the direct wired Ethernet link.  Different subnet from
 * the OTG link (192.168.7.x) so the two can coexist without routing conflicts.
 * IP assignment and DHCP serving are handled by systemd-networkd (see
 * setup_system.sh); these constants are used for mDNS pinning and overlay
 * display only. */
#define ETH_LINK_IP_CIDR  "192.168.8.2/30"
#define ETH_HOST_IP       "192.168.8.1"

/*
 * Find the first physical wired Ethernet interface that is not the OTG gadget
 * netdev and not a wireless interface.  Writes the interface name into out[cap]
 * and returns 0 on success, -1 if none is found.
 *
 * Detection criteria (all must hold):
 *   - Not "lo" (loopback)
 *   - Not exclude_iface (the USB gadget netdev, e.g. "usb0")
 *   - /sys/class/net/<name>/wireless/ does NOT exist  (not Wi-Fi)
 *   - /sys/class/net/<name>/device    DOES exist      (physical hardware, not virtual)
 */
static int detect_wired_eth_iface(const char *exclude_iface,
				   char *out, size_t cap)
{
	DIR *d;
	struct dirent *e;

	if (!out || cap == 0)
		return -1;

	d = opendir("/sys/class/net");
	if (!d)
		return -1;

	while ((e = readdir(d)) != NULL) {
		/* /sys/class/net/<name>/wireless  and  .../device are at most
		 * "/sys/class/net/" (15) + IFNAMSIZ (16) + "/wireless" (9) + NUL = 41 */
		char path[64];
		struct stat st;
		const char *name = e->d_name;

		if (name[0] == '.')
			continue;
		if (strcmp(name, "lo") == 0)
			continue;
		if (exclude_iface && strcmp(name, exclude_iface) == 0)
			continue;
		/* Interface names are bounded by IFNAMSIZ (16); skip anything longer. */
		if (strlen(name) >= 16)
			continue;

		snprintf(path, sizeof(path), "/sys/class/net/%s/wireless", name);
		if (stat(path, &st) == 0 && S_ISDIR(st.st_mode))
			continue;

		snprintf(path, sizeof(path), "/sys/class/net/%s/device", name);
		if (stat(path, &st) != 0)
			continue;

		snprintf(out, cap, "%s", name);
		closedir(d);
		return 0;
	}

	closedir(d);
	return -1;
}

/* ----------------------------------------------------------------
 * main()
 * ---------------------------------------------------------------- */

int main(int argc, char **argv)
{
	struct runtime_cli_args cli;
	struct server_runtime server;
	struct kms_state kms;
	struct breezy_state bstate;
	struct usb_gadget_config gadget_cfg;
	struct usb_gadget_state gadget_state;
	struct link_services_state link_state;
	struct link_services_config link_cfg;
	bool link_cfg_valid = false;
	struct link_services_state eth_link_state;
	struct link_services_config eth_link_cfg;
	bool eth_link_cfg_valid = false;
	char eth_iface[32] = "";
	pthread_t srv_thread;
	bool srv_thread_created = false;
	int exit_code = EXIT_FAILURE;
	const char *drm_device = NULL;

	memset(&server, 0, sizeof(server));
	server.listen_fd = -1;
	memset(&kms, 0, sizeof(kms));
	kms.drm_fd = -1;
	memset(&bstate, 0, sizeof(bstate));
	kms.state = &bstate;
	memset(&gadget_state, 0, sizeof(gadget_state));
	memset(&link_state, 0, sizeof(link_state));
	memset(&eth_link_state, 0, sizeof(eth_link_state));

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
		link_cfg.mtu = gadget_cfg.rndis_mtu;
		/*
		 * Pin the link mDNS name (breezy.local) to the gadget's own
		 * address on this link so it tracks the configured CIDR rather
		 * than the default. Strip the "/prefix" suffix.
		 */
		if (gadget_cfg.rndis_ip_cidr[0]) {
			char *slash;
			snprintf(link_cfg.link_ip, sizeof(link_cfg.link_ip), "%s",
				 gadget_cfg.rndis_ip_cidr);
			slash = strchr(link_cfg.link_ip, '/');
			if (slash)
				*slash = '\0';
		}
		link_services_start(&link_cfg, &link_state);
		link_cfg_valid = true;
	}

	/*
	 * Start DHCP + mDNS on the wired Ethernet port for hosts that connect
	 * via a direct cable rather than the OTG USB port.  Auto-detects the
	 * first physical non-wireless interface that isn't the gadget netdev.
	 * Assigns the SBC 192.168.8.2/30 and hands the host 192.168.8.1.
	 * breezy.local resolves correctly from either path because mDNS is
	 * link-local: each host sees the record on its own interface.
	 */
	if (detect_wired_eth_iface(gadget_state.netdev_name[0]
				       ? gadget_state.netdev_name : NULL,
				   eth_iface, sizeof(eth_iface)) == 0) {
		memset(&eth_link_cfg, 0, sizeof(eth_link_cfg));
		snprintf(eth_link_cfg.iface,    sizeof(eth_link_cfg.iface),
			 "%s", eth_iface);
		snprintf(eth_link_cfg.link_ip,  sizeof(eth_link_cfg.link_ip),
			 "%s", ETH_LINK_IP_CIDR);
		/* Strip the "/prefix" suffix to get the bare IP for mDNS pinning. */
		{
			char *slash = strchr(eth_link_cfg.link_ip, '/');
			if (slash)
				*slash = '\0';
		}
		/* Publish breezy.local on this interface.  Use the OTG name when
		 * available so both paths share the same friendly name; fall back
		 * to the hardcoded default so mDNS works even without the gadget. */
		snprintf(eth_link_cfg.link_name, sizeof(eth_link_cfg.link_name),
			 "%s", (link_cfg_valid && link_cfg.link_name[0])
				 ? link_cfg.link_name : "breezy");

		link_services_start(&eth_link_cfg, &eth_link_state);
		eth_link_cfg_valid = true;
		printf("Direct Ethernet link active on %s (%s, host %s)\n",
		       eth_iface, ETH_LINK_IP_CIDR, ETH_HOST_IP);
	}

	if (server_runtime_init_devices(&server) != 0) {
		fprintf(stderr, "Failed to initialise USB/IP device runtimes\n");
		goto out;
	}
	atomic_init(&server.viewer_stop_requested, false);

	/*
	 * Populate address state once — these fields survive renderer reconnect
	 * cycles and are never wiped by renderer_destroy().
	 */
	breezy_state_set_addresses(&bstate,
				   link_cfg_valid ? &link_cfg : NULL,
				   gadget_state.netdev_name[0]
					? gadget_state.netdev_name : NULL);
	if (eth_link_cfg_valid)
		breezy_state_set_eth_link(&bstate,
					  eth_iface,
					  eth_link_cfg.link_ip,
					  ETH_HOST_IP);

	if (pthread_create(&srv_thread, NULL, server_thread_main, &server) != 0) {
		perror("pthread_create server");
		goto out;
	}
	srv_thread_created = true;

	/*
	 * Renderer reconnect loop.  The USB/IP server thread and all decode
	 * sessions remain running throughout; only the DRM/GBM/EGL/GL layer
	 * is torn down and rebuilt on each reconnect.
	 */
	while (!stop_requested) {
		if (renderer_init(&kms, drm_device) != 0) {
			fprintf(stderr, "kms: renderer init failed, retrying in 1 s\n");
			nanosleep(&(struct timespec){1, 0}, NULL);
			continue;
		}

		if (kms_init_display_textures(&kms, &server) != 0) {
			fprintf(stderr, "kms: display texture init failed, retrying in 1 s\n");
			renderer_destroy(&kms);
			nanosleep(&(struct timespec){1, 0}, NULL);
			continue;
		}

		printf("DisplayLink KMS compositor running with %zu display slot(s)\n",
		       kms.display_count);

		kms_run(&kms, &server);
		renderer_destroy(&kms);

		if (!stop_requested)
			printf("kms: display disconnected, waiting for reconnect\n");
	}

	exit_code = EXIT_SUCCESS;

	out:
	stop_requested = 1;
	if (srv_thread_created) {
		if (server.listen_fd >= 0)
			shutdown(server.listen_fd, SHUT_RDWR);
		pthread_join(srv_thread, NULL);
	}
	server_runtime_destroy(&server);
	link_services_stop(&link_state);
	link_services_stop(&eth_link_state);
	usb_gadget_teardown(&gadget_state);
	breezy_imu_destroy();
	dp_destroy();
	return exit_code;
}
