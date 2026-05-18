#define _POSIX_C_SOURCE 200809L

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES2/gl2.h>
#include <fcntl.h>
#include <gbm.h>
#include <getopt.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <drm_fourcc.h>
#include <xf86drm.h>
#include <xf86drmMode.h>

#define ARRAY_SIZE(values) (sizeof(values) / sizeof((values)[0]))

#define DEFAULT_DRM_DEVICE "/dev/dri/card0"
#define DEFAULT_STREAM_COUNT 3u
#define DEFAULT_STREAM_WIDTH 320u
#define DEFAULT_STREAM_HEIGHT 180u
#define DEMO_PI 3.14159265358979323846f

static volatile sig_atomic_t stop_requested = 0;

struct options {
    const char *drm_device_path;
    unsigned int stream_count;
    unsigned int stream_width;
    unsigned int stream_height;
    unsigned int max_frames;
    bool verbose;
};

struct drm_pipeline {
    uint32_t connector_id;
    uint32_t crtc_id;
    drmModeModeInfo mode;
    drmModeCrtc *saved_crtc;
};

struct drm_fb {
    int fd;
    uint32_t fb_id;
};

struct renderer {
    int drm_fd;
    struct drm_pipeline pipeline;
    struct gbm_device *gbm_device;
    struct gbm_surface *gbm_surface;
    EGLDisplay egl_display;
    EGLContext egl_context;
    EGLSurface egl_surface;
    GLuint program;
    GLuint vertex_buffer;
    GLint uniform_mvp;
    GLint uniform_sampler;
    GLuint attrib_position;
    GLuint attrib_texcoord;
    struct gbm_bo *previous_bo;
    uint32_t width;
    uint32_t height;
};

struct stream_surface {
    GLuint texture_id;
    uint8_t *pixels;
    uint32_t width;
    uint32_t height;
    float phase_offset;
};

struct vertex {
    float position[3];
    float uv[2];
};

static const struct vertex quad_vertices[] = {
    {{-0.5f, -0.5f, 0.0f}, {0.0f, 1.0f}},
    {{ 0.5f, -0.5f, 0.0f}, {1.0f, 1.0f}},
    {{ 0.5f,  0.5f, 0.0f}, {1.0f, 0.0f}},
    {{-0.5f, -0.5f, 0.0f}, {0.0f, 1.0f}},
    {{ 0.5f,  0.5f, 0.0f}, {1.0f, 0.0f}},
    {{-0.5f,  0.5f, 0.0f}, {0.0f, 0.0f}},
};

static void request_stop(int signo)
{
    (void)signo;
    stop_requested = 1;
}

static void options_init(struct options *opts)
{
    memset(opts, 0, sizeof(*opts));
    opts->drm_device_path = DEFAULT_DRM_DEVICE;
    opts->stream_count = DEFAULT_STREAM_COUNT;
    opts->stream_width = DEFAULT_STREAM_WIDTH;
    opts->stream_height = DEFAULT_STREAM_HEIGHT;
}

static void usage(const char *argv0)
{
    fprintf(stderr,
            "Usage: %s [options]\n"
            "\n"
            "A small Breezy Box DRM/KMS + GBM/EGL/GLES scene demo. Each demo\n"
            "stream is uploaded as a texture and rendered onto its own floating\n"
            "panel in a 3D scene.\n"
            "\n"
            "Options:\n"
            "  --device <path>          DRM device node (default: %s)\n"
            "  --stream-count <count>   Number of textured panels to render (default: %u)\n"
            "  --stream-width <pixels>  Width of each demo texture (default: %u)\n"
            "  --stream-height <pixels> Height of each demo texture (default: %u)\n"
            "  --frames <count>         Exit after rendering N frames (default: run until SIGINT)\n"
            "  --verbose                Print selected mode and renderer info\n"
            "  -h, --help               Show this help text\n",
            argv0,
            DEFAULT_DRM_DEVICE,
            DEFAULT_STREAM_COUNT,
            DEFAULT_STREAM_WIDTH,
            DEFAULT_STREAM_HEIGHT);
}

static int parse_unsigned_arg(const char *text, unsigned int *out)
{
    char *end = NULL;
    unsigned long value;

    if (!text || !*text) {
        return -1;
    }

    value = strtoul(text, &end, 10);
    if (!end || *end != '\0' || value == 0ul || value > UINT32_MAX) {
        return -1;
    }

    *out = (unsigned int)value;
    return 0;
}

static int parse_args(int argc, char **argv, struct options *opts)
{
    static const struct option long_options[] = {
        {"device", required_argument, NULL, 'd'},
        {"stream-count", required_argument, NULL, 'n'},
        {"stream-width", required_argument, NULL, 'w'},
        {"stream-height", required_argument, NULL, 'h'},
        {"frames", required_argument, NULL, 'f'},
        {"verbose", no_argument, NULL, 'v'},
        {"help", no_argument, NULL, '?'},
        {NULL, 0, NULL, 0},
    };
    int opt;
    int option_index = 0;

    while ((opt = getopt_long(argc, argv, "", long_options, &option_index)) != -1) {
        switch (opt) {
        case 'd':
            opts->drm_device_path = optarg;
            break;
        case 'n':
            if (parse_unsigned_arg(optarg, &opts->stream_count) != 0) {
                fprintf(stderr, "invalid --stream-count value: %s\n", optarg);
                return -1;
            }
            break;
        case 'w':
            if (parse_unsigned_arg(optarg, &opts->stream_width) != 0) {
                fprintf(stderr, "invalid --stream-width value: %s\n", optarg);
                return -1;
            }
            break;
        case 'h':
            if (parse_unsigned_arg(optarg, &opts->stream_height) != 0) {
                fprintf(stderr, "invalid --stream-height value: %s\n", optarg);
                return -1;
            }
            break;
        case 'f':
            if (parse_unsigned_arg(optarg, &opts->max_frames) != 0) {
                fprintf(stderr, "invalid --frames value: %s\n", optarg);
                return -1;
            }
            break;
        case 'v':
            opts->verbose = true;
            break;
        case '?':
            usage(argv[0]);
            return 1;
        default:
            usage(argv[0]);
            return -1;
        }
    }

    if (optind != argc) {
        usage(argv[0]);
        return -1;
    }

    return 0;
}

static double monotonic_seconds(void)
{
    struct timespec now;

    clock_gettime(CLOCK_MONOTONIC, &now);
    return (double)now.tv_sec + (double)now.tv_nsec / 1000000000.0;
}

static void mat4_identity(float *out)
{
    memset(out, 0, sizeof(float) * 16u);
    out[0] = 1.0f;
    out[5] = 1.0f;
    out[10] = 1.0f;
    out[15] = 1.0f;
}

static void mat4_multiply(float *out, const float *left, const float *right)
{
    float tmp[16];
    unsigned int row;
    unsigned int col;

    for (col = 0; col < 4u; ++col) {
        for (row = 0; row < 4u; ++row) {
            tmp[col * 4u + row] =
                left[0u * 4u + row] * right[col * 4u + 0u] +
                left[1u * 4u + row] * right[col * 4u + 1u] +
                left[2u * 4u + row] * right[col * 4u + 2u] +
                left[3u * 4u + row] * right[col * 4u + 3u];
        }
    }

    memcpy(out, tmp, sizeof(tmp));
}

static void mat4_translation(float *out, float x, float y, float z)
{
    mat4_identity(out);
    out[12] = x;
    out[13] = y;
    out[14] = z;
}

static void mat4_scale(float *out, float x, float y, float z)
{
    mat4_identity(out);
    out[0] = x;
    out[5] = y;
    out[10] = z;
}

static void mat4_rotation_x(float *out, float radians)
{
    const float cosine = cosf(radians);
    const float sine = sinf(radians);

    mat4_identity(out);
    out[5] = cosine;
    out[6] = sine;
    out[9] = -sine;
    out[10] = cosine;
}

static void mat4_rotation_y(float *out, float radians)
{
    const float cosine = cosf(radians);
    const float sine = sinf(radians);

    mat4_identity(out);
    out[0] = cosine;
    out[2] = -sine;
    out[8] = sine;
    out[10] = cosine;
}

static void mat4_perspective(float *out, float fovy_radians, float aspect, float near_z, float far_z)
{
    const float f = 1.0f / tanf(fovy_radians * 0.5f);

    memset(out, 0, sizeof(float) * 16u);
    out[0] = f / aspect;
    out[5] = f;
    out[10] = (far_z + near_z) / (near_z - far_z);
    out[11] = -1.0f;
    out[14] = (2.0f * far_z * near_z) / (near_z - far_z);
}

static void drm_fb_destroy_callback(struct gbm_bo *bo, void *data)
{
    struct drm_fb *fb = data;

    (void)bo;
    if (!fb) {
        return;
    }

    if (fb->fb_id != 0u) {
        drmModeRmFB(fb->fd, fb->fb_id);
    }
    free(fb);
}

static struct drm_fb *drm_fb_get_from_bo(int drm_fd, struct gbm_bo *bo)
{
    struct drm_fb *fb = gbm_bo_get_user_data(bo);

    if (fb) {
        return fb;
    }

    fb = calloc(1u, sizeof(*fb));
    if (!fb) {
        return NULL;
    }

    fb->fd = drm_fd;

    {
        const uint32_t width = gbm_bo_get_width(bo);
        const uint32_t height = gbm_bo_get_height(bo);
        const uint32_t stride = gbm_bo_get_stride(bo);
        const uint32_t handle = gbm_bo_get_handle(bo).u32;
        uint32_t handles[4] = {handle, 0u, 0u, 0u};
        uint32_t strides[4] = {stride, 0u, 0u, 0u};
        uint32_t offsets[4] = {0u, 0u, 0u, 0u};

        if (drmModeAddFB2(drm_fd,
                          width,
                          height,
                          DRM_FORMAT_XRGB8888,
                          handles,
                          strides,
                          offsets,
                          &fb->fb_id,
                          0u) != 0 &&
            drmModeAddFB(drm_fd, width, height, 24u, 32u, stride, handle, &fb->fb_id) != 0) {
            free(fb);
            return NULL;
        }
    }

    gbm_bo_set_user_data(bo, fb, drm_fb_destroy_callback);
    return fb;
}

static int find_active_pipeline(int drm_fd, struct drm_pipeline *pipeline)
{
    drmModeRes *resources = NULL;
    drmModeConnector *connector = NULL;
    drmModeEncoder *encoder = NULL;
    int result = -1;
    uint32_t connector_index;

    memset(pipeline, 0, sizeof(*pipeline));

    resources = drmModeGetResources(drm_fd);
    if (!resources) {
        perror("drmModeGetResources");
        return -1;
    }

    for (connector_index = 0u; connector_index < (uint32_t)resources->count_connectors; ++connector_index) {
        uint32_t encoder_index;

        connector = drmModeGetConnector(drm_fd, resources->connectors[connector_index]);
        if (!connector) {
            continue;
        }

        if (connector->connection != DRM_MODE_CONNECTED || connector->count_modes == 0) {
            drmModeFreeConnector(connector);
            connector = NULL;
            continue;
        }

        if (connector->encoder_id != 0u) {
            encoder = drmModeGetEncoder(drm_fd, connector->encoder_id);
        }

        if (!encoder) {
            for (encoder_index = 0u; encoder_index < (uint32_t)connector->count_encoders; ++encoder_index) {
                drmModeEncoder *candidate = drmModeGetEncoder(drm_fd, connector->encoders[encoder_index]);

                if (!candidate) {
                    continue;
                }

                for (int crtc_index = 0; crtc_index < resources->count_crtcs; ++crtc_index) {
                    if ((candidate->possible_crtcs & (1u << crtc_index)) == 0u) {
                        continue;
                    }

                    encoder = candidate;
                    pipeline->crtc_id = resources->crtcs[crtc_index];
                    break;
                }

                if (encoder == candidate) {
                    break;
                }

                drmModeFreeEncoder(candidate);
            }
        }

        if (!encoder) {
            drmModeFreeConnector(connector);
            connector = NULL;
            continue;
        }

        pipeline->connector_id = connector->connector_id;
        if (pipeline->crtc_id == 0u) {
            pipeline->crtc_id = encoder->crtc_id;
        }
        pipeline->mode = connector->modes[0];
        pipeline->saved_crtc = drmModeGetCrtc(drm_fd, pipeline->crtc_id);
        result = 0;
        break;
    }

    if (encoder) {
        drmModeFreeEncoder(encoder);
    }
    if (connector) {
        drmModeFreeConnector(connector);
    }
    drmModeFreeResources(resources);
    return result;
}

static GLuint compile_shader(GLenum type, const char *source)
{
    GLuint shader = glCreateShader(type);
    GLint compiled = GL_FALSE;

    glShaderSource(shader, 1, &source, NULL);
    glCompileShader(shader);
    glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
    if (compiled != GL_TRUE) {
        GLint log_length = 0;
        char *log_buffer = NULL;

        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &log_length);
        if (log_length > 0) {
            log_buffer = calloc((size_t)log_length + 1u, 1u);
        }
        if (log_buffer) {
            glGetShaderInfoLog(shader, log_length, NULL, log_buffer);
            fprintf(stderr, "shader compile failed: %s\n", log_buffer);
            free(log_buffer);
        } else {
            fprintf(stderr, "shader compile failed\n");
        }
        glDeleteShader(shader);
        return 0u;
    }

    return shader;
}

static int renderer_create_program(struct renderer *renderer)
{
    static const char *vertex_source =
        "attribute vec3 a_position;\n"
        "attribute vec2 a_texcoord;\n"
        "uniform mat4 u_mvp;\n"
        "varying vec2 v_texcoord;\n"
        "void main(void) {\n"
        "  gl_Position = u_mvp * vec4(a_position, 1.0);\n"
        "  v_texcoord = a_texcoord;\n"
        "}\n";
    static const char *fragment_source =
        "precision mediump float;\n"
        "uniform sampler2D u_texture;\n"
        "varying vec2 v_texcoord;\n"
        "void main(void) {\n"
        "  vec4 color = texture2D(u_texture, v_texcoord);\n"
        "  float vignette = 0.88 + 0.12 * smoothstep(0.0, 0.55, 1.0 - distance(v_texcoord, vec2(0.5)));\n"
        "  gl_FragColor = vec4(color.rgb * vignette, 1.0);\n"
        "}\n";
    GLuint vertex_shader = 0u;
    GLuint fragment_shader = 0u;
    GLuint program = 0u;
    GLint linked = GL_FALSE;

    vertex_shader = compile_shader(GL_VERTEX_SHADER, vertex_source);
    fragment_shader = compile_shader(GL_FRAGMENT_SHADER, fragment_source);
    if (!vertex_shader || !fragment_shader) {
        goto fail;
    }

    program = glCreateProgram();
    glAttachShader(program, vertex_shader);
    glAttachShader(program, fragment_shader);
    glBindAttribLocation(program, 0u, "a_position");
    glBindAttribLocation(program, 1u, "a_texcoord");
    glLinkProgram(program);
    glGetProgramiv(program, GL_LINK_STATUS, &linked);
    if (linked != GL_TRUE) {
        GLint log_length = 0;
        char *log_buffer = NULL;

        glGetProgramiv(program, GL_INFO_LOG_LENGTH, &log_length);
        if (log_length > 0) {
            log_buffer = calloc((size_t)log_length + 1u, 1u);
        }
        if (log_buffer) {
            glGetProgramInfoLog(program, log_length, NULL, log_buffer);
            fprintf(stderr, "program link failed: %s\n", log_buffer);
            free(log_buffer);
        } else {
            fprintf(stderr, "program link failed\n");
        }
        goto fail;
    }

    renderer->program = program;
    renderer->attrib_position = 0u;
    renderer->attrib_texcoord = 1u;
    renderer->uniform_mvp = glGetUniformLocation(program, "u_mvp");
    renderer->uniform_sampler = glGetUniformLocation(program, "u_texture");

    glGenBuffers(1, &renderer->vertex_buffer);
    glBindBuffer(GL_ARRAY_BUFFER, renderer->vertex_buffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quad_vertices), quad_vertices, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0u);

    glDeleteShader(vertex_shader);
    glDeleteShader(fragment_shader);
    return 0;

fail:
    if (program) {
        glDeleteProgram(program);
    }
    if (vertex_shader) {
        glDeleteShader(vertex_shader);
    }
    if (fragment_shader) {
        glDeleteShader(fragment_shader);
    }
    return -1;
}

static int renderer_init(struct renderer *renderer, const struct options *opts)
{
    static const EGLint config_attributes[] = {
        EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
        EGL_RED_SIZE, 8,
        EGL_GREEN_SIZE, 8,
        EGL_BLUE_SIZE, 8,
        EGL_ALPHA_SIZE, 0,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
        EGL_NONE,
    };
    static const EGLint context_attributes[] = {
        EGL_CONTEXT_CLIENT_VERSION, 2,
        EGL_NONE,
    };
    EGLConfig config = NULL;
    EGLint num_configs = 0;
    PFNEGLGETPLATFORMDISPLAYEXTPROC get_platform_display = NULL;

    memset(renderer, 0, sizeof(*renderer));
    renderer->drm_fd = -1;
    renderer->egl_display = EGL_NO_DISPLAY;
    renderer->egl_context = EGL_NO_CONTEXT;
    renderer->egl_surface = EGL_NO_SURFACE;

    renderer->drm_fd = open(opts->drm_device_path, O_RDWR | O_CLOEXEC);
    if (renderer->drm_fd < 0) {
        perror(opts->drm_device_path);
        return -1;
    }

    if (find_active_pipeline(renderer->drm_fd, &renderer->pipeline) != 0) {
        fprintf(stderr, "unable to find a connected DRM connector and CRTC\n");
        return -1;
    }

    renderer->width = renderer->pipeline.mode.hdisplay;
    renderer->height = renderer->pipeline.mode.vdisplay;

    renderer->gbm_device = gbm_create_device(renderer->drm_fd);
    if (!renderer->gbm_device) {
        perror("gbm_create_device");
        return -1;
    }

    renderer->gbm_surface = gbm_surface_create(renderer->gbm_device,
                                               renderer->width,
                                               renderer->height,
                                               GBM_FORMAT_XRGB8888,
                                               GBM_BO_USE_SCANOUT | GBM_BO_USE_RENDERING);
    if (!renderer->gbm_surface) {
        perror("gbm_surface_create");
        return -1;
    }

    get_platform_display = (PFNEGLGETPLATFORMDISPLAYEXTPROC)
        eglGetProcAddress("eglGetPlatformDisplayEXT");
    if (get_platform_display) {
        renderer->egl_display = get_platform_display(EGL_PLATFORM_GBM_KHR,
                                                     renderer->gbm_device,
                                                     NULL);
    }
    if (renderer->egl_display == EGL_NO_DISPLAY) {
        renderer->egl_display = eglGetDisplay((EGLNativeDisplayType)renderer->gbm_device);
    }
    if (renderer->egl_display == EGL_NO_DISPLAY) {
        fprintf(stderr, "eglGetDisplay failed\n");
        return -1;
    }

    if (!eglInitialize(renderer->egl_display, NULL, NULL)) {
        fprintf(stderr, "eglInitialize failed\n");
        return -1;
    }

    if (!eglChooseConfig(renderer->egl_display,
                         config_attributes,
                         &config,
                         1,
                         &num_configs) ||
        num_configs != 1) {
        fprintf(stderr, "eglChooseConfig failed\n");
        return -1;
    }

    if (!eglBindAPI(EGL_OPENGL_ES_API)) {
        fprintf(stderr, "eglBindAPI failed\n");
        return -1;
    }

    renderer->egl_context = eglCreateContext(renderer->egl_display,
                                             config,
                                             EGL_NO_CONTEXT,
                                             context_attributes);
    if (renderer->egl_context == EGL_NO_CONTEXT) {
        fprintf(stderr, "eglCreateContext failed\n");
        return -1;
    }

    renderer->egl_surface = eglCreateWindowSurface(renderer->egl_display,
                                                   config,
                                                   (EGLNativeWindowType)renderer->gbm_surface,
                                                   NULL);
    if (renderer->egl_surface == EGL_NO_SURFACE) {
        fprintf(stderr, "eglCreateWindowSurface failed\n");
        return -1;
    }

    if (!eglMakeCurrent(renderer->egl_display,
                        renderer->egl_surface,
                        renderer->egl_surface,
                        renderer->egl_context)) {
        fprintf(stderr, "eglMakeCurrent failed\n");
        return -1;
    }

    if (renderer_create_program(renderer) != 0) {
        return -1;
    }

    glViewport(0, 0, (GLsizei)renderer->width, (GLsizei)renderer->height);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glClearColor(0.04f, 0.05f, 0.08f, 1.0f);

    if (opts->verbose) {
        printf("DRM mode: %ux%u@%u connector=%u crtc=%u\n",
               renderer->width,
               renderer->height,
               renderer->pipeline.mode.vrefresh,
               renderer->pipeline.connector_id,
               renderer->pipeline.crtc_id);
        printf("GL vendor: %s\n", glGetString(GL_VENDOR));
        printf("GL renderer: %s\n", glGetString(GL_RENDERER));
    }

    return 0;
}

static void renderer_destroy(struct renderer *renderer)
{
    if (renderer->previous_bo && renderer->gbm_surface) {
        gbm_surface_release_buffer(renderer->gbm_surface, renderer->previous_bo);
        renderer->previous_bo = NULL;
    }

    if (renderer->program != 0u) {
        glDeleteProgram(renderer->program);
        renderer->program = 0u;
    }
    if (renderer->vertex_buffer != 0u) {
        glDeleteBuffers(1, &renderer->vertex_buffer);
        renderer->vertex_buffer = 0u;
    }

    if (renderer->egl_display != EGL_NO_DISPLAY) {
        eglMakeCurrent(renderer->egl_display,
                       EGL_NO_SURFACE,
                       EGL_NO_SURFACE,
                       EGL_NO_CONTEXT);
    }
    if (renderer->egl_surface != EGL_NO_SURFACE) {
        eglDestroySurface(renderer->egl_display, renderer->egl_surface);
        renderer->egl_surface = EGL_NO_SURFACE;
    }
    if (renderer->egl_context != EGL_NO_CONTEXT) {
        eglDestroyContext(renderer->egl_display, renderer->egl_context);
        renderer->egl_context = EGL_NO_CONTEXT;
    }
    if (renderer->egl_display != EGL_NO_DISPLAY) {
        eglTerminate(renderer->egl_display);
        renderer->egl_display = EGL_NO_DISPLAY;
    }

    if (renderer->gbm_surface) {
        gbm_surface_destroy(renderer->gbm_surface);
        renderer->gbm_surface = NULL;
    }
    if (renderer->gbm_device) {
        gbm_device_destroy(renderer->gbm_device);
        renderer->gbm_device = NULL;
    }

    if (renderer->pipeline.saved_crtc) {
        drmModeSetCrtc(renderer->drm_fd,
                       renderer->pipeline.saved_crtc->crtc_id,
                       renderer->pipeline.saved_crtc->buffer_id,
                       renderer->pipeline.saved_crtc->x,
                       renderer->pipeline.saved_crtc->y,
                       &renderer->pipeline.connector_id,
                       1,
                       &renderer->pipeline.saved_crtc->mode);
        drmModeFreeCrtc(renderer->pipeline.saved_crtc);
        renderer->pipeline.saved_crtc = NULL;
    }

    if (renderer->drm_fd >= 0) {
        close(renderer->drm_fd);
        renderer->drm_fd = -1;
    }
}

static int stream_surfaces_init(struct stream_surface *streams,
                                unsigned int stream_count,
                                uint32_t width,
                                uint32_t height)
{
    unsigned int index;

    for (index = 0u; index < stream_count; ++index) {
        streams[index].width = width;
        streams[index].height = height;
        streams[index].phase_offset = (float)index * 1.7f;
        streams[index].pixels = calloc((size_t)width * (size_t)height, 4u);
        if (!streams[index].pixels) {
            fprintf(stderr, "unable to allocate stream pixels\n");
            return -1;
        }

        glGenTextures(1, &streams[index].texture_id);
        glBindTexture(GL_TEXTURE_2D, streams[index].texture_id);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexImage2D(GL_TEXTURE_2D,
                     0,
                     GL_RGBA,
                     (GLsizei)width,
                     (GLsizei)height,
                     0,
                     GL_RGBA,
                     GL_UNSIGNED_BYTE,
                     streams[index].pixels);
    }

    glBindTexture(GL_TEXTURE_2D, 0u);
    return 0;
}

static void stream_surfaces_destroy(struct stream_surface *streams, unsigned int stream_count)
{
    unsigned int index;

    if (!streams) {
        return;
    }

    for (index = 0u; index < stream_count; ++index) {
        if (streams[index].texture_id != 0u) {
            glDeleteTextures(1, &streams[index].texture_id);
        }
        free(streams[index].pixels);
    }
}

static void stream_surface_update_pixels(struct stream_surface *stream,
                                         unsigned int stream_index,
                                         double scene_time)
{
    uint32_t x;
    uint32_t y;
    const float phase = (float)scene_time + stream->phase_offset;

    for (y = 0u; y < stream->height; ++y) {
        for (x = 0u; x < stream->width; ++x) {
            const size_t offset = ((size_t)y * stream->width + x) * 4u;
            const float nx = (float)x / (float)(stream->width - 1u);
            const float ny = (float)y / (float)(stream->height - 1u);
            const float wave = 0.5f + 0.5f * sinf((nx * 10.0f) + phase * 2.3f);
            const float scan = 0.5f + 0.5f * sinf((ny * 18.0f) - phase * 3.7f);
            const float pulse = 0.5f + 0.5f * sinf((float)stream_index * 0.9f + phase * 1.4f);
            const bool border = (x < 4u) || (y < 4u) || (x + 5u > stream->width) || (y + 5u > stream->height);
            const bool marker = ((x / 24u) % 2u == 0u) && (y < 12u + 6u * stream_index);
            const uint8_t red = (uint8_t)(255.0f * (0.15f + 0.65f * wave));
            const uint8_t green = (uint8_t)(255.0f * (0.10f + 0.70f * scan));
            const uint8_t blue = (uint8_t)(255.0f * (0.12f + 0.68f * pulse));

            stream->pixels[offset + 0u] = border ? 235u : red;
            stream->pixels[offset + 1u] = marker ? 235u : green;
            stream->pixels[offset + 2u] = border ? 235u : blue;
            stream->pixels[offset + 3u] = 255u;
        }
    }
}

static void stream_surface_upload(const struct stream_surface *stream)
{
    glBindTexture(GL_TEXTURE_2D, stream->texture_id);
    glTexSubImage2D(GL_TEXTURE_2D,
                    0,
                    0,
                    0,
                    (GLsizei)stream->width,
                    (GLsizei)stream->height,
                    GL_RGBA,
                    GL_UNSIGNED_BYTE,
                    stream->pixels);
}

static void build_panel_mvp(float *out,
                            uint32_t output_width,
                            uint32_t output_height,
                            const struct stream_surface *stream,
                            unsigned int stream_index,
                            unsigned int stream_count,
                            double scene_time)
{
    float projection[16];
    float translation[16];
    float rotation_y[16];
    float rotation_x[16];
    float scale[16];
    float tmp[16];
    float model[16];
    const float aspect = (float)output_width / (float)output_height;
    const float stream_aspect = (float)stream->width / (float)stream->height;
    const float centered_index = (float)stream_index - ((float)stream_count - 1.0f) * 0.5f;
    const float x = centered_index * 1.55f;
    const float y = 0.15f * sinf((float)scene_time * 0.9f + centered_index * 0.7f);
    const float z = -4.6f - 0.35f * fabsf(centered_index);
    const float yaw = -0.24f * centered_index + 0.08f * sinf((float)scene_time * 0.7f + centered_index);
    const float tilt = -0.18f + 0.04f * cosf((float)scene_time * 0.5f + centered_index);
    const float panel_height = 1.15f;
    const float panel_width = panel_height * stream_aspect;

    mat4_perspective(projection, 55.0f * (DEMO_PI / 180.0f), aspect, 0.1f, 40.0f);
    mat4_translation(translation, x, y, z);
    mat4_rotation_y(rotation_y, yaw);
    mat4_rotation_x(rotation_x, tilt);
    mat4_scale(scale, panel_width, panel_height, 1.0f);

    mat4_multiply(tmp, translation, rotation_y);
    mat4_multiply(model, tmp, rotation_x);
    mat4_multiply(tmp, model, scale);
    mat4_multiply(out, projection, tmp);
}

static int renderer_present(struct renderer *renderer)
{
    struct gbm_bo *bo;
    struct drm_fb *fb;
    uint32_t connector_id = renderer->pipeline.connector_id;

    if (!eglSwapBuffers(renderer->egl_display, renderer->egl_surface)) {
        fprintf(stderr, "eglSwapBuffers failed\n");
        return -1;
    }

    bo = gbm_surface_lock_front_buffer(renderer->gbm_surface);
    if (!bo) {
        fprintf(stderr, "gbm_surface_lock_front_buffer failed\n");
        return -1;
    }

    fb = drm_fb_get_from_bo(renderer->drm_fd, bo);
    if (!fb) {
        fprintf(stderr, "unable to allocate a framebuffer for the GBM buffer object\n");
        gbm_surface_release_buffer(renderer->gbm_surface, bo);
        return -1;
    }

    if (drmModeSetCrtc(renderer->drm_fd,
                       renderer->pipeline.crtc_id,
                       fb->fb_id,
                       0,
                       0,
                       &connector_id,
                       1,
                       &renderer->pipeline.mode) != 0) {
        perror("drmModeSetCrtc");
        gbm_surface_release_buffer(renderer->gbm_surface, bo);
        return -1;
    }

    if (renderer->previous_bo) {
        gbm_surface_release_buffer(renderer->gbm_surface, renderer->previous_bo);
    }
    renderer->previous_bo = bo;
    return 0;
}

static int renderer_draw_scene(struct renderer *renderer,
                               struct stream_surface *streams,
                               unsigned int stream_count,
                               double scene_time)
{
    unsigned int index;

    glViewport(0, 0, (GLsizei)renderer->width, (GLsizei)renderer->height);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glUseProgram(renderer->program);
    glBindBuffer(GL_ARRAY_BUFFER, renderer->vertex_buffer);
    glEnableVertexAttribArray(renderer->attrib_position);
    glEnableVertexAttribArray(renderer->attrib_texcoord);
    glVertexAttribPointer(renderer->attrib_position,
                          3,
                          GL_FLOAT,
                          GL_FALSE,
                          (GLsizei)sizeof(struct vertex),
                          (const void *)offsetof(struct vertex, position));
    glVertexAttribPointer(renderer->attrib_texcoord,
                          2,
                          GL_FLOAT,
                          GL_FALSE,
                          (GLsizei)sizeof(struct vertex),
                          (const void *)offsetof(struct vertex, uv));

    glUniform1i(renderer->uniform_sampler, 0);
    glActiveTexture(GL_TEXTURE0);

    for (index = 0u; index < stream_count; ++index) {
        float mvp[16];

        stream_surface_update_pixels(&streams[index], index, scene_time);
        stream_surface_upload(&streams[index]);
        build_panel_mvp(mvp,
                        renderer->width,
                        renderer->height,
                        &streams[index],
                        index,
                        stream_count,
                        scene_time);

        glBindTexture(GL_TEXTURE_2D, streams[index].texture_id);
        glUniformMatrix4fv(renderer->uniform_mvp, 1, GL_FALSE, mvp);
        glDrawArrays(GL_TRIANGLES, 0, (GLsizei)ARRAY_SIZE(quad_vertices));
    }

    glBindTexture(GL_TEXTURE_2D, 0u);
    glBindBuffer(GL_ARRAY_BUFFER, 0u);
    glDisableVertexAttribArray(renderer->attrib_position);
    glDisableVertexAttribArray(renderer->attrib_texcoord);
    return renderer_present(renderer);
}

int main(int argc, char **argv)
{
    struct options opts;
    struct renderer renderer;
    struct stream_surface *streams = NULL;
    unsigned int frames_rendered = 0u;
    double start_time;
    int status;

    options_init(&opts);
    status = parse_args(argc, argv, &opts);
    if (status != 0) {
        return status < 0 ? EXIT_FAILURE : EXIT_SUCCESS;
    }

    signal(SIGINT, request_stop);
    signal(SIGTERM, request_stop);

    if (renderer_init(&renderer, &opts) != 0) {
        renderer_destroy(&renderer);
        return EXIT_FAILURE;
    }

    streams = calloc(opts.stream_count, sizeof(*streams));
    if (!streams) {
        fprintf(stderr, "unable to allocate stream metadata\n");
        renderer_destroy(&renderer);
        return EXIT_FAILURE;
    }

    if (stream_surfaces_init(streams,
                             opts.stream_count,
                             opts.stream_width,
                             opts.stream_height) != 0) {
        stream_surfaces_destroy(streams, opts.stream_count);
        free(streams);
        renderer_destroy(&renderer);
        return EXIT_FAILURE;
    }

    start_time = monotonic_seconds();
    while (!stop_requested) {
        const double scene_time = monotonic_seconds() - start_time;

        if (renderer_draw_scene(&renderer, streams, opts.stream_count, scene_time) != 0) {
            status = EXIT_FAILURE;
            goto done;
        }

        frames_rendered += 1u;
        if (opts.max_frames != 0u && frames_rendered >= opts.max_frames) {
            break;
        }
    }

    status = EXIT_SUCCESS;

done:
    stream_surfaces_destroy(streams, opts.stream_count);
    free(streams);
    renderer_destroy(&renderer);
    return status;
}