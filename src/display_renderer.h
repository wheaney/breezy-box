#pragma once

#include <GLES2/gl2.h>

/*
 * Column-major 4×4 matrix: m[col][row].
 * Passed directly to glUniformMatrix4fv with transpose=GL_FALSE.
 */
typedef struct { float m[4][4]; } ESMatrix;

/* ----------------------------------------------------------------
 * Matrix math
 *
 * NOTE: es_multiply(result, a, b) computes  result = b * a
 *       (second argument is the left-hand operand).
 *       Call sites read naturally in right-to-left transform order:
 *           es_multiply(&mvp, &model, &proj_view)  →  mvp = proj_view * model
 * ---------------------------------------------------------------- */

void es_load_identity(ESMatrix *r);
void es_multiply(ESMatrix *result, const ESMatrix *a, const ESMatrix *b);

/* Symmetric perspective: fovy is the full vertical FOV in degrees. */
void es_perspective(ESMatrix *result, float fovy, float aspect,
                    float nearZ, float farZ);

/*
 * Device-FOV perspective matching KWin CameraController.buildPerspectiveMatrix()
 * and GNOME perspective().  Built from the fixed device vertical FOV half-tangent
 * (heightUnitDistance / 2) and device aspect (width/height) — independent of the
 * display distance, so distance only affects world placement, not the projection.
 */
void es_perspective_unit(ESMatrix *result,
                         float fov_half_vertical_tangent,
                         float device_aspect,
                         float nearZ, float farZ);

/* Model matrix: Y-rotation by angle, then translation (tx, ty, tz). */
void es_display_model(ESMatrix *m, float angle, float tx, float ty, float tz);

/*
 * View rotation from unit quaternion (w, x, y, z, EUS frame).
 * Builds the conjugate rotation so world geometry counter-rotates
 * relative to the head pose.
 */
void es_view_from_quat(ESMatrix *r, float qw, float qx, float qy, float qz);

/*
 * Forward model rotation from unit quaternion (w, x, y, z, EUS frame).
 * The counterpart to es_view_from_quat (which builds the conjugate for the
 * camera): this rotates geometry by the quaternion, matching es_display_model's
 * handedness so it can be composed with translations as a model matrix.
 */
void es_model_from_quat(ESMatrix *r, float qw, float qx, float qy, float qz);

/* ----------------------------------------------------------------
 * Curved display mesh — pre-computed once per device config change.
 * Each vertex stores (x, y, z, s, t) tightly packed; ready to pass
 * directly to display_renderer_draw_mesh.
 * ---------------------------------------------------------------- */

#define DISPLAY_MESH_MAX_VERTS 256

typedef struct { float x, y, z, s, t; } display_vertex;

struct display_mesh {
    display_vertex verts[DISPLAY_MESH_MAX_VERTS];
    int            vert_count;
};

/*
 * Build a curved (or flat) surface mesh for one monitor, mirroring
 * KWin's CurvableDisplayMesh.qml generateMesh().
 *
 *   radius              completeScreenDistancePixels
 *   default_h/v_radians defaultDistanceHorizontal/VerticalRadians from fovDetails
 *   dev_w / dev_h       fovDetails.widthPixels / heightPixels
 *   size_adj_w/h        fovDetails.sizeAdjustedWidth/HeightPixels
 */

/* ----------------------------------------------------------------
 * Textured-quad renderer
 * Owns the GLSL program and the procedural placeholder texture.
 * All functions require an active EGL/GL context.
 * ---------------------------------------------------------------- */

struct display_renderer {
    GLuint prog;
    GLint  attr_pos;
    GLint  attr_uv;
    GLint  unif_mvp;
    GLint  unif_tex;
    GLuint placeholder_tex;

    /* Second program: same vertex shader, fragment shader passes alpha
     * through so RGBA textures (e.g. text overlays) composite correctly. */
    GLuint alpha_prog;
    GLint  alpha_attr_pos;
    GLint  alpha_attr_uv;
    GLint  alpha_unif_mvp;
    GLint  alpha_unif_tex;
};

/* Compile shaders and create the placeholder texture.  Returns 0 / -1. */
int  display_renderer_init(struct display_renderer *r);

/* Release all GL objects before destroying the EGL context. */
void display_renderer_destroy(struct display_renderer *r);

/*
 * Draw one textured quad centred at the model-space origin.
 *   mvp    – pre-built MVP matrix
 *   tex    – GL texture to sample (use r->placeholder_tex as fallback)
 *   w_half – half-width along X
 *   h_half – half-height along Y
 */
void display_renderer_draw_quad(const struct display_renderer *r,
                                const ESMatrix *mvp,
                                GLuint tex,
                                float w_half, float h_half);

/*
 * Like display_renderer_draw_quad but uses the alpha-pass shader so that
 * GL_BLEND must be enabled by the caller before this call.  Intended for
 * RGBA text/overlay textures that need transparency.
 */
void display_renderer_draw_quad_alpha(const struct display_renderer *r,
                                      const ESMatrix *mvp,
                                      GLuint tex,
                                      float w_half, float h_half);

/*
 * Draw a pre-computed vertex mesh as a GL_TRIANGLE_STRIP.
 *   mvp        – pre-built MVP matrix
 *   tex        – GL texture
 *   verts      – tightly-packed floats: x,y,z,s,t per vertex
 *   vert_count – total number of vertices
 */
void display_renderer_draw_mesh(const struct display_renderer *r,
                                const ESMatrix *mvp,
                                GLuint tex,
                                const float *verts, int vert_count);

/*
 * Generate a mipmap chain for the given 2D texture and set trilinear
 * minification (GL_LINEAR_MIPMAP_LINEAR) with GL_LINEAR magnification and
 * clamped wrapping.  Call after the texture's level-0 image has been uploaded.
 * Reduces edge shimmer when the texture is minified or sampled at a
 * non-integer scale (e.g. text under continuous head motion).  Renderer-
 * agnostic: requires only an active GL context.
 */
void display_renderer_texture_mipmap_trilinear(GLuint tex);
