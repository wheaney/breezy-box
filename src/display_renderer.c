#include "display_renderer.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#define DR_PLACEHOLDER_SIZE 64u
#define DR_PI 3.14159265358979323846f

/* ----------------------------------------------------------------
 * GLSL sources
 * ---------------------------------------------------------------- */

static const char *g_vert_src =
    "attribute vec4 a_pos;\n"
    "attribute vec2 a_uv;\n"
    "uniform mat4 u_mvp;\n"
    "varying vec2 v_uv;\n"
    "void main() {\n"
    "    gl_Position = u_mvp * a_pos;\n"
    "    v_uv = a_uv;\n"
    "}\n";

static const char *g_frag_src =
    "precision mediump float;\n"
    "uniform sampler2D u_tex;\n"
    "varying vec2 v_uv;\n"
    "void main() {\n"
    "    gl_FragColor = vec4(texture2D(u_tex, v_uv).rgb, 1.0);\n"
    "}\n";

/* Alpha-pass variant: preserves the texture alpha channel for GL_BLEND. */
static const char *g_frag_alpha_src =
    "precision mediump float;\n"
    "uniform sampler2D u_tex;\n"
    "varying vec2 v_uv;\n"
    "void main() {\n"
    "    gl_FragColor = texture2D(u_tex, v_uv);\n"
    "}\n";

/* ----------------------------------------------------------------
 * Matrix math
 * ---------------------------------------------------------------- */

void es_load_identity(ESMatrix *r)
{
    memset(r, 0, sizeof(*r));
    r->m[0][0] = r->m[1][1] = r->m[2][2] = r->m[3][3] = 1.0f;
}

/* result = b * a  (second argument is left-hand operand) */
void es_multiply(ESMatrix *result, const ESMatrix *a, const ESMatrix *b)
{
    ESMatrix tmp;
    int i;

    for (i = 0; i < 4; i++) {
        tmp.m[i][0] = a->m[i][0]*b->m[0][0] + a->m[i][1]*b->m[1][0]
                    + a->m[i][2]*b->m[2][0] + a->m[i][3]*b->m[3][0];
        tmp.m[i][1] = a->m[i][0]*b->m[0][1] + a->m[i][1]*b->m[1][1]
                    + a->m[i][2]*b->m[2][1] + a->m[i][3]*b->m[3][1];
        tmp.m[i][2] = a->m[i][0]*b->m[0][2] + a->m[i][1]*b->m[1][2]
                    + a->m[i][2]*b->m[2][2] + a->m[i][3]*b->m[3][2];
        tmp.m[i][3] = a->m[i][0]*b->m[0][3] + a->m[i][1]*b->m[1][3]
                    + a->m[i][2]*b->m[2][3] + a->m[i][3]*b->m[3][3];
    }
    memcpy(result, &tmp, sizeof(tmp));
}

static void es_frustum(ESMatrix *result,
                       float left, float right,
                       float bottom, float top,
                       float nearZ, float farZ)
{
    float dX = right - left;
    float dY = top - bottom;
    float dZ = farZ - nearZ;
    ESMatrix frust;

    if (nearZ <= 0.0f || farZ <= 0.0f || dX <= 0.0f || dY <= 0.0f || dZ <= 0.0f)
        return;

    memset(&frust, 0, sizeof(frust));
    frust.m[0][0] = 2.0f * nearZ / dX;
    frust.m[1][1] = 2.0f * nearZ / dY;
    frust.m[2][0] = (right + left) / dX;
    frust.m[2][1] = (top + bottom) / dY;
    frust.m[2][2] = -(nearZ + farZ) / dZ;
    frust.m[2][3] = -1.0f;
    frust.m[3][2] = -2.0f * nearZ * farZ / dZ;

    es_load_identity(result);
    es_multiply(result, &frust, result);
}

void es_perspective(ESMatrix *result, float fovy, float aspect,
                    float nearZ, float farZ)
{
    float h = tanf(fovy / 360.0f * DR_PI) * nearZ;
    float w = h * aspect;

    es_frustum(result, -w, w, -h, h, nearZ, farZ);
}

/*
 * Device-FOV perspective, mirroring KWin's CameraController.buildPerspectiveMatrix()
 * and GNOME's perspective() exactly.  The frustum is derived from the fixed device
 * vertical FOV half-tangent (heightUnitDistance / 2) and the device aspect ratio —
 * NOT from the display distance.  Moving displays farther/closer changes only their
 * world placement, never the projection, so distance is honored visually.
 *
 *   fov_half_vertical_tangent = diagonalToCrossFOVs(...).heightUnitDistance / 2
 *   device_aspect             = device_width / device_height
 */
void es_perspective_unit(ESMatrix *result,
                         float fov_half_vertical_tangent,
                         float device_aspect,
                         float nearZ, float farZ)
{
    float f  = 1.0f / fov_half_vertical_tangent;
    float nf = 1.0f / (nearZ - farZ);

    memset(result, 0, sizeof(*result));
    /* Column-major m[col][row]; matches the KWin row-major literal transposed. */
    result->m[0][0] = f / device_aspect;
    result->m[1][1] = f;
    result->m[2][2] = (farZ + nearZ) * nf;
    result->m[2][3] = -1.0f;
    result->m[3][2] = (2.0f * farZ * nearZ) * nf;
}

void es_display_model(ESMatrix *m, float angle, float tx, float ty, float tz)
{
    float ca = cosf(angle);
    float sa = sinf(angle);

    memset(m, 0, sizeof(*m));
    m->m[0][0] =  ca;
    m->m[0][2] = -sa;
    m->m[1][1] =  1.0f;
    m->m[2][0] =  sa;
    m->m[2][2] =  ca;
    m->m[3][0] =  tx;
    m->m[3][1] =  ty;
    m->m[3][2] =  tz;
    m->m[3][3] =  1.0f;
}

void es_view_from_quat(ESMatrix *r, float qw, float qx, float qy, float qz)
{
    /* Conjugate: invert rotation so world counter-rotates with head. */
    float cx = -qx, cy = -qy, cz = -qz;

    float xx = cx * cx, yy = cy * cy, zz = cz * cz;
    float xy = cx * cy, xz = cx * cz, yz = cy * cz;
    float wx = qw * cx, wy = qw * cy, wz = qw * cz;

    memset(r, 0, sizeof(*r));
    r->m[0][0] = 1.0f - 2.0f * (yy + zz);
    r->m[0][1] =        2.0f * (xy + wz);
    r->m[0][2] =        2.0f * (xz - wy);
    r->m[1][0] =        2.0f * (xy - wz);
    r->m[1][1] = 1.0f - 2.0f * (xx + zz);
    r->m[1][2] =        2.0f * (yz + wx);
    r->m[2][0] =        2.0f * (xz + wy);
    r->m[2][1] =        2.0f * (yz - wx);
    r->m[2][2] = 1.0f - 2.0f * (xx + yy);
    r->m[3][3] =  1.0f;
}

void es_model_from_quat(ESMatrix *r, float qw, float qx, float qy, float qz)
{
    float xx = qx * qx, yy = qy * qy, zz = qz * qz;
    float xy = qx * qy, xz = qx * qz, yz = qy * qz;
    float wx = qw * qx, wy = qw * qy, wz = qw * qz;

    memset(r, 0, sizeof(*r));
    /* Column-major m[col][row]; the transpose of es_view_from_quat's conjugate. */
    r->m[0][0] = 1.0f - 2.0f * (yy + zz);
    r->m[0][1] =        2.0f * (xy + wz);
    r->m[0][2] =        2.0f * (xz - wy);
    r->m[1][0] =        2.0f * (xy - wz);
    r->m[1][1] = 1.0f - 2.0f * (xx + zz);
    r->m[1][2] =        2.0f * (yz + wx);
    r->m[2][0] =        2.0f * (xz + wy);
    r->m[2][1] =        2.0f * (yz - wx);
    r->m[2][2] = 1.0f - 2.0f * (xx + yy);
    r->m[3][3] = 1.0f;
}

/* ----------------------------------------------------------------
 * Mesh generation — moved to dp_generate_mesh_vertices() in display_placement.c
 * which delegates to generateMeshVertices() in the shared JS bundle.
 * ---------------------------------------------------------------- */
#if 0  /* kept for reference; remove once callers are confirmed clean */
static void display_renderer_build_mesh_ref(
        float radius,
        float default_h_radians, float default_v_radians,
        unsigned dev_w, unsigned dev_h,
        float size_adj_w, float size_adj_h,
        float cnx, float cny, float cnz,
        unsigned mon_w, unsigned mon_h,
        int horizontal_wrap,
        int curved,
        struct display_mesh *out)
{
    /* Mirrors CurvableDisplayMesh.qml: pick flat or curved conversion functions
     * per axis based on both the wrap axis and the curved flag, matching
     *   horizontalConversions = horizontalWrap && curvedDisplay ? curved : flat
     *   verticalConversions   = verticalWrap   && curvedDisplay ? curved : flat
     */
    const float segs_per_rad = 20.0f / (DR_PI * 0.5f);
    int vertical_wrap = !horizontal_wrap;
    int h_curved = horizontal_wrap && curved;
    int v_curved = vertical_wrap   && curved;

    /* Radians per axis — curved uses linear angular scale, flat uses asin. */
    float h_edge = sqrtf(size_adj_w * 0.5f * size_adj_w * 0.5f + radius * radius);
    float h_radians = h_curved
        ? ((dev_w > 0u) ? (default_h_radians / (float)dev_w * (float)mon_w) : 0.0f)
        : ((h_edge > 0.0f) ? 2.0f * asinf(fminf((float)mon_w * 0.5f / h_edge, 1.0f)) : 0.0f);

    float v_edge = sqrtf(size_adj_h * 0.5f * size_adj_h * 0.5f + radius * radius);
    float v_radians = v_curved
        ? ((dev_h > 0u) ? (default_v_radians / (float)dev_h * (float)mon_h) : 0.0f)
        : ((v_edge > 0.0f) ? 2.0f * asinf(fminf((float)mon_h * 0.5f / v_edge, 1.0f)) : 0.0f);

    /* Segment count — curved uses arc-proportional count, flat always gives 1.
     * Mirrors radiansToSegments: curved → ceil(radians * segsPerRadian), flat → 1. */
    int segments = 1;
    if (h_curved)      segments = (int)ceilf(h_radians * segs_per_rad);
    else if (v_curved) segments = (int)ceilf(v_radians * segs_per_rad);
    if (segments < 1) segments = 1;

    int count = 0;
    for (int i = 0; i <= segments; i++) {
        float frac = (float)i / (float)segments;

        float s0, t0, s1, t1;
        if (!vertical_wrap) {
            s0 = s1 = frac;
            t0 = 1.0f;  /* screen bottom → GL y- */
            t1 = 0.0f;  /* screen top    → GL y+ */
        } else {
            s0 = 0.0f;
            s1 = 1.0f;
            t0 = t1 = frac;
        }

        float pairs[2][2] = { {s0, t0}, {s1, t1} };
        for (int p = 0; p < 2; p++) {
            if (count >= DISPLAY_MESH_MAX_VERTS)
                goto done;

            float s = pairs[p][0];
            float t = pairs[p][1];
            float xOff = s - 0.5f;
            float yOff = 0.5f - t;  /* GLES2: t=0 → GL y+, t=1 → GL y- */
            float xOffPx, yOffPx, zOffPx = 0.0f;

            if (h_curved) {
                float xOffRad = xOff * h_radians;
                xOffPx = sinf(xOffRad) * radius;
                zOffPx = radius - cosf(xOffRad) * radius;
            } else {
                xOffPx = (float)mon_w * xOff;
            }

            if (v_curved) {
                float yOffRad = yOff * v_radians;
                yOffPx = sinf(yOffRad) * radius;
                zOffPx = radius - cosf(yOffRad) * radius;
            } else {
                yOffPx = (float)mon_h * yOff;
            }

            out->verts[count].x = cnx + xOffPx;
            out->verts[count].y = cny + yOffPx;
            out->verts[count].z = cnz + zOffPx;
            out->verts[count].s = s;
            out->verts[count].t = t;
            count++;
        }
    }
done:
    out->vert_count = count;
}
#endif  /* reference only */

/* ----------------------------------------------------------------
 * Renderer lifecycle
 * ---------------------------------------------------------------- */

int display_renderer_init(struct display_renderer *r)
{
    GLuint vs, fs;
    GLint status;
    uint8_t placeholder[DR_PLACEHOLDER_SIZE * DR_PLACEHOLDER_SIZE * 4];
    uint32_t px, py;

    memset(r, 0, sizeof(*r));

    vs = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vs, 1, &g_vert_src, NULL);
    glCompileShader(vs);
    glGetShaderiv(vs, GL_COMPILE_STATUS, &status);
    if (!status) {
        char log[512];
        glGetShaderInfoLog(vs, sizeof(log), NULL, log);
        fprintf(stderr, "display_renderer: vertex shader: %s\n", log);
        glDeleteShader(vs);
        return -1;
    }

    fs = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fs, 1, &g_frag_src, NULL);
    glCompileShader(fs);
    glGetShaderiv(fs, GL_COMPILE_STATUS, &status);
    if (!status) {
        char log[512];
        glGetShaderInfoLog(fs, sizeof(log), NULL, log);
        fprintf(stderr, "display_renderer: fragment shader: %s\n", log);
        glDeleteShader(vs);
        glDeleteShader(fs);
        return -1;
    }

    r->prog = glCreateProgram();
    glAttachShader(r->prog, vs);
    glAttachShader(r->prog, fs);
    glLinkProgram(r->prog);
    glDeleteShader(vs);
    glDeleteShader(fs);
    glGetProgramiv(r->prog, GL_LINK_STATUS, &status);
    if (!status) {
        char log[512];
        glGetProgramInfoLog(r->prog, sizeof(log), NULL, log);
        fprintf(stderr, "display_renderer: link: %s\n", log);
        glDeleteProgram(r->prog);
        r->prog = 0;
        return -1;
    }

    r->attr_pos    = glGetAttribLocation(r->prog,  "a_pos");
    r->attr_uv     = glGetAttribLocation(r->prog,  "a_uv");
    r->unif_mvp    = glGetUniformLocation(r->prog, "u_mvp");
    r->unif_tex    = glGetUniformLocation(r->prog, "u_tex");

    /* Alpha-pass program: same vertex shader, alpha-preserving fragment. */
    {
        GLuint avs = glCreateShader(GL_VERTEX_SHADER);
        GLuint afs = glCreateShader(GL_FRAGMENT_SHADER);
        glShaderSource(avs, 1, &g_vert_src, NULL);
        glCompileShader(avs);
        glShaderSource(afs, 1, &g_frag_alpha_src, NULL);
        glCompileShader(afs);
        r->alpha_prog = glCreateProgram();
        glAttachShader(r->alpha_prog, avs);
        glAttachShader(r->alpha_prog, afs);
        glLinkProgram(r->alpha_prog);
        glDeleteShader(avs);
        glDeleteShader(afs);
        glGetProgramiv(r->alpha_prog, GL_LINK_STATUS, &status);
        if (!status) {
            char log[512];
            glGetProgramInfoLog(r->alpha_prog, sizeof(log), NULL, log);
            fprintf(stderr, "display_renderer: alpha program link: %s\n", log);
            glDeleteProgram(r->alpha_prog);
            r->alpha_prog = 0;
            /* Non-fatal: overlay will fall back to opaque draw. */
        } else {
            r->alpha_attr_pos = glGetAttribLocation(r->alpha_prog,  "a_pos");
            r->alpha_attr_uv  = glGetAttribLocation(r->alpha_prog,  "a_uv");
            r->alpha_unif_mvp = glGetUniformLocation(r->alpha_prog, "u_mvp");
            r->alpha_unif_tex = glGetUniformLocation(r->alpha_prog, "u_tex");
        }
    }

    /* Procedural cyan/teal checkerboard placeholder */
    for (py = 0; py < DR_PLACEHOLDER_SIZE; py++) {
        for (px = 0; px < DR_PLACEHOLDER_SIZE; px++) {
            bool checker = ((px / 8u) + (py / 8u)) & 1u;
            uint8_t *p = &placeholder[(py * DR_PLACEHOLDER_SIZE + px) * 4u];
            p[0] = checker ? 0x00u : 0x00u;
            p[1] = checker ? 0xbbu : 0x33u;
            p[2] = checker ? 0xffu : 0x88u;
            p[3] = 0xffu;
        }
    }
    glGenTextures(1, &r->placeholder_tex);
    glBindTexture(GL_TEXTURE_2D, r->placeholder_tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA,
                 (GLsizei)DR_PLACEHOLDER_SIZE, (GLsizei)DR_PLACEHOLDER_SIZE,
                 0, GL_RGBA, GL_UNSIGNED_BYTE, placeholder);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glBindTexture(GL_TEXTURE_2D, 0);

    return 0;
}

void display_renderer_destroy(struct display_renderer *r)
{
    if (!r)
        return;
    if (r->placeholder_tex)
        glDeleteTextures(1, &r->placeholder_tex);
    if (r->alpha_prog)
        glDeleteProgram(r->alpha_prog);
    if (r->prog)
        glDeleteProgram(r->prog);
    memset(r, 0, sizeof(*r));
}

/* ----------------------------------------------------------------
 * Draw
 * ---------------------------------------------------------------- */

void display_renderer_draw_quad(const struct display_renderer *r,
                                const ESMatrix *mvp,
                                GLuint tex,
                                float w_half, float h_half)
{
    GLfloat verts[4 * 5] = {
        -w_half,  h_half, 0.0f,  0.0f, 0.0f,
         w_half,  h_half, 0.0f,  1.0f, 0.0f,
        -w_half, -h_half, 0.0f,  0.0f, 1.0f,
         w_half, -h_half, 0.0f,  1.0f, 1.0f,
    };

    glUseProgram(r->prog);
    glUniformMatrix4fv(r->unif_mvp, 1, GL_FALSE, (const GLfloat *)&mvp->m[0][0]);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, tex);
    glUniform1i(r->unif_tex, 0);

    glEnableVertexAttribArray((GLuint)r->attr_pos);
    glEnableVertexAttribArray((GLuint)r->attr_uv);
    glVertexAttribPointer((GLuint)r->attr_pos, 3, GL_FLOAT, GL_FALSE,
                          5 * (GLsizei)sizeof(GLfloat), verts);
    glVertexAttribPointer((GLuint)r->attr_uv,  2, GL_FLOAT, GL_FALSE,
                          5 * (GLsizei)sizeof(GLfloat), verts + 3);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    glDisableVertexAttribArray((GLuint)r->attr_pos);
    glDisableVertexAttribArray((GLuint)r->attr_uv);

    glBindTexture(GL_TEXTURE_2D, 0);
}

void display_renderer_draw_quad_alpha(const struct display_renderer *r,
                                      const ESMatrix *mvp,
                                      GLuint tex,
                                      float w_half, float h_half)
{
    GLuint prog     = r->alpha_prog  ? r->alpha_prog  : r->prog;
    GLint  apos     = r->alpha_prog  ? r->alpha_attr_pos : r->attr_pos;
    GLint  auv      = r->alpha_prog  ? r->alpha_attr_uv  : r->attr_uv;
    GLint  amvp     = r->alpha_prog  ? r->alpha_unif_mvp : r->unif_mvp;
    GLint  atex     = r->alpha_prog  ? r->alpha_unif_tex : r->unif_tex;

    GLfloat verts[4 * 5] = {
        -w_half,  h_half, 0.0f,  0.0f, 0.0f,
         w_half,  h_half, 0.0f,  1.0f, 0.0f,
        -w_half, -h_half, 0.0f,  0.0f, 1.0f,
         w_half, -h_half, 0.0f,  1.0f, 1.0f,
    };

    glUseProgram(prog);
    glUniformMatrix4fv(amvp, 1, GL_FALSE, (const GLfloat *)&mvp->m[0][0]);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, tex);
    glUniform1i(atex, 0);

    glEnableVertexAttribArray((GLuint)apos);
    glEnableVertexAttribArray((GLuint)auv);
    glVertexAttribPointer((GLuint)apos, 3, GL_FLOAT, GL_FALSE,
                          5 * (GLsizei)sizeof(GLfloat), verts);
    glVertexAttribPointer((GLuint)auv,  2, GL_FLOAT, GL_FALSE,
                          5 * (GLsizei)sizeof(GLfloat), verts + 3);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    glDisableVertexAttribArray((GLuint)apos);
    glDisableVertexAttribArray((GLuint)auv);

    glBindTexture(GL_TEXTURE_2D, 0);
}

void display_renderer_draw_mesh(const struct display_renderer *r,
                                const ESMatrix *mvp,
                                GLuint tex,
                                const float *verts, int vert_count)
{
    if (!r || !mvp || !verts || vert_count < 3)
        return;

    glUseProgram(r->prog);
    glUniformMatrix4fv(r->unif_mvp, 1, GL_FALSE, (const GLfloat *)&mvp->m[0][0]);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, tex);
    glUniform1i(r->unif_tex, 0);

    glEnableVertexAttribArray((GLuint)r->attr_pos);
    glEnableVertexAttribArray((GLuint)r->attr_uv);
    glVertexAttribPointer((GLuint)r->attr_pos, 3, GL_FLOAT, GL_FALSE,
                          5 * (GLsizei)sizeof(GLfloat), verts);
    glVertexAttribPointer((GLuint)r->attr_uv,  2, GL_FLOAT, GL_FALSE,
                          5 * (GLsizei)sizeof(GLfloat), verts + 3);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, vert_count);
    glDisableVertexAttribArray((GLuint)r->attr_pos);
    glDisableVertexAttribArray((GLuint)r->attr_uv);

    glBindTexture(GL_TEXTURE_2D, 0);
}

/* True if `ext` appears as a whole token in the active GL extension string. */
static bool dr_has_gl_ext(const char *ext)
{
    const char *list = (const char *)glGetString(GL_EXTENSIONS);
    size_t len = ext ? strlen(ext) : 0;

    if (!list || !len)
        return false;
    for (const char *p = strstr(list, ext); p; p = strstr(p + len, ext)) {
        if ((p == list || p[-1] == ' ') && (p[len] == ' ' || p[len] == '\0'))
            return true;
    }
    return false;
}

void display_renderer_texture_mipmap_trilinear(GLuint tex)
{
    if (!tex)
        return;

    glBindTexture(GL_TEXTURE_2D, tex);

    /*
     * GLES 2.0 only allows mipmaps on non-power-of-two textures when
     * GL_OES_texture_npot is present; without it glGenerateMipmap on an NPOT
     * texture is an error and sampling returns black.  The overlay/text
     * textures are sized to the string and are essentially always NPOT, so
     * fall back to plain GL_LINEAR when the extension is missing rather than
     * risk a black quad.
     */
    if (dr_has_gl_ext("GL_OES_texture_npot")) {
        /*
         * Trilinear minification (GL_LINEAR_MIPMAP_LINEAR): when a texture is
         * minified or sampled at a non-integer scale — which happens
         * continuously under head jitter in an XR scene — trilinear blends
         * between mip levels instead of crawling across texels, the dominant
         * source of edge shimmer for texture-sampled content (e.g. text).
         * Magnification stays GL_LINEAR; mipmaps only affect minification.
         */
        glGenerateMipmap(GL_TEXTURE_2D);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    } else {
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    }
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);
}
