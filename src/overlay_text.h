#pragma once

#include <GLES2/gl2.h>
#include <stdbool.h>
#include <stdint.h>

/*
 * overlay_text — CPU-rendered bitmap-font text → GL texture.
 *
 * Text is rendered into a CPU RGBA buffer using a 7×9 monospace bitmap font
 * (printable ASCII only), then uploaded as a GL_RGBA GL_UNSIGNED_BYTE texture.
 * The texture is (re-)created only when the string changes, so it is cheap to
 * call every frame: pass the same string and nothing happens.
 *
 * Usage:
 *   struct overlay_text ot = {0};
 *   overlay_text_update(&ot, "Hello world");   // creates / updates texture
 *   // ot.tex, ot.tex_w, ot.tex_h are ready
 *   overlay_text_destroy(&ot);                 // glDeleteTextures
 */

/* Glyph cell dimensions (pixels in the raster bitmap) */
#define OT_GLYPH_W  7
#define OT_GLYPH_H  9
#define OT_PADDING  4   /* pixels above/below and left/right of the text block */
#define OT_SCALE    3   /* integer upscale factor applied before upload */

#define OT_MAX_TEXT 512

struct overlay_text {
    GLuint   tex;
    uint32_t tex_w;
    uint32_t tex_h;
    char     last_text[OT_MAX_TEXT];
    bool     initialized;
};

/*
 * Create or update the GL texture so it shows `text`.
 * Does nothing if `text` matches the string already rendered.
 * Requires an active GL context.
 */
void overlay_text_update(struct overlay_text *ot, const char *text);

/* Delete the GL texture and reset the struct. */
void overlay_text_destroy(struct overlay_text *ot);
