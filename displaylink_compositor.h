#ifndef DISPLAYLINK_COMPOSITOR_H
#define DISPLAYLINK_COMPOSITOR_H

#include <stdbool.h>
#include <stdint.h>

#include "udl_sink.h"

struct displaylink_compositor_surface {
	uint32_t *pixels;
	uint32_t width;
	uint32_t height;
	uint32_t stride_pixels;
};

bool displaylink_compositor_surface_init(struct displaylink_compositor_surface *surface,
					      uint32_t width,
					      uint32_t height);

void displaylink_compositor_surface_destroy(struct displaylink_compositor_surface *surface);

bool displaylink_compositor_surface_blit_damage(struct displaylink_compositor_surface *dst,
						 const uint32_t *src,
						 uint32_t src_stride_pixels,
						 uint32_t src_width,
						 uint32_t src_height,
						 uint32_t dst_x,
						 uint32_t dst_y,
						 const struct udl_sink_damage *damage,
						 struct udl_sink_damage *out_damage);

#endif