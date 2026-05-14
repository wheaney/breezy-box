#include "displaylink_compositor.h"

#include <stdlib.h>
#include <string.h>

static void displaylink_compositor_merge_damage(struct udl_sink_damage *dst,
						const struct udl_sink_damage *src)
{
	if (!dst || !src || !src->touched)
		return;

	if (!dst->touched) {
		*dst = *src;
		return;
	}

	if (src->x1 < dst->x1)
		dst->x1 = src->x1;
	if (src->y1 < dst->y1)
		dst->y1 = src->y1;
	if (src->x2 > dst->x2)
		dst->x2 = src->x2;
	if (src->y2 > dst->y2)
		dst->y2 = src->y2;
	dst->pixel_count += src->pixel_count;
	dst->touched = true;
}

bool displaylink_compositor_surface_init(struct displaylink_compositor_surface *surface,
					      uint32_t width,
					      uint32_t height)
{
	size_t pixel_count;

	if (!surface || width == 0u || height == 0u)
		return false;
	if ((size_t)height > SIZE_MAX / (size_t)width)
		return false;

	pixel_count = (size_t)width * (size_t)height;
	memset(surface, 0, sizeof(*surface));
	surface->pixels = calloc(pixel_count, sizeof(*surface->pixels));
	if (!surface->pixels)
		return false;

	surface->width = width;
	surface->height = height;
	surface->stride_pixels = width;
	return true;
}

void displaylink_compositor_surface_destroy(struct displaylink_compositor_surface *surface)
{
	if (!surface)
		return;

	free(surface->pixels);
	memset(surface, 0, sizeof(*surface));
}

bool displaylink_compositor_surface_blit_damage(struct displaylink_compositor_surface *dst,
						 const uint32_t *src,
						 uint32_t src_stride_pixels,
						 uint32_t src_width,
						 uint32_t src_height,
						 uint32_t dst_x,
						 uint32_t dst_y,
						 const struct udl_sink_damage *damage,
						 struct udl_sink_damage *out_damage)
{
	uint32_t src_x1;
	uint32_t src_y1;
	uint32_t src_x2;
	uint32_t src_y2;
	uint32_t copy_width;
	uint32_t copy_height;
	uint32_t row;
	struct udl_sink_damage blit_damage;

	if (!dst || !dst->pixels || !src || src_stride_pixels == 0u)
		return false;
	if (!damage || !damage->touched)
		return true;
	if (src_width == 0u || src_height == 0u)
		return true;
	if (dst_x >= dst->width || dst_y >= dst->height)
		return true;

	src_x1 = damage->x1 < src_width ? damage->x1 : src_width - 1u;
	src_y1 = damage->y1 < src_height ? damage->y1 : src_height - 1u;
	src_x2 = damage->x2 < src_width ? damage->x2 : src_width - 1u;
	src_y2 = damage->y2 < src_height ? damage->y2 : src_height - 1u;
	if (src_x2 < src_x1 || src_y2 < src_y1)
		return true;

	copy_width = src_x2 - src_x1 + 1u;
	copy_height = src_y2 - src_y1 + 1u;
	if (copy_width > dst->width - dst_x - src_x1)
		copy_width = dst->width - dst_x - src_x1;
	if (copy_height > dst->height - dst_y - src_y1)
		copy_height = dst->height - dst_y - src_y1;
	if (copy_width == 0u || copy_height == 0u)
		return true;

	for (row = 0u; row < copy_height; ++row) {
		memcpy(dst->pixels + ((size_t)(dst_y + src_y1 + row) * dst->stride_pixels) + dst_x + src_x1,
		       src + ((size_t)(src_y1 + row) * src_stride_pixels) + src_x1,
		       (size_t)copy_width * sizeof(*src));
	}

	udl_sink_clear_damage(&blit_damage);
	blit_damage.touched = true;
	blit_damage.x1 = dst_x + src_x1;
	blit_damage.y1 = dst_y + src_y1;
	blit_damage.x2 = blit_damage.x1 + copy_width - 1u;
	blit_damage.y2 = blit_damage.y1 + copy_height - 1u;
	blit_damage.pixel_count = copy_width * copy_height;
	displaylink_compositor_merge_damage(out_damage, &blit_damage);

	return true;
}