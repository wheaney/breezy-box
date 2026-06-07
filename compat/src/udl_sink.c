#include "udl_sink.h"
#include "zerokvm_udl.h"

#include <stdlib.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

/*
 * Minimum interval between full-frame RGB565→xrgb8888 syncs.  Each sync scans
 * and converts the accumulated dirty region; throttling it to roughly one
 * display frame keeps the decode thread from pegging on redundant conversions.
 * Tunable via BREEZY_UDL_SYNC_INTERVAL_MS (default 12 ms).
 */
static uint64_t sync_interval_ns(void)
{
	static uint64_t cached;   /* 0 until first computed */

	if (cached == 0u) {
		const char *v = getenv("BREEZY_UDL_SYNC_INTERVAL_MS");
		unsigned long ms = 12u;

		if (v && v[0] != '\0') {
			char *end = NULL;
			unsigned long parsed = strtoul(v, &end, 10);

			if (end && *end == '\0' && parsed > 0u && parsed < 1000u)
				ms = parsed;
		}
		cached = (uint64_t)ms * 1000000ULL;
	}
	return cached;
}

static uint64_t sink_monotonic_ns(void)
{
	struct timespec ts;

	if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
		return 0u;
	return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

static void *udl_sink_get_bridge_handle(const struct udl_sink *sink)
{
	return sink ? (void *)sink->plane8 : NULL;
}

static void udl_sink_set_bridge_handle(struct udl_sink *sink, void *handle)
{
	if (!sink)
		return;
	sink->plane8 = (uint8_t *)handle;
}

static void udl_damage_from_area(const zerokvm_frame_area_t *area,
				      struct udl_sink_damage *damage)
{
	const uint32_t width = area ? (uint32_t)area->modified_x2 - (uint32_t)area->modified_x1 : 0u;
	const uint32_t height = area ? (uint32_t)area->modified_y2 - (uint32_t)area->modified_y1 : 0u;

	udl_sink_clear_damage(damage);
	if (!area || area->modified_x2 <= area->modified_x1 || area->modified_y2 <= area->modified_y1)
		return;

	damage->touched = true;
	damage->x1 = area->modified_x1;
	damage->y1 = area->modified_y1;
	damage->x2 = area->modified_x2;
	damage->y2 = area->modified_y2;
	damage->pixel_count = width * height;
}

static int udl_sink_sync_output(struct udl_sink *sink, struct udl_sink_damage *damage)
{
	zerokvm_frame_area_t area = {0};
	void *handle;

	handle = udl_sink_get_bridge_handle(sink);
	if (!sink || !handle)
		return -1;

	/*
	 * GPU handles RGB565→RGB conversion in the fragment shader, so we only
	 * need the raw RGB565 copy (half the bandwidth of XRGB8888). The xrgb8888
	 * buffer is skipped entirely — nothing reads it for display any more.
	 */
	if (zerokvm_udl_copy_framebuffers(handle,
					 sink->framebuffer,
					 sink->stride_pixels,
					 NULL,
					 0u,
					 &area) != 0)
		return -1;

	if (zerokvm_udl_copy_registers(handle, sink->registers, sizeof(sink->registers)) != 0)
		return -1;

	if (damage)
		udl_damage_from_area(&area, damage);

	return 0;
}

static bool udl_transport_ensure_pending_capacity(struct udl_transport *transport, size_t required)
{
	uint8_t *new_pending;
	size_t new_capacity;

	if (!transport)
		return false;
	if (required <= transport->pending_capacity)
		return true;

	new_capacity = transport->pending_capacity ? transport->pending_capacity : 4096u;
	while (new_capacity < required) {
		if (new_capacity > (SIZE_MAX / 2u)) {
			new_capacity = required;
			break;
		}
		new_capacity *= 2u;
	}

	new_pending = realloc(transport->pending, new_capacity);
	if (!new_pending)
		return false;

	transport->pending = new_pending;
	transport->pending_capacity = new_capacity;
	return true;
}

void udl_sink_init(struct udl_sink *sink,
			   uint16_t *framebuffer,
			   uint32_t width,
			   uint32_t height,
			   uint32_t stride_pixels)
{
	if (!sink)
		return;

	memset(sink, 0, sizeof(*sink));
	sink->framebuffer = framebuffer;
	sink->width = width;
	sink->height = height;
	sink->stride_pixels = stride_pixels;
	udl_sink_set_bridge_handle(sink, zerokvm_udl_create(width, height));
}

void udl_sink_destroy(struct udl_sink *sink)
{
	void *handle;

	if (!sink)
		return;

	handle = udl_sink_get_bridge_handle(sink);
	if (handle)
		zerokvm_udl_destroy(handle);
	memset(sink, 0, sizeof(*sink));
}

void udl_sink_attach_xrgb8888_output(struct udl_sink *sink,
				     uint32_t *framebuffer,
				     uint32_t stride_pixels)
{
	if (!sink)
		return;

	sink->framebuffer_xrgb8888 = framebuffer;
	sink->stride_pixels_xrgb8888 = stride_pixels;
}

void udl_sink_clear_damage(struct udl_sink_damage *damage)
{
	if (!damage)
		return;

	memset(damage, 0, sizeof(*damage));
}

void udl_transport_init(struct udl_transport *transport,
				struct udl_sink *sink)
{
	if (!transport)
		return;

	memset(transport, 0, sizeof(*transport));
	transport->sink = sink;
}

void udl_transport_set_detailed_stats(struct udl_transport *transport,
				      bool enabled)
{
	if (transport)
		transport->collect_detailed_stats = enabled;
}

void udl_transport_set_writerlx16_span_stats(struct udl_transport *transport,
					     bool enabled)
{
	if (transport)
		transport->collect_writerlx16_span_stats = enabled;
}

void udl_transport_set_writecomp_debug(bool enabled)
{
	(void)enabled;
}

void udl_transport_set_writecomp_live_table_decode(bool enabled)
{
	(void)enabled;
}

void udl_transport_set_writecomp_live_table_zerokvm(bool enabled)
{
	(void)enabled;
}

void udl_transport_set_writecomp_live_table_state_carry(bool enabled)
{
	(void)enabled;
}

void udl_transport_set_writecomp_live_table_state_global(bool enabled)
{
	(void)enabled;
}

void udl_transport_set_writecomp_live_table_pixel_carry(bool enabled)
{
	(void)enabled;
}

void udl_transport_set_writecomp_live_table_diff_low16(bool enabled)
{
	(void)enabled;
}

void udl_transport_set_writecomp_live_table_reference_leaf(bool enabled)
{
	(void)enabled;
}

void udl_transport_set_writecomp_live_table_no_change_bit(bool enabled)
{
	(void)enabled;
}

void udl_transport_set_writecomp_live_table_state_seed(bool enabled, uint16_t seed)
{
	(void)enabled;
	(void)seed;
}

void udl_transport_set_writecomp_live_table_state_seed16(bool enabled, uint16_t seed)
{
	(void)enabled;
	(void)seed;
}

void udl_transport_set_writecomp_live_table_state_seed8(bool enabled, uint16_t seed)
{
	(void)enabled;
	(void)seed;
}

void udl_transport_set_writecomp_hybrid_static_live(bool enabled)
{
	(void)enabled;
}

void udl_transport_set_writecomp_trace_target(bool enabled,
					      uint8_t command_type,
					      uint32_t pixel_index,
					      size_t byte_offset,
					      uint8_t bit_offset)
{
	(void)enabled;
	(void)command_type;
	(void)pixel_index;
	(void)byte_offset;
	(void)bit_offset;
}

void udl_transport_set_writecomp_trace_visible_pixel(bool enabled,
					     uint32_t visible_pixel_index)
{
	(void)enabled;
	(void)visible_pixel_index;
}

void udl_transport_set_writecomp_trace_first_failure(bool enabled)
{
	(void)enabled;
}

void udl_transport_reset(struct udl_transport *transport)
{
	void *handle;

	if (!transport || !transport->sink)
		return;

	transport->pending_len = 0u;
	memset(&transport->stats, 0, sizeof(transport->stats));
	transport->last_feed_pending_prefix_len = 0u;
	transport->last_feed_first_resync_offset = 0u;
	transport->last_feed_first_resync_offset_valid = false;
	transport->last_feed_first_resync_reason = UDL_TRANSPORT_RESYNC_NONE;
	transport->writecomp_quarantine_active = false;
	transport->writecomp_quarantine_noncomp_ok = 0u;
	transport->writecomp_quarantine_budget = 0u;

	handle = udl_sink_get_bridge_handle(transport->sink);
	if (handle)
		zerokvm_udl_destroy(handle);
	udl_sink_set_bridge_handle(transport->sink,
				  zerokvm_udl_create(transport->sink->width, transport->sink->height));
}

void udl_transport_destroy(struct udl_transport *transport)
{
	if (!transport)
		return;

	free(transport->pending);
	memset(transport, 0, sizeof(*transport));
}

enum udl_transport_result udl_transport_feed(struct udl_transport *transport,
					     const uint8_t *buffer,
					     size_t length,
					     struct udl_sink_damage *damage)
{
	int processed;
	size_t old_pending_len;
	void *handle;

	handle = udl_sink_get_bridge_handle(transport ? transport->sink : NULL);
	if (!transport || !transport->sink || !handle || (!buffer && length != 0u))
		return UDL_TRANSPORT_ERR_INVALID_ARGUMENT;

	udl_sink_clear_damage(damage);
	transport->last_feed_pending_prefix_len = transport->pending_len;
	transport->last_feed_first_resync_offset = 0u;
	transport->last_feed_first_resync_offset_valid = false;
	transport->last_feed_first_resync_reason = UDL_TRANSPORT_RESYNC_NONE;
	if (length == 0u)
		return UDL_TRANSPORT_OK;

	old_pending_len = transport->pending_len;

	/*
	 * Fast path: no partial command carried over from a previous call, so
	 * feed the caller's buffer directly without copying it into `pending`.
	 * This eliminates a memcpy of every bulk packet in the steady-state case.
	 */
	if (transport->pending_len == 0u) {
		processed = zerokvm_udl_process(handle, buffer, length);
		if (processed < 0) {
			transport->stats.decode_errors += 1u;
			transport->last_feed_first_resync_reason = UDL_TRANSPORT_RESYNC_INVALID_COMMAND;
			return UDL_TRANSPORT_OK;
		}
		if ((size_t)processed < length) {
			/* Partial command at end — stash the remainder for next call. */
			const size_t remaining = length - (size_t)processed;
			if (!udl_transport_ensure_pending_capacity(transport, remaining))
				return UDL_TRANSPORT_ERR_NO_MEMORY;
			memcpy(transport->pending, buffer + processed, remaining);
			transport->pending_len = remaining;
		}
	} else {
		/* Slow path: prepend leftovers from the previous call. */
		if (!udl_transport_ensure_pending_capacity(transport, transport->pending_len + length))
			return UDL_TRANSPORT_ERR_NO_MEMORY;

		memcpy(transport->pending + transport->pending_len, buffer, length);
		transport->pending_len += length;
		processed = zerokvm_udl_process(handle,
						transport->pending,
						transport->pending_len);
		if (processed < 0) {
			transport->stats.decode_errors += 1u;
			transport->pending_len = 0u;
			transport->last_feed_pending_prefix_len = old_pending_len;
			transport->last_feed_first_resync_reason = UDL_TRANSPORT_RESYNC_INVALID_COMMAND;
			return UDL_TRANSPORT_OK;
		}

		if ((size_t)processed < transport->pending_len) {
			const size_t remaining = transport->pending_len - (size_t)processed;
			memmove(transport->pending, transport->pending + processed, remaining);
			transport->pending_len = remaining;
		} else {
			transport->pending_len = 0u;
		}
	}

	if (processed > 0)
		transport->stats.decoded_commands += 1u;

	/*
	 * Throttle the full-frame RGB565→xrgb8888 scan to at most once per
	 * SYNC_INTERVAL_NS.  At ~200 bulk packets/s the unthrottled scan pegged
	 * one core; the render loop calls udl_transport_flush_sync() on every
	 * vsync so the last decoded state is always visible.
	 */
	transport->last_feed_sync_ns = 0u;
	{
		uint64_t now = sink_monotonic_ns();

		if (now == 0u || now - transport->last_sync_ns >= sync_interval_ns()) {
			uint64_t sync_t0 = sink_monotonic_ns();

			if (udl_sink_sync_output(transport->sink, damage) != 0)
				return UDL_TRANSPORT_ERR_INVALID_ARGUMENT;
			transport->last_feed_sync_ns = sink_monotonic_ns() - sync_t0;
			transport->last_sync_ns = now;
			transport->sync_pending = false;
		} else {
			transport->sync_pending = true;
			udl_sink_clear_damage(damage);
		}
	}

	if (processed > 0 && damage && !damage->touched)
		transport->stats.no_damage_commands += 1u;

	return UDL_TRANSPORT_OK;
}

void udl_transport_flush_sync(struct udl_transport *transport,
			      struct udl_sink_damage *damage)
{
	if (!transport || !transport->sync_pending)
		return;

	if (udl_sink_sync_output(transport->sink, damage) == 0) {
		transport->last_sync_ns = sink_monotonic_ns();
		transport->sync_pending = false;
	}
}

struct udl_transport_stats udl_transport_get_stats(const struct udl_transport *transport)
{
	static const struct udl_transport_stats empty_stats = {0};

	if (!transport)
		return empty_stats;

	return transport->stats;
}

int64_t udl_transport_get_and_reset_decode_stats(struct udl_transport *transport,
						  int64_t *out_writecomp16,
						  int64_t *out_writecomp8,
						  int64_t *out_writerlx16,
						  int64_t *out_write16,
						  int64_t *out_fill16,
						  int64_t *out_copy16,
						  int64_t *out_other)
{
	void *handle;

	handle = udl_sink_get_bridge_handle(transport ? transport->sink : NULL);
	if (!handle)
		return 0;
	return zerokvm_udl_get_and_reset_stats(handle,
					       out_writecomp16,
					       out_writecomp8,
					       out_writerlx16,
					       out_write16,
					       out_fill16,
					       out_copy16,
					       out_other);
}

enum udl_sink_result udl_sink_decode_buffer(struct udl_sink *sink,
					    const uint8_t *buffer,
					    size_t length,
					    struct udl_sink_damage *damage)
{
	int processed;
	void *handle;

	handle = udl_sink_get_bridge_handle(sink);
	if (!sink || !handle || (!buffer && length != 0u))
		return UDL_SINK_ERR_INVALID_ARGUMENT;

	udl_sink_clear_damage(damage);
	if (length == 0u)
		return UDL_SINK_OK;

	processed = zerokvm_udl_process(handle, buffer, length);
	if (processed < 0)
		return UDL_SINK_ERR_INVALID_COMMAND;
	if ((size_t)processed < length)
		return UDL_SINK_ERR_TRUNCATED_COMMAND;
	if (udl_sink_sync_output(sink, damage) != 0)
		return UDL_SINK_ERR_INVALID_ARGUMENT;

	return UDL_SINK_OK;
}

uint8_t udl_sink_get_color_depth(const struct udl_sink *sink)
{
	void *handle = udl_sink_get_bridge_handle(sink);
	if (!sink || !handle)
		return 0u;

	return zerokvm_udl_get_color_depth(handle);
}

uint16_t udl_sink_get_hpixels(const struct udl_sink *sink)
{
	void *handle = udl_sink_get_bridge_handle(sink);
	if (!sink || !handle)
		return 0u;

	return zerokvm_udl_get_hpixels(handle);
}

uint16_t udl_sink_get_vpixels(const struct udl_sink *sink)
{
	void *handle = udl_sink_get_bridge_handle(sink);
	if (!sink || !handle)
		return 0u;

	return zerokvm_udl_get_vpixels(handle);
}

uint32_t udl_sink_get_base16bpp(const struct udl_sink *sink)
{
	void *handle = udl_sink_get_bridge_handle(sink);
	if (!sink || !handle)
		return 0u;

	return zerokvm_udl_get_base16bpp(handle);
}

uint32_t udl_sink_get_base8bpp(const struct udl_sink *sink)
{
	void *handle = udl_sink_get_bridge_handle(sink);
	if (!sink || !handle)
		return 0u;

	return zerokvm_udl_get_base8bpp(handle);
}

const char *udl_transport_result_string(enum udl_transport_result result)
{
	switch (result) {
	case UDL_TRANSPORT_OK:
		return "ok";
	case UDL_TRANSPORT_ERR_INVALID_ARGUMENT:
		return "invalid argument";
	case UDL_TRANSPORT_ERR_NO_MEMORY:
		return "out of memory";
	default:
		return "unknown";
	}
}

const char *udl_sink_result_string(enum udl_sink_result result)
{
	switch (result) {
	case UDL_SINK_OK:
		return "ok";
	case UDL_SINK_ERR_INVALID_ARGUMENT:
		return "invalid argument";
	case UDL_SINK_ERR_NO_MEMORY:
		return "out of memory";
	case UDL_SINK_ERR_TRUNCATED_COMMAND:
		return "truncated command";
	case UDL_SINK_ERR_UNSUPPORTED_COMMAND:
		return "unsupported command";
	case UDL_SINK_ERR_INVALID_COMMAND:
		return "invalid command";
	case UDL_SINK_ERR_ADDRESS_RANGE:
		return "address out of range";
	default:
		return "unknown";
	}
}