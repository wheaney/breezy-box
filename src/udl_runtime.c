#define _POSIX_C_SOURCE 200809L

#include "common.h"
#include "udl_runtime.h"
/* Include kms_udl_device.h for the full struct options definition.
 * This avoids a circular dependency: kms_udl_runtime.h only forward-declares
 * struct options; the implementation needs the full layout here. */
#include "udl_device.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <time.h>
#include <unistd.h>

static uint64_t udl_now_ns(void)
{
	struct timespec ts;

	if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
		return 0u;
	return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}
#include <pthread.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <limits.h>

int udl_compute_backing_geometry(uint32_t visible_width,
				 uint32_t visible_height,
				 uint32_t *backing_width_out,
				 uint32_t *backing_height_out)
{
	const uint64_t min_pixels = (uint64_t)UDL_MIN_BACKING_16BPP_BYTES / sizeof(uint16_t);
	uint64_t backing_width;
	uint64_t backing_height;
	uint64_t min_height;

	if (!backing_width_out || !backing_height_out || visible_width == 0u || visible_height == 0u)
		return -1;

	backing_width = visible_width;
	if (backing_width < UDL_MIN_BACKING_WIDTH)
		backing_width = UDL_MIN_BACKING_WIDTH;

	backing_height = visible_height;
	min_height = (min_pixels + backing_width - 1u) / backing_width;
	if (backing_height < min_height)
		backing_height = min_height;

	if (backing_width > UINT32_MAX || backing_height > UINT32_MAX)
		return -1;

	*backing_width_out = (uint32_t)backing_width;
	*backing_height_out = (uint32_t)backing_height;
	return 0;
}

uint32_t udl_visible_width(const struct udl_runtime *runtime)
{
	const uint16_t signaled_width = udl_sink_get_hpixels(&runtime->sink);

	if (signaled_width != 0u && signaled_width <= runtime->width)
		return signaled_width;
	return runtime->width;
}

uint32_t udl_visible_height(const struct udl_runtime *runtime)
{
	const uint16_t signaled_height = udl_sink_get_vpixels(&runtime->sink);

	if (signaled_height != 0u && signaled_height <= runtime->height)
		return signaled_height;
	return runtime->height;
}

void udl_clip_damage_to_visible(const struct udl_sink_damage *input,
				uint32_t visible_width,
				uint32_t visible_height,
				struct udl_sink_damage *output)
{
	uint32_t x1;
	uint32_t y1;
	uint32_t x2;
	uint32_t y2;
	uint64_t clipped_pixels;

	if (!output) {
		return;
	}

	udl_sink_clear_damage(output);
	if (!input || !input->touched || visible_width == 0u || visible_height == 0u) {
		return;
	}

	x1 = input->x1;
	y1 = input->y1;
	x2 = input->x2;
	y2 = input->y2;

	if (x1 > visible_width)
		x1 = visible_width;
	if (y1 > visible_height)
		y1 = visible_height;
	if (x2 > visible_width)
		x2 = visible_width;
	if (y2 > visible_height)
		y2 = visible_height;

	if (x1 >= x2 || y1 >= y2) {
		return;
	}

	output->touched = true;
	output->x1 = x1;
	output->y1 = y1;
	output->x2 = x2;
	output->y2 = y2;
	clipped_pixels = (uint64_t)(x2 - x1) * (uint64_t)(y2 - y1);
	output->pixel_count = clipped_pixels > UINT32_MAX ? UINT32_MAX : (uint32_t)clipped_pixels;
}

void decode_queue_clear(struct udl_runtime *runtime)
{
	while (runtime->decode_queue_count > 0u) {
		struct decode_packet *packet = &runtime->decode_queue[runtime->decode_queue_head];

		free(packet->data);
		packet->data = NULL;
		packet->length = 0u;
		runtime->decode_queue_head = (runtime->decode_queue_head + 1u) % DECODE_QUEUE_CAPACITY;
		runtime->decode_queue_count -= 1u;
	}
	runtime->decode_queue_head = 0u;
	runtime->decode_queue_tail = 0u;
}

static void udl_runtime_merge_damage(struct udl_sink_damage *dst,
					     const struct udl_sink_damage *src)
{
	uint64_t pixel_count;

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
	pixel_count = (uint64_t)(dst->x2 - dst->x1) * (uint64_t)(dst->y2 - dst->y1);
	dst->pixel_count = pixel_count > UINT32_MAX ? UINT32_MAX : (uint32_t)pixel_count;
}

void udl_runtime_record_damage(struct udl_runtime *runtime, const struct udl_sink_damage *damage)
{
	if (!runtime || !damage || !damage->touched || !runtime->damage_mutex_initialized)
		return;

	pthread_mutex_lock(&runtime->damage_mutex);
	udl_runtime_merge_damage(&runtime->pending_damage, damage);
	pthread_mutex_unlock(&runtime->damage_mutex);
	atomic_store(&runtime->frame_dirty, true);
}

bool udl_runtime_consume_damage(struct udl_runtime *runtime, struct udl_sink_damage *damage)
{
	if (!runtime || !damage || !runtime->damage_mutex_initialized)
		return false;

	pthread_mutex_lock(&runtime->damage_mutex);
	*damage = runtime->pending_damage;
	udl_sink_clear_damage(&runtime->pending_damage);
	pthread_mutex_unlock(&runtime->damage_mutex);
	return damage->touched;
}

/*
 * Force the throttled full-frame sync to run now, converting the accumulated
 * dirty region.  Called by the decode thread when the queue has drained, so
 * the final decoded state is presented promptly once the stream goes idle.
 * Runs on the decode thread, which owns the transport — no extra locking.
 */
static void udl_runtime_flush_pending(struct udl_runtime *runtime)
{
	struct udl_sink_damage flush_damage;

	if (!runtime->transport.sync_pending)
		return;

	udl_sink_clear_damage(&flush_damage);
	udl_transport_flush_sync(&runtime->transport, &flush_damage);
	if (flush_damage.touched) {
		struct udl_sink_damage flush_visible;

		udl_clip_damage_to_visible(&flush_damage,
					   runtime->last_hpixels != 0u ? runtime->last_hpixels : runtime->width,
					   runtime->last_vpixels != 0u ? runtime->last_vpixels : runtime->height,
					   &flush_visible);
		udl_runtime_record_damage(runtime, &flush_visible);
	}
}

static void *decode_thread_main(void *arg)
{
	struct udl_runtime *runtime = arg;

	for (;;) {
		struct decode_packet packet = {0};
		bool queue_drained;

		pthread_mutex_lock(&runtime->decode_mutex);
		while (runtime->decode_queue_count == 0u &&
		       !stop_requested &&
		       !atomic_load(&runtime->stop_requested)) {
			pthread_cond_wait(&runtime->decode_cond_not_empty, &runtime->decode_mutex);
		}
		if ((stop_requested || atomic_load(&runtime->stop_requested)) && runtime->decode_queue_count == 0u) {
			pthread_mutex_unlock(&runtime->decode_mutex);
			break;
		}

		packet = runtime->decode_queue[runtime->decode_queue_head];
		runtime->decode_queue[runtime->decode_queue_head].data = NULL;
		runtime->decode_queue[runtime->decode_queue_head].length = 0u;
		runtime->decode_queue_head = (runtime->decode_queue_head + 1u) % DECODE_QUEUE_CAPACITY;
		runtime->decode_queue_count -= 1u;
		queue_drained = (runtime->decode_queue_count == 0u);
		pthread_cond_signal(&runtime->decode_cond_not_full);
		pthread_mutex_unlock(&runtime->decode_mutex);

		if (packet.data && packet.length > 0u)
			(void)udl_runtime_feed(runtime, packet.data, packet.length);
		free(packet.data);

		/*
		 * The expensive full-frame sync is throttled inside the feed, so when
		 * the queue has drained (stream paused / decode caught up) force a
		 * flush to present the final frame.  When the decoder is behind, the
		 * queue stays non-empty and this never fires, so it adds no load in
		 * the saturated case.
		 */
		if (queue_drained)
			udl_runtime_flush_pending(runtime);
	}

	return NULL;
}

int udl_runtime_enqueue_bulk_owned(struct udl_runtime *runtime, uint8_t *data, size_t length)
{
	if (!runtime || !runtime->enabled) {
		free(data);
		return 0;
	}
	if (!data || length == 0u) {
		free(data);
		return 0;
	}

	pthread_mutex_lock(&runtime->decode_mutex);
	while (runtime->decode_queue_count == DECODE_QUEUE_CAPACITY &&
	       !stop_requested &&
	       !atomic_load(&runtime->stop_requested)) {
		pthread_cond_wait(&runtime->decode_cond_not_full, &runtime->decode_mutex);
	}
	if (stop_requested || atomic_load(&runtime->stop_requested)) {
		pthread_mutex_unlock(&runtime->decode_mutex);
		free(data);
		return -1;
	}

	runtime->decode_queue[runtime->decode_queue_tail].data = data;
	runtime->decode_queue[runtime->decode_queue_tail].length = length;
	runtime->decode_queue_tail = (runtime->decode_queue_tail + 1u) % DECODE_QUEUE_CAPACITY;
	runtime->decode_queue_count += 1u;
	pthread_cond_signal(&runtime->decode_cond_not_empty);
	pthread_mutex_unlock(&runtime->decode_mutex);
	return 0;
}

int udl_runtime_enqueue_bulk(struct udl_runtime *runtime, const uint8_t *data, size_t length)
{
	uint8_t *copy;

	if (!runtime || !runtime->enabled)
		return 0;
	if (!data || length == 0u)
		return 0;

	copy = malloc(length);
	if (!copy)
		return -1;
	memcpy(copy, data, length);
	return udl_runtime_enqueue_bulk_owned(runtime, copy, length);
}

int udl_runtime_init(struct udl_runtime *runtime, const struct options *opts)
{
	size_t pixel_count;

	memset(runtime, 0, sizeof(*runtime));
	runtime->gadget_fd = -1;
	runtime->enabled = opts->decode_stream;
	runtime->show_window = opts->show_window;
	runtime->verbose = opts->verbose;
	runtime->width = opts->decode_width;
	runtime->height = opts->decode_height;
	runtime->backing_width = opts->decode_width;
	runtime->backing_height = opts->decode_height;
	runtime->window_scale = opts->window_scale;
	runtime->window_title = opts->monitor_name;
	{
		const char *v = getenv("BREEZY_UDL_DECODE_STATS");
		runtime->decode_stats_enabled = v && v[0] != '\0' && strcmp(v, "0") != 0;
	}
	atomic_init(&runtime->stop_requested, false);
	atomic_init(&runtime->frame_dirty, false);

	if (!runtime->enabled)
		return 0;
	if (udl_compute_backing_geometry(runtime->width,
					 runtime->height,
					 &runtime->backing_width,
					 &runtime->backing_height) != 0)
		return -1;
	if ((size_t)runtime->backing_height > SIZE_MAX / (size_t)runtime->backing_width)
		return -1;

	pixel_count = (size_t)runtime->backing_width * (size_t)runtime->backing_height;
	runtime->framebuffer_rgb565 = calloc(pixel_count, sizeof(*runtime->framebuffer_rgb565));
	runtime->framebuffer_xrgb8888 = calloc(pixel_count, sizeof(*runtime->framebuffer_xrgb8888));
	if (!runtime->framebuffer_rgb565 || !runtime->framebuffer_xrgb8888)
		return -1;
	udl_sink_init(&runtime->sink,
		      runtime->framebuffer_rgb565,
		      runtime->backing_width,
		      runtime->backing_height,
		      runtime->backing_width);
	udl_transport_init(&runtime->transport, &runtime->sink);
	udl_transport_set_detailed_stats(&runtime->transport, runtime->verbose);
	udl_transport_set_writerlx16_span_stats(&runtime->transport, runtime->verbose);
	udl_transport_set_writecomp_debug(runtime->verbose);
	udl_sink_attach_xrgb8888_output(&runtime->sink,
				       runtime->framebuffer_xrgb8888,
				       runtime->backing_width);
	udl_sink_clear_damage(&runtime->pending_damage);

	if (pthread_mutex_init(&runtime->damage_mutex, NULL) != 0)
		return -1;
	runtime->damage_mutex_initialized = true;

	if (pthread_mutex_init(&runtime->decode_mutex, NULL) != 0)
		return -1;
	if (pthread_cond_init(&runtime->decode_cond_not_empty, NULL) != 0)
		return -1;
	if (pthread_cond_init(&runtime->decode_cond_not_full, NULL) != 0)
		return -1;
	runtime->decode_sync_initialized = true;
	if (pthread_create(&runtime->decode_thread, NULL, decode_thread_main, runtime) != 0) {
		perror("pthread_create decode thread");
		return -1;
	}
	runtime->decode_thread_created = true;

	return 0;
}

void udl_runtime_destroy(struct udl_runtime *runtime)
{
	if (!runtime)
		return;

	/*
	 * Tell our own threads to stop.  This must be set unconditionally — not
	 * only in the gadget branch below — because the decode thread's exit
	 * condition is (global stop_requested || this flag).  At process shutdown
	 * the global flag is already 1, but when destroy() is called mid-run (e.g.
	 * a hot-reload dimension change reinit), the global flag is still 0, so
	 * without this the broadcast below would wake the decode thread, it would
	 * re-check both flags as false, and pthread_join would block forever.
	 */
	atomic_store(&runtime->stop_requested, true);

	/* Signal gadget reader to stop and close the fd to unblock poll(). */
	if (runtime->gadget_thread_created) {
		if (runtime->gadget_fd >= 0) {
			close(runtime->gadget_fd);
			runtime->gadget_fd = -1;
		}
		(void)pthread_join(runtime->gadget_thread, NULL);
		runtime->gadget_thread_created = false;
	} else if (runtime->gadget_fd >= 0) {
		close(runtime->gadget_fd);
		runtime->gadget_fd = -1;
	}

	if (runtime->decode_thread_created) {
		pthread_mutex_lock(&runtime->decode_mutex);
		pthread_cond_broadcast(&runtime->decode_cond_not_empty);
		pthread_cond_broadcast(&runtime->decode_cond_not_full);
		pthread_mutex_unlock(&runtime->decode_mutex);
		(void)pthread_join(runtime->decode_thread, NULL);
		runtime->decode_thread_created = false;
	}
	if (runtime->decode_sync_initialized) {
		pthread_mutex_lock(&runtime->decode_mutex);
		decode_queue_clear(runtime);
		pthread_mutex_unlock(&runtime->decode_mutex);
		pthread_cond_destroy(&runtime->decode_cond_not_empty);
		pthread_cond_destroy(&runtime->decode_cond_not_full);
		pthread_mutex_destroy(&runtime->decode_mutex);
		runtime->decode_sync_initialized = false;
	}
	if (runtime->damage_mutex_initialized) {
		pthread_mutex_destroy(&runtime->damage_mutex);
		runtime->damage_mutex_initialized = false;
	}

	udl_transport_destroy(&runtime->transport);
	udl_sink_destroy(&runtime->sink);
	free(runtime->framebuffer_rgb565);
	free(runtime->framebuffer_xrgb8888);
	memset(runtime, 0, sizeof(*runtime));
}

static void *gadget_reader_thread_main(void *arg)
{
	struct udl_runtime *runtime = arg;
	uint8_t buf[64 * 1024];

	for (;;) {
		struct pollfd pfd;
		int r;
		ssize_t n;

		if (stop_requested || atomic_load(&runtime->stop_requested))
			break;
		if (runtime->gadget_fd < 0)
			break;

		pfd.fd      = runtime->gadget_fd;
		pfd.events  = POLLIN;
		pfd.revents = 0;
		r = poll(&pfd, 1, 200);
		if (r < 0) {
			if (errno == EINTR)
				continue;
			break;
		}
		if (r == 0)
			continue;

		if (!(pfd.revents & POLLIN))
			break;

		n = read(runtime->gadget_fd, buf, sizeof(buf));
		if (n > 0) {
			(void)udl_runtime_enqueue_bulk(runtime,
						       (const uint8_t *)buf,
						       (size_t)n);
		} else if (n == 0) {
			break;
		} else if (errno != EINTR && errno != EAGAIN) {
			if (errno != EBADF) /* fd closed by destroy */
				perror("udl_gadget read");
			break;
		}
	}

	return NULL;
}

int udl_runtime_start_gadget_reader(struct udl_runtime *runtime,
				    const char *device_path)
{
	if (!runtime || !device_path)
		return -1;
	if (runtime->gadget_thread_created)
		return 0;

	runtime->gadget_fd = open(device_path, O_RDONLY | O_CLOEXEC);
	if (runtime->gadget_fd < 0) {
		fprintf(stderr, "udl_gadget: open(%s): %s\n",
			device_path, strerror(errno));
		return -1;
	}

	if (pthread_create(&runtime->gadget_thread, NULL,
			   gadget_reader_thread_main, runtime) != 0) {
		perror("pthread_create gadget reader");
		close(runtime->gadget_fd);
		runtime->gadget_fd = -1;
		return -1;
	}

	runtime->gadget_thread_created = true;
	fprintf(stderr, "udl_gadget: reader started on %s\n", device_path);
	return 0;
}

int udl_runtime_feed(struct udl_runtime *runtime, const uint8_t *data, size_t length)
{
	struct udl_sink_damage damage;
	struct udl_sink_damage visible_damage;
	struct udl_transport_stats stats;
	enum udl_transport_result result;
	uint64_t decoded_delta = 0u;
	uint64_t decode_errors_delta = 0u;
	uint64_t writereg_delta = 0u;
	uint64_t writerl16_delta = 0u;
	uint64_t writerlx16_delta = 0u;
	uint64_t no_damage_delta = 0u;
	uint16_t hpixels = 0u;
	uint16_t vpixels = 0u;
	uint32_t base16bpp = 0u;
	uint32_t base8bpp = 0u;
	uint8_t color_depth = 0u;
	bool snapshot_changed = false;
	bool had_resync = false;
	bool should_log = false;
	uint32_t visible_width;
	uint32_t visible_height;
	uint64_t feed_t0 = 0u;
	uint64_t decode_t0 = 0u;

	if (!runtime || !runtime->enabled || !data || length == 0u)
		return 0;

	if (runtime->decode_stats_enabled)
		feed_t0 = udl_now_ns();

	udl_sink_clear_damage(&damage);
	if (runtime->decode_stats_enabled)
		decode_t0 = udl_now_ns();
	result = udl_transport_feed(&runtime->transport, data, length, &damage);
	if (runtime->decode_stats_enabled) {
		uint64_t now = udl_now_ns();
		uint64_t sync_ns = runtime->transport.last_feed_sync_ns;
		uint64_t total = now - decode_t0;

		/* Split the transport time into the actual decode vs the sync/convert. */
		runtime->stats_sync_ns += sync_ns;
		runtime->stats_decode_ns += (total > sync_ns) ? (total - sync_ns) : 0u;
		runtime->stats_feed_count += 1u;
		runtime->stats_payload_bytes += length;
		runtime->stats_queue_depth_sum += runtime->decode_queue_count;
		if (sync_ns > 0u) {
			runtime->stats_sync_count += 1u;
			runtime->stats_dirty_rows_sum +=
				(damage.y2 > damage.y1) ? (damage.y2 - damage.y1) : 0u;
		}
	}
	stats = udl_transport_get_stats(&runtime->transport);
	hpixels = udl_sink_get_hpixels(&runtime->sink);
	vpixels = udl_sink_get_vpixels(&runtime->sink);
	base16bpp = udl_sink_get_base16bpp(&runtime->sink);
	base8bpp = udl_sink_get_base8bpp(&runtime->sink);
	color_depth = udl_sink_get_color_depth(&runtime->sink);
	decoded_delta = stats.decoded_commands - runtime->last_stats.decoded_commands;
	decode_errors_delta = stats.decode_errors - runtime->last_stats.decode_errors;
	writereg_delta = stats.writereg_commands - runtime->last_stats.writereg_commands;
	writerl16_delta = stats.writerl16_commands - runtime->last_stats.writerl16_commands;
	writerlx16_delta = stats.writerlx16_commands - runtime->last_stats.writerlx16_commands;
	no_damage_delta = stats.no_damage_commands - runtime->last_stats.no_damage_commands;
	had_resync = runtime->transport.last_feed_first_resync_reason != UDL_TRANSPORT_RESYNC_NONE;
	snapshot_changed = !runtime->have_decoder_snapshot ||
		(runtime->last_hpixels != hpixels) ||
		(runtime->last_vpixels != vpixels) ||
		(runtime->last_color_depth != color_depth) ||
		(runtime->last_base16bpp != base16bpp) ||
		(runtime->last_base8bpp != base8bpp);
	runtime->feed_count += 1u;
	should_log = runtime->verbose && (runtime->feed_count <= 8u ||
				   length >= 4096u ||
				   damage.touched ||
					   snapshot_changed ||
					   had_resync ||
					   decode_errors_delta > 0u);
	runtime->last_stats = stats;
	runtime->last_hpixels = hpixels;
	runtime->last_vpixels = vpixels;
	runtime->last_color_depth = color_depth;
	runtime->last_base16bpp = base16bpp;
	runtime->last_base8bpp = base8bpp;
	runtime->have_decoder_snapshot = true;
	visible_width = (hpixels != 0u && hpixels <= runtime->width) ? hpixels : runtime->width;
	visible_height = (vpixels != 0u && vpixels <= runtime->height) ? vpixels : runtime->height;
	udl_clip_damage_to_visible(&damage,
				   visible_width,
				   visible_height,
				   &visible_damage);
	if (result != UDL_TRANSPORT_OK) {
		fprintf(stderr, "UDL transport error: %s\n", udl_transport_result_string(result));
		return -1;
	}
	if (should_log) {
		fprintf(stderr,
			"UDL decode #%llu len=%zu damage=%s pixels=%u rect=[%u,%u]-[%u,%u] hpix=%u vpix=%u depth=%u base16=0x%08x base8=0x%08x cmds+=%llu errs+=%llu wreg+=%llu wl16+=%llu wrlx16+=%llu nodmg+=%llu\n",
			(unsigned long long)runtime->feed_count,
			length,
			visible_damage.touched ? "yes" : "no",
			visible_damage.pixel_count,
			visible_damage.x1,
			visible_damage.y1,
			visible_damage.x2,
			visible_damage.y2,
			(unsigned int)hpixels,
			(unsigned int)vpixels,
			(unsigned int)color_depth,
			(unsigned int)base16bpp,
			(unsigned int)base8bpp,
			(unsigned long long)decoded_delta,
			(unsigned long long)decode_errors_delta,
			(unsigned long long)writereg_delta,
			(unsigned long long)writerl16_delta,
			(unsigned long long)writerlx16_delta,
			(unsigned long long)no_damage_delta);
	}
	if (runtime->verbose && (decode_errors_delta > 0u || had_resync)) {
		/* log_hex_window is defined in kms_udl_device.c — forward declaration needed */
		extern void log_hex_window(const char *label,
					   const uint8_t *data,
					   size_t length,
					   size_t offset,
					   size_t window_length);
		log_hex_window("Problem bulk head", data, length, 0u, 128u);
		if (had_resync) {
			fprintf(stderr,
				"UDL resync: reason=%s pending_prefix=%zu input_offset=%s%zu\n",
				udl_resync_reason_name(runtime->transport.last_feed_first_resync_reason),
				runtime->transport.last_feed_pending_prefix_len,
				runtime->transport.last_feed_first_resync_offset_valid ? "" : "n/a ",
				runtime->transport.last_feed_first_resync_offset_valid ? runtime->transport.last_feed_first_resync_offset : 0u);
		}
		if (runtime->transport.last_feed_first_resync_offset_valid &&
		    runtime->transport.last_feed_first_resync_offset < length) {
			const size_t resync_offset = runtime->transport.last_feed_first_resync_offset;
			const size_t resync_window_offset = resync_offset > 32u ?
				resync_offset - 32u : 0u;

			log_hex_window("Problem bulk around resync",
				       data,
				       length,
				       resync_window_offset,
				       96u);
		}
	}
	if (runtime->verbose && writereg_delta > 0u) {
		extern void log_udl_writereg_commands(const uint8_t *data, size_t length);
		log_udl_writereg_commands(data, length);
	}
	udl_runtime_record_damage(runtime, &visible_damage);

	if (runtime->decode_stats_enabled) {
		uint64_t now = udl_now_ns();

		runtime->stats_feed_ns += now - feed_t0;
		if (runtime->stats_window_start_ns == 0u)
			runtime->stats_window_start_ns = now;
		else if (now - runtime->stats_window_start_ns >= 1000000000ULL) {
			double secs = (double)(now - runtime->stats_window_start_ns) / 1e9;
			uint64_t fc = runtime->stats_feed_count ? runtime->stats_feed_count : 1u;
			uint64_t sc = runtime->stats_sync_count ? runtime->stats_sync_count : 1u;

			{
				int64_t comp16_raw = 0, comp8_raw = 0, rlx16 = 0, w16 = 0, fill16 = 0, copy16 = 0, other = 0;
				int64_t bridge_pkts = udl_transport_get_and_reset_decode_stats(
					&runtime->transport, &comp16_raw, &comp8_raw, &rlx16, &w16, &fill16, &copy16, &other);
				/* Lower 32 bits = pixel count; upper 32 bits = max tableIndex seen. */
				int64_t comp16      = comp16_raw & 0xffffffffLL;
				int64_t comp8       = comp8_raw  & 0xffffffffLL;
				int64_t c16_maxidx  = (uint32_t)(comp16_raw >> 32);
				int64_t c8_maxidx   = (uint32_t)(comp8_raw  >> 32);
				int64_t total_px = comp16 + comp8 + rlx16 + w16 + fill16 + copy16 + other;

				if (total_px == 0)
					total_px = 1;
				fprintf(stderr,
					"UDL decode stats: %.0f pkt/s  %.1f MB/s  feed=%.1f%% (decode=%.1f%% sync=%.1f%%)  "
					"sync=%.0f/s avg_dirty_rows=%.0f  avg_qdepth=%.1f  decode_us/pkt=%.1f  "
					"bridge_pkts=%" PRId64 "  px%%: comp16=%.0f(maxstate=%" PRId64 ") comp8=%.0f(maxstate=%" PRId64 ") rlx16=%.0f w16=%.0f fill16=%.0f copy16=%.0f other=%.0f\n",
					runtime->stats_feed_count / secs,
					(double)runtime->stats_payload_bytes / secs / 1e6,
					100.0 * (double)runtime->stats_feed_ns / (secs * 1e9),
					100.0 * (double)runtime->stats_decode_ns / (secs * 1e9),
					100.0 * (double)runtime->stats_sync_ns / (secs * 1e9),
					runtime->stats_sync_count / secs,
					(double)runtime->stats_dirty_rows_sum / (double)sc,
					(double)runtime->stats_queue_depth_sum / (double)fc,
					(double)runtime->stats_decode_ns / (double)fc / 1000.0,
					bridge_pkts,
					100.0 * (double)comp16 / (double)total_px, c16_maxidx,
					100.0 * (double)comp8  / (double)total_px, c8_maxidx,
					100.0 * (double)rlx16  / (double)total_px,
					100.0 * (double)w16    / (double)total_px,
					100.0 * (double)fill16 / (double)total_px,
					100.0 * (double)copy16 / (double)total_px,
					100.0 * (double)other  / (double)total_px);
			}

			runtime->stats_window_start_ns = now;
			runtime->stats_feed_count = 0u;
			runtime->stats_feed_ns = 0u;
			runtime->stats_decode_ns = 0u;
			runtime->stats_sync_ns = 0u;
			runtime->stats_sync_count = 0u;
			runtime->stats_payload_bytes = 0u;
			runtime->stats_dirty_rows_sum = 0u;
			runtime->stats_queue_depth_sum = 0u;
		}
	}

	return 0;
}
