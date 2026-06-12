#pragma once

#include "udl_sink.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdatomic.h>
#include <pthread.h>

#define DECODE_QUEUE_CAPACITY 128u

struct decode_packet {
	uint8_t *data;
	size_t length;
};

struct udl_runtime {
	struct udl_sink sink;
	struct udl_transport transport;
	uint16_t *framebuffer_rgb565;
	uint32_t *framebuffer_xrgb8888;
	struct udl_sink_damage pending_damage;
	struct udl_transport_stats last_stats;
	uint32_t width;
	uint32_t height;
	uint32_t backing_width;
	uint32_t backing_height;
	uint32_t last_base16bpp;
	uint32_t last_base8bpp;
	pthread_t viewer_thread;
	pthread_t decode_thread;
	pthread_t gadget_thread;
	struct decode_packet decode_queue[DECODE_QUEUE_CAPACITY];
	uint32_t decode_queue_head;
	uint32_t decode_queue_tail;
	uint32_t decode_queue_count;
	pthread_mutex_t damage_mutex;
	pthread_mutex_t decode_mutex;
	pthread_cond_t decode_cond_not_empty;
	pthread_cond_t decode_cond_not_full;
	uint64_t feed_count;
	atomic_bool stop_requested;
	atomic_bool frame_dirty;
	bool enabled;
	bool show_window;
	bool verbose;
	bool have_decoder_snapshot;
	bool viewer_thread_created;
	bool decode_thread_created;
	bool gadget_thread_created;
	bool damage_mutex_initialized;
	bool decode_sync_initialized;
	uint8_t last_color_depth;
	uint32_t window_scale;
	uint16_t last_hpixels;
	uint16_t last_vpixels;
	const char *window_title;
	int gadget_fd;   /* fd for /dev/udl_gadget; -1 when not in use */

	/* Decode-thread profiling (enabled by BREEZY_UDL_DECODE_STATS=1). */
	bool decode_stats_enabled;
	uint64_t stats_window_start_ns;
	uint64_t stats_feed_count;        /* packets fed in this window */
	uint64_t stats_feed_ns;           /* total ns inside udl_runtime_feed */
	uint64_t stats_decode_ns;         /* ns inside udl_transport_feed (decode) */
	uint64_t stats_sync_ns;           /* ns inside the framebuffer sync/convert */
	uint64_t stats_sync_count;        /* number of full-frame syncs in window */
	uint64_t stats_payload_bytes;     /* bulk bytes decoded in window */
	uint64_t stats_dirty_rows_sum;    /* sum of dirty-row counts per sync */
	uint64_t stats_queue_depth_sum;   /* sum of queue depth sampled per packet */
};

/* Forward declaration — full definition in kms_udl_device.h */
struct options;

/* Function declarations */
int udl_compute_backing_geometry(uint32_t visible_width,
				 uint32_t visible_height,
				 uint32_t *backing_width_out,
				 uint32_t *backing_height_out);

uint32_t udl_visible_width(const struct udl_runtime *runtime);
uint32_t udl_visible_height(const struct udl_runtime *runtime);
void udl_clip_damage_to_visible(const struct udl_sink_damage *input,
				uint32_t visible_width,
				uint32_t visible_height,
				struct udl_sink_damage *output);

void decode_queue_clear(struct udl_runtime *runtime);

int udl_runtime_init(struct udl_runtime *runtime, const struct options *opts);
void udl_runtime_destroy(struct udl_runtime *runtime);
int udl_runtime_enqueue_bulk(struct udl_runtime *runtime, const uint8_t *data, size_t length);
int udl_runtime_enqueue_bulk_owned(struct udl_runtime *runtime, uint8_t *data, size_t length);
void udl_runtime_record_damage(struct udl_runtime *runtime, const struct udl_sink_damage *damage);
bool udl_runtime_consume_damage(struct udl_runtime *runtime, struct udl_sink_damage *damage);
int udl_runtime_feed(struct udl_runtime *runtime, const uint8_t *data, size_t length);

/*
 * Start a background thread reading raw USB bulk data from a gadget char
 * device (e.g. /dev/udl_gadget) and feeding it into the decode queue.
 * Returns 0 on success.  The thread is stopped by udl_runtime_destroy().
 */
int udl_runtime_start_gadget_reader(struct udl_runtime *runtime,
				    const char *device_path);
