#define _POSIX_C_SOURCE 200809L

#include <dirent.h>
#include <errno.h>
#include <getopt.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include <SDL2/SDL.h>

#include "displaylink_compositor.h"
#include "displaylink_session.h"

#define DEFAULT_RING_SURFACES 2u
#define DEFAULT_POLL_INTERVAL_MS 16u
#define DEFAULT_WINDOW_SCALE 1u
#define MAX_SESSION_COUNT 8u

struct demo_options {
	uint32_t canvas_width;
	uint32_t canvas_height;
	uint32_t window_scale;
	uint32_t poll_interval_ms;
	bool show_window;
	const char *dump_image_path;
};

struct demo_ring_storage {
	struct displaylink_output_surface surfaces[DEFAULT_RING_SURFACES];
	uint32_t *pixels[DEFAULT_RING_SURFACES];
};

struct demo_session_slot {
	struct displaylink_session_options options;
	struct demo_ring_storage ring;
	struct displaylink_session *session;
	uint32_t layout_x;
	uint32_t layout_y;
	uint32_t width;
	uint32_t height;
	uint64_t last_sequence;
	bool started;
};

static volatile sig_atomic_t demo_stop_requested = 0;

static void demo_on_signal(int signo)
{
	(void)signo;
	demo_stop_requested = 1;
}

static void demo_install_signal_handlers(void)
{
	struct sigaction action;

	memset(&action, 0, sizeof(action));
	action.sa_handler = demo_on_signal;
	sigemptyset(&action.sa_mask);
	sigaction(SIGINT, &action, NULL);
	sigaction(SIGTERM, &action, NULL);
	sigaction(SIGHUP, &action, NULL);
	sigaction(SIGQUIT, &action, NULL);
}

static void demo_usage(const char *argv0)
{
	fprintf(stderr,
		"Usage: %s [demo-options] --session [raw-gadget session args] [--session ...]\n"
		"\n"
		"Demo options:\n"
		"  --canvas-width PIXELS   Override composite canvas width\n"
		"  --canvas-height PIXELS  Override composite canvas height\n"
		"  --window-scale FACTOR   Integer scale factor for the SDL viewer (default: 1)\n"
		"  --poll-interval-ms MS   Main-loop poll interval in milliseconds (default: 16)\n"
		"  --show-window           Show one SDL window for the composed output\n"
		"  --dump-image PATH       Write the final composite as a binary PPM on exit\n"
		"  --help                  Show this message\n"
		"\n"
		"Session blocks:\n"
		"  Start each in-process DisplayLink session with --session and then pass the\n"
		"  same options accepted by displaylink_gadget_raw_gadget, for example:\n"
		"\n"
		"  sudo %s --show-window --session --udc-device fe800000.usb --udc-driver fe800000.usb --monitor-name Left \\\n"
		"       --session --udc-device fe900000.usb --udc-driver fe900000.usb --monitor-name Right\n",
		argv0,
		argv0);
}

static void demo_default_options(struct demo_options *opts)
{
	memset(opts, 0, sizeof(*opts));
	opts->window_scale = DEFAULT_WINDOW_SCALE;
	opts->poll_interval_ms = DEFAULT_POLL_INTERVAL_MS;
	opts->show_window = false;
}

static int demo_parse_u32(const char *text, uint32_t *value)
{
	char *end = NULL;
	unsigned long parsed;

	errno = 0;
	parsed = strtoul(text, &end, 0);
	if (errno != 0 || !end || *end != '\0' || parsed > 0xffffffffUL)
		return -1;

	*value = (uint32_t)parsed;
	return 0;
}

static int demo_parse_args(int argc,
			   char **argv,
			   struct demo_options *opts,
			   size_t *session_start_index)
{
	static const struct option long_options[] = {
		{ "canvas-width", required_argument, NULL, 'W' },
		{ "canvas-height", required_argument, NULL, 'H' },
		{ "window-scale", required_argument, NULL, 'S' },
		{ "poll-interval-ms", required_argument, NULL, 'P' },
		{ "show-window", no_argument, NULL, 's' },
		{ "dump-image", required_argument, NULL, 'o' },
		{ "session", no_argument, NULL, 1000 },
		{ "help", no_argument, NULL, 'h' },
		{ NULL, 0, NULL, 0 },
	};
	int opt;

	if (!opts || !session_start_index)
		return -1;

	optind = 1;
	while ((opt = getopt_long(argc, argv, "+W:H:S:P:so:h", long_options, NULL)) != -1) {
		switch (opt) {
		case 'W':
			if (demo_parse_u32(optarg, &opts->canvas_width) != 0)
				return -1;
			break;
		case 'H':
			if (demo_parse_u32(optarg, &opts->canvas_height) != 0)
				return -1;
			break;
		case 'S':
			if (demo_parse_u32(optarg, &opts->window_scale) != 0)
				return -1;
			break;
		case 'P':
			if (demo_parse_u32(optarg, &opts->poll_interval_ms) != 0)
				return -1;
			break;
		case 's':
			opts->show_window = true;
			break;
		case 'o':
			opts->dump_image_path = optarg;
			break;
		case 1000:
			*session_start_index = (size_t)(optind - 1);
			return 0;
		case 'h':
			demo_usage(argv[0]);
			exit(0);
		default:
			return -1;
		}
	}

	*session_start_index = (size_t)optind;
	return 0;
}

static int demo_write_ppm(const char *path,
			 const struct displaylink_compositor_surface *surface,
			 uint32_t visible_width,
			 uint32_t visible_height)
{
	FILE *file;
	uint8_t *row_buffer;
	uint32_t row;
	uint32_t column;

	if (!path || !surface || !surface->pixels)
		return 0;
	if (visible_width == 0u || visible_height == 0u)
		return 0;
	if (visible_width > surface->width || visible_height > surface->height)
		return -1;

	file = fopen(path, "wb");
	if (!file) {
		perror(path);
		return -1;
	}

	if (fprintf(file, "P6\n%u %u\n255\n", visible_width, visible_height) < 0) {
		perror(path);
		fclose(file);
		return -1;
	}

	row_buffer = malloc((size_t)visible_width * 3u);
	if (!row_buffer) {
		fclose(file);
		return -1;
	}

	for (row = 0u; row < visible_height; ++row) {
		const uint32_t *src = surface->pixels + ((size_t)row * surface->stride_pixels);

		for (column = 0u; column < visible_width; ++column) {
			const uint32_t pixel = src[column];
			row_buffer[(size_t)column * 3u + 0u] = (uint8_t)((pixel >> 16) & 0xffu);
			row_buffer[(size_t)column * 3u + 1u] = (uint8_t)((pixel >> 8) & 0xffu);
			row_buffer[(size_t)column * 3u + 2u] = (uint8_t)(pixel & 0xffu);
		}

		if (fwrite(row_buffer, 1u, (size_t)visible_width * 3u, file) != (size_t)visible_width * 3u) {
			perror(path);
			free(row_buffer);
			fclose(file);
			return -1;
		}
	}

	free(row_buffer);
	if (fclose(file) != 0) {
		perror(path);
		return -1;
	}

	return 0;
}

static int demo_allocate_ring(struct demo_session_slot *slot)
{
	uint32_t index;
	size_t pixel_count;

	if (!slot)
		return -1;
	if (slot->options.decode_width == 0u || slot->options.decode_height == 0u)
		return -1;
	if ((size_t)slot->options.decode_height > SIZE_MAX / (size_t)slot->options.decode_width)
		return -1;

	pixel_count = (size_t)slot->options.decode_width * (size_t)slot->options.decode_height;
	for (index = 0u; index < DEFAULT_RING_SURFACES; ++index) {
		slot->ring.pixels[index] = calloc(pixel_count, sizeof(*slot->ring.pixels[index]));
		if (!slot->ring.pixels[index])
			return -1;
		slot->ring.surfaces[index].pixels = slot->ring.pixels[index];
		slot->ring.surfaces[index].width = slot->options.decode_width;
		slot->ring.surfaces[index].height = slot->options.decode_height;
		slot->ring.surfaces[index].stride_pixels = slot->options.decode_width;
	}

	slot->options.output_ring.surfaces = slot->ring.surfaces;
	slot->options.output_ring.surface_count = DEFAULT_RING_SURFACES;
	slot->options.output_surface.pixels = NULL;
	slot->options.output_surface.width = 0u;
	slot->options.output_surface.height = 0u;
	slot->options.output_surface.stride_pixels = 0u;
	slot->options.show_window = false;
	return 0;
}

static void demo_free_ring(struct demo_session_slot *slot)
{
	uint32_t index;

	if (!slot)
		return;

	for (index = 0u; index < DEFAULT_RING_SURFACES; ++index) {
		free(slot->ring.pixels[index]);
		slot->ring.pixels[index] = NULL;
		slot->ring.surfaces[index].pixels = NULL;
	}
	slot->options.output_ring.surfaces = NULL;
	slot->options.output_ring.surface_count = 0u;
}

static int demo_parse_session_block(int argc,
				    char **argv,
				    struct demo_session_slot *slot)
{
	char **session_argv;
	int index;
	int rc = -1;

	if (!slot)
		return -1;
	if (argc <= 0 || !argv)
		return -1;

	displaylink_session_options_init(&slot->options);
	session_argv = calloc((size_t)argc + 1u, sizeof(*session_argv));
	if (!session_argv)
		return -1;
	session_argv[0] = (char *)"displaylink_session";
	for (index = 0; index < argc; ++index)
		session_argv[index + 1] = argv[index];
	if (displaylink_session_parse_args(argc + 1, session_argv, &slot->options) != 0)
		goto out;
	if (demo_allocate_ring(slot) != 0)
		goto out;
	if (displaylink_session_validate_options(&slot->options) != 0) {
		demo_free_ring(slot);
		goto out;
	}

	slot->width = slot->options.decode_width;
	slot->height = slot->options.decode_height;
	rc = 0;

out:
	free(session_argv);
	return rc;
}

static int demo_parse_sessions(int argc,
			       char **argv,
			       size_t start_index,
			       struct demo_session_slot *slots,
			       size_t *slot_count)
{
	size_t session_index = 0u;
	size_t block_start;
	size_t index;

	if (!slots || !slot_count || start_index >= (size_t)argc || strcmp(argv[start_index], "--session") != 0)
		return -1;

	block_start = start_index + 1u;
	for (index = start_index + 1u; index <= (size_t)argc; ++index) {
		const bool at_end = index == (size_t)argc;
		const bool next_block = !at_end && strcmp(argv[index], "--session") == 0;

		if (!at_end && !next_block)
			continue;
		if (block_start == index)
			return -1;
		if (session_index == MAX_SESSION_COUNT)
			return -1;
		if (demo_parse_session_block((int)(index - block_start),
					     &argv[block_start],
					     &slots[session_index]) != 0)
			return -1;
		session_index += 1u;
		block_start = index + 1u;
	}

	*slot_count = session_index;
	return session_index == 0u ? -1 : 0;
}

static void demo_layout_slots(struct demo_session_slot *slots,
			      size_t slot_count,
			      uint32_t *canvas_width,
			      uint32_t *canvas_height)
{
	uint32_t total_width = 0u;
	uint32_t max_height = 0u;
	size_t index;

	for (index = 0u; index < slot_count; ++index) {
		slots[index].layout_x = total_width;
		slots[index].layout_y = 0u;
		total_width += slots[index].width;
		if (slots[index].height > max_height)
			max_height = slots[index].height;
	}

	if (canvas_width && *canvas_width == 0u)
		*canvas_width = total_width;
	if (canvas_height && *canvas_height == 0u)
		*canvas_height = max_height;
}

static const char *demo_session_label(const struct demo_session_slot *slot,
				      size_t index)
{
	if (slot && slot->options.monitor_name && slot->options.monitor_name[0] != '\0')
		return slot->options.monitor_name;
	if (slot && slot->options.serial_string && slot->options.serial_string[0] != '\0')
		return slot->options.serial_string;
	(void)index;
	return "session";
}

static int demo_count_available_udcs(void)
{
	DIR *directory;
	struct dirent *entry;
	int count = 0;

	directory = opendir("/sys/class/udc");
	if (!directory)
		return -1;

	while ((entry = readdir(directory)) != NULL) {
		if (entry->d_name[0] == '.')
			continue;
		count += 1;
	}

	closedir(directory);
	return count;
}

static bool demo_udc_exists(const char *udc_device)
{
	char path[512];
	struct stat st;

	if (!udc_device || udc_device[0] == '\0')
		return false;
	if (snprintf(path, sizeof(path), "/sys/class/udc/%s", udc_device) >= (int)sizeof(path))
		return false;
	return stat(path, &st) == 0;
}

static int demo_validate_udc_requests(const struct demo_session_slot *slots, size_t slot_count)
{
	int available_udcs;
	size_t index;
	size_t other_index;

	if (!slots || slot_count == 0u)
		return -1;

	available_udcs = demo_count_available_udcs();
	if (available_udcs == 0) {
		fprintf(stderr,
			"No gadget-capable UDCs are exposed under /sys/class/udc, so the demo cannot start any DisplayLink sessions.\n");
		return -1;
	}
	if (slot_count > 1u && available_udcs > 0 && available_udcs < (int)slot_count) {
		fprintf(stderr,
			"The demo was asked to start %zu sessions, but only %d UDC instance(s) are currently exposed under /sys/class/udc. This board/image cannot run that many raw-gadget sessions at once.\n",
			slot_count,
			available_udcs);
	}

	for (index = 0u; index < slot_count; ++index) {
		if (!slots[index].options.udc_device || slots[index].options.udc_device[0] == '\0') {
			if (slot_count > 1u) {
				fprintf(stderr,
					"%s is missing --udc-device. Multi-session runs should name each UDC explicitly so the demo can verify they are distinct and present.\n",
					demo_session_label(&slots[index], index));
				return -1;
			}
			continue;
		}
		if (!demo_udc_exists(slots[index].options.udc_device)) {
			fprintf(stderr,
				"%s requested UDC '%s', but /sys/class/udc/%s does not exist on this system.\n",
				demo_session_label(&slots[index], index),
				slots[index].options.udc_device,
				slots[index].options.udc_device);
			return -1;
		}
		for (other_index = index + 1u; other_index < slot_count; ++other_index) {
			if (slots[other_index].options.udc_device &&
			    strcmp(slots[index].options.udc_device,
				   slots[other_index].options.udc_device) == 0) {
				fprintf(stderr,
					"%s and %s both requested UDC '%s'. Each raw-gadget session needs its own distinct UDC.\n",
					demo_session_label(&slots[index], index),
					demo_session_label(&slots[other_index], other_index),
					slots[index].options.udc_device);
				return -1;
			}
		}
	}

	return 0;
}

static bool demo_frame_to_damage(const struct displaylink_output_frame *frame,
				struct udl_sink_damage *damage)
{
	if (!frame || !damage)
		return false;

	udl_sink_clear_damage(damage);
	if (!frame->damage.touched)
		return false;

	damage->touched = true;
	damage->x1 = frame->damage.x1;
	damage->y1 = frame->damage.y1;
	damage->x2 = frame->damage.x2;
	damage->y2 = frame->damage.y2;
	damage->pixel_count = frame->damage.pixel_count;
	return true;
}

static int demo_start_sessions(struct demo_session_slot *slots, size_t slot_count)
{
	size_t index;

	for (index = 0u; index < slot_count; ++index) {
		slots[index].session = displaylink_session_create(&slots[index].options);
		if (!slots[index].session)
			return -1;
		if (displaylink_session_start(slots[index].session) != 0)
			return -1;
		slots[index].started = true;
	}

	return 0;
}

static void demo_stop_sessions(struct demo_session_slot *slots, size_t slot_count)
{
	size_t index;

	for (index = 0u; index < slot_count; ++index) {
		if (slots[index].session)
			displaylink_session_request_stop(slots[index].session);
	}
	for (index = 0u; index < slot_count; ++index) {
		if (slots[index].session)
			(void)displaylink_session_join(slots[index].session);
	}
	for (index = 0u; index < slot_count; ++index) {
		if (slots[index].session) {
			displaylink_session_destroy(slots[index].session);
			slots[index].session = NULL;
		}
		slots[index].started = false;
	}
}

static bool demo_any_session_failed(const struct demo_session_slot *slots, size_t slot_count)
{
	size_t index;

	for (index = 0u; index < slot_count; ++index) {
		if (slots[index].session &&
		    slots[index].started &&
		    !displaylink_session_is_running(slots[index].session) &&
		    displaylink_session_get_exit_code(slots[index].session) != EXIT_SUCCESS)
			return true;
	}

	return false;
}

int main(int argc, char **argv)
{
	struct demo_options demo_opts;
	struct demo_session_slot slots[MAX_SESSION_COUNT];
	struct displaylink_compositor_surface canvas;
	SDL_Window *window = NULL;
	SDL_Renderer *renderer = NULL;
	SDL_Texture *texture = NULL;
	struct timespec delay;
	size_t session_count = 0u;
	size_t session_start_index = 0u;
	size_t index;
	bool have_damage = false;
	int exit_code = EXIT_FAILURE;

	memset(slots, 0, sizeof(slots));
	memset(&canvas, 0, sizeof(canvas));
	demo_default_options(&demo_opts);
	if (demo_parse_args(argc, argv, &demo_opts, &session_start_index) != 0 ||
	    session_start_index == 0u ||
	    demo_parse_sessions(argc, argv, session_start_index, slots, &session_count) != 0) {
		demo_usage(argv[0]);
		goto out;
	}
	if (demo_validate_udc_requests(slots, session_count) != 0)
		goto out;

	demo_layout_slots(slots, session_count, &demo_opts.canvas_width, &demo_opts.canvas_height);
	if (demo_opts.canvas_width == 0u || demo_opts.canvas_height == 0u) {
		fprintf(stderr, "invalid composite canvas size\n");
		goto out;
	}
	if (!displaylink_compositor_surface_init(&canvas, demo_opts.canvas_width, demo_opts.canvas_height)) {
		fprintf(stderr, "unable to allocate composite canvas\n");
		goto out;
	}

	if (demo_opts.show_window) {
		if (SDL_Init(SDL_INIT_VIDEO) != 0) {
			fprintf(stderr,
				"SDL_Init failed: %s; continuing without a preview window\n",
				SDL_GetError());
			demo_opts.show_window = false;
		} else {
			window = SDL_CreateWindow("DisplayLink Multi-Session Demo",
					      SDL_WINDOWPOS_CENTERED,
					      SDL_WINDOWPOS_CENTERED,
					      (int)(demo_opts.canvas_width * demo_opts.window_scale),
					      (int)(demo_opts.canvas_height * demo_opts.window_scale),
					      SDL_WINDOW_RESIZABLE);
			if (!window) {
				fprintf(stderr,
					"SDL_CreateWindow failed: %s; continuing without a preview window\n",
					SDL_GetError());
				demo_opts.show_window = false;
			} else {
				renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
				if (!renderer)
					renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE);
				if (!renderer) {
					fprintf(stderr,
						"SDL_CreateRenderer failed: %s; continuing without a preview window\n",
						SDL_GetError());
					SDL_DestroyWindow(window);
					window = NULL;
					demo_opts.show_window = false;
				} else {
					texture = SDL_CreateTexture(renderer,
						    SDL_PIXELFORMAT_ARGB8888,
						    SDL_TEXTUREACCESS_STREAMING,
						    (int)canvas.width,
						    (int)canvas.height);
					if (!texture) {
						fprintf(stderr,
							"SDL_CreateTexture failed: %s; continuing without a preview window\n",
							SDL_GetError());
						SDL_DestroyRenderer(renderer);
						SDL_DestroyWindow(window);
						renderer = NULL;
						window = NULL;
						demo_opts.show_window = false;
					}
				}
			}
		}
	}

	demo_install_signal_handlers();
	if (demo_start_sessions(slots, session_count) != 0) {
		fprintf(stderr, "failed to start one or more displaylink sessions\n");
		goto out;
	}

	delay.tv_sec = demo_opts.poll_interval_ms / 1000u;
	delay.tv_nsec = (long)(demo_opts.poll_interval_ms % 1000u) * 1000000L;
	while (!demo_stop_requested) {
		SDL_Event event;

		have_damage = false;
		if (window) {
			while (SDL_PollEvent(&event)) {
				if (event.type == SDL_QUIT)
					demo_stop_requested = 1;
			}
		}
		for (index = 0u; index < session_count; ++index) {
			struct displaylink_output_frame frame;
			struct udl_sink_damage damage;
			struct udl_sink_damage composed_damage;

			if (!slots[index].session)
				continue;
			if (!displaylink_session_acquire_latest_output(slots[index].session, &frame))
				continue;
			if (frame.sequence == slots[index].last_sequence) {
				displaylink_session_release_output(slots[index].session, &frame);
				continue;
			}
			slots[index].last_sequence = frame.sequence;
			udl_sink_clear_damage(&damage);
			udl_sink_clear_damage(&composed_damage);
			if (demo_frame_to_damage(&frame, &damage) &&
			    !displaylink_compositor_surface_blit_damage(&canvas,
							 frame.surface.pixels,
							 frame.surface.stride_pixels,
							 frame.visible_width != 0u ? frame.visible_width : frame.surface.width,
							 frame.visible_height != 0u ? frame.visible_height : frame.surface.height,
							 slots[index].layout_x,
							 slots[index].layout_y,
							 &damage,
							 &composed_damage)) {
				displaylink_session_release_output(slots[index].session, &frame);
				fprintf(stderr, "compositor blit failed for session %zu\n", index);
				demo_stop_requested = 1;
				break;
			}
			have_damage = have_damage || composed_damage.touched;
			displaylink_session_release_output(slots[index].session, &frame);
		}

		if (demo_any_session_failed(slots, session_count)) {
			fprintf(stderr, "one or more displaylink sessions exited early\n");
			demo_stop_requested = 1;
		}

		if (window && have_damage) {
			SDL_Rect dst_rect;
			SDL_UpdateTexture(texture,
					  NULL,
					  canvas.pixels,
					  (int)(canvas.stride_pixels * sizeof(*canvas.pixels)));
			SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
			SDL_RenderClear(renderer);
			dst_rect.x = 0;
			dst_rect.y = 0;
			SDL_GetRendererOutputSize(renderer, &dst_rect.w, &dst_rect.h);
			SDL_RenderCopy(renderer, texture, NULL, &dst_rect);
			SDL_RenderPresent(renderer);
		}

		nanosleep(&delay, NULL);
	}

	exit_code = EXIT_SUCCESS;

out:
	demo_stop_sessions(slots, session_count);
	if (demo_opts.dump_image_path)
		(void)demo_write_ppm(demo_opts.dump_image_path, &canvas, canvas.width, canvas.height);
	if (texture)
		SDL_DestroyTexture(texture);
	if (renderer)
		SDL_DestroyRenderer(renderer);
	if (window)
		SDL_DestroyWindow(window);
	if (demo_opts.show_window || texture || renderer || window)
		SDL_Quit();
	displaylink_compositor_surface_destroy(&canvas);
	for (index = 0u; index < session_count; ++index)
		demo_free_ring(&slots[index]);
	return exit_code;
}