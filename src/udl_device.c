#define _POSIX_C_SOURCE 200809L

#include "common.h"
#include "udl_device.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <ctype.h>
#include <limits.h>
#include <time.h>
#include <unistd.h>

/* ----------------------------------------------------------------
 * host_to_le16 helper
 * ---------------------------------------------------------------- */
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define host_to_le16(value) ((uint16_t)(value))
#else
#define host_to_le16(value) __builtin_bswap16((uint16_t)(value))
#endif

/* ----------------------------------------------------------------
 * File-local constants
 * ---------------------------------------------------------------- */
static const uint8_t k_displaylink_nullkey1[16] = {
	0x57, 0xcd, 0xdc, 0xa7, 0x1c, 0x88, 0x5e, 0x15,
	0x60, 0xfe, 0xc6, 0x97, 0x16, 0x3d, 0x47, 0xf2,
};

static const uint8_t k_displaylink_nullkey2[16] = {
	0x47, 0x3d, 0x16, 0x97, 0xc6, 0xfe, 0x60, 0x15,
	0x5e, 0x88, 0x1c, 0xa7, 0xdc, 0xb7, 0x6f, 0xf2,
};

static const uint8_t k_windows_probe_reply[4] = {
	0x01, 0x00, 0x00, 0x00,
};

static const uint8_t k_windows_status_reply[4] = {
	0x01, 0x00, 0x00, 0x00,
};

#define USBIP_CADENCE_LOG_INTERVAL_NS 1000000000ULL

/* ----------------------------------------------------------------
 * Crypto: stream and map tables (file-local)
 * ---------------------------------------------------------------- */
static uint8_t g_udl_crypt_stream[UDL_CRYPT_STREAM_PERIOD];
static uint16_t g_udl_crypt_map[UDL_CRYPT_MAP_SIZE];
static bool g_udl_crypt_initialized = false;

static uint64_t monotonic_time_ns(void)
{
	struct timespec ts;

	if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
		return 0u;
	return ((uint64_t)ts.tv_sec * 1000000000ULL) + (uint64_t)ts.tv_nsec;
}

static void usbip_cadence_reset(struct device_runtime *runtime, uint64_t now_ns)
{
	if (!runtime)
		return;

	memset(&runtime->usbip_cadence, 0, sizeof(runtime->usbip_cadence));
	runtime->usbip_cadence.window_start_ns = now_ns;
}

static void usbip_cadence_note_control_submit(struct device_runtime *runtime,
					      const struct usbip_header *request,
					      size_t payload_length)
{
	if (!runtime)
		return;

	runtime->usbip_cadence.control_submit_count += 1u;
	if (request && request->base.direction == USBIP_DIR_OUT)
		runtime->usbip_cadence.control_out_bytes += payload_length;
}

static void usbip_cadence_note_control_reply(struct device_runtime *runtime,
					     size_t payload_length)
{
	if (!runtime)
		return;

	runtime->usbip_cadence.control_in_reply_bytes += payload_length;
}

static void usbip_cadence_note_bulk_out(struct device_runtime *runtime, size_t payload_length)
{
	if (!runtime)
		return;

	runtime->usbip_cadence.bulk_out_submit_count += 1u;
	runtime->usbip_cadence.bulk_out_bytes += payload_length;
	if (payload_length == 0u) {
		runtime->usbip_cadence.bulk_out_zero_count += 1u;
	} else if (payload_length <= 256u) {
		runtime->usbip_cadence.bulk_out_le_256_count += 1u;
	} else if (payload_length <= 4096u) {
		runtime->usbip_cadence.bulk_out_le_4k_count += 1u;
	} else if (payload_length <= 16384u) {
		runtime->usbip_cadence.bulk_out_le_16k_count += 1u;
	} else if (payload_length <= 65536u) {
		runtime->usbip_cadence.bulk_out_le_64k_count += 1u;
	} else {
		runtime->usbip_cadence.bulk_out_gt_64k_count += 1u;
	}
}

static void usbip_cadence_note_interrupt_in(struct device_runtime *runtime, size_t payload_length)
{
	if (!runtime)
		return;

	runtime->usbip_cadence.interrupt_in_submit_count += 1u;
	runtime->usbip_cadence.interrupt_in_reply_bytes += payload_length;
}

static void usbip_cadence_note_error(struct device_runtime *runtime)
{
	if (!runtime)
		return;

	runtime->usbip_cadence.error_submit_count += 1u;
}

static void usbip_cadence_note_unlink(struct device_runtime *runtime)
{
	if (!runtime)
		return;

	runtime->usbip_cadence.unlink_count += 1u;
}

static void usbip_cadence_maybe_log(struct device_runtime *runtime, bool force)
{
	const char *busid;
	uint64_t now_ns;
	uint64_t elapsed_ns;
	double elapsed_s;
	double ctrl_rate;
	double bulk_rate;
	double bulk_mib_s;
	double bulk_avg_kib;
	double intr_rate;
	double errors_rate;
	struct usbip_cadence_window *stats;

	if (!runtime || !runtime->usbip_cadence_logging_enabled)
		return;

	now_ns = monotonic_time_ns();
	stats = &runtime->usbip_cadence;
	if (stats->window_start_ns == 0u)
		stats->window_start_ns = now_ns;
	if (stats->window_start_ns == 0u || now_ns < stats->window_start_ns)
		return;

	elapsed_ns = now_ns - stats->window_start_ns;
	if (!force && elapsed_ns < USBIP_CADENCE_LOG_INTERVAL_NS)
		return;
	if (stats->control_submit_count == 0u &&
	    stats->bulk_out_submit_count == 0u &&
	    stats->interrupt_in_submit_count == 0u &&
	    stats->unlink_count == 0u &&
	    stats->error_submit_count == 0u) {
		stats->window_start_ns = now_ns;
		return;
	}
	if (elapsed_ns == 0u)
		elapsed_ns = 1u;

	elapsed_s = (double)elapsed_ns / 1000000000.0;
	ctrl_rate = (double)stats->control_submit_count / elapsed_s;
	bulk_rate = (double)stats->bulk_out_submit_count / elapsed_s;
	bulk_mib_s = ((double)stats->bulk_out_bytes / (1024.0 * 1024.0)) / elapsed_s;
	bulk_avg_kib = stats->bulk_out_submit_count > 0u ?
		((double)stats->bulk_out_bytes / 1024.0) / (double)stats->bulk_out_submit_count : 0.0;
	intr_rate = (double)stats->interrupt_in_submit_count / elapsed_s;
	errors_rate = (double)stats->error_submit_count / elapsed_s;
	busid = runtime->usbip_device.busid[0] != '\0' ? runtime->usbip_device.busid :
		(runtime->opts.busid ? runtime->opts.busid : "?");

	fprintf(stderr,
		"USB/IP cadence [%s]: ctrl=%.1f/s ctrl_out=%.1f KiB/s ctrl_in=%.1f KiB/s bulk_out=%.1f/s bulk_avg=%.1f KiB bulk_bw=%.2f MiB/s bulk_bins={0:%llu <=256B:%llu <=4KiB:%llu <=16KiB:%llu <=64KiB:%llu >64KiB:%llu} intr_in=%.1f/s intr_in_reply=%.1f KiB/s unlink=%llu err=%.1f/s window=%.2fs\n",
		busid,
		ctrl_rate,
		((double)stats->control_out_bytes / 1024.0) / elapsed_s,
		((double)stats->control_in_reply_bytes / 1024.0) / elapsed_s,
		bulk_rate,
		bulk_avg_kib,
		bulk_mib_s,
		(unsigned long long)stats->bulk_out_zero_count,
		(unsigned long long)stats->bulk_out_le_256_count,
		(unsigned long long)stats->bulk_out_le_4k_count,
		(unsigned long long)stats->bulk_out_le_16k_count,
		(unsigned long long)stats->bulk_out_le_64k_count,
		(unsigned long long)stats->bulk_out_gt_64k_count,
		intr_rate,
		((double)stats->interrupt_in_reply_bytes / 1024.0) / elapsed_s,
		(unsigned long long)stats->unlink_count,
		errors_rate,
		elapsed_s);

	usbip_cadence_reset(runtime, now_ns);
}

static uint16_t udl_crypt_crc12(const uint8_t *data, size_t len)
{
	uint16_t rem = 0u;
	size_t i;

	if (!data)
		return 0u;

	for (i = 0u; i < len; ++i) {
		uint8_t bit;

		for (bit = 0u; bit < 8u; ++bit) {
			rem = (uint16_t)((rem << 1u) | ((data[i] >> bit) & 0x01u));
			if ((rem & 0x1000u) != 0u)
				rem ^= UDL_CRYPT_POLY;
		}
	}

	return rem;
}

static void udl_crypt_init_tables(void)
{
	unsigned coeffs[32];
	unsigned coeff_count = 0u;
	unsigned i;
	uint16_t val = 0x0001u;

	if (g_udl_crypt_initialized)
		return;

	memset(g_udl_crypt_stream, 0, sizeof(g_udl_crypt_stream));
	memset(g_udl_crypt_map, 0, sizeof(g_udl_crypt_map));

	for (i = 0u; i < 32u; ++i) {
		const unsigned tmp = 1u << i;

		if ((tmp & UDL_CRYPT_LFSR12) == 0u)
			continue;

		coeffs[coeff_count++] = i;
	}

	for (i = 0u; i < UDL_CRYPT_STREAM_PERIOD; ++i) {
		unsigned j;

		g_udl_crypt_stream[i] = (uint8_t)(val & 0xffu);
		g_udl_crypt_map[val & 0x0fffu] = (uint16_t)i;

		for (j = 0u; j < 8u; ++j) {
			unsigned k;
			unsigned res = 0u;

			for (k = 0u; k < coeff_count; ++k)
				res ^= (unsigned)((val >> coeffs[k]) & 0x01u);

			val = (uint16_t)(((val << 1u) ^ (res & 0x01u)) & 0x0fffu);
		}
	}

	g_udl_crypt_initialized = true;
}

static void udl_decrypt_bulk_payload(struct device_runtime *runtime,
					      uint8_t *payload,
					      size_t payload_length)
{
	size_t i;
	uint16_t offset;

	if (!runtime || !payload || payload_length == 0u || !runtime->decrypt_enabled)
		return;

	offset = runtime->decrypt_offset;
	for (i = 0u; i < payload_length; ++i) {
		payload[i] ^= g_udl_crypt_stream[offset];
		offset += 1u;
		if (offset >= UDL_CRYPT_STREAM_PERIOD)
			offset = 0u;
	}

	runtime->decrypt_offset = offset;
}

/* ----------------------------------------------------------------
 * Control RAM helpers
 * ---------------------------------------------------------------- */
void udl_control_ram_seed(struct device_runtime *runtime)
{
	if (!runtime)
		return;

	memset(runtime->control_ram, 0, sizeof(runtime->control_ram));
	runtime->control_ram[UDL_CONTROL_RAM_STARTUP_STATUS_ADDR] = UDL_CONTROL_RAM_STARTUP_STATUS_VALUE;
}

static uint8_t udl_control_ram_read_byte(const struct device_runtime *runtime, uint16_t address)
{
	if (!runtime)
		return 0u;

	if (address >= UDL_CONTROL_RAM_GRAPHICS_BASE && address < UDL_CONTROL_RAM_GRAPHICS_LIMIT)
		return runtime->udl.sink.registers[address - UDL_CONTROL_RAM_GRAPHICS_BASE];

	return runtime->control_ram[address];
}

static void udl_control_ram_write_byte(struct device_runtime *runtime,
					      uint16_t address,
					      uint8_t value)
{
	if (!runtime)
		return;

	runtime->control_ram[address] = value;
	if (address >= UDL_CONTROL_RAM_GRAPHICS_BASE && address < UDL_CONTROL_RAM_GRAPHICS_LIMIT)
		runtime->udl.sink.registers[address - UDL_CONTROL_RAM_GRAPHICS_BASE] = value;
}

/* ----------------------------------------------------------------
 * Hex logging helpers
 * ---------------------------------------------------------------- */
static void log_hex_preview(const char *label, const uint8_t *data, size_t length)
{
	size_t index;
	size_t preview_length;

	if (!data)
		return;

	preview_length = length < 32u ? length : 32u;
	fprintf(stderr, "%s (%zu bytes):", label ? label : "payload", length);
	for (index = 0u; index < preview_length; ++index)
		fprintf(stderr, " %02x", data[index]);
	if (preview_length < length)
		fprintf(stderr, " ...");
	fputc('\n', stderr);
}

void log_hex_window(const char *label,
		    const uint8_t *data,
		    size_t length,
		    size_t offset,
		    size_t window_length)
{
	size_t index;
	size_t end;

	if (!data || offset >= length)
		return;

	end = offset + window_length;
	if (end > length)
		end = length;

	fprintf(stderr,
		"%s [%zu..%zu of %zu bytes]:",
		label ? label : "payload window",
		offset,
		end,
		length);
	for (index = offset; index < end; ++index)
		fprintf(stderr, " %02x", data[index]);
	if (end < length)
		fprintf(stderr, " ...");
	fputc('\n', stderr);
}

static void log_hex_dump(const char *label,
			 const uint8_t *data,
			 size_t length,
			 size_t bytes_per_line)
{
	size_t offset;

	if (!data || length == 0u)
		return;
	if (bytes_per_line == 0u)
		bytes_per_line = 16u;

	fprintf(stderr, "%s (%zu bytes):\n", label ? label : "payload dump", length);
	for (offset = 0u; offset < length; offset += bytes_per_line) {
		size_t index;
		size_t line_end = offset + bytes_per_line;

		if (line_end > length)
			line_end = length;

		fprintf(stderr, "  %04zx:", offset);
		for (index = offset; index < line_end; ++index)
			fprintf(stderr, " %02x", data[index]);
		fputc('\n', stderr);
	}
}

/* ----------------------------------------------------------------
 * UDL command helpers
 * ---------------------------------------------------------------- */
static size_t udl_e0_command_length(const uint8_t *command, size_t length)
{
	if (!command || length < UDL_CMD_E0_TOTAL_SIZE)
		return 0u;

	if (length == UDL_CMD_E0_TOTAL_SIZE + UDL_CMD_E0_TRAILER_SIZE &&
	    command[UDL_CMD_E0_TOTAL_SIZE] != UDL_MSG_BULK) {
		return UDL_CMD_E0_TOTAL_SIZE + UDL_CMD_E0_TRAILER_SIZE;
	}

	if (length >= UDL_CMD_E0_TOTAL_SIZE + UDL_CMD_E0_TRAILER_SIZE + 1u &&
	    command[UDL_CMD_E0_TOTAL_SIZE] != UDL_MSG_BULK &&
	    command[UDL_CMD_E0_TOTAL_SIZE + UDL_CMD_E0_TRAILER_SIZE] == UDL_MSG_BULK) {
		return UDL_CMD_E0_TOTAL_SIZE + UDL_CMD_E0_TRAILER_SIZE;
	}

	return UDL_CMD_E0_TOTAL_SIZE;
}

static void maybe_log_first_e0_command(struct device_runtime *runtime,
				       const uint8_t *data,
				       size_t length)
{
	size_t offset;

	if (!runtime || runtime->first_e0_dumped || !runtime->opts.verbose || !data || length < 2u)
		return;

	for (offset = 0u; offset + 2u <= length; ++offset) {
		size_t command_len;

		if (data[offset] != UDL_MSG_BULK || data[offset + 1u] != UDL_CMD_OPAQUE_E0)
			continue;

		command_len = udl_e0_command_length(&data[offset], length - offset);
		if (command_len == 0u)
			return;

		fprintf(stderr,
			"Captured first UDL Huffman/init command at bulk offset %zu len=%zu%s\n",
			offset,
			command_len,
			command_len > UDL_CMD_E0_TOTAL_SIZE ? " (with trailer)" : "");
		log_hex_dump("UDL Huffman/init command dump", &data[offset], command_len, 16u);
		runtime->first_e0_dumped = true;
		return;
	}
}

static bool udl_bulk_contains_huffman_init(const uint8_t *data, size_t length)
{
	size_t offset;

	if (!data || length < 6u)
		return false;

	for (offset = 0u; offset + 6u <= length; ++offset) {
		if (data[offset] == UDL_MSG_BULK &&
		    data[offset + 1u] == UDL_CMD_OPAQUE_E0 &&
		    data[offset + 2u] == 0x26u &&
		    data[offset + 3u] == 0x38u &&
		    data[offset + 4u] == 0x71u &&
		    data[offset + 5u] == 0xcdu)
			return true;
	}

	return false;
}

static uint64_t udl_payload_signature64(const uint8_t *data, size_t length)
{
	uint64_t hash = 1469598103934665603ull;
	size_t offset;

	if (!data)
		return 0u;

	for (offset = 0u; offset < length; ++offset) {
		hash ^= (uint64_t)data[offset];
		hash *= 1099511628211ull;
	}

	return hash;
}

const char *udl_resync_reason_name(enum udl_transport_resync_reason reason)
{
	switch (reason) {
	case UDL_TRANSPORT_RESYNC_NONE:
		return "none";
	case UDL_TRANSPORT_RESYNC_NON_BULK:
		return "non-bulk";
	case UDL_TRANSPORT_RESYNC_DOUBLE_BULK:
		return "double-bulk";
	case UDL_TRANSPORT_RESYNC_INVALID_COMMAND:
		return "invalid-command";
	default:
		return "unknown";
	}
}

/* ----------------------------------------------------------------
 * LE reply helpers
 * ---------------------------------------------------------------- */
static void store_le32_reply(uint8_t *dst, uint32_t value)
{
	if (!dst)
		return;

	dst[0] = (uint8_t)(value & 0xffu);
	dst[1] = (uint8_t)((value >> 8) & 0xffu);
	dst[2] = (uint8_t)((value >> 16) & 0xffu);
	dst[3] = (uint8_t)((value >> 24) & 0xffu);
}

static void store_le16_reply(uint8_t *dst, uint16_t value)
{
	if (!dst)
		return;

	dst[0] = (uint8_t)(value & 0xffu);
	dst[1] = (uint8_t)((value >> 8) & 0xffu);
}

/* ----------------------------------------------------------------
 * MS OS descriptor builders
 * ---------------------------------------------------------------- */
static size_t build_ms_os_string_descriptor(uint8_t *buffer, size_t capacity)
{
	static const uint8_t descriptor[] = {
		18u,
		USB_DT_STRING,
		'M', 0u,
		'S', 0u,
		'F', 0u,
		'T', 0u,
		'1', 0u,
		'0', 0u,
		'0', 0u,
		UDL_MS_OS_VENDOR_CODE,
		0u,
	};

	if (!buffer || capacity < sizeof(descriptor))
		return 0u;

	memcpy(buffer, descriptor, sizeof(descriptor));
	return sizeof(descriptor);
}

static size_t build_ms_os_compat_id_descriptor(uint8_t *buffer, size_t capacity)
{
	if (!buffer || capacity < 16u)
		return 0u;

	memset(buffer, 0, 16u);
	store_le32_reply(&buffer[0], 16u);
	store_le16_reply(&buffer[4], 0x0100u);
	store_le16_reply(&buffer[6], UDL_MS_OS_EXT_COMPAT_INDEX);
	buffer[8] = 0u;
	return 16u;
}

static size_t build_ms_os_ext_properties_descriptor(uint8_t *buffer, size_t capacity)
{
	if (!buffer || capacity < 10u)
		return 0u;

	memset(buffer, 0, 10u);
	store_le32_reply(&buffer[0], 10u);
	store_le16_reply(&buffer[4], 0x0100u);
	store_le16_reply(&buffer[6], UDL_MS_OS_EXT_PROPERTIES_INDEX);
	store_le16_reply(&buffer[8], 0u);
	return 10u;
}

/* ----------------------------------------------------------------
 * UDL register name and writereg logging
 * ---------------------------------------------------------------- */
static const char *udl_register_name(uint8_t reg)
{
	switch (reg) {
	case 0x01u:
		return "XDISPLAYSTART_HI";
	case 0x02u:
		return "XDISPLAYSTART_LO";
	case 0x03u:
		return "XDISPLAYEND_HI";
	case 0x04u:
		return "XDISPLAYEND_LO";
	case 0x05u:
		return "YDISPLAYSTART_HI";
	case 0x06u:
		return "YDISPLAYSTART_LO";
	case 0x07u:
		return "YDISPLAYEND_HI";
	case 0x08u:
		return "YDISPLAYEND_LO";
	case 0x09u:
		return "XENDCOUNT_HI";
	case 0x0au:
		return "XENDCOUNT_LO";
	case 0x0bu:
		return "HSYNCSTART_HI";
	case 0x0cu:
		return "HSYNCSTART_LO";
	case 0x0du:
		return "HSYNCEND_HI";
	case 0x0eu:
		return "HSYNCEND_LO";
	case 0x00u:
		return "COLORDEPTH";
	case 0x0fu:
		return "HPIXELS_HI";
	case 0x10u:
		return "HPIXELS_LO";
	case 0x11u:
		return "YENDCOUNT_HI";
	case 0x12u:
		return "YENDCOUNT_LO";
	case 0x13u:
		return "VSYNCSTART_HI";
	case 0x14u:
		return "VSYNCSTART_LO";
	case 0x15u:
		return "VSYNCEND_HI";
	case 0x16u:
		return "VSYNCEND_LO";
	case 0x17u:
		return "VPIXELS_HI";
	case 0x18u:
		return "VPIXELS_LO";
	case 0x1bu:
		return "PIXELCLOCK5KHZ_HI";
	case 0x1cu:
		return "PIXELCLOCK5KHZ_LO";
	case 0x1fu:
		return "BLANKMODE";
	case 0x20u:
		return "BASE16_ADDR2";
	case 0x21u:
		return "BASE16_ADDR1";
	case 0x22u:
		return "BASE16_ADDR0";
	case 0x26u:
		return "BASE8_ADDR2";
	case 0x27u:
		return "BASE8_ADDR1";
	case 0x28u:
		return "BASE8_ADDR0";
	case 0xffu:
		return "VIDREG";
	default:
		return NULL;
	}
}

void log_udl_writereg_commands(const uint8_t *data, size_t length)
{
	size_t offset;
	size_t logged = 0u;
	bool truncated = false;
	bool found = false;

	if (!data || length < 4u)
		return;

	for (offset = 0u; offset + 4u <= length; ++offset) {
		if (data[offset] == 0xafu && data[offset + 1u] == 0x20u) {
			const uint8_t reg = data[offset + 2u];
			const uint8_t value = data[offset + 3u];
			const char *name = udl_register_name(reg);

			if (logged >= 32u) {
				truncated = true;
				break;
			}

			fprintf(stderr,
				"UDL WRITEREG reg=0x%02x%s%s%s value=0x%02x\n",
				(unsigned int)reg,
				name ? " (" : "",
				name ? name : "",
				name ? ")" : "",
				(unsigned int)value);
			logged += 1u;
			found = true;
			offset += 3u;
		}
	}

	if (truncated) {
		fprintf(stderr,
			"UDL WRITEREG log truncated after %zu entries in one payload\n",
			logged);
	}

	if (!found && length <= 64u)
		fprintf(stderr, "UDL WRITEREG scan: no register writes found in %zu-byte payload\n", length);
}

/* ----------------------------------------------------------------
 * Windows bulk state tracking
 * ---------------------------------------------------------------- */
static uint32_t windows_probe_reply_value(const struct device_runtime *runtime)
{
	(void)runtime;

	/* Tubecable's documented 0x06 poll-status reply is a 32-bit word,
	 * with 0xf0005000 observed as the healthy/default device state.
	 */
	return UDL_WINDOWS_POLL_STATUS_OK;
}

static uint32_t windows_status_reply_value(const struct device_runtime *runtime)
{
	if (!runtime)
		return 1u;
	if (!runtime->key_loaded)
		return 1u;
	if (!runtime->windows_display_ready)
		return 1u;
	return 0u;
}

static uint32_t windows_query_05_reply_value(const struct device_runtime *runtime)
{
	if (!runtime)
		return 0u;
	return runtime->key_loaded ? UDL_VENDOR_REQUEST_WINDOWS_QUERY_05_REPLY : 0u;
}

static void update_windows_bulk_state(struct device_runtime *runtime,
					      const uint8_t *data,
					      size_t length)
{
	size_t offset;
	bool state_changed = false;

	if (!runtime || !data || length < 4u)
		return;

	if (udl_bulk_contains_huffman_init(data, length) && !runtime->huffman_table_loaded) {
		runtime->huffman_table_loaded = true;
		state_changed = true;
	}

	for (offset = 0u; offset + 4u <= length; ++offset) {
		if (data[offset] != 0xafu || data[offset + 1u] != 0x20u)
			continue;

		if (data[offset + 2u] == 0x1fu && data[offset + 3u] == 0x00u && !runtime->windows_blankmode_on) {
			runtime->windows_blankmode_on = true;
			state_changed = true;
		}
		if (data[offset + 2u] == 0xffu && data[offset + 3u] == 0xffu && !runtime->windows_vidreg_unlock) {
			runtime->windows_vidreg_unlock = true;
			state_changed = true;
		}
	}

	if (runtime->windows_display_ready !=
	    (runtime->key_loaded && runtime->huffman_table_loaded &&
	     runtime->windows_blankmode_on &&
	     runtime->windows_vidreg_unlock)) {
		runtime->windows_display_ready = runtime->key_loaded &&
			runtime->huffman_table_loaded &&
			runtime->windows_blankmode_on &&
			runtime->windows_vidreg_unlock;
		state_changed = true;
	}

	if (state_changed && runtime->opts.verbose) {
		fprintf(stderr,
			"Windows display state: key_loaded=%s huffman_loaded=%s vendor_14_seen=%s blankmode_on=%s vidreg_unlock=%s hpix=%u vpix=%u ready=%s\n",
			runtime->key_loaded ? "yes" : "no",
			runtime->huffman_table_loaded ? "yes" : "no",
			runtime->vendor_14_seen ? "yes" : "no",
			runtime->windows_blankmode_on ? "yes" : "no",
			runtime->windows_vidreg_unlock ? "yes" : "no",
			(unsigned int)udl_sink_get_hpixels(&runtime->udl.sink),
			(unsigned int)udl_sink_get_vpixels(&runtime->udl.sink),
			runtime->windows_display_ready ? "yes" : "no");
	}
}

/* ----------------------------------------------------------------
 * EDID builders
 * ---------------------------------------------------------------- */
int load_edid_file(const char *path, uint8_t edid[256])
{
	FILE *file;
	size_t bytes_read;
	uint8_t extra_byte;

	/* Only the base block is served from a file; clear the extension area so a
	 * reused buffer never leaves stale CEA bytes behind it. */
	memset(edid + 128, 0, 128u);

	file = fopen(path, "rb");
	if (!file) {
		perror(path);
		return -1;
	}

	bytes_read = fread(edid, 1u, 128u, file);
	if (bytes_read != 128u) {
		fprintf(stderr, "EDID file '%s' must contain exactly 128 bytes\n", path);
		fclose(file);
		return -1;
	}
	if (fread(&extra_byte, 1u, 1u, file) == 1u || edid[126] != 0u) {
		uint32_t sum = 0u;
		size_t index;

		fprintf(stderr,
			"EDID file '%s' contains extension data; only the base 128-byte block will be served\n",
			path);
		edid[126] = 0u;
		for (index = 0u; index < 127u; ++index)
			sum += edid[index];
		edid[127] = (uint8_t)((256u - (sum & 0xffu)) & 0xffu);
	}

	fclose(file);
	return 0;
}

static uint16_t encode_manufacturer_id(char first, char second, char third)
{
	return (uint16_t)((((uint16_t)(first - '@')) & 0x1fu) << 10 |
			 (((uint16_t)(second - '@')) & 0x1fu) << 5 |
			 (((uint16_t)(third - '@')) & 0x1fu));
}

static void write_ascii_monitor_name(uint8_t *dst, const char *name)
{
	size_t length = strlen(name);

	memset(dst, ' ', 13u);
	if (length > 13u)
		length = 13u;
	memcpy(dst, name, length);
	if (length < 13u)
		dst[length] = '\n';
}

static void build_preferred_dtd_rb(uint8_t dtd[18],
				  uint32_t width,
				  uint32_t height,
				  uint32_t refresh_hz)
{
	/* CVT-RB inspired fixed-blanking timing suitable for a synthetic preferred mode. */
	const uint32_t h_blank = 160u;
	const uint32_t v_blank = 31u;
	const uint32_t h_sync_offset = 48u;
	const uint32_t h_sync_width = 32u;
	const uint32_t v_sync_offset = 3u;
	const uint32_t v_sync_width = 5u;
	const uint32_t h_total = width + h_blank;
	const uint32_t v_total = height + v_blank;
	const uint32_t safe_refresh_hz = refresh_hz == 0u ? 60u : refresh_hz;
	const uint32_t pixel_clock_10khz =
		(uint32_t)((((uint64_t)h_total * (uint64_t)v_total * safe_refresh_hz) + 5000u) / 10000u);

	memset(dtd, 0, 18u);
	dtd[0] = (uint8_t)(pixel_clock_10khz & 0xffu);
	dtd[1] = (uint8_t)((pixel_clock_10khz >> 8) & 0xffu);
	dtd[2] = (uint8_t)(width & 0xffu);
	dtd[3] = (uint8_t)(h_blank & 0xffu);
	dtd[4] = (uint8_t)(((width >> 8) & 0x0fu) << 4) | (uint8_t)((h_blank >> 8) & 0x0fu);
	dtd[5] = (uint8_t)(height & 0xffu);
	dtd[6] = (uint8_t)(v_blank & 0xffu);
	dtd[7] = (uint8_t)(((height >> 8) & 0x0fu) << 4) | (uint8_t)((v_blank >> 8) & 0x0fu);
	dtd[8] = (uint8_t)(h_sync_offset & 0xffu);
	dtd[9] = (uint8_t)(h_sync_width & 0xffu);
	dtd[10] = (uint8_t)(((v_sync_offset & 0x0fu) << 4) | (v_sync_width & 0x0fu));
	dtd[11] = (uint8_t)(((h_sync_offset >> 8) & 0x03u) << 6 |
			   ((h_sync_width >> 8) & 0x03u) << 4 |
			   ((v_sync_offset >> 4) & 0x03u) << 2 |
			   ((v_sync_width >> 4) & 0x03u));
	dtd[12] = 0x40u;
	dtd[13] = 0x2cu;
	dtd[14] = 0x45u;
	dtd[15] = 0x00u;
	dtd[16] = 0x00u;
	dtd[17] = 0x1eu;
}

void build_default_edid(uint8_t edid[256],
			const char *monitor_name,
			uint32_t width,
			uint32_t height,
			uint32_t refresh_hz,
			bool strict_native_mode,
			bool allow_30hz_fallback)
{
	uint32_t sum = 0u;
	size_t index;
	const uint16_t manufacturer = encode_manufacturer_id('B', 'B', 'X');
	static const uint8_t chromaticity[10] = {
		0xee, 0x91, 0xa3, 0x54, 0x4c,
		0x99, 0x26, 0x0f, 0x50, 0x54,
	};
	uint8_t preferred_dtd[18];
	uint8_t alternate_dtd[18];
	uint32_t width_clamped;
	uint32_t height_clamped;
	uint32_t preferred_refresh;
	uint32_t alternate_refresh;
	uint32_t preferred_hfreq_khz;
	uint32_t alternate_hfreq_khz;
	uint32_t preferred_pixel_clock_10khz;
	uint8_t min_hfreq_khz;
	uint8_t max_hfreq_khz;
	uint8_t min_vertical_hz;
	uint8_t max_vertical_hz;
	uint8_t max_pixel_clock_10mhz;

	if (width == 0u) {
		width = 1920u;
	}
	if (height == 0u) {
		height = 1080u;
	}
	width_clamped = width > 4095u ? 4095u : width;
	height_clamped = height > 4095u ? 4095u : height;
	preferred_refresh = refresh_hz == 0u ? 60u : refresh_hz;
	alternate_refresh = 0u;
	if (allow_30hz_fallback)
		alternate_refresh = (preferred_refresh == 30u) ? 60u : 30u;
	if (!strict_native_mode && alternate_refresh == 0u)
		alternate_refresh = (preferred_refresh == 30u) ? 60u : 30u;

	preferred_pixel_clock_10khz = (uint32_t)((((uint64_t)(width_clamped + 160u) *
						   (uint64_t)(height_clamped + 31u) *
						   preferred_refresh) + 5000u) / 10000u);
	preferred_hfreq_khz = preferred_pixel_clock_10khz == 0u ? 1u :
		(uint32_t)(((uint64_t)preferred_pixel_clock_10khz * 10u + ((uint64_t)(width_clamped + 160u) / 2u)) /
			   (uint64_t)(width_clamped + 160u));
	if (preferred_hfreq_khz == 0u)
		preferred_hfreq_khz = 1u;

	alternate_hfreq_khz = preferred_hfreq_khz;
	if (alternate_refresh != 0u) {
		const uint32_t alternate_pixel_clock_10khz =
			(uint32_t)((((uint64_t)(width_clamped + 160u) *
					     (uint64_t)(height_clamped + 31u) *
					     alternate_refresh) + 5000u) / 10000u);

		alternate_hfreq_khz = alternate_pixel_clock_10khz == 0u ? 1u :
			(uint32_t)(((uint64_t)alternate_pixel_clock_10khz * 10u + ((uint64_t)(width_clamped + 160u) / 2u)) /
				   (uint64_t)(width_clamped + 160u));
		if (alternate_hfreq_khz == 0u)
			alternate_hfreq_khz = 1u;
	}

	if (strict_native_mode) {
		min_vertical_hz = (uint8_t)(preferred_refresh > 255u ? 255u : preferred_refresh);
		max_vertical_hz = min_vertical_hz;
		min_hfreq_khz = (uint8_t)(preferred_hfreq_khz > 255u ? 255u : preferred_hfreq_khz);
		max_hfreq_khz = min_hfreq_khz;
		max_pixel_clock_10mhz = (uint8_t)(((preferred_pixel_clock_10khz + 999u) / 1000u) > 255u ?
			255u : ((preferred_pixel_clock_10khz + 999u) / 1000u));
		if (max_pixel_clock_10mhz == 0u)
			max_pixel_clock_10mhz = 1u;
	} else {
		uint32_t min_v = preferred_refresh;
		uint32_t max_v = preferred_refresh;
		uint32_t min_h = preferred_hfreq_khz;
		uint32_t max_h = preferred_hfreq_khz;

		if (alternate_refresh != 0u) {
			if (alternate_refresh < min_v)
				min_v = alternate_refresh;
			if (alternate_refresh > max_v)
				max_v = alternate_refresh;
			if (alternate_hfreq_khz < min_h)
				min_h = alternate_hfreq_khz;
			if (alternate_hfreq_khz > max_h)
				max_h = alternate_hfreq_khz;
		}
		if (min_v > 30u)
			min_v = 30u;
		if (max_v < 75u)
			max_v = 75u;
		if (min_h > 30u)
			min_h = 30u;
		if (max_h < 70u)
			max_h = 70u;
		min_vertical_hz = (uint8_t)(min_v > 255u ? 255u : min_v);
		max_vertical_hz = (uint8_t)(max_v > 255u ? 255u : max_v);
		min_hfreq_khz = (uint8_t)(min_h > 255u ? 255u : min_h);
		max_hfreq_khz = (uint8_t)(max_h > 255u ? 255u : max_h);
		max_pixel_clock_10mhz = 0x0fu;
	}

	memset(edid, 0, 128u);
	edid[0] = 0x00;
	edid[1] = 0xff;
	edid[2] = 0xff;
	edid[3] = 0xff;
	edid[4] = 0xff;
	edid[5] = 0xff;
	edid[6] = 0xff;
	edid[7] = 0x00;
	edid[8] = (uint8_t)(manufacturer >> 8);
	edid[9] = (uint8_t)manufacturer;
	edid[10] = 0x01;
	edid[11] = 0x00;
	edid[12] = 0x01;
	edid[13] = 0x00;
	edid[16] = 1u;
	edid[17] = 34u;
	edid[18] = 0x01;
	edid[19] = 0x03;
	edid[20] = 0x81;
	edid[21] = 0x40;
	edid[22] = 0x2d;
	edid[23] = 0x78;
	edid[24] = 0x0f;
	memcpy(&edid[25], chromaticity, sizeof(chromaticity));
	for (index = 35u; index <= 37u; ++index)
		edid[index] = 0x00;
	for (index = 38u; index < 54u; ++index)
		edid[index] = 0x01;
	build_preferred_dtd_rb(preferred_dtd, width_clamped, height_clamped, preferred_refresh);
	memcpy(&edid[54], preferred_dtd, sizeof(preferred_dtd));
	if (alternate_refresh != 0u)
		build_preferred_dtd_rb(alternate_dtd, width_clamped, height_clamped, alternate_refresh);
	edid[72] = 0x00;
	edid[73] = 0x00;
	edid[74] = 0x00;
	edid[75] = 0xfc;
	edid[76] = 0x00;
	write_ascii_monitor_name(&edid[77], monitor_name ? monitor_name : "Breezy Box");
	edid[90] = 0x00;
	edid[91] = 0x00;
	edid[92] = 0x00;
	edid[93] = 0xfd;
	edid[94] = 0x00;
	edid[95] = min_vertical_hz;
	edid[96] = max_vertical_hz;
	edid[97] = min_hfreq_khz;
	edid[98] = max_hfreq_khz;
	edid[99] = max_pixel_clock_10mhz;
	edid[100] = 0x00;
	edid[101] = 0x0a;
	edid[102] = 0x20;
	edid[103] = 0x20;
	edid[104] = 0x20;
	edid[105] = 0x20;
	edid[106] = 0x20;
	edid[107] = 0x20;
	if (alternate_refresh != 0u) {
		memcpy(&edid[108], alternate_dtd, sizeof(alternate_dtd));
	} else {
		edid[108] = 0x00;
		edid[109] = 0x00;
		edid[110] = 0x00;
		edid[111] = 0x10;
	}

	/*
	 * Single 128-byte base block only.  A CEA-861 extension was tried to coax
	 * Windows into offering sub-50Hz modes, but the DisplayLink Windows driver
	 * ignores the EDID timing list (synthesizes its own modes), so it never
	 * helped — and setting extensions=1 breaks the Linux udl driver, which
	 * validates the header but only reads block 0.  Keep it off.
	 */
	edid[126] = 0u;
	memset(&edid[128], 0, 128u);

	for (index = 0u; index < 127u; ++index)
		sum += edid[index];
	edid[127] = (uint8_t)((256u - (sum & 0xffu)) & 0xffu);
}

/* ----------------------------------------------------------------
 * USB descriptor builders
 * ---------------------------------------------------------------- */
void build_vendor_descriptor(struct device_runtime *runtime)
{
	static const uint8_t vendor_template[UDL_VENDOR_DESCRIPTOR_LENGTH] = {
		UDL_VENDOR_DESCRIPTOR_LENGTH,
		UDL_VENDOR_DESCRIPTOR_TYPE,
		(uint8_t)(UDL_VENDOR_DESCRIPTOR_VERSION & 0xffu),
		(uint8_t)(UDL_VENDOR_DESCRIPTOR_VERSION >> 8),
		UDL_VENDOR_DESCRIPTOR_LENGTH - 2u,
		(uint8_t)(UDL_VENDOR_KEY_DEVICE_CAPS & 0xffu),
		(uint8_t)(UDL_VENDOR_KEY_DEVICE_CAPS >> 8),
		1u,
		0x03u,
		(uint8_t)(UDL_VENDOR_KEY_FIRMWARE_REV & 0xffu),
		(uint8_t)(UDL_VENDOR_KEY_FIRMWARE_REV >> 8),
		4u,
		0x01u, 0x00u, 0x03u, 0xd0u,
		(uint8_t)(UDL_VENDOR_KEY_MAX_PIXELS & 0xffu),
		(uint8_t)(UDL_VENDOR_KEY_MAX_PIXELS >> 8),
		4u,
		0x00u, 0x00u, 0x00u, 0x00u,
		(uint8_t)(UDL_VENDOR_KEY_LINK_MODE & 0xffu),
		(uint8_t)(UDL_VENDOR_KEY_LINK_MODE >> 8),
		1u,
		0x02u,
	};
	const uint64_t pixels = (uint64_t)runtime->opts.decode_width * (uint64_t)runtime->opts.decode_height;
		uint32_t pixel_limit = pixels > UINT32_MAX ? UINT32_MAX : (uint32_t)pixels;

	if (pixel_limit < UDL_VENDOR_DEFAULT_PIXEL_LIMIT)
		pixel_limit = UDL_VENDOR_DEFAULT_PIXEL_LIMIT;

	memcpy(runtime->vendor_descriptor, vendor_template, sizeof(vendor_template));
	store_le32_reply(&runtime->vendor_descriptor[19], pixel_limit);
	runtime->vendor_descriptor_len = sizeof(vendor_template);
}

void build_device_descriptor(struct device_runtime *runtime)
{
	const bool super_speed = usb_speed_is_super(runtime->opts.usb_speed);

	memset(&runtime->device_descriptor, 0, sizeof(runtime->device_descriptor));
	runtime->device_descriptor.bLength = USB_DT_DEVICE_SIZE;
	runtime->device_descriptor.bDescriptorType = USB_DT_DEVICE;
	runtime->device_descriptor.bcdUSB = host_to_le16(super_speed ? 0x0300u : 0x0200u);
	runtime->device_descriptor.bDeviceClass = USB_CLASS_PER_INTERFACE;
	runtime->device_descriptor.bDeviceSubClass = 0u;
	runtime->device_descriptor.bDeviceProtocol = 0u;
	runtime->device_descriptor.bMaxPacketSize0 = super_speed ? 9u : 64u;
	runtime->device_descriptor.idVendor = host_to_le16(runtime->opts.vendor_id);
	runtime->device_descriptor.idProduct = host_to_le16(runtime->opts.product_id);
	runtime->device_descriptor.bcdDevice = host_to_le16(UDL_BCD_DEVICE);
	runtime->device_descriptor.iManufacturer = 1u;
	runtime->device_descriptor.iProduct = 2u;
	runtime->device_descriptor.iSerialNumber = 3u;
	runtime->device_descriptor.bNumConfigurations = 1u;
}

void build_device_qualifier(struct device_runtime *runtime)
{
	memset(&runtime->qualifier_descriptor, 0, sizeof(runtime->qualifier_descriptor));
	runtime->qualifier_descriptor.bLength = sizeof(runtime->qualifier_descriptor);
	runtime->qualifier_descriptor.bDescriptorType = USB_DT_DEVICE_QUALIFIER;
	runtime->qualifier_descriptor.bcdUSB = host_to_le16(0x0200u);
	runtime->qualifier_descriptor.bDeviceClass = USB_CLASS_PER_INTERFACE;
	runtime->qualifier_descriptor.bDeviceSubClass = 0u;
	runtime->qualifier_descriptor.bDeviceProtocol = 0u;
	runtime->qualifier_descriptor.bMaxPacketSize0 = 64u;
	runtime->qualifier_descriptor.bNumConfigurations = 1u;
}

void build_bos_descriptor(struct device_runtime *runtime)
{
	memset(&runtime->bos_descriptor, 0, sizeof(runtime->bos_descriptor));
	if (!usb_speed_is_super(runtime->opts.usb_speed))
		return;

	runtime->bos_descriptor.bos.bLength = USB_DT_BOS_SIZE;
	runtime->bos_descriptor.bos.bDescriptorType = USB_DT_BOS;
	runtime->bos_descriptor.bos.wTotalLength = host_to_le16(sizeof(runtime->bos_descriptor));
	runtime->bos_descriptor.bos.bNumDeviceCaps = 1u;

	runtime->bos_descriptor.ss_cap.bLength = USB_DT_USB_SS_CAP_SIZE;
	runtime->bos_descriptor.ss_cap.bDescriptorType = USB_DT_DEVICE_CAPABILITY;
	runtime->bos_descriptor.ss_cap.bDevCapabilityType = USB_SS_CAP_TYPE;
	runtime->bos_descriptor.ss_cap.bmAttributes = 0u;
	runtime->bos_descriptor.ss_cap.wSpeedSupported = host_to_le16(
		USB_FULL_SPEED_OPERATION |
		USB_HIGH_SPEED_OPERATION |
		USB_5GBPS_OPERATION);
	runtime->bos_descriptor.ss_cap.bFunctionalitySupport = 3u;
	runtime->bos_descriptor.ss_cap.bU1devExitLat = 0u;
	runtime->bos_descriptor.ss_cap.bU2DevExitLat = host_to_le16(0u);
}

static size_t build_string_descriptor(const struct device_runtime *runtime,
					      uint8_t *buffer,
					      size_t capacity,
					      uint8_t index)
{
	const char *text = NULL;
	size_t length;
	size_t i;

	if (index == 0u) {
		if (capacity < 4u)
			return 0u;
		buffer[0] = 4u;
		buffer[1] = USB_DT_STRING;
		buffer[2] = (uint8_t)(USB_LANG_EN_US & 0xffu);
		buffer[3] = (uint8_t)(USB_LANG_EN_US >> 8);
		return 4u;
	}

	if (index == UDL_MS_OS_STRING_INDEX)
		return build_ms_os_string_descriptor(buffer, capacity);

	switch (index) {
	case 1u:
		text = runtime->opts.manufacturer_string;
		break;
	case 2u:
		text = runtime->opts.product_string;
		break;
	case 3u:
		text = runtime->opts.serial_string;
		break;
	default:
		return 0u;
	}

	length = strlen(text);
	if (length > 126u || 2u + (length * 2u) > capacity)
		return 0u;

	buffer[0] = (uint8_t)(2u + (length * 2u));
	buffer[1] = USB_DT_STRING;
	for (i = 0u; i < length; ++i) {
		buffer[2u + (i * 2u)] = (uint8_t)text[i];
		buffer[2u + (i * 2u) + 1u] = 0u;
	}

	return (size_t)buffer[0];
}

static size_t build_config_descriptor(const struct device_runtime *runtime,
					      uint8_t *buffer,
					      size_t capacity,
					      bool other_speed,
					      bool super_speed)
{
	struct gadget_config_block block;
	struct gadget_ss_config_block ss_block;
	const uint16_t total_length = (uint16_t)(super_speed ? sizeof(ss_block) : sizeof(block));

	if (capacity < total_length)
		return 0u;

	if (super_speed) {
		memset(&ss_block, 0, sizeof(ss_block));
		ss_block.config.bLength = USB_DT_CONFIG_SIZE;
		ss_block.config.bDescriptorType = USB_DT_CONFIG;
		ss_block.config.wTotalLength = host_to_le16(total_length);
		ss_block.config.bNumInterfaces = 1u;
		ss_block.config.bConfigurationValue = 1u;
		ss_block.config.bmAttributes = USB_CONFIG_ATT_ONE;
		ss_block.config.bMaxPower = 8u;

		ss_block.interface.bLength = USB_DT_INTERFACE_SIZE;
		ss_block.interface.bDescriptorType = USB_DT_INTERFACE;
		ss_block.interface.bNumEndpoints = 3u;
		ss_block.interface.bInterfaceClass = USB_CLASS_VENDOR_SPEC;
		memcpy(ss_block.vendor_descriptor,
		       runtime->vendor_descriptor,
		       sizeof(ss_block.vendor_descriptor));

		ss_block.bulk_out_primary.bLength = USB_DT_ENDPOINT_SIZE;
		ss_block.bulk_out_primary.bDescriptorType = USB_DT_ENDPOINT;
		ss_block.bulk_out_primary.bEndpointAddress = UDL_PRIMARY_BULK_OUT_EP;
		ss_block.bulk_out_primary.bmAttributes = USB_ENDPOINT_XFER_BULK;
		ss_block.bulk_out_primary.wMaxPacketSize = host_to_le16(1024u);

		ss_block.bulk_out_primary_companion.bLength = USB_DT_SS_EP_COMP_SIZE;
		ss_block.bulk_out_primary_companion.bDescriptorType = USB_DT_SS_ENDPOINT_COMP;

		ss_block.interrupt_in.bLength = USB_DT_ENDPOINT_SIZE;
		ss_block.interrupt_in.bDescriptorType = USB_DT_ENDPOINT;
		ss_block.interrupt_in.bEndpointAddress = UDL_INTERRUPT_IN_EP;
		ss_block.interrupt_in.bmAttributes = USB_ENDPOINT_XFER_INT;
		ss_block.interrupt_in.wMaxPacketSize = host_to_le16(UDL_INTERRUPT_PACKET_SIZE);
		ss_block.interrupt_in.bInterval = UDL_INTERRUPT_INTERVAL;

		ss_block.interrupt_in_companion.bLength = USB_DT_SS_EP_COMP_SIZE;
		ss_block.interrupt_in_companion.bDescriptorType = USB_DT_SS_ENDPOINT_COMP;
		ss_block.interrupt_in_companion.wBytesPerInterval = host_to_le16(UDL_INTERRUPT_PACKET_SIZE);

		ss_block.bulk_out_aux.bLength = USB_DT_ENDPOINT_SIZE;
		ss_block.bulk_out_aux.bDescriptorType = USB_DT_ENDPOINT;
		ss_block.bulk_out_aux.bEndpointAddress = UDL_AUX_BULK_OUT_EP;
		ss_block.bulk_out_aux.bmAttributes = USB_ENDPOINT_XFER_BULK;
		ss_block.bulk_out_aux.wMaxPacketSize = host_to_le16(1024u);

		ss_block.bulk_out_aux_companion.bLength = USB_DT_SS_EP_COMP_SIZE;
		ss_block.bulk_out_aux_companion.bDescriptorType = USB_DT_SS_ENDPOINT_COMP;

		memcpy(buffer, &ss_block, sizeof(ss_block));
		return sizeof(ss_block);
	}

	memset(&block, 0, sizeof(block));
	block.config.bLength = USB_DT_CONFIG_SIZE;
	block.config.bDescriptorType = other_speed ? USB_DT_OTHER_SPEED_CONFIG : USB_DT_CONFIG;
	block.config.wTotalLength = host_to_le16(total_length);
	block.config.bNumInterfaces = 1u;
	block.config.bConfigurationValue = 1u;
	block.config.bmAttributes = USB_CONFIG_ATT_ONE;
	block.config.bMaxPower = 125u;

	block.interface.bLength = USB_DT_INTERFACE_SIZE;
	block.interface.bDescriptorType = USB_DT_INTERFACE;
	block.interface.bNumEndpoints = 3u;
	block.interface.bInterfaceClass = USB_CLASS_VENDOR_SPEC;
	memcpy(block.vendor_descriptor,
	       runtime->vendor_descriptor,
	       sizeof(block.vendor_descriptor));

	block.bulk_out_primary.bLength = USB_DT_ENDPOINT_SIZE;
	block.bulk_out_primary.bDescriptorType = USB_DT_ENDPOINT;
	block.bulk_out_primary.bEndpointAddress = UDL_PRIMARY_BULK_OUT_EP;
	block.bulk_out_primary.bmAttributes = USB_ENDPOINT_XFER_BULK;
	block.bulk_out_primary.wMaxPacketSize = host_to_le16(other_speed ? 64u : 512u);

	block.interrupt_in.bLength = USB_DT_ENDPOINT_SIZE;
	block.interrupt_in.bDescriptorType = USB_DT_ENDPOINT;
	block.interrupt_in.bEndpointAddress = UDL_INTERRUPT_IN_EP;
	block.interrupt_in.bmAttributes = USB_ENDPOINT_XFER_INT;
	block.interrupt_in.wMaxPacketSize = host_to_le16(UDL_INTERRUPT_PACKET_SIZE);
	block.interrupt_in.bInterval = UDL_INTERRUPT_INTERVAL;

	block.bulk_out_aux.bLength = USB_DT_ENDPOINT_SIZE;
	block.bulk_out_aux.bDescriptorType = USB_DT_ENDPOINT;
	block.bulk_out_aux.bEndpointAddress = UDL_AUX_BULK_OUT_EP;
	block.bulk_out_aux.bmAttributes = USB_ENDPOINT_XFER_BULK;
	block.bulk_out_aux.wMaxPacketSize = host_to_le16(other_speed ? 64u : 512u);

	memcpy(buffer, &block, sizeof(block));
	return sizeof(block);
}

/* ----------------------------------------------------------------
 * fill_usbip_device
 * ---------------------------------------------------------------- */
void fill_usbip_device(struct device_runtime *runtime)
{
	memset(&runtime->usbip_device, 0, sizeof(runtime->usbip_device));
	(void)snprintf(runtime->usbip_device.path, sizeof(runtime->usbip_device.path), "%s", runtime->opts.device_path);
	(void)snprintf(runtime->usbip_device.busid, sizeof(runtime->usbip_device.busid), "%s", runtime->opts.busid);
	runtime->usbip_device.busnum = 1u;
	runtime->usbip_device.devnum = 2u;
	runtime->usbip_device.speed = (uint32_t)runtime->opts.usb_speed;
	runtime->usbip_device.idVendor = runtime->opts.vendor_id;
	runtime->usbip_device.idProduct = runtime->opts.product_id;
	runtime->usbip_device.bcdDevice = UDL_BCD_DEVICE;
	runtime->usbip_device.bDeviceClass = runtime->device_descriptor.bDeviceClass;
	runtime->usbip_device.bDeviceSubClass = runtime->device_descriptor.bDeviceSubClass;
	runtime->usbip_device.bDeviceProtocol = runtime->device_descriptor.bDeviceProtocol;
	runtime->usbip_device.bConfigurationValue = 1u;
	runtime->usbip_device.bNumConfigurations = 1u;
	runtime->usbip_device.bNumInterfaces = 1u;

	memset(&runtime->usbip_interface, 0, sizeof(runtime->usbip_interface));
	runtime->usbip_interface.bInterfaceClass = USB_CLASS_VENDOR_SPEC;
	runtime->usbip_interface.bInterfaceSubClass = 0u;
	runtime->usbip_interface.bInterfaceProtocol = 0u;
}

/* ----------------------------------------------------------------
 * Control endpoint request handlers
 * ---------------------------------------------------------------- */
static int prepare_standard_request(struct device_runtime *runtime,
				    const struct usb_ctrlrequest *setup,
				    struct control_result *result)
{
	const uint8_t request = setup->bRequest;
	const uint8_t request_type = setup->bRequestType;
	const uint16_t value = (uint16_t)setup->wValue;
	const uint16_t index = (uint16_t)setup->wIndex;
	const uint16_t length = (uint16_t)setup->wLength;
	size_t response_length;

	if ((request_type & USB_TYPE_MASK) != USB_TYPE_STANDARD)
		return 1;

	switch (request) {
	case USB_REQ_GET_DESCRIPTOR:
		switch ((uint8_t)(value >> 8)) {
		case USB_DT_DEVICE:
			memcpy(result->data, &runtime->device_descriptor, sizeof(runtime->device_descriptor));
			result->actual_length = sizeof(runtime->device_descriptor);
			result->action = CONTROL_ACTION_DATA_IN;
			return 0;
		case USB_DT_BOS:
			if (!usb_speed_is_super(runtime->opts.usb_speed))
				return 1;
			memcpy(result->data, &runtime->bos_descriptor, sizeof(runtime->bos_descriptor));
			result->actual_length = sizeof(runtime->bos_descriptor);
			result->action = CONTROL_ACTION_DATA_IN;
			return 0;
		case USB_DT_DEVICE_QUALIFIER:
			memcpy(result->data, &runtime->qualifier_descriptor, sizeof(runtime->qualifier_descriptor));
			result->actual_length = sizeof(runtime->qualifier_descriptor);
			result->action = CONTROL_ACTION_DATA_IN;
			return 0;
		case USB_DT_CONFIG:
			response_length = build_config_descriptor(runtime,
						  result->data,
						  sizeof(result->data),
						  false,
						  usb_speed_is_super(runtime->opts.usb_speed));
			if (response_length == 0u)
				return -1;
			result->actual_length = response_length;
			result->action = CONTROL_ACTION_DATA_IN;
			return 0;
		case USB_DT_OTHER_SPEED_CONFIG:
			response_length = build_config_descriptor(runtime,
						  result->data,
						  sizeof(result->data),
						  true,
						  false);
			if (response_length == 0u)
				return -1;
			result->actual_length = response_length;
			result->action = CONTROL_ACTION_DATA_IN;
			return 0;
		case USB_DT_STRING:
			response_length = build_string_descriptor(runtime,
						 result->data,
						 sizeof(result->data),
						 (uint8_t)(value & 0xffu));
			if (response_length == 0u)
				return 1;
			result->actual_length = response_length;
			result->action = CONTROL_ACTION_DATA_IN;
			return 0;
		case UDL_VENDOR_DESCRIPTOR_TYPE:
			memcpy(result->data, runtime->vendor_descriptor, runtime->vendor_descriptor_len);
			result->actual_length = runtime->vendor_descriptor_len;
			result->action = CONTROL_ACTION_DATA_IN;
			return 0;
		default:
			return 1;
		}
	case USB_REQ_GET_STATUS:
		result->data[0] = 0u;
		result->data[1] = 0u;
		result->actual_length = 2u;
		result->action = CONTROL_ACTION_DATA_IN;
		return 0;
	case USB_REQ_GET_CONFIGURATION:
		result->data[0] = runtime->current_configuration;
		result->actual_length = 1u;
		result->action = CONTROL_ACTION_DATA_IN;
		return 0;
	case USB_REQ_GET_INTERFACE:
		(void)index;
		result->data[0] = 0u;
		result->actual_length = 1u;
		result->action = CONTROL_ACTION_DATA_IN;
		return 0;
	case USB_REQ_SET_CONFIGURATION:
		runtime->current_configuration = (uint8_t)(value & 0xffu);
		result->action = CONTROL_ACTION_ACK;
		result->actual_length = 0u;
		return 0;
	case USB_REQ_SET_INTERFACE:
		(void)index;
		result->action = CONTROL_ACTION_ACK;
		result->actual_length = 0u;
		return 0;
	default:
		break;
	}

	if (request_type == 0x00u && length == 0u) {
		result->action = CONTROL_ACTION_ACK;
		result->actual_length = 0u;
		return 0;
	}

	return 1;
}

static int prepare_vendor_request(struct device_runtime *runtime,
				  const struct usb_ctrlrequest *setup,
				  const uint8_t *payload,
				  size_t payload_length,
				  struct control_result *result)
{
	const uint8_t request_type = setup->bRequestType;
	const uint8_t request = setup->bRequest;
	const uint16_t value = (uint16_t)setup->wValue;
	const uint16_t index = (uint16_t)setup->wIndex;
	const uint16_t length = (uint16_t)setup->wLength;

	if (request_type == (USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE) &&
	    request == UDL_MS_OS_VENDOR_CODE &&
	    value == 0u) {
		size_t response_length = 0u;

		switch (index) {
		case UDL_MS_OS_EXT_COMPAT_INDEX:
			response_length = build_ms_os_compat_id_descriptor(result->data, sizeof(result->data));
			break;
		case UDL_MS_OS_EXT_PROPERTIES_INDEX:
			response_length = build_ms_os_ext_properties_descriptor(result->data, sizeof(result->data));
			break;
		default:
			return 1;
		}

		if (response_length == 0u)
			return -1;
		result->actual_length = response_length;
		result->action = CONTROL_ACTION_DATA_IN;
		if (runtime->opts.verbose) {
			fprintf(stderr,
				"Replied to Microsoft OS descriptor request wIndex=0x%04x len=%zu\n",
				(unsigned int)index,
				response_length);
		}
		return 0;
	}

	if (request_type == (USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE) &&
	    request == UDL_VENDOR_REQUEST_MEMORY_POKE &&
	    value == 0u) {
		if (length != 1u || payload_length != 1u) {
			fprintf(stderr, "Unexpected control RAM poke payload length %zu\n", payload_length);
			return -1;
		}
		udl_control_ram_write_byte(runtime, index, payload[0]);
		if (runtime->opts.verbose) {
			fprintf(stderr,
				"Control RAM poke: addr=0x%04x value=0x%02x\n",
				(unsigned int)index,
				(unsigned int)payload[0]);
		}
		result->action = CONTROL_ACTION_ACK;
		result->actual_length = payload_length;
		return 0;
	}

	if (request_type == (USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE) &&
	    request == UDL_VENDOR_REQUEST_MEMORY_PEEK &&
	    value == 0u &&
	    length == 1u) {
		const uint8_t peek_value = udl_control_ram_read_byte(runtime, index);

		result->data[0] = peek_value;
		result->actual_length = 1u;
		result->action = CONTROL_ACTION_DATA_IN;
		if (runtime->opts.verbose) {
			fprintf(stderr,
				"Control RAM peek: addr=0x%04x -> 0x%02x\n",
				(unsigned int)index,
				(unsigned int)peek_value);
		}
		return 0;
	}

	if (request_type == (USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE) &&
	    request == UDL_VENDOR_REQUEST_SET_KEY) {
		const bool same_key = runtime->key_loaded &&
			memcmp(runtime->last_key, payload, sizeof(runtime->last_key)) == 0;
		const bool preserve_init_state = same_key &&
			(runtime->huffman_table_loaded ||
			 runtime->windows_blankmode_on ||
			 runtime->windows_vidreg_unlock ||
			 runtime->windows_display_ready);
		const bool is_nullkey1 = memcmp(payload, k_displaylink_nullkey1, sizeof(k_displaylink_nullkey1)) == 0;
		const bool is_nullkey2 = memcmp(payload, k_displaylink_nullkey2, sizeof(k_displaylink_nullkey2)) == 0;

		if (payload_length != sizeof(k_displaylink_nullkey1) || length != sizeof(k_displaylink_nullkey1)) {
			fprintf(stderr, "Unexpected encryption key payload length %zu\n", payload_length);
			return -1;
		}
		memcpy(runtime->last_key, payload, sizeof(runtime->last_key));
		runtime->key_loaded = true;
		if (is_nullkey1 || is_nullkey2) {
			/* The known null keys disable on-wire encryption. */
			runtime->decrypt_enabled = false;
			runtime->decrypt_offset = 0u;
		} else {
			udl_crypt_init_tables();
			runtime->decrypt_offset = g_udl_crypt_map[udl_crypt_crc12(payload, sizeof(runtime->last_key)) & 0x0fffu];
			runtime->decrypt_enabled = true;
		}
		if (!preserve_init_state) {
			runtime->vendor_14_seen = false;
			runtime->huffman_table_loaded = false;
			runtime->windows_blankmode_on = false;
			runtime->windows_vidreg_unlock = false;
			runtime->windows_display_ready = false;
		} else if (runtime->opts.verbose) {
			fprintf(stderr,
				"Preserving Windows init state across repeated key load\n");
		}
		if (runtime->opts.verbose) {
			if (is_nullkey1) {
				fprintf(stderr, "Loaded known DisplayLink null key #1 (stream decrypt disabled)\n");
			} else if (is_nullkey2) {
				fprintf(stderr, "Loaded known DisplayLink null key #2 (stream decrypt disabled)\n");
			} else {
				fprintf(stderr,
					"Received non-standard encryption key payload (stream decrypt enabled at offset 0x%03x)\n",
					(unsigned int)runtime->decrypt_offset);
				log_hex_preview("Encryption key payload", payload, payload_length);
			}
		}
		result->action = CONTROL_ACTION_ACK;
		result->actual_length = payload_length;
		return 0;
	}

	if (request_type == (USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE) &&
	    request == UDL_VENDOR_REQUEST_UNKNOWN_14 &&
	    value == 0u &&
	    index == 0u &&
	    length == 0u &&
	    payload_length == 0u) {
		runtime->vendor_14_seen = true;
		if (runtime->opts.verbose) {
			fprintf(stderr,
				"Accepted vendor 0x14 init pulse: key_loaded=%s huffman_loaded=%s ready=%s\n",
				runtime->key_loaded ? "yes" : "no",
				runtime->huffman_table_loaded ? "yes" : "no",
				runtime->windows_display_ready ? "yes" : "no");
		}
		result->action = CONTROL_ACTION_ACK;
		result->actual_length = 0u;
		return 0;
	}

	if (request_type == (USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE) &&
	    request == UDL_VENDOR_REQUEST_WINDOWS_QUERY_05 &&
	    value == 0u &&
	    index == 0x0002u &&
	    length == sizeof(uint32_t)) {
		const uint32_t reply_value = windows_query_05_reply_value(runtime);

		store_le32_reply(result->data, reply_value);
		result->actual_length = sizeof(uint32_t);
		result->action = CONTROL_ACTION_DATA_IN;
		if (runtime->opts.verbose) {
			fprintf(stderr,
				"Windows query 0x05 reply: 0x%08x (key_loaded=%s huffman_loaded=%s ready=%s)\n",
				(unsigned int)reply_value,
				runtime->key_loaded ? "yes" : "no",
				runtime->huffman_table_loaded ? "yes" : "no",
				runtime->windows_display_ready ? "yes" : "no");
		}
		return 0;
	}

	if (request_type == (USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE) &&
	    request == UDL_VENDOR_REQUEST_WINDOWS_PROBE &&
	    value == 0u &&
	    index == 0u &&
	    length == sizeof(k_windows_probe_reply)) {
		const uint32_t reply_value = windows_probe_reply_value(runtime);

		store_le32_reply(result->data, reply_value);
		result->actual_length = sizeof(k_windows_probe_reply);
		result->action = CONTROL_ACTION_DATA_IN;
		if (runtime->opts.verbose) {
			fprintf(stderr,
				"Windows probe reply: 0x%08x (key_loaded=%s huffman_loaded=%s ready=%s)\n",
				(unsigned int)reply_value,
				runtime->key_loaded ? "yes" : "no",
				runtime->huffman_table_loaded ? "yes" : "no",
				runtime->windows_display_ready ? "yes" : "no");
		}
		return 0;
	}

	if (request_type == (USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE) &&
	    request == UDL_VENDOR_REQUEST_WINDOWS_STATUS &&
	    value == 0xffffu &&
	    index == 0xffffu &&
	    length == sizeof(k_windows_status_reply)) {
		const uint32_t reply_value = windows_status_reply_value(runtime);

		store_le32_reply(result->data, reply_value);
		result->actual_length = sizeof(k_windows_status_reply);
		result->action = CONTROL_ACTION_DATA_IN;
		if (runtime->opts.verbose) {
			fprintf(stderr,
				"Windows status reply: 0x%08x (key_loaded=%s huffman_loaded=%s ready=%s)\n",
				(unsigned int)reply_value,
				runtime->key_loaded ? "yes" : "no",
				runtime->huffman_table_loaded ? "yes" : "no",
				runtime->windows_display_ready ? "yes" : "no");
		}
		return 0;
	}

	if (request_type == (USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE) &&
	    request == UDL_VENDOR_REQUEST_EDID &&
	    (index & 0x00ffu) == (UDL_EDID_INDEX & 0x00ffu)) {
		const uint16_t byte_index = (uint16_t)(value >> 8);
		size_t reply_length = length;
		size_t copy_length = 0u;

		if (reply_length > sizeof(result->data)) {
			reply_length = sizeof(result->data);
		}

		/*
		 * The udl wire protocol prepends a 0x00 status byte before the
		 * EDID payload (kernel udl driver reads reply[1] for each byte).
		 * Enforce at least 2 bytes so the status byte is always present.
		 */
		if (reply_length < 2u)
			reply_length = 2u;
		if (reply_length > sizeof(result->data))
			reply_length = sizeof(result->data);

		memset(result->data, 0, reply_length);
		result->data[0] = 0x00; /* status: success */
		if ((size_t)byte_index < sizeof(runtime->edid)) {
			copy_length = sizeof(runtime->edid) - (size_t)byte_index;
			if (copy_length > reply_length - 1u) {
				copy_length = reply_length - 1u;
			}
		}
		if (copy_length > 0u) {
			memcpy(result->data + 1u,
			       runtime->edid + byte_index,
			       copy_length);
		}
		result->actual_length = reply_length;
		result->action = CONTROL_ACTION_DATA_IN;
		if (runtime->opts.verbose) {
			fprintf(stderr,
				"Replied to EDID request offset=0x%02x wIndex=0x%04x len=%zu copy=%zu\n",
				(unsigned int)byte_index,
				(unsigned int)index,
				reply_length,
				copy_length);
		}
		return 0;
	}

	return 1;
}

static int handle_control_submit(struct device_runtime *runtime,
				 const struct usbip_header *request,
				 const uint8_t *payload,
				 size_t payload_length,
				 struct control_result *result)
{
	const struct usb_ctrlrequest *setup = (const struct usb_ctrlrequest *)request->u.cmd_submit.setup;
	int rc;

	memset(result, 0, sizeof(*result));
	result->action = CONTROL_ACTION_STALL;
	result->status = -EPIPE;

	if (runtime->opts.verbose) {
		fprintf(stderr,
			"USB/IP setup: bmRequestType=0x%02x bRequest=0x%02x wValue=0x%04x wIndex=0x%04x wLength=%u\n",
			setup->bRequestType,
			setup->bRequest,
			(unsigned int)((uint16_t)setup->wValue),
			(unsigned int)((uint16_t)setup->wIndex),
			(unsigned int)((uint16_t)setup->wLength));
	}

	rc = prepare_standard_request(runtime, setup, result);
	if (rc < 0)
		return -1;
	if (rc == 0) {
		result->status = (result->action == CONTROL_ACTION_STALL) ? -EPIPE : 0;
		return 0;
	}

	rc = prepare_vendor_request(runtime, setup, payload, payload_length, result);
	if (rc < 0)
		return -1;
	if (rc == 0) {
		result->status = 0;
		return 0;
	}

	if (runtime->opts.verbose)
		fprintf(stderr, "Unhandled setup request, returning stall status\n");
	result->action = CONTROL_ACTION_STALL;
	result->status = -EPIPE;
	return 0;
}

static int send_submit_response(int fd,
				const struct usbip_header *request,
				int status,
				size_t actual_length,
				const uint8_t *payload)
{
	struct usbip_header response;
	struct iovec iov[2];
	int iovcnt = 1;
	size_t total_length = sizeof(response);

	memset(&response, 0, sizeof(response));
	response.base.command = USBIP_RET_SUBMIT;
	response.base.seqnum = request->base.seqnum;
	response.u.ret_submit.status = status;
	response.u.ret_submit.actual_length = (int32_t)actual_length;
	response.u.ret_submit.start_frame = 0;
	response.u.ret_submit.number_of_packets = 0;
	response.u.ret_submit.error_count = 0;
	usbip_header_correct_endian(&response, 1);
	iov[0].iov_base = &response;
	iov[0].iov_len = sizeof(response);
	if (payload && actual_length > 0u) {
		iov[1].iov_base = (void *)payload;
		iov[1].iov_len = actual_length;
		iovcnt = 2;
		total_length += actual_length;
	}
	return send_iov_all(fd, iov, iovcnt) == (ssize_t)total_length ? 0 : -1;
}

static int send_unlink_response(int fd, uint32_t seqnum, int status)
{
	struct usbip_header response;

	memset(&response, 0, sizeof(response));
	response.base.command = USBIP_RET_UNLINK;
	response.base.seqnum = seqnum;
	response.u.ret_unlink.status = status;
	usbip_header_correct_endian(&response, 1);
	return send_all(fd, &response, sizeof(response)) == (ssize_t)sizeof(response) ? 0 : -1;
}

int handle_submit(struct device_runtime *runtime, int fd, struct usbip_header *request)
{
	const int32_t requested_length = request->u.cmd_submit.transfer_buffer_length;
	uint8_t *payload = NULL;
	struct control_result result;
	size_t payload_length = 0u;
	bool payload_is_reuse_buf = false;
	int rc;

#define FREE_PAYLOAD() do { if (payload && !payload_is_reuse_buf) free(payload); payload = NULL; } while (0)

	if (requested_length < 0 || requested_length > (int32_t)MAX_TRANSFER_BUFFER_SIZE) {
		fprintf(stderr, "Unsupported transfer length %d\n", requested_length);
		return -1;
	}
	payload_length = (size_t)requested_length;
	if (request->base.direction == USBIP_DIR_OUT && payload_length > 0u) {
		/*
		 * Control transfers (ep 0) are small and synchronous — use the
		 * pre-allocated reuse buffer to avoid a malloc/free per packet.
		 * Bulk OUT transfers are handed to the decode queue (owned), so
		 * they still need a fresh allocation that the queue can free.
		 */
		if (request->base.ep == 0u && payload_length <= runtime->bulk_recv_buf_size) {
			payload = runtime->bulk_recv_buf;
			payload_is_reuse_buf = true;
		} else {
			payload = malloc(payload_length);
			if (!payload) {
				perror("malloc transfer payload");
				return -1;
			}
		}
		if (recv_all(fd, payload, payload_length) != (ssize_t)payload_length)
			goto fail;
	}

	if (request->base.ep == 0u) {
		usbip_cadence_note_control_submit(runtime, request, payload_length);
		if (handle_control_submit(runtime, request, payload, payload_length, &result) != 0)
			goto fail;
		if (result.action == CONTROL_ACTION_DATA_IN && result.actual_length > (size_t)(uint16_t)((struct usb_ctrlrequest *)request->u.cmd_submit.setup)->wLength)
			result.actual_length = (uint16_t)((struct usb_ctrlrequest *)request->u.cmd_submit.setup)->wLength;
		if (result.action == CONTROL_ACTION_DATA_IN)
			usbip_cadence_note_control_reply(runtime, result.actual_length);
		if (result.status != 0)
			usbip_cadence_note_error(runtime);
		rc = send_submit_response(fd,
					 request,
					 result.status,
					 result.action == CONTROL_ACTION_DATA_IN ? result.actual_length : (result.status == 0 ? payload_length : 0u),
					 result.action == CONTROL_ACTION_DATA_IN ? result.data : NULL);
		usbip_cadence_maybe_log(runtime, false);
		FREE_PAYLOAD();
		return rc;
	}

	if ((request->base.ep == UDL_PRIMARY_BULK_OUT_EP || request->base.ep == UDL_AUX_BULK_OUT_EP) &&
	    request->base.direction == USBIP_DIR_OUT) {
		usbip_cadence_note_bulk_out(runtime, payload_length);
		if (runtime->opts.verbose) {
			const uint64_t payload_signature = udl_payload_signature64(payload, payload_length);

			runtime->bulk_submit_count += 1u;
			if (runtime->bulk_submit_count <= 8u ||
			    payload_length <= 64u ||
			    payload_length >= 4096u ||
			    (runtime->bulk_submit_count % 64u) == 0u) {
				fprintf(stderr,
					"USB/IP bulk OUT ep=0x%02x #%llu len=%zu sig=0x%016llx key_loaded=%s huffman_loaded=%s ready=%s\n",
					(unsigned int)request->base.ep,
					(unsigned long long)runtime->bulk_submit_count,
					payload_length,
					(unsigned long long)payload_signature,
					runtime->key_loaded ? "yes" : "no",
					runtime->huffman_table_loaded ? "yes" : "no",
					runtime->windows_display_ready ? "yes" : "no");
				if (runtime->bulk_submit_count <= 4u || payload_length <= 64u)
					log_hex_preview("Bulk OUT preview", payload, payload_length);
			}
			maybe_log_first_e0_command(runtime, payload, payload_length);
		}
		udl_decrypt_bulk_payload(runtime, payload, payload_length);
		update_windows_bulk_state(runtime, payload, payload_length);
		if (runtime->opts.decode_stream) {
			if (udl_runtime_enqueue_bulk_owned(&runtime->udl, payload, payload_length) != 0) {
				usbip_cadence_note_error(runtime);
				rc = send_submit_response(fd, request, -EIO, 0u, NULL);
			} else {
				rc = send_submit_response(fd, request, 0, payload_length, NULL);
			}
			payload = NULL;
		} else {
			rc = send_submit_response(fd, request, 0, payload_length, NULL);
		}
		usbip_cadence_maybe_log(runtime, false);
		FREE_PAYLOAD();
		return rc;
	}

	if (request->base.ep == (UDL_INTERRUPT_IN_EP & 0x7fu) && request->base.direction == USBIP_DIR_IN) {
		uint8_t interrupt_payload[UDL_INTERRUPT_PACKET_SIZE] = {0};
		size_t actual_length = payload_length;

		if (actual_length > sizeof(interrupt_payload))
			actual_length = sizeof(interrupt_payload);
		if (runtime->opts.verbose && actual_length > 0u) {
			fprintf(stderr,
				"USB/IP interrupt IN ep=0x%02x len=%zu returning zeroed status packet\n",
				(unsigned int)UDL_INTERRUPT_IN_EP,
				actual_length);
		}
		usbip_cadence_note_interrupt_in(runtime, actual_length);
		FREE_PAYLOAD();
		rc = send_submit_response(fd, request, 0, actual_length, interrupt_payload);
		usbip_cadence_maybe_log(runtime, false);
		return rc;
	}

	if (runtime->opts.verbose) {
		fprintf(stderr,
			"Unsupported USB/IP submit: command=%u ep=%u dir=%u len=%zu\n",
			request->base.command,
			request->base.ep,
			request->base.direction,
			payload_length);
	}
	usbip_cadence_note_error(runtime);
	rc = send_submit_response(fd, request, -EPIPE, 0u, NULL);
	usbip_cadence_maybe_log(runtime, false);
	FREE_PAYLOAD();
	return rc;

fail:
	usbip_cadence_note_error(runtime);
	FREE_PAYLOAD();
	return -1;
}

/* ----------------------------------------------------------------
 * Import session loop
 * ---------------------------------------------------------------- */
int run_import_session(struct device_runtime *runtime, int fd)
{
	int session_status = 0;
	const bool ack_batch = usbip_ack_batch_enabled();
	unsigned acks_since_flush = 0u;

	usbip_cadence_reset(runtime, monotonic_time_ns());

	if (ack_batch)
		usbip_set_cork(fd, 1);

	while (!stop_requested) {
		struct usbip_header request;
		ssize_t rc;

		/*
		 * Flush accumulated RET_SUBMIT replies before we block: either when
		 * nothing more is buffered (the host paused), or once we have held a
		 * capped number so its URB pool cannot starve mid-stream.
		 */
		if (ack_batch &&
		    (acks_since_flush >= USBIP_ACK_BATCH_MAX || !usbip_input_pending(fd))) {
			usbip_flush_cork(fd);
			acks_since_flush = 0u;
		}

		memset(&request, 0, sizeof(request));
		rc = recv_all(fd, &request, sizeof(request));
		if (rc != (ssize_t)sizeof(request)) {
			session_status = -1;
			break;
		}
		usbip_header_correct_endian(&request, 0);

		switch (request.base.command) {
		case USBIP_CMD_SUBMIT:
			if (handle_submit(runtime, fd, &request) != 0) {
				session_status = -1;
				goto out;
			}
			acks_since_flush++;
			break;
		case USBIP_CMD_UNLINK:
			usbip_cadence_note_unlink(runtime);
			if (send_unlink_response(fd, request.base.seqnum, 0) != 0) {
				session_status = -1;
				goto out;
			}
			usbip_cadence_maybe_log(runtime, false);
			acks_since_flush++;
			break;
		default:
			fprintf(stderr, "Unsupported USB/IP command %u\n", request.base.command);
			session_status = -1;
			goto out;
		}
	}

out:
	/* Push any corked replies before the socket is torn down. */
	if (ack_batch)
		usbip_flush_cork(fd);
	usbip_cadence_maybe_log(runtime, true);
	return session_status;
}
