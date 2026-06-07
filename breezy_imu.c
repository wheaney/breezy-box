#define _POSIX_C_SOURCE 200809L

#include "breezy_imu.h"

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>
#include <sys/inotify.h>
#include <sys/select.h>
#include <time.h>
#include <unistd.h>

/* ----------------------------------------------------------------
 * Binary layout of /dev/shm/breezy_desktop_imu
 *
 * Each field has three constants: _O (byte offset), _S (element size), _C (count).
 * Each _O is the previous field's _O + _S * _C, chained through the enum so the
 * compiler and IDE both see real integer constants (not macro text).
 * Mirrors the DataView namespace in the KWin plugin exactly.
 * ---------------------------------------------------------------- */

enum imu_layout {
    DV_UINT8_SIZE = 1,
    DV_BOOL_SIZE  = 1,
    DV_UINT_SIZE  = 4,
    DV_FLOAT_SIZE = 4,

    DV_VERSION_O = 0,
    DV_VERSION_S = DV_UINT8_SIZE,
    DV_VERSION_C = 1,

    DV_ENABLED_O = DV_VERSION_O        + DV_VERSION_S        * DV_VERSION_C,
    DV_ENABLED_S = DV_BOOL_SIZE,
    DV_ENABLED_C = 1,

    DV_LOOK_AHEAD_CFG_O = DV_ENABLED_O        + DV_ENABLED_S        * DV_ENABLED_C,
    DV_LOOK_AHEAD_CFG_S = DV_FLOAT_SIZE,
    DV_LOOK_AHEAD_CFG_C = 4,

    DV_DISPLAY_RES_O = DV_LOOK_AHEAD_CFG_O + DV_LOOK_AHEAD_CFG_S * DV_LOOK_AHEAD_CFG_C,
    DV_DISPLAY_RES_S = DV_UINT_SIZE,
    DV_DISPLAY_RES_C = 2,

    DV_DISPLAY_FOV_O = DV_DISPLAY_RES_O    + DV_DISPLAY_RES_S    * DV_DISPLAY_RES_C,
    DV_DISPLAY_FOV_S = DV_FLOAT_SIZE,
    DV_DISPLAY_FOV_C = 1,

    DV_LENS_DIST_RATIO_O = DV_DISPLAY_FOV_O    + DV_DISPLAY_FOV_S    * DV_DISPLAY_FOV_C,
    DV_LENS_DIST_RATIO_S = DV_FLOAT_SIZE,
    DV_LENS_DIST_RATIO_C = 1,

    DV_SBS_ENABLED_O = DV_LENS_DIST_RATIO_O + DV_LENS_DIST_RATIO_S * DV_LENS_DIST_RATIO_C,
    DV_SBS_ENABLED_S = DV_BOOL_SIZE,
    DV_SBS_ENABLED_C = 1,

    DV_CUSTOM_BANNER_O = DV_SBS_ENABLED_O   + DV_SBS_ENABLED_S   * DV_SBS_ENABLED_C,
    DV_CUSTOM_BANNER_S = DV_BOOL_SIZE,
    DV_CUSTOM_BANNER_C = 1,

    DV_SF_ENABLED_O = DV_CUSTOM_BANNER_O  + DV_CUSTOM_BANNER_S  * DV_CUSTOM_BANNER_C,
    DV_SF_ENABLED_S = DV_BOOL_SIZE,
    DV_SF_ENABLED_C = 1,

    DV_SF_ORIGIN_DATA_O = DV_SF_ENABLED_O    + DV_SF_ENABLED_S    * DV_SF_ENABLED_C,
    DV_SF_ORIGIN_DATA_S = DV_FLOAT_SIZE,
    DV_SF_ORIGIN_DATA_C = 16,

    DV_POSE_POSITION_O = DV_SF_ORIGIN_DATA_O + DV_SF_ORIGIN_DATA_S * DV_SF_ORIGIN_DATA_C,
    DV_POSE_POSITION_S = DV_FLOAT_SIZE,
    DV_POSE_POSITION_C = 3,

    DV_POSE_DATE_MS_O = DV_POSE_POSITION_O  + DV_POSE_POSITION_S  * DV_POSE_POSITION_C,
    DV_POSE_DATE_MS_S = DV_UINT_SIZE,
    DV_POSE_DATE_MS_C = 2,   /* two uint32s = LE uint64 */

    DV_POSE_ORIENT_O = DV_POSE_DATE_MS_O   + DV_POSE_DATE_MS_S   * DV_POSE_DATE_MS_C,
    DV_POSE_ORIENT_S = DV_FLOAT_SIZE,
    DV_POSE_ORIENT_C = 16,   /* 4 rows × 4 floats */

    DV_PARITY_O = DV_POSE_ORIENT_O    + DV_POSE_ORIENT_S    * DV_POSE_ORIENT_C,
    DV_PARITY_S = DV_UINT8_SIZE,
    DV_PARITY_C = 1,

    IMU_BUFFER_SIZE = DV_PARITY_O + DV_PARITY_S * DV_PARITY_C
};

#define IMU_SHM_DIR  "/dev/shm"
#define IMU_SHM_PATH "/dev/shm/breezy_desktop_imu"

/* Pose is considered stale after this many milliseconds. */
#define IMU_STALE_MS 5000

/* ----------------------------------------------------------------
 * Internal state
 * ---------------------------------------------------------------- */

static pthread_t        g_thread;
static atomic_bool      g_stop;
static pthread_mutex_t  g_pose_mutex = PTHREAD_MUTEX_INITIALIZER;
static struct breezy_imu_pose         g_pose;
static struct breezy_imu_device_config g_config;
static bool             g_pose_valid   = false;
static bool             g_config_valid = false;
static bool             g_initialized  = false;

/* ----------------------------------------------------------------
 * Helpers
 * ---------------------------------------------------------------- */

static uint64_t realtime_ms(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	return (uint64_t)ts.tv_sec * 1000u + (uint64_t)(ts.tv_nsec / 1000000L);
}

static bool check_parity(const uint8_t *buf)
{
	uint8_t parity = 0;
	int i;

	for (i = 0; i < DV_POSE_DATE_MS_S * DV_POSE_DATE_MS_C; i++)
		parity ^= buf[DV_POSE_DATE_MS_O + i];

	for (i = 0; i < DV_POSE_ORIENT_S * DV_POSE_ORIENT_C; i++)
		parity ^= buf[DV_POSE_ORIENT_O + i];

	return parity == buf[DV_PARITY_O];
}

static void parse_and_store(void)
{
	uint8_t buf[IMU_BUFFER_SIZE];
	int fd;
	ssize_t n;

	fd = open(IMU_SHM_PATH, O_RDONLY);
	if (fd < 0)
		return;

	n = 0;
	while (n < (ssize_t)IMU_BUFFER_SIZE) {
		ssize_t r = read(fd, buf + n, (size_t)(IMU_BUFFER_SIZE - n));
		if (r <= 0)
			break;
		n += r;
	}
	close(fd);

	if (n != (ssize_t)IMU_BUFFER_SIZE)
		return;
	if (!check_parity(buf))
		return;

	/* Little-endian uint64 from two uint32 slots */
	uint64_t ts_ms = 0;
	memcpy(&ts_ms, buf + DV_POSE_DATE_MS_O, sizeof(ts_ms));
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	ts_ms = __builtin_bswap64(ts_ms);
#endif

	const uint64_t now_ms = realtime_ms();
	if (ts_ms != 0 && now_ms > ts_ms && (now_ms - ts_ms) > IMU_STALE_MS)
		return;

	struct breezy_imu_pose pose;

	pose.enabled = (buf[DV_ENABLED_O] != 0);
	pose.timestamp_ms = ts_ms;

	/* position: NWU → EUS: (-y, z, -x) */
	float pos[3];
	memcpy(pos, buf + DV_POSE_POSITION_O, sizeof(pos));
	pose.pos_x = -pos[1];
	pose.pos_y =  pos[2];
	pose.pos_z = -pos[0];

	/*
	 * POSE_ORIENT: 4 rows × 4 floats.
	 * Row 0 (T0): [x_nwu, y_nwu, z_nwu, w_nwu]
	 * NWU → EUS: w unchanged, x = -y_nwu, y = z_nwu, z = -x_nwu
	 * Row 3 is NOT a quaternion; it holds per-sample timestamps.
	 */
	float q[4];
	memcpy(q, buf + DV_POSE_ORIENT_O, sizeof(q));
	pose.quat_w =  q[3];
	pose.quat_x = -q[1];
	pose.quat_y =  q[2];
	pose.quat_z = -q[0];

	/* Device config — relatively static, parsed alongside every pose update. */
	struct breezy_imu_device_config config;
	config.version = buf[DV_VERSION_O];
	config.enabled = (buf[DV_ENABLED_O] != 0);

	uint32_t res[2];
	memcpy(res, buf + DV_DISPLAY_RES_O, sizeof(res));
	config.display_width  = res[0];
	config.display_height = res[1];

	memcpy(&config.diagonal_fov_deg,    buf + DV_DISPLAY_FOV_O,      sizeof(float));
	memcpy(&config.lens_distance_ratio, buf + DV_LENS_DIST_RATIO_O,  sizeof(float));

	pthread_mutex_lock(&g_pose_mutex);
	g_pose   = pose;
	g_config = config;
	g_pose_valid   = true;
	g_config_valid = true;
	pthread_mutex_unlock(&g_pose_mutex);
}

/* ----------------------------------------------------------------
 * Watcher thread
 * Watches /dev/shm for file creation and the file itself for writes.
 * ---------------------------------------------------------------- */

static void *imu_thread(void *arg)
{
	(void)arg;

	int ifd = inotify_init1(IN_NONBLOCK);
	if (ifd < 0) {
		perror("breezy_imu: inotify_init1");
		return NULL;
	}

	int dir_wd  = inotify_add_watch(ifd, IMU_SHM_DIR,
	                                IN_CREATE | IN_MOVED_TO);
	int file_wd = inotify_add_watch(ifd, IMU_SHM_PATH,
	                                IN_MODIFY | IN_CLOSE_WRITE);

	/* Parse once immediately in case the file already exists. */
	if (file_wd >= 0)
		parse_and_store();

	char ev_buf[sizeof(struct inotify_event) + NAME_MAX + 1]
		__attribute__((aligned(__alignof__(struct inotify_event))));

	while (!atomic_load_explicit(&g_stop, memory_order_relaxed)) {
		fd_set rfds;
		struct timeval tv = {0, 100000}; /* 100 ms wake to check stop */

		FD_ZERO(&rfds);
		FD_SET(ifd, &rfds);
		int ret = select(ifd + 1, &rfds, NULL, NULL, &tv);
		if (ret < 0) {
			if (errno == EINTR)
				continue;
			break;
		}
		if (ret == 0)
			continue;

		ssize_t len = read(ifd, ev_buf, sizeof(ev_buf));
		if (len <= 0)
			continue;

		const struct inotify_event *ev;
		for (char *p = ev_buf; p < ev_buf + len;
		     p += sizeof(struct inotify_event) + ev->len) {
			ev = (const struct inotify_event *)p;

			if (ev->wd == dir_wd) {
				/* File appeared in the directory. */
				if (ev->len > 0 &&
				    strcmp(ev->name, "breezy_desktop_imu") == 0) {
					if (file_wd >= 0)
						inotify_rm_watch(ifd, file_wd);
					file_wd = inotify_add_watch(
						ifd, IMU_SHM_PATH,
						IN_MODIFY | IN_CLOSE_WRITE);
					parse_and_store();
				}
			} else if (ev->wd == file_wd) {
				parse_and_store();
			}
		}
	}

	if (file_wd >= 0)
		inotify_rm_watch(ifd, file_wd);
	if (dir_wd >= 0)
		inotify_rm_watch(ifd, dir_wd);
	close(ifd);
	return NULL;
}

/* ----------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------- */

int breezy_imu_init(void)
{
	if (g_initialized)
		return 0;

	atomic_store(&g_stop, false);
	memset(&g_pose,   0, sizeof(g_pose));
	memset(&g_config, 0, sizeof(g_config));
	g_pose_valid   = false;
	g_config_valid = false;

	if (pthread_create(&g_thread, NULL, imu_thread, NULL) != 0) {
		perror("breezy_imu: pthread_create");
		return -1;
	}

	g_initialized = true;
	return 0;
}

void breezy_imu_destroy(void)
{
	if (!g_initialized)
		return;
	atomic_store(&g_stop, true);
	pthread_join(g_thread, NULL);
	g_initialized = false;
}

bool breezy_imu_try_get_pose(struct breezy_imu_pose *out)
{
	pthread_mutex_lock(&g_pose_mutex);
	bool valid = g_pose_valid;
	if (valid)
		*out = g_pose;
	pthread_mutex_unlock(&g_pose_mutex);

	if (!valid)
		return false;

	/* Reject stale poses: driver timestamp is wall-clock ms. */
	if (out->timestamp_ms != 0) {
		uint64_t now = realtime_ms();
		if (now > out->timestamp_ms &&
		    (now - out->timestamp_ms) > IMU_STALE_MS)
			return false;
	}

	return true;
}

bool breezy_imu_get_config(struct breezy_imu_device_config *out)
{
	pthread_mutex_lock(&g_pose_mutex);
	bool valid = g_config_valid;
	if (valid)
		*out = g_config;
	pthread_mutex_unlock(&g_pose_mutex);
	return valid;
}
