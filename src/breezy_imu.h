#pragma once

#include <stdbool.h>
#include <stdint.h>

#define BREEZY_IMU_EXPECTED_VERSION 5

/*
 * Pose snapshot in EUS coordinates (East-Up-South).
 * Quaternion is scalar-first: w, x, y, z.
 * Converted from the NWU frame written by the XR driver.
 *
 * Smooth-follow fields mirror XRLinuxDriver's smooth_follow output, exposed the
 * same way the GNOME/KWin plugins consume them:
 *   sf_enabled            – driver's smooth-follow toggle state.
 *   sf_quat_{w,x,y,z}     – the "origin" orientation the displays are anchored
 *                           to (EUS).  While smooth follow is engaged the driver
 *                           slerps this toward the head pose, and the renderer
 *                           drives the camera from it so the displays follow.
 *                           The driver clears sf_enabled ~immediately on toggle
 *                           off but keeps slerping the origin back for ~1s, so
 *                           callers must keep using this data during that window.
 */
struct breezy_imu_pose {
	float quat_w, quat_x, quat_y, quat_z;
	float pos_x, pos_y, pos_z;
	uint64_t timestamp_ms;
	bool enabled;
	bool sf_enabled;
	float sf_quat_w, sf_quat_x, sf_quat_y, sf_quat_z;
};

/*
 * Static device properties from the XR driver.
 * These change at most when a different device is connected; poll at ~1 Hz.
 */
struct breezy_imu_device_config {
	uint32_t display_width;       /* physical display width in pixels */
	uint32_t display_height;      /* physical display height in pixels */
	float    diagonal_fov_deg;    /* diagonal field-of-view of the optics */
	float    lens_distance_ratio; /* IPD / lens offset ratio */
	uint8_t  version;             /* protocol version written by the driver */
	bool     enabled;             /* driver's own enabled flag */
};

/*
 * Start the inotify watcher thread that reads /dev/shm/breezy_desktop_imu
 * whenever the file is modified.  Call once before using the other APIs.
 * Returns 0 on success, -1 on error.
 */
int breezy_imu_init(void);

/*
 * Stop the watcher thread and free resources.
 */
void breezy_imu_destroy(void);

/*
 * Fill *out with the most-recently-parsed valid pose.
 * Returns true if a valid, non-stale pose is available.
 */
bool breezy_imu_try_get_pose(struct breezy_imu_pose *out);

/*
 * Fill *out with the most-recently-parsed device config.
 * Returns true if the file has been read at least once.
 * The caller must independently verify staleness via breezy_imu_try_get_pose.
 */
bool breezy_imu_get_config(struct breezy_imu_device_config *out);
