#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Screen-space position and resolution of a single monitor. */
struct dp_monitor_info {
    int32_t  x, y;      /* virtual desktop origin (pixels) */
    uint32_t width;     /* horizontal resolution */
    uint32_t height;    /* vertical resolution */
};

/* GL-space transform for a display on the arc. */
struct dp_placement {
    float tx, ty, tz;    /* GL centerLook position: (-west, up, -north) */
    float cnx, cny, cnz; /* GL centerNoRotate: same axes, used as mesh base before Y-rotation */
    float angle;         /* Y-axis rotation in radians (rotationAngleRadians.y) */
};

/*
 * Initialize the shared QuickJS runtime, evaluate the JS bundle, and cache
 * function references.  Call once before any other dp_* function.
 * Returns 0 on success, -1 on failure.
 */
int  dp_init(void);

/*
 * Release the shared QuickJS runtime and all cached state.
 */
void dp_destroy(void);

/*
 * Result of diagonalToCrossFOVs (mirrors shared/math.js exactly).
 * Unit-distance values are relative to a viewer-to-display distance of 1.0:
 *   height_unit = 2*tan(vfov/2),  width_unit = 2*tan(hfov/2)
 * Multiply by the actual arc radius to get half-extents for draw_quad.
 */
struct dp_fov_lengths {
    float horizontal_radians;
    float vertical_radians;
    float diagonal_length_unit;
    float width_unit;
    float height_unit;
};

/*
 * Convert a diagonal FOV (radians) and aspect ratio to cross-FOV values by
 * delegating to diagonalToCrossFOVs in the bundled shared/math.js.
 * Returns 0 on success, -1 on failure.
 */
int dp_diagonal_to_cross_fovs(float diagonal_radians, float aspect_ratio,
                               struct dp_fov_lengths *out);

/*
 * Result of buildFovDetails (mirrors shared/displayPlacement.js exactly).
 * Captures all placement-relevant scalar fields from the returned object.
 */
struct dp_fov_details {
    float complete_dist_px;       /* completeScreenDistancePixels */
    float horizontal_radians;     /* defaultDistanceHorizontalRadians */
    float vertical_radians;       /* defaultDistanceVerticalRadians */
    float lens_distance_px;       /* lensDistancePixels */
    float full_screen_dist_px;    /* fullScreenDistancePixels */
};

/*
 * Build the fovDetails object via buildFovDetails() in the shared JS bundle.
 * Mirrors KWin's Displays.buildFovDetails() with distanceAdjustedSize = 1.
 *
 *   device_width/height        – glasses display resolution in pixels
 *   diagonal_fov_rad           – glasses diagonal FOV in radians
 *   lens_distance_ratio        – from device config (0 = screen at lens)
 *   default_display_distance   – arc zoom factor (1.0 = fill FOV exactly)
 *   wrapping_scheme            – "horizontal", "vertical", or "flat"
 *                                (resolve "automatic" before calling)
 *   curved_display             – true enables curved arc geometry
 *
 * Returns 0 on success, -1 on failure.
 */
int dp_build_fov_details(uint32_t device_width,
                          uint32_t device_height,
                          float diagonal_fov_rad,
                          float lens_distance_ratio,
                          float default_display_distance,
                          const char *wrapping_scheme,
                          bool curved_display,
                          struct dp_fov_details *out);

/*
 * Compute GL placements for n monitors arranged on a curved arc using the
 * shared JS placement logic (monitorsToPlacements from displayPlacement.js).
 *
 * Monitor x/y/width/height should already be viewport-centred and
 * size-adjusted by the caller (see breezy_settings helpers).
 *
 *   device_width/height       – physical glasses display resolution in pixels
 *   diagonal_fov_rad          – glasses diagonal FOV in radians
 *   lens_distance_ratio       – from device config (0 = screen at lens)
 *   display_distance_default  – arc zoom factor (max of distance + toggle range)
 *   size_adj_width/height     – device dims * distance_adjusted_size
 *   arc_radius_gl             – arc radius in GL units (sets overall scale;
 *                               pass fov_details.complete_dist_px for 1:1)
 *   wrapping_scheme           – "horizontal", "vertical", or "flat"
 *                               (resolve "automatic" before calling)
 *   curved_display            – true enables curved arc geometry
 *   monitor_spacing           – fraction of viewport width (e.g. 0.02)
 *
 * placements[i] is filled for each monitors[i]; indices are preserved.
 * Returns 0 on success, -1 on failure (error written to stderr).
 */
int dp_compute_placements(const struct dp_monitor_info *monitors,
                          size_t n,
                          uint32_t device_width,
                          uint32_t device_height,
                          float diagonal_fov_rad,
                          float lens_distance_ratio,
                          float display_distance_default,
                          float size_adj_width,
                          float size_adj_height,
                          float arc_radius_gl,
                          const char *wrapping_scheme,
                          bool curved_display,
                          float monitor_spacing,
                          struct dp_placement *placements);
