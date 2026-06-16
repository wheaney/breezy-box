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
    /* NWU centerLook [north, west, up] as returned by monitorsToPlacements,
     * kept for dp_find_focused_monitor (which works in the driver's NWU frame). */
    float ln_north, ln_west, ln_up;
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
 *
 * width_px / height_px are the raw device resolution passed to buildFovDetails.
 * size_adj_width / size_adj_height default to width_px / height_px and should
 * be overwritten by the caller when a display-size setting is in effect
 * (= device_px * distance_adjusted_size).  Both fields are required by
 * dp_generate_mesh_vertices.
 */
struct dp_fov_details {
    float complete_dist_px;       /* completeScreenDistancePixels */
    float horizontal_radians;     /* defaultDistanceHorizontalRadians */
    float vertical_radians;       /* defaultDistanceVerticalRadians */
    float lens_distance_px;       /* lensDistancePixels */
    float full_screen_dist_px;    /* fullScreenDistancePixels */
    float width_px;               /* widthPixels (raw device width) */
    float height_px;              /* heightPixels (raw device height) */
    float size_adj_width;         /* sizeAdjustedWidthPixels */
    float size_adj_height;        /* sizeAdjustedHeightPixels */
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
 * Generate triangle-strip mesh vertices for one monitor by calling
 * generateMeshVertices() in the shared JS bundle.  The shared function
 * uses fovConversionFns (flat or curved per axis) for radians and segment
 * count — no conversion math is duplicated in C.
 *
 * Vertices are written to verts_out as tightly-packed (x, y, z, s, t) floats,
 * with the centre offset (cx, cy, cz) already added.  The layout is compatible
 * with display_vertex / display_renderer_draw_mesh.
 *
 *   fov            – fovDetails from dp_build_fov_details; size_adj_width /
 *                    size_adj_height must be set before calling
 *   mon_w / mon_h   – this monitor's pixel dimensions (size-adjusted)
 *   wrapping_scheme – "horizontal" | "vertical" | "flat"; both wrap flags are
 *                     derived independently so "flat" correctly disables both
 *                     (mirrors CurvableDisplayMesh.qml exactly)
 *   curved          – 1 = arc geometry, 0 = flat quad
 *   cx/cy/cz        – GL-space centre offset (centerNoRotate from dp_placement)
 *   verts_out       – output buffer, 5 floats per vertex
 *   max_verts       – capacity of verts_out
 *
 * Returns the number of vertices written, or -1 on error.
 */
int dp_generate_mesh_vertices(const struct dp_fov_details *fov,
                               float mon_w, float mon_h,
                               const char *wrapping_scheme, int curved,
                               float cx, float cy, float cz,
                               float *verts_out, int max_verts);

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

/*
 * Find which monitor the head pose is looking at, via the shared
 * findFocusedMonitor() in displayPlacement.js.
 *
 * The pose quaternion (qw,qx,qy,qz) and position (east,up,south) are in the
 * driver's EUS frame; they are converted to NWU internally with the shared
 * eusToNwu* helpers.  placements must carry the NWU centerLook populated by
 * dp_compute_placements; monitors are the same size-adjusted infos used there.
 *
 *   current_focused_index     – previously focused index (-1 if none)
 *   focused_monitor_distance   – display_distance / display_distance_default
 *                                (< 1 when a focused display is zoomed in)
 *   smooth_follow_enabled      – when true, the shared finder locks onto
 *                                current_focused_index (if set) and otherwise
 *                                returns the closest display unconditionally,
 *                                bypassing FOCUS_THRESHOLD — matching the
 *                                references' follow-mode focus selection.
 *
 * Returns the focused monitor index, -1 if none, or -1 on error.
 */
int dp_find_focused_monitor(const struct dp_placement *placements,
                            const struct dp_monitor_info *monitors,
                            size_t n,
                            float qw, float qx, float qy, float qz,
                            float pos_east, float pos_up, float pos_south,
                            int current_focused_index,
                            float focused_monitor_distance,
                            bool smooth_follow_enabled,
                            uint32_t device_width,
                            uint32_t device_height,
                            float diagonal_fov_rad,
                            float lens_distance_ratio,
                            float display_distance_default,
                            float size_adj_width,
                            float size_adj_height,
                            const char *wrapping_scheme,
                            bool curved_display);

/*
 * Ease a display's distance multiplier toward its target, via the shared
 * easeDistance() in zoomOnFocus.js.  Returns 0 on success, -1 on error;
 * *out_distance receives the eased multiplier and *out_done whether the
 * transition has completed (may be NULL).
 */
int dp_ease_distance(float start_distance, float target_distance,
                     float elapsed_ms, bool gaining_focus,
                     bool needs_sequence_delay,
                     float *out_distance, bool *out_done);
