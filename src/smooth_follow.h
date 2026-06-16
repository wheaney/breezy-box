#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "display_renderer.h" /* ESMatrix */

/*
 * Smooth follow — renderer-agnostic state machine + geometry.
 *
 * Mirrors XRLinuxDriver's smooth_follow output as consumed by the GNOME/KWin
 * plugins.  This module owns the *behaviour* (the enable/disable easing, the
 * slerp-back window, the focused-display centring math) and is deliberately free
 * of any DRM/KMS, EGL or GL specifics so a different renderer can reuse it.  It
 * depends only on the project's generic matrix type (ESMatrix) and es_* helpers.
 *
 * Two coordinated effects, both driven from quaternions in the EUS world frame:
 *
 *   1. Camera follow.  While follow is engaged — or still slerping back after a
 *      disable — the camera is driven from the smooth-follow *origin* orientation
 *      instead of the head pose, so every display swings to follow the head.
 *      See smooth_follow_use_origin() / smooth_follow_camera_quat().
 *
 *   2. Focused-display centring.  The looked-at display additionally eases to a
 *      centred-in-front position locked to the (residual) head pose, via
 *      smoothFollowRotation = origin ⊗ conj(pose).  See smooth_follow_is_centring()
 *      / smooth_follow_focused_model().
 *
 * The driver clears its enabled flag immediately on toggle-off but keeps slerping
 * the origin back toward the head for ~1s; smooth_follow_update() holds the camera
 * on the origin for that window so the transition stays smooth.
 */

struct smooth_follow_state {
	bool     enabled;             /* latest driver flag (this frame) */
	bool     enabled_prev;        /* previous frame's flag, for edge detection */
	bool     disabling;           /* follow off but origin still slerping back */
	bool     have_pose;           /* pose_quat / origin_quat are valid */
	uint64_t disabling_until_ms;  /* end of the slerp-back window */
	float    pose_quat[4];        /* head pose orientation, EUS [w,x,y,z] */
	float    origin_quat[4];      /* smooth-follow origin orientation, EUS [w,x,y,z] */
	int      focus_slot;          /* display slot being centred, -1 = none */
	float    progress;            /* 0 = arc position, 1 = centred-in-front */
	float    trans_from;          /* centring ease endpoints / timing */
	float    trans_to;
	uint64_t trans_begin_ms;
	bool     trans_active;
};

/*
 * Resolve which display slot the (origin) orientation is looking at, used by
 * smooth_follow_update() on the frame follow engages to pick the display to
 * centre.  Returns a slot index or -1 for none.  May be NULL, in which case no
 * display is centred (the camera still follows).  The orientation is the
 * smooth-follow origin quaternion and the position is the head pose position,
 * both as passed to smooth_follow_update().
 */
typedef int (*smooth_follow_focus_fn)(void *ctx,
                                      const float origin_quat[4],
                                      const float pose_pos[3]);

/* Zero-initialise (focus_slot = -1, everything cleared). */
void smooth_follow_init(struct smooth_follow_state *sf);

/*
 * Drop any in-flight centring without disturbing the enable/camera tracking.
 * Call when the display layout changes and slot indices are no longer stable.
 */
void smooth_follow_reset_focus(struct smooth_follow_state *sf);

/*
 * Advance the state machine for this frame.  Call once before reading any of the
 * accessors below.
 *
 *   have_pose    – whether pose_quat/origin_quat/pose_pos hold valid data.
 *   enabled      – the driver's smooth-follow flag this frame.
 *   pose_quat    – head pose orientation, EUS [w,x,y,z].
 *   origin_quat  – smooth-follow origin orientation, EUS [w,x,y,z].
 *   pose_pos     – head pose position (forwarded to focus_fn); may be NULL.
 *   now_ms       – monotonic-ish wall clock in milliseconds.
 *   focus_fn/ctx – consulted only on the enabling edge to choose the centred
 *                  display; may be NULL.
 */
void smooth_follow_update(struct smooth_follow_state *sf,
                          bool have_pose, bool enabled,
                          const float pose_quat[4],
                          const float origin_quat[4],
                          const float pose_pos[3],
                          uint64_t now_ms,
                          smooth_follow_focus_fn focus_fn, void *focus_ctx);

/* True when the camera should be driven from the origin (vs the head pose). */
bool smooth_follow_use_origin(const struct smooth_follow_state *sf);

/*
 * Orientation the camera should use this frame (origin while following, else the
 * head pose), as [w,x,y,z].  Returns NULL when no pose is available.
 */
const float *smooth_follow_camera_quat(const struct smooth_follow_state *sf);

/* True if the given display slot is mid-centre (use smooth_follow_focused_model). */
bool smooth_follow_is_centring(const struct smooth_follow_state *sf, int slot);

/*
 * The display slot smooth follow currently owns while engaged, or -1 if none /
 * not following.  Callers that also run zoom-on-focus should lock the zoom focus
 * onto this slot so the two features agree on the followed (centred) display
 * rather than the zoom independently re-deriving focus from gaze against the
 * displays' arc positions.
 */
int smooth_follow_focus_slot(const struct smooth_follow_state *sf);

/*
 * Build the model matrix for the display being centred, easing between its arc
 * position (progress 0) and centred-in-front (progress 1).  The display's mesh
 * is assumed to bake its centerNoRotate offset (cnx,cny,cnz), as produced by
 * dp_generate_mesh_vertices(); the arc rotation is replaced by the full
 * smooth-follow rotation and only the position interpolates.
 *
 *   cnx,cny,cnz – GL/EUS centerNoRotate baked into the mesh.
 *   angle       – the display's arc Y-rotation (placement angle, radians).
 *   scale       – distance multiplier (e.g. monitor_distance / all_distance).
 */
void smooth_follow_focused_model(const struct smooth_follow_state *sf,
                                 float cnx, float cny, float cnz,
                                 float angle, float scale,
                                 ESMatrix *out_model);
