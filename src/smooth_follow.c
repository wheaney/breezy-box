#include "smooth_follow.h"

#include <math.h>
#include <string.h>

/*
 * Timings mirror the KWin reference (the closest architectural match — a single
 * shared camera rather than GNOME's per-display shader):
 *   SF_TRANSITION_MS – focused display centre/un-centre ease
 *                      (BreezyDesktop.qml smoothFollowTransitionAnimation).
 *   SF_DISABLING_MS  – keep the camera on the origin after follow turns off,
 *                      while the driver slerps the origin back
 *                      (CameraController.qml smoothFollowDisablingTimer).
 */
#define SF_TRANSITION_MS 150.0f
#define SF_DISABLING_MS  750u

/* ----------------------------------------------------------------
 * Quaternion helpers — scalar-first [w,x,y,z], EUS world frame.
 * ---------------------------------------------------------------- */

static void quat_conj(const float q[4], float o[4])
{
	o[0] = q[0]; o[1] = -q[1]; o[2] = -q[2]; o[3] = -q[3];
}

/* Hamilton product o = a ⊗ b. */
static void quat_mul(const float a[4], const float b[4], float o[4])
{
	o[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
	o[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
	o[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
	o[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

/* Rotate vector v by quaternion q (forward): o = v + 2w(q×v) + 2 q×(q×v). */
static void quat_rotate_vec(const float q[4], const float v[3], float o[3])
{
	float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
	float tx = 2.0f * (qy*v[2] - qz*v[1]);
	float ty = 2.0f * (qz*v[0] - qx*v[2]);
	float tz = 2.0f * (qx*v[1] - qy*v[0]);
	o[0] = v[0] + qw*tx + (qy*tz - qz*ty);
	o[1] = v[1] + qw*ty + (qz*tx - qx*tz);
	o[2] = v[2] + qw*tz + (qx*ty - qy*tx);
}

/* ----------------------------------------------------------------
 * State machine
 * ---------------------------------------------------------------- */

void smooth_follow_init(struct smooth_follow_state *sf)
{
	memset(sf, 0, sizeof(*sf));
	sf->focus_slot = -1;
}

void smooth_follow_reset_focus(struct smooth_follow_state *sf)
{
	sf->focus_slot   = -1;
	sf->progress     = 0.0f;
	sf->trans_active = false;
}

/* Begin (or retarget) the focused-display centre/un-centre ease. */
static void begin_transition(struct smooth_follow_state *sf, float target, uint64_t now_ms)
{
	sf->trans_from     = sf->progress;
	sf->trans_to       = target;
	sf->trans_begin_ms = now_ms;
	sf->trans_active   = (sf->progress != target);
	if (!sf->trans_active) {
		sf->progress = target;
		if (target == 0.0f)
			sf->focus_slot = -1;
	}
}

void smooth_follow_update(struct smooth_follow_state *sf,
                          bool have_pose, bool enabled,
                          const float pose_quat[4],
                          const float origin_quat[4],
                          const float pose_pos[3],
                          uint64_t now_ms,
                          smooth_follow_focus_fn focus_fn, void *focus_ctx)
{
	sf->have_pose = have_pose;
	if (have_pose) {
		memcpy(sf->pose_quat,   pose_quat,   sizeof(sf->pose_quat));
		memcpy(sf->origin_quat, origin_quat, sizeof(sf->origin_quat));
	}

	bool now = have_pose && enabled;

	if (now != sf->enabled_prev) {
		if (now) {
			/* Enabling: cancel any pending disable window and ease the
			 * looked-at display (per the origin orientation) to centre. */
			sf->disabling = false;
			int slot = focus_fn ? focus_fn(focus_ctx, sf->origin_quat, pose_pos) : -1;
			if (slot >= 0) {
				sf->focus_slot = slot;
				begin_transition(sf, 1.0f, now_ms);
			}
			/* slot < 0: nothing centres, but the camera still follows. */
		} else {
			/*
			 * Disabling: the driver has already cleared the flag but keeps
			 * slerping the origin back toward the head, so keep the camera on
			 * the origin until the window expires.  Ease the centred display
			 * back to its arc position meanwhile.
			 */
			sf->disabling          = true;
			sf->disabling_until_ms = now_ms + SF_DISABLING_MS;
			if (sf->focus_slot >= 0)
				begin_transition(sf, 0.0f, now_ms);
		}
		sf->enabled_prev = now;
	}
	sf->enabled = now;

	if (sf->disabling && now_ms >= sf->disabling_until_ms)
		sf->disabling = false;

	if (sf->trans_active) {
		float t = (float)(now_ms - sf->trans_begin_ms) / SF_TRANSITION_MS;
		if (t >= 1.0f) {
			sf->progress     = sf->trans_to;
			sf->trans_active = false;
			if (sf->trans_to == 0.0f)
				sf->focus_slot = -1;
		} else {
			sf->progress = sf->trans_from + (sf->trans_to - sf->trans_from) * t;
		}
	}
}

bool smooth_follow_use_origin(const struct smooth_follow_state *sf)
{
	return sf->enabled || sf->disabling;
}

const float *smooth_follow_camera_quat(const struct smooth_follow_state *sf)
{
	if (!sf->have_pose)
		return NULL;
	return smooth_follow_use_origin(sf) ? sf->origin_quat : sf->pose_quat;
}

bool smooth_follow_is_centring(const struct smooth_follow_state *sf, int slot)
{
	return sf->have_pose && slot == sf->focus_slot && sf->progress > 0.0f;
}

int smooth_follow_focus_slot(const struct smooth_follow_state *sf)
{
	return sf->enabled ? sf->focus_slot : -1;
}

void smooth_follow_focused_model(const struct smooth_follow_state *sf,
                                 float cnx, float cny, float cnz,
                                 float angle, float scale,
                                 ESMatrix *out_model)
{
	/* smoothFollowRotation = origin ⊗ conj(pose): the residual head motion the
	 * centred display tracks once the camera is already running on the origin. */
	float cpose[4], sfrot[4];
	quat_conj(sf->pose_quat, cpose);
	quat_mul(sf->origin_quat, cpose, sfrot);

	float ca = cosf(angle), sa = sinf(angle);

	/* Arc position: scale · R(angle) · centerNoRotate (Y-rotation only). */
	float arc[3] = {
		scale * (ca * cnx + sa * cnz),
		scale * cny,
		scale * (-sa * cnx + ca * cnz)
	};
	/* Centred straight ahead at the monitor's north distance
	 * (monitorCenterNorth = -cnz), rotated into the follow frame. */
	float vc[3] = { 0.0f, 0.0f, cnz * scale };
	float cen[3];
	quat_rotate_vec(sfrot, vc, cen);

	float p = sf->progress;
	float fp[3] = {
		arc[0] + (cen[0] - arc[0]) * p,
		arc[1] + (cen[1] - arc[1]) * p,
		arc[2] + (cen[2] - arc[2]) * p
	};

	/* model = T(fp) · R(sfrot) · T(-centerNoRotate); the mesh bakes
	 * centerNoRotate, so undo it before rotating about the origin. */
	ESMatrix tneg, rq, tpos, rt;
	es_display_model(&tneg, 0.0f, -cnx, -cny, -cnz);
	es_model_from_quat(&rq, sfrot[0], sfrot[1], sfrot[2], sfrot[3]);
	es_display_model(&tpos, 0.0f, fp[0], fp[1], fp[2]);
	es_multiply(&rt, &tneg, &rq);        /* rt    = rq · tneg        */
	es_multiply(out_model, &rt, &tpos);  /* model = tpos · rq · tneg */
}
