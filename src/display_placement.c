#define _POSIX_C_SOURCE 200809L

#include "display_placement.h"
#include "display_placement_bundle.h"
#include "quickjs.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define DP_MAX_MONITORS 32u

/* ----------------------------------------------------------------
 * Shared QuickJS runtime — initialized once, reused by all callers.
 * ---------------------------------------------------------------- */

static JSRuntime *g_rt;
static JSContext *g_ctx;
static JSValue    g_fn_monitors_to_placements;
static JSValue    g_fn_diagonal_to_cross_fovs;
static JSValue    g_fn_build_fov_details;
static JSValue    g_fn_generate_mesh_vertices;
static JSValue    g_fn_find_focused_monitor;
static JSValue    g_fn_eus_to_nwu_quat;
static JSValue    g_fn_eus_to_nwu_vector;
static JSValue    g_fn_ease_distance;

/* ----------------------------------------------------------------
 * Marshalling helpers
 *
 * These keep the JS<->C glue in one place so the per-call code reads as
 * data flow, not QuickJS bookkeeping.  Numbers/bools/ints in QuickJS are
 * not reference-counted, so float/bool arguments never need freeing; only
 * strings, arrays and objects do (freed explicitly at each call site).
 * ---------------------------------------------------------------- */

/* If val is a JS exception, print it to stderr and return true. Frees nothing. */
static bool dp_is_exc(JSValue val, const char *what)
{
    if (!JS_IsException(val))
        return false;
    JSValue exc = JS_GetException(g_ctx);
    const char *msg = JS_ToCString(g_ctx, exc);
    fprintf(stderr, "dp: %s error: %s\n", what, msg ? msg : "(unknown)");
    JS_FreeCString(g_ctx, msg);
    JS_FreeValue(g_ctx, exc);
    return true;
}

/* Read one numeric property of obj as a double (0 if missing/non-numeric). */
static double dp_get_float(JSValue obj, const char *field)
{
    JSValue v = JS_GetPropertyStr(g_ctx, obj, field);
    double d = 0.0;
    JS_ToFloat64(g_ctx, &d, v);
    JS_FreeValue(g_ctx, v);
    return d;
}

/* Read the first n numeric elements of a JS array into out. */
static void dp_get_floats(JSValue arr, double *out, size_t n)
{
    for (size_t i = 0u; i < n; i++) {
        JSValue v = JS_GetPropertyUint32(g_ctx, arr, (uint32_t)i);
        out[i] = 0.0;
        JS_ToFloat64(g_ctx, &out[i], v);
        JS_FreeValue(g_ctx, v);
    }
}

/* Build a JS number array [vals[0], ..]; caller frees. */
static JSValue dp_new_floats(const double *vals, size_t n)
{
    JSValue arr = JS_NewArray(g_ctx);
    for (size_t i = 0u; i < n; i++)
        JS_SetPropertyUint32(g_ctx, arr, (uint32_t)i, JS_NewFloat64(g_ctx, vals[i]));
    return arr;
}

/* Call a single-arg JS conversion fn (eusToNwu*) returning a length-n array. */
static int dp_call_convert(JSValue fn, const double *in, size_t n, double *out)
{
    JSValue arg = dp_new_floats(in, n);
    JSValue res = JS_Call(g_ctx, fn, JS_UNDEFINED, 1, &arg);
    JS_FreeValue(g_ctx, arg);
    if (dp_is_exc(res, "frame conversion")) {
        JS_FreeValue(g_ctx, res);
        return -1;
    }
    dp_get_floats(res, out, n);
    JS_FreeValue(g_ctx, res);
    return 0;
}

/*
 * Build a fovDetails object via the shared buildFovDetails().  Returns the
 * object (caller frees) or JS_EXCEPTION on failure.  scheme defaults to
 * "horizontal" when empty.
 */
static JSValue dp_build_fov_obj(uint32_t dev_w, uint32_t dev_h, float diag_rad,
                                float lens_ratio, float dist_default,
                                const char *scheme, bool curved)
{
    const char *s = (scheme && *scheme) ? scheme : "horizontal";
    JSValue args[7] = {
        JS_NewFloat64(g_ctx, (double)dev_w),
        JS_NewFloat64(g_ctx, (double)dev_h),
        JS_NewFloat64(g_ctx, (double)diag_rad),
        JS_NewFloat64(g_ctx, (double)lens_ratio),
        JS_NewFloat64(g_ctx, (double)dist_default),
        JS_NewString (g_ctx, s),
        JS_NewBool   (g_ctx, curved ? 1 : 0)
    };
    JSValue r = JS_Call(g_ctx, g_fn_build_fov_details, JS_UNDEFINED, 7, args);
    JS_FreeValue(g_ctx, args[5]);
    if (dp_is_exc(r, "buildFovDetails")) {
        JS_FreeValue(g_ctx, r);
        return JS_EXCEPTION;
    }
    return r;
}

/*
 * Override sizeAdjusted{Width,Height}Pixels on a fovDetails object so the arc
 * extents are computed relative to the scaled viewport, not the raw device
 * resolution.  Used by both placement and focus paths.
 */
static void dp_set_size_adj(JSValue fov, float saw, float sah)
{
    JS_SetPropertyStr(g_ctx, fov, "sizeAdjustedWidthPixels",  JS_NewFloat64(g_ctx, (double)saw));
    JS_SetPropertyStr(g_ctx, fov, "sizeAdjustedHeightPixels", JS_NewFloat64(g_ctx, (double)sah));
}

/* ----------------------------------------------------------------
 * Runtime lifecycle
 * ---------------------------------------------------------------- */

int dp_init(void)
{
    JSValue global, eval_result;

    g_rt = JS_NewRuntime();
    if (!g_rt) {
        fprintf(stderr, "dp: failed to create QuickJS runtime\n");
        return -1;
    }
    g_ctx = JS_NewContext(g_rt);
    if (!g_ctx) {
        fprintf(stderr, "dp: failed to create QuickJS context\n");
        JS_FreeRuntime(g_rt);
        g_rt = NULL;
        return -1;
    }

    eval_result = JS_Eval(g_ctx, k_display_placement_js,
                          strlen(k_display_placement_js),
                          "<bundle>", JS_EVAL_TYPE_GLOBAL);
    if (dp_is_exc(eval_result, "JS bundle eval")) {
        JS_FreeValue(g_ctx, eval_result);
        JS_FreeContext(g_ctx);
        JS_FreeRuntime(g_rt);
        g_ctx = NULL;
        g_rt  = NULL;
        return -1;
    }
    JS_FreeValue(g_ctx, eval_result);

    global = JS_GetGlobalObject(g_ctx);
    g_fn_monitors_to_placements  = JS_GetPropertyStr(g_ctx, global, "monitorsToPlacements");
    g_fn_diagonal_to_cross_fovs  = JS_GetPropertyStr(g_ctx, global, "diagonalToCrossFOVs");
    g_fn_build_fov_details        = JS_GetPropertyStr(g_ctx, global, "buildFovDetails");
    g_fn_generate_mesh_vertices   = JS_GetPropertyStr(g_ctx, global, "generateMeshVertices");
    g_fn_find_focused_monitor     = JS_GetPropertyStr(g_ctx, global, "findFocusedMonitor");
    g_fn_eus_to_nwu_quat          = JS_GetPropertyStr(g_ctx, global, "eusToNwuQuat");
    g_fn_eus_to_nwu_vector        = JS_GetPropertyStr(g_ctx, global, "eusToNwuVector");
    g_fn_ease_distance            = JS_GetPropertyStr(g_ctx, global, "easeDistance");
    JS_FreeValue(g_ctx, global);

    /* Every binding must resolve to a function or the bundle is malformed. */
    static const struct { JSValue *fn; const char *name; } required[] = {
        { &g_fn_monitors_to_placements, "monitorsToPlacements" },
        { &g_fn_diagonal_to_cross_fovs, "diagonalToCrossFOVs" },
        { &g_fn_build_fov_details,      "buildFovDetails" },
        { &g_fn_generate_mesh_vertices, "generateMeshVertices" },
        { &g_fn_find_focused_monitor,   "findFocusedMonitor" },
        { &g_fn_eus_to_nwu_quat,        "eusToNwuQuat" },
        { &g_fn_eus_to_nwu_vector,      "eusToNwuVector" },
        { &g_fn_ease_distance,          "easeDistance" },
    };
    for (size_t i = 0u; i < sizeof(required) / sizeof(required[0]); i++) {
        if (!JS_IsFunction(g_ctx, *required[i].fn)) {
            fprintf(stderr, "dp: %s not found in JS bundle\n", required[i].name);
            dp_destroy();
            return -1;
        }
    }

    return 0;
}

void dp_destroy(void)
{
    if (!g_ctx)
        return;

    JSValue *fns[] = {
        &g_fn_monitors_to_placements, &g_fn_diagonal_to_cross_fovs,
        &g_fn_build_fov_details,      &g_fn_generate_mesh_vertices,
        &g_fn_find_focused_monitor,   &g_fn_eus_to_nwu_quat,
        &g_fn_eus_to_nwu_vector,      &g_fn_ease_distance,
    };
    for (size_t i = 0u; i < sizeof(fns) / sizeof(fns[0]); i++) {
        JS_FreeValue(g_ctx, *fns[i]);
        *fns[i] = JS_UNDEFINED;
    }

    JS_FreeContext(g_ctx);
    JS_FreeRuntime(g_rt);
    g_ctx = NULL;
    g_rt  = NULL;
}

/* ----------------------------------------------------------------
 * dp_diagonal_to_cross_fovs
 * ---------------------------------------------------------------- */

int dp_diagonal_to_cross_fovs(float diagonal_radians, float aspect_ratio,
                               struct dp_fov_lengths *out)
{
    JSValue args[2], result;

    if (!out || !g_ctx)
        return -1;

    args[0] = JS_NewFloat64(g_ctx, (double)diagonal_radians);
    args[1] = JS_NewFloat64(g_ctx, (double)aspect_ratio);
    result  = JS_Call(g_ctx, g_fn_diagonal_to_cross_fovs, JS_UNDEFINED, 2, args);

    if (dp_is_exc(result, "diagonalToCrossFOVs")) {
        JS_FreeValue(g_ctx, result);
        return -1;
    }

    out->horizontal_radians   = (float)dp_get_float(result, "horizontalRadians");
    out->vertical_radians     = (float)dp_get_float(result, "verticalRadians");
    out->diagonal_length_unit = (float)dp_get_float(result, "diagonalLengthUnitDistance");
    out->width_unit           = (float)dp_get_float(result, "widthUnitDistance");
    out->height_unit          = (float)dp_get_float(result, "heightUnitDistance");

    JS_FreeValue(g_ctx, result);
    return 0;
}

/* ----------------------------------------------------------------
 * dp_build_fov_details
 * ---------------------------------------------------------------- */

int dp_build_fov_details(uint32_t device_width,
                          uint32_t device_height,
                          float diagonal_fov_rad,
                          float lens_distance_ratio,
                          float default_display_distance,
                          const char *wrapping_scheme,
                          bool curved_display,
                          struct dp_fov_details *out)
{
    JSValue result;

    if (!out || !g_ctx)
        return -1;

    result = dp_build_fov_obj(device_width, device_height, diagonal_fov_rad,
                              lens_distance_ratio, default_display_distance,
                              wrapping_scheme, curved_display);
    if (JS_IsException(result)) {
        JS_FreeValue(g_ctx, result);
        return -1;
    }

    out->complete_dist_px     = (float)dp_get_float(result, "completeScreenDistancePixels");
    out->horizontal_radians   = (float)dp_get_float(result, "defaultDistanceHorizontalRadians");
    out->vertical_radians     = (float)dp_get_float(result, "defaultDistanceVerticalRadians");
    out->lens_distance_px     = (float)dp_get_float(result, "lensDistancePixels");
    out->full_screen_dist_px  = (float)dp_get_float(result, "fullScreenDistancePixels");
    out->width_px             = (float)dp_get_float(result, "widthPixels");
    out->height_px            = (float)dp_get_float(result, "heightPixels");
    /* size_adj_* default to device dims; caller overrides if display_size != 1 */
    out->size_adj_width       = (float)dp_get_float(result, "sizeAdjustedWidthPixels");
    out->size_adj_height      = (float)dp_get_float(result, "sizeAdjustedHeightPixels");

    JS_FreeValue(g_ctx, result);
    return 0;
}

/* ----------------------------------------------------------------
 * dp_compute_placements
 * ---------------------------------------------------------------- */

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
                          struct dp_placement *placements)
{
    double gl_scale;
    JSValue fov_obj     = JS_UNDEFINED;
    JSValue monitor_arr = JS_UNDEFINED;
    JSValue spacing     = JS_UNDEFINED;
    JSValue call_result = JS_UNDEFINED;
    int ret = -1;
    size_t i;

    if (!monitors || !placements || n == 0u)
        return 0;

    if (!g_ctx) {
        fprintf(stderr, "dp: dp_init() was not called\n");
        return -1;
    }

    if (n > DP_MAX_MONITORS) {
        fprintf(stderr, "dp: too many monitors (max %u)\n", DP_MAX_MONITORS);
        return -1;
    }

    if (device_width == 0 || device_height == 0) {
        fprintf(stderr, "dp: device_width and device_height must be non-zero\n");
        return -1;
    }

    /*
     * Delegate fovDetails construction to the shared buildFovDetails(), then
     * override sizeAdjustedWidth/HeightPixels so monitorsToPlacements uses the
     * correct size-adjusted viewport reference.  completeScreenDistancePixels
     * gives the GL unit scale.
     */
    fov_obj = dp_build_fov_obj(device_width, device_height, diagonal_fov_rad,
                               lens_distance_ratio, display_distance_default,
                               wrapping_scheme, curved_display);
    if (JS_IsException(fov_obj)) {
        JS_FreeValue(g_ctx, fov_obj);
        return -1;
    }

    /* Uniform scale from pixel-space NWU coords to GL units. */
    gl_scale = (double)arc_radius_gl / dp_get_float(fov_obj, "completeScreenDistancePixels");
    dp_set_size_adj(fov_obj, size_adj_width, size_adj_height);

    monitor_arr = JS_NewArray(g_ctx);
    for (i = 0u; i < n; i++) {
        JSValue mon = JS_NewObject(g_ctx);
        JS_SetPropertyStr(g_ctx, mon, "x",      JS_NewInt32(g_ctx,  monitors[i].x));
        JS_SetPropertyStr(g_ctx, mon, "y",      JS_NewInt32(g_ctx,  monitors[i].y));
        JS_SetPropertyStr(g_ctx, mon, "width",  JS_NewUint32(g_ctx, monitors[i].width));
        JS_SetPropertyStr(g_ctx, mon, "height", JS_NewUint32(g_ctx, monitors[i].height));
        JS_SetPropertyUint32(g_ctx, monitor_arr, (uint32_t)i, mon);
    }

    spacing = JS_NewFloat64(g_ctx, (double)monitor_spacing);

    {
        JSValue args[3] = { fov_obj, monitor_arr, spacing };
        call_result = JS_Call(g_ctx, g_fn_monitors_to_placements,
                              JS_UNDEFINED, 3, args);
    }

    if (dp_is_exc(call_result, "monitorsToPlacements"))
        goto cleanup;

    {
        JSValue len_val = JS_GetPropertyStr(g_ctx, call_result, "length");
        uint32_t result_len = 0u;
        JS_ToUint32(g_ctx, &result_len, len_val);
        JS_FreeValue(g_ctx, len_val);

        for (i = 0u; i < (size_t)result_len && i < n; i++) {
            JSValue item = JS_GetPropertyUint32(g_ctx, call_result, (uint32_t)i);
            int32_t orig_idx = (int32_t)dp_get_float(item, "originalIndex");
            double cl[3], cn[3], rot_x, rot_y;
            JSValue v;

            if (orig_idx < 0 || (size_t)orig_idx >= n) {
                JS_FreeValue(g_ctx, item);
                continue;
            }

            /* centerLook: NWU position with arc rotation applied */
            v = JS_GetPropertyStr(g_ctx, item, "centerLook");
            dp_get_floats(v, cl, 3u);
            JS_FreeValue(g_ctx, v);

            /* centerNoRotate: NWU position of monitor before arc rotation */
            v = JS_GetPropertyStr(g_ctx, item, "centerNoRotate");
            dp_get_floats(v, cn, 3u);
            JS_FreeValue(g_ctx, v);

            v = JS_GetPropertyStr(g_ctx, item, "rotationAngleRadians");
            rot_x = dp_get_float(v, "x");
            rot_y = dp_get_float(v, "y");
            JS_FreeValue(g_ctx, v);

            JS_FreeValue(g_ctx, item);

            /*
             * NWU → GL: GL_X = -west * gl_scale, GL_Y = up * gl_scale, GL_Z = -north * gl_scale
             * (cl/cn are [north, west, up]).
             */
            placements[orig_idx].tx    = (float)(-cl[1] * gl_scale);
            placements[orig_idx].ty    = (float)( cl[2] * gl_scale);
            placements[orig_idx].tz    = (float)(-cl[0] * gl_scale);
            placements[orig_idx].cnx   = (float)(-cn[1] * gl_scale);
            placements[orig_idx].cny   = (float)( cn[2] * gl_scale);
            placements[orig_idx].cnz   = (float)(-cn[0] * gl_scale);
            placements[orig_idx].angle   = (float)rot_y;
            placements[orig_idx].angle_x = (float)rot_x;
            /* Keep NWU centerLook for findFocusedMonitor (pixel units, unscaled). */
            placements[orig_idx].ln_north = (float)cl[0];
            placements[orig_idx].ln_west  = (float)cl[1];
            placements[orig_idx].ln_up    = (float)cl[2];
        }
    }

    ret = 0;

cleanup:
    JS_FreeValue(g_ctx, call_result);
    JS_FreeValue(g_ctx, spacing);
    JS_FreeValue(g_ctx, monitor_arr);
    JS_FreeValue(g_ctx, fov_obj);
    return ret;
}

/* ----------------------------------------------------------------
 * dp_generate_mesh_vertices
 * ---------------------------------------------------------------- */

int dp_generate_mesh_vertices(const struct dp_fov_details *fov,
                               float mon_w, float mon_h,
                               const char *wrapping_scheme, int curved,
                               float cx, float cy, float cz,
                               float *verts_out, int max_verts)
{
    if (!fov || !verts_out || max_verts <= 0 || !g_ctx)
        return -1;

    /* Build a minimal fovDetails JS object from the C struct fields. */
    JSValue fov_obj = JS_NewObject(g_ctx);
    JS_SetPropertyStr(g_ctx, fov_obj, "completeScreenDistancePixels",
                      JS_NewFloat64(g_ctx, (double)fov->complete_dist_px));
    JS_SetPropertyStr(g_ctx, fov_obj, "defaultDistanceHorizontalRadians",
                      JS_NewFloat64(g_ctx, (double)fov->horizontal_radians));
    JS_SetPropertyStr(g_ctx, fov_obj, "defaultDistanceVerticalRadians",
                      JS_NewFloat64(g_ctx, (double)fov->vertical_radians));
    JS_SetPropertyStr(g_ctx, fov_obj, "widthPixels",
                      JS_NewFloat64(g_ctx, (double)fov->width_px));
    JS_SetPropertyStr(g_ctx, fov_obj, "heightPixels",
                      JS_NewFloat64(g_ctx, (double)fov->height_px));
    dp_set_size_adj(fov_obj, fov->size_adj_width, fov->size_adj_height);

    JSValue args[8] = {
        fov_obj,
        JS_NewFloat64(g_ctx, (double)mon_w),
        JS_NewFloat64(g_ctx, (double)mon_h),
        JS_NewString (g_ctx, wrapping_scheme ? wrapping_scheme : "flat"),
        JS_NewBool   (g_ctx, curved),
        JS_NewFloat64(g_ctx, (double)cx),
        JS_NewFloat64(g_ctx, (double)cy),
        JS_NewFloat64(g_ctx, (double)cz),
    };

    JSValue result = JS_Call(g_ctx, g_fn_generate_mesh_vertices,
                             JS_UNDEFINED, 8, args);
    JS_FreeValue(g_ctx, fov_obj);
    JS_FreeValue(g_ctx, args[3]); /* wrapping_scheme string */

    if (dp_is_exc(result, "generateMeshVertices")) {
        JS_FreeValue(g_ctx, result);
        return -1;
    }

    JSValue len_val = JS_GetPropertyStr(g_ctx, result, "length");
    uint32_t js_len = 0u;
    JS_ToUint32(g_ctx, &js_len, len_val);
    JS_FreeValue(g_ctx, len_val);

    int count = 0;
    for (uint32_t i = 0u; i < js_len && count < max_verts; i++) {
        JSValue vtx = JS_GetPropertyUint32(g_ctx, result, i);
        float *v = verts_out + (size_t)count * 5u;
        v[0] = (float)dp_get_float(vtx, "x");
        v[1] = (float)dp_get_float(vtx, "y");
        v[2] = (float)dp_get_float(vtx, "z");
        v[3] = (float)dp_get_float(vtx, "s");
        v[4] = (float)dp_get_float(vtx, "t");
        JS_FreeValue(g_ctx, vtx);
        count++;
    }

    JS_FreeValue(g_ctx, result);
    return count;
}

/* ----------------------------------------------------------------
 * dp_find_focused_monitor
 * ---------------------------------------------------------------- */

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
                            bool curved_display)
{
    if (!placements || !monitors || n == 0u || !g_ctx)
        return -1;
    if (n > DP_MAX_MONITORS)
        return -1;

    int result_index = -1;

    /* Rebuild fovDetails (needed by findFocusedMonitor for the angle math, and
     * for fullScreenDistancePixels to scale the head position into pixel units). */
    const char *scheme = (wrapping_scheme && *wrapping_scheme) ? wrapping_scheme : "horizontal";
    JSValue fov_obj = dp_build_fov_obj(device_width, device_height, diagonal_fov_rad,
                                       lens_distance_ratio, display_distance_default,
                                       scheme, curved_display);
    if (JS_IsException(fov_obj)) {
        JS_FreeValue(g_ctx, fov_obj);
        return -1;
    }
    dp_set_size_adj(fov_obj, size_adj_width, size_adj_height);
    /* findFocusedMonitor reads monitorWrappingScheme off fovDetails. */
    JS_SetPropertyStr(g_ctx, fov_obj, "monitorWrappingScheme", JS_NewString(g_ctx, scheme));

    /*
     * Convert the EUS head pose → NWU frame.  The monitor centerLook vectors are
     * in pixel units, so the 6DoF position (normalized by the driver) must be
     * scaled by fullScreenDistancePixels first — mirroring the references'
     * posePosition.times(fovDetails.fullScreenDistancePixels).  For 3DoF the
     * position is ~0 and the scale is a no-op.  Scaling commutes with the axis
     * permutation, so we apply it in the EUS frame before converting.
     */
    double full_screen_dist_px = dp_get_float(fov_obj, "fullScreenDistancePixels");
    double quat_eus[4] = { (double)qx, (double)qy, (double)qz, (double)qw };
    double pos_eus[3]  = { (double)pos_east  * full_screen_dist_px,
                           (double)pos_up    * full_screen_dist_px,
                           (double)pos_south * full_screen_dist_px };
    double quat_nwu[4];
    double pos_nwu[3];
    if (dp_call_convert(g_fn_eus_to_nwu_quat, quat_eus, 4u, quat_nwu) != 0 ||
        dp_call_convert(g_fn_eus_to_nwu_vector, pos_eus, 3u, pos_nwu) != 0) {
        JS_FreeValue(g_ctx, fov_obj);
        return -1;
    }

    /* monitorVectors: NWU centerLook arrays kept on the placements. */
    JSValue monitor_vectors = JS_NewArray(g_ctx);
    JSValue monitor_details = JS_NewArray(g_ctx);
    for (size_t i = 0u; i < n; i++) {
        double cl[3] = { (double)placements[i].ln_north,
                         (double)placements[i].ln_west,
                         (double)placements[i].ln_up };
        JS_SetPropertyUint32(g_ctx, monitor_vectors, (uint32_t)i, dp_new_floats(cl, 3u));

        JSValue d = JS_NewObject(g_ctx);
        JS_SetPropertyStr(g_ctx, d, "x",      JS_NewInt32(g_ctx,  monitors[i].x));
        JS_SetPropertyStr(g_ctx, d, "y",      JS_NewInt32(g_ctx,  monitors[i].y));
        JS_SetPropertyStr(g_ctx, d, "width",  JS_NewUint32(g_ctx, monitors[i].width));
        JS_SetPropertyStr(g_ctx, d, "height", JS_NewUint32(g_ctx, monitors[i].height));
        JS_SetPropertyUint32(g_ctx, monitor_details, (uint32_t)i, d);
    }

    JSValue quat_arr = dp_new_floats(quat_nwu, 4u);
    JSValue pos_arr  = dp_new_floats(pos_nwu, 3u);

    JSValue args[8] = {
        quat_arr,
        pos_arr,
        monitor_vectors,
        JS_NewInt32(g_ctx, current_focused_index),
        JS_NewFloat64(g_ctx, (double)focused_monitor_distance),
        JS_NewBool(g_ctx, smooth_follow_enabled ? 1 : 0),
        fov_obj,
        monitor_details
    };
    JSValue res = JS_Call(g_ctx, g_fn_find_focused_monitor, JS_UNDEFINED, 8, args);

    if (!dp_is_exc(res, "findFocusedMonitor")) {
        int32_t idx = -1;
        JS_ToInt32(g_ctx, &idx, res);
        result_index = idx;
    }

    JS_FreeValue(g_ctx, res);
    JS_FreeValue(g_ctx, quat_arr);
    JS_FreeValue(g_ctx, pos_arr);
    JS_FreeValue(g_ctx, monitor_vectors);
    JS_FreeValue(g_ctx, monitor_details);
    JS_FreeValue(g_ctx, fov_obj);
    return result_index;
}

/* ----------------------------------------------------------------
 * dp_ease_distance — thin wrapper over the shared easeDistance().
 * ---------------------------------------------------------------- */

int dp_ease_distance(float start_distance, float target_distance,
                     float elapsed_ms, bool gaining_focus,
                     bool needs_sequence_delay,
                     float *out_distance, bool *out_done)
{
    if (!out_distance || !g_ctx)
        return -1;

    JSValue args[5] = {
        JS_NewFloat64(g_ctx, (double)start_distance),
        JS_NewFloat64(g_ctx, (double)target_distance),
        JS_NewFloat64(g_ctx, (double)elapsed_ms),
        JS_NewBool(g_ctx, gaining_focus ? 1 : 0),
        JS_NewBool(g_ctx, needs_sequence_delay ? 1 : 0)
    };
    JSValue res = JS_Call(g_ctx, g_fn_ease_distance, JS_UNDEFINED, 5, args);
    if (dp_is_exc(res, "easeDistance")) {
        JS_FreeValue(g_ctx, res);
        return -1;
    }

    *out_distance = (float)dp_get_float(res, "distance");
    if (out_done) {
        JSValue dn = JS_GetPropertyStr(g_ctx, res, "done");
        *out_done = JS_ToBool(g_ctx, dn) ? true : false;
        JS_FreeValue(g_ctx, dn);
    }
    JS_FreeValue(g_ctx, res);
    return 0;
}
