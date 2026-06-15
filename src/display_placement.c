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
    if (JS_IsException(eval_result)) {
        JSValue exc = JS_GetException(g_ctx);
        const char *msg = JS_ToCString(g_ctx, exc);
        fprintf(stderr, "dp: JS bundle eval error: %s\n", msg ? msg : "(unknown)");
        JS_FreeCString(g_ctx, msg);
        JS_FreeValue(g_ctx, exc);
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

    if (!JS_IsFunction(g_ctx, g_fn_monitors_to_placements)) {
        fprintf(stderr, "dp: monitorsToPlacements not found in JS bundle\n");
        dp_destroy();
        return -1;
    }
    if (!JS_IsFunction(g_ctx, g_fn_diagonal_to_cross_fovs)) {
        fprintf(stderr, "dp: diagonalToCrossFOVs not found in JS bundle\n");
        dp_destroy();
        return -1;
    }
    if (!JS_IsFunction(g_ctx, g_fn_build_fov_details)) {
        fprintf(stderr, "dp: buildFovDetails not found in JS bundle\n");
        dp_destroy();
        return -1;
    }
    if (!JS_IsFunction(g_ctx, g_fn_generate_mesh_vertices)) {
        fprintf(stderr, "dp: generateMeshVertices not found in JS bundle\n");
        dp_destroy();
        return -1;
    }
    if (!JS_IsFunction(g_ctx, g_fn_find_focused_monitor)) {
        fprintf(stderr, "dp: findFocusedMonitor not found in JS bundle\n");
        dp_destroy();
        return -1;
    }
    if (!JS_IsFunction(g_ctx, g_fn_eus_to_nwu_quat) ||
        !JS_IsFunction(g_ctx, g_fn_eus_to_nwu_vector)) {
        fprintf(stderr, "dp: eusToNwu* not found in JS bundle\n");
        dp_destroy();
        return -1;
    }
    if (!JS_IsFunction(g_ctx, g_fn_ease_distance)) {
        fprintf(stderr, "dp: easeDistance not found in JS bundle\n");
        dp_destroy();
        return -1;
    }

    return 0;
}

void dp_destroy(void)
{
    if (!g_ctx)
        return;
    JS_FreeValue(g_ctx, g_fn_monitors_to_placements);
    JS_FreeValue(g_ctx, g_fn_diagonal_to_cross_fovs);
    JS_FreeValue(g_ctx, g_fn_build_fov_details);
    JS_FreeValue(g_ctx, g_fn_generate_mesh_vertices);
    JS_FreeValue(g_ctx, g_fn_find_focused_monitor);
    JS_FreeValue(g_ctx, g_fn_eus_to_nwu_quat);
    JS_FreeValue(g_ctx, g_fn_eus_to_nwu_vector);
    JS_FreeValue(g_ctx, g_fn_ease_distance);
    g_fn_monitors_to_placements  = JS_UNDEFINED;
    g_fn_diagonal_to_cross_fovs  = JS_UNDEFINED;
    g_fn_build_fov_details        = JS_UNDEFINED;
    g_fn_generate_mesh_vertices   = JS_UNDEFINED;
    g_fn_find_focused_monitor     = JS_UNDEFINED;
    g_fn_eus_to_nwu_quat          = JS_UNDEFINED;
    g_fn_eus_to_nwu_vector        = JS_UNDEFINED;
    g_fn_ease_distance            = JS_UNDEFINED;
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
    int ret = -1;

    if (!out || !g_ctx)
        return -1;

    args[0] = JS_NewFloat64(g_ctx, (double)diagonal_radians);
    args[1] = JS_NewFloat64(g_ctx, (double)aspect_ratio);
    result  = JS_Call(g_ctx, g_fn_diagonal_to_cross_fovs, JS_UNDEFINED, 2, args);

    if (JS_IsException(result)) {
        JSValue exc = JS_GetException(g_ctx);
        const char *msg = JS_ToCString(g_ctx, exc);
        fprintf(stderr, "dp: diagonalToCrossFOVs error: %s\n", msg ? msg : "(unknown)");
        JS_FreeCString(g_ctx, msg);
        JS_FreeValue(g_ctx, exc);
        goto cleanup;
    }

    {
        JSValue v;
        double d;

#define READ_FIELD(field, member) \
        v = JS_GetPropertyStr(g_ctx, result, field); \
        JS_ToFloat64(g_ctx, &d, v); \
        JS_FreeValue(g_ctx, v); \
        out->member = (float)d

        READ_FIELD("horizontalRadians",         horizontal_radians);
        READ_FIELD("verticalRadians",            vertical_radians);
        READ_FIELD("diagonalLengthUnitDistance", diagonal_length_unit);
        READ_FIELD("widthUnitDistance",          width_unit);
        READ_FIELD("heightUnitDistance",         height_unit);
#undef READ_FIELD
    }
    ret = 0;

cleanup:
    JS_FreeValue(g_ctx, result);
    return ret;
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
    JSValue args[7], result;
    int ret = -1;

    if (!out || !g_ctx)
        return -1;

    args[0] = JS_NewFloat64(g_ctx, (double)device_width);
    args[1] = JS_NewFloat64(g_ctx, (double)device_height);
    args[2] = JS_NewFloat64(g_ctx, (double)diagonal_fov_rad);
    args[3] = JS_NewFloat64(g_ctx, (double)lens_distance_ratio);
    args[4] = JS_NewFloat64(g_ctx, (double)default_display_distance);
    args[5] = JS_NewString(g_ctx, wrapping_scheme ? wrapping_scheme : "horizontal");
    args[6] = JS_NewBool(g_ctx, curved_display ? 1 : 0);
    result  = JS_Call(g_ctx, g_fn_build_fov_details, JS_UNDEFINED, 7, args);
    JS_FreeValue(g_ctx, args[5]);

    if (JS_IsException(result)) {
        JSValue exc = JS_GetException(g_ctx);
        const char *msg = JS_ToCString(g_ctx, exc);
        fprintf(stderr, "dp: buildFovDetails error: %s\n", msg ? msg : "(unknown)");
        JS_FreeCString(g_ctx, msg);
        JS_FreeValue(g_ctx, exc);
        goto cleanup;
    }

    {
        JSValue v;
        double d;

#define READ_FIELD(field, member) \
        v = JS_GetPropertyStr(g_ctx, result, field); \
        JS_ToFloat64(g_ctx, &d, v); \
        JS_FreeValue(g_ctx, v); \
        out->member = (float)d

        READ_FIELD("completeScreenDistancePixels",      complete_dist_px);
        READ_FIELD("defaultDistanceHorizontalRadians",  horizontal_radians);
        READ_FIELD("defaultDistanceVerticalRadians",    vertical_radians);
        READ_FIELD("lensDistancePixels",                lens_distance_px);
        READ_FIELD("fullScreenDistancePixels",          full_screen_dist_px);
        READ_FIELD("widthPixels",                       width_px);
        READ_FIELD("heightPixels",                      height_px);
        /* size_adj_* default to device dims; caller overrides if display_size != 1 */
        READ_FIELD("sizeAdjustedWidthPixels",           size_adj_width);
        READ_FIELD("sizeAdjustedHeightPixels",          size_adj_height);
#undef READ_FIELD
    }
    ret = 0;

cleanup:
    JS_FreeValue(g_ctx, result);
    return ret;
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
    int32_t auto_x[DP_MAX_MONITORS];
    bool use_auto_x;
    double complete_dist_px;
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

    /* If all x=0 and multiple monitors, assign x cumulatively left-to-right. */
    use_auto_x = (n > 1u);
    for (i = 0u; i < n && use_auto_x; i++) {
        if (monitors[i].x != 0)
            use_auto_x = false;
    }
    if (use_auto_x) {
        auto_x[0] = 0;
        for (i = 1u; i < n; i++)
            auto_x[i] = auto_x[i - 1] + (int32_t)monitors[i - 1].width;
    } else {
        for (i = 0u; i < n; i++)
            auto_x[i] = monitors[i].x;
    }

    /*
     * Delegate fovDetails construction entirely to the shared JS buildFovDetails().
     * Then override sizeAdjustedWidth/HeightPixels with the caller-computed values so
     * monitorsToPlacements uses the correct size-adjusted viewport reference dimensions.
     * Extract completeScreenDistancePixels to derive the GL unit scale.
     */
    {
        JSValue bfd_args[7];
        JSValue bfd_result;
        JSValue v;
        const char *scheme = (wrapping_scheme && *wrapping_scheme) ? wrapping_scheme : "horizontal";

        bfd_args[0] = JS_NewFloat64(g_ctx, (double)device_width);
        bfd_args[1] = JS_NewFloat64(g_ctx, (double)device_height);
        bfd_args[2] = JS_NewFloat64(g_ctx, (double)diagonal_fov_rad);
        bfd_args[3] = JS_NewFloat64(g_ctx, (double)lens_distance_ratio);
        bfd_args[4] = JS_NewFloat64(g_ctx, (double)display_distance_default);
        bfd_args[5] = JS_NewString(g_ctx, scheme);
        bfd_args[6] = JS_NewBool(g_ctx, curved_display ? 1 : 0);
        bfd_result  = JS_Call(g_ctx, g_fn_build_fov_details, JS_UNDEFINED, 7, bfd_args);
        JS_FreeValue(g_ctx, bfd_args[5]);

        if (JS_IsException(bfd_result)) {
            JSValue exc = JS_GetException(g_ctx);
            const char *msg = JS_ToCString(g_ctx, exc);
            fprintf(stderr, "dp: buildFovDetails error: %s\n", msg ? msg : "(unknown)");
            JS_FreeCString(g_ctx, msg);
            JS_FreeValue(g_ctx, exc);
            JS_FreeValue(g_ctx, bfd_result);
            return -1;
        }

        v = JS_GetPropertyStr(g_ctx, bfd_result, "completeScreenDistancePixels");
        JS_ToFloat64(g_ctx, &complete_dist_px, v);
        JS_FreeValue(g_ctx, v);

        /* Override size-adjusted dimensions so monitorsToPlacements computes arc
         * extents relative to the scaled viewport, not the raw device resolution. */
        JS_SetPropertyStr(g_ctx, bfd_result, "sizeAdjustedWidthPixels",
                          JS_NewFloat64(g_ctx, (double)size_adj_width));
        JS_SetPropertyStr(g_ctx, bfd_result, "sizeAdjustedHeightPixels",
                          JS_NewFloat64(g_ctx, (double)size_adj_height));

        fov_obj = bfd_result; /* transferred; freed in cleanup */
    }

    /* Uniform scale from pixel-space NWU coords to GL units. */
    gl_scale = (double)arc_radius_gl / complete_dist_px;

    monitor_arr = JS_NewArray(g_ctx);
    for (i = 0u; i < n; i++) {
        JSValue mon = JS_NewObject(g_ctx);
        JS_SetPropertyStr(g_ctx, mon, "x",      JS_NewInt32(g_ctx,  auto_x[i]));
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

    if (JS_IsException(call_result)) {
        JSValue exc = JS_GetException(g_ctx);
        const char *msg = JS_ToCString(g_ctx, exc);
        fprintf(stderr, "dp: monitorsToPlacements error: %s\n", msg ? msg : "(unknown)");
        JS_FreeCString(g_ctx, msg);
        JS_FreeValue(g_ctx, exc);
        goto cleanup;
    }

    {
        JSValue len_val = JS_GetPropertyStr(g_ctx, call_result, "length");
        uint32_t result_len = 0u;
        JS_ToUint32(g_ctx, &result_len, len_val);
        JS_FreeValue(g_ctx, len_val);

        for (i = 0u; i < (size_t)result_len && i < n; i++) {
            JSValue item, orig_val, cl, cn, rot, rot_y_val;
            JSValue v0, v1, v2;
            int32_t orig_idx;
            double cl_north, cl_west, cl_up;
            double cn_north, cn_west, cn_up;
            double rot_y;

            item = JS_GetPropertyUint32(g_ctx, call_result, (uint32_t)i);

            orig_val = JS_GetPropertyStr(g_ctx, item, "originalIndex");
            JS_ToInt32(g_ctx, &orig_idx, orig_val);
            JS_FreeValue(g_ctx, orig_val);

            if (orig_idx < 0 || (size_t)orig_idx >= n) {
                JS_FreeValue(g_ctx, item);
                continue;
            }

            /* centerLook: NWU position with arc rotation applied */
            cl  = JS_GetPropertyStr(g_ctx, item, "centerLook");
            v0  = JS_GetPropertyUint32(g_ctx, cl, 0u);
            v1  = JS_GetPropertyUint32(g_ctx, cl, 1u);
            v2  = JS_GetPropertyUint32(g_ctx, cl, 2u);
            JS_ToFloat64(g_ctx, &cl_north, v0);
            JS_ToFloat64(g_ctx, &cl_west,  v1);
            JS_ToFloat64(g_ctx, &cl_up,    v2);
            JS_FreeValue(g_ctx, v0);
            JS_FreeValue(g_ctx, v1);
            JS_FreeValue(g_ctx, v2);
            JS_FreeValue(g_ctx, cl);

            /* centerNoRotate: NWU position of monitor before arc rotation */
            cn  = JS_GetPropertyStr(g_ctx, item, "centerNoRotate");
            v0  = JS_GetPropertyUint32(g_ctx, cn, 0u);
            v1  = JS_GetPropertyUint32(g_ctx, cn, 1u);
            v2  = JS_GetPropertyUint32(g_ctx, cn, 2u);
            JS_ToFloat64(g_ctx, &cn_north, v0);
            JS_ToFloat64(g_ctx, &cn_west,  v1);
            JS_ToFloat64(g_ctx, &cn_up,    v2);
            JS_FreeValue(g_ctx, v0);
            JS_FreeValue(g_ctx, v1);
            JS_FreeValue(g_ctx, v2);
            JS_FreeValue(g_ctx, cn);

            rot       = JS_GetPropertyStr(g_ctx, item, "rotationAngleRadians");
            rot_y_val = JS_GetPropertyStr(g_ctx, rot, "y");
            JS_ToFloat64(g_ctx, &rot_y, rot_y_val);
            JS_FreeValue(g_ctx, rot_y_val);
            JS_FreeValue(g_ctx, rot);

            JS_FreeValue(g_ctx, item);

            /*
             * NWU → GL: GL_X = -west * gl_scale, GL_Y = up * gl_scale, GL_Z = -north * gl_scale
             */
            placements[orig_idx].tx    = (float)(-cl_west  * gl_scale);
            placements[orig_idx].ty    = (float)( cl_up    * gl_scale);
            placements[orig_idx].tz    = (float)(-cl_north * gl_scale);
            placements[orig_idx].cnx   = (float)(-cn_west  * gl_scale);
            placements[orig_idx].cny   = (float)( cn_up    * gl_scale);
            placements[orig_idx].cnz   = (float)(-cn_north * gl_scale);
            placements[orig_idx].angle = (float)( rot_y);
            /* Keep NWU centerLook for findFocusedMonitor (pixel units, unscaled). */
            placements[orig_idx].ln_north = (float)cl_north;
            placements[orig_idx].ln_west  = (float)cl_west;
            placements[orig_idx].ln_up    = (float)cl_up;
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
    JS_SetPropertyStr(g_ctx, fov_obj, "sizeAdjustedWidthPixels",
                      JS_NewFloat64(g_ctx, (double)fov->size_adj_width));
    JS_SetPropertyStr(g_ctx, fov_obj, "sizeAdjustedHeightPixels",
                      JS_NewFloat64(g_ctx, (double)fov->size_adj_height));

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

    if (JS_IsException(result)) {
        JSValue exc = JS_GetException(g_ctx);
        const char *msg = JS_ToCString(g_ctx, exc);
        fprintf(stderr, "dp: generateMeshVertices error: %s\n", msg ? msg : "(unknown)");
        JS_FreeCString(g_ctx, msg);
        JS_FreeValue(g_ctx, exc);
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
        JSValue vx  = JS_GetPropertyStr(g_ctx, vtx, "x");
        JSValue vy  = JS_GetPropertyStr(g_ctx, vtx, "y");
        JSValue vz  = JS_GetPropertyStr(g_ctx, vtx, "z");
        JSValue vs  = JS_GetPropertyStr(g_ctx, vtx, "s");
        JSValue vt_ = JS_GetPropertyStr(g_ctx, vtx, "t");
        double dx, dy, dz, ds, dt;
        JS_ToFloat64(g_ctx, &dx, vx);
        JS_ToFloat64(g_ctx, &dy, vy);
        JS_ToFloat64(g_ctx, &dz, vz);
        JS_ToFloat64(g_ctx, &ds, vs);
        JS_ToFloat64(g_ctx, &dt, vt_);
        JS_FreeValue(g_ctx, vx); JS_FreeValue(g_ctx, vy); JS_FreeValue(g_ctx, vz);
        JS_FreeValue(g_ctx, vs); JS_FreeValue(g_ctx, vt_); JS_FreeValue(g_ctx, vtx);

        float *v = verts_out + (size_t)count * 5u;
        v[0] = (float)dx;
        v[1] = (float)dy;
        v[2] = (float)dz;
        v[3] = (float)ds;
        v[4] = (float)dt;
        count++;
    }

    JS_FreeValue(g_ctx, result);
    return count;
}

/* ----------------------------------------------------------------
 * dp_find_focused_monitor
 * ---------------------------------------------------------------- */

/* Build a JS Float array [a, b, c] from a C array. */
static JSValue dp_new_num_array(const double *vals, size_t n)
{
    JSValue arr = JS_NewArray(g_ctx);
    for (size_t i = 0u; i < n; i++)
        JS_SetPropertyUint32(g_ctx, arr, (uint32_t)i, JS_NewFloat64(g_ctx, vals[i]));
    return arr;
}

/* Call a single-arg JS conversion fn (eusToNwu*) returning a length-n array into out. */
static int dp_call_convert(JSValue fn, const double *in, size_t n, double *out)
{
    JSValue arg = dp_new_num_array(in, n);
    JSValue res = JS_Call(g_ctx, fn, JS_UNDEFINED, 1, &arg);
    JS_FreeValue(g_ctx, arg);
    if (JS_IsException(res)) {
        JS_FreeValue(g_ctx, res);
        return -1;
    }
    for (size_t i = 0u; i < n; i++) {
        JSValue v = JS_GetPropertyUint32(g_ctx, res, (uint32_t)i);
        JS_ToFloat64(g_ctx, &out[i], v);
        JS_FreeValue(g_ctx, v);
    }
    JS_FreeValue(g_ctx, res);
    return 0;
}

int dp_find_focused_monitor(const struct dp_placement *placements,
                            const struct dp_monitor_info *monitors,
                            size_t n,
                            float qw, float qx, float qy, float qz,
                            float pos_east, float pos_up, float pos_south,
                            int current_focused_index,
                            float focused_monitor_distance,
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

    /* Convert EUS head pose → NWU frame using the shared helpers. */
    double quat_eus[4] = { (double)qx, (double)qy, (double)qz, (double)qw };
    double pos_eus[3]  = { (double)pos_east, (double)pos_up, (double)pos_south };
    double quat_nwu[4];
    double pos_nwu[3];
    if (dp_call_convert(g_fn_eus_to_nwu_quat, quat_eus, 4u, quat_nwu) != 0)
        return -1;
    if (dp_call_convert(g_fn_eus_to_nwu_vector, pos_eus, 3u, pos_nwu) != 0)
        return -1;

    /* Rebuild fovDetails (needed by findFocusedMonitor for the angle math). */
    JSValue fov_obj = JS_UNDEFINED;
    {
        JSValue bfd_args[7];
        const char *scheme = (wrapping_scheme && *wrapping_scheme) ? wrapping_scheme : "horizontal";
        bfd_args[0] = JS_NewFloat64(g_ctx, (double)device_width);
        bfd_args[1] = JS_NewFloat64(g_ctx, (double)device_height);
        bfd_args[2] = JS_NewFloat64(g_ctx, (double)diagonal_fov_rad);
        bfd_args[3] = JS_NewFloat64(g_ctx, (double)lens_distance_ratio);
        bfd_args[4] = JS_NewFloat64(g_ctx, (double)display_distance_default);
        bfd_args[5] = JS_NewString(g_ctx, scheme);
        bfd_args[6] = JS_NewBool(g_ctx, curved_display ? 1 : 0);
        fov_obj = JS_Call(g_ctx, g_fn_build_fov_details, JS_UNDEFINED, 7, bfd_args);
        JS_FreeValue(g_ctx, bfd_args[5]);
        if (JS_IsException(fov_obj)) {
            JS_FreeValue(g_ctx, fov_obj);
            return -1;
        }
        JS_SetPropertyStr(g_ctx, fov_obj, "sizeAdjustedWidthPixels",
                          JS_NewFloat64(g_ctx, (double)size_adj_width));
        JS_SetPropertyStr(g_ctx, fov_obj, "sizeAdjustedHeightPixels",
                          JS_NewFloat64(g_ctx, (double)size_adj_height));
        /* findFocusedMonitor reads monitorWrappingScheme off fovDetails. */
        JS_SetPropertyStr(g_ctx, fov_obj, "monitorWrappingScheme",
                          JS_NewString(g_ctx, scheme));
    }

    /* monitorVectors: NWU centerLook arrays kept on the placements. */
    JSValue monitor_vectors = JS_NewArray(g_ctx);
    JSValue monitor_details = JS_NewArray(g_ctx);
    for (size_t i = 0u; i < n; i++) {
        double cl[3] = { (double)placements[i].ln_north,
                         (double)placements[i].ln_west,
                         (double)placements[i].ln_up };
        JS_SetPropertyUint32(g_ctx, monitor_vectors, (uint32_t)i, dp_new_num_array(cl, 3u));

        JSValue d = JS_NewObject(g_ctx);
        JS_SetPropertyStr(g_ctx, d, "x",      JS_NewInt32(g_ctx,  monitors[i].x));
        JS_SetPropertyStr(g_ctx, d, "y",      JS_NewInt32(g_ctx,  monitors[i].y));
        JS_SetPropertyStr(g_ctx, d, "width",  JS_NewUint32(g_ctx, monitors[i].width));
        JS_SetPropertyStr(g_ctx, d, "height", JS_NewUint32(g_ctx, monitors[i].height));
        JS_SetPropertyUint32(g_ctx, monitor_details, (uint32_t)i, d);
    }

    JSValue quat_arr = dp_new_num_array(quat_nwu, 4u);
    JSValue pos_arr  = dp_new_num_array(pos_nwu, 3u);

    JSValue args[8] = {
        quat_arr,
        pos_arr,
        monitor_vectors,
        JS_NewInt32(g_ctx, current_focused_index),
        JS_NewFloat64(g_ctx, (double)focused_monitor_distance),
        JS_NewBool(g_ctx, 0),               /* smoothFollowEnabled = false */
        fov_obj,
        monitor_details
    };
    JSValue res = JS_Call(g_ctx, g_fn_find_focused_monitor, JS_UNDEFINED, 8, args);

    if (JS_IsException(res)) {
        JSValue exc = JS_GetException(g_ctx);
        const char *msg = JS_ToCString(g_ctx, exc);
        fprintf(stderr, "dp: findFocusedMonitor error: %s\n", msg ? msg : "(unknown)");
        JS_FreeCString(g_ctx, msg);
        JS_FreeValue(g_ctx, exc);
    } else {
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
    if (JS_IsException(res)) {
        JS_FreeValue(g_ctx, res);
        return -1;
    }

    JSValue dv = JS_GetPropertyStr(g_ctx, res, "distance");
    JSValue dn = JS_GetPropertyStr(g_ctx, res, "done");
    double d = (double)target_distance;
    JS_ToFloat64(g_ctx, &d, dv);
    *out_distance = (float)d;
    if (out_done)
        *out_done = JS_ToBool(g_ctx, dn) ? true : false;
    JS_FreeValue(g_ctx, dv);
    JS_FreeValue(g_ctx, dn);
    JS_FreeValue(g_ctx, res);
    return 0;
}
