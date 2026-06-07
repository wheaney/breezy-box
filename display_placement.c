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
    g_fn_monitors_to_placements = JS_GetPropertyStr(g_ctx, global, "monitorsToPlacements");
    g_fn_diagonal_to_cross_fovs = JS_GetPropertyStr(g_ctx, global, "diagonalToCrossFOVs");
    g_fn_build_fov_details       = JS_GetPropertyStr(g_ctx, global, "buildFovDetails");
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

    return 0;
}

void dp_destroy(void)
{
    if (!g_ctx)
        return;
    JS_FreeValue(g_ctx, g_fn_monitors_to_placements);
    JS_FreeValue(g_ctx, g_fn_diagonal_to_cross_fovs);
    JS_FreeValue(g_ctx, g_fn_build_fov_details);
    g_fn_monitors_to_placements = JS_UNDEFINED;
    g_fn_diagonal_to_cross_fovs = JS_UNDEFINED;
    g_fn_build_fov_details       = JS_UNDEFINED;
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
    args[5] = JS_NewString(g_ctx, "horizontal");
    args[6] = JS_NewBool(g_ctx, 1);
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

        READ_FIELD("completeScreenDistancePixels", complete_dist_px);
        READ_FIELD("defaultDistanceHorizontalRadians", horizontal_radians);
        READ_FIELD("defaultDistanceVerticalRadians",   vertical_radians);
        READ_FIELD("lensDistancePixels",               lens_distance_px);
        READ_FIELD("fullScreenDistancePixels",         full_screen_dist_px);
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
                          float arc_radius_gl,
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
     * Extract completeScreenDistancePixels to derive the GL unit scale.
     */
    {
        JSValue bfd_args[7];
        JSValue bfd_result;
        JSValue v;

        bfd_args[0] = JS_NewFloat64(g_ctx, (double)device_width);
        bfd_args[1] = JS_NewFloat64(g_ctx, (double)device_height);
        bfd_args[2] = JS_NewFloat64(g_ctx, (double)diagonal_fov_rad);
        bfd_args[3] = JS_NewFloat64(g_ctx, (double)lens_distance_ratio);
        bfd_args[4] = JS_NewFloat64(g_ctx, 1.0);   /* defaultDisplayDistance */
        bfd_args[5] = JS_NewString(g_ctx, "horizontal");
        bfd_args[6] = JS_NewBool(g_ctx, 1);        /* curvedDisplay */
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

    spacing = JS_NewFloat64(g_ctx, 0.0);

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
