/*
 ***********************************************************
 * File   : lateral.cpp
 *
 * X/Y stillness detector. See lateral.h for the design and every constant.
 ***********************************************************
 */

#include "lateral.h"

LateralMonitor lat_monitor;

LateralMonitor::LateralMonitor()
{
    last_ax     = 0.0f;
    last_ay     = 0.0f;
    last_ms     = 0;
    have_last   = false;
    last_m      = 0.0f;
    quiet       = 0;
    calibrating = false;
    bucket_start = 0;
    bucket_max  = 0.0f;
    bucket_used = false;
    peak        = 0.0f;
    n_buckets   = 0;
    xy_still    = 0.0f;
}

void LateralMonitor::calib_begin(uint32_t now_ms)
{
    calibrating  = true;
    bucket_start = now_ms;
    bucket_max   = 0.0f;
    bucket_used  = false;
    peak         = 0.0f;
    n_buckets    = 0;
    xy_still     = 0.0f;
    quiet        = 0;

    /*
     * Drop the anchor as well. A calibration that begins after the FSM has
     * been running would otherwise difference its first sample against one
     * taken an unknown time ago.
     */
    have_last    = false;
}

void LateralMonitor::add(float ax, float ay, uint32_t now_ms)
{
    uint32_t dt;
    float    m;

    if (!have_last) {
        last_ax   = ax;
        last_ay   = ay;
        last_ms   = now_ms;
        have_last = true;
        return;
    }

    dt = now_ms - last_ms;
    m  = fabs(ax - last_ax) + fabs(ay - last_ay);

    last_ax = ax;
    last_ay = ay;
    last_ms = now_ms;

    /*
     * Gaps too long to difference produce no metric at all -- not a zero.
     * Crediting a blanked stretch as quiet is the one arithmetic mistake in
     * this file that could release a beacon on a moving counterweight.
     */
    if (dt == 0 || dt > XY_MAX_DT_MS) {
        return;
    }

    last_m = m;

    /*
     * The quiet run only advances on a real metric, and any metric at or
     * above the threshold resets it to zero -- so XY_RELEASE_POLLS means
     * CONSECUTIVE quiet observations, not "quiet more often than not".
     */
    if (xy_still > 0.0f) {
        if (m < xy_still) {
            if (quiet < 0xFF) {
                quiet++;
            }
        } else {
            quiet = 0;
        }
    }

    if (!calibrating) {
        return;
    }

    /*
     * Bucket accounting. A bucket is closed by the arrival of a sample past
     * its end rather than by wall clock, so a bucket that received nothing at
     * all is never recorded -- which is what makes the XY_CALIB_MIN_BUCKETS
     * quorum able to notice a stalled timer or a silent sensor.
     */
    while ((now_ms - bucket_start) >= XY_CALIB_BUCKET_MS) {
        if (bucket_used && n_buckets < XY_CALIB_BUCKETS) {
            bmax[n_buckets++] = bucket_max;
        }
        bucket_start += XY_CALIB_BUCKET_MS;
        bucket_max    = 0.0f;
        bucket_used   = false;
    }

    if (!bucket_used || m > bucket_max) {
        bucket_max  = m;
        bucket_used = true;
    }
}

void LateralMonitor::arm_fallback()
{
    calibrating = false;
    xy_still    = XY_STILL_FALLBACK;
    quiet       = 0;
}

bool LateralMonitor::calib_finish()
{
    uint8_t i, j;
    float   v;
    float   srt[XY_CALIB_BUCKETS];

    calibrating = false;

    /* Close whatever bucket is still open. */
    if (bucket_used && n_buckets < XY_CALIB_BUCKETS) {
        bmax[n_buckets++] = bucket_max;
    }

    if (n_buckets == 0) {
        return false;
    }

    /*
     * ⚠️ SORT A COPY. bmax[] used to be sorted IN PLACE, which destroyed the
     * time order of the per-second maxima -- and time order is the whole point
     * of the log dump added 2026-08-14 (§XY-BMAX in movement_service.cpp).
     * Deciding whether a 5 s window is safe means asking what the FIRST five
     * buckets would have produced, and a sorted array cannot answer that.
     *
     * Costs XY_CALIB_BUCKETS floats of stack, transiently, in loop() context
     * with no ISR below it. The alternative was printing from inside this
     * class, which would put Serial in a file that is otherwise pure
     * arithmetic.
     */
    for (i = 0; i < n_buckets; i++) {
        srt[i] = bmax[i];
    }

    /* Insertion sort; n <= XY_CALIB_BUCKETS. */
    for (i = 1; i < n_buckets; i++) {
        v = srt[i];
        for (j = i; j > 0 && srt[j - 1] > v; j--) {
            srt[j] = srt[j - 1];
        }
        srt[j] = v;
    }

    /*
     * Set before the quorum check so a rejected window still reports a real
     * number in the log. A rejection that prints peak=0.0000 looks like a
     * dead sensor and would send the next hour in the wrong direction.
     */
    peak = srt[n_buckets - 1];

    if (n_buckets < XY_CALIB_MIN_BUCKETS) {
        return false;
    }

    if (peak > XY_CALIB_MOVE_MSS) {
        /* Something moved. A threshold learned from this is wrong upward. */
        return false;
    }

    /*
     * Lower-middle element on an even count. The lower of the two is the
     * smaller threshold, which is the safe direction.
     */
    v = srt[(n_buckets - 1) / 2] * XY_STILL_MARGIN;

    if (v < XY_STILL_MIN) {
        v = XY_STILL_MIN;
    } else if (v > XY_STILL_MAX) {
        v = XY_STILL_MAX;
    }

    xy_still = v;
    quiet    = 0;

    return true;
}
