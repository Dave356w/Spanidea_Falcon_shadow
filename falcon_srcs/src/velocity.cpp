/*
 * velocity.cpp -- windowed velocity integral. See velocity.h for the reasoning,
 * the measured noise figures, and the five guards this implements.
 */

#include "velocity.h"

VelocityWindow vel_window;

VelocityWindow::VelocityWindow()
{
    for (uint8_t i = 0; i < VEL_BINS; i++) {
        bin_v[i]   = 0.0f;
        bin_cov[i] = 0;
    }

    cur            = 0;
    cur_bin_start  = 0;
    last_sample_ms = 0;
    have_last      = false;

    base           = 0.0f;
    prime_count    = 0;
    w_high_since   = 0;
    track_baseline = true;

    w_cache        = 0.0f;
    total_cov      = 0;
}

void VelocityWindow::reset(uint32_t now_ms)
{
    for (uint8_t i = 0; i < VEL_BINS; i++) {
        bin_v[i]   = 0.0f;
        bin_cov[i] = 0;
    }

    cur           = 0;
    cur_bin_start = now_ms;
    w_cache       = 0.0f;
    total_cov     = 0;

    /*
     * Deliberately NOT clearing have_last / last_sample_ms: the next sample
     * should still be credited a sane dt rather than being treated as the
     * first one ever seen. The baseline and its prime count also survive --
     * they are the expensive things to rebuild, and a reset means "forget the
     * window", not "forget what level the device sits at".
     */
}

/*
 * Roll the ring forward to the bin containing now_ms, zeroing everything
 * stepped over.
 *
 * Bins that are skipped because no sample arrived contribute ZERO velocity
 * rather than an extrapolation of the last known acceleration (guard 5). Their
 * coverage is zero too, so a window full of gaps fails valid() instead of
 * quietly reporting a number derived from almost no data.
 */
void VelocityWindow::advance_to(uint32_t now_ms)
{
    uint32_t elapsed = now_ms - cur_bin_start;

    if (elapsed < VEL_BIN_MS) {
        return;
    }

    uint32_t steps = elapsed / VEL_BIN_MS;

    if (steps >= VEL_BINS) {
        /* Nothing in the window survives. */
        for (uint8_t i = 0; i < VEL_BINS; i++) {
            bin_v[i]   = 0.0f;
            bin_cov[i] = 0;
        }
        cur           = 0;
        cur_bin_start = now_ms;
        return;
    }

    for (uint32_t s = 0; s < steps; s++) {
        cur = (uint8_t)((cur + 1) % VEL_BINS);
        bin_v[cur]   = 0.0f;
        bin_cov[cur] = 0;
    }

    cur_bin_start += steps * VEL_BIN_MS;
}

void VelocityWindow::tick(uint32_t now_ms)
{
    advance_to(now_ms);
    recompute();
}

void VelocityWindow::recompute()
{
    float    sum = 0.0f;
    uint32_t cov = 0;

    for (uint8_t i = 0; i < VEL_BINS; i++) {
        sum += bin_v[i];
        cov += bin_cov[i];
    }

    w_cache   = sum;
    total_cov = (cov > 0xFFFF) ? 0xFFFF : (uint16_t)cov;
}

void VelocityWindow::add(float accel_mss, uint32_t now_ms)
{
    uint32_t dt;

    /*
     * Guard 2 -- prime the baseline before anything enters the window.
     *
     * An UNBIASED RUNNING MEAN over the first VEL_PRIME_SAMPLES, not the first
     * sample. Seeding from one sample was tried on hardware 2026-08-07 and
     * failed exactly the way §3.1 warns: the boot sample happened to sit ~0.05
     * high, and a 0.05 m/s^2 baseline error integrated over a 5 s window is
     * 0.26 m/s -- past the departure gate, permanently, on a bench.
     *
     * Precision needed: bias in w is eps * VEL_WINDOW_MS, so keeping w's bias
     * under 0.02 m/s needs eps < 0.004 m/s^2. Per-sample sigma measured on the
     * bench is ~0.013, so 32 samples gives eps ~ 0.0023 and a bias of ~0.011.
     * That is where VEL_PRIME_SAMPLES comes from -- do not lower it without
     * redoing this arithmetic.
     *
     * Nothing is integrated while priming, and the window is cleared when
     * priming completes, so no sample taken against an unconverged baseline can
     * survive into a reported value.
     */
    if (prime_count < VEL_PRIME_SAMPLES) {
        prime_count++;
        base += (accel_mss - base) / (float)prime_count;
        last_sample_ms = now_ms;
        have_last      = true;

        if (prime_count >= VEL_PRIME_SAMPLES) {
            reset(now_ms);
        }
        return;
    }

    if (!have_last) {
        last_sample_ms = now_ms;
        have_last      = true;
        return;
    }

    dt = now_ms - last_sample_ms;
    last_sample_ms = now_ms;

    if (dt == 0) {
        return;
    }

    /*
     * Guard 3 -- clamp dt so a blanked stretch cannot be credited as though the
     * current sample had held throughout it.
     *
     * The buzzer runs 200 ms on with a 50 ms ringdown, so during an alarm the
     * real gap between accepted samples reaches ~570 ms while this allows 400.
     * The arrival integral therefore reads LOW through a beep cycle. That is
     * chosen: an under-counted arrival delays release, and §14.4 says the safe
     * direction is always "alarm longer".
     */
    if (dt > VEL_MAX_DT_MS) {
        dt = VEL_MAX_DT_MS;
    }

    advance_to(now_ms);

    bin_v[cur]   += (accel_mss - base) * ((float)dt / 1000.0f);
    bin_cov[cur] += (uint16_t)dt;

    recompute();

    /*
     * Guard 1 -- baseline tracking, frozen whenever it could eat the signal.
     *
     * track_baseline is cleared by the caller outside STATE_MONITORING and
     * while the buzzer sounds. The |w| test is the third leg: a departure slow
     * enough to stay under the departure gate is exactly the case where an
     * unfrozen tracker would absorb it and guarantee the miss.
     */
    if (track_baseline && fabs(w_cache) < VEL_BASELINE_FREEZE_MPS) {
        float alpha = (float)dt / (float)VEL_BASELINE_TAU_MS;

        if (alpha > 0.05f) {
            alpha = 0.05f;
        }

        base += alpha * (accel_mss - base);
        w_high_since = 0;
        return;
    }

    /*
     * ESCAPE FROM THE FREEZE DEADLOCK.
     *
     * The freeze above exists so a slow real departure cannot be absorbed by
     * its own detector. But it takes |w| as evidence that something is moving,
     * and |w| is computed FROM the baseline -- so a baseline that is wrong
     * holds itself frozen and can never recover. That is not hypothetical: it
     * is what the first hardware run did on 2026-08-07, sitting on a bench at
     * w = -0.26 indefinitely, and it is the §3.1 failure ("latched a stale
     * baseline it could never clear") reappearing inside the guard written to
     * prevent it.
     *
     * The way out is to notice that a REAL transient is brief. w is a trailing
     * window, so a genuine departure decays back through the freeze level
     * within a window or two, and in any case the FSM leaves STATE_MONITORING
     * within MOVEMENT_DETECTION_TIMEOUT_MS and track_baseline goes false.
     * Sustained |w| above the freeze level WHILE STILL MONITORING therefore
     * means the baseline is wrong, not that the car is moving: holding
     * 0.03 m/s for 15 s would be a real 0.03 m/s velocity step sustained
     * without ever tripping any other detector.
     *
     * So: re-prime. Cheaper and more honest than trying to nudge the baseline,
     * because the running mean converges in VEL_PRIME_SAMPLES and reports
     * nothing while it does.
     */
    if (track_baseline) {
        if (w_high_since == 0) {
            w_high_since = now_ms;
        } else if ((now_ms - w_high_since) > VEL_REPRIME_MS) {
            prime_count  = 0;
            w_high_since = 0;
            reset(now_ms);
        }
    } else {
        w_high_since = 0;
    }
}

float VelocityWindow::w()
{
    return w_cache;
}

bool VelocityWindow::valid()
{
    return (prime_count >= VEL_PRIME_SAMPLES) &&
           (total_cov >= VEL_MIN_COVERAGE_MS);
}
