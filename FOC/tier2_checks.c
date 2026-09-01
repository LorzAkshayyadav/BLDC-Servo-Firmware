/* =============================================================================
 * tier2_checks.c  --  the 62.5 us protection tier.
 *
 * WHERE THIS SITS
 *   T0   gate driver, under a microsecond, autonomous
 *   T1'  analog watchdogs, ~3.5 us, hardware comparison, no software
 *   T2   THIS FILE, one PWM period
 *   T3   supervisory, milliseconds to seconds
 *
 *   So these checks are NOT the fast overcurrent trip -- the watchdogs are,
 *   and they act without reaching any code.  What lands here is everything
 *   the watchdogs structurally cannot see: a current that is within limits
 *   but inconsistent, a bus that has drifted, an accumulated thermal load, a
 *   speed the encoder can no longer track.
 *
 * EVERY CHECK RETURNS A SPECIFIC CAUSE
 *   There is no generic fault code.  A field failure gives you one power
 *   cycle's worth of evidence, and "overcurrent" tells you far less than
 *   "current sum non-zero" -- the second says a sensor or the ground path,
 *   not the motor.
 * ============================================================================= */

#include "tier2_checks.h"
#include "hal_sections.h"
#include "control_params.h"

/* Consecutive periods a current must exceed the limit before it trips.
 *
 * The analog watchdogs already cover the fast case, so this check exists for
 * a SUSTAINED overcurrent below the watchdog threshold -- a stalled motor
 * pulling rated current forever, say.  Requiring persistence stops a single
 * noisy sample from tripping a drive that is operating correctly. */
#define OVERCURRENT_PERSIST   4u

static DTCM_BSS uint32_t s_overcurrent_count;

ITCM_FUNC hal_trip_cause_t tier2_fast(int32_t i_a_ma, int32_t i_b_ma,
                                      int32_t i_c_ma, int32_t v_bus_mv,
                                      const params_t *p)
{
    /* ---- 1. Current sum ------------------------------------------------
     *
     * In a three-wire motor the three phase currents must sum to zero.  When
     * they do not, the fault is in the MEASUREMENT or the wiring, not in the
     * motor: a failed sensor, a saturated amplifier, or a ground fault
     * providing a fourth path for current to leave by.
     *
     * This is the entire reason phase A is measured at all.  It sits on the
     * independent converter and cannot be sampled simultaneously with B and
     * C, so it is useless for control -- but a sum check does not care about
     * skew, so full sensor redundancy comes essentially free (section 4.4).
     *
     * Checked FIRST because every value below depends on the currents being
     * trustworthy. */
    int32_t sum = i_a_ma + i_b_ma + i_c_ma;
    if (sum < 0) { sum = -sum; }
    if (sum > p->current_sum_tol_ma) {
        return HAL_TRIP_CURRENT_SUM;
    }

    /* ---- 2. Sustained overcurrent -------------------------------------- */
    int32_t ab = (i_a_ma < 0) ? -i_a_ma : i_a_ma;
    int32_t bb = (i_b_ma < 0) ? -i_b_ma : i_b_ma;
    int32_t cb = (i_c_ma < 0) ? -i_c_ma : i_c_ma;
    int32_t peak = (ab > bb) ? ab : bb;  if (cb > peak) { peak = cb; }

    if (peak > p->i_limit_ma) {
        if (++s_overcurrent_count >= OVERCURRENT_PERSIST) {
            return HAL_TRIP_OVERCURRENT_FILTERED;
        }
    } else if (s_overcurrent_count != 0u) {
        s_overcurrent_count--;      /* leaky, so isolated peaks decay */
    }

    /* ---- 3. Bus voltage -------------------------------------------------
     *
     * Overvoltage usually means regeneration into a supply that cannot absorb
     * it -- a decelerating load with no brake chopper.  Undervoltage means
     * the supply is sagging, and continuing to modulate into a collapsing bus
     * produces current the model does not predict. */
    if (v_bus_mv > p->v_bus_max_mv) { return HAL_TRIP_BUS_OVERVOLT;  }
    if (v_bus_mv < p->v_bus_min_mv) { return HAL_TRIP_BUS_UNDERVOLT; }

    return HAL_TRIP_NONE;
}

/* -----------------------------------------------------------------------------
 * Checks that need more than one period of history.  Called from the motion
 * or supervisory task, not the ISR.
 * -------------------------------------------------------------------------- */
hal_trip_cause_t tier2_slow(int32_t speed_rpm, uint32_t thermal_accum,
                            const params_t *p)
{
    const uint32_t mag = (speed_rpm < 0) ? (uint32_t)(-speed_rpm)
                                         : (uint32_t)speed_rpm;

    /* The limit that binds here is usually NOT the motor's rating.  The
     * iC-MU200 master track is specified to 7 kHz, above which FRQ_CNV sets
     * and the tracking converter has stopped following the input -- the
     * position output silently stops being valid while still looking like a
     * number.  p->speed_limit_rpm is derived from that, below the mechanical
     * rating. */
    if (mag > p->speed_limit_rpm) {
        return HAL_TRIP_OVERSPEED;
    }

    if (thermal_accum > THERMAL_ACCUM_LIMIT) {
        return HAL_TRIP_THERMAL;
    }

    return HAL_TRIP_NONE;
}

void tier2_reset(void)
{
    s_overcurrent_count = 0u;
}