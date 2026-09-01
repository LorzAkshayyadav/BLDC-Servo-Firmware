/* =============================================================================
 * angle.c  --  the electrical angle and the speed estimate.
 *
 * WHY THE ANGLE IS A 32-BIT ACCUMULATOR AND NOT DEGREES OR RADIANS
 *   The full uint32_t range is exactly one electrical revolution.  Wraparound
 *   is therefore natural integer overflow -- free, with no comparison, no
 *   branch, and no rounding drift accumulating over hours of running.  Angle
 *   advance becomes an integer addition and the trig table index is a shift
 *   (arch section 8.1).
 *
 *   Every "% 360" and every "if (theta > TWO_PI) theta -= TWO_PI" that would
 *   otherwise appear in this file is absent for that reason.
 *
 * L1 HAS ALREADY DONE THE POLE-PAIR MULTIPLY
 *   hal_enc_sample_t.angle is documented as electrical, phase-accumulator
 *   scale.  So the mechanical-to-electrical conversion and the commutation
 *   offset live in hal_encoder.c, and this file only adds the advance term.
 * ============================================================================= */

#include "angle.h"
#include "hal.h"
#include "hal_sections.h"
#include "motor_params.h"

/* -----------------------------------------------------------------------------
 * Speed, in rpm, from two mechanical positions and the MEASURED interval.
 *
 * The interval is measured rather than assumed because the section 9.1 clock
 * discipline trims the carrier to track the distributed clock: the nominal
 * 62.5 us is not constant, and it differs slightly between joints in the same
 * machine.  Using a constant here would put that drift straight into velocity
 * as a per-axis scale error.
 *
 * position is a 32-bit accumulator over one MECHANICAL revolution, so the
 * difference is wrap-safe by the same argument as the electrical angle.
 * -------------------------------------------------------------------------- */
ITCM_FUNC int32_t angle_speed_estimate(uint32_t position, uint32_t prev_position,
                                       uint32_t interval_ticks)
{
    if (interval_ticks == 0u) {
        return 0;               /* first period; caller discards it anyway */
    }

    /* Deliberate unsigned subtract then signed reinterpret: gives the signed
     * shortest-path difference across the wrap with no branch. */
    const int32_t d = (int32_t)(position - prev_position);

    /* rev/s   = d / 2^32 / (interval_ticks / HAL_TICK_HZ)
     * rpm     = d * 60 * HAL_TICK_HZ / (2^32 * interval_ticks)
     *
     * 64-bit intermediate: d can be +/-2^31 and HAL_TICK_HZ is 2.4e8, so the
     * product overflows 32 bits by a wide margin. */
    return (int32_t)(((int64_t)d * 60 * (int64_t)HAL_TICK_HZ) /
                     ((int64_t)interval_ticks << 32));
}

/* -----------------------------------------------------------------------------
 * The angle the control chain actually uses.
 *
 * Two delays are being compensated, and they add:
 *
 *   ACQUISITION  the angle was latched at 10.94 us (TIM1 CCR4 = 2625) but the
 *                current is sampled at the carrier peak, 31.25 us -- so the
 *                angle is already 20.3 us old when it meets its currents.
 *   ACTUATION    the duty computed now is preloaded and takes effect at the
 *                next reload, one full period later.
 *
 * Both are proportional to speed, which is why a single per-rpm coefficient
 * covers them.  Tune it by sweeping at constant speed and load and taking the
 * value that minimises current -- it is not calculable to useful accuracy,
 * because it also absorbs filter group delay in the encoder and the
 * modulator's own half-period offset (arch section 8.1).
 * -------------------------------------------------------------------------- */
ITCM_FUNC angle_t angle_advance(uint32_t raw_angle, uint32_t prev_position,
                                uint32_t interval_ticks, uint32_t advance_per_rpm)
{
    const int32_t rpm = angle_speed_estimate(raw_angle, prev_position,
                                             interval_ticks);

    /* One addition, and the wrap is the addition overflowing.  Signed rpm
     * means the advance automatically reverses with direction. */
    return (angle_t)(raw_angle + (uint32_t)(rpm * (int32_t)advance_per_rpm));
}

/* -----------------------------------------------------------------------------
 * Forced angle, for bring-up stages 3 and 4.
 *
 * Generates a constant-rate electrical angle with no encoder involved, which
 * is what makes "open loop, forced angle" (stage 3) and "current loop closed
 * on the forced angle" (stage 4) possible BEFORE the commutation offset has
 * been calibrated at stage 5.
 *
 * The accumulator is deliberately file-static rather than passed in: there is
 * exactly one motor, and threading state through the call would suggest
 * otherwise.
 * -------------------------------------------------------------------------- */
static DTCM_BSS angle_t s_forced;

ITCM_FUNC angle_t angle_forced_step(int32_t speed_rpm, uint32_t interval_ticks)
{
    /* increment = speed_rpm/60 * pole_pairs * 2^32 * (interval / HAL_TICK_HZ) */
    const int64_t inc = ((int64_t)speed_rpm * MOTOR_POLE_PAIRS *
                         (int64_t)interval_ticks << 32) /
                        (60LL * (int64_t)HAL_TICK_HZ);

    s_forced += (angle_t)inc;
    return s_forced;
}

void angle_forced_reset(void)
{
    s_forced = 0u;
}

/* -----------------------------------------------------------------------------
 * Sine and cosine, together, from one lookup.
 *
 * Returning BOTH from a single call is not a convenience: arch section 8.1
 * wants the same pair used for the forward and the reverse transform, and
 * handing back one struct makes that coherence structural rather than
 * something a future reader has to remember.  Two separate sin() and cos()
 * calls could silently be given different angles.
 * -------------------------------------------------------------------------- */
ITCM_FUNC void angle_sincos(angle_t theta, sincos_t *out)
{
    *out = fixed_sincos(theta);     /* table in data TCM, zero wait state */
}