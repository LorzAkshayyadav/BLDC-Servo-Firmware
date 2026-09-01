#ifndef ANGLE_H
#define ANGLE_H

#include <stdint.h>
#include "common/fixed.h"

/* phase accumulator, trig table, advance term */

/**
 * @brief Compute this period's compensated electrical angle.
 *
 * A single integer addition (arch section 8.1): adds the delay-compensation
 * advance term, computed from the measured (not nominal) interval and the
 * tuned angle_advance_per_rpm coefficient, to the raw electrical angle
 * already provided by hal_enc_read(). Using the measured TIM5 interval
 * rather than a nominal 62.5 us keeps the arch section 9.1 clock trim from
 * leaking into velocity as a per-joint scale error.
 *
 * @param raw_angle       Electrical angle for this period, from
 *                         hal_enc_sample_t::angle (HAL_ENC_MOTOR).
 * @param prev_position   Motor mechanical position at the previous period,
 *                         for the first-difference speed term.
 * @param interval_ticks  Measured interval since the previous period,
 *                         HAL_TICK_HZ ticks.
 * @param advance_per_rpm Tuning coefficient, config/control_params.h /
 *                         params_t::angle_advance_per_rpm.
 * @return                Compensated electrical angle to feed
 *                         angle_sincos() for both Park and inverse Park.
 */
angle_t angle_advance(uint32_t raw_angle, uint32_t prev_position,
                      uint32_t interval_ticks, uint32_t advance_per_rpm);

/**
 * @brief Look up sin/cos of an electrical angle.
 *
 * Thin wrapper over fixed_sincos() (common/fixed.h) kept in foc/ so callers
 * only ever need to include foc/angle.h for the angle pipeline, not reach
 * into common/ directly.
 *
 * @param theta Electrical angle, phase-accumulator scale.
 * @param out   Filled with the sin/cos pair. Must not be NULL.
 */
void angle_sincos(angle_t theta, sincos_t *out);

/**
 * @brief Per-period first difference of mechanical position, for the tier-2
 *        over-speed check (arch section 5.2).
 *
 * Noise is tolerable for a threshold comparison -- this is deliberately not
 * the smoothed 4-period velocity estimate the motion task uses (arch
 * section 8.1, contracts.h velocity_accum_t).
 *
 * @param position       Current mechanical position.
 * @param prev_position  Previous period's mechanical position.
 * @param interval_ticks Measured interval since the previous period.
 * @return               Instantaneous speed estimate, rpm.
 */
int32_t angle_speed_estimate(uint32_t position, uint32_t prev_position,
                             uint32_t interval_ticks);
/**
 * @brief Free-running electrical angle at a commanded speed, no encoder.
 *
 * Stage 3 ("open loop, forced angle") and stage 4 ("current loop closed on
 * the forced angle") both need an angle that does not depend on a calibrated
 * commutation offset -- which is only found at stage 5.
 */
angle_t angle_forced_step(int32_t speed_rpm, uint32_t interval_ticks);

/** @brief Zero the forced-angle accumulator. Call before entering a forced mode. */
void angle_forced_reset(void);
#endif /* ANGLE_H */
