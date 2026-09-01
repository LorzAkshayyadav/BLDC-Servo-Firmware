#ifndef SUPERVISOR_H
#define SUPERVISOR_H

#include <stdint.h>

/* 1 kHz task: cross-checks, IWDG refresh */

/**
 * @brief Initialise supervisor-task state.
 *
 * Call once before the carrier starts, alongside foc_init() (foc/foc.h) and
 * motion_init() (motion/motion.h). Zeroes the I^2t thermal accumulator and
 * the last-run timestamp so the first period's checks do not run against
 * leftover state from a previous enable.
 */
void supervisor_init(void);

/**
 * @brief The 1 kHz supervisory task: cross-checks, thermal accumulation,
 *        over-speed, brake/profile sequencing, and the independent
 *        watchdog refresh.
 *
 * Pended from the FOC ISR every 16th period (HAL_SUPERVISOR_DIVIDER,
 * hal/hal.h). Arch section 7, L4: refreshing the watchdog here rather than
 * from the main loop turns the refresh into an end-to-end liveness proof of
 * the whole chain (TIM1 -> ADC -> FOC ISR -> pend -> this task), not just
 * evidence that the main loop is still spinning.
 *
 * Runs every check that would cost ISR budget to detect, milliseconds
 * later, something that only develops on a mechanical timescale:
 * transmission-integrity and torque-path cross-checks (arch section 8.3),
 * I^2t thermal accumulation, and the over-speed/thermal tier-2 checks
 * (tier2_slow(), foc/tier2_checks.h) -- the ISR itself never computes speed
 * or accumulates heat. A fault from any of them converges on
 * hal_safe_state() the same as every other tier.
 *
 * The watchdog refresh is deliberately LAST: refreshing before the checks
 * would let a supervisor that faults every cycle still keep the watchdog
 * happy, which is exactly the condition the watchdog exists to catch.
 */
void supervisor_task(void);

/**
 * @brief Latest motor/load transmission-position discrepancy.
 *
 * Published as process data because its slow drift over months is a
 * genuine predictive-maintenance signal for gear wear and magnet ageing
 * (arch section 8.3), not just an input to the pass/fail threshold check.
 *
 * @return Absolute discrepancy between the motor position (projected
 *         through the gear ratio) and the load position, load-encoder
 *         counts.
 */
uint32_t supervisor_transmission_discrepancy(void);

/**
 * @brief Latest predicted-vs-measured torque discrepancy.
 *
 * Detects a drifting current sensor, an incorrect motor constant, or
 * gearbox degradation (arch section 8.3) -- each shows up only as
 * disagreement between the current-derived estimate and the RS-485 torque
 * sensor reading, not on either signal alone.
 *
 * @return Absolute discrepancy between predicted and measured torque,
 *         milli-newton-metres.
 */
uint32_t supervisor_torque_discrepancy(void);

/**
 * @brief Current I^2t thermal accumulator value.
 *
 * Rises with the square of q-axis current above the rated limit and decays
 * linearly below it (see accumulate_thermal(), app/supervisor.c), so a
 * brief overload is absorbed and a sustained one is not -- inherently a
 * low-pass, so it needs no filter of its own.
 *
 * @return Running thermal accumulator, compared against
 *         THERMAL_ACCUM_LIMIT (config/control_params.h) by tier2_slow().
 */
uint32_t supervisor_thermal_accum(void);

#endif /* SUPERVISOR_H */
