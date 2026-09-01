#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>

/* L3 task, 4 kHz, pended off the network ISR. No direct register access -- go through HAL/. */

/* task entry, runs the three loops in order */

/**
 * @brief Initialise motion-layer state. Call before the carrier starts,
 *        alongside foc_init() (foc/foc.h).
 *
 * Zeroes the position/velocity/torque loop integrators and any interpolator
 * state, so the first pend after the carrier starts does not run against
 * leftover state from a previous enable.
 */
void motion_init(void);

/**
 * @brief Zero the position, velocity and torque loop integrators.
 *
 * Called directly by hal_safety.c's hal_safe_state() at arch section 5.5
 * step 3, alongside foc_reset_integrators() (foc/foc.h) -- same rationale:
 * omitting this makes re-enable trip instantly, because an outer loop
 * restarts with a wound-up integrator demanding a step the drive cannot
 * yet safely take. See CODE_LAYOUT.md's layer-rules table for why
 * hal_safety.c is allowed to call upward into L3 for this.
 */
void motion_reset_integrators(void);

/**
 * @brief The 4 kHz motion task: position, velocity and torque loops, in
 *        that order.
 *
 * Pended from the FOC ISR (hal_pend_motion(), hal/hal.h) every
 * HAL_MOTION_DIVIDER-th period. Runs, in order: the velocity divide
 * (velocity_from_accum(), motion/motion.c) on the ISR's completed
 * accumulation window; setpoint interpolation (interpolator_step()) so the
 * position loop never sees the raw network-rate staircase; the position
 * loop on the LOAD encoder; velocity feedforward added to the setpoint;
 * trajectory limiting; the velocity loop on the MOTOR encoder;
 * acceleration feedforward; the torque loop from the RS-485 sensor;
 * current limiting -- then publishes the result as one atomic word to
 * g_current_ref (common/contracts.h) for the FOC ISR to consume next
 * period.
 *
 * Position closes on the load and velocity on the motor deliberately (see
 * motion/motion.c's file banner): the inner loop then sees a rigid plant
 * and can run fast, while the outer loop sees the transmission's real
 * gear error and backlash.
 */
void motion_task(void);

/**
 * @brief Latest 4 kHz velocity-loop estimate, on the motor encoder.
 *
 * Computed once per motion_task() call from the ISR's 4-period position/tick
 * accumulation (velocity_accum_t, common/contracts.h) -- the single divide
 * happens here, at 4 kHz, not in the ISR, so it costs nothing on the 16 kHz
 * critical path (see velocity_from_accum(), motion/motion.c).
 *
 * @return Instantaneous motor-shaft velocity, rpm. Also what
 *         supervisor_task() (app/supervisor.h) feeds to tier2_slow()'s
 *         over-speed check (foc/tier2_checks.h), since the ISR itself never
 *         computes speed.
 */
int32_t motion_velocity_rpm(void);

#endif /* MOTION_H */
