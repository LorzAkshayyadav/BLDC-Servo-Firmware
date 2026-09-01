#ifndef VELOCITY_LOOP_H
#define VELOCITY_LOOP_H

#include <stdint.h>
#include "common/contracts.h"

/* on the MOTOR encoder, accumulated dt */

/**
 * @brief Reset the velocity loop's integrator and clip flag.
 *
 * Called from motion_init() and motion_reset_integrators()
 * (motion/motion.h) -- omitting this on a fault-recovery path restarts the
 * loop with a wound-up integrator demanding a step the drive cannot yet
 * safely take, the same rationale as the current loop
 * (foc/current_pi.h).
 */
void velocity_loop_reset(void);

/**
 * @brief One velocity-loop step, on the MOTOR encoder.
 *
 * Measured at the motor rather than the load so the plant is the rotor
 * inertia through a rigid shaft -- no transmission compliance, no
 * backlash, no two-inertia resonance -- which is what lets this loop run
 * fast; the position loop closing on the load is what accounts for gear
 * error instead. Uses the same conditional-integration anti-windup scheme
 * as the current loop (foc/current_pi.c): the integrator only accumulates
 * when the previous step did not saturate, or when the new increment moves
 * the output back toward the linear region.
 *
 * @param setpoint_rpm Velocity setpoint, rpm (position loop output plus
 *                     velocity feedforward, already trajectory-limited).
 * @param actual_rpm   Measured motor-shaft velocity, rpm
 *                     (motion_velocity_rpm(), motion/motion.h).
 * @param p            Current parameter set (kp_velocity_q15,
 *                     ki_velocity_q15, i_limit_ma used here -- the output
 *                     is a current demand, clamped to i_limit_ma).
 * @return             Q-axis current demand for the current loop, mA.
 */
int32_t velocity_loop_step(int32_t setpoint_rpm, int32_t actual_rpm,
                           const params_t *p);

#endif /* VELOCITY_LOOP_H */
