#ifndef POSITION_LOOP_H
#define POSITION_LOOP_H

#include <stdint.h>
#include "common/contracts.h"

/* on the LOAD encoder */

/**
 * @brief Reset the position loop's state.
 *
 * The loop is proportional-only and carries no integrator (see
 * position_loop.c's file banner for why a second integrator in this cascade
 * would produce the classic geared-joint hunt), so there is nothing to
 * zero. Exists so motion_reset_integrators() (motion/motion.h) reads as
 * complete rather than leaving a reader wondering whether this loop was
 * forgotten.
 */
void position_loop_reset(void);

/**
 * @brief One position-loop step, on the LOAD encoder.
 *
 * Proportional only, deliberately: the velocity loop below already
 * integrates, and steady-state position error is already driven to zero by
 * that integrator whenever the command is constant. Integral action here
 * would instead produce the classic geared-joint hunt, where two
 * integrators wind against backlash and the joint oscillates through the
 * dead zone.
 *
 * @param cmd    Interpolated position command (interpolator_step(),
 *               motion/interpolator.h), load-encoder scale.
 * @param actual Measured load position, same accumulator scale as @p cmd --
 *               the error is the shortest-path difference across the
 *               32-bit wrap, so both must be on the same
 *               full-scale-per-revolution basis.
 * @param p      Current parameter set (kp_position_q15 used here).
 * @return       Velocity setpoint for the velocity loop, rpm.
 */
int32_t position_loop_step(int32_t cmd, int32_t actual, const params_t *p);

#endif /* POSITION_LOOP_H */
