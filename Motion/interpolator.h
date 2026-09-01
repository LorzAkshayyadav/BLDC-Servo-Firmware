#ifndef INTERPOLATOR_H
#define INTERPOLATOR_H

#include <stdint.h>

/* network cycle -> 4 kHz, section 9.3 */

/**
 * @brief Reset the interpolator to the unprimed state.
 *
 * Called from motion_init() (motion/motion.h). Until the first setpoint
 * arrives, interpolator_step() returns 0 rather than interpolating toward
 * an arbitrary origin.
 */
void interpolator_reset(void);

/**
 * @brief Register a new network setpoint.
 *
 * Called from the SYNC0 context when a new setpoint arrives (arch section
 * 9.3), not from the 4 kHz motion task. Starts a fresh linear interpolation
 * from the previous target -- or from @p position itself, if this is the
 * very first setpoint received, so the drive does not command a full
 * traverse to the first commanded position at whatever rate the
 * interpolator allows.
 *
 * @param position New target position, load-encoder scale.
 */
void interpolator_new_setpoint(int32_t position);

/**
 * @brief One interpolation step, at the 4 kHz motion-task rate.
 *
 * Linearly interpolates between the previous and current network setpoint
 * over STEPS_PER_CYCLE motion-task periods, so the position loop sees a
 * piecewise-constant velocity instead of a staircase -- removing the
 * impulse train a naive pass-through would feed into the transmission
 * resonance. Deliberately one network cycle behind (see the file banner in
 * motion/interpolator.c for why extrapolating instead would be the worse
 * trade).
 *
 * @return Interpolated position command for this motion-task period, or 0
 *         if no setpoint has arrived yet.
 */
int32_t interpolator_step(void);

#endif /* INTERPOLATOR_H */
