#ifndef FEEDFORWARD_H
#define FEEDFORWARD_H

#include <stdint.h>
#include <stdbool.h>

/* feedforward control */

/**
 * @brief Publish the master's velocity/acceleration feedforward terms.
 *
 * Called from the fieldbus layer when a new position command arrives
 * (common/contracts.h's position_cmd_t carries velocity_ff/accel_ff/
 * ff_valid from the same source). Each term is used only if the master
 * actually supplied it -- differentiating the position setpoint to
 * synthesise a missing one would reintroduce the impulse train the
 * interpolator (motion/interpolator.h) exists to remove.
 *
 * @param velocity_rpm  Commanded velocity feedforward, rpm.
 * @param accel_ma      Commanded acceleration feedforward, expressed as a
 *                      current (i = J*alpha/Kt), mA.
 * @param have_velocity true if the master supplied @p velocity_rpm.
 * @param have_accel    true if the master supplied @p accel_ma.
 */
void feedforward_set(int32_t velocity_rpm, int32_t accel_ma,
                     bool have_velocity, bool have_accel);

/**
 * @brief Velocity feedforward term for this period.
 *
 * Added to the velocity loop's SETPOINT (motion/velocity_loop.h), not its
 * output, so the velocity loop still sees the full error and its
 * integrator does not have to fight the feedforward term.
 *
 * @return Velocity feedforward, rpm, or 0 if the master has not supplied
 *         one.
 */
int32_t feedforward_velocity(void);

/**
 * @brief Acceleration feedforward term for this period.
 *
 * @return Acceleration feedforward expressed as a current, mA, or 0 if the
 *         master has not supplied one.
 */
int32_t feedforward_acceleration(void);

#endif /* FEEDFORWARD_H */
