#ifndef TRAJ_LIMIT_H
#define TRAJ_LIMIT_H

#include <stdint.h>
#include "common/contracts.h"

/* trajectory limiting */

/**
 * @brief Reset the trajectory limiter's rate-of-change history.
 *
 * Called from motion_init() (motion/motion.h). Without this the first
 * traj_limit_velocity() call after enable would rate-limit against
 * whatever the previous-velocity state happened to hold, producing a
 * spurious acceleration clamp on the very first step.
 */
void traj_limit_reset(void);

/**
 * @brief Limit a velocity setpoint by magnitude and by rate of change.
 *
 * Magnitude alone is not enough: a naive clamp produces a discontinuity,
 * and a discontinuity in velocity is an infinite acceleration demand --
 * exactly the torque step the limiter exists to prevent. The rate limit
 * (TRAJ_MAX_ACCEL_RPM_PER_STEP, config/control_params.h) is what makes the
 * magnitude limit safe.
 *
 * @param velocity_rpm Requested velocity setpoint, rpm.
 * @param p            Current parameter set (speed_limit_rpm used here for
 *                     the magnitude limit).
 * @return             Velocity setpoint, magnitude- and rate-limited, rpm.
 */
int32_t traj_limit_velocity(int32_t velocity_rpm, const params_t *p);

/**
 * @brief Limit a q-axis current demand by magnitude.
 *
 * Deliberately no rate limit: the current loop's whole purpose is to
 * change current fast, and a slew limit here would fight it and present as
 * a mistuned current loop. The physical rate limit is the bus voltage
 * against the winding inductance, which the current loop already respects
 * because it saturates on available voltage.
 *
 * @param i_q_ma Requested q-axis current demand, mA.
 * @param p      Current parameter set (i_limit_ma used here).
 * @return       Current demand, clamped to +/- i_limit_ma.
 */
int32_t traj_limit_current(int32_t i_q_ma, const params_t *p);

#endif /* TRAJ_LIMIT_H */
