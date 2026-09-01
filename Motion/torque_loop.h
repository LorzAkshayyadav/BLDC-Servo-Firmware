#ifndef TORQUE_LOOP_H
#define TORQUE_LOOP_H

#include <stdint.h>
#include "common/contracts.h"

/* on the RS-485 sensor */

/**
 * @brief Reset the torque loop's state.
 *
 * The torque sensor is observation-only; this compatibility hook keeps the
 * motion task interface stable without introducing any torque correction.
 */
void torque_loop_reset(void);

/**
 * @brief Compatibility no-op for the torque loop.
 *
 * The torque sensor is not used to modify the q-axis current command. It is
 * only observed for diagnostics and monitoring, so this function simply passes
 * the current demand through unchanged.
 *
 * @param i_q_ref_ma Q-axis current demand before/after this pass-through, mA.
 * @param p          Current parameter set.
 * @return           The unchanged q-axis current demand, mA.
 */
int32_t torque_loop_step(int32_t i_q_ref_ma, const params_t *p);

#endif /* TORQUE_LOOP_H */
