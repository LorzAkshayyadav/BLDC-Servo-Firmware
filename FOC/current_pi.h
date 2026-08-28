#ifndef CURRENT_PI_H
#define CURRENT_PI_H

#include <stdint.h>
#include <stdbool.h>

/* current PI control, with anti-windup */

/** @brief One axis's integrator state. d and q each get their own instance. */
typedef struct {
    int32_t integrator_mv;   /* accumulated integral term, millivolts       */
    bool    clipped;         /* true if the last step saturated the output */
} pi_state_t;

/**
 * @brief One PI step: integrate, saturate, anti-windup.
 *
 * Anti-windup stops integrating when the output clips rather than unwinding
 * after the fact, so a saturated axis does not overshoot once it un-clips.
 *
 * @param st        Integrator state for this axis (d or q). Persists across
 *                   calls; never shared between axes.
 * @param error_ma  Reference minus measured current, mA.
 * @param kp_q15    Proportional gain, Q15, from config/control_params.h /
 *                   params_t (arch section 13: design-generic, not tuned
 *                   here).
 * @param ki_q15    Integral gain, Q15.
 * @param v_bus_mv  Bus voltage, mV -- the output saturation limit.
 * @return          Commanded voltage for this axis, mV, clamped to
 *                   +/- v_bus_mv.
 */
int32_t pi_step(pi_state_t *st, int32_t error_ma, int32_t kp_q15,
                int32_t ki_q15, int32_t v_bus_mv);

/**
 * @brief Clear one axis's integrator.
 *
 * Called by both PI axes' owner during the hal_safe_state() reset hook
 * (registered via hal_register_safe_state_hooks() in hal.h): omitting this
 * makes re-enable trip instantly, because the current loop would otherwise
 * restart with a wound-up integrator demanding full voltage into a
 * stationary rotor (IMPLEMENTATION_SPEC.md, hal_safety.c traps).
 *
 * @param st Integrator state to clear.
 */
void current_pi_reset(pi_state_t *st);

#endif /* CURRENT_PI_H */
