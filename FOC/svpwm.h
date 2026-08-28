#ifndef SVPWM_H
#define SVPWM_H

#include <stdint.h>
#include <stdbool.h>
#include "foc/transforms.h"

/* modulator, dead-time comp, duty clamp report */

typedef struct {
    uint16_t a, b, c;   /* per-phase duty, 0 .. hal_pwm_duty_clamp()        */
    uint16_t max;       /* max(a,b,c), for saturation reporting             */
    bool     clamped;   /* true if the clamp in hal_pwm_set_duty() will bite */
} duty_t;

/**
 * @brief Space-vector modulation: alpha/beta voltage to three phase duties.
 *
 * Over-modulation policy: TBD -- state it explicitly in the implementation
 * rather than letting it fall out of the arithmetic (arch section 6,
 * svpwm.c Implement list).
 *
 * @param v_alpha_beta Commanded stationary-frame voltage, mV.
 * @param v_bus_mv     Bus voltage, mV.
 * @return             Three phase duties plus saturation reporting. Callers
 *                      must not duplicate the clamp arithmetic themselves --
 *                      hal_pwm_set_duty() applies hal_pwm_duty_clamp() to
 *                      the maximum of the three, and `max`/`clamped` here
 *                      are for feedback reporting, not enforcement.
 */
duty_t svpwm(ab_t v_alpha_beta, int32_t v_bus_mv);

/**
 * @brief Adjust duty for dead-time-induced voltage error.
 *
 * Compensation direction depends on the sign of the phase current, which is
 * why it is a separate step from svpwm() rather than folded in: the two
 * phase currents used here are the same ones already available to the
 * caller, not a re-derivation.
 *
 * @param duty   Uncompensated duty from svpwm().
 * @param i_b_ma Phase B current, mA.
 * @param i_c_ma Phase C current, mA.
 * @return       Dead-time-compensated duty.
 */
duty_t svpwm_deadtime_compensate(duty_t duty, int32_t i_b_ma, int32_t i_c_ma);

#endif /* SVPWM_H */
