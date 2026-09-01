/* =============================================================================
 * current_pi.c  --  the two current regulators.
 *
 * UNITS
 *   error in mA, output in mV, integrator in mV.  Keeping the integrator in
 *   output units rather than error units means the anti-windup limit is the
 *   same number as the output limit, so there is one clamp instead of two
 *   that have to be kept consistent.
 *
 * THE GAINS ARE NOT GUESSES
 *   For a current loop on an R-L load the analytic tuning is
 *       Kp = L * omega_bw        Ki = R * omega_bw
 *   which is why stage 4 sysID measures L and R first: it turns tuning from a
 *   search into a calculation.  The values arrive here as Q15 through
 *   control_params.h.
 * ============================================================================= */

#include "current_pi.h"
#include "hal_sections.h"

void current_pi_reset(pi_state_t *st)
{
    /* Called from foc_reset_integrators(), which hal_safe_state() calls as
     * step 3 of the arch section 5.5 sequence.  Skip it and re-enable trips
     * instantly, because the loop restarts with a wound-up integrator
     * demanding full voltage into a stationary rotor -- it presents as a
     * hardware fault and is not one. */
    st->integrator_mv = 0;
    st->clipped       = false;
}

ITCM_FUNC int32_t pi_step(pi_state_t *st, int32_t error_ma, int32_t kp_q15,
                          int32_t ki_q15, int32_t v_bus_mv)
{
    /* Available voltage per axis.
     *
     * With space-vector modulation the dq vector magnitude can reach
     * v_bus/sqrt(3) before the modulator saturates.  Limiting each axis to
     * v_bus/2 is slightly conservative and, more usefully, cheap -- and it
     * leaves headroom for the dead-time compensation that svpwm adds after
     * this. */
    const int32_t limit = v_bus_mv / 2;

    const int32_t p_term = (int32_t)(((int64_t)error_ma * kp_q15) >> 15);
    const int32_t i_inc  = (int32_t)(((int64_t)error_ma * ki_q15) >> 15);

    /* CONDITIONAL INTEGRATION.
     *
     * The integrator only accumulates when the previous step did NOT
     * saturate.  That is the whole anti-windup scheme, and it is chosen over
     * back-calculation because it needs no extra tuning constant and cannot
     * itself be mis-tuned.
     *
     * Note the asymmetry: integration is also allowed if the new increment
     * would move the output back towards the linear region.  Without that,
     * a loop that saturates once can never recover, because the integrator
     * would be frozen at exactly the value keeping it saturated. */
    if (!st->clipped ||
        ((st->integrator_mv > 0) && (i_inc < 0)) ||
        ((st->integrator_mv < 0) && (i_inc > 0))) {
        st->integrator_mv += i_inc;
    }

    /* Clamp the integrator on its own as well, so a long saturation cannot
     * leave it holding a value that takes many periods to unwind once the
     * demand returns to something achievable. */
    if (st->integrator_mv >  limit) { st->integrator_mv =  limit; }
    if (st->integrator_mv < -limit) { st->integrator_mv = -limit; }

    int32_t out = p_term + st->integrator_mv;

    st->clipped = false;
    if (out >  limit) { out =  limit; st->clipped = true; }
    if (out < -limit) { out = -limit; st->clipped = true; }

    return out;
}