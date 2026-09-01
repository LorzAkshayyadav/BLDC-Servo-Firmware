/* =============================================================================
 * velocity_loop.c  --  inner loop, on the MOTOR encoder.
 *
 * WHY THE MOTOR ENCODER AND NOT THE LOAD
 *   Measured at the motor, the plant is the rotor inertia through a rigid
 *   shaft -- no transmission compliance, no backlash, no two-inertia
 *   resonance.  So this loop can be made fast.  Measured at the load it would
 *   see the resonance directly and the achievable bandwidth would drop to
 *   well below the antiresonance (arch section 8).
 *
 *   The cost is that this loop does not know about gear error.  That is the
 *   position loop's job, and it closes on the load precisely so that
 *   something in the cascade does.
 * ============================================================================= */

#include "velocity_loop.h"
#include "hal_sections.h"

static DTCM_BSS struct {
    int32_t integrator;
    bool    clipped;
} s;

void velocity_loop_reset(void)
{
    s.integrator = 0;
    s.clipped    = false;
}

int32_t velocity_loop_step(int32_t setpoint_rpm, int32_t actual_rpm,
                           const params_t *p)
{
    const int32_t err = setpoint_rpm - actual_rpm;

    const int32_t p_term = (int32_t)(((int64_t)err * p->kp_velocity_q15) >> 15);
    const int32_t i_inc  = (int32_t)(((int64_t)err * p->ki_velocity_q15) >> 15);

    /* Conditional integration, same scheme as the current loop: the
     * integrator accumulates only when the previous step did not saturate,
     * OR when the new increment moves the output back toward the linear
     * region.  Without that second clause a loop that saturates once can
     * never recover, because the integrator freezes at exactly the value
     * keeping it saturated. */
    if (!s.clipped ||
        ((s.integrator > 0) && (i_inc < 0)) ||
        ((s.integrator < 0) && (i_inc > 0))) {
        s.integrator += i_inc;
    }

    if (s.integrator >  p->i_limit_ma) { s.integrator =  p->i_limit_ma; }
    if (s.integrator < -p->i_limit_ma) { s.integrator = -p->i_limit_ma; }

    int32_t out = p_term + s.integrator;

    s.clipped = false;
    if (out >  p->i_limit_ma) { out =  p->i_limit_ma; s.clipped = true; }
    if (out < -p->i_limit_ma) { out = -p->i_limit_ma; s.clipped = true; }

    return out;
}