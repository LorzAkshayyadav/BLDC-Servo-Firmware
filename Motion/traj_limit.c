/* =============================================================================
 * traj_limit.c  --  the last thing between a computed setpoint and the drive.
 *
 * WHY LIMITING IS NOT THE SAME AS CLAMPING
 *   Clamping a value is one line.  What matters here is that a limit which
 *   simply truncates produces a DISCONTINUITY, and a discontinuity in
 *   velocity is an infinite acceleration demand -- so a naive velocity clamp
 *   creates exactly the torque step it was meant to prevent.
 *
 *   So velocity is limited by magnitude AND by rate of change.  The second is
 *   the acceleration limit, and it is what makes the first safe.
 * ============================================================================= */

#include "traj_limit.h"
#include "hal_sections.h"
#include "control_params.h"

static DTCM_BSS int32_t s_prev_velocity;

void traj_limit_reset(void)
{
    s_prev_velocity = 0;
}

int32_t traj_limit_velocity(int32_t velocity_rpm, const params_t *p)
{
    /* 1. Magnitude.
     *
     * The binding limit is usually NOT the motor's rating: above 7 kHz of
     * master-track frequency the iC-MU200's tracking converter stops
     * following the input and the position output silently stops being valid
     * while still looking like a number.  p->speed_limit_rpm is derived from
     * that (arch section 5.2, over-speed). */
    int32_t v = velocity_rpm;
    if (v >  (int32_t)p->speed_limit_rpm) { v =  (int32_t)p->speed_limit_rpm; }
    if (v < -(int32_t)p->speed_limit_rpm) { v = -(int32_t)p->speed_limit_rpm; }

    /* 2. Rate of change, which is what stops step 1 from being harmful. */
    const int32_t max_step = TRAJ_MAX_ACCEL_RPM_PER_STEP;
    const int32_t delta    = v - s_prev_velocity;

    if (delta >  max_step) { v = s_prev_velocity + max_step; }
    if (delta < -max_step) { v = s_prev_velocity - max_step; }

    s_prev_velocity = v;
    return v;
}

int32_t traj_limit_current(int32_t i_q_ma, const params_t *p)
{
    /* Current is limited by magnitude only -- deliberately no rate limit.
     *
     * The current loop's whole purpose is to change current fast, and a slew
     * limit here would fight it and show up as a sluggish torque response
     * that looks like a mistuned current loop.  The physical rate limit is
     * the bus voltage against the winding inductance, which the current loop
     * already respects because it saturates on available voltage. */
    int32_t i = i_q_ma;
    if (i >  p->i_limit_ma) { i =  p->i_limit_ma; }
    if (i < -p->i_limit_ma) { i = -p->i_limit_ma; }
    return i;
}