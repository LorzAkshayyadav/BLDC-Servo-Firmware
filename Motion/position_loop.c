/* =============================================================================
 * position_loop.c  --  outer loop, on the LOAD encoder.
 *
 * PROPORTIONAL ONLY, DELIBERATELY
 *   The velocity loop below this one already integrates.  Adding a second
 *   integrator here makes the cascade a double integrator, which needs
 *   careful lead compensation to stay stable and gains nothing for a servo
 *   joint: steady-state position error is already driven to zero by the
 *   velocity loop's integrator whenever the command is constant.
 *
 *   Integral action here is what produces the classic geared-joint hunt --
 *   the two integrators wind against backlash and the joint oscillates
 *   through the dead zone.
 * ============================================================================= */

#include "position_loop.h"
#include "hal_sections.h"

void position_loop_reset(void)
{
    /* Nothing to reset: no state.  The function exists so that
     * motion_reset_integrators() reads as complete rather than leaving a
     * reader wondering whether this loop was forgotten. */
}

int32_t position_loop_step(int32_t cmd, int32_t actual, const params_t *p)
{
    /* Unsigned subtract then signed reinterpret: the shortest-path error
     * across the 32-bit position wrap, with no branch.  A plain signed
     * subtraction of two large positions near the wrap would give an error of
     * nearly a full revolution in the wrong direction. */
    const int32_t err = (int32_t)((uint32_t)cmd - (uint32_t)actual);

    return (int32_t)(((int64_t)err * p->kp_position_q15) >> 15);
}