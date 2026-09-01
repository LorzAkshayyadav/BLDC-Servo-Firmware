/* =============================================================================
 * interpolator.c  --  network setpoint rate to loop rate.
 *
 * THE PROBLEM IT SOLVES  (arch section 9.3)
 *   Setpoints arrive at the network cycle rate -- typically 1 kHz -- while
 *   the position loop runs at 4 kHz.  Feed them straight through and the loop
 *   sees a STAIRCASE: position jumps once per network cycle and holds.
 *
 *   Differentiate a staircase and you get an impulse train.  The velocity
 *   loop sees a velocity step, the current loop sees a torque step, and the
 *   joint is excited at the network rate on every cycle.  In a geared joint
 *   that is both audible and mechanically excitatory -- it feeds energy
 *   straight into the transmission resonance.
 *
 *   Linear interpolation between consecutive setpoints is the minimum fix and
 *   removes the impulses entirely: velocity becomes piecewise constant rather
 *   than impulsive.
 *
 * WHY IT IS DELIBERATELY ONE CYCLE BEHIND
 *   Interpolating between the previous and current setpoint means the output
 *   lags by one network cycle.  The alternative -- extrapolating forward from
 *   the last two -- has no lag but amplifies any jitter in setpoint delivery
 *   and overshoots on every direction reversal.  For a drive whose carrier is
 *   already disciplined to the distributed clock, one cycle of predictable
 *   lag is much the better trade.
 * ============================================================================= */

#include "interpolator.h"
#include "contracts.h"
#include "hal_sections.h"
#include "board_limits.h"

/* Loop periods per network cycle.  With a 1 ms network cycle and a 4 kHz
 * motion task this is 4.  Derived rather than hard-coded so that changing the
 * network cycle does not silently leave the interpolator interpolating over
 * the wrong span -- which would show up as a small periodic velocity error,
 * easy to mistake for a tuning problem. */
#define STEPS_PER_CYCLE   ((CARRIER_HZ / HAL_MOTION_DIVIDER) / 1000u)

static DTCM_BSS struct {
    int32_t  from;
    int32_t  to;
    uint32_t step;
    bool     primed;
} s;

void interpolator_reset(void)
{
    s.from   = 0;
    s.to     = 0;
    s.step   = 0u;
    s.primed = false;
}

/* Called from the SYNC0 context when a new setpoint arrives. */
void interpolator_new_setpoint(int32_t position)
{
    if (!s.primed) {
        /* First setpoint: start from it rather than from zero, or the drive
         * would command a full traverse to the first commanded position at
         * whatever rate the interpolator allows. */
        s.from   = position;
        s.primed = true;
    } else {
        s.from = s.to;
    }
    s.to   = position;
    s.step = 0u;
}

int32_t interpolator_step(void)
{
    if (!s.primed) {
        return 0;
    }

    if (s.step < STEPS_PER_CYCLE) {
        s.step++;
    }

    /* Unsigned difference so the interpolation is correct across the position
     * wrap, then scaled and added back.  Interpolating the raw values would
     * traverse the long way round whenever the setpoint crosses zero. */
    const int32_t delta = (int32_t)((uint32_t)s.to - (uint32_t)s.from);

    return (int32_t)((uint32_t)s.from +
                     (uint32_t)(int32_t)(((int64_t)delta * (int32_t)s.step) /
                                         (int32_t)STEPS_PER_CYCLE));
}