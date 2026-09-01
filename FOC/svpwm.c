/* =============================================================================
 * svpwm.c  --  the modulator.
 *
 * MIN-MAX INJECTION, NOT SECTOR LOOKUP
 *   The textbook SVPWM computes a sector, then two vector dwell times, then
 *   maps them back to three duties.  This produces IDENTICAL duties by a
 *   different route: convert to three phase voltages, then subtract the mean
 *   of the largest and smallest from all three.
 *
 *   Identical, because that offset is precisely the common-mode term the
 *   sector method injects.  But it is branch-free, has no sector table, and
 *   cannot get the boundary between sectors wrong -- which is the classic
 *   SVPWM bug and the one that produces a torque ripple at six times
 *   electrical frequency that looks like a mechanical problem.
 *
 * WHY THE COMMON-MODE TERM IS FREE
 *   Only the DIFFERENCES between the three duties produce current in a
 *   floating star point.  Adding the same amount to all three changes the
 *   applied vector not at all, while lowering the peak -- which buys the
 *   1.155x extension over sinusoidal modulation.
 * ============================================================================= */

#include "svpwm.h"
#include "hal.h"
#include "hal_sections.h"
#include "board_limits.h"
#include "config/control_params.h"  /* DEADTIME_COMP_DEADBAND_MA, DEAD_TIME_DTG */

#define SQRT3_OVER_2_Q15   28378    /* 0.8660254 * 32768 */

ITCM_FUNC duty_t svpwm(ab_t v_alpha_beta, int32_t v_bus_mv)
{
    duty_t d;

    if (v_bus_mv < 1000) {
        /* No bus, no output.  Guards the division below and, more usefully,
         * means a bus collapse produces zero duty rather than a divide fault
         * inside the ISR. */
        d.a = d.b = d.c = 0u;
        d.max = 0u; d.clamped = false;
        return d;
    }

    /* Inverse Clarke: three phase voltages from alpha/beta. */
    const int32_t va = v_alpha_beta.alpha;
    const int32_t half_a = -v_alpha_beta.alpha / 2;
    const int32_t sq3_b  = (int32_t)(((int64_t)v_alpha_beta.beta *
                                      SQRT3_OVER_2_Q15) >> 15);
    const int32_t vb = half_a + sq3_b;
    const int32_t vc = half_a - sq3_b;

    /* Min-max injection. */
    int32_t vmax = (va > vb) ? va : vb;  if (vc > vmax) { vmax = vc; }
    int32_t vmin = (va < vb) ? va : vb;  if (vc < vmin) { vmin = vc; }
    const int32_t offset = -(vmax + vmin) / 2;

    /* Voltage to duty.  Centre is ARR/2: zero volts on a phase means 50%
     * duty, not 0%, because the bridge can only produce +/-v_bus/2 about the
     * midpoint. */
    const int32_t centre = (int32_t)CARRIER_ARR / 2;
    const int32_t scale  = (int32_t)CARRIER_ARR;

    int32_t da = centre + (int32_t)(((int64_t)(va + offset) * scale) / v_bus_mv);
    int32_t db = centre + (int32_t)(((int64_t)(vb + offset) * scale) / v_bus_mv);
    int32_t dc = centre + (int32_t)(((int64_t)(vc + offset) * scale) / v_bus_mv);

    if (da < 0) { da = 0; }  if (da > (int32_t)CARRIER_ARR) { da = CARRIER_ARR; }
    if (db < 0) { db = 0; }  if (db > (int32_t)CARRIER_ARR) { db = CARRIER_ARR; }
    if (dc < 0) { dc = 0; }  if (dc > (int32_t)CARRIER_ARR) { dc = CARRIER_ARR; }

    d.a = (uint16_t)da;
    d.b = (uint16_t)db;
    d.c = (uint16_t)dc;

    d.max = (d.a > d.b) ? d.a : d.b;  if (d.c > d.max) { d.max = d.c; }

    /* Reported, NOT applied.  hal_pwm_set_duty() owns the low-side sensing
     * clamp and applies it internally; duplicating the arithmetic here would
     * give two places to get CCR_MAX wrong.  This flag exists so the scope
     * buffer and the feedback contract can show when the clamp was biting,
     * which is otherwise invisible from L2. */
    d.clamped = (d.max > hal_pwm_duty_clamp());

    return d;
}

/* -----------------------------------------------------------------------------
 * Dead-time compensation.
 *
 * During the dead time both devices in a leg are off and the phase voltage is
 * decided by the direction of the CURRENT, not by the commanded duty: the
 * body diode that conducts is the one the current forces on.  So the applied
 * volt-seconds are short of the commanded value by one dead time, with a sign
 * set by the current direction.
 *
 * The correction is therefore a fixed number of counts, added or subtracted
 * per phase according to current sign.  It does not scale with duty or speed.
 *
 * THE DEADBAND IS THE IMPORTANT PART
 *   Near zero current the sign is noise, and a compensation that flips with
 *   it injects a square wave at whatever rate the noise happens to have.
 *   That is far worse than no compensation at all, and it is the usual reason
 *   dead-time compensation makes a drive rougher rather than smoother.  Below
 *   the threshold the correction is simply not applied.
 * -------------------------------------------------------------------------- */
ITCM_FUNC duty_t svpwm_deadtime_compensate(duty_t duty, int32_t i_b_ma,
                                           int32_t i_c_ma)
{
    /* Third current from the other two: for a three-wire motor it is
     * determined, which is why the control path never needs to measure it
     * (arch section 4.4). */
    const int32_t i_a_ma = -(i_b_ma + i_c_ma);

    const int32_t dz = DEADTIME_COMP_DEADBAND_MA;
    const int32_t k  = (int32_t)DEAD_TIME_DTG;

    int32_t a = duty.a, b = duty.b, c = duty.c;

    if (i_a_ma >  dz) { a += k; } else if (i_a_ma < -dz) { a -= k; }
    if (i_b_ma >  dz) { b += k; } else if (i_b_ma < -dz) { b -= k; }
    if (i_c_ma >  dz) { c += k; } else if (i_c_ma < -dz) { c -= k; }

    if (a < 0) { a = 0; }  if (a > (int32_t)CARRIER_ARR) { a = CARRIER_ARR; }
    if (b < 0) { b = 0; }  if (b > (int32_t)CARRIER_ARR) { b = CARRIER_ARR; }
    if (c < 0) { c = 0; }  if (c > (int32_t)CARRIER_ARR) { c = CARRIER_ARR; }

    duty.a = (uint16_t)a;
    duty.b = (uint16_t)b;
    duty.c = (uint16_t)c;
    duty.max = (duty.a > duty.b) ? duty.a : duty.b;
    if (duty.c > duty.max) { duty.max = duty.c; }
    duty.clamped = (duty.max > hal_pwm_duty_clamp());

    return duty;
}