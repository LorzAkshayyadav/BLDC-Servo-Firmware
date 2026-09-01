/* =============================================================================
 * motion.c  --  L3, 4 kHz, pended from the FOC ISR.
 *
 * WHY THIS IS PENDED AND NOT CALLED
 *   The ISR raises a software interrupt and returns.  The NVIC runs this as
 *   soon as the CPU drops below priority 1, so a long motion computation
 *   cannot delay the next control period.  A pending interrupt stays pending
 *   -- nothing is lost, only deferred by at most the execution time of
 *   everything above it, which is ~13 us against a 250 us deadline.
 *
 * THE ORDER, AND WHY POSITION AND VELOCITY USE DIFFERENT ENCODERS
 *   Velocity closes on the MOTOR encoder; position closes on the LOAD
 *   encoder (arch section 8).  The inner loop then sees a rigid plant and can
 *   be made fast, while the outer loop sees what the application actually
 *   cares about -- including gear error, backlash and compliance, which is
 *   precisely the point.
 *
 *   Closing position directly on the load does introduce the two-inertia
 *   resonance through the transmission.  Section 8 says to address that only
 *   if measurement shows it limits you, not pre-emptively.  So there is no
 *   notch filter here, deliberately, until stage 6 says otherwise.
 * ============================================================================= */

#include "motion.h"
#include "position_loop.h"
#include "velocity_loop.h"
#include "torque_loop.h"
#include "interpolator.h"
#include "feedforward.h"
#include "traj_limit.h"
#include "hal.h"
#include "contracts.h"
#include "hal_sections.h"

static DTCM_BSS struct {
    int32_t velocity_rpm;
    bool    first;
} s;

void motion_init(void)
{
    s.velocity_rpm = 0;
    s.first        = true;

    position_loop_reset();
    velocity_loop_reset();
    torque_loop_reset();
    interpolator_reset();
}

void motion_reset_integrators(void)
{
    /* Called from hal_safe_state(), the same declared upward crossing as
     * foc_reset_integrators().  Every integrator in L3, or re-enable applies
     * whatever the loops had wound up to before the fault. */
    position_loop_reset();
    velocity_loop_reset();
    torque_loop_reset();
}

/* -----------------------------------------------------------------------------
 * The velocity divide.
 *
 * The ISR accumulates position deltas and MEASURED intervals for four periods
 * and publishes the pair; the single divide happens here, at 4 kHz, not in
 * the ISR.
 *
 * Four periods rather than one because differentiation amplifies quantisation
 * noise in inverse proportion to the interval.  The four consecutive
 * differences telescope to the total displacement, so nothing is discarded --
 * you get the noise performance of the slow rate AND the fidelity of the fast
 * one.  At 19-bit that is 0.46 rpm per LSB instead of 1.83.
 *
 * d_ticks is measured, not assumed: the section 9.1 clock discipline trims
 * the carrier, so the nominal 62.5 us is not constant and differs slightly
 * between joints in the same machine.
 * -------------------------------------------------------------------------- */
static int32_t velocity_from_accum(const velocity_accum_t *a)
{
    if (a->samples == 0u || a->d_ticks == 0u) {
        return 0;
    }
    return (int32_t)(((int64_t)a->d_position * 60 * (int64_t)HAL_TICK_HZ) /
                     ((int64_t)a->d_ticks << 32));
}

void motion_task(void)
{
    const params_t *p = &g_params.slot[CONTRACT_TAKE(g_params)];

    /* One pointer copy of each shared datum, taken once.  Same rule as the
     * ISR: re-reading mid-task reintroduces the tear the double buffer exists
     * to avoid. */
    const feedback_t       *fb  = &g_feedback.slot[CONTRACT_TAKE(g_feedback)];
    const velocity_accum_t *acc = &g_velocity.slot[CONTRACT_TAKE(g_velocity)];

    s.velocity_rpm = velocity_from_accum(acc);

    /* Setpoint interpolation FIRST.  Setpoints arrive at the network cycle
     * rate while this loop runs faster; without interpolation the loop sees a
     * staircase, which becomes a velocity step and then a torque step at the
     * network rate -- audible, and mechanically excitatory in a geared joint
     * (arch section 9.3). */
    const int32_t pos_cmd = interpolator_step();

    /* Position on the LOAD encoder. */
    int32_t vel_setpoint = position_loop_step(pos_cmd, (int32_t)fb->position_load, p);

    /* Feedforward added to the setpoint, not to the output, so the velocity
     * loop still sees the full error and its integrator does not have to
     * fight the feedforward term. */
    vel_setpoint += feedforward_velocity();

    vel_setpoint = traj_limit_velocity(vel_setpoint, p);

    /* Velocity on the MOTOR encoder. */
    int32_t i_q_ref = velocity_loop_step(vel_setpoint, s.velocity_rpm, p);

    i_q_ref += feedforward_acceleration();

    /* Torque loop kept as a compatibility hook only.  The RS-485 torque
     * sensor is for observation and diagnostics, not for modifying the q-axis
     * current demand. */
    i_q_ref = torque_loop_step(i_q_ref, p);

    i_q_ref = traj_limit_current(i_q_ref, p);

    /* PUBLISH as ONE aligned word.
     *
     * current_ref_t is asserted to be exactly one word and word-aligned in
     * contracts.c, so this single store is atomic against the ISR -- d and q
     * can never be read from different updates.  Writing the two fields
     * separately would be the classic torn publish. */
    current_ref_t r;
    r.v.i_d_ma = 0;                  /* no field weakening on this drive */
    r.v.i_q_ma = (int16_t)i_q_ref;
    g_current_ref.word = r.word;

    s.first = false;
}

int32_t motion_velocity_rpm(void)
{
    return s.velocity_rpm;
}