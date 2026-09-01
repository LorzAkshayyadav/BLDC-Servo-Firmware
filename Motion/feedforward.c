/* =============================================================================
 * feedforward.c  --  velocity and acceleration terms from the master.
 *
 * WHY FEEDFORWARD AND NOT MORE GAIN
 *   A cascade with feedback only must generate an ERROR before it produces
 *   an output -- following error is not a defect of tuning, it is how the
 *   loop works.  Raising gains to reduce it eventually costs stability,
 *   especially in a geared joint where the transmission resonance sets a
 *   hard ceiling.
 *
 *   Feedforward sidesteps the trade entirely: if the master supplies the
 *   commanded velocity and acceleration, most of the required output can be
 *   computed rather than regulated, and the feedback loops only have to
 *   correct what the model got wrong.
 *
 * ADDED TO THE SETPOINT, NOT THE OUTPUT
 *   Velocity feedforward is added to the velocity loop's SETPOINT, so the
 *   velocity loop still sees the full error and its integrator does not have
 *   to fight the feedforward term.  Adding it after the loop would leave the
 *   integrator winding to cancel a signal that is already correct.
 *
 * ONLY IF THE MASTER PROVIDES THEM
 *   Section 9.3: use commanded velocity and acceleration feedforward as well
 *   IF the master provides them.  Differentiating the position setpoint here
 *   to synthesise them would be self-defeating -- the setpoint is exactly the
 *   staircase the interpolator exists to smooth, and differentiating it gives
 *   back the impulse train.
 * ============================================================================= */

#include "feedforward.h"
#include "contracts.h"
#include "hal_sections.h"

static DTCM_BSS struct {
    int32_t velocity_rpm;
    int32_t accel_ma;
    bool    have_velocity;
    bool    have_accel;
} s;

void feedforward_set(int32_t velocity_rpm, int32_t accel_ma,
                     bool have_velocity, bool have_accel)
{
    s.velocity_rpm  = velocity_rpm;
    s.accel_ma      = accel_ma;
    s.have_velocity = have_velocity;
    s.have_accel    = have_accel;
}

int32_t feedforward_velocity(void)
{
    return s.have_velocity ? s.velocity_rpm : 0;
}

int32_t 



feedforward_acceleration(void)
{
    /* Acceleration feedforward is a CURRENT, because torque is proportional
     * to current and acceleration is proportional to torque: i = J*alpha/Kt.
     * The master supplies alpha; J and Kt come from motor_params.h, so the
     * conversion belongs wherever those are known -- which is the fieldbus
     * layer that receives it, not here. */
    return s.have_accel ? s.accel_ma : 0;
}