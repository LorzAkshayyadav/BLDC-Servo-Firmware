/* =============================================================================
 * brake_seq.c  --  commanded brake sequencing.  NOT the STO path.
 *
 * THE RULE, AND WHAT HAPPENS IF YOU GET IT BACKWARDS  (arch section 11)
 *   Torque is applied BEFORE the brake releases.
 *   The brake engages BEFORE torque is removed.
 *   The reverse order drops the load.
 *
 *   Both halves matter.  Releasing the brake before torque exists lets a
 *   vertical joint fall until the loop catches it.  Removing torque before
 *   the brake holds does the same thing at the other end of the move.
 *
 * THIS BRAKE IS POWER-TO-RELEASE, WHICH MAKES ENGAGING THE SLOW DIRECTION
 *   Cutting the duty does not engage the brake -- it starts the coil current
 *   decaying, and the spring cannot move the armature until the field has
 *   collapsed.  So the engage path WAITS, and the wait is a measured number
 *   from hal_brake_engage_delay_us(), not a guess.
 *
 *   Releasing is fast: energise and the armature pulls clear.  The asymmetry
 *   is why this is a state machine rather than two function calls.
 *
 * WHY THE FAULT PATH DOES NOT USE THIS FILE
 *   hal_safe_state() commands the brake and does not wait.  It cannot: torque
 *   has already gone in steps 1 and 2, and blocking for tens of milliseconds
 *   inside a priority-0 handler is not an option.  The load is unheld for the
 *   brake's engagement time, which is inherent to STO on any drive and
 *   belongs in the risk register rather than in firmware.
 * ============================================================================= */

#include "brake_seq.h"
#include "hal.h"
#include "hal_sections.h"

typedef enum {
    BRK_ENGAGED = 0,
    BRK_RELEASING_PULL,     /* full duty, dragging the armature clear      */
    BRK_RELEASED,           /* reduced duty, holding it clear              */
    BRK_ENGAGING_WAIT,      /* duty zero, waiting for the field to collapse */
} brk_state_t;

static DTCM_BSS struct {
    brk_state_t state;
    uint32_t    timer_us;
} s;

/* Pull-in at full duty then drop to hold: substantially less heat than a DC
 * hold, and the coil is energised continuously on this topology because
 * released IS the powered state (arch section 3.1). */
#define PULL_DURATION_US   200000u

void brake_seq_init(void)
{
    s.state    = BRK_ENGAGED;
    s.timer_us = 0u;
    hal_brake_engage();
}

void brake_seq_request_release(void)
{
    if (s.state == BRK_ENGAGED) {
        s.state    = BRK_RELEASING_PULL;
        s.timer_us = 0u;
        hal_brake_release_pull();
    }
}

void brake_seq_request_engage(void)
{
    if (s.state == BRK_RELEASED || s.state == BRK_RELEASING_PULL) {
        s.state    = BRK_ENGAGING_WAIT;
        s.timer_us = 0u;
        hal_brake_engage();
    }
}

/* Called from the 1 kHz supervisory task. */
void brake_seq_poll(uint32_t elapsed_us)
{
    s.timer_us += elapsed_us;

    switch (s.state) {
    case BRK_RELEASING_PULL:
        if (s.timer_us >= PULL_DURATION_US) {
            hal_brake_release_hold();
            s.state = BRK_RELEASED;
        }
        break;

    case BRK_ENGAGING_WAIT:
        if (s.timer_us >= hal_brake_engage_delay_us()) {
            s.state = BRK_ENGAGED;
        }
        break;

    default:
        break;
    }
}

/* The two questions the drive state machine asks.
 *
 * It must not release torque until brake_seq_is_engaged() is true, and must
 * not command motion until brake_seq_is_released() is true.  Returning false
 * during both transitions is what enforces the ordering -- the state machine
 * simply waits. */
bool brake_seq_is_engaged(void)  { return s.state == BRK_ENGAGED; }
bool brake_seq_is_released(void) { return s.state == BRK_RELEASED; }