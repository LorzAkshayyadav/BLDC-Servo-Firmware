/* =============================================================================
 * hal_time.c  --  TIM5, the free-running timebase.
 *
 * ONE TIMER, ONE OWNER.  This file owns TIM5 and nothing else.  The two other
 * time-ish verbs in hal.h live with their timers instead:
 *   hal_carrier_position()  -> hal_pwm.c       (TIM1)
 *   hal_deadline_kick()     -> hal_sequencer.c (TIM6)
 *
 * THE RULE THIS FILE ENFORCES BY DOING NOTHING
 *   Software never writes TIM5->CNT.  That is not a style preference: the
 *   wrap-safe subtraction in common/fixed.h is only correct if the counter is
 *   strictly monotonic modulo 2^32.  A single write anywhere breaks every
 *   interval measurement that spans it, silently, and the symptom is a
 *   missed-period fault that did not happen.
 *
 *   There is deliberately no hal_time_reset(), no hal_time_set(), and no
 *   overflow interrupt.  At 240 MHz the counter wraps every 17.9 s and there
 *   is nothing to handle.
 * ============================================================================= */

#include "hal.h"
#include "stm32h7xx.h"

/* -----------------------------------------------------------------------------
 * Read the timebase.
 *
 * One load.  No HAL call, because HAL_TIM_ReadCapturedValue and friends carry
 * handle indirection and state checks that cost more than the measurement is
 * worth, and this is called at least twice per 62.5 us period from the ISR.
 * -------------------------------------------------------------------------- */
uint32_t hal_time_now(void)
{
    return TIM5->CNT;
}