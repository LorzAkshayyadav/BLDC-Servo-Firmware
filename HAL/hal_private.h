/* =============================================================================
 * hal_private.h  --  L1-internal declarations.
 *
 * WHY THIS FILE EXISTS
 *   hal.h is the narrow verb set L2/L3/L4 are allowed to see, and it must
 *   contain no ST types so foc/ and motion/ can compile on a host (arch
 *   section 2).  But files inside hal/ legitimately need to call each other:
 *   hal_sequencer.c has to start the carrier last, and hal_safety.c has to
 *   generate the software break.  Those calls belong here, not in hal.h.
 *
 *   NOTHING OUTSIDE hal/ MAY INCLUDE THIS FILE.  Tools/layer_check.py should
 *   treat an include of hal_private.h from foc/, motion/ or app/ as a
 *   violation, the same as an include of stm32h7xx.h.
 * ============================================================================= */
#ifndef HAL_PRIVATE_H
#define HAL_PRIVATE_H

#include <stdint.h>
#include <stdbool.h>

/* -- TIM1, owned by hal_pwm.c --------------------------------------------- */

/**
 * @brief Enable the six channel outputs and the two trigger channels, with
 *        the master output disabled and all duties at zero.
 *
 * Does NOT start the counter.  Arch section 4: the carrier is the master and
 * every trigger consumer must already be armed when it starts, so starting it
 * is hal_sequencer.c's job and happens last.
 */
void hal_pwm_configure(void);

/**
 * @brief Start the carrier counter.  Called last in the init sequence.
 *
 * Outputs remain disabled; hal_pwm_outputs_enable() is a separate, later,
 * deliberate act via the drive state machine.
 */
void hal_pwm_carrier_start(void);

/**
 * @brief Generate a software break event.
 *
 * Sets BDTR's break flag exactly as the hardware break input would, so the
 * outputs latch to their idle state and recovery requires the same explicit
 * action (arch section 5.5 item 1).  Used by hal_safe_state().
 */
void hal_pwm_break_now(void);

/* -- TIM6, owned by hal_sequencer.c --------------------------------------- */

/** @brief Stop the deadline monitor.  Used when entering the safe state, so
 *         a trip does not immediately produce a second trip. */
void hal_deadline_stop(void);

#endif /* HAL_PRIVATE_H */