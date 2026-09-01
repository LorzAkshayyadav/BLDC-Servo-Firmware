#ifndef BRAKE_SEQ_H
#define BRAKE_SEQ_H

#include <stdint.h>
#include <stdbool.h>

/* engage-before-torque-removal sequencing */

/**
 * @brief Initialise the brake sequencer to the engaged state.
 *
 * Call once before the carrier starts. Commands the brake engaged
 * (hal_brake_engage(), hal/hal.h) unconditionally, since power-on state is
 * unknown and this topology is power-to-RELEASE -- engaged, with no coil
 * current, is the safe default.
 */
void brake_seq_init(void);

/**
 * @brief Request the brake begin releasing.
 *
 * Only takes effect from the engaged state; a request while already
 * releasing or released is a no-op. Torque must already be applied before
 * calling this (arch section 11): releasing the brake before torque exists
 * lets a vertical joint fall until the loop catches it.
 *
 * The drive state machine must not command motion until
 * brake_seq_is_released() reports true -- releasing is not instantaneous,
 * because the armature needs a full-duty pull before it is safe to drop to
 * the reduced hold duty.
 */
void brake_seq_request_release(void);

/**
 * @brief Request the brake begin engaging.
 *
 * Only takes effect from the released or releasing-pull state. The drive
 * state machine must not remove torque until brake_seq_is_engaged() reports
 * true (arch section 11) -- the reverse order drops the load. Engaging is
 * the slow direction on this power-to-release topology: cutting the duty
 * only starts the coil current decaying, and the spring cannot move the
 * armature until the field has collapsed (hal_brake_engage_delay_us(),
 * hal/hal.h, is the measured wait this is timed against, not a guess).
 */
void brake_seq_request_engage(void);

/**
 * @brief Advance the brake sequencer state machine.
 *
 * Called from the 1 kHz supervisory task (app/supervisor.c) with the time
 * elapsed since the previous call, so the pull and engage timers are built
 * from measured elapsed time rather than an assumed fixed period.
 *
 * @param elapsed_us Time elapsed since the previous call, microseconds.
 */
void brake_seq_poll(uint32_t elapsed_us);

/**
 * @brief Whether the brake has finished engaging.
 *
 * The drive state machine must not remove torque until this is true (arch
 * section 11) -- removing it before the brake holds drops the load exactly
 * as releasing early does at the other end of the move.
 *
 * @return true once the brake is fully engaged and holding.
 */
bool brake_seq_is_engaged(void);

/**
 * @brief Whether the brake has finished releasing.
 *
 * The drive state machine must not command motion until this is true (arch
 * section 11).
 *
 * @return true once the brake is fully released and holding clear.
 */
bool brake_seq_is_released(void);

#endif /* BRAKE_SEQ_H */
