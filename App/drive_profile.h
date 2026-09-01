#ifndef DRIVE_PROFILE_H
#define DRIVE_PROFILE_H

/* CiA 402 state machine (supervisory, 1 kHz) */

/**
 * @brief Advance the CiA 402 drive-profile state machine by one supervisor
 *        period.
 *
 * Called from the 1 kHz supervisory task (app/supervisor.c), after the
 * cross-checks and thermal accumulation so a fault raised this period is
 * reflected before the state machine reacts to it.
 *
 * TODO: not yet implemented (app/drive_profile.c is currently an empty
 * scaffold) -- declared here so app/supervisor.c compiles against a real
 * prototype ahead of that work.
 */
void drive_profile_poll(void);

#endif /* DRIVE_PROFILE_H */
