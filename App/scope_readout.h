#ifndef SCOPE_READOUT_H
#define SCOPE_READOUT_H

/* drains the in-RAM scope over CoE */

/**
 * @brief Service queued scope buffer readout requests.
 *
 * Called every iteration of the app/servo_main.c main loop. Enqueues work
 * rather than performing transfers itself; drains foc/scope_log.h's buffer
 * over CoE via scope_log_read().
 */
void scope_readout_poll(void);

#endif /* SCOPE_READOUT_H */
