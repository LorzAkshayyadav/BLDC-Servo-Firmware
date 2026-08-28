#ifndef FIELDBUS_APP_H
#define FIELDBUS_APP_H

/* EtherCAT slave stack glue */

/**
 * @brief Service the EtherCAT slave stack: network state machine, mailbox, CoE.
 *
 * Called every iteration of the app/servo_main.c main loop. Must audit every
 * critical section on this path (arch section 6.1): an interrupt disable or
 * raised priority mask anywhere in here delays the control ISR by its full
 * duration regardless of priority, and delays the fast protection path with
 * it. Leaving the operational state through this stack must command a
 * controlled stop, not a coast.
 */
void fieldbus_app_poll(void);

#endif /* FIELDBUS_APP_H */
