#ifndef FW_UPDATE_H
#define FW_UPDATE_H

/* firmware update handling */

/**
 * @brief Service a staged firmware update, if one is in progress.
 *
 * Called every iteration of the app/servo_main.c main loop. Staged in AXI
 * SRAM (hal_sections.h: AXI_DATA); enqueues work rather than blocking the
 * main loop on flash operations.
 */
void fw_update_poll(void);

#endif /* FW_UPDATE_H */
