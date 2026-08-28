#ifndef PARAM_SERVICE_H
#define PARAM_SERVICE_H

/* parameter read/write, publishes g_params */

/**
 * @brief Service queued parameter read/write requests.
 *
 * Called every iteration of the app/servo_main.c main loop. Applies
 * plausibility checking before publishing, and publishes through the
 * params_t double buffer (common/contracts.h) as a whole set -- never field
 * by field, or the current loop can run one period with a new Kp and an old
 * Ki (arch section 12).
 */
void param_service_poll(void);

#endif /* PARAM_SERVICE_H */
