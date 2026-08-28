#ifndef MOTION_H
#define MOTION_H

/* L3 task, 4 kHz, pended off the network ISR. No direct register access -- go through HAL/. */

/* task entry, runs the three loops in order */

/**
 * @brief Initialise motion-layer state. Call before the carrier starts,
 *        alongside foc_init() (foc/foc.h).
 *
 * Zeroes the position/velocity/torque loop integrators and any interpolator
 * state, so the first pend after the carrier starts does not run against
 * leftover state from a previous enable.
 */
void motion_init(void);

/**
 * @brief Zero the position, velocity and torque loop integrators.
 *
 * Called directly by hal_safety.c's hal_safe_state() at arch section 5.5
 * step 3, alongside foc_reset_integrators() (foc/foc.h) -- same rationale:
 * omitting this makes re-enable trip instantly, because an outer loop
 * restarts with a wound-up integrator demanding a step the drive cannot
 * yet safely take. See CODE_LAYOUT.md's layer-rules table for why
 * hal_safety.c is allowed to call upward into L3 for this.
 */
void motion_reset_integrators(void);

#endif /* MOTION_H */
