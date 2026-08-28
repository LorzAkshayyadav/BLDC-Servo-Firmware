#ifndef EXCITATION_H
#define EXCITATION_H

#include <stdint.h>
#include <stdbool.h>

/* Stage-4 sysID excitation injection (IMPLEMENTATION_SPEC.md section 6/10).
 *
 * Injects the coherent multi-sine test signal from config/sysid_params.h
 * onto i_q_ref so foc/correlate.c and the offline Tools/sysid_fit.py can fit
 * the two-inertia plant model. MUST NOT be active outside an explicit sysID
 * mode -- this is test instrumentation, not a production control path, and
 * config/build_config.h's BRINGUP_STAGE guard is what is expected to keep it
 * out of a STAGE_PRODUCTION build. */

typedef struct {
    uint32_t sample_index;   /* advances once per call, wraps at the sweep
                              * period; drives the coherent frequency table */
    bool     active;
} excitation_state_t;

/** @brief Arm the excitation generator at the start of a sysID capture. */
void excitation_start(excitation_state_t *st);

/** @brief Disarm; excitation_step() returns 0 until excitation_start() again. */
void excitation_stop(excitation_state_t *st);

/**
 * @brief Next excitation sample.
 *
 * Called once per FOC ISR period from the sysID mode's ref-generation path
 * (not from the normal current-loop path), added to i_q_ref before the PI
 * step so the closed loop's response to it is what gets identified.
 *
 * @param st Excitation state, advanced by this call.
 * @return   Next sample, mA, to add to i_q_ref. 0 when not active.
 */
int32_t excitation_step(excitation_state_t *st);

#endif /* EXCITATION_H */
