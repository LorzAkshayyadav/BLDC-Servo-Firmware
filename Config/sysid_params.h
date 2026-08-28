/* =============================================================================
 * sysid_params.h  --  GENERATED FILE.  Do not hand-edit.
 *
 * Produced by Tools/sysid_plan.py: a coherent frequency list for the stage-4
 * two-inertia system identification sweep (IMPLEMENTATION_SPEC.md sections
 * 4, 10). "Coherent" means every frequency is an exact integer number of
 * cycles within the capture window, so foc/correlate.c's online correlation
 * and Tools/sysid_fit.py's offline fit see no spectral leakage between bins.
 *
 * Design-generic, like config/control_params.h: this is the TEST PLAN (which
 * frequencies, what amplitude, how long), not a per-unit measurement result.
 * The fitted plant (inertia, damping, resonance) that Tools/sysid_fit.py
 * produces from a captured run is unit-specific and belongs in
 * config/motor_params.h, not here.
 *
 * To regenerate:
 *   python Tools/sysid_plan.py --out config/sysid_params.h
 * ============================================================================= */
#ifndef CONFIG_SYSID_PARAMS_H
#define CONFIG_SYSID_PARAMS_H

#include <stdint.h>

/* PLACEHOLDER -- Tools/sysid_plan.py does not exist yet. */
#ifndef SYSID_FREQ_COUNT
#define SYSID_FREQ_COUNT           0u
#warning "config/sysid_params.h is ungenerated -- run Tools/sysid_plan.py before enabling BRINGUP_STAGE >= STAGE_CURRENT_LOOP sysID excitation"
#endif

/* Excitation amplitude, mA, added to i_q_ref by foc/excitation.c. Small
 * enough to stay well inside the current limit at every commanded operating
 * point tested, large enough to clear the current-sense noise floor --
 * characterise by measurement, not by guessing. */
#ifndef SYSID_EXCITATION_AMPLITUDE_MA
#define SYSID_EXCITATION_AMPLITUDE_MA 0
#endif

/* Capture window length, in FOC ISR periods (16 kHz), covering at least one
 * full cycle of the lowest frequency in the (currently empty) frequency
 * list, with margin for settling. */
#ifndef SYSID_CAPTURE_PERIODS
#define SYSID_CAPTURE_PERIODS      0u
#endif

/* Populated by the generator as:
 *   static const uint32_t SYSID_FREQUENCIES_MHZ_Q0[SYSID_FREQ_COUNT] = {...};
 * (frequencies in milli-Hz, fixed point, to stay off the no-float-on-the-
 * control-path path even though excitation runs at bring-up stage 4, not in
 * production) -- left undeclared while SYSID_FREQ_COUNT is 0 so a consumer
 * that forgets to check the count gets a compile error, not an empty sweep. */

#endif /* CONFIG_SYSID_PARAMS_H */
