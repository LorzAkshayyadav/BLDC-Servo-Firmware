/* =============================================================================
 * foc.h  --  the L2 entry points.
 *
 * Every other file in foc/ has a header; foc_isr.c did not, yet two files
 * outside this directory have to reach into it.  This is that header, and it
 * is deliberately tiny: three functions is the entire upward-facing surface
 * of the torque control layer.
 *
 * WHO CALLS WHAT, AND WHY IT IS A DECLARED EXCEPTION
 *   hal/isr_vectors.c  -> foc_isr()               once per PWM period
 *   hal/hal_safety.c   -> foc_reset_integrators() from hal_safe_state()
 *
 *   Both are L1 calling upward into L2, which the layer rules otherwise
 *   forbid.  They are allowed because the alternative is worse in each case:
 *
 *     - The vector body is register work (clearing ADC1->ISR, testing the
 *       watchdog flags first per risk R3) and must not live in L2.
 *
 *     - Arch section 5.5 item 3 requires the integrators reset as part of the
 *       safe state.  Routing that through a registered callback keeps L1
 *       pointing downward but makes an unregistered callback silently do
 *       nothing on a fault path, and re-enable then trips instantly because
 *       the current loop restarts with a wound-up integrator demanding full
 *       voltage into a stationary rotor.  It presents as a hardware fault and
 *       is not one.  A direct call cannot be silently absent -- it is a link
 *       error.
 *
 *   Record both in CODE_LAYOUT.md.  A boundary crossing that is documented is
 *   a design decision; one that is not is a bug waiting to be copied.
 * ============================================================================= */
#ifndef FOC_H
#define FOC_H

#include <stdbool.h>

/**
 * @brief Initialise torque-control state.  Call before the carrier starts.
 *
 * Zeroes both PI integrators, the period counter, the velocity accumulators
 * and the previous-position history, and marks the first period so the
 * interval check does not fault on an interval measured against a
 * meaningless initial timestamp.
 */
void foc_init(void);

/**
 * @brief The 16 kHz control ISR body.
 *
 * Called from ADC_IRQHandler once per PWM period, after that handler has
 * tested the phase B and C watchdog flags and cleared JEOS.  Every input is
 * already resident when this runs -- arch section 4.1: the data existing is
 * what caused the handler to begin, so there is no readiness check to write.
 *
 * Must not block, must not poll in a loop, and must have a computable worst
 * case (P5).
 */
void foc_isr(void);

/**
 * @brief Zero both current-loop integrators.
 *
 * Part of the arch section 5.5 safe-state sequence, called from
 * hal_safe_state().  Safe to call from any context and at any priority; it
 * writes two words and touches nothing else.
 */
void foc_reset_integrators(void);

#endif /* FOC_H */