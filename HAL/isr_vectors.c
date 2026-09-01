/* =============================================================================
 * isr_vectors.c  --  interrupt entry points.
 *
 * THE ONE FILE THAT LEGITIMATELY SPANS THE BOUNDARY
 *   Vector bodies are register work -- testing ADC flags, clearing JEOS -- so
 *   they cannot live in L2.  But they must call into L2 and L3.  That upward
 *   call is declared here and in hal_safety.c, and nowhere else.
 *
 * TWO KINDS OF HANDLER HERE, AND THE RULE THAT SEPARATES THEM
 *   Our policy is that ST HAL driver functions are avoided ON THE PER-PERIOD
 *   PATH ONLY, not on principle.  Applied here:
 *
 *     ADC_IRQn, ADC3_IRQn   -- WE own the vector.  ADC_IRQn is the 16 kHz
 *                              control entry and the hardest deadline in the
 *                              system; routing it through HAL_ADC_IRQHandler
 *                              would walk every flag and evaluate callbacks
 *                              before reaching our code, hundreds of cycles of
 *                              VARIABLE overhead (arch section 6.1).  Variable
 *                              is the problem: section 15 wants an ISR
 *                              duration histogram with near-zero spread, and
 *                              HAL dispatch shows up as spread that looks like
 *                              a memory placement fault.
 *                              ADC3_IRQn is the phase A watchdog -- a trip
 *                              path where latency is the whole point.
 *
 *     TIM1_BRK, TIM3, TIM6  -- CUBEMX owns the vector; we implement the HAL
 *                              callback.  A break, a SYNC0 capture and a
 *                              deadline trip have deadlines of 100 us and up,
 *                              so ~150 cycles of dispatch is irrelevant, and
 *                              letting CubeMX keep these means three fewer
 *                              settings to remember and three fewer chances
 *                              for a regeneration to break the build.
 *
 * >>> REQUIRED CUBEMX SETTING -- TWO ROWS ONLY <<<
 *   NVIC view, Code generation tab, UNCHECK "Generate IRQ handler" for:
 *
 *       ADC_IRQn        ADC3_IRQn
 *
 *   Leave "Enabled" ticked on both -- that is a separate checkbox and it is
 *   what produces the HAL_NVIC_EnableIRQ call we still need.  Leave every
 *   other vector exactly as it is.
 *
 *   If the setting is ever lost, the failure is a duplicate-symbol link
 *   error, which is the right way for it to fail.
 * ============================================================================= */

#include "hal.h"
#include "hal_private.h"
#include "hal_sections.h"
#include "foc.h"                 /* declared upward call */
#include "motion.h"              /* declared upward call */
#include "adc.h"
#include "tim.h"
#include "supervisor.h"
/* =============================================================================
 * Software-pended tasks.
 *
 * The control ISR never CALLS motion or the supervisor.  It pends them and
 * returns, and the NVIC runs them once the CPU drops below priority 1.  A
 * pending interrupt stays pending -- the controller latches the bit and
 * services it when it can -- so nothing is starved, only deferred by at most
 * the execution time of everything above it (arch section 6.2).
 *
 * TIM7 and LPTIM1 are borrowed purely as vector NUMBERS.  Neither peripheral
 * is instantiated in this project, so neither can raise its interrupt by
 * itself.  Borrowing a peripheral we DO use -- TIM4, say -- would work today
 * and break silently the first time someone enabled an interrupt on it.
 * ========================================================================== */
#define MOTION_IRQn       TIM7_IRQn
#define SUPERVISOR_IRQn   LPTIM1_IRQn

ITCM_FUNC void hal_pend_motion(void)     { NVIC_SetPendingIRQ(MOTION_IRQn); }
ITCM_FUNC void hal_pend_supervisor(void) { NVIC_SetPendingIRQ(SUPERVISOR_IRQn); }

void hal_init_pended_vectors(void)
{
    /* Priority follows rate: the faster deadline gets the higher priority, so
     * every task's interference stays far below its own deadline (Table 5). */
    HAL_NVIC_SetPriority(MOTION_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(MOTION_IRQn);
    HAL_NVIC_SetPriority(SUPERVISOR_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(SUPERVISOR_IRQn);
}

void TIM7_IRQHandler(void)   { motion_task(); }
void LPTIM1_IRQHandler(void) { supervisor_task(); }

/* =============================================================================
 * OURS -- priority 1, the control vector.
 * ========================================================================== */

ITCM_FUNC void ADC_IRQHandler(void)
{
    /* WATCHDOG FLAGS FIRST, BEFORE ANYTHING ELSE.
     *
     * ADC1 and ADC2 share this vector with the control ISR, which is accepted
     * risk R3.  The mitigation is exactly this ordering: a threshold crossing
     * is handled before the control chain, not after it.  Worst case a
     * crossing arriving mid-ISR waits for it to finish -- still an order of
     * magnitude better than one PWM period.
     *
     * Both status registers are read BEFORE either is acted on, so a fault
     * that trips two phases reports the one that actually crossed rather than
     * whichever happened to be tested first. */
    const uint32_t isr1 = ADC1->ISR;
    const uint32_t isr2 = ADC2->ISR;

    if (isr1 & ADC_ISR_AWD1) {
        ADC1->ISR = ADC_ISR_AWD1;
        hal_safe_state(HAL_TRIP_OVERCURRENT_B);
        return;
    }
    if (isr2 & ADC_ISR_AWD1) {
        ADC2->ISR = ADC_ISR_AWD1;
        hal_safe_state(HAL_TRIP_OVERCURRENT_C);
        return;
    }

    if (isr1 & ADC_ISR_JEOS) {
        /* Plain write, not |=.  A read-modify-write would also clear any flag
         * that set itself between the read and the write -- including a
         * watchdog crossing arriving right now. */
        ADC1->ISR = ADC_ISR_JEOS;
        foc_isr();
    }
}

/* OURS -- priority 0.  Phase A on its own vector, which is the mitigation
 * recorded against R3 for B and C having to share the control vector. */
ITCM_FUNC void ADC3_IRQHandler(void)
{
    if (ADC3->ISR & ADC_ISR_AWD1) {
        ADC3->ISR = ADC_ISR_AWD1;
        hal_safe_state(HAL_TRIP_OVERCURRENT_A);
    }
}

/* =============================================================================
 * CUBEMX'S VECTORS -- we implement the callbacks.
 *
 * These are reached via HAL_TIM_IRQHandler() from stm32h7xx_it.c, which
 * clears the flag before calling us.  So none of these clear anything.
 * ========================================================================== */

/* Break / STO trip, priority 0.
 *
 * The hardware has already removed torque -- MOE cleared the instant the
 * break input asserted, with no software involved.  This only records why,
 * which is why it is allowed to be this short and why the HAL dispatch
 * latency ahead of it does not matter. */
void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {
        hal_safe_state(HAL_TRIP_BREAK_STO);
    }
}

void HAL_TIMEx_Break2Callback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {
        hal_safe_state(HAL_TRIP_BREAK_STO);
    }
}

/* Deadline monitor, priority 0.
 *
 * Priority 0 is not a preference: to catch a control ISR stuck in a loop this
 * must PREEMPT that ISR, which sits at priority 1.  Any lower and it never
 * runs in exactly the case it exists for.  Section 15 asks you to prove it
 * twice -- once by disabling the control interrupt, once with the ISR in a
 * deliberate infinite loop.
 *
 * PeriodElapsed is shared across every timer, but TIM6 is the only one in
 * this project with UIE set, so the guard is a formality that will earn its
 * keep the day someone adds another. */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        hal_safe_state(HAL_TRIP_DEADLINE);
    }
}

/* SYNC0 capture, priority 3.
 *
 * The counter value was latched in HARDWARE at the signal edge, so the
 * capture is the phase error directly, free of interrupt latency --
 * measurably better than reading a counter inside a handler, which measures
 * phase plus however long the handler took to start (arch section 9.1).
 *
 * Which means the dispatch latency ahead of this callback costs nothing: it
 * delays when we LEARN the value, not the value itself.  That is the specific
 * reason this vector can stay with HAL while ADC_IRQn cannot.
 *
 * No computation here.  The value goes to the clock-discipline loop, which
 * runs at 1 kHz with a time constant of seconds and low authority. */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        hal_fieldbus_sync_captured(
            HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2));
    }
}