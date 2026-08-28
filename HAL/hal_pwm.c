/* =============================================================================
 * hal_pwm.c  --  TIM1: the carrier, the six gate outputs, and both triggers.
 *
 * WHAT THIS FILE DOES *NOT* DO
 *   It does not configure TIM1.  MX_TIM1_Init() in platform/Core/Src/tim.c
 *   already produces the correct register set: ARR 7500 centre-aligned with
 *   RCR 1 and ARPE on, TRGO = OC4REF, TRGO2 = OC5REF, CH4 and CH5 in PWM
 *   mode 2 at 2625 and 7480, both break inputs enabled at polarity low, dead
 *   time 60, and OSSR/OSSI enabled with AOE disabled.
 *
 *   Duplicating any of that here would create a second source of truth, and
 *   the copy in this file would not be regenerated when the .ioc changes.
 *
 * HAL USAGE IN THIS FILE
 *   ST HAL macros are used wherever one exists, because they expand to the
 *   same register access with a name that says what it means.  Three places
 *   use raw register access, each for a stated reason:
 *
 *     EGR.BG    -- HAL has no "generate a break event" function at all.
 *     CCER      -- TIM_CCxChannelCmd is static, and HAL_TIM_PWM_Start bundles
 *                  CCER, MOE and CEN together.  We need them separate because
 *                  the carrier must start after every trigger consumer is
 *                  armed (arch section 4).
 *     MOE off   -- see the warning at hal_pwm_outputs_disable().
 *
 *   HAL *driver* functions are avoided on the per-period path only, and for a
 *   specific reason rather than on principle: they carry __HAL_LOCK, state
 *   checks that can refuse the call, and callback dispatch.  A function that
 *   can return HAL_BUSY has no meaningful failure handling inside a 62.5 us
 *   deadline.  Off that path they are used freely.
 * ============================================================================= */

#include "hal.h"
#include "hal_private.h"
#include "hal_sections.h"
#include "board_limits.h"
#include "tim.h"                 /* CubeMX: htim1 */

/* All six channel outputs plus the two pin-less trigger channels.
 *
 * CC4E/CC5E are enabled for certainty rather than necessity: OC4REF and
 * OC5REF are produced by the compare units regardless of CCER, which only
 * gates the signal to a pad.  Channel 5 has no pad on any STM32, and channel
 * 4's alternate pin is unassigned here, so enabling them costs nothing and
 * removes a question at bring-up. */
#define CCER_ALL  (TIM_CCER_CC1E | TIM_CCER_CC1NE | \
                   TIM_CCER_CC2E | TIM_CCER_CC2NE | \
                   TIM_CCER_CC3E | TIM_CCER_CC3NE | \
                   TIM_CCER_CC4E | TIM_CCER_CC5E)

/* Clock-discipline authority (arch section 9.1): the trim is a fraction of a
 * percent, because you are correcting crystal drift, not slewing the drive.
 * 64 ticks on a 7500 ARR is 0.85%, generous for a crystal and far too small
 * to be a control action.  A trim loop asking for more has diverged, and
 * clamping is the correct response. */
#define TRIM_AUTHORITY_TICKS   64

/* -----------------------------------------------------------------------------
 * Configuration: outputs armed, master output OFF, counter not yet running.
 * -------------------------------------------------------------------------- */
void hal_pwm_configure(void)
{
    /* Order matters.  MOE is cleared FIRST so enabling the channel outputs
     * cannot momentarily drive the gates.  With MOE clear and OSSI enabled
     * the pins are actively driven to their idle state -- both RESET -- which
     * is genuinely off.  Duty 0 with MOE set is not off: under PWM mode 1
     * OCxREF stays low all period, so all three low-side devices conduct. */
    hal_pwm_outputs_disable();

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0u);

    /* No public HAL call enables a channel output without also setting MOE
     * and starting the counter. */
    htim1.Instance->CCER |= CCER_ALL;

    /* Clear any break latched while the peripheral was being set up, or left
     * from a previous run.  STO is commonly asserted at power-on. */
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK);
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK2);
}

void hal_pwm_carrier_start(void)
{
    __HAL_TIM_ENABLE(&htim1);
}

/* -----------------------------------------------------------------------------
 * Master output enable.  Fails closed.
 *
 * Arch section 5.5: re-enable only by explicit command through the drive
 * state machine, never self-clearing.  This is that command, and it refuses
 * rather than asserting so the state machine can stay in "ready to switch on"
 * and publish the reason.
 * -------------------------------------------------------------------------- */
bool hal_pwm_outputs_enable(void)
{
    if (!hal_sto_channel_1_ok() || !hal_sto_channel_2_ok()) {
        return false;                    /* STO asserted; nothing to discuss */
    }

    /* A latched break flag means the cause has not been acknowledged.  With
     * AOE disabled MOE would not stay set anyway, producing a confusing
     * "it enabled and then stopped". */
    if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_BREAK) ||
        __HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_BREAK2)) {
        return false;
    }

    /* Start from zero every time.  Enabling into whatever was left in the
     * compare registers is how a re-enable produces a torque step. */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0u);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0u);

    __HAL_TIM_MOE_ENABLE(&htim1);
    return true;
}

/* -----------------------------------------------------------------------------
 * WARNING -- do not "simplify" this to __HAL_TIM_MOE_DISABLE().
 *
 * ST's __HAL_TIM_MOE_DISABLE only clears MOE if no channel outputs are
 * enabled:
 *
 *     if ((CCER & TIM_CCER_CCxE_MASK) == 0)
 *       if ((CCER & TIM_CCER_CCxNE_MASK) == 0)
 *          BDTR &= ~TIM_BDTR_MOE;
 *
 * hal_pwm_configure() enables all six, so both tests fail and the macro is a
 * no-op.  It would compile, look correct in review, and leave the bridge
 * live.  The _UNCONDITIONALLY variant is the one that means what its name
 * says.
 * -------------------------------------------------------------------------- */
void hal_pwm_outputs_disable(void)
{
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
}

/* -----------------------------------------------------------------------------
 * Software break.
 *
 * EGR.BG sets the break flag exactly as the hardware input would: MOE clears,
 * the outputs go to their idle state, and the flag latches.  Because AOE is
 * disabled they stay off until something explicitly re-enables them -- the
 * same recovery a hardware break demands (arch section 5.5 item 1).
 *
 * Clearing MOE directly would turn the outputs off but leave no latched
 * evidence and nothing for hal_pwm_outputs_enable() to refuse on.
 * -------------------------------------------------------------------------- */
ITCM_FUNC void hal_pwm_break_now(void)
{
    htim1.Instance->EGR = TIM_EGR_BG;    /* no HAL equivalent exists */
}

/* -----------------------------------------------------------------------------
 * Duty write.  Hot path, every 62.5 us, from the FOC ISR.
 * -------------------------------------------------------------------------- */
uint16_t hal_pwm_duty_clamp(void)
{
    return (uint16_t)CCR_MAX;
}

ITCM_FUNC void hal_pwm_set_duty(uint16_t a, uint16_t b, uint16_t c)
{
    uint16_t hi = (a > b) ? a : b;
    if (c > hi) { hi = c; }

    if (hi > (uint16_t)CCR_MAX) {
        uint16_t excess = (uint16_t)(hi - (uint16_t)CCR_MAX);
        uint16_t lo     = (a < b) ? a : b;
        if (c < lo) { lo = c; }

        if (lo >= excess) {
            /* COMMON-MODE SHIFT -- free and exact.
             *
             * Only the differences between the three duties produce current
             * in a floating star point, so subtracting the same amount from
             * all three lowers the peak without altering the applied vector
             * at all.  SVPWM has already injected a common-mode term; this
             * moves it. */
            a = (uint16_t)(a - excess);
            b = (uint16_t)(b - excess);
            c = (uint16_t)(c - excess);
        } else {
            /* The shift would drive a phase below zero, so the demand exceeds
             * what the clamped window can express.  Scale instead: direction
             * preserved, magnitude lost.
             *
             * Reaching this branch means svpwm.c produced a vector the
             * low-side sensing window cannot support.  Backstop, not the
             * intended over-modulation path -- if the scope buffer shows it
             * in normal operation, the fix belongs upstream. */
            uint32_t scale = ((uint32_t)CCR_MAX << 15) / hi;
            a = (uint16_t)(((uint32_t)a * scale) >> 15);
            b = (uint16_t)(((uint32_t)b * scale) >> 15);
            c = (uint16_t)(((uint32_t)c * scale) >> 15);
        }
    }

    /* Preloaded, so these take effect at the next reload.  The duty computed
     * this period applies to the next one, which is the delay the angle
     * advance term in foc/angle.c compensates. */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, a);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, b);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, c);
}

/* -----------------------------------------------------------------------------
 * Carrier position, for the late-entry check only (arch section 7, L1).
 *
 * Cannot detect a skipped period: it is periodic and reads the same value on
 * the entry after a miss as on a normal entry.  That is what TIM5 is for.
 * -------------------------------------------------------------------------- */
ITCM_FUNC uint16_t hal_carrier_position(void)
{
    return (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);
}

/* -----------------------------------------------------------------------------
 * Clock discipline (arch section 9.1).
 *
 * ARPE is enabled, so the new reload takes effect at the next update event
 * rather than truncating the period in progress.  __HAL_TIM_SET_AUTORELOAD
 * also updates htim1.Init.Period, keeping the handle honest -- worth the
 * second store on a path that runs at 1 kHz at most.
 * -------------------------------------------------------------------------- */
void hal_carrier_trim(int16_t delta_ticks)
{
    if (delta_ticks >  TRIM_AUTHORITY_TICKS) { delta_ticks =  TRIM_AUTHORITY_TICKS; }
    if (delta_ticks < -TRIM_AUTHORITY_TICKS) { delta_ticks = -TRIM_AUTHORITY_TICKS; }

    uint32_t arr = (uint32_t)((int32_t)CARRIER_ARR + delta_ticks);
    __HAL_TIM_SET_AUTORELOAD(&htim1, arr);

    /* CCR5 tracks ARR so the sampling aperture stays centred on the peak.
     * The peak IS ARR, so a trigger at a fixed absolute value would drift
     * away from it as ARR moves.  Small -- 64 ticks is 267 ns -- but the
     * correction is one subtraction.
     *
     * CCR4 is deliberately NOT adjusted.  It sets the slave reset instant and
     * therefore the protection sampling phase, and a 64-tick shift is under a
     * tenth of the 750-tick gap between protection samples, so the injected
     * trigger stays clear of both neighbours. */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_5, arr - (SAMPLE_APERTURE_TICKS / 2u));
}