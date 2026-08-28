/* =============================================================================
 * hal_sequencer.c  --  TIM2, TIM3, TIM4, TIM5, TIM6, and the start order.
 *
 * WHAT "SEQUENCER" MEANS HERE
 *   Not just TIM4.  This file owns every timer that is not the carrier and
 *   not the brake, and -- more importantly -- it owns the ORDER in which all
 *   of them, including the carrier, are started.
 *
 *   That ordering is the reason hal_pwm.c does not start TIM1 itself.  From
 *   the instant CEN is set on the carrier, it pulses both GETSENS lines,
 *   raises two DMA requests and triggers three converters, every 62.5 us,
 *   forever.  Every one of those consumers must already exist.  The function
 *   that knows the order should be the one that calls it.
 *
 * THE SLAVE-MODE SUBTLETY THAT COSTS PEOPLE A DAY
 *   TIM2, TIM3 and TIM4 are in slave reset mode on ITR0.  Reset mode zeroes
 *   CNT when the trigger arrives -- it does NOT enable the counter.  CEN must
 *   still be set on each slave independently, before the master starts.  A
 *   slave with CEN clear sits at zero being repeatedly reset and produces
 *   nothing, with no error anywhere.
 * ============================================================================= */

#include "hal.h"
#include "hal_private.h"
#include "hal_sections.h"
#include "board_limits.h"
#include "tim.h"                 /* CubeMX: htim2, htim3, htim4, htim5, htim6 */

/* -----------------------------------------------------------------------------
 * Clear a timer's update flag before enabling its interrupt.
 *
 * Configuring a timer generates an update event, which latches UIF.  Enable
 * the interrupt without clearing it first and the handler fires immediately,
 * before the timer has ever counted anything.  For TIM6 that means a deadline
 * trip during initialisation -- the drive enters the safe state before the
 * carrier has started, and the fault log says the control ISR stopped when it
 * had not yet begun.
 * -------------------------------------------------------------------------- */
static void clear_stale_update(TIM_HandleTypeDef *h)
{
    __HAL_TIM_CLEAR_FLAG(h, TIM_FLAG_UPDATE);
}

/* -----------------------------------------------------------------------------
 * Deadline monitor (arch section 7, layer L3).
 *
 * One-pulse mode, so the hardware clears CEN on overflow.  A kick therefore
 * has to re-enable the counter, not merely reload it -- which is also what
 * makes the monitor latch: once it has fired it stays stopped until the ISR
 * that is supposed to be running kicks it again.
 * -------------------------------------------------------------------------- */
ITCM_FUNC void hal_deadline_kick(void)
{
    __HAL_TIM_SET_COUNTER(&htim6, 0u);
    __HAL_TIM_ENABLE(&htim6);
}

void hal_deadline_stop(void)
{
    __HAL_TIM_DISABLE(&htim6);
    clear_stale_update(&htim6);
}

/* -----------------------------------------------------------------------------
 * Start everything, master last.
 * -------------------------------------------------------------------------- */
bool hal_init_sequencer(void)
{
    /* ---- 1. TIM5: the free-running timebase ---------------------------
     *
     * First, because everything below it may want to timestamp, and because
     * the interval check in the very first FOC ISR reads a value that must
     * already be advancing.
     *
     * HAL_TIM_Base_Start, not _Start_IT.  There is no overflow handler and
     * there must not be one: at 240 MHz the counter wraps every 17.9 s and
     * unsigned subtraction handles that with no code (common/fixed.h). */
    if (HAL_TIM_Base_Start(&htim5) != HAL_OK) {
        return false;
    }

    /* ---- 2. TIM6: deadline monitor, armed but NOT running --------------
     *
     * Deliberately not started.  It is kicked by the first FOC ISR entry; if
     * it were running now it would expire 100 us from here, during encoder
     * initialisation, and trip.
     *
     * Only the interrupt is enabled, and only after the stale UIF from
     * configuration is cleared. */
    clear_stale_update(&htim6);
    __HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE);

    /* ---- 3. TIM3: fieldbus phase detector ------------------------------
     *
     * Input capture on CH2 (SYNC0 from the LAN9253), slaved reset to TIM1 so
     * the captured count is a phase error directly, with no software latency
     * in it (arch section 9.1).
     *
     * Started before the carrier so that no SYNC0 edge is missed, though in
     * practice the fieldbus is not yet in an operational state. */
    clear_stale_update(&htim3);
    if (HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2) != HAL_OK) {
        return false;
    }

    /* ---- 4. TIM2: protection sampling at 320 kHz -----------------------
     *
     * CH2 in toggle mode drives the ADC regular trigger for all three
     * converters.  Nothing reads the results; the analog watchdogs do the
     * work and stay silent unless a threshold is crossed (arch section 5.3).
     *
     * TOGGLE gives ONE EDGE PER WRAP, so the ADCs must be configured for
     * BOTH edges or the rate silently halves to 160 kHz and the tier-1'
     * detection latency doubles.  Verify at stage 1 by counting conversions
     * per period on a scope: twenty, not ten. */
    clear_stale_update(&htim2);
    if (HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_2) != HAL_OK) {
        return false;
    }

    /* ---- 5. TIM4: acquisition sequencer --------------------------------
     *
     * CH1 and CH4 drive the two GETSENS pins as PWM outputs.  They carry
     * identical compare values so both encoders latch at the same instant,
     * which is what makes the transmission cross-check in section 8.3
     * meaningful -- a discrepancy is then genuine gear error rather than
     * partly an artefact of sampling the two ends at different moments.
     *
     * PWM mode 1 with the timer slaved-reset means the output is high from
     * the reset until the compare, giving a bounded pulse.  The iC-MBE needs
     * GETSENS returned low before EOT rises; a level held longer than the
     * process-data cycle raises an error flag in the master. */
    if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) != HAL_OK) { return false; }
    if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4) != HAL_OK) { return false; }

    /* CH2 and CH3 have no pins.  Their only job is to raise a DMA request
     * that writes CSTART to each encoder SPI, 6 us after the latch. */
    if (HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_2) != HAL_OK) { return false; }
    if (HAL_TIM_OC_Start(&htim4, TIM_CHANNEL_3) != HAL_OK) { return false; }

    /* THE BITS CUBEMX DOES NOT SET.
     *
     * A compare match raises a DMA request only if CCxDE is set in DIER.
     * CubeMX has no checkbox for this, and HAL sets it only inside
     * HAL_TIM_OC_Start_DMA -- which we cannot use, because that function
     * points the DMA at TIM4's own CCR register on the assumption you want
     * to update a compare value.  We want it pointed at SPI->CR1.
     *
     * So the channels are started without DMA above, and the request enables
     * are set here.  Without these two bits everything looks correct on a
     * scope -- the GETSENS pulses appear, the compare flags set -- and no SPI
     * transfer ever starts. */
    __HAL_TIM_ENABLE_DMA(&htim4, TIM_DMA_CC2);
    __HAL_TIM_ENABLE_DMA(&htim4, TIM_DMA_CC3);

    /* ---- 6. The carrier, last ------------------------------------------
     *
     * Everything above is now waiting on TIM1's TRGO.  The slaves have CEN
     * set and sit at whatever count they reached; the first reset aligns
     * them.  The ADCs are armed, the encoder masters are configured and
     * their DMA streams are armed (hal_init_encoders, called earlier).
     *
     * After this line the system is live at 16 kHz. */
    hal_pwm_carrier_start();

    return true;
}