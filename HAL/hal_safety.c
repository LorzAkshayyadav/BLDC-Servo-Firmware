/* =============================================================================
 * hal_safety.c  --  STO, the holding brake, and the terminal safe state.
 *
 * THIS FILE CROSSES THE LAYER BOUNDARY, DELIBERATELY
 *   hal_safe_state() calls upward into L2 and L3: foc_reset_integrators(),
 *   motion_reset_integrators() and scope_log_freeze().  Together with
 *   isr_vectors.c that makes two L1 files that call upward, and no third is
 *   permitted (see CODE_LAYOUT.md).
 *
 *   The alternative -- registering those as callbacks at init -- keeps L1
 *   pointing only downward, and fails silently if one is never registered.
 *   On a fault path, silence is the worst possible failure mode.  A direct
 *   call cannot be absent: it is a link error.
 *
 * THE BRAKE IS POWER-TO-RELEASE
 *   Spring-applied.  Duty 0 means ENGAGED, and the brake engages by itself
 *   the moment power is lost, TIM15's break fires, or the PWM stops.  Every
 *   failure mode of this file therefore ends with the joint held, which is
 *   why there is no symbol named BRAKE_HOLD anywhere in the API -- a future
 *   reader could too easily read "hold" as hold-engaged.
 * ============================================================================= */

#include "hal.h"
#include "hal_private.h"
#include "hal_sections.h"
#include "board_limits.h"
#include "contracts.h"
#include "foc.h"                 /* declared upward call */
#include "motion.h"              /* declared upward call */
#include "scope_log.h"           /* declared upward call */
#include "tim.h"                 /* CubeMX: htim15 */
#include "gpio.h"                /* CubeMX: pin macros */

/* -----------------------------------------------------------------------------
 * Brake duty.
 *
 * TIM15 ARR is 14999 at a 240 MHz kernel clock, so the carrier is 16 kHz and
 * full duty is 15000 counts.  Pull-in needs full voltage to drag the armature
 * off the friction plate against the spring; holding it clear needs far less,
 * and running at full duty continuously is where brake coils cook.
 *
 * OPEN: both figures come from the brake datasheet.  The hold value in
 * particular must stay above the drop-out threshold with margin across
 * temperature, because a coil that warms up and drops out mid-move engages
 * the brake against a driven motor.
 * -------------------------------------------------------------------------- */
#define BRAKE_ARR                 14999u
#define BRAKE_DUTY_ENGAGED        0u
#ifndef BRAKE_DUTY_RELEASE_PULL
#define BRAKE_DUTY_RELEASE_PULL   BRAKE_ARR            /* 100% */
#define BRAKE_DUTY_RELEASE_HOLD   (BRAKE_ARR / 2u)     /* 50%, PLACEHOLDER */
#warning "Brake pull/hold duties are placeholders -- take them from the datasheet"
#endif

/* Electrical decay plus mechanical seating.  On a power-to-release brake this
 * is the SLOW direction: cutting duty does not engage the brake, it only
 * starts the coil current decaying, and the spring cannot move the armature
 * until the field has collapsed.
 *
 * OPEN: measure it.  Scope the coil current against a load-cell or the load
 * encoder.  If the driver has only a plain freewheel diode the decay is an
 * L/R time constant of tens of milliseconds and dominates everything; a Zener
 * or TVS in series with the diode collapses it far faster at the cost of
 * dissipation in the clamp.  That is a schematic question, not a firmware
 * one. */
#ifndef BRAKE_ENGAGE_DELAY_US
#define BRAKE_ENGAGE_DELAY_US     50000u               /* 50 ms, PLACEHOLDER */
#warning "BRAKE_ENGAGE_DELAY_US is a placeholder -- measure at bring-up stage 6"
#endif

/* -----------------------------------------------------------------------------
 * STO channel monitors.
 *
 * Read as levels, never as edges.  The EXTI configuration on PB10/PB13 exists
 * so a change can wake the supervisory task, but the authoritative answer is
 * always the pin state right now -- an edge handler that missed a transition
 * would leave a stale belief about whether torque is permitted.
 *
 * OPEN: polarity.  These read true for "OK, not asserted"; confirm against
 * the STO circuit, because getting it inverted means hal_pwm_outputs_enable()
 * cheerfully enables the bridge while STO is active.
 * -------------------------------------------------------------------------- */
bool hal_sto_channel_1_ok(void)
{
    return HAL_GPIO_ReadPin(STO1_INPUT_MON_GPIO_Port,
                            STO1_INPUT_MON_Pin) == GPIO_PIN_SET;
}

bool hal_sto_channel_2_ok(void)
{
    return HAL_GPIO_ReadPin(STO2_INPUT_MON_GPIO_Port,
                            STO2_INPUT_MON_Pin) == GPIO_PIN_SET;
}

/* Assert both MCU-driven STO channels.  Independent of the timer break path:
 * arch section 5.5 item 2 wants both, because the break depends on TIM1 still
 * being alive and STO does not. */
ITCM_FUNC static void sto_assert_both(void)
{
    HAL_GPIO_WritePin(STO1_MCU_CTRL_GPIO_Port, STO1_MCU_CTRL_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(STO2_MCU_CTRL_GPIO_Port, STO2_MCU_CTRL_Pin, GPIO_PIN_RESET);
}

/* -----------------------------------------------------------------------------
 * Brake.
 * -------------------------------------------------------------------------- */
void hal_brake_release_pull(void)
{
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, BRAKE_DUTY_RELEASE_PULL);
}

void hal_brake_release_hold(void)
{
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, BRAKE_DUTY_RELEASE_HOLD);
}

ITCM_FUNC void hal_brake_engage(void)
{
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, BRAKE_DUTY_ENGAGED);
}

uint32_t hal_brake_engage_delay_us(void)
{
    return BRAKE_ENGAGE_DELAY_US;
}

/* -----------------------------------------------------------------------------
 * The safe state.  Arch section 5.5, in order.
 *
 * Every fault path from every tier converges here: the analog watchdogs, the
 * tier-2 checks in the ISR, the deadline monitor, the supervisory
 * cross-checks, and a failed initialisation.
 *
 * Runs at whatever priority the caller had, including priority 0 from a
 * watchdog handler, so everything in it must be bounded and non-blocking.
 * There is no delay, no wait and no loop below.
 * -------------------------------------------------------------------------- */
ITCM_FUNC void hal_safe_state(hal_trip_cause_t cause)
{
    /* 1. Software break.
     *
     * First, and before anything that takes time, because until this executes
     * the bridge is still driving whatever duty the last ISR wrote.  Latches
     * exactly as a hardware break would, so recovery needs the same explicit
     * action. */
    hal_pwm_break_now();

    /* 2. Both STO channels -- the independent path, not dependent on the
     *    timer.  If TIM1 has stopped, step 1 did nothing and this is what
     *    removes torque. */
    sto_assert_both();

    /* 3. Integrators.
     *
     * OMIT THIS AND RE-ENABLE TRIPS INSTANTLY.  The current loop restarts
     * with a wound-up integrator demanding full voltage into a stationary
     * rotor, which presents as a hardware fault and is not one (arch section
     * 5.5 item 3).  This is the step people leave out.
     *
     * Both calls are the declared upward crossing described at the top. */
    foc_reset_integrators();
    motion_reset_integrators();

    /* 4. Brake.
     *
     * Commanded here, but NOT waited for.  Arch section 5.5 item 4 asks for
     * the brake to hold before torque is removed, and on a commanded stop
     * motion/brake_seq.c does exactly that ordering with the delay from
     * hal_brake_engage_delay_us().
     *
     * On a fault path there is nothing to sequence: torque has already gone
     * in steps 1 and 2, and blocking here for tens of milliseconds inside a
     * priority-0 handler is not an option.  So the load is unheld for the
     * brake's engagement time.  That is inherent to STO on any drive, it
     * belongs in the risk register rather than in firmware, and pretending
     * otherwise by adding a delay here would make it worse, not better. */
    hal_brake_engage();

    /* 5. Latch the cause.
     *
     * Atomic bit-set only, never a read-modify-write, because this runs at
     * several priorities and a lower-priority writer must not lose a bit set
     * by a higher one (arch section 12).  First cause wins for the summary
     * field; the bit field accumulates all of them, which matters because a
     * single real fault usually produces a cascade. */
    fault_set(cause);

    /* 6. Freeze the scope.
     *
     * Idempotent, so the first freeze wins.  In a cascade -- overcurrent
     * trips the watchdog, the safe state removes torque, the current collapse
     * trips a tier-2 check -- the later ones must not overwrite the capture
     * of the cause with a capture of the consequences. */
    scope_log_freeze();

    /* 7. Stop the deadline monitor.
     *
     * Otherwise it expires 100 us from now and trips again, burying the real
     * cause under a deadline fault.  It is re-armed by the first ISR entry
     * after a successful re-enable. */
    hal_deadline_stop();

    /* Deliberately absent: any attempt to re-enable, retry, or recover.
     * Arch section 5.5 item 7 -- re-enable only by explicit command through
     * the drive state machine, never self-clearing. */
}

/* -----------------------------------------------------------------------------
 * Initialisation.  Runs before the ADCs, the encoders and the carrier.
 * -------------------------------------------------------------------------- */
bool hal_init_safety(void)
{
    /* Brake first, and engaged.  It already is -- spring-applied, and TIM15
     * has not started -- but making it explicit means the intent survives
     * someone later changing the start order. */
    hal_brake_engage();

    if (HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1) != HAL_OK) {
        return false;
    }

    /* STO outputs asserted until something deliberately releases them.  The
     * drive comes up with torque prohibited, and the drive state machine has
     * to ask for it. */
    sto_assert_both();

    /* Arm TIM1's outputs with MOE off.  hal_pwm.c owns the registers; the
     * decision that this happens during safety init, before anything else can
     * touch the bridge, belongs here. */
    hal_pwm_configure();

    /* Do NOT fail on STO being asserted.  At power-on it usually is, because
     * the external safety circuit has not released yet, and refusing to
     * initialise would prevent the drive from ever reaching a state where it
     * could report why.  hal_pwm_outputs_enable() is where that check bites. */
    return true;
}