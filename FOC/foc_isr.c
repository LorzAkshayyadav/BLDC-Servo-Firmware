/* =============================================================================
 * foc_isr.c  --  L2.  The single interrupt on the control path.
 *
 * Resident in instruction TCM.  Contains NO register access and NO pin
 * numbers; everything device-specific is behind hal.h.  This file must compile
 * for a host PC against test/host/hal_stub.c -- if it does not, the compiler
 * errors are an exhaustive list of your layer violations.
 *
 * WHY THIS FUNCTION NEVER WAITS FOR ANYTHING
 *   TIM1 raises no interrupt at all.  It drives the converters through an
 *   internal trigger, in hardware.  When the injected sequence finishes, the
 *   converter raises the interrupt, and that handler is this function.  The
 *   ISR cannot begin before its input data exists, because the data existing
 *   is what caused it to begin.  There is no code path where the handler runs
 *   and the data is absent, and therefore no readiness check to write.
 *   (Arch section 4.1.)
 *
 * ORDER OF OPERATIONS IS LOAD-BEARING.  Each numbered step below has a reason
 * recorded against it.  Reordering them silently breaks something.
 * ========================================================================== */

#include "hal/hal.h"
#include "hal/hal_sections.h"
#include "common/contracts.h"
#include "config/board_limits.h"
#include "config/control_params.h"
#include "config/motor_params.h"
#include "foc/foc.h"
#include "foc/transforms.h"
#include "foc/current_pi.h"
#include "foc/svpwm.h"
#include "foc/angle.h"
#include "foc/tier2_checks.h"
#include "foc/scope_log.h"

/* VELOCITY_ACCUM_DEPTH (config/control_params.h) and HAL_MOTION_DIVIDER
 * (hal/hal.h) are the same divider counted from two directions that cannot
 * include each other (arch section 2); this is the file that includes both,
 * so it is the one that can catch them drifting apart. */
_Static_assert(VELOCITY_ACCUM_DEPTH == HAL_MOTION_DIVIDER,
               "velocity accumulation depth must match the motion pend divider");

/* ---- state owned exclusively by this context, in zero-wait-state memory --- */
static DTCM_BSS uint32_t s_last_entry_ticks;
static DTCM_BSS uint32_t s_period_counter;
static DTCM_BSS uint32_t s_prev_position_motor;
static DTCM_BSS uint32_t s_late_entry_events;
static DTCM_BSS pi_state_t s_pi_d;
static DTCM_BSS pi_state_t s_pi_q;

/* Jitter band for the interval check.  Set from the histogram measured at
 * bring-up stage 8, not guessed.  Worst-case interference from above is about
 * 13 us per arch section 6.1, but the FOC ISR sits at priority 1 with only the
 * priority-0 fault paths above it, so the real spread should be small. */
#define INTERVAL_NOMINAL  HAL_TICKS_PER_PERIOD
#define INTERVAL_TOL      (INTERVAL_NOMINAL / 20u)      /* +-5 %, provisional */

/**
 * @brief Initialise torque-control state. Call before the carrier starts.
 *
 * Zeroes both PI integrators, the period counter, the velocity accumulators
 * and the previous-position history, and marks the first period so the
 * interval check does not fault on an interval measured against a
 * meaningless initial timestamp: without this, entry 1's
 * `delta = now - s_last_entry_ticks` would be measured against whatever
 * s_last_entry_ticks happened to hold at boot and almost certainly trip
 * HAL_TRIP_PERIOD_MISSED before the first period ever completes.
 */
void foc_init(void)
{
    foc_reset_integrators();

    s_period_counter      = 0u;
    s_late_entry_events   = 0u;
    s_prev_position_motor = 0u;

    g_velocity.slot[0] = (velocity_accum_t){0};
    g_velocity.slot[1] = (velocity_accum_t){0};

    s_last_entry_ticks = hal_time_now();
}

/**
 * @brief L2 control ISR: one full current-loop period, start to finish.
 *
 * Entered once per PWM carrier period, triggered by the injected ADC
 * conversion completing in hardware -- never waited on, per the file banner
 * above. Runs the full chain in a fixed order: deadline kick, interval
 * classification, acquisition, tier-2 checks, Clarke/Park, current PI,
 * inverse Park/SVPWM, duty output, velocity accumulation, feedback publish,
 * lower-layer pend, scope log, and re-arm. Each step's ordering rationale is
 * documented inline above that step; do not reorder them.
 *
 * Returns early (without reaching re-arm) on any fault path, having already
 * called @ref hal_safe_state for that cause.
 *
 * Resident in ITCM; must not block or call anything that can block.
 */
ITCM_FUNC void foc_isr(void)
{
    /* ---------------------------------------------------------------------
     * 1.  Reload the deadline monitor FIRST.
     *     If anything below this line hangs, TIM6 expires and its priority-0
     *     handler forces the safe state.  Reloading later would leave a
     *     window in which a hang is undetectable.
     * ------------------------------------------------------------------- */
    hal_deadline_kick();

    /* ---------------------------------------------------------------------
     * 2.  Timing.  Read the free-running clock and classify the interval.
     *
     *     TIM1's counter is periodic, so it can catch a LATE entry but not a
     *     SKIPPED period -- a skipped period leaves it looking exactly
     *     normal.  A software counter incremented here cannot help either: it
     *     counts ISR executions, not periods, so it has no way to notice its
     *     own absence.  Hence the independent monotonic clock.
     *     (Arch section 7, layers L1 and L2.)
     * ------------------------------------------------------------------- */
    const uint32_t now   = hal_time_now();
    const uint32_t delta = now - s_last_entry_ticks;   /* wrap-safe */
    s_last_entry_ticks   = now;

    if (delta > INTERVAL_NOMINAL + INTERVAL_TOL) {
        /* Missed period.  The angle is stale by a full period and the previous
         * duty was reapplied blind into a moving rotor.  For a robot joint
         * there is no benefit to limping. */
        hal_safe_state(HAL_TRIP_PERIOD_MISSED);
        return;
    }
    if (delta < INTERVAL_NOMINAL - INTERVAL_TOL) {
        /* Two entries too close together: a spurious trigger or re-entrancy.
         * The acquisition chain is not behaving as designed. */
        hal_safe_state(HAL_TRIP_PERIOD_SHORT);
        return;
    }
    {   /* Late entry: data is valid, only the timing is not.  Complete the
         * period normally, count the event, fault only on repetition.  The
         * deviation doubles as a free jitter measurement. */
        const uint16_t carrier = hal_carrier_position();
        if (carrier > CARRIER_ENTRY_EXPECTED + CARRIER_ENTRY_TOL) {
            s_late_entry_events++;
            if (s_late_entry_events > LATE_ENTRY_LIMIT) {
                hal_safe_state(HAL_TRIP_DEADLINE);
                return;
            }
        } else if (s_late_entry_events) {
            s_late_entry_events--;      /* leaky, so isolated events decay */
        }
    }

    /* ---------------------------------------------------------------------
     * 3.  Take ONE copy of every shared index, here, once.
     *     Re-reading mid-computation reintroduces the tear the double buffer
     *     exists to avoid.
     * ------------------------------------------------------------------- */
    const params_t *p = &g_params.slot[CONTRACT_TAKE(g_params)];
    const current_ref_t ref = { .word = g_current_ref.word };

    /* ---------------------------------------------------------------------
     * 4.  Acquisition.  All of it is already resident: the injected group
     *     converted at the carrier peak, and the encoder frames landed about
     *     11 us before that.  Nothing here waits.
     * ------------------------------------------------------------------- */
    hal_analog_t a;
    hal_adc_read(&a);

    hal_enc_sample_t enc_m, enc_l;
    hal_enc_read(HAL_ENC_MOTOR, &enc_m);
    hal_enc_read(HAL_ENC_LOAD,  &enc_l);

    /* The motor encoder feeds commutation: a stale or CRC-failed frame is
     * fatal this period.  The load encoder feeds the 4 kHz position loop and
     * could arguably tolerate one period, but start strict and relax only if
     * measurement shows it firing. */
    if (!enc_m.fresh)      { hal_safe_state(HAL_TRIP_ENC_MOTOR_STALE);  return; }
    if (!enc_m.status_ok)  { hal_safe_state(HAL_TRIP_ENC_MOTOR_STATUS); return; }
    if (!enc_l.fresh)      { hal_safe_state(HAL_TRIP_ENC_LOAD_STALE);   return; }
    if (!enc_l.status_ok)  { hal_safe_state(HAL_TRIP_ENC_LOAD_STATUS);  return; }

    /* ---------------------------------------------------------------------
     * 5.  Offset removal (L2's job per arch section 2; hal already applied the
     *     low-side sign inversion and the INA241 gain/shunt scale).
     * ------------------------------------------------------------------- */
    const int32_t i_b = a.i_b_ma - MOTOR_I_OFFSET_B_MA;
    const int32_t i_c = a.i_c_ma - MOTOR_I_OFFSET_C_MA;
    const int32_t i_a = a.i_a_ma - MOTOR_I_OFFSET_A_MA;

    /* ---------------------------------------------------------------------
     * 6.  Tier-2 checks that must run before the control chain uses the data.
     *     The sum check detects a failed sensor, a saturated amplifier or a
     *     ground fault.  Phase A's timing relationship to B and C is
     *     irrelevant for it, which is why phase A on the independent converter
     *     costs nothing (arch section 4.4).
     * ------------------------------------------------------------------- */
    hal_trip_cause_t trip = tier2_fast(i_a, i_b, i_c, a.v_bus_mv, p);
    if (trip != HAL_TRIP_NONE) { hal_safe_state(trip); return; }

    /* ---------------------------------------------------------------------
     * 7.  Angle.  Sample once, apply the advance term once, and use the
     *     resulting sine/cosine pair for BOTH the forward and the reverse
     *     transform.  That makes coherence between them structural rather
     *     than something to remember.
     *
     *     The advance term compensates the delay between latching the angle
     *     (10.94 us) and the new duty taking effect (next reload).  With
     *     CCR4 = 2625 that is 20.3 us of acquisition staleness plus one
     *     period of actuation delay.  Tune the coefficient by sweeping it at
     *     constant speed and load and taking the value that minimises
     *     current (arch section 8.1).
     * ------------------------------------------------------------------- */
    const uint32_t theta = angle_advance(enc_m.angle, s_prev_position_motor,
                                         delta, p->angle_advance_per_rpm);
    sincos_t sc;
    angle_sincos(theta, &sc);          /* table lookup in data TCM */

    /* ---------------------------------------------------------------------
     * 8.  Control chain.  Two currents carry all the information there is for
     *     a three-wire motor; the third is determined by the other two.
     * ------------------------------------------------------------------- */
    dq_t   i_dq  = park(clarke_2ph(i_b, i_c), &sc);
    dq_t   v_dq;
    v_dq.d = pi_step(&s_pi_d, ref.v.i_d_ma - i_dq.d, p->kp_current_q15,
                     p->ki_current_q15, a.v_bus_mv);
    v_dq.q = pi_step(&s_pi_q, ref.v.i_q_ma - i_dq.q, p->kp_current_q15,
                     p->ki_current_q15, a.v_bus_mv);

    duty_t duty = svpwm(inv_park(v_dq, &sc), a.v_bus_mv);
    duty = svpwm_deadtime_compensate(duty, i_b, i_c);

    /* ---------------------------------------------------------------------
     * 9.  Output.  hal_pwm applies the low-side-sense duty clamp; L2 reports
     *     saturation but does not duplicate the clamp arithmetic.
     * ------------------------------------------------------------------- */
    hal_pwm_set_duty(duty.a, duty.b, duty.c);

    /* ---------------------------------------------------------------------
     * 10.  Velocity accumulation.  Accumulate every period, publish every
     *      fourth.  The four consecutive differences telescope to the total
     *      displacement, so nothing is discarded and the quantisation noise
     *      floor drops by four.  dt is the MEASURED interval, because the
     *      clock-discipline loop trims the carrier and the nominal period is
     *      therefore not constant.
     * ------------------------------------------------------------------- */
    {
        velocity_accum_t *acc = CONTRACT_WRITE_SLOT(g_velocity);
        acc->d_position += (int32_t)(enc_m.position - s_prev_position_motor);
        acc->d_ticks    += delta;
        acc->samples++;
        s_prev_position_motor = enc_m.position;
    }

    /* ---------------------------------------------------------------------
     * 11.  Publish feedback, then pend the slower layers.
     *      The ISR never CALLS motion or supervisor -- it pends them, and the
     *      NVIC services them as soon as the CPU drops below priority 1.
     *      A pending interrupt stays pending; nothing is lost.
     * ------------------------------------------------------------------- */
    {
        feedback_t *f = CONTRACT_WRITE_SLOT(g_feedback);
        f->angle_electrical = theta;
        f->position_motor   = enc_m.position;
        f->position_load    = enc_l.position;
        f->i_d_ma           = i_dq.d;
        f->i_q_ma           = i_dq.q;
        f->v_bus_mv         = a.v_bus_mv;
        f->status_word      = g_fault_word;
        f->stamp            = now;
        f->duty_max_applied = duty.max;
        f->duty_clamped     = duty.clamped;
        CONTRACT_PUBLISH(g_feedback);
    }

    s_period_counter++;

    if ((s_period_counter % HAL_MOTION_DIVIDER) == 0u) {
        CONTRACT_PUBLISH(g_velocity);
        {   /* zero the newly inactive half; the ISR owns both, so the motion
             * task never writes and P4 holds */
            velocity_accum_t *w = CONTRACT_WRITE_SLOT(g_velocity);
            w->d_position = 0; w->d_ticks = 0u; w->samples = 0u;
        }
        hal_pend_motion();
    }
    if ((s_period_counter % HAL_SUPERVISOR_DIVIDER) == 0u) {
        hal_pend_supervisor();
        /* The independent watchdog is refreshed ONLY by the supervisory task.
         * That refresh proves the entire chain end to end, because the
         * supervisory task only runs if this ISR pended it. */
    }

    /* ---------------------------------------------------------------------
     * 12.  Scope logging.  Eight channels at the full control rate, written
     *      circularly, well under one percent of the period budget.  The
     *      decisive property is pre-trigger capture: any fault freezes the
     *      buffer, so you get the half second leading up to every trip
     *      automatically, including the ones nobody watched.
     * ------------------------------------------------------------------- */
    scope_log(now, i_dq, v_dq, theta, a.v_bus_mv, duty);

    /* ---------------------------------------------------------------------
     * 13.  Re-arm the acquisition chain.  Clears SPI EOT on both buses --
     *      required before the next CSTART takes effect -- clears the DMA
     *      transfer-complete flags BEFORE re-arming the normal-mode streams,
     *      cycles the software chip select on the load encoder, and fires the
     *      pre-armed PDVALIDx reset.
     *
     *      Clear-then-arm is the order that matters.  Arm-then-clear leaves a
     *      window in which a fast transfer sets the flag and you immediately
     *      clear it, so a genuinely missing frame then reads as present.
     * ------------------------------------------------------------------- */
    hal_enc_rearm();
}

/* =============================================================================
 * foc_reset_integrators()  --  called directly by hal_safety.c.
 *
 * hal_safety.c is one of the two files CODE_LAYOUT.md's layer-rules table
 * allows to call upward into L2 (the other is isr_vectors.c, for foc_isr()
 * itself), specifically for arch section 5.5 steps 3 and 6. A direct call
 * cannot be silently absent the way a forgotten registration call could be;
 * Tools/layer_check.py enforces that no third file gets the same permission.
 * ========================================================================== */
void foc_reset_integrators(void)
{
    current_pi_reset(&s_pi_d);
    current_pi_reset(&s_pi_q);
}

/* =============================================================================
 * The vector-level handler lives in hal/isr_vectors.c, not here, because it
 * touches ADC registers.  Its shape is fixed by arch section 5.4 and risk R3:
 * the phase B and C analog watchdogs share ADC_IRQn with this ISR, so their
 * flags are checked FIRST, before anything else in that handler.  In the worst
 * case a crossing on those phases is serviced when this ISR finishes -- still
 * an order of magnitude better than one PWM period.
 *
 * Do NOT route this through HAL_ADC_IRQHandler.  The generated dispatcher walks
 * every flag and evaluates callbacks before your code runs: hundreds of cycles
 * of variable overhead on your hardest deadline (arch section 6.1).
 * ========================================================================== */
