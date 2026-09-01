/* =============================================================================
 * foc_isr.c  --  the 16 kHz control ISR.  L2.  No register access, no pin
 *                numbers; this file must compile for a host.
 *
 * ONE INTERRUPT PER PERIOD
 *   TIM1 raises none.  It drives the converters in hardware, and the
 *   converter raises the interrupt when the injected sequence completes.  So
 *   this function cannot begin before its inputs exist, because the data
 *   existing is what caused it to begin -- there is no readiness check here
 *   and there must never be one (arch section 4.1).
 *
 * THE ORDER IS THE DESIGN
 *   Deadline monitor, timing classification, one pointer copy, acquisition,
 *   validity, tier-2, angle, control, OUTPUT, then everything that can wait.
 *   Duty reaches the gates before velocity accumulation, publishing, logging
 *   or re-arming, because those cost microseconds and the actuation delay is
 *   already a period.
 * ============================================================================= */

#include "foc.h"
#include "transforms.h"
#include "current_pi.h"
#include "svpwm.h"
#include "angle.h"
#include "tier2_checks.h"
#include "scope_log.h"
#include "hal.h"
#include "contracts.h"
#include "control_params.h"
#include "motor_params.h"
#include "board_limits.h"
#include "hal_sections.h"

/* Nominal interval in free-running-clock ticks.  Not a constant in practice:
 * the section 9.1 trim moves the carrier period by a fraction of a percent to
 * track the distributed clock, which is exactly why the MEASURED interval is
 * used for dt everywhere below and this value only bounds the fault check. */
#define INTERVAL_NOMINAL   (CARRIER_TICKS)
#define INTERVAL_TOL       (CARRIER_TICKS / 4u)

/* At entry the carrier is just past the peak and counting DOWN, so a LATER
 * entry reads a SMALLER counter value.  Getting this direction wrong gives a
 * check that never fires. */

#define LATE_ENTRY_LIMIT        8u

/* VELOCITY_ACCUM_DEPTH (config/control_params.h) and HAL_MOTION_DIVIDER
 * (hal/hal.h) are the same divider counted from two directions that cannot
 * include each other; this is the file that includes both, so it is the one
 * that can catch them drifting apart. */
_Static_assert(VELOCITY_ACCUM_DEPTH == HAL_MOTION_DIVIDER,
               "velocity accumulation depth must match the motion pend divider");

static DTCM_BSS struct {
    uint32_t last_entry_ticks;
    uint32_t prev_position_motor;
    uint32_t period_counter;
    uint32_t late_entry_events;
    bool     first_period;
    pi_state_t pi_d;
    pi_state_t pi_q;
} s;

/* -----------------------------------------------------------------------------
 * Initialisation.  Runs before the carrier starts.
 * -------------------------------------------------------------------------- */
void foc_init(void)
{
    s.last_entry_ticks    = 0u;
    s.prev_position_motor = 0u;
    s.period_counter      = 0u;
    s.late_entry_events   = 0u;

    /* THE FIRST-ENTRY GUARD.
     *
     * On entry one, last_entry_ticks is meaningless, so the interval would be
     * whatever the free-running clock happened to hold -- almost certainly
     * outside tolerance, and the drive would trip on a missed period before
     * it had ever run one. */
    s.first_period = true;

    current_pi_reset(&s.pi_d);
    current_pi_reset(&s.pi_q);
}

void foc_reset_integrators(void)
{
    /* Called from hal_safe_state(), possibly at priority 0.  Two words, no
     * loop, safe from any context.  Arch section 5.5 item 3: omit this and
     * re-enable trips instantly, because the loop restarts with a wound-up
     * integrator demanding full voltage into a stationary rotor. */
    current_pi_reset(&s.pi_d);
    current_pi_reset(&s.pi_q);
}

/* -----------------------------------------------------------------------------
 * The ISR.
 * -------------------------------------------------------------------------- */
ITCM_FUNC void foc_isr(void)
{
    /* 1. Deadline monitor first.  If anything below hangs, TIM6 expires and
     *    its priority-0 handler forces the safe state.  Reloading later would
     *    leave a window in which a hang is undetectable. */
    hal_deadline_kick();

    const uint32_t t_entry = hal_time_now();
    const uint32_t delta   = t_entry - s.last_entry_ticks;   /* wrap-safe */
    s.last_entry_ticks     = t_entry;

    /* 2. Timing classification.
     *
     *    TIM1's counter is periodic: it catches a LATE entry but not a
     *    SKIPPED period, which leaves it looking exactly normal.  A software
     *    counter incremented here cannot help either -- it counts ISR
     *    executions, not periods, so it cannot notice its own absence.  Hence
     *    the independent monotonic clock (arch section 7, L1 and L2). */
    if (s.first_period) {
        s.first_period = false;
    } else if (delta > INTERVAL_NOMINAL + INTERVAL_TOL) {
        /* The angle is stale by a full period and the previous duty was
         * reapplied blind into a moving rotor.  For a robot joint there is no
         * benefit to limping. */
        hal_safe_state(HAL_TRIP_PERIOD_MISSED);
        return;
    } else if (delta < INTERVAL_NOMINAL - INTERVAL_TOL) {
        hal_safe_state(HAL_TRIP_PERIOD_SHORT);
        return;
    } else {
        const uint16_t carrier = hal_carrier_position();
        if (carrier < CARRIER_ENTRY_EXPECTED - CARRIER_ENTRY_TOL) {
            /* Late, not missed: the data is valid, only the timing is not.
             * Complete the period normally and fault only on repetition.  The
             * deviation doubles as a free jitter measurement. */
            if (++s.late_entry_events > LATE_ENTRY_LIMIT) {
                hal_safe_state(HAL_TRIP_DEADLINE);
                return;
            }
        } else if (s.late_entry_events != 0u) {
            s.late_entry_events--;      /* leaky, so isolated events decay */
        }
    }

    /* 3. ONE copy of every shared datum, here, once.  Re-reading mid-ISR
     *    reintroduces the tear the double buffer exists to avoid. */
    const params_t *p = &g_params.slot[CONTRACT_TAKE(g_params)];
    const current_ref_t ref = { .word = g_current_ref.word };

    /* 4. Acquisition.  Everything is already resident: the injected group
     *    converted at the carrier peak, the encoder frames landed ~11 us
     *    earlier.  Nothing waits. */
    hal_analog_t a;
    hal_adc_read(&a);

    hal_enc_sample_t enc_m, enc_l;
    hal_enc_read(HAL_ENC_MOTOR, &enc_m);
    hal_enc_read(HAL_ENC_LOAD,  &enc_l);

    /* Offset removal (L2's job per arch section 2; hal already applied the
     * low-side sign inversion and the INA241 gain/shunt scale). */
    const int32_t i_b = a.i_b_ma - MOTOR_I_OFFSET_B_MA;
    const int32_t i_c = a.i_c_ma - MOTOR_I_OFFSET_C_MA;
    const int32_t i_a = a.i_a_ma - MOTOR_I_OFFSET_A_MA;

    /* 5. Validity.  The motor encoder feeds commutation, so a stale or
     *    CRC-failed frame is fatal this period.  The load encoder feeds the
     *    4 kHz position loop and could arguably tolerate one period; start
     *    strict and relax only if measurement shows it firing. */
    if (!enc_m.fresh)     { hal_safe_state(HAL_TRIP_ENC_MOTOR_STALE);  return; }
    if (!enc_m.status_ok) { hal_safe_state(HAL_TRIP_ENC_MOTOR_STATUS); return; }
    if (!enc_l.fresh)     { hal_safe_state(HAL_TRIP_ENC_LOAD_STALE);   return; }
    if (!enc_l.status_ok) { hal_safe_state(HAL_TRIP_ENC_LOAD_STATUS);  return; }

    /* 6. Tier-2 checks, before the control chain uses the data.  The sum
     *    check detects a failed sensor, a saturated amplifier or a ground
     *    fault; phase A's timing relationship to B and C is irrelevant for
     *    it, which is why phase A on the independent converter costs nothing
     *    (arch section 4.4). */
    hal_trip_cause_t trip = tier2_fast(i_a, i_b, i_c, a.v_bus_mv, p);
    if (trip != HAL_TRIP_NONE) { hal_safe_state(trip); return; }

    /* 7. Angle.  Sampled once, advance applied once, and the resulting
     *    sine/cosine pair used for BOTH transforms -- which makes coherence
     *    between them structural rather than something to remember.
     *
     *    The advance compensates the delay from latching the angle at
     *    10.94 us to the new duty taking effect at the next reload.  Tune the
     *    coefficient by sweeping at constant speed and load and taking the
     *    value that minimises current (arch section 8.1).
     *
     *    In the forced-angle modes the encoder is ignored and the angle is
     *    generated here, which is what makes stages 3 and 4 possible without
     *    a valid commutation offset. */
    angle_t theta;
    switch (p->ctrl_mode) {
    case CTRL_MODE_OPEN_LOOP_VOLTAGE:
    case CTRL_MODE_CURRENT_FORCED:
        theta = angle_forced_step(p->forced_speed_rpm, delta);
        break;
    default:
        theta = angle_advance(enc_m.angle, s.prev_position_motor, delta,
                              p->angle_advance_per_rpm);
        break;
    }

    sincos_t sc;
    angle_sincos(theta, &sc);           /* table lookup, data TCM */

    /* 8. Control chain.  Two currents carry all the information there is for
     *    a three-wire motor; the third is determined by the other two, which
     *    is why the control path uses only the guaranteed-simultaneous pair
     *    (arch section 4.4). */
    const ab_t i_ab = clarke_2ph(i_b, i_c);
    const dq_t i_dq = park(i_ab, &sc);

    dq_t v_dq;
    switch (p->ctrl_mode) {
    case CTRL_MODE_IDLE:
        v_dq.d = 0; v_dq.q = 0;
        break;

    case CTRL_MODE_FIXED_DUTY:
    case CTRL_MODE_OPEN_LOOP_VOLTAGE:
        /* Voltage commanded directly; the current loop is open.  Stage 1
         * uses this with the motor disconnected, stage 3 with it connected
         * and the angle forced. */
        v_dq.d = ref.v.i_d_ma;          /* reinterpreted as a voltage demand */
        v_dq.q = ref.v.i_q_ma;
        break;

    default:
        v_dq.d = pi_step(&s.pi_d, ref.v.i_d_ma - i_dq.d,
                         p->kp_current_q15, p->ki_current_q15, a.v_bus_mv);
        v_dq.q = pi_step(&s.pi_q, ref.v.i_q_ma - i_dq.q,
                         p->kp_current_q15, p->ki_current_q15, a.v_bus_mv);
        break;
    }

    duty_t duty = svpwm(inv_park(v_dq, &sc), a.v_bus_mv);
    duty = svpwm_deadtime_compensate(duty, i_b, i_c);

    /* 9. OUTPUT.  Everything below this line can wait; this cannot.
     *    hal_pwm applies the low-side duty clamp internally, so L2 must not
     *    duplicate that arithmetic. */
    hal_pwm_set_duty(duty.a, duty.b, duty.c);

    /* 10. Velocity accumulation.  Every period, published every fourth.
     *     Four consecutive differences telescope to the total displacement,
     *     so nothing is discarded and the quantisation noise floor drops by
     *     four.  dt is the MEASURED interval because the clock-discipline
     *     loop trims the carrier and the nominal period is not constant. */
    {
        velocity_accum_t *acc = CONTRACT_WRITE_SLOT(g_velocity);
        acc->d_position += (int32_t)(enc_m.position - s.prev_position_motor);
        acc->d_ticks    += delta;
        acc->samples++;
    }
    s.prev_position_motor = enc_m.position;

    /* 11. Publish feedback, then pend the slower layers.
     *     The ISR never CALLS motion or the supervisor -- it pends them, and
     *     the NVIC services them once the CPU drops below priority 1.  A
     *     pending interrupt stays pending; nothing is lost. */
    {
        feedback_t *f = CONTRACT_WRITE_SLOT(g_feedback);
        f->angle_electrical = theta;
        f->position_motor   = enc_m.position;
        f->position_load    = enc_l.position;
        f->i_d_ma           = i_dq.d;
        f->i_q_ma           = i_dq.q;
        f->v_bus_mv         = a.v_bus_mv;
        f->status_word      = g_fault_word;
        f->stamp            = t_entry;
        f->duty_max_applied = duty.max;
        f->duty_clamped     = duty.clamped;
        CONTRACT_PUBLISH(g_feedback);
    }

    s.period_counter++;

    if ((s.period_counter & (HAL_MOTION_DIVIDER - 1u)) == 0u) {
        CONTRACT_PUBLISH(g_velocity);
        {   /* Zero the newly inactive half.  The ISR owns both slots, so the
             * motion task never writes and single-writer (P4) holds. */
            velocity_accum_t *w = CONTRACT_WRITE_SLOT(g_velocity);
            w->d_position = 0; w->d_ticks = 0u; w->samples = 0u;
        }
        hal_pend_motion();
    }
    if ((s.period_counter & (HAL_SUPERVISOR_DIVIDER - 1u)) == 0u) {
        /* The independent watchdog is refreshed ONLY by the supervisory task.
         * That refresh proves the whole chain end to end, because the
         * supervisory task only runs if this ISR pended it. */
        hal_pend_supervisor();
    }

    /* 12. Re-arm the acquisition chain.  Clears SPI EOT on both buses --
     *     required before the next CSTART takes effect -- clears the DMA
     *     transfer-complete flags BEFORE re-arming the normal-mode streams,
     *     cycles the software chip select on the load encoder, and fires the
     *     pre-armed PDVALIDx reset.
     *
     *     Clear-then-arm is the order that matters.  Arm-then-clear leaves a
     *     window in which a fast transfer sets the flag and it is immediately
     *     cleared, so a genuinely missing frame then reads as present. */
    hal_enc_rearm();

    /* 13. Scope.  Last, so isr_ticks measures the whole period's work.
     *
     *     Every field is filled.  A partially populated frame is worse than
     *     no frame: the channel map is chosen at runtime, so an unwritten
     *     member plots as a flat zero that looks like a real measurement. */
    {
        scope_frame_t log;
        log.i_a = i_a;   log.i_b = i_b;   log.i_c = i_c;
        log.i_alpha = i_ab.alpha; log.i_beta = i_ab.beta;
        log.i_d = i_dq.d;            log.i_q = i_dq.q;
        log.i_d_ref = ref.v.i_d_ma;  log.i_q_ref = ref.v.i_q_ma;
        log.v_d = v_dq.d;            log.v_q = v_dq.q;
        log.angle_el = (int32_t)theta;
        log.velocity = 0;                          /* motion task publishes it */
        log.duty_a = duty.a; log.duty_b = duty.b; log.duty_c = duty.c;
        log.v_bus = a.v_bus_mv;
        log.period_ticks = (int32_t)delta;
        log.fault_word = (int32_t)g_fault_word;
        log.isr_ticks = (int32_t)(hal_time_now() - t_entry);
        scope_log_write(&log);
    }
}