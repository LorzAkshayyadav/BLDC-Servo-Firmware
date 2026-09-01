/* =============================================================================
 * supervisor.c  --  L4, 1 kHz, pended from the FOC ISR every 16th period.
 *
 * THE WATCHDOG REFRESH IS THE POINT OF THIS FILE
 *   The independent watchdog is refreshed HERE and nowhere else.  That single
 *   restriction turns a timer into an end-to-end liveness proof: this task
 *   only runs because the FOC ISR pended it, the FOC ISR only runs because
 *   the ADC completed a conversion, and the ADC only converts because TIM1 is
 *   triggering it.  So a refresh proves the entire chain (arch section 7, L4).
 *
 *   Refresh it from the main loop instead and the watchdog proves only that
 *   the main loop is alive -- which it would be with the control loop
 *   completely dead.
 *
 * WHY THE CROSS-CHECKS LIVE HERE AND NOT IN THE ISR
 *   Both compare quantities that move on a mechanical timescale.  Running
 *   them at 16 kHz would cost ISR budget to detect, milliseconds later,
 *   something that took seconds to develop.
 * ============================================================================= */

#include "supervisor.h"
#include "hal.h"
#include "contracts.h"
#include "motion.h"
#include "brake_seq.h"
#include "drive_profile.h"
#include "tier2_checks.h"
#include "motor_params.h"
#include "hal_sections.h"

#define SUPERVISOR_PERIOD_US   1000u

static DTCM_BSS struct {
    uint32_t thermal_accum;
    uint32_t last_stamp;
    uint32_t transmission_discrepancy;
    uint32_t torque_discrepancy;
} s;

void supervisor_init(void)
{
    s.thermal_accum = 0u;
    s.last_stamp    = 0u;
}

/* -----------------------------------------------------------------------------
 * Transmission integrity  (arch section 8.3).
 *
 * Motor angle divided by the gear ratio must agree with load angle, within
 * backlash plus compliance plus margin.
 *
 * WHAT IT ACTUALLY DETECTS, which is more than it sounds: a slipping
 * coupling, a stripped gear or failed flexspline, either encoder drifting or
 * losing its magnet, and swapped encoder wiring.  None of those are visible
 * to any single sensor -- they are only visible as DISAGREEMENT, which is why
 * having encoders on both ends of the gearbox is worth this check.
 *
 * The running discrepancy is published as process data because its slow drift
 * over months is a genuine predictive-maintenance signal for gear wear and
 * magnet ageing.
 * -------------------------------------------------------------------------- */
static hal_trip_cause_t check_transmission(const feedback_t *fb,
                                           const params_t *p)
{
    /* Motor position through the gear ratio, in load-side units. */
    const uint32_t motor_at_load =
        (uint32_t)(((uint64_t)fb->position_motor * GEAR_RATIO_DEN) /
                   GEAR_RATIO_NUM);

    const int32_t err = (int32_t)(motor_at_load - fb->position_load);
    const uint32_t mag = (err < 0) ? (uint32_t)(-err) : (uint32_t)err;

    s.transmission_discrepancy = mag;

    /* The threshold is built from ACCURACY, not resolution: absolute angular
     * accuracy is 2 LSB referenced to 12 bits of one sine period, which at 32
     * pole pairs is about 20 arcsec per encoder -- eight times coarser than
     * the 2.5 arcsec output resolution.  Sizing this from resolution produces
     * a check that trips on quantisation alone. */
    return (mag > p->transmission_tol) ? HAL_TRIP_TRANSMISSION_CHECK
                                       : HAL_TRIP_NONE;
}

/* -----------------------------------------------------------------------------
 * Torque path  (arch section 8.3).
 *
 * Torque predicted from measured current, motor constant, gear ratio and
 * efficiency must agree with the measured joint torque.  Detects a drifting
 * current sensor, an incorrect motor constant, and gearbox degradation.
 *
 * Tolerates a missing sensor frame by simply skipping -- section 8 requires
 * the torque path to tolerate a missing frame without disturbing the drive,
 * and a cross-check that faults on a late diagnostic reading would be worse
 * than no cross-check.
 * -------------------------------------------------------------------------- */
static hal_trip_cause_t check_torque_path(const feedback_t *fb,
                                          const params_t *p)
{
    hal_torque_sample_t sample;
    hal_torque_read(&sample);

    if (!sample.valid) {
        return HAL_TRIP_NONE;
    }

    const int32_t predicted_mnm =
        (int32_t)(((int64_t)fb->i_q_ma * MOTOR_KT_MNM_PER_A *
                   GEAR_RATIO_NUM) / (1000 * GEAR_RATIO_DEN));

    const int32_t err = predicted_mnm - sample.torque_mnm;
    const uint32_t mag = (err < 0) ? (uint32_t)(-err) : (uint32_t)err;

    s.torque_discrepancy = mag;

    return (mag > p->torque_path_tol_mnm) ? HAL_TRIP_TORQUE_CHECK
                                          : HAL_TRIP_NONE;
}

/* -----------------------------------------------------------------------------
 * I^2t accumulation.
 *
 * Inherently a low-pass, so it needs no filter of its own -- which is why
 * removing the IIR cascade costs this check nothing.  Rises with the square
 * of current and decays linearly, so a brief overload is absorbed and a
 * sustained one is not.
 * -------------------------------------------------------------------------- */
static void accumulate_thermal(const feedback_t *fb, const params_t *p)
{
    const int32_t i = (fb->i_q_ma < 0) ? -fb->i_q_ma : fb->i_q_ma;
    const uint32_t sq = (uint32_t)(((int64_t)i * i) >> 10);
    const uint32_t rated = (uint32_t)(((int64_t)p->i_limit_ma *
                                       p->i_limit_ma) >> 10);

    if (sq > rated) {
        s.thermal_accum += (sq - rated);
    } else if (s.thermal_accum > 0u) {
        const uint32_t decay = (rated - sq) >> 4;
        s.thermal_accum = (s.thermal_accum > decay) ? (s.thermal_accum - decay)
                                                    : 0u;
    }
}

/* -----------------------------------------------------------------------------
 * The task.
 * -------------------------------------------------------------------------- */
void supervisor_task(void)
{
    const params_t  *p  = &g_params.slot[CONTRACT_TAKE(g_params)];
    const feedback_t *fb = &g_feedback.slot[CONTRACT_TAKE(g_feedback)];

    hal_trip_cause_t trip;

    trip = check_transmission(fb, p);
    if (trip != HAL_TRIP_NONE) { hal_safe_state(trip); }

    trip = check_torque_path(fb, p);
    if (trip != HAL_TRIP_NONE) { hal_safe_state(trip); }

    accumulate_thermal(fb, p);

    trip = tier2_slow(motion_velocity_rpm(), s.thermal_accum, p);
    if (trip != HAL_TRIP_NONE) { hal_safe_state(trip); }

    brake_seq_poll(SUPERVISOR_PERIOD_US);
    drive_profile_poll();

    /* LAST, and only after everything above has had a chance to fault.
     *
     * Refreshing before the checks would mean a supervisor that faults every
     * cycle still keeps the watchdog happy -- which is exactly the condition
     * the watchdog should catch. */
    hal_watchdog_refresh();
}

uint32_t supervisor_transmission_discrepancy(void) { return s.transmission_discrepancy; }
uint32_t supervisor_torque_discrepancy(void)       { return s.torque_discrepancy; }
uint32_t supervisor_thermal_accum(void)            { return s.thermal_accum; }