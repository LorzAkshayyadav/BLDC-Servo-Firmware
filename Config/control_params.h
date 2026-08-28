/* =============================================================================
 * control_params.h  --  design-generic control tuning.  NOT unit-specific.
 *
 * IMPLEMENTATION_SPEC.md section 4: arch section 13 keeps this separate from
 * motor_params.h for the same reason board_limits.h is separate from both --
 * gains belong to the design, per-unit measurements belong to the physical
 * joint, and board limits belong to the power stage. Confusing any two of
 * those makes a commissioned unit unreproducible.
 *
 * TRAP: do not put a gain here that was tuned on one physical joint. Once
 * stage 4 sysID runs on a specific unit, the RESULT lands in the runtime
 * params_t (common/contracts.h, written by app/param_service.c), not here --
 * these are only the compile-time defaults g_params is initialised with
 * before that write happens.
 * ============================================================================= */
#ifndef CONTROL_PARAMS_H
#define CONTROL_PARAMS_H

#include <stdint.h>

/* gains, limits, rates, angle advance, trips */

/* -- Late-entry classification (foc_isr.c step 2) -------------------------
 * CARRIER_ENTRY_EXPECTED/TOL live in config/board_limits.h instead, because
 * they come straight out of the trigger-placement constants that header
 * owns (CCR5_INJECTED_TRIGGER, SAMPLE_APERTURE_TICKS) -- foc_isr.c includes
 * both headers directly. config/ headers deliberately do not include each
 * other (layer table, arch section 2): each stays a self-contained set of
 * <stdint.h>-only macros, so nothing here doubles as a hidden dependency on
 * board_limits.h. LATE_ENTRY_LIMIT is a tuning threshold (how many
 * consecutive/leaky late events before it is a fault, not a jitter
 * measurement), which is why it lives here instead. */
#ifndef LATE_ENTRY_LIMIT
#define LATE_ENTRY_LIMIT        8u        /* PLACEHOLDER */
#warning "LATE_ENTRY_LIMIT is a placeholder -- set from the stage-1 jitter histogram"
#endif

/* -- Current loop -----------------------------------------------------------
 * Computed from measured L and R once stage 4 sysID runs (Tools/sysid_fit.py),
 * not guessed. These are the compile-time defaults only. */
#ifndef DEFAULT_KP_CURRENT_Q15
#define DEFAULT_KP_CURRENT_Q15  0
#define DEFAULT_KI_CURRENT_Q15  0
#warning "DEFAULT_K{P,I}_CURRENT_Q15 are placeholders -- run stage-4 sysID"
#endif

/* -- Velocity and position loops -------------------------------------------
 * Rates: velocity loop closes on the motor encoder at HAL_MOTION_DIVIDER
 * (4 kHz); position loop closes on the load encoder at the same rate
 * (arch section 8). */
#ifndef DEFAULT_KP_VELOCITY_Q15
#define DEFAULT_KP_VELOCITY_Q15 0
#define DEFAULT_KI_VELOCITY_Q15 0
#define DEFAULT_KP_POSITION_Q15 0
#warning "DEFAULT_K{P,I}_VELOCITY_Q15 / DEFAULT_KP_POSITION_Q15 are placeholders -- tune at stage 6"
#endif

/* -- Angle advance -----------------------------------------------------------
 * Compensates the delay between sampling the currents and the new duty
 * taking effect. Tune by sweeping at constant speed and load and taking the
 * coefficient that minimises current (arch section 8.1) -- not by
 * computation from CCR4/timing alone, because it also absorbs whatever the
 * inverter and sensing chain add beyond the nominal delay. */
#ifndef DEFAULT_ANGLE_ADVANCE_PER_RPM
#define DEFAULT_ANGLE_ADVANCE_PER_RPM 0u
#warning "DEFAULT_ANGLE_ADVANCE_PER_RPM is a placeholder -- sweep at stage 3/4"
#endif

/* -- Tier-2 thresholds that are NOT board limits ---------------------------
 * (board_limits.h owns AWD_LIMIT_*, which is the fast per-sample hardware
 * watchdog; these are the slower, filtered tier-2 checks in foc/tier2_checks.c) */
#ifndef DEFAULT_CURRENT_SUM_TOL_MA
#define DEFAULT_CURRENT_SUM_TOL_MA   0
#warning "DEFAULT_CURRENT_SUM_TOL_MA is a placeholder -- characterise by measurement"
#endif

#ifndef DEFAULT_V_BUS_MAX_MV
#define DEFAULT_V_BUS_MAX_MV    0
#define DEFAULT_V_BUS_MIN_MV    0
#warning "DEFAULT_V_BUS_{MAX,MIN}_MV are placeholders -- from the supply spec"
#endif

/* Margin below ENCODER_TRACK_FREQ_MAX_HZ (config/board_limits.h), converted
 * to rpm via the commissioned unit's pole pairs -- so the compile-time
 * default here is deliberately 0 (always trips) until motor_params.h has a
 * real MOTOR_POLE_PAIRS to convert with; app/param_service.c must compute
 * and publish the real value before this can be trusted at stage 4+. */
#ifndef DEFAULT_SPEED_LIMIT_RPM
#define DEFAULT_SPEED_LIMIT_RPM 0u
#warning "DEFAULT_SPEED_LIMIT_RPM is a placeholder -- derive from ENCODER_TRACK_FREQ_MAX_HZ and MOTOR_POLE_PAIRS"
#endif

/* -- Filter cut-offs, monitoring paths ONLY (foc/iir.h) --------------------
 * Never used inside the current loop: phase lag there costs bandwidth and
 * margin directly, and the PI integrator is already a low-pass. */
#ifndef DEFAULT_FILTER_OVERCURRENT_HZ
#define DEFAULT_FILTER_OVERCURRENT_HZ  0u
#define DEFAULT_FILTER_THERMAL_HZ      0u
#define DEFAULT_FILTER_TORQUE_XCHECK_HZ 0u
#warning "DEFAULT_FILTER_*_HZ are placeholders -- see Tools/filter_design.py / config/filter_coeffs.h"
#endif

/* -- Velocity accumulation depth --------------------------------------------
 * 4 periods gives 250 us and quarters the quantisation noise floor without
 * discarding any sample (arch section 8.1). This is the same count as
 * HAL_MOTION_DIVIDER (hal/hal.h) -- not #include'd here (config/ headers
 * stay <stdint.h>-only, arch section 2) -- so a file that includes both
 * (e.g. foc_isr.c) should _Static_assert the two are equal rather than
 * silently trusting them to be kept in sync by hand. */
#define VELOCITY_ACCUM_DEPTH    4u

#endif /* CONTROL_PARAMS_H */
