#ifndef TIER2_CHECKS_H
#define TIER2_CHECKS_H

#include <stdint.h>
#include "hal/hal.h"
#include "common/contracts.h"

/* sum, filtered overcurrent, bus, speed, thermal */

/**
 * @brief Every tier-2 check that must run before the control chain uses
 *        this period's acquisition (arch section 5.2): current sum, filtered
 *        overcurrent, bus over/undervoltage.
 *
 * Deliberately excludes over-speed, encoder status/staleness and missed-
 * period detection: encoder status is checked directly in foc_isr.c
 * immediately after hal_enc_read() (fatal this period, no filtering
 * applicable), and missed/short-period detection needs only hal_time_now()
 * and hal_carrier_position(), so both run before this is even called. See
 * tier2_slow() for the checks that need the angle/speed data computed later
 * in the same period.
 *
 * @param i_a_ma   Phase A current, mA (independent converter, sum-check only).
 * @param i_b_ma   Phase B current, mA (simultaneous pair).
 * @param i_c_ma   Phase C current, mA (simultaneous pair).
 * @param v_bus_mv Bus voltage, mV.
 * @param p        Current parameter set (CONTRACT_TAKE'd once by the caller).
 * @return         HAL_TRIP_NONE if every check passes, else the specific
 *                  cause -- there is no generic fault code.
 */
hal_trip_cause_t tier2_fast(int32_t i_a_ma, int32_t i_b_ma, int32_t i_c_ma,
                            int32_t v_bus_mv, const params_t *p);

/**
 * @brief Tier-2 checks that need the angle/speed pipeline: over-speed
 *        against the encoder track limit, and accumulated thermal loading.
 *
 * Called after angle.c's per-period speed estimate is available, later in
 * the ISR than tier2_fast().
 *
 * @param speed_rpm       Instantaneous speed from angle_speed_estimate().
 * @param thermal_accum   Running thermal accumulator, caller-owned state.
 * @param p               Current parameter set.
 * @return                HAL_TRIP_NONE, HAL_TRIP_OVERSPEED or HAL_TRIP_THERMAL.
 */
hal_trip_cause_t tier2_slow(int32_t speed_rpm, uint32_t thermal_accum,
                            const params_t *p);

#endif /* TIER2_CHECKS_H */
