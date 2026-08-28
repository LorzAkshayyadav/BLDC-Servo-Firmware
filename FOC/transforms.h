#ifndef TRANSFORMS_H
#define TRANSFORMS_H

#include <stdint.h>
#include "common/fixed.h"

/* Clarke (two-current), Park, inverse Park */

/* Reused for both currents (mA) and voltages (mV) -- like hal.h, the units
 * are carried by convention and the comment at the call site, not by the
 * type system. */
typedef struct { int32_t alpha; int32_t beta; } ab_t;
typedef struct { int32_t d; int32_t q; } dq_t;

/**
 * @brief Clarke transform from the two guaranteed-simultaneous currents.
 *
 * Two currents carry all the information there is for a three-wire motor;
 * the third is determined by the other two (arch section 4.4). Phase A sits
 * on the independent converter and is a plausibility check only -- it must
 * never be an input to this transform.
 *
 * @param i_b_ma Phase B current, mA.
 * @param i_c_ma Phase C current, mA.
 * @return       Stationary-frame alpha/beta pair.
 */
ab_t clarke_2ph(int32_t i_b_ma, int32_t i_c_ma);

/**
 * @brief Park transform: stationary frame to rotating d/q frame.
 *
 * Takes a pre-computed sincos_t rather than an angle so a forward/reverse
 * pair built from two separate lookups can never silently use different
 * angles (arch section 8.1).
 *
 * @param v  Stationary-frame alpha/beta pair.
 * @param sc Sin/cos of the electrical angle, from fixed_sincos()/angle_sincos().
 * @return   Rotating-frame d/q pair.
 */
dq_t park(ab_t v, const sincos_t *sc);

/**
 * @brief Inverse Park transform: rotating d/q frame to stationary frame.
 * @param v  Rotating-frame d/q pair (voltage, in the current loop's use).
 * @param sc Sin/cos of the electrical angle. Must be the SAME sincos_t
 *           instance used by the matching park() call this period.
 * @return   Stationary-frame alpha/beta pair, ready for svpwm().
 */
ab_t inv_park(dq_t v, const sincos_t *sc);

#endif /* TRANSFORMS_H */
