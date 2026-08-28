/**
 * @file fixed.h
 * @brief Fixed-point types and arithmetic. No float anywhere on the control path.
 *
 * @details
 * WHY NO FLOAT: the M7 has an FPU, so float would work. Fixed point is chosen
 * for one specific property from arch section 8.1: the electrical angle
 * carried as a phase accumulator makes wraparound natural integer overflow --
 * free, with no comparison, no branch, and no rounding drift accumulating
 * over hours of running. Angle advance becomes an integer addition and the
 * trig table index is a shift.
 *
 * Mixing float and fixed on the same path reintroduces conversion cost at
 * every boundary, so the whole path is fixed.
 */
#ifndef COMMON_FIXED_H
#define COMMON_FIXED_H

#include <stdint.h>
#include <stdbool.h>

/* -- Scalar types ---------------------------------------------------------- */
typedef int16_t  q15_t;   /**< -1.0 .. +0.99997, step 2^-15 */
typedef int32_t  q31_t;   /**< -1.0 .. +0.9999999995 */

/**
 * @brief Electrical angle, phase-accumulator scale.
 *
 * FULL 32-BIT RANGE IS ONE ELECTRICAL REVOLUTION. That is the whole trick:
 * 0x00000000 is 0 degrees, 0xFFFFFFFF is just short of 360, and adding past
 * the top wraps to the bottom with no code at all.
 */
typedef uint32_t angle_t;

#define ANGLE_FROM_DEG(d)   ((angle_t)((d) * (4294967296.0 / 360.0)))
#define ANGLE_QUARTER       ((angle_t)0x40000000u)   /* 90 electrical degrees */

/* -- Trig table ------------------------------------------------------------ */
/* Around ten kilobytes in zero-wait-state data memory, accurate far beyond
 * encoder noise, and faster than the hardware accelerator this part lacks
 * would have been (arch section 10.1).  1024 entries, quarter wave not used:
 * a full table costs 2 KB more and removes a quadrant branch from the ISR. */
#define TRIG_TABLE_LOG2     10
#define TRIG_TABLE_SIZE     (1u << TRIG_TABLE_LOG2)
#define ANGLE_TO_TRIG_IDX(a) ((uint32_t)(a) >> (32 - TRIG_TABLE_LOG2))

typedef struct { q15_t sin; q15_t cos; } sincos_t;

/**
 * @brief Look up sine and cosine of an electrical angle in one call.
 *
 * Arch section 8.1: sample the angle once per period, apply the advance
 * once, and use the resulting pair for both the forward and the reverse
 * transform. One call makes that coherence structural rather than something
 * to remember.
 *
 * @param a Electrical angle, phase-accumulator scale.
 * @return  The sin/cos pair for @p a.
 */
sincos_t fixed_sincos(angle_t a);

/* -- Arithmetic ------------------------------------------------------------ */

/**
 * @brief Q15 x Q15 -> Q15 multiply, rounded.
 *
 * Kept as a static inline so the ISR pays a multiply and a shift, not a call.
 *
 * @param a First Q15 operand.
 * @param b Second Q15 operand.
 * @return  Rounded Q15 product.
 */
static inline q15_t q15_mul(q15_t a, q15_t b)
{
    return (q15_t)(((int32_t)a * (int32_t)b + 0x4000) >> 15);
}

/**
 * @brief Saturate a 32-bit value into the Q15 range.
 *
 * @param v Value to clamp.
 * @return  @p v clamped to [-32768, 32767].
 */
static inline q15_t q15_sat(int32_t v)
{
    if (v >  32767) return  32767;
    if (v < -32768) return -32768;
    return (q15_t)v;
}

/**
 * @brief Saturating add, used by the PI integrators.
 *
 * Reports whether it clipped so the caller can stop integrating --
 * anti-windup without a second branch.
 *
 * @param a       First Q31 operand.
 * @param b       Second Q31 operand.
 * @param clipped Out: set true if the sum saturated, false otherwise.
 * @return        @p a + @p b, saturated to the Q31 range.
 */
static inline q31_t q31_add_sat(q31_t a, q31_t b, bool *clipped)
{
    int64_t s = (int64_t)a + (int64_t)b;
    if (s >  INT32_MAX) { *clipped = true;  return INT32_MAX; }
    if (s <  INT32_MIN) { *clipped = true;  return INT32_MIN; }
    *clipped = false;
    return (q31_t)s;
}

/* -- Interval arithmetic --------------------------------------------------- */

/**
 * @brief Elapsed ticks between two free-running-clock reads.
 *
 * Correct across the 17.9 s wrap for any true interval under 2^32 ticks,
 * with no comparison and no branch, PROVIDED software never writes the
 * counter. That rule is what makes this function valid; see hal_time_now()
 * in hal.h.
 *
 * @param then Earlier tick reading.
 * @param now  Later tick reading.
 * @return     Elapsed ticks, via deliberate unsigned wraparound.
 */
static inline uint32_t ticks_since(uint32_t then, uint32_t now)
{
    return now - then;          /* deliberate unsigned wrap */
}

#endif /* COMMON_FIXED_H */
