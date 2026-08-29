#include "transforms.h"

/* Clarke (two-current), Park, inverse Park */
ab_t clarke_2ph(int32_t i_b_ma, int32_t i_c_ma){
    /* The two-current Clarke transform is a 2x2 rotation matrix:
     *   [ alpha ]   [ 1      -0.5 ] [ i_b ]
     *   [ beta  ] = [ 0  sqrt(3)/2 ] [ i_c ]
     *
     * The sqrt(3)/2 is about 0.86602540378, which is exactly 0xDEBC in Q15.
     * The -0.5 is exactly -16384 in Q15.  The alpha output is scaled by
     * 1.5 to avoid the extra multiply by two that would be needed to
     * convert the beta output back to the original units.
     */
    ab_t out;
    out.alpha = (i_b_ma - (i_c_ma >> 1)) * 3 / 2;
    out.beta  = (int32_t)(((int64_t)i_c_ma * (int32_t)0xDEBC) >> 15);
    return out;
}
dq_t park(ab_t v, const sincos_t *sc)
{
    /* The Park transform is a 2x2 rotation matrix:
     *   [ d ]   [  cos(theta)  sin(theta) ] [ alpha ]
     *   [ q ] = [ -sin(theta)  cos(theta) ] [ beta  ]
     *
     * The sin/cos pair is pre-computed and passed in so that a forward/reverse
     * pair built from two separate lookups can never silently use different
     * angles (arch section 8.1).
     */
    dq_t out;
    /* Compute d-axis component in Q15 fixed-point arithmetic:
     * d = alpha*cos(theta) + beta*sin(theta)
     * Multiplications produce Q30 values (Q15*Q15), so shift right by 15
     * to return to Q15 before casting to int32_t.
     */
    out.d = (int32_t)(((int64_t)v.alpha * sc->cos + (int64_t)v.beta * sc->sin) >> 15);
    out.q = (int32_t)(((int64_t)v.beta * sc->cos - (int64_t)v.alpha * sc->sin) >> 15);
    return out;
}
ab_t inv_park(dq_t v, const sincos_t *sc){
    /* The inverse Park transform is a 2x2 rotation matrix:
     *   [ alpha ]   [  cos(theta) -sin(theta) ] [ d ]
     *   [ beta  ] = [  sin(theta)  cos(theta) ] [ q ]
     *
     * The sin/cos pair is pre-computed and passed in so that a forward/reverse
     * pair built from two separate lookups can never silently use different
     * angles (arch section 8.1).
     */
    ab_t out;
    /* Compute alpha-axis component in Q15 fixed-point arithmetic:
     * alpha = d*cos(theta) - q*sin(theta)
     * Multiplications produce Q30 values (Q15*Q15), so shift right by 15
     * to return to Q15 before casting to int32_t.
     */
    out.alpha = (int32_t)(((int64_t)v.d * sc->cos - (int64_t)v.q * sc->sin) >> 15);
    out.beta  = (int32_t)(((int64_t)v.d * sc->sin + (int64_t)v.q * sc->cos) >> 15);
    return out;
}