#ifndef CORRELATE_H
#define CORRELATE_H

#include <stdint.h>

/* Stage-4 sysID online correlation (IMPLEMENTATION_SPEC.md section 6/10).
 *
 * Accumulates, per excitation frequency, the correlation of the measured
 * response against sin/cos references at that frequency -- a running DFT
 * bin, computed incrementally instead of requiring the whole capture to be
 * held in memory at once. Tools/sysid_fit.py does the offline two-inertia
 * fit from the finished bins; this is only the accumulation. */

typedef struct {
    int64_t  accum_re;
    int64_t  accum_im;
    uint32_t n_samples;
} correlate_bin_t;

/** @brief Zero one frequency bin's accumulator at the start of a capture. */
void correlate_reset(correlate_bin_t *bin);

/**
 * @brief Accumulate one sample's contribution to one frequency bin.
 *
 * Called once per FOC ISR period, per active frequency bin, with the same
 * excitation sample foc/excitation.c injected this period correlated
 * against the measured response.
 *
 * @param bin        Frequency bin being accumulated into.
 * @param input      Excitation sample injected this period (from
 *                    excitation_step()).
 * @param output     Measured response this period (e.g. i_q_ma or a
 *                    velocity estimate, depending on what is being identified).
 * @param ref_cos_q15 cos(2*pi*f*t) for this bin's frequency, Q15.
 * @param ref_sin_q15 sin(2*pi*f*t) for this bin's frequency, Q15.
 */
void correlate_accumulate(correlate_bin_t *bin, int32_t input, int32_t output,
                          int32_t ref_cos_q15, int32_t ref_sin_q15);

#endif /* CORRELATE_H */
