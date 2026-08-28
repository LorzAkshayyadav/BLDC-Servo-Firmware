/* =============================================================================
 * filter_coeffs.h  --  GENERATED FILE.  Do not hand-edit.
 *
 * Produced by Tools/filter_design.py from the design spec recorded below in
 * FILTER_COEFFS_DESIGN_SPEC. Never hand-typed (IMPLEMENTATION_SPEC.md
 * section 4): a biquad coefficient set that looks plausible but was
 * transcribed with one digit wrong is exactly the kind of error that passes
 * review and fails only as a subtly wrong monitoring filter, which is what
 * FILTER_COEFFS_CHECKSUM exists to catch -- config/build_config.h or the
 * consuming .c file should verify it against a checksum computed the same
 * way by Tools/filter_design.py and fail the build on mismatch, rather than
 * silently trusting a file nothing regenerated after a design change.
 *
 * To regenerate:
 *   python Tools/filter_design.py --spec <design-spec> --out config/filter_coeffs.h
 *
 * Feeds foc/iir.h cascades for the filtered-overcurrent check, the thermal
 * accumulator, the torque cross-check and the scope display ONLY -- never
 * the current loop itself (arch section 6).
 * ============================================================================= */
#ifndef CONFIG_FILTER_COEFFS_H
#define CONFIG_FILTER_COEFFS_H

#include <stdint.h>

/* PLACEHOLDER -- Tools/filter_design.py does not exist yet, so no filter has
 * actually been designed. Every array below is empty until it does; do not
 * pass FILTER_OVERCURRENT_N_STAGES > 0 without the matching coefficients. */
#ifndef FILTER_COEFFS_DESIGN_SPEC
#define FILTER_COEFFS_DESIGN_SPEC "UNGENERATED -- run Tools/filter_design.py"
#define FILTER_COEFFS_CHECKSUM    0u
#warning "config/filter_coeffs.h is ungenerated -- run Tools/filter_design.py before enabling any monitoring-path filter"
#endif

#define FILTER_OVERCURRENT_N_STAGES   0u
#define FILTER_THERMAL_N_STAGES       0u
#define FILTER_TORQUE_XCHECK_N_STAGES 0u

/* Populated by the generator as:
 *   static const iir_biquad_coeffs_t FILTER_OVERCURRENT_COEFFS[] = { ... };
 * left undeclared here (rather than as an empty array) so a consumer that
 * forgets to check the N_STAGES == 0 guard gets a compile error instead of
 * silently filtering through zero stages. */

#endif /* CONFIG_FILTER_COEFFS_H */
