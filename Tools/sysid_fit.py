#!/usr/bin/env python3
"""
sysid_fit.py -- offline two-inertia plant fit from a captured sysID run.

IMPLEMENTATION_SPEC.md sections 4 and 10. Takes the per-frequency
correlation bins accumulated on-target by foc/correlate.c (drained over CoE
via app/scope_readout.c, per bin: accum_re, accum_im, n_samples, plus the
excitation frequency each bin belongs to from config/sysid_params.h) and
fits a two-inertia model:

    motor inertia J1, load inertia J2, shaft stiffness K, shaft damping C,
    plus whatever the excitation axis's own electrical/current-loop
    bandwidth contributes to the measured response.

The fitted result is UNIT-SPECIFIC (this joint's actual inertias and
compliance) and belongs in config/motor_params.h once accepted, not here --
this script only produces the fit; a human decides whether to write it back.

Usage:
    sysid_fit.py --bins capture.json --out fit_report.json

NOT YET IMPLEMENTED: no capture file format exists yet because
app/scope_readout.c and foc/correlate.c are both still stubs. This is a
scaffold for the fit step, not the capture pipeline.
"""

import sys


def two_inertia_transfer_function(j1, j2, k, c):
    """Return the analytic two-inertia velocity/torque transfer function.

    Standard form: G(s) = (J2 s^2 + C s + K) /
                          (s * (J1 J2 s^2 + (J1 + J2) C s + (J1 + J2) K))

    @param j1 Motor-side inertia.
    @param j2 Load-side inertia.
    @param k  Shaft stiffness.
    @param c  Shaft damping.
    @return   A callable taking angular frequency (rad/s) and returning the
              complex frequency response.
    """
    def g(omega):
        s = 1j * omega
        num = j2 * s**2 + c * s + k
        den = s * (j1 * j2 * s**2 + (j1 + j2) * c * s + (j1 + j2) * k)
        return num / den
    return g


def main() -> int:
    """@return 2 always -- see module docstring; no capture pipeline exists yet."""
    print(__doc__, file=sys.stderr)
    print(
        "sysid_fit.py: no capture file exists yet -- app/scope_readout.c and "
        "foc/correlate.c are both still stubs. Nothing to fit.",
        file=sys.stderr,
    )
    return 2


if __name__ == "__main__":
    sys.exit(main())
