#!/usr/bin/env python3
"""
sysid_plan.py -- generate config/sysid_params.h: a coherent frequency list
for the stage-4 two-inertia system identification sweep.

IMPLEMENTATION_SPEC.md sections 4 and 10. "Coherent" means every excitation
frequency is an exact integer number of cycles within the capture window, so
foc/correlate.c's online correlation and this tool's offline counterpart
(Tools/sysid_fit.py) see no spectral leakage between bins:

    f_k = k * (sample_rate_hz / capture_periods),  k a positive integer

Usage:
    sysid_plan.py --sample-rate 16000 --capture-periods 4096 \
                   --f-min 1 --f-max 500 --n-freqs 20 \
                   --out config/sysid_params.h

Picks n_freqs bins geometrically spaced between f_min and f_max (round each
to the nearest coherent bin), which gives even coverage of a plant expected
to show both a rigid-body region and a resonance without needing to know the
resonance frequency in advance.

NOT YET IMPLEMENTED: capture length and frequency range are a property of
the physical joint (arch section 10, item 2: MPC per encoder blocks the
over-speed trip and cross-check threshold this would need to stay under).
This is a scaffold until a commissioned unit exists to characterise.
"""

import sys


def coherent_frequencies(sample_rate_hz: float, capture_periods: int,
                         f_min_hz: float, f_max_hz: float, n_freqs: int):
    """Pick n_freqs coherent bins, geometrically spaced, within [f_min, f_max].

    @param sample_rate_hz   Excitation/measurement sample rate, Hz.
    @param capture_periods  Capture window length, in samples.
    @param f_min_hz         Lowest frequency to consider, Hz.
    @param f_max_hz         Highest frequency to consider, Hz.
    @param n_freqs          Number of bins to select.
    @return                 Sorted list of unique coherent frequencies, Hz.
    """
    bin_hz = sample_rate_hz / capture_periods
    if f_min_hz < bin_hz:
        raise ValueError(
            f"f_min_hz ({f_min_hz}) is below the frequency resolution "
            f"({bin_hz} Hz) of a {capture_periods}-sample window at "
            f"{sample_rate_hz} Hz -- lengthen the capture window"
        )
    ratio = (f_max_hz / f_min_hz) ** (1.0 / max(n_freqs - 1, 1))
    freqs = set()
    f = f_min_hz
    for _ in range(n_freqs):
        k = max(1, round(f / bin_hz))
        freqs.add(k * bin_hz)
        f *= ratio
    return sorted(freqs)


def main() -> int:
    """@return 2 always -- see module docstring; no commissioned unit to plan for yet."""
    print(__doc__, file=sys.stderr)
    print(
        "sysid_plan.py: no target frequency range / capture length agreed "
        "yet -- nothing to generate. See IMPLEMENTATION_SPEC.md section 4 "
        "(config/sysid_params.h) and open item 2 (MPC per encoder).",
        file=sys.stderr,
    )
    return 2


if __name__ == "__main__":
    sys.exit(main())
