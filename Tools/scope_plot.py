#!/usr/bin/env python3
"""
scope_plot.py -- unwrap and plot a capture from the in-RAM oscilloscope.

WHY THIS EXISTS
    An encoder interface occupies the trace pins (arch section 16, risk R2),
    so there is no SWO and no ETM on this board.  The in-RAM scope is the only
    instrument that can see inside the control loop, and this is how you look
    at what it recorded.

USAGE
    Stage 4 onward, over SWD:
        (gdb) source Tools/scope_capture.gdb
        (gdb) scope_capture
        $ python Tools/scope_plot.py

    Later, from the UART or CoE readout:
        $ python Tools/scope_plot.py --format csv --input capture.csv

TIME AXIS CONVENTION
    t = 0 is the LAST sample, i.e. the instant the buffer froze.  Everything
    is negative.  That is deliberate: the question this tool answers is
    almost always "what happened in the N milliseconds before the trip", and
    reading a fault backwards from zero is how you think about it.

    With a post-trigger capture (scope_log_trigger(n)), the triggering event
    sits at -n * dt rather than at 0.  Pass --trigger-at to mark it.

CHANNEL NAMES ARE PARSED FROM THE HEADER
    The source index stored in the metadata is a word offset into
    scope_frame_t.  Rather than duplicating that field order here -- where it
    would silently drift the first time someone inserts a field -- the struct
    is parsed out of foc/scope_log.h at run time.
"""

import argparse
import re
import struct
import sys
from pathlib import Path

CHANNELS = 8
DEPTH = 8000
TIM_TICK_HZ = 240_000_000
CARRIER_HZ = 16_000


# -----------------------------------------------------------------------------
# Scaling, keyed by field-name prefix.
#
# Everything in the buffer is a raw int32 -- uniform width keeps a type switch
# out of the ISR's copy loop -- so the meaning has to be reapplied here.
# -----------------------------------------------------------------------------
def scaler_for(name, args):
    """Return (fn, unit_label) for a scope_frame_t field name."""
    if name.startswith("i_"):
        # q15 fraction of full-scale current. Full scale comes from the shunt
        # value and the INA241 gain, so it is a board constant, not a guess.
        return (lambda v: v / 32768.0 * args.i_full_scale, "A")

    if name.startswith("v_") and name != "v_bus":
        return (lambda v: v / 32768.0 * args.v_full_scale, "V")

    if name == "v_bus":
        return (lambda v: v / 32768.0 * args.vbus_full_scale, "V")

    if name == "angle_el":
        # angle_t is a 32-bit phase accumulator: the full range is one
        # electrical revolution, which is the whole point of the
        # representation (arch section 8.1). Stored into an int32, so
        # reinterpret as unsigned before scaling.
        return (lambda v: (v & 0xFFFFFFFF) / 4294967296.0 * 360.0, "deg el")

    if name.startswith("duty_"):
        return (lambda v: v / 7500.0 * 100.0, "%")

    if name in ("isr_ticks", "period_ticks"):
        return (lambda v: v / TIM_TICK_HZ * 1e6, "us")

    if name == "velocity":
        return (lambda v: float(v), "raw")

    if name == "fault_word":
        return (lambda v: float(v), "code")

    return (lambda v: float(v), "raw")


def parse_frame_fields(header_path):
    """Extract scope_frame_t member names, in declaration order."""
    text = Path(header_path).read_text()
    m = re.search(r"typedef\s+struct\s*\{(.*?)\}\s*scope_frame_t\s*;", text, re.S)
    if not m:
        raise SystemExit(f"could not find scope_frame_t in {header_path}")

    fields = []
    for line in m.group(1).splitlines():
        line = re.sub(r"/\*.*?\*/", " ", line)          # strip inline comments
        line = re.sub(r"/\*.*$", " ", line)             # ...and unterminated
        line = line.strip()
        if not line.startswith("int32_t"):
            continue
        decl = line[len("int32_t"):].split(";")[0]
        for nm in decl.split(","):
            nm = nm.strip()
            if nm:
                fields.append(nm)
    return fields


def read_meta(path):
    meta = {"decimation": 1, "wrapped": 0, "write_index": 0, "armed": 1,
            "source": list(range(CHANNELS))}
    for line in Path(path).read_text().splitlines():
        parts = line.split()
        if not parts:
            continue
        if parts[0] == "source":
            meta["source"] = [int(x) for x in parts[1:1 + CHANNELS]]
        elif parts[0] in meta:
            meta[parts[0]] = int(parts[1])
    return meta


def read_binary(path, meta):
    """Read scope.bin and return rows in OLDEST-FIRST order."""
    raw = Path(path).read_bytes()
    expect = DEPTH * CHANNELS * 4
    if len(raw) != expect:
        print(f"warning: {path} is {len(raw)} bytes, expected {expect}",
              file=sys.stderr)

    flat = struct.unpack(f"<{len(raw)//4}i", raw)
    rows = [flat[i * CHANNELS:(i + 1) * CHANNELS]
            for i in range(len(flat) // CHANNELS)]

    # Unwrap. When the buffer has wrapped, the oldest sample is the one the
    # write pointer was about to overwrite; when it has not, only
    # [0, write_index) holds anything.
    w = meta["write_index"]
    if meta["wrapped"]:
        return rows[w:] + rows[:w]
    return rows[:w]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--input", default="scope.bin")
    ap.add_argument("--meta", default="scope_meta.txt")
    ap.add_argument("--header", default="foc/scope_log.h")
    ap.add_argument("--format", choices=["bin", "csv"], default="bin")
    ap.add_argument("--csv-out", help="write the decoded capture to a CSV file")
    ap.add_argument("--no-plot", action="store_true")
    ap.add_argument("--trigger-at", type=int, default=0,
                    help="post-trigger sample count; marks the event on the "
                         "time axis at -n*dt instead of at 0")
    # Board constants. These are placeholders until the shunt value, INA241
    # gain and bus divider are confirmed -- see config/board_limits.h.
    ap.add_argument("--i-full-scale", type=float, default=1.0,
                    help="amps at q15 full scale (shunt x INA241 gain)")
    ap.add_argument("--v-full-scale", type=float, default=1.0)
    ap.add_argument("--vbus-full-scale", type=float, default=1.0)
    args = ap.parse_args()

    fields = parse_frame_fields(args.header)

    if args.format == "bin":
        meta = read_meta(args.meta)
        rows = read_binary(args.input, meta)
    else:
        import csv
        meta = {"decimation": 1, "source": list(range(CHANNELS))}
        with open(args.input) as fh:
            rdr = csv.reader(fh)
            rows = [[int(v) for v in r] for r in rdr if r and not r[0].startswith("#")]

    if not rows:
        raise SystemExit("no samples: buffer was empty, or never armed")

    names, scales, units = [], [], []
    for ch in range(CHANNELS):
        idx = meta["source"][ch]
        nm = fields[idx] if idx < len(fields) else f"src{idx}"
        fn, unit = scaler_for(nm, args)
        names.append(nm)
        scales.append(fn)
        units.append(unit)

    dt = meta["decimation"] / CARRIER_HZ
    n = len(rows)
    t = [(i - (n - 1)) * dt for i in range(n)]      # t = 0 at the freeze

    print(f"{n} samples, decimation {meta['decimation']}, "
          f"{dt*1e6:.1f} us/sample, {n*dt*1000:.1f} ms span")
    for ch in range(CHANNELS):
        print(f"  ch{ch}: {names[ch]} [{units[ch]}]")

    if args.csv_out:
        with open(args.csv_out, "w") as fh:
            fh.write("t_s," + ",".join(f"{names[c]}_{units[c]}"
                                       for c in range(CHANNELS)) + "\n")
            for i, row in enumerate(rows):
                vals = ",".join(f"{scales[c](row[c]):.6g}" for c in range(CHANNELS))
                fh.write(f"{t[i]:.9f},{vals}\n")
        print(f"wrote {args.csv_out}")

    if args.no_plot:
        return

    try:
        import matplotlib.pyplot as plt
    except ImportError:
        raise SystemExit("matplotlib not installed; use --csv-out instead")

    fig, axes = plt.subplots(CHANNELS, 1, sharex=True, figsize=(11, 13))
    for ch, ax in enumerate(axes):
        ax.plot(t, [scales[ch](r[ch]) for r in rows], linewidth=0.8)
        ax.set_ylabel(f"{names[ch]}\n[{units[ch]}]", fontsize=8)
        ax.grid(True, alpha=0.3)
        if args.trigger_at:
            ax.axvline(-args.trigger_at * dt, color="r", linewidth=0.8, alpha=0.7)

    axes[-1].set_xlabel("time before freeze [s]")
    axes[0].set_title(f"scope capture  --  {n} samples @ {1/dt/1000:.1f} kHz")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
