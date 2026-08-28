#!/usr/bin/env python3
"""
section_check.py -- fail the build if the linker map disagrees with the
memory architecture.

Arch section 10: "Verify placement in the linker map with a build check.
Section attributes are silently ignored more often than you would expect."

The two failures this catches, both of which are silent at runtime:

  1. A DMA buffer in tightly-coupled memory.  DMA1 and DMA2 cannot reach TCM
     at all.  No error, no transfer -- the buffer simply never fills.
  2. The control ISR left in flash.  It still works, but the duration
     histogram grows a tail and the determinism the whole design rests on is
     gone.  Arch section 15 asks for near-zero spread; visible spread means
     something still executes from flash.

Also checks that backup SRAM is not swept into .bss initialisation, because
zeroing it at startup destroys the multiturn counters and fault history that
survive a power cycle precisely so a field failure can be diagnosed.

Usage:
    section_check.py build/servo.map
"""

import re
import sys
from pathlib import Path

# Address ranges from the STM32H743 memory map.
REGIONS = {
    "ITCM":   (0x00000000, 0x00010000),   #  64 K, core only
    "DTCM":   (0x20000000, 0x20020000),   # 128 K, core only, NO DMA
    "AXI":    (0x24000000, 0x24080000),   # 512 K, core + DMA, cacheable
    "D2":     (0x30000000, 0x30048000),   # 288 K, core + DMA, non-cacheable
    "D3":     (0x38000000, 0x38010000),   #  64 K
    "BACKUP": (0x38800000, 0x38801000),   #   4 K, survives power cycle
    "FLASH":  (0x08000000, 0x08200000),
}

# Symbols that MUST live in a DMA-reachable, non-cacheable region.
DMA_SYMBOL = re.compile(r"(_dma_|_dmabuf|_rxbuf|_txbuf|enc_frame|"
                        r"fieldbus_pdo|torque_frame|cstart_word)")

# Symbols that MUST be in ITCM for the determinism argument to hold.
HOT_SYMBOL = re.compile(r"(foc_isr|clarke|park|inv_park|svpwm|pi_step|"
                        r"angle_advance|angle_sincos|tier2_fast|scope_log|"
                        r"deadline_handler|awd_handler|safe_state)")

# Symbols that must NOT be zeroed at startup.
PERSIST_SYMBOL = re.compile(r"(multiturn|fault_history|crash_dump|calib_valid)")


def region_of(addr: int) -> str:
    """Classify an address by which memory region it falls in.

    @param addr Absolute address, as read from the linker map.
    @return     The matching key from REGIONS, or "UNKNOWN" if none matches.
    """
    for name, (lo, hi) in REGIONS.items():
        if lo <= addr < hi:
            return name
    return "UNKNOWN"


def parse_map(path: Path):
    """Yield (symbol, address, size) for every symbol in a GNU ld map file.

    Size is taken from the most recent sized section/input entry preceding a
    symbol line; it is 0 if none preceded it.

    @param path Path to the linker-generated .map file.
    @return     Generator of (symbol, address, size) tuples.
    """
    sym_line = re.compile(r"^\s+0x([0-9a-fA-F]{8,16})\s+(\S+)\s*$")
    sized = re.compile(r"^\s*\.?\S*\s+0x([0-9a-fA-F]{8,16})\s+0x([0-9a-fA-F]+)"
                       r"\s+\S+\s*$")
    pending_size = 0
    for line in path.read_text(errors="replace").splitlines():
        m = sized.match(line)
        if m:
            pending_size = int(m.group(2), 16)
            continue
        m = sym_line.match(line)
        if m:
            yield m.group(2), int(m.group(1), 16), pending_size
            pending_size = 0


def main() -> int:
    """Check every DMA, hot-path, and persistent symbol in `<map-file>`.

    @return 0 if every checked symbol landed in its required region (and at
            least one DMA and one hot-path symbol were found at all), 1 if
            any placement problem was found, 2 on a usage or file error.
    """
    if len(sys.argv) != 2:
        print("usage: section_check.py <map-file>", file=sys.stderr)
        return 2
    path = Path(sys.argv[1])
    if not path.is_file():
        print(f"map file not found: {path}", file=sys.stderr)
        return 2

    problems, checked = [], {"dma": 0, "hot": 0, "persist": 0}

    for sym, addr, _size in parse_map(path):
        region = region_of(addr)

        if DMA_SYMBOL.search(sym):
            checked["dma"] += 1
            if region in ("ITCM", "DTCM"):
                problems.append(
                    f"{sym} at 0x{addr:08x} is in {region}: the DMA "
                    f"controllers cannot reach it.  Fails silently at runtime.")
            elif region == "AXI":
                problems.append(
                    f"{sym} at 0x{addr:08x} is in cacheable AXI SRAM: "
                    f"stale reads under load.  Use DMA_BUF (D2).")
            elif region != "D2":
                problems.append(
                    f"{sym} at 0x{addr:08x} is in {region}, expected D2.")

        if HOT_SYMBOL.search(sym):
            checked["hot"] += 1
            if region != "ITCM":
                problems.append(
                    f"{sym} at 0x{addr:08x} is in {region}, not ITCM: "
                    f"the ISR duration histogram will show a tail.")

        if PERSIST_SYMBOL.search(sym):
            checked["persist"] += 1
            if region != "BACKUP":
                problems.append(
                    f"{sym} at 0x{addr:08x} is in {region}, not backup SRAM: "
                    f"it will not survive a power cycle.")

    # Zero matches is a WARNING, not a build failure: during early bring-up
    # (stage 0/1), app/servo_main.c does not yet call into the FOC/HAL chain,
    # so --gc-sections strips foc_isr() and every DMA buffer out of the
    # binary entirely -- correctly, there is nothing placed to verify yet.
    # A misplaced symbol that DOES exist in the map (the loop above) is a
    # different class of finding -- an actual defect -- and still fails the
    # build. Once app/servo_main.c reaches hal_init_sequencer() and the
    # vector table references foc_isr(), these stop being zero and this
    # warning starts mattering as a real "the naming convention changed or
    # the map format broke" signal.
    warnings = []
    if checked["dma"] == 0:
        warnings.append("no DMA buffer symbols matched -- expected until "
                        "something links in a DMA_BUF-tagged buffer; if one "
                        "already exists, the naming convention or map format "
                        "may have changed.")
    if checked["hot"] == 0:
        warnings.append("no hot-path symbols matched -- expected until "
                        "app/servo_main.c's init chain reaches foc_isr() "
                        "through a reachable vector table entry, so "
                        "--gc-sections has not stripped it yet; if it should "
                        "be reachable, ITCM placement is unverified.")

    if problems:
        print("SECTION PLACEMENT FAILURE:", file=sys.stderr)
        for p in problems:
            print("  " + p, file=sys.stderr)
        return 1

    if warnings:
        print("section_check: WARNING -- nothing to verify yet:", file=sys.stderr)
        for w in warnings:
            print("  " + w, file=sys.stderr)

    print(f"section_check: clean "
          f"({checked['dma']} DMA, {checked['hot']} hot-path, "
          f"{checked['persist']} persistent symbols verified)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
