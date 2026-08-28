#!/usr/bin/env python3
"""
layer_check.py -- fail the build if device-specific symbols escape hal/.

Arch section 2: "L2 and L3 contain no register access and no pin numbers.
Everything device-specific lives in L1.  Enforce it with a build check that
fails if peripheral symbols appear outside the abstraction directory."

Arch section 13: "Layer discipline that exists only in a document decays
within months; the check that fails the build is what actually holds the
boundary."

Run from CMake as a pre-build step:
    add_custom_target(layer_check ALL
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/layer_check.py
                ${CMAKE_SOURCE_DIR}/src)
"""

import re
import sys
from pathlib import Path

# Directories that ARE allowed to know about the silicon.
ALLOWED_DIRS = {"hal", "platform"}

# Patterns that mean "this file knows what chip it is on".
FORBIDDEN = [
    # Peripheral instance names
    (re.compile(r"\b(TIM|ADC|SPI|USART|UART|DMA|GPIO|EXTI|RCC|IWDG|QUADSPI)"
                r"[0-9A-K]*\s*->"), "direct register access"),
    (re.compile(r"\b(TIM|ADC|SPI|DMA)[0-9]+_(IRQn|IRQHandler)\b"),
     "interrupt vector name"),
    # ST headers
    (re.compile(r'#include\s*[<"]stm32'), "ST device header"),
    (re.compile(r'#include\s*[<"]core_cm'), "CMSIS core header"),
    (re.compile(r'#include\s*[<"].*_hal'), "ST HAL driver header"),
    # HAL handle types and API
    (re.compile(r"\bH?AL_[A-Z][A-Za-z_]*\s*\("), "ST HAL function call"),
    (re.compile(r"\b(TIM|ADC|SPI|DMA|UART|USART)_HandleTypeDef\b"),
     "ST HAL handle type"),
    # Pin numbers
    (re.compile(r"\bGPIO_PIN_[0-9]+\b"), "pin number"),
    (re.compile(r"\bGPIO[A-K]\b"), "GPIO port name"),
    # Register bit macros
    (re.compile(r"\b(TIM|ADC|SPI|DMA|RCC)_[A-Z0-9]+_[A-Z0-9_]+\b"),
     "register bit macro"),
    # CMSIS intrinsics that imply core knowledge on a portable path
    (re.compile(r"\b__(disable|enable)_irq\b"), "global interrupt mask"),
    (re.compile(r"\bNVIC_[A-Za-z]+\s*\("), "direct NVIC call"),
]

# hal.h is the public surface.  L2/L3 include it; it must itself stay clean,
# which we check by holding it to the same rules despite living in hal/.
STRICT_IN_HAL = {"hal.h", "hal_sections.h"}

# CODE_LAYOUT.md's layer-rules table: the only two hal/ files allowed to call
# upward into L2/L3, and the only headers they are allowed to reach for when
# they do. Everything else in hal/ must stay downward-only.
UPWARD_CALL_ALLOWED_FILES = {"isr_vectors.c", "hal_safety.c"}
UPWARD_INCLUDE = re.compile(
    r'#include\s*["<](foc/foc\.h|foc/scope_log\.h|motion/motion\.h)[">]')

# hal_private.h is hal-internal cross-file plumbing (e.g. hal_safety.c
# reaching into hal_pwm.c's software-break trigger) -- it must never be
# visible outside hal/, the same way no ST header may be.
HAL_PRIVATE_INCLUDE = re.compile(r'#include\s*["<](hal/)?hal_private\.h[">]')


def check_file(path: Path, root: Path) -> list[str]:
    """Scan one source file for device-specific symbols outside hal/.

    Non-code lines (comments) are skipped so the rationale text in hal.h and
    foc_isr.c can name registers freely -- only code lines are held to the
    layering rule.

    @param path Absolute path of the file to scan.
    @param root Absolute path of the source root `path` is relative to.
    @return     One message per violation found; empty if the file is clean
                or is outside the strict set (see ALLOWED_DIRS/STRICT_IN_HAL).
    """
    rel = path.relative_to(root)
    top = rel.parts[0] if len(rel.parts) > 1 else ""
    in_allowed = top in ALLOWED_DIRS
    strict = (not in_allowed) or (path.name in STRICT_IN_HAL)
    if not strict:
        return []

    problems = []
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except OSError as exc:
        return [f"{rel}: cannot read ({exc})"]

    for lineno, line in enumerate(text.splitlines(), 1):
        stripped = line.strip()
        # Comments carry explanation, not code.  Skip them so the rationale
        # comments in hal.h and foc_isr.c can name registers freely.
        if stripped.startswith(("*", "//", "/*")):
            continue
        for pattern, why in FORBIDDEN:
            m = pattern.search(line)
            if m:
                problems.append(
                    f"{rel}:{lineno}: {why}: {m.group(0)!r}")
                break
    return problems


def check_upward_includes(path: Path, root: Path) -> list[str]:
    """Scan one file for the two upward-boundary violations layer_check owns
    alongside the device-specifics check: hal_private.h escaping hal/, and
    foc/motion headers being reached from a hal/ file other than the two
    CODE_LAYOUT.md names as deliberate exceptions.

    Comments are skipped for the same reason as check_file(): rationale text
    is allowed to name these files, only #include lines are held to the rule.

    @param path Absolute path of the file to scan.
    @param root Absolute path of the source root `path` is relative to.
    @return     One message per violation found; empty if the file is clean.
    """
    rel = path.relative_to(root)
    top = rel.parts[0] if len(rel.parts) > 1 else ""

    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except OSError as exc:
        return [f"{rel}: cannot read ({exc})"]

    problems = []
    for lineno, line in enumerate(text.splitlines(), 1):
        stripped = line.strip()
        if stripped.startswith(("*", "//", "/*")):
            continue

        if top != "hal" and HAL_PRIVATE_INCLUDE.search(line):
            problems.append(
                f"{rel}:{lineno}: hal_private.h is hal-internal -- may only "
                f"be included from hal/")

        if top == "hal" and path.name not in UPWARD_CALL_ALLOWED_FILES:
            m = UPWARD_INCLUDE.search(line)
            if m:
                problems.append(
                    f"{rel}:{lineno}: upward include of {m.group(1)!r} -- "
                    f"only {sorted(UPWARD_CALL_ALLOWED_FILES)} may call "
                    f"upward into L2/L3 (CODE_LAYOUT.md, layer rules)")

    return problems


def main() -> int:
    """Walk `<src-root>` and fail if any file violates the layer boundary.

    @return 0 if every scanned .c/.h file is clean, 1 if any violation was
            found, 2 on a usage or filesystem error.
    """
    if len(sys.argv) != 2:
        print("usage: layer_check.py <src-root>", file=sys.stderr)
        return 2
    root = Path(sys.argv[1]).resolve()
    if not root.is_dir():
        print(f"not a directory: {root}", file=sys.stderr)
        return 2

    problems = []
    upward_problems = []
    for path in sorted(root.rglob("*")):
        if path.suffix in {".c", ".h"} and path.is_file():
            problems.extend(check_file(path, root))
            upward_problems.extend(check_upward_includes(path, root))

    if problems:
        print("LAYER VIOLATION -- device specifics outside hal/:", file=sys.stderr)
        for p in problems:
            print("  " + p, file=sys.stderr)
        print(f"\n{len(problems)} violation(s).  L2 and L3 must compile for a "
              f"host with L1 stubbed out; these are the reasons they cannot.",
              file=sys.stderr)

    if upward_problems:
        print("UPWARD-CALL VIOLATION -- see CODE_LAYOUT.md layer rules:",
              file=sys.stderr)
        for p in upward_problems:
            print("  " + p, file=sys.stderr)
        print(f"\n{len(upward_problems)} violation(s).", file=sys.stderr)

    if problems or upward_problems:
        return 1

    print("layer_check: clean")
    return 0


if __name__ == "__main__":
    sys.exit(main())
