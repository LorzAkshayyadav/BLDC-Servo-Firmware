/* =============================================================================
 * contracts.c  --  storage for every datum contracts.h declares `extern`.
 *
 * IMPLEMENTATION_SPEC.md section 3: declaring these in the header without
 * defining them anywhere links fine until first use, then fails with an
 * unhelpful message. This file is that definition, plus the static
 * assertions that make current_ref_t's single-word atomicity a checked
 * invariant instead of something that has to be remembered.
 *
 * Placement: control state that the FOC ISR touches every period lives in
 * DTCM (zero wait state, no cache). Nothing here lives in D2 -- no DMA
 * controller reads or writes any of these directly; DMA lands in hal_*.c's
 * own D2 buffers first and gets copied in through hal_*_read().
 * ========================================================================== */

#include "contracts.h"
#include "hal/hal_sections.h"

/* -- 1. Feedback and status -------------------------------------------- */
DTCM_BSS feedback_contract_t g_feedback;

/* -- 2. Current reference ------------------------------------------------
 * A single aligned word: the atomic single-word write arch section 12
 * relies on is only real if this can never grow past 32 bits or lose
 * alignment. Checked here, once, rather than trusted at every call site. */
DTCM_BSS current_ref_t g_current_ref;
_Static_assert(sizeof(current_ref_t) == sizeof(uint32_t),
               "current_ref_t must be exactly one word: coherent d/q "
               "publish relies on a single atomic store (arch section 12)");
_Static_assert(_Alignof(current_ref_t) == _Alignof(uint32_t),
               "current_ref_t must be word-aligned: an unaligned 32-bit "
               "store is not atomic against interrupts on Cortex-M7");

/* -- 3. Velocity accumulator --------------------------------------------- */
DTCM_BSS velocity_contract_t g_velocity;

/* -- 4. Position command -------------------------------------------------- */
DTCM_BSS position_cmd_contract_t g_position_cmd;

/* -- 5. Gains and parameters ---------------------------------------------- */
DTCM_BSS params_contract_t g_params;

/* -- 6. Fault word --------------------------------------------------------
 * Atomic bit-set only (fault_set() in contracts.h); cleared solely by an
 * explicit reset command through the drive state machine, never here. */
DTCM_BSS volatile uint32_t g_fault_word;
