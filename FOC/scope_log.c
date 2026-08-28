/* =============================================================================
 * scope_log.c  --  circular capture with pre-trigger freeze.
 *
 * LAYOUT: INTERLEAVED, NOT PLANAR
 *   g_buf[sample][channel], so one period writes eight consecutive int32 --
 *   32 bytes, exactly one Cortex-M7 cache line.  One line fill and one
 *   write-back per period.
 *
 *   The planar alternative, g_buf[channel][sample], would touch eight
 *   separate cache lines every period and evict useful control state to do
 *   it.  Readout becomes slightly more work in exchange, which is the right
 *   trade: readout runs in the main loop with no deadline, capture runs in
 *   the ISR with a hard one.
 *
 * WHERE IT LIVES
 *   AXI SRAM.  Not D2, which is reserved for DMA buffers and is the scarcer
 *   region; not DTCM, where 250 KB would not fit alongside the stack and
 *   control state.  Arch section 10 assigns the scope buffer to AXI
 *   explicitly.
 * ============================================================================= */

#include "scope_log.h"
#include "hal_sections.h"
#include <string.h>

/* 8000 x 8 x 4 = 250 KB. */
static AXI_DATA int32_t g_buf[SCOPE_DEPTH][SCOPE_CHANNELS];

/* Control state in DTCM: touched every period, and small. */
static DTCM_BSS struct {
    uint32_t write_index;
    uint32_t decimation;
    uint32_t decim_count;
    uint32_t post_remaining;
    bool     armed;
    bool     wrapped;
    bool     trigger_pending;
    uint8_t  source[SCOPE_CHANNELS];
} g;

/* -----------------------------------------------------------------------------
 * Default channel set: what you want during current-loop tuning at stage 4,
 * which is the first place this earns its keep.
 * -------------------------------------------------------------------------- */
void scope_log_init(void)
{
    memset(g_buf, 0, sizeof(g_buf));

    g.write_index     = 0u;
    g.decimation      = 1u;
    g.decim_count     = 0u;
    g.post_remaining  = 0u;
    g.wrapped         = false;
    g.trigger_pending = false;

    g.source[0] = (uint8_t)(offsetof(scope_frame_t, i_q)      / sizeof(int32_t));
    g.source[1] = (uint8_t)(offsetof(scope_frame_t, i_q_ref)  / sizeof(int32_t));
    g.source[2] = (uint8_t)(offsetof(scope_frame_t, i_d)      / sizeof(int32_t));
    g.source[3] = (uint8_t)(offsetof(scope_frame_t, i_d_ref)  / sizeof(int32_t));
    g.source[4] = (uint8_t)(offsetof(scope_frame_t, v_q)      / sizeof(int32_t));
    g.source[5] = (uint8_t)(offsetof(scope_frame_t, angle_el) / sizeof(int32_t));
    g.source[6] = (uint8_t)(offsetof(scope_frame_t, v_bus)    / sizeof(int32_t));
    g.source[7] = (uint8_t)(offsetof(scope_frame_t, isr_ticks)/ sizeof(int32_t));

    g.armed = true;      /* last, so a half-built state is never recorded to */
}

/* -----------------------------------------------------------------------------
 * The hot path.
 * -------------------------------------------------------------------------- */
ITCM_FUNC void scope_log_write(const scope_frame_t *f)
{
    if (!g.armed) {
        return;                      /* frozen: one branch, nothing else */
    }

    if (++g.decim_count < g.decimation) {
        return;
    }
    g.decim_count = 0u;

    /* Read the frame as a word array.  Every member is int32_t, which is why
     * the struct is declared that way even for signals that are naturally
     * q15 -- a uniform width removes a switch from this loop. */
    const int32_t *src = (const int32_t *)f;
    int32_t *row = g_buf[g.write_index];

    for (uint32_t ch = 0u; ch < SCOPE_CHANNELS; ch++) {
        row[ch] = src[g.source[ch]];
    }

    if (++g.write_index >= SCOPE_DEPTH) {
        g.write_index = 0u;
        g.wrapped     = true;
    }

    /* Post-trigger countdown.  Checked after the write so that a trigger with
     * post_samples = 0 still captures the period in which the fault was
     * detected, not just the periods before it. */
    if (g.trigger_pending) {
        if (g.post_remaining == 0u) {
            g.armed           = false;
            g.trigger_pending = false;
        } else {
            g.post_remaining--;
        }
    }
}

/* -----------------------------------------------------------------------------
 * Freeze.  Called from hal_safe_state(), possibly several times.
 * -------------------------------------------------------------------------- */
void scope_log_freeze(void)
{
    /* IDEMPOTENCE IS THE WHOLE POINT.
     *
     * One real fault usually produces a cascade: the watchdog trips on
     * overcurrent, the safe state removes torque, the resulting current
     * collapse trips a tier-2 check, the brake engaging trips another.  If
     * every one of those froze the buffer, the last would win and the capture
     * would show the consequences rather than the cause.
     *
     * Testing g.armed rather than a separate "already frozen" flag also makes
     * this safe against a freeze arriving while a post-trigger countdown is
     * still running: the earlier trigger's window is abandoned in favour of
     * stopping now, which is correct -- a real fault outranks a deliberate
     * capture. */
    if (g.armed) {
        g.armed           = false;
        g.trigger_pending = false;
    }
}

void scope_log_trigger(uint32_t post_samples)
{
    if (!g.armed || g.trigger_pending) {
        return;                      /* already frozen, or already counting */
    }

    if (post_samples > SCOPE_DEPTH) {
        /* More post-trigger than the buffer holds would wrap past and
         * overwrite the event that caused the trigger. */
        post_samples = SCOPE_DEPTH;
    }

    g.post_remaining  = post_samples;
    g.trigger_pending = true;
}

void scope_log_rearm(void)
{
    g.write_index     = 0u;
    g.decim_count     = 0u;
    g.post_remaining  = 0u;
    g.wrapped         = false;
    g.trigger_pending = false;
    g.armed           = true;
}

bool scope_log_select(uint32_t ch, uint32_t source)
{
    if (ch >= SCOPE_CHANNELS || source >= SCOPE_SOURCE_COUNT) {
        return false;
    }
    /* Written while the ISR may be reading it.  A single byte store is
     * atomic on this core, so the worst case is one period recorded with a
     * mix of old and new channel assignments -- visible as one odd sample,
     * not as corruption. */
    g.source[ch] = (uint8_t)source;
    return true;
}

void scope_log_set_decimation(uint32_t n)
{
    g.decimation = (n == 0u) ? 1u : n;
}

/* -----------------------------------------------------------------------------
 * Readout.  Main loop, no deadline.
 * -------------------------------------------------------------------------- */
void scope_log_status(scope_status_t *out)
{
    out->frozen      = !g.armed;
    out->write_index = g.write_index;
    out->wrapped     = g.wrapped;
    out->decimation  = g.decimation;
    for (uint32_t i = 0u; i < SCOPE_CHANNELS; i++) {
        out->source[i] = g.source[i];
    }
}

bool scope_log_read(uint32_t i, int32_t out[SCOPE_CHANNELS])
{
    uint32_t valid = g.wrapped ? SCOPE_DEPTH : g.write_index;
    if (i >= valid) {
        return false;
    }

    /* Unwrap here so the caller never reasons about the write pointer.  When
     * wrapped, the oldest sample is the one the write pointer is about to
     * overwrite. */
    uint32_t phys = g.wrapped ? ((g.write_index + i) % SCOPE_DEPTH) : i;

    for (uint32_t ch = 0u; ch < SCOPE_CHANNELS; ch++) {
        out[ch] = g_buf[phys][ch];
    }
    return true;
}