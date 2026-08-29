/* =============================================================================
 * scope_log.h  --  the in-RAM oscilloscope.
 *
 * WHY THIS EXISTS AT ALL (arch section 16, risk R2)
 *   An encoder interface occupies the trace pins, so there is no SWO and no
 *   ETM on this board.  That removes every conventional way of watching a
 *   transient:
 *
 *     printf over UART   -- formatting one line costs longer than the 62.5 us
 *                           deadline, so the act of printing destroys the
 *                           timing being printed about.
 *     halting the core   -- the motor does not halt with it.  The bridge holds
 *                           the last duty written and the loop regulating it
 *                           is gone.
 *     live watch vars    -- the debugger polls at maybe 10 Hz; a current step
 *                           settles in a millisecond.
 *     an actual scope    -- cannot see i_q, an integrator, or the angle
 *                           advance term.  Those are not on any pin.
 *
 *   So this is not a convenience.  It is the only instrument that can see
 *   inside the control loop on this hardware, and arch section 10.1 records
 *   it as the mitigation for R2.
 *
 * THE PROPERTY THAT MATTERS: PRE-TRIGGER CAPTURE
 *   Logging tells you what happened after you decided to look.  This tells
 *   you what happened before you knew to.  The buffer runs circularly at all
 *   times and a fault freezes it, so every trip -- including the 3 a.m. one
 *   during a soak run with nobody watching -- comes with the half second
 *   leading up to it already captured.
 * ============================================================================= */
#ifndef SCOPE_LOG_H
#define SCOPE_LOG_H

#include <stdint.h>
#include <stdbool.h>

/* 8 channels x 8000 samples x 4 bytes = 250 KB in AXI SRAM.
 * 8000 samples at 16 kHz is exactly 0.5 s (arch section 10.1).
 *
 * Eight int32 is 32 bytes, which is exactly one Cortex-M7 cache line, so the
 * interleaved layout below costs one line fill and one write-back per period
 * rather than eight.  That is why channels are interleaved and not planar. */
#define SCOPE_CHANNELS   8u
#define SCOPE_DEPTH      8000u

/* -----------------------------------------------------------------------------
 * Everything the ISR can log.
 *
 * The ISR fills this as it goes.  That is close to free: it already holds
 * these values, and most are needed for the feedback publish at ISR exit
 * anyway.  Selection happens here rather than in the ISR so that changing
 * which channels are recorded needs no rebuild -- see scope_log_select().
 * -------------------------------------------------------------------------- */
typedef struct {
    int32_t i_a, i_b, i_c;          /* phase currents, after offset and sign  */
    int32_t i_alpha, i_beta;        /* Clarke                                 */
    int32_t i_d, i_q;               /* Park                                   */
    int32_t i_d_ref, i_q_ref;       /* what the motion layer asked for        */
    int32_t v_d, v_q;               /* PI output                              */
    int32_t angle_el;               /* after the advance term                 */
    int32_t velocity;               /* per-period estimate                    */
    int32_t duty_a, duty_b, duty_c; /* AFTER the clamp, so what really went   */
                                    /* to the gates, not what was asked for   */
    int32_t v_bus;
    int32_t isr_ticks;              /* entry-to-exit, for the section 15
                                     * duration histogram                     */
    int32_t period_ticks;           /* TIM5 interval; deviation is a free
                                     * jitter measurement (arch section 7 L1) */
    uint32_t fault_word;
} scope_frame_t;

#define SCOPE_SOURCE_COUNT  (sizeof(scope_frame_t) / sizeof(int32_t))

/** @brief Zero the buffer, arm it, default the channel selection. */
void scope_log_init(void);

/**
 * @brief Record one period.  Called from the FOC ISR, hot path.
 *
 * Eight indexed loads and eight stores, around 33 ns of a 62.5 us period --
 * under 0.05%, which is the "well under one percent of the period budget" in
 * arch section 10.1.
 *
 * Does nothing once frozen, so the fault path costs one branch.
 */
void scope_log_write(const scope_frame_t *f);

/**
 * @brief Stop recording and preserve what is in the buffer.
 *
 * Called from hal_safe_state() as step 6 of the arch section 5.5 sequence,
 * and callable from a debugger or the object dictionary for a manual capture.
 *
 * IDEMPOTENT BY DESIGN.  A single fault frequently produces several: an
 * overcurrent trips the watchdog, the safe state removes torque, the sudden
 * current collapse trips a tier-2 check, and the brake engaging trips
 * another.  If each one froze the buffer, the last would win and you would
 * capture the consequences instead of the cause.  Only the first freeze
 * takes effect.
 */
void scope_log_freeze(void);

/**
 * @brief Freeze after @p post_samples more periods.
 *
 * post_samples = 0 is exactly scope_log_freeze(): pure pre-trigger, the half
 * second before the event.  Non-zero centres the window on the event, which
 * is what you want when deliberately provoking something -- a current step at
 * stage 4, or a fault injection at stage 8.
 *
 * Capped at SCOPE_DEPTH, because asking for more post-trigger than the buffer
 * holds would overwrite the very event you triggered on.
 */
void scope_log_trigger(uint32_t post_samples);

/** @brief Re-arm after a capture has been read out.  Clears the frozen flag
 *         and restarts circular recording.  Does NOT zero the buffer. */
void scope_log_rearm(void);

/**
 * @brief Choose what a channel records.
 *
 * @param ch      0 .. SCOPE_CHANNELS-1
 * @param source  word index into scope_frame_t
 *
 * Runtime, so the channel set can be changed over CoE between captures
 * without a rebuild -- which matters because the interesting eight signals
 * during current-loop tuning are not the interesting eight during a
 * resonance hunt.
 */
bool scope_log_select(uint32_t ch, uint32_t source);

/**
 * @brief Decimate by @p n periods per stored sample.
 *
 * n = 1 gives the full 16 kHz over 0.5 s.  n = 16 gives 1 kHz over 8 s, which
 * is the right window for a velocity-loop transient or a thermal ramp -- both
 * far too slow to fit in half a second.
 */
void scope_log_set_decimation(uint32_t n);

/* -----------------------------------------------------------------------------
 * Readout support (app/scope_readout.c).
 * -------------------------------------------------------------------------- */
typedef struct {
    bool     frozen;
    uint32_t write_index;   /* where the next sample WOULD have gone; the
                             * oldest valid sample sits here when wrapped     */
    bool     wrapped;       /* false means only [0, write_index) are valid    */
    uint32_t decimation;
    uint8_t  source[SCOPE_CHANNELS];
} scope_status_t;

void scope_log_status(scope_status_t *out);

/**
 * @brief Read one sample row.  @p i = 0 is the OLDEST sample.
 *
 * Unwraps the circular buffer, so the caller never has to reason about where
 * the write pointer was. Returns false past the end of valid data.
 */
bool scope_log_read(uint32_t i, int32_t out[SCOPE_CHANNELS]);

#endif /* SCOPE_LOG_H */