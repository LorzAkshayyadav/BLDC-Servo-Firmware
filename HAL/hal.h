/* =============================================================================
 * hal.h  --  L1 public surface.  The ONLY header L2 (torque/) and L3 (motion/)
 *            may include from the hardware side.
 *
 * THE RULE THAT MAKES THIS FILE WORK:
 *
 *   This header includes stdint.h, stdbool.h and config/ headers.  Nothing else.
 *   No stm32h7xx.h.  No HAL driver headers.  No register names.  No pin numbers.
 *
 *   That restriction is what lets torque/ and motion/ compile for a host PC
 *   against test/host/hal_stub.c.  If you ever need to add an ST type to this
 *   file, the verb is wrong -- redesign it so the ST type stays inside hal_*.c.
 *
 * UNITS AND CONVENTIONS (fixed point throughout, no float on the fast path):
 *
 *   angle      uint32_t phase accumulator.  Full scale 2^32 == 360 deg
 *              electrical.  Wraparound is natural integer overflow: no branch,
 *              no comparison, no rounding drift.  See arch section 8.1.
 *   current    int32_t milliamps, motor convention (positive = into the phase).
 *              hal_adc already applied the low-side shunt sign inversion and
 *              the INA241 gain/shunt scale.  Residual per-unit offset removal
 *              is L2's job (arch section 2) using config/motor_params.h.
 *   voltage    int32_t millivolts.
 *   duty       uint16_t, 0 .. HAL_DUTY_FULL.  Modulator output.  hal_pwm
 *              applies the low-side-sense duty clamp and writes the preloaded
 *              compare registers.
 *   time       uint32_t free-running ticks at HAL_TICK_HZ.  Differences are
 *              wrap-safe by unsigned subtraction for intervals < 2^32 ticks.
 *   torque     int32_t milli-newton-metres.
 * ========================================================================== */

#ifndef HAL_H
#define HAL_H

#include <stdint.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------
 * Compile-time facts about the timebase.  Derived from the frozen clock tree
 * (HSE 24 MHz -> PLL1P 480 MHz -> AHB 240 MHz -> timer kernel 240 MHz) and
 * from TIM1 ARR = 7500 in centre-aligned mode.
 * ------------------------------------------------------------------------- */
#define HAL_TICK_HZ            240000000u   /* TIM1/2/4/5 counter clock      */
#define HAL_TICKS_PER_US       240u
#define HAL_PWM_HZ             16000u
#define HAL_TICKS_PER_PERIOD   15000u       /* 2 * ARR, centre-aligned       */
#define HAL_DUTY_FULL          7500u        /* == ARR                        */

#define HAL_MOTION_DIVIDER     4u           /* 4 kHz                         */
#define HAL_SUPERVISOR_DIVIDER 16u          /* 1 kHz                         */

/* =========================================================================
 * ACQUISITION -- consumed by the FOC ISR
 * ========================================================================= */

/* The guaranteed-simultaneous pair.  Phase B on ADC1, phase C on ADC2, both
 * sampled at the same instant by the dual injected group.  Phase A comes from
 * the independent converter and is NOT simultaneous with these two -- it is a
 * plausibility check only (arch section 4.4). */
typedef struct {
    int32_t i_b_ma;      /* simultaneous pair                               */
    int32_t i_c_ma;      /* simultaneous pair                               */
    int32_t i_a_ma;      /* independent, sum-check only                      */
    int32_t v_bus_mv;
} hal_analog_t;

/**
 * @brief Read the simultaneous phase currents and bus voltage.
 *
 * Reads the four injected result registers. No waiting: by construction the
 * ISR cannot run before these exist (arch section 4.1).
 *
 * @param out Filled with the latest acquisition. Must not be NULL.
 */
void hal_adc_read(hal_analog_t *out);

/* -------------------------------------------------------------------------
 * Encoder samples.  One iC-MBE master chip per encoder, each holding a
 * CRC-checked BiSS frame in its own RAM bank.  status_ok folds the whole
 * status byte (EOT, nPDERR, nAGSERR, nDELAYERR, nSPIERR) into one verdict;
 * status_raw is kept for diagnostics and the fault word.
 * ------------------------------------------------------------------------- */
typedef struct {
    uint32_t angle;       /* electrical, phase-accumulator scale             */
    uint32_t position;    /* mechanical, full encoder resolution             */
    uint8_t  status_raw;
    bool     status_ok;
    bool     fresh;       /* DMA transfer-complete flag was set at entry     */
    uint32_t stamp;       /* HAL_TICK_HZ ticks at ISR entry                  */
} hal_enc_sample_t;

typedef enum { HAL_ENC_MOTOR = 0, HAL_ENC_LOAD = 1 } hal_enc_id_t;

/**
 * @brief Read the latest decoded frame from one BiSS encoder.
 *
 * Polls the DMA transfer-complete flag (never interrupt-driven, arch table 4),
 * evaluates the status byte, and decodes the frame. Sets fresh=false rather
 * than blocking if the frame did not arrive.
 *
 * @param id  Which encoder to read.
 * @param out Filled with the decoded sample. Must not be NULL.
 */
void hal_enc_read(hal_enc_id_t id, hal_enc_sample_t *out);

/**
 * @brief Re-arm both encoder acquisition chains for the next period.
 *
 * Called once at ISR exit. Clears SPI EOT on both buses, clears the DMA
 * transfer-complete flags, re-arms the normal-mode streams, deasserts and
 * reasserts the software chip select on the load encoder, and fires the
 * pre-armed PDVALIDx reset transaction.
 *
 * Order matters: clear-then-arm, never arm-then-clear, or a fast transfer can
 * set the flag just before it is cleared and a genuinely missing frame then
 * reads as present.
 */
void hal_enc_rearm(void);

/* -------------------------------------------------------------------------
 * Torque sensor.  RS-485, motion layer only, never the current loop.
 * Must tolerate a missing frame without disturbing the drive (arch section 8).
 * ------------------------------------------------------------------------- */
typedef struct {
    int32_t  torque_mnm;
    bool     valid;
    uint32_t stamp;
} hal_torque_sample_t;

/**
 * @brief Read the latest torque sensor frame, if one has arrived.
 *
 * RS-485, motion layer only, never the current loop. Must tolerate a missing
 * frame without disturbing the drive (arch section 8): on timeout, @p out
 * is returned with `valid = false` rather than blocking.
 *
 * @param out Filled with the latest sample (or `valid = false`). Must not
 *            be NULL.
 */
void hal_torque_read(hal_torque_sample_t *out);

/* =========================================================================
 * MODULATOR OUTPUT
 * ========================================================================= */

/**
 * @brief Enable the bridge. Fails closed.
 *
 * Arch section 5.5: re-enable only by explicit command through the drive
 * state machine, never self-clearing. This is that command.
 *
 * Refuses, returning false without changing anything, if either STO channel
 * is asserted or if a break flag is still latched. It returns a status rather
 * than asserting so the drive state machine can remain in "ready to switch
 * on" and publish the reason, instead of faulting on a condition that is
 * often just "the operator has not released the estop yet".
 *
 * Clears all three duties to zero before enabling, so a re-enable cannot
 * produce a torque step from whatever was left in the compare registers.
 *
 * @return true if the outputs are now live.
 */
bool hal_pwm_outputs_enable(void);

/**
 * @brief Disable the bridge without latching a fault.
 *
 * For a commanded stop, where the drive is leaving operation-enabled through
 * the normal state transitions. NOT a fault path: use hal_safe_state() for
 * anything that needs the cause recorded and the recovery gated.
 */
void hal_pwm_outputs_disable(void);

/**
 * @brief Write the three phase duty cycles for the next PWM period.
 *
 * Writes the three preloaded compare registers. Values take effect at the
 * next reload, so the duty computed this period applies to the next one.
 * Applies @ref hal_pwm_duty_clamp internally -- callers must not duplicate
 * the clamp arithmetic.
 *
 * @param a Phase A duty, 0 .. HAL_DUTY_FULL.
 * @param b Phase B duty, 0 .. HAL_DUTY_FULL.
 * @param c Phase C duty, 0 .. HAL_DUTY_FULL.
 */
void hal_pwm_set_duty(uint16_t a, uint16_t b, uint16_t c);

/**
 * @brief Duty ceiling imposed by the low-side current-sense conduction window.
 *
 * Derived from `HAL_DUTY_FULL - clamp >= dead_time + INA241 settling +
 * aperture offset`. The real number comes from scoping the amplifier output
 * at bring-up stage 3 and lives in config/board_limits.h; this accessor
 * exists so callers can report saturation without knowing where the value
 * came from.
 *
 * @return The maximum duty value any phase may be commanded to.
 */
uint16_t hal_pwm_duty_clamp(void);

/* =========================================================================
 * TIME  --  TIM5, 32-bit, free-running, never written by software
 * ========================================================================= */

/**
 * @brief Convert a tick count to microseconds.
 * @param t Duration in HAL_TICK_HZ ticks.
 * @return  @p t converted to microseconds (truncated).
 */
static inline uint32_t hal_ticks_to_us(uint32_t t) { return t / HAL_TICKS_PER_US; }

/**
 * @brief Read the free-running timebase.
 *
 * Single register read, no HAL call. Never written by software; see
 * ticks_since() in common/fixed.h for the wrap-safe interval convention
 * this relies on.
 *
 * @return Current tick count at HAL_TICK_HZ.
 */
uint32_t hal_time_now(void);

/**
 * @brief Carrier counter position at the instant of the call.
 *
 * Ramps 0 .. HAL_DUTY_FULL on the way up, then back down. Used only for the
 * late-entry check (arch section 7, L1); it cannot detect a skipped period
 * because it is periodic.
 *
 * @return Current carrier position, 0 .. HAL_DUTY_FULL.
 */
uint16_t hal_carrier_position(void);

/* =========================================================================
 * DEADLINE MONITOR  --  TIM6, one-pulse, priority 0
 * ========================================================================= */

/**
 * @brief Reload the independent deadline watchdog (TIM6).
 *
 * Must be called from the FOC ISR on every entry, first thing. If the ISR
 * stops, TIM6 expires instead and its priority-0 handler forces the safe
 * state.
 */
void hal_deadline_kick(void);

/* =========================================================================
 * SAFETY  --  every fault path from every tier converges here
 * ========================================================================= */

typedef enum {
    HAL_TRIP_NONE = 0,
    HAL_TRIP_AWD_PHASE_A,
    HAL_TRIP_AWD_PHASE_B,
    HAL_TRIP_AWD_PHASE_C,
    HAL_TRIP_BREAK_STO,
    HAL_TRIP_DEADLINE,
    HAL_TRIP_PERIOD_MISSED,
    HAL_TRIP_PERIOD_SHORT,
    HAL_TRIP_CURRENT_SUM,
    HAL_TRIP_OVERCURRENT_FILTERED,
    HAL_TRIP_BUS_OVERVOLT,
    HAL_TRIP_BUS_UNDERVOLT,
    HAL_TRIP_OVERSPEED,
    HAL_TRIP_ENC_MOTOR_STALE,
    HAL_TRIP_ENC_MOTOR_STATUS,
    HAL_TRIP_ENC_LOAD_STALE,
    HAL_TRIP_ENC_LOAD_STATUS,
    HAL_TRIP_THERMAL,
    HAL_TRIP_TRANSMISSION_CHECK,
    HAL_TRIP_TORQUE_CHECK,
    HAL_TRIP_FIELDBUS_WATCHDOG,
    HAL_TRIP_COUNT
} hal_trip_cause_t;

/**
 * @brief Force the drive into the safe state. Every fault path converges here.
 *
 * Executes the arch section 5.5 sequence, in order, from any context:
 *   1. software break event -> outputs latch to the same state a hardware
 *      break would produce, recovery requires the same explicit action
 *   2. both STO channels asserted -- the independent path
 *   3. all controller integrators reset (omit this and re-enable trips
 *      instantly, because the current loop restarts wound up) -- the
 *      implementation calls foc_reset_integrators() and
 *      motion_reset_integrators() directly (foc/foc.h, motion/motion.h);
 *      see CODE_LAYOUT.md's layer-rules table for why hal_safety.c, alone
 *      with isr_vectors.c, is allowed to call upward for this
 *   4. holding brake engaged
 *   5. cause latched to the status word and the battery-backed fault history
 *   6. scope buffer frozen (scope_log_freeze(), foc/scope_log.h), preserving
 *      the half second before the event
 *
 * Never self-clearing. Re-enable only through the drive state machine.
 *
 * @param cause The trip cause to latch and record.
 */
void hal_safe_state(hal_trip_cause_t cause);

/*
 * Brake is power-to-RELEASE (spring-applied). duty 0 == ENGAGED. There is
 * deliberately no symbol called BRAKE_HOLD in this API, because on this
 * brake topology a reader has a 50/50 chance of misreading it.
 */

/** @brief Drive the brake at full duty to pull the armature clear. */
void hal_brake_release_pull(void);

/** @brief Drive the brake at reduced duty to hold it clear once released. */
void hal_brake_release_hold(void);

/** @brief Drive the brake at duty 0; the spring applies it. */
void hal_brake_engage(void);

/**
 * @brief Worst-case time for the brake to mechanically seat once commanded.
 * @return Delay in microseconds: electrical decay plus mechanical engagement.
 */
uint32_t hal_brake_engage_delay_us(void);

/**
 * @brief Read the independent safe-torque-off channel 1 state.
 * @return true if channel 1 reports OK (not asserted/tripped).
 */
bool hal_sto_channel_1_ok(void);

/**
 * @brief Read the independent safe-torque-off channel 2 state.
 * @return true if channel 2 reports OK (not asserted/tripped).
 */
bool hal_sto_channel_2_ok(void);

/* =========================================================================
 * FIELDBUS  --  single owner, advanced only from the transfer-completion
 *               interrupt, servicing a request queue (arch section 9.2)
 * ========================================================================= */

/**
 * @brief Read the hardware-latched phase of the last SYNC0 edge, if any.
 *
 * Phase is relative to the carrier reference, in HAL_TICK_HZ ticks, bounded
 * to [0, HAL_TICKS_PER_PERIOD). Free of interrupt latency because the
 * capture happens at the signal edge (arch section 9.1).
 *
 * @param phase_ticks Out: latched phase, valid only when this returns true.
 * @return true if a new SYNC0 edge arrived since the last call, false if no
 *         edge has arrived (most periods) -- @p phase_ticks is left untouched.
 */
bool hal_fieldbus_phase(uint32_t *phase_ticks);

/**
 * @brief Nudge the carrier period to discipline it to the fieldbus clock.
 *
 * Low authority: a fraction of a percent. This corrects crystal drift, not
 * slewing the drive. Callers must freeze this (pass 0) on signal loss,
 * never integrate the correction away.
 *
 * @param delta_ticks Signed trim applied to the next carrier period.
 */
void hal_carrier_trim(int16_t delta_ticks);

/**
 * @brief Queue a mailbox message for transmission on the fieldbus.
 *
 * Advanced only from the transfer-completion interrupt, servicing a request
 * queue (arch section 9.2); safe to call from any context that accepts the
 * queuing latency.
 *
 * @param buf Message bytes to enqueue. Not retained after the call returns.
 * @param len Length of @p buf in bytes.
 */
void hal_fieldbus_enqueue_mailbox(const uint8_t *buf, uint16_t len);

/**
 * @brief Read the count of fieldbus mailbox queue overruns since startup.
 * @return Number of messages dropped because the queue was full.
 */
uint32_t hal_fieldbus_overrun_count(void);

/* =========================================================================
 * PERSISTENT STATE  --  backup SRAM, never zeroed at startup
 * ========================================================================= */

typedef struct {
    int32_t  multiturn_motor;
    int32_t  multiturn_load;
    uint32_t fault_history_head;
    uint32_t integrity;          /* if this fails: FAULT requiring
                                  * re-referencing, NOT a silent zero        */
} hal_persist_t;

/**
 * @brief Check whether backup SRAM holds a valid persistent-state record.
 *
 * If this fails, treat it as a fault requiring re-referencing, NOT a reason
 * to assume zero -- a confidently wrong joint position can otherwise be
 * reported while both encoders read perfectly.
 *
 * @return true if the integrity check passes.
 */
bool hal_persist_valid(void);

/**
 * @brief Read the persistent-state record from backup SRAM.
 * @param out Filled with the stored record. Must not be NULL.
 */
void hal_persist_read(hal_persist_t *out);

/**
 * @brief Write the persistent-state record to backup SRAM.
 * @param in Record to store. Must not be NULL.
 */
void hal_persist_write(const hal_persist_t *in);

/**
 * @brief Append one fault occurrence to the battery-backed fault history.
 * @param cause Trip cause being recorded.
 * @param stamp HAL_TICK_HZ timestamp of the event.
 */
void hal_persist_log_fault(hal_trip_cause_t cause, uint32_t stamp);

/* =========================================================================
 * TASK PENDING  --  the ISR pends the slower layers; it never calls them
 * ========================================================================= */

/** @brief Pend the motion task (L3) to run at its next opportunity. */
void hal_pend_motion(void);

/** @brief Pend the supervisor task (L4) to run at its next opportunity. */
void hal_pend_supervisor(void);

/* =========================================================================
 * INITIALISATION  --  called from app/, in this order, before anything turns
 * ========================================================================= */

/**
 * @brief Configure MPU regions and caches. First call in servo_main().
 *
 * Clocks are already configured by the time this runs -- CubeMX's
 * SystemClock_Config() (HSE -> PLL1P -> AHB -> timer kernel) runs before
 * servo_main() is ever called (see app/servo_main.c). This must still
 * precede anything that places a buffer: the non-cacheable MPU attribute on
 * RAM_D2 (hal/hal_sections.h: DMA_BUF) is what keeps DMA and the core
 * agreeing, and getting it wrong is silent -- stale reads under load, not a
 * fault at setup time.
 *
 * @return true on success.
 */
bool hal_init_memory(void);

/**
 * @brief Arm the protection tier (bring-up stage 0), before anything else.
 * @return true on success.
 */
bool hal_init_safety(void);

/**
 * @brief Calibrate the ADCs and arm the analog watchdogs.
 * @return true on success.
 */
bool hal_init_adc(void);

/**
 * @brief Bring up both BiSS encoder interfaces.
 *
 * Polls NERR for up to 20 ms, configures each iC-MBE, runs its INIT
 * sequence, and reads back the measured line delay.
 *
 * @return true on success.
 */
bool hal_init_encoders(void);

/**
 * @brief Start the timer sequencer: TIM2/4/5/6, then TIM1 last.
 * @return true on success.
 */
bool hal_init_sequencer(void);

/**
 * @brief Bring up the fieldbus interface.
 * @return true on success.
 */
bool hal_init_fieldbus(void);

/**
 * @brief Read back the line delay measured during encoder INIT.
 * @param id Which encoder to query.
 * @return Measured line delay, in quarter-BiSS-clock units.
 */
uint32_t hal_enc_measured_line_delay_ticks(hal_enc_id_t id);

#endif /* HAL_H */
