/* =============================================================================
 * hal_encoder.c  --  two iC-MBE masters, each in front of an iC-MU200.
 *
 * THE CHAIN IS TWO STAGES, NOT ONE  (addendum section 1)
 *   The MCU's SPI transaction is NOT the encoder frame.  There are two
 *   transfers on two buses, and only the second involves the MCU:
 *
 *     STAGE 1  GETSENS pulse -> the iC-MBE runs a BiSS frame against the
 *              iC-MU200, checks the CRC, writes the result into its own
 *              process-data RAM, raises EOT and swaps its RAM bank.  The MCU
 *              takes no part.  ~3.4 us.
 *
 *     STAGE 2  the MCU reads that RAM.  A plain register read of data that is
 *              already resident and already validated.  ~2.7 us at 15 MHz.
 *
 *   Once those are seen as separate, several worries dissolve.  Read timing
 *   has NOTHING to do with when the position was measured -- that was fixed
 *   by the GETSENS edge.  Jitter in stage 2 costs no accuracy at all.
 *
 * WHY THE READ IS TIMER-DRIVEN AND NOT EOT-TRIGGERED
 *   EOT rising genuinely does mean the data is ready, so triggering on it
 *   would work.  It is still the wrong choice: the read start would then
 *   depend on interrupt latency and therefore on total software load, so
 *   adding work to another task in six months would move it.  A timer-driven
 *   read is immune to all of that, costs zero interrupts, and lands in a
 *   known quiet window after the bank swap -- which removes the tearing
 *   hazard instead of managing it (addendum section 3).
 *
 *   EOT is used as a CHECK instead, read from the status byte that arrives
 *   ahead of the data anyway.  The pin becomes optional instrumentation.
 * ============================================================================= */

#include "hal.h"
#include "hal_private.h"
#include "hal_sections.h"
#include "board_limits.h"
#include "motor_params.h"
#include "spi.h"                 /* CubeMX: hspi1, hspi3 */
#include "dma.h"
#include "gpio.h"
#include "hal_encoder_ll.h"
#include <string.h>

/* -----------------------------------------------------------------------------
 * The read-out transaction.
 *
 * Opcode 0x09, "Read Registers 0": reads from address 0x00 with NO address
 * byte, so the clock is uniform and the whole transfer is DMA-able.
 *
 * Opcode 0x03 must be avoided.  It specifies t_L2 = 90 ns of clock-high time
 * between the address byte and the first data byte, and a hardware SPI driven
 * by DMA produces a uniform clock and cannot insert that gap.  The datasheet
 * notes 0x03 exists for compatibility with an earlier part.
 *
 *   MOSI:  0x09  dummy dummy dummy dummy
 *   MISO:   --   STAT  PD0   PD1   PD2
 *
 * With NOCRC = 1 the iC-MBE still performs the CRC check and reports the
 * verdict in the status byte, but does not store the CRC bits -- so a 21-bit
 * process-data length is 3 bytes rather than 8.
 * -------------------------------------------------------------------------- */
#define ENC_OPCODE_READ0     0x09u
#define ENC_PD_BYTES         3u
#define ENC_FRAME_BYTES      (2u + ENC_PD_BYTES)   /* opcode + status + data */

/* Status byte bits (all active LOW except EOT).  Reading the position gets
 * this for free -- it arrives BEFORE the data, so the arch section 5.2
 * encoder checks cost no extra transaction, no extra pin and no interrupt. */
#define ST_EOT               (1u << 7)   /* 1 = frame terminated             */
#define ST_nERR              (1u << 0)
#define ST_nAGSERR           (1u << 1)   /* GETSENS held too long            */
#define ST_nDELAYERR         (1u << 2)
#define ST_nPDERR            (1u << 3)   /* process-data CRC verdict         */
#define ST_nSPIERR           (1u << 5)

/* DMA buffers.  D2 SRAM, non-cacheable, 32-byte aligned.
 *
 * Placed here and NOT in DTCM: the general-purpose DMA controllers cannot
 * reach tightly-coupled memory, and a buffer put there fails QUIETLY -- no
 * error, no transfer (arch section 10).  Tools/section_check.py verifies the
 * placement in the map, because a silently ignored section attribute produces
 * exactly this bug. */
static DMA_BUF uint8_t g_tx[2][ENC_FRAME_BYTES];
static DMA_BUF uint8_t g_rx[2][ENC_FRAME_BYTES];

/* The word each CSTART stream writes into SPI->CR1 once per period. */
static DMA_BUF uint32_t g_cstart[2];

/* Shadow copy in DTCM.
 *
 * THE REASON THIS EXISTS: with a single (not double) DMA buffer, a frame that
 * runs long is being written while the ISR reads it.  The transfer-complete
 * flag catches that, but the buffer contents are then half old and half new.
 * Copying the validated position into a shadow means a late frame falls back
 * on an INTACT previous value rather than a torn one. */
static DTCM_BSS struct {
    uint32_t angle;
    uint32_t position;
    uint32_t stamp;
    uint8_t  status_raw;
    bool     valid;
} g_shadow[2];

static DTCM_BSS uint32_t g_line_delay_ticks[2];

static SPI_TypeDef *const SPI_OF[2] = { SPI3, SPI1 };   /* motor, load */

/* =============================================================================
 * Read-out, from the FOC ISR.
 * ========================================================================== */

ITCM_FUNC void hal_enc_read(hal_enc_id_t id, hal_enc_sample_t *out)
{
    const uint32_t i = (uint32_t)id;

    /* Polled, not interrupt-driven.  Table 4 lists encoder transfers as zero
     * interrupts per period: the stagger in section 4 places the read so that
     * it has always finished by now, so this flag is expected to be set and
     * checking it costs three instructions.  Taking the interrupt instead
     * would add two vector entries per period for no diagnostic gain. */
    const bool complete = hal_dma_tc_flag(i);

    if (!complete) {
        /* The frame did not finish.  Fall back on the shadow, and let the
         * staleness of the stamp make it visible to the ISR's tier-2 check
         * rather than silently returning last period's angle as if fresh. */
        out->angle      = g_shadow[i].angle;
        out->position   = g_shadow[i].position;
        out->status_raw = g_shadow[i].status_raw;
        out->status_ok  = false;
        out->fresh      = false;
        out->stamp      = g_shadow[i].stamp;
        return;
    }

    const uint8_t *rx = g_rx[i];
    const uint8_t  st = rx[1];          /* status byte precedes the data */

    /* One byte answers every arch section 5.2 encoder question:
     *   EOT       did the BiSS frame terminate
     *   nPDERR    did the iC-MBE's own CRC check pass
     *   nAGSERR   was GETSENS still high when the frame ended
     *   nSPIERR   did the master understand this transaction
     *
     * nAGSERR is the one that catches a CCR1/CCR4 value grown too large, and
     * nDELAYERR catches a read scheduled before the frame completed -- i.e. a
     * CCR2/CCR3 set too early.  Both are configuration errors that would
     * otherwise present as intermittent bad angles. */
    const bool ok = (st & ST_EOT) &&
                    (st & ST_nPDERR) && (st & ST_nAGSERR) &&
                    (st & ST_nDELAYERR) && (st & ST_nSPIERR) && (st & ST_nERR);

    uint32_t pd = ((uint32_t)rx[2] << 16) | ((uint32_t)rx[3] << 8) | rx[4];

    if (ok) {
        /* Left-align the process data into the 32-bit accumulator scale.  The
         * top 21 bits carry position; everything below is zero.
         * Doing it here means L2 never has to know the encoder's resolution,
         * and a 19-bit and a 20-bit encoder look identical upward. */
        static uint8_t enc_res[2] = { ENC_MOTOR_RESOLUTION_BITS, ENC_LOAD_RESOLUTION_BITS };
        const uint32_t pos = pd << (32u - enc_res[i]);

        g_shadow[i].position   = pos;
        /* Electrical angle: multiply by pole pairs and add the commutation
         * offset.  The multiply overflowing IS the modulo -- one electrical
         * revolution per wrap, which is the whole reason the accumulator is
         * 32-bit (arch section 8.1). */
        g_shadow[i].angle      = (pos * MOTOR_POLE_PAIRS) + COMMUTATION_OFFSET;
        g_shadow[i].stamp      = hal_time_now();
        g_shadow[i].valid      = true;
    }
    g_shadow[i].status_raw = st;

    out->angle      = g_shadow[i].angle;
    out->position   = g_shadow[i].position;
    out->status_raw = st;
    out->status_ok  = ok;
    out->fresh      = true;
    out->stamp      = g_shadow[i].stamp;
}

/* =============================================================================
 * Re-arm, at ISR exit.
 * ========================================================================== */

ITCM_FUNC void hal_enc_rearm(void)
{
    for (uint32_t i = 0u; i < 2u; i++) {
        SPI_TypeDef *spi = SPI_OF[i];

        /* EOT must be cleared before the next CSTART takes effect.  Without
         * this the timer's DMA request writes CR1, the SPI ignores it, and no
         * transfer ever starts again -- after exactly one working period. */
        spi->IFCR = SPI_IFCR_EOTC | SPI_IFCR_TXTFC;

        /* CLEAR the transfer-complete flag BEFORE re-arming the stream.
         *
         * Arm-then-clear leaves a window in which a fast transfer sets the
         * flag and it is immediately cleared, so a genuinely missing frame
         * then reads as present next period.  That is a silently wrong angle,
         * which is the failure mode this whole file is arranged to avoid. */
        hal_dma_tc_clear(i);
        hal_dma_rearm(i, g_rx[i], g_tx[i], ENC_FRAME_BYTES);

        spi->CR2 = ENC_FRAME_BYTES;      /* TSIZE for the next transfer */
        spi->CR1 |= SPI_CR1_SPE;
    }

    /* Load-encoder chip select, software-driven.
     *
     * PC13 has no SPI1_NSS alternate function, so unlike SPI3 -- where PA15
     * carries hardware NSS -- this one is a GPIO write.  PC13 also sits in
     * the backup domain with a 2 MHz drive limit, which is ample for one
     * framing edge per period but rules out anything faster.
     *
     * PA4 is a real SPI1_NSS pin and is unused on this package, but the
     * pinout is frozen. */
    HAL_GPIO_WritePin(POS_ENC_CS_GPIO_Port, POS_ENC_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(POS_ENC_CS_GPIO_Port, POS_ENC_CS_Pin, GPIO_PIN_RESET);

    /* PDVALIDx reset, fire and forget.
     *
     * The iC-MBE recommends clearing these after reading, so that updated
     * sensor data is RECOGNISABLE as updated.  Skip it and a lost GETSENS
     * pulse leaves the previous frame's data in RAM with EOT still set from
     * that earlier frame -- readable, plausible, and stale.  Exactly the
     * silent staleness section 7 exists to prevent.
     *
     * Pre-armed, so this costs two register writes.  It has 43 us before the
     * next GETSENS and no deadline. */
    hal_enc_pdvalid_reset_start();
}

uint32_t hal_enc_measured_line_delay_ticks(hal_enc_id_t id)
{
    return g_line_delay_ticks[(uint32_t)id];
}

/* =============================================================================
 * Bring-up.
 * ========================================================================== */

static bool mbe_wait_ready(uint32_t i)
{
    /* Startup takes up to 20 ms, signalled on the error pin.  Poll it; do NOT
     * use a fixed delay, and treat a timeout as a hardware fault (addendum
     * section 7.1).  This runs once, before the carrier starts, so a bounded
     * poll here does not violate P5.
     *
     * The 20 ms is the iC-MBE loading its own configuration; nothing can be
     * written to it before that completes. */
    const uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 30u) {
        if (hal_enc_nerr_released(i)) {
            return true;
        }
    }
    return false;
}

bool hal_init_encoders(void)
{
    for (uint32_t i = 0u; i < 2u; i++) {
        if (!mbe_wait_ready(i)) {
            return false;
        }

        /* Register configuration.  The ordering below is not arbitrary: the
         * interface and channel setup must be in place before AGS is enabled,
         * or the first GETSENS arrives at an unconfigured master.
         *
         * 0xE5 bit 6 is the trap.  The register map lists it as "Must be 1"
         * and it is writable, so an init routine that only sets the fields it
         * cares about leaves it at 0 and control communication silently does
         * not work. */
        static const struct { uint8_t addr, val; } cfg_a[] = {
            { 0xE5u, 0x40u },                    /* MANDATORY bit 6 = 1      */
            { 0xF5u, 0x01u },                    /* IO_CFG: EOT + GETSENS    */
            { 0xEDu, 0x01u },                    /* CFGCH1 = BiSS C          */
            { 0xE6u, 0x03u },                    /* FREQS: f_MA = 10 MHz     */
            { 0xE7u, 0x02u },                    /* SINGLEBANK=0, NOCRC=1    */
            { 0xE8u, 0x7Du },                    /* CYCLETIME = AGSINFINITE  */
        };
        static const struct { uint8_t addr, val; } cfg_b[] = {
            { 0xEFu, 0x00u },                    /* ACTnSENS1 = sensor       */
            { 0xF4u, 0x01u },                    /* AGS=1: GETSENS triggering*/
        };

        for (uint32_t k = 0u; k < (sizeof(cfg_a) / sizeof(cfg_a[0])); k++) {
            if (!hal_enc_write_reg(i, cfg_a[k].addr, cfg_a[k].val)) {
                return false;
            }
        }

        /* PDLEN (0xC0) is per-encoder, not shared: motor and load may
         * legitimately run different MPC values (config/motor_params.h), so
         * it cannot sit in the cfg_a[]/cfg_b[] tables above. Written here,
         * between CYCLETIME and ACTnSENS1, to preserve the original ordering
         * -- interface/channel setup before AGS is enabled. */
        static const uint8_t pdlen[2] = { ENC_MOTOR_PDLEN, ENC_LOAD_PDLEN };
        if (!hal_enc_write_reg(i, 0xC0u, pdlen[i])) {
            return false;
        }

        for (uint32_t k = 0u; k < (sizeof(cfg_b) / sizeof(cfg_b[0])); k++) {
            if (!hal_enc_write_reg(i, cfg_b[k].addr, cfg_b[k].val)) {
                return false;
            }
        }

        /* INIT measures the line delay and stores it in the process-data RAM.
         *
         * This is the term that could not be computed: it depends on cable
         * length, and the chip measures it for you.  Read it back, convert
         * from quarter-BiSS-clock units, and record it -- it is what finally
         * fixes TIM4's CCR2/CCR3 rather than the estimate they hold now
         * (addendum section 5.1, term 3). */
        if (!hal_enc_run_init(i)) {
            return false;
        }
        g_line_delay_ticks[i] = hal_enc_read_line_delay(i);

        /* Pre-build the read-out frame.  Only the opcode differs from zero;
         * the rest are dummies whose only job is to clock the response out. */
        memset(g_tx[i], 0, ENC_FRAME_BYTES);
        g_tx[i][0] = ENC_OPCODE_READ0;

        /* One word, written by the TIM4 compare DMA request straight into
         * SPI->CR1, which is what starts the transfer with no CPU at all.
         * A DMA request moves ONE item, so it cannot push a whole frame --
         * but it can set the bit that launches one already fully armed. */
        g_cstart[i] = SPI_CR1_SPE | SPI_CR1_CSTART;
    }

    /* Arm the streams and the CSTART writes.  After this the acquisition
     * chain is entirely hardware: TIM4 pulses GETSENS, waits 6 us, raises two
     * DMA requests, and the frames land in D2 with the CPU asleep. */
    return hal_enc_arm_dma(ENC_FRAME_BYTES, g_rx, g_tx, g_cstart);
}