/* =============================================================================
 * hal_encoder_ll.h  --  the register and DMA plumbing under hal_encoder.c.
 *
 * WHY THIS IS A SEPARATE FILE
 *   hal_encoder.c is about the iC-MBE PROTOCOL: the two-stage chain, opcode
 *   0x09, the status byte, the bank swap.  Everything here is about SPI and
 *   DMA REGISTERS.  Keeping them apart means the protocol reads as protocol,
 *   and every stream manipulation in the acquisition path sits in one file
 *   where the clear-before-arm ordering can be checked at a glance.
 *
 *   L1-internal.  Nothing outside hal/ may include it, same rule as
 *   hal_private.h.
 * ============================================================================= */
#ifndef HAL_ENCODER_LL_H
#define HAL_ENCODER_LL_H

#include <stdint.h>
#include <stdbool.h>

#define ENC_COUNT   2u          /* 0 = motor (SPI3), 1 = load (SPI1) */

/* -- DMA, per encoder ------------------------------------------------------ */

/** @brief Has the RX stream completed this period's frame?
 *
 *  Polled from the FOC ISR, never taken as an interrupt: Table 4 lists encoder
 *  transfers as zero interrupts per period. Three instructions.
 */
bool hal_dma_tc_flag(uint32_t i);

/** @brief Clear the transfer-complete flag.
 *
 *  MUST be called BEFORE re-arming. Arm-then-clear leaves a window in which a
 *  fast transfer sets the flag and it is immediately cleared, so a genuinely
 *  missing frame reads as present next period — a silently wrong angle.
 */
void hal_dma_tc_clear(uint32_t i);

/** @brief Reload NDTR and the memory addresses, and re-enable both streams. */
void hal_dma_rearm(uint32_t i, uint8_t *rx, const uint8_t *tx, uint32_t n);

/* -- iC-MBE control, initialisation only ----------------------------------- */

bool     hal_enc_nerr_released(uint32_t i);
bool     hal_enc_write_reg(uint32_t i, uint8_t addr, uint8_t val);
bool     hal_enc_run_init(uint32_t i);
uint32_t hal_enc_read_line_delay(uint32_t i);

/**
 * @brief Arm the RX, TX and CSTART streams. After this the chain is hardware.
 *
 * @param n   Frame length in bytes. Tied to the rx/tx row stride via @p n
 *            rather than a fixed dimension, because a hardcoded row width
 *            here that silently diverges from the caller's real frame size
 *            (hal_encoder.c's ENC_FRAME_BYTES) would miscompute every
 *            per-encoder buffer address past index 0.
 * @param rx  [ENC_COUNT][n] receive buffers, one row per encoder.
 * @param tx  [ENC_COUNT][n] transmit buffers, one row per encoder.
 */
bool hal_enc_arm_dma(uint32_t n, uint8_t rx[ENC_COUNT][n], uint8_t tx[ENC_COUNT][n],
                     uint32_t cstart[ENC_COUNT]);

/** @brief Fire the pre-armed PDVALIDx reset. Two register writes, no deadline. */
void hal_enc_pdvalid_reset_start(void);

#endif /* HAL_ENCODER_LL_H */
