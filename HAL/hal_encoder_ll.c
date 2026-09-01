/* =============================================================================
 * hal_encoder_ll.c  --  SPI and DMA registers for the encoder chain.
 *
 * THE ORDERING RULE THIS FILE EXISTS TO CENTRALISE
 *   Every stream manipulation in the acquisition path is here, so the
 *   clear-before-arm sequence can be verified in one place rather than
 *   trusted at each call site.
 *
 * WHY DIRECT REGISTER ACCESS AND NOT HAL_DMA_Start
 *   HAL_DMA_Start validates parameters, checks handle state, and can return
 *   HAL_BUSY. Inside the FOC ISR a call that can decline has no meaningful
 *   failure handling — there is no second chance before the next period. The
 *   initialisation functions further down DO use HAL, because they run once
 *   with no deadline and the checking is worth having there.
 * ============================================================================= */

#include "hal_encoder_ll.h"
#include "hal.h"
#include "hal_sections.h"
#include "spi.h"
#include "dma.h"
#include "gpio.h"
#include "tim.h"

/* Motor on DMA1, load on DMA2 — arch section 3.3 asks for independent
 * hardware, so a stall on one controller cannot delay the other encoder. */
static DMA_Stream_TypeDef *const RX_STREAM[ENC_COUNT] = { DMA1_Stream0, DMA2_Stream0 };
static DMA_Stream_TypeDef *const TX_STREAM[ENC_COUNT] = { DMA1_Stream1, DMA2_Stream1 };
static SPI_TypeDef        *const SPI_DEV[ENC_COUNT]   = { SPI3, SPI1 };
extern DMA_HandleTypeDef hdma_tim4_ch2;
extern DMA_HandleTypeDef hdma_tim4_ch3;
/* Transfer-complete flag position differs per stream: LISR/HISR each cover
 * four streams with six bits apiece. Stream 0 puts TCIF at bit 5 of LISR. */
#define TCIF_STREAM0   (1u << 5)

ITCM_FUNC bool hal_dma_tc_flag(uint32_t i)
{
    DMA_TypeDef *dma = (i == 0u) ? DMA1 : DMA2;
    return (dma->LISR & TCIF_STREAM0) != 0u;
}

ITCM_FUNC void hal_dma_tc_clear(uint32_t i)
{
    DMA_TypeDef *dma = (i == 0u) ? DMA1 : DMA2;
    dma->LIFCR = TCIF_STREAM0;
}

ITCM_FUNC void hal_dma_rearm(uint32_t i, uint8_t *rx, const uint8_t *tx,
                             uint32_t n)
{
    DMA_Stream_TypeDef *r = RX_STREAM[i];
    DMA_Stream_TypeDef *t = TX_STREAM[i];

    /* Disable before touching NDTR or the addresses — the controller ignores
     * writes to those while EN is set, silently, so skipping this produces a
     * stream that keeps using last period's configuration.
     *
     * The readback loop is bounded rather than a spin: EN clears once any
     * in-flight beat completes, which is a handful of cycles, but P5 forbids
     * an unbounded wait in an ISR. Exhausting the count means the DMA is
     * wedged, and the caller finds out through the TC flag never setting. */
    r->CR &= ~DMA_SxCR_EN;
    t->CR &= ~DMA_SxCR_EN;
    for (uint32_t guard = 0u; guard < 64u; guard++) {
        if (((r->CR | t->CR) & DMA_SxCR_EN) == 0u) { break; }
    }

    r->NDTR = n;
    r->M0AR = (uint32_t)rx;
    t->NDTR = n;
    t->M0AR = (uint32_t)tx;

    r->CR |= DMA_SxCR_EN;
    t->CR |= DMA_SxCR_EN;
}

/* -----------------------------------------------------------------------------
 * PDVALIDx reset — opcode 0x02, address 0xF1.
 *
 * WHY IT MATTERS: without it a lost GETSENS pulse leaves the previous frame's
 * data in the iC-MBE's RAM with EOT still set from that earlier frame. The
 * next read returns data that is readable, plausible and stale — exactly the
 * silent staleness section 7 exists to catch, and the status byte would not
 * reveal it.
 *
 * Fire and forget: 43 us before the next GETSENS and no deadline, so this
 * simply queues the transfer and returns.
 * -------------------------------------------------------------------------- */
static DMA_BUF uint8_t g_pdvalid_tx[ENC_COUNT][3] = {
    { 0x02u, 0xF1u, 0x00u },
    { 0x02u, 0xF1u, 0x00u },
};

ITCM_FUNC void hal_enc_pdvalid_reset_start(void)
{
    for (uint32_t i = 0u; i < ENC_COUNT; i++) {
        SPI_DEV[i]->CR2 = 3u;
        /* Deliberately not waiting for completion, and deliberately not
         * checking the result. A failed PDVALID reset shows up on the NEXT
         * period as a stale frame, which the status byte and the TC flag
         * both catch — so there is nothing this path could usefully do about
         * it that is not already covered. */
    }
}

/* =============================================================================
 * Initialisation.  Runs once, before the carrier, with no deadline — so HAL
 * driver calls are the right tool here.
 * ========================================================================== */

bool hal_enc_nerr_released(uint32_t i)
{
    return (i == 0u)
        ? (HAL_GPIO_ReadPin(MOTOR_ENC_ERR_GPIO_Port, MOTOR_ENC_ERR_Pin) == GPIO_PIN_SET)
        : (HAL_GPIO_ReadPin(POS_ENC_ERR_GPIO_Port,   POS_ENC_ERR_Pin)   == GPIO_PIN_SET);
}

static bool spi_xfer(uint32_t i, const uint8_t *tx, uint8_t *rx, uint16_t n)
{
    SPI_HandleTypeDef *h = (i == 0u) ? &hspi3 : &hspi1;
    return HAL_SPI_TransmitReceive(h, (uint8_t *)tx, rx, n, 10u) == HAL_OK;
}

bool hal_enc_write_reg(uint32_t i, uint8_t addr, uint8_t val)
{
    /* Opcode 0x02: write one register. Byte-by-byte, because the iC-MBE does
     * not support burst writes to its registers. */
    const uint8_t tx[3] = { 0x02u, addr, val };
    uint8_t rx[3];
    return spi_xfer(i, tx, rx, 3u);
}

bool hal_enc_run_init(uint32_t i)
{
    /* INIT measures the line delay against the connected iC-MU200 and stores
     * the result in process-data RAM. This is the term that could not be
     * computed — it depends on cable length, and the chip measures it for
     * you (addendum section 5.1, term 3). */
    const uint8_t tx[2] = { 0x02u, 0xF0u };
    uint8_t rx[2];
    if (!spi_xfer(i, tx, rx, 2u)) {
        return false;
    }
    HAL_Delay(2u);          /* init sequence; no deadline here */
    return true;
}

uint32_t hal_enc_read_line_delay(uint32_t i)
{
    /* Stored in PDATA1(7:0) in quarter-BiSS-clock units:
     *     t = (1 + PDATA1) / (4 * f_MA)
     * At f_MA = 10 MHz one unit is 25 ns, which is 6 ticks of the 240 MHz
     * timebase. Converting here means the caller gets the same unit as every
     * other timing value in the system (P3). */
    const uint8_t tx[3] = { 0x09u, 0x00u, 0x00u };
    uint8_t rx[3];
    if (!spi_xfer(i, tx, rx, 3u)) {
        return 0u;
    }
    return ((uint32_t)rx[2] + 1u) * (HAL_TICK_HZ / 40000000u);
}

bool hal_enc_arm_dma(uint32_t n, uint8_t rx[ENC_COUNT][n], uint8_t tx[ENC_COUNT][n],
                     uint32_t cstart[ENC_COUNT])
{
    for (uint32_t i = 0u; i < ENC_COUNT; i++) {
        SPI_TypeDef *spi = SPI_DEV[i];

        spi->CR2 = n;                       /* TSIZE */
        spi->CFG1 |= SPI_CFG1_RXDMAEN | SPI_CFG1_TXDMAEN;

        hal_dma_tc_clear(i);
        hal_dma_rearm(i, rx[i], tx[i], n);

        spi->CR1 |= SPI_CR1_SPE;

        /* The CSTART stream: ONE word into SPI->CR1, triggered by TIM4's
         * compare DMA request.
         *
         * A DMA request moves exactly one item, so it cannot push a whole
         * frame — but it can set the bit that launches one already fully
         * armed. That is what makes the acquisition CPU-free. */
        DMA_HandleTypeDef *cs = (i == 0u) ? &hdma_tim4_ch2 : &hdma_tim4_ch3;
        if (HAL_DMA_Start(cs, (uint32_t)&cstart[i],
                          (uint32_t)&spi->CR1, 1u) != HAL_OK) {
            return false;
        }
    }
    return true;
}
