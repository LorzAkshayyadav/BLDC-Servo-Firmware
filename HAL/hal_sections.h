/* =============================================================================
 * hal_sections.h  --  memory placement, expressed once so it cannot be got
 *                     wrong at individual call sites.
 *
 * TWO SILENT HAZARDS THIS FILE EXISTS TO REMOVE (arch section 10):
 *
 *   1. The general-purpose DMA controllers cannot reach tightly-coupled
 *      memory.  A buffer placed there fails quietly: no error, no transfer.
 *   2. Cacheable memory shared with DMA gives stale reads intermittently and
 *      under load, which is among the hardest classes of bug to diagnose.
 *
 * Solved by policy rather than by vigilance: a non-cacheable region for every
 * DMA buffer, and tightly-coupled memory for everything the core owns alone.
 * Set up once.  The alternative -- cache maintenance at every transfer site --
 * must be got right by every future call site forever.
 *
 * Section attributes are silently ignored more often than you would expect,
 * so tools/section_check.py fails the build if the linker map disagrees.
 * ========================================================================== */

#ifndef HAL_SECTIONS_H
#define HAL_SECTIONS_H

/* -----------------------------------------------------------------------------
 * Instruction TCM -- 64 K, core only, zero wait state, never cached.
 * Deterministic rather than merely fast: the point is that the ISR duration
 * histogram has near-zero spread (arch section 15).  Visible spread means
 * something still executes from flash.
 *
 * Contents: control ISR, transforms, controllers, modulator, motion loop,
 * fault handlers, fast abstraction paths, vector table.
 * --------------------------------------------------------------------------- */
#define ITCM_FUNC   __attribute__((section(".itcm_text"), used, noinline))

/* -----------------------------------------------------------------------------
 * Data TCM -- 128 K, core only, NO DMA REACH AT ALL.
 * Main stack, trigonometric table, control state, integrators, motion state,
 * decoded encoder values, parameters.
 *
 * Putting a DMA buffer here is the hazard above.  section_check.py greps for
 * anything named *_dma_* or *_buf in this section and fails.
 * --------------------------------------------------------------------------- */
#define DTCM_DATA   __attribute__((section(".dtcm_data")))
#define DTCM_BSS    __attribute__((section(".dtcm_bss")))
#define DTCM_CONST  __attribute__((section(".dtcm_rodata")))

/* -----------------------------------------------------------------------------
 * D2 SRAM -- 288 K, core + DMA, marked NON-CACHEABLE by the MPU.
 * Every DMA buffer: both encoders, fieldbus, torque sensor.  Same domain as
 * the DMA controllers, so no bridge delay.
 *
 * DMA_BUF also forces 32-byte alignment.  That is not required for
 * correctness once the region is non-cacheable, but it keeps every buffer on
 * a cache-line boundary so that if the MPU config is ever wrong the failure
 * is a clean one buffer at a time rather than neighbouring buffers
 * corrupting each other.
 * --------------------------------------------------------------------------- */
#define DMA_BUF     __attribute__((section(".d2_sram"), aligned(32)))

/* -----------------------------------------------------------------------------
 * AXI SRAM -- 512 K, core + DMA, cacheable.
 * Scope buffer, firmware staging, object dictionary, general allocation.
 * Anything not named in this file defaults here.
 * --------------------------------------------------------------------------- */
#define AXI_DATA    __attribute__((section(".axi_sram")))

/* -----------------------------------------------------------------------------
 * Backup SRAM -- 4 K, core only, survives a power cycle, NEVER zeroed at
 * startup.  Multiturn counters, fault history, crash dump, calibration
 * validity.
 *
 * The encoders are absolute within one revolution only, so multiturn position
 * must live somewhere that survives a power cycle, and flash wear rules flash
 * out.  If the integrity check fails, that is a fault requiring
 * re-referencing, not a reason to assume zero -- you would otherwise report a
 * confidently wrong joint position while both encoders read perfectly.
 * --------------------------------------------------------------------------- */
#define BACKUP_DATA __attribute__((section(".backup_sram")))

/* -----------------------------------------------------------------------------
 * Linker script must define, and section_check.py must confirm:
 *
 *   .itcm_text     > ITCMRAM     AT> FLASH   (copied by startup)
 *   .dtcm_data     > DTCMRAM     AT> FLASH
 *   .dtcm_bss      > DTCMRAM     (NOLOAD)
 *   .dtcm_rodata   > DTCMRAM     AT> FLASH
 *   .d2_sram       > RAM_D2      (NOLOAD)
 *   .axi_sram      > RAM_D1      (NOLOAD)
 *   .backup_sram   > RAM_D3_BKP  (NOLOAD)   <-- must NOT be in .bss init
 *
 * And the MPU must mark RAM_D2 as Device or Normal-non-cacheable.  With the
 * D-cache currently disabled in the .ioc this is belt and braces; enable the
 * D-cache later and it becomes load-bearing.
 * --------------------------------------------------------------------------- */

#endif /* HAL_SECTIONS_H */
