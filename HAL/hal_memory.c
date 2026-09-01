/* =============================================================================
 * hal_memory.c  --  MPU regions and caches, before anything places a buffer.
 *
 * WHY THIS RUNS BEFORE hal_init_safety()/hal_init_adc()/hal_init_encoders()
 *   Every one of those places a DMA_BUF (hal/hal_sections.h) in RAM_D2 before
 *   the carrier ever starts. If the non-cacheable MPU region for RAM_D2 is
 *   not live by the time the first buffer is written, the failure is silent:
 *   a stale read under load, not a fault at setup time (arch section 10).
 *
 * WHY REGION 1, NOT REGION 0
 *   platform/Core/Src/main.c's CubeMX-generated MPU_Config() already claimed
 *   region 0 for its own background no-access configuration, and runs before
 *   SystemClock_Config()/HAL_Init(), long before servo_main() calls this.
 *   Disabling/re-enabling the MPU here to add region 1 does not disturb
 *   region 0's already-programmed state -- each numbered region's registers
 *   persist independently of the global enable bit.
 *
 * WHY 512 KB, NOT 288 KB
 *   RAM_D2 is 288 KB (linker/stm32h743_servo.ld: ORIGIN 0x30000000, LENGTH
 *   288K), but the Cortex-M7 MPU requires power-of-two region sizes aligned
 *   to their own size. 512 KB is the smallest power of two that covers it,
 *   based at the same 0x30000000 that already satisfies 512 KB alignment.
 *   The 224 KB tail beyond the real RAM_D2 is unpopulated address space --
 *   marking it non-cacheable too costs nothing.
 *
 * WHY THIS DOES NOT ENABLE THE D-CACHE
 *   hal/hal_sections.h is explicit: the D-cache is currently disabled in the
 *   .ioc, so this MPU region is "belt and braces" for now and becomes
 *   load-bearing only once the D-cache is enabled later. Turning the cache on
 *   is a separate, deliberate change, not a side effect of this file.
 * ============================================================================= */

#include "hal.h"
#include "main.h"

bool hal_init_memory(void)
{
    MPU_Region_InitTypeDef mpu = {0};

    HAL_MPU_Disable();

    mpu.Enable           = MPU_REGION_ENABLE;
    mpu.Number           = MPU_REGION_NUMBER1;
    mpu.BaseAddress      = 0x30000000u;                  /* RAM_D2, linker script */
    mpu.Size             = MPU_REGION_SIZE_512KB;
    mpu.SubRegionDisable = 0x00u;
    mpu.TypeExtField     = MPU_TEX_LEVEL0;
    mpu.AccessPermission = MPU_REGION_FULL_ACCESS;
    /* Every DMA_BUF is a buffer, never code -- no instruction fetch should
     * ever come from here, so mark it execute-never. */
    mpu.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
    mpu.IsShareable      = MPU_ACCESS_SHAREABLE;
    mpu.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
    mpu.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;

    HAL_MPU_ConfigRegion(&mpu);
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

    return true;
}
