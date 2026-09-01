/* =============================================================================
 * hal_watchdog.c  --  IWDG1, refreshed only from the supervisory task.
 *
 * WHY REFRESHING HERE IS THE WHOLE POINT (app/supervisor.h has the full
 * argument; this is the one-line version)
 *   Refreshing from anywhere else would only prove that context is alive.
 *   Refreshing here, called only from the 1 kHz supervisory task and only as
 *   its last step, proves the whole chain end to end: TIM1 -> ADC -> FOC ISR
 *   -> pend -> this task (arch section 7, L4).
 *
 * TIMEOUT, FROM iwdg.c'S CUBEMX CONFIGURATION
 *   Prescaler /32, Reload 249, LSI ~32 kHz: (249+1) * 32 / 32000 = 250 ms.
 *   Window is 4095 (the reload value's ceiling), i.e. no early-refresh
 *   restriction -- HAL_IWDG_Refresh() is valid at any point in the count.
 * ============================================================================= */

#include "hal.h"
#include "iwdg.h"      /* CubeMX: hiwdg1 */

void hal_watchdog_refresh(void)
{
    HAL_IWDG_Refresh(&hiwdg1);
}
