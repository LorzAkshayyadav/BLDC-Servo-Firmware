/* =============================================================================
 * fault_codes.h  --  every cause that can enter the safe state.
 *
 * Arch section 5.5 item 5: the cause is latched to a status word, exposed as
 * process data, and appended to the battery-backed fault history.  That is
 * only useful if the cause is specific, so there is no generic FAULT_OTHER.
 *
 * The tier is encoded in the top nibble so a fault history read tells you
 * immediately which layer caught it, which is the first thing you want to
 * know from a field failure.
 * ============================================================================= */
#ifndef COMMON_FAULT_CODES_H
#define COMMON_FAULT_CODES_H

#include <stdint.h>

typedef enum {
    FAULT_NONE                  = 0x0000,

    /* T0 -- gate driver internal, under a microsecond.  Reported, not
     * detected, by us: it acted autonomously.  With the comparator tier
     * absent this is the only sub-microsecond protection in the system
     * (risk R1, dependency D1). */
    FAULT_DRIVER_DESAT          = 0x0001,
    FAULT_DRIVER_UVLO           = 0x0002,
    FAULT_DRIVER_THERMAL        = 0x0003,

    /* T1' -- oversampled analog watchdogs, about 3.5 us.  The replacement
     * for the missing hardware trip, roughly three times slower. */
    FAULT_AWD_PHASE_A           = 0x1001,
    FAULT_AWD_PHASE_B           = 0x1002,
    FAULT_AWD_PHASE_C           = 0x1003,

    /* T2 -- FOC ISR, every 62.5 us. */
    FAULT_CURRENT_SUM           = 0x2001,   /* three currents do not sum to 0 */
    FAULT_OVERCURRENT_FILTERED  = 0x2002,
    FAULT_BUS_OVERVOLTAGE       = 0x2003,
    FAULT_BUS_UNDERVOLTAGE      = 0x2004,
    FAULT_OVERSPEED             = 0x2005,
    FAULT_ENC_MOTOR_FRAME       = 0x2006,   /* DMA TC never set              */
    FAULT_ENC_MOTOR_CRC         = 0x2007,   /* nPDERR in the status byte     */
    FAULT_ENC_MOTOR_STALE       = 0x2008,
    FAULT_ENC_LOAD_FRAME        = 0x2009,
    FAULT_ENC_LOAD_CRC          = 0x200A,
    FAULT_ENC_LOAD_STALE        = 0x200B,
    FAULT_ENC_FRAME_INIT        = 0x200C,   /* read scheduled before EOT     */
    FAULT_THERMAL_ACCUM         = 0x200D,
    FAULT_PERIOD_MISSED         = 0x200E,   /* TIM5 interval ~= 2 periods    */
    FAULT_PERIOD_SHORT          = 0x200F,   /* two entries too close         */

    /* T3 -- supervisory, 1 kHz.  Milliseconds to seconds. */
    FAULT_TEMP_POWER_STAGE      = 0x3001,
    FAULT_TEMP_MOTOR            = 0x3002,
    FAULT_FIELDBUS_WATCHDOG     = 0x3003,
    FAULT_TRANSMISSION_XCHECK   = 0x3004,   /* slipping coupling, stripped
                                             * gear, failed flexspline, drifting
                                             * encoder, swapped wiring       */
    FAULT_TORQUE_XCHECK         = 0x3005,   /* drifting sensor, wrong Kt,
                                             * gearbox degradation           */
    FAULT_PARAM_IMPLAUSIBLE     = 0x3006,
    FAULT_SYNC0_LOST            = 0x3007,
    FAULT_NONIUS_CTR            = 0x3008,   /* NON_CTR: absolute position may
                                             * be wrong after a restart      */

    /* Layer failures caught from outside (arch section 7). */
    FAULT_ISR_STOPPED           = 0x4001,   /* TIM6 deadline monitor         */
    FAULT_STO_ASSERTED          = 0x4002,
    FAULT_BREAK_LATCHED         = 0x4003,

    /* Persistent state.  If the integrity check fails, that is a fault
     * requiring re-referencing, NOT a reason to assume zero: you would
     * otherwise report a confidently wrong joint position while both
     * encoders read perfectly (arch section 10.1). */
    FAULT_MULTITURN_INVALID     = 0x5001,
    FAULT_CALIBRATION_INVALID   = 0x5002,
} fault_code_t;

/**
 * @brief Extract the originating tier from a fault code's top nibble.
 * @param c A @ref fault_code_t value.
 * @return  The tier number (e.g. 0 for T0, 2 for T2, 4 for a layer failure).
 */
#define FAULT_TIER(c)   (((uint16_t)(c)) >> 12)

#endif /* COMMON_FAULT_CODES_H */
