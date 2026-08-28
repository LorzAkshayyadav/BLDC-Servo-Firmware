/* =============================================================================
 * motor_params.h  --  PER COMMISSIONED UNIT.  Tuned values are written back
 *                     here, so this file is unit-specific, not design-generic.
 *
 * Arch section 13 keeps this separate from control_params.h for that reason:
 * gains belong to the design, these belong to the physical joint in front of
 * you, and confusing the two makes a commissioned unit unreproducible.
 * ============================================================================= */
#ifndef CONFIG_MOTOR_PARAMS_H
#define CONFIG_MOTOR_PARAMS_H

#include <stdint.h>

#define UNIT_SERIAL             "UNCOMMISSIONED"

/* -- Motor ---------------------------------------------------------------- */
#define MOTOR_POLE_PAIRS        0u        /* OPEN: from the motor datasheet   */
#define MOTOR_KT_MNM_PER_A      0u        /* torque constant, verified stage 5 */

/* Commutation offset between the encoder zero and the motor's electrical
 * zero.  Found by calibration at stage 5, not by assumption. */
#define COMMUTATION_OFFSET      0u        /* angle_t units */

/* -- Current sense offsets -------------------------------------------------
 * Per-unit residual offset removal (arch section 2): hal_adc_read() has
 * already applied the low-side sign inversion and the INA241 gain/shunt
 * scale, so what is left here is this specific board's zero-current
 * amplifier offset. Measured with the bridge disabled at commissioning, not
 * assumed, because it varies unit to unit with the amplifier and shunt.
 * foc_isr.c subtracts these before the sum check and Clarke transform. */
#ifndef MOTOR_I_OFFSET_A_MA
#define MOTOR_I_OFFSET_A_MA     0
#define MOTOR_I_OFFSET_B_MA     0
#define MOTOR_I_OFFSET_C_MA     0
#warning "MOTOR_I_OFFSET_*_MA are placeholders -- measure at commissioning"
#endif

/* -- Transmission --------------------------------------------------------- */
#define GEAR_RATIO_NUM          0u
#define GEAR_RATIO_DEN          1u

/* -- Encoders ------------------------------------------------------------- */
/* MPC is the master-track pole-pair count of the magnetic target and it sets
 * everything downstream: PDLEN, the iC-MBE read length, the over-speed trip,
 * and the cross-check threshold.
 *
 *   MPC 0x4 -> 16/15 nonius, 18 bit, 24000 rpm limit
 *   MPC 0x5 -> 32/31 nonius, 19 bit, 12000 rpm limit
 *   MPC 0x6 -> 64/63 nonius, 20 bit,  6000 rpm limit
 *
 * The two encoders may legitimately differ: the motor side needs speed
 * headroom, the load side sits behind the gearbox and can take resolution.
 *
 * Design review note 4 in the iC-MU datasheet argues against 64/63 in the
 * QFN48 package: package stress raises analogue offset in the nonius track,
 * reducing phase margin, which appears as NON_CTR and can give an incorrect
 * absolute position after a restart. */
#define ENC_MOTOR_MPC           0x5u
#define ENC_LOAD_MPC            0x5u

/* Filter selection is a resolution-versus-latency trade and the latency lands
 * directly on angle staleness.  FILT 0x1 gives 14-bit interpolation at under
 * 1 us; the reset default 0x6 gives 41 us, which exceeds a whole PWM period
 * and is frequency dependent, so the angle advance term cannot compensate it. */
#define ENC_MOTOR_FILT          0x1u      /* <1 us, 14 bit, 15 dB   */
#define ENC_LOAD_FILT           0x3u      /* 10 us is free at 4 kHz */

/* Measured by the iC-MBE during its INIT sequence, in quarter-BiSS-clock
 * units, and read back at startup.  Recorded here per unit because it depends
 * on cable length.  Addendum section 5.1 term 3: this was the genuinely
 * unknown term, and the chip measures it for you. */
#define ENC_MOTOR_LINE_DELAY    0u        /* filled at commissioning */
#define ENC_LOAD_LINE_DELAY     0u

/* -- Cross-check thresholds (arch section 8.3) ---------------------------- */
/* Build these from ACCURACY, not resolution.  Absolute angular accuracy is
 * 2 LSB referenced to 12 bits of the sine period, so at 32 pole pairs one
 * period is 11.25 degrees mechanical and 2 LSB is about 20 arcsec per
 * encoder -- eight times the 2.5 arcsec output resolution.  Size the margin
 * from 20 arcsec plus backlash plus compliance, then characterise by
 * measurement at stage 6 rather than guessing. */
#define XCHECK_TRANSMISSION_TOL 0u
#define XCHECK_TORQUE_TOL       0u

#endif /* CONFIG_MOTOR_PARAMS_H */
