/* =============================================================================
 * board_limits.h  --  values that come from the POWER STAGE, not the control
 *                     design.  Every one is a measurement or a datasheet
 *                     number.  None may be guessed.
 *
 * The #warning guards below are deliberate.  Arch principle P7: where the
 * hardware prevents the intended mechanism, the architecture names the
 * shortfall, quantifies it, and records it -- rather than quietly
 * substituting something weaker and calling it equivalent.  A placeholder
 * that silently ships as a real limit is exactly that failure.
 *
 * NAMING NOTE: avoid PERIPHERAL_WORD_WORD-shaped names here (e.g. a
 * hypothetical TIM_TICK_HZ or ADC_APERTURE_TICKS), even though every value
 * in this file is timer/ADC-derived. Tools/layer_check.py's register-bit
 * heuristic cannot tell "TIM_TICK_HZ, a config constant" from "TIM_CR1_CEN,
 * a real register bit" by pattern alone, and it is not this file's job to
 * carry that ambiguity into a check meant to run in CI.
 * ============================================================================= */
#ifndef CONFIG_BOARD_LIMITS_H
#define CONFIG_BOARD_LIMITS_H

#include <stdint.h>

/* -- Carrier -------------------------------------------------------------- */
/* Centre-aligned, so one period is 2 x ARR counts.  240 MHz / 15000 = 16 kHz
 * exactly, and 750 divides 15000 twenty times, which is what keeps the
 * protection sampling uniform across the reset (arch section 5.3). */
#define CARRIER_TICK_HZ             240000000u
#define CARRIER_ARR             7500u
#define CARRIER_TICKS           (2u * CARRIER_ARR)      /* 15000 */
#define CARRIER_HZ              (CARRIER_TICK_HZ / CARRIER_TICKS)
#define TICKS_PER_US            (CARRIER_TICK_HZ / 1000000u)

/* -- Dead time ------------------------------------------------------------ */
/* DTG x t_DTS, t_DTS = 1/240 MHz = 4.167 ns.  60 -> 250 ns.
 * OPEN, dependency D1: confirm against gate driver and FET switching times
 * before stage 3.  This is a power-electronics number. */
#define DEAD_TIME_DTG           60u

/* -- Duty clamp, forced by LOW-SIDE current sensing ----------------------- */
/* A low-side shunt only carries phase current while its low-side device
 * conducts, and all three conduct only in the null vector at the carrier
 * peak.  So the highest-duty phase sets how much quiet time the sample gets:
 *
 *   CARRIER_ARR - CCR_max - DEAD_TIME_DTG  >=  settling + half aperture
 *
 * OPEN: INA241 settling time after the common-mode step at the switching
 * edge.  Scope the amplifier output at stage 3. */
#ifndef INA241_SETTLE_TICKS
#define INA241_SETTLE_TICKS     240u      /* PLACEHOLDER: 1 us assumed */
#warning "INA241_SETTLE_TICKS is a placeholder -- measure at bring-up stage 3"
#endif
#define SAMPLE_APERTURE_TICKS      66u       /* 16.5 ADC cycles at 60 MHz */
#define CCR_MAX  (CARRIER_ARR - DEAD_TIME_DTG - INA241_SETTLE_TICKS \
                              - (SAMPLE_APERTURE_TICKS / 2u))

/* -- Trigger placement ---------------------------------------------------- */
/* CCR4 slaves TIM2/TIM3/TIM4.  PWM mode 2, so the rising edge is on the
 * UP-count -- under PWM mode 1 the polarity inverts and the whole acquisition
 * chain lands after the current sample instead of before it. */
#define CCR4_SLAVE_RESET        2625u     /* 10.9375 us after the trough */

/* CCR5 -> TRGO2 -> injected groups.  Placed so the sampling APERTURE is
 * centred on the peak, not so the trigger lands on it. */
#define CCR5_INJECTED_TRIGGER   (CARRIER_ARR - (SAMPLE_APERTURE_TICKS / 2u))

/* -- FOC ISR entry, for foc_isr.c's late-entry check ----------------------
 * The injected conversion that triggers the ISR completes SAMPLE_APERTURE_TICKS
 * after CCR5_INJECTED_TRIGGER, so the carrier counter (still counting up,
 * short of the ARR peak) should read close to that sum at ISR entry.  This
 * is trigger placement, not a control tuning value, which is why it lives
 * here rather than in config/control_params.h -- foc_isr.c includes both
 * headers directly (see LATE_ENTRY_LIMIT in control_params.h, the tuning
 * threshold that goes with this jitter measurement). */
#define CARRIER_ENTRY_EXPECTED  (CCR5_INJECTED_TRIGGER + SAMPLE_APERTURE_TICKS)
#ifndef CARRIER_ENTRY_TOL
#define CARRIER_ENTRY_TOL       120u      /* 0.5 us; PLACEHOLDER */
#warning "CARRIER_ENTRY_TOL is a placeholder -- set from the jitter histogram at bring-up stage 1"
#endif

/* TIM2: 750 counts = 3.125 us = 320 kHz, twenty samples per period.
 * Toggle mode on CH2 gives one edge per wrap, so the ADC regular trigger
 * must be set to BOTH edges or the rate halves to 160 kHz. */
#define PROTECTION_ARR          749u

/* TIM4 sequencer, all values in ticks AFTER the slave reset. */
#define GETSENS_PULSE_TICKS     240u      /* 1 us; must fall before EOT rises */
#ifndef ENC_READ_START_TICKS
#define ENC_READ_START_TICKS   1440u      /* 6 us: 3.4 us frame + 2.6 margin */
#warning "ENC_READ_START_TICKS assumes a computed frame -- confirm at stage 2"
#endif

/* TIM6 deadline monitor at 1 MHz.  Longer than one period plus worst-case
 * entry jitter so it never false-trips, short enough that a stale voltage
 * vector does no damage.  A false trip is evidence of a long critical
 * section, not a reason to raise this number. */
#define DEADLINE_PSC            239u
#define DEADLINE_ARR            99u       /* 100 us = 1.6 carrier periods */

/* -- Protection thresholds ------------------------------------------------ */
/* OPEN: derive from shunt value, INA241 gain, and the fault current the
 * devices survive at the MEASURED trip latency.  Arch section 5.4 and risk
 * R1 -- the let-through energy calculation belongs to power electronics.
 * Set BELOW normal peak current for stage 0 so an injected fault
 * demonstrably reaches the safe state on each phase in turn.
 *
 * Bidirectional sensing means a shoot-through can appear as a large
 * excursion in EITHER direction, so both limits are load-bearing. */
#ifndef AWD_LIMIT_HI_Q15
#define AWD_LIMIT_HI_Q15        0
#define AWD_LIMIT_LO_Q15        0
#warning "Analog watchdog limits unset -- bring-up stage 0 cannot pass"
#endif

/* -- Over-speed ----------------------------------------------------------- */
/* The iC-MU200 master-track signal frequency limit is 7 kHz; above it
 * FRQ_CNV sets and the tracking converter has stopped following the input.
 * A different and usually lower limit than the motor's own rating. */
#define ENCODER_TRACK_FREQ_MAX_HZ  7000u

#endif /* CONFIG_BOARD_LIMITS_H */
