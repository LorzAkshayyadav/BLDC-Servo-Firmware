/* =============================================================================
 * hal_adc.c  --  three converters, two groups, two completely different jobs.
 *
 * THE SPLIT (arch section 3.2)
 *   INJECTED serves control.  Fires once per PWM period on TIM1_TRGO2, just
 *   before the carrier peak, and produces the currents the FOC ISR consumes.
 *   ADC1 and ADC2 run as a dual pair in injected-simultaneous mode so phases
 *   B and C are sampled at the same instant.
 *
 *   REGULAR serves protection.  Fires 320 kHz on TIM2_CC2 and produces
 *   NOTHING THE CPU EVER READS.  Its only output is an interrupt when an
 *   analog watchdog threshold is crossed.  That is the whole tier-1' design:
 *   the sample rate is affordable precisely because no software touches the
 *   results.
 *
 * WHY PHASE A IS DIFFERENT
 *   Phase A sits on ADC3, which cannot be lockstepped with the ADC1/ADC2
 *   pair.  Rather than carry that as a dependency, the control path uses only
 *   the guaranteed-simultaneous pair; for a three-wire motor those two carry
 *   all the information there is.  Phase A serves the sum-check only, and its
 *   timing relationship to the others is irrelevant for that (section 4.4).
 * ============================================================================= */

#include "hal.h"
#include "hal_sections.h"
#include "board_limits.h"
#include "adc.h"                 /* CubeMX: hadc1, hadc2, hadc3 */

/* -----------------------------------------------------------------------------
 * Scaling.  OPEN ITEMS -- every one of these is a board constant.
 *
 * A low-side shunt develops V = I x R, the INA241 amplifies it about a
 * mid-scale reference, and the ADC digitises the result.  Working backwards:
 *
 *     dV_uV   = (counts - offset) * VREF_MV * 1000 / 65536
 *     I_mA    = dV_uV / GAIN / R_SHUNT_mOHM
 *
 * because I(mA) x R(mOhm) = V(uV) exactly, which keeps the whole conversion
 * in integers with no scaling gymnastics.
 * -------------------------------------------------------------------------- */
#ifndef ADC_VREF_MV
#define ADC_VREF_MV            3300
#define SHUNT_MILLIOHM         5
#define INA241_GAIN            20
#define VBUS_DIV_NUM           1        /* v_bus_mv = counts * VREF * NUM/DEN */
#define VBUS_DIV_DEN           1
#warning "ADC scaling constants are placeholders -- shunt, INA241 gain, bus divider"
#endif

/* Low-side sensing: the shunt sees current leaving the phase into the
 * negative rail, so the sensed polarity is inverted relative to motor
 * convention.  Kept as a named constant because arch section 14 stage 3 has
 * "sign of every sensed current confirmed" as a pass criterion, and flipping
 * it must be a one-character edit rather than an expression rewrite. */
#define CURRENT_SIGN           (-1)

/* Q16 multiplier from raw counts to milliamps, folded once at compile time so
 * the ISR pays one multiply and one shift instead of three divisions. */
/* mA per raw count, in Q16.
 *
 *   I_mA = counts * VREF_MV * 1000 / (65536 * GAIN * R_mOHM)
 *
 * because I(mA) x R(mOhm) = V(uV) exactly. The 2^16 of the Q16 scaling and
 * the 2^16 of the ADC full scale cancel, which is why this reduces to a
 * division that fits comfortably in 32 bits. Writing it in the unreduced
 * form overflows at compile time and silently yields a wrong constant. */
#define MA_PER_COUNT_Q16 \
    (((int32_t)ADC_VREF_MV * 1000) / (INA241_GAIN * SHUNT_MILLIOHM))

#define OFFSET_CAL_SAMPLES     256u

static DTCM_BSS struct {
    uint16_t offset_b;
    uint16_t offset_c;
    uint16_t offset_a;
} g;

/* -----------------------------------------------------------------------------
 * The hot path.  Four register reads, no waiting.
 *
 * Arch section 4.1: the ISR cannot begin before its inputs exist, because the
 * data existing is what caused it to begin.  So there is no readiness check
 * here and there must never be one -- adding a poll would reintroduce the
 * latency the whole acquisition chain exists to remove.
 * -------------------------------------------------------------------------- */
ITCM_FUNC void hal_adc_read(hal_analog_t *out)
{
    /* Injected rank 1 on each converter.  ADC1 rank 2 carries the bus
     * voltage, which is sampled 417 ns after phase B -- irrelevant for a
     * quantity that moves on a millisecond timescale. */
    int32_t raw_b = (int32_t)ADC1->JDR1;
    int32_t raw_v = (int32_t)ADC1->JDR2;
    int32_t raw_c = (int32_t)ADC2->JDR1;
    int32_t raw_a = (int32_t)ADC3->JDR1;

    out->i_b_ma = (int32_t)((((int64_t)(raw_b - g.offset_b)) *
                             MA_PER_COUNT_Q16 * CURRENT_SIGN) >> 16);
    out->i_c_ma = (int32_t)((((int64_t)(raw_c - g.offset_c)) *
                             MA_PER_COUNT_Q16 * CURRENT_SIGN) >> 16);
    out->i_a_ma = (int32_t)((((int64_t)(raw_a - g.offset_a)) *
                             MA_PER_COUNT_Q16 * CURRENT_SIGN) >> 16);

    out->v_bus_mv = (raw_v * ADC_VREF_MV / 65536) * VBUS_DIV_NUM / VBUS_DIV_DEN;
}

/* -----------------------------------------------------------------------------
 * Offset calibration.
 *
 * The INA241 is bidirectional about a mid-scale reference, so zero current
 * reads near half full scale rather than near zero.  That resting value is
 * what has to be subtracted, and it is not exactly half scale -- it drifts
 * with the reference, the amplifier and temperature.
 *
 * WHY SOFTWARE-TRIGGERED REGULAR CONVERSIONS AND NOT THE INJECTED GROUP
 *   The injected group fires on TIM1_TRGO2, and TIM1 has not started yet --
 *   hal_init_sequencer() runs after this.  So EXTEN is temporarily cleared,
 *   the regular group is software-triggered, and the trigger is restored
 *   afterwards.  The regular group happens to carry exactly the three phase
 *   channels, so it measures the right thing.
 *
 * Arch section 14 stage 1 lists "offsets converge" as a pass criterion, which
 * is a re-check with the carrier running -- this is the cold measurement that
 * gets the watchdog thresholds close enough to arm.
 * -------------------------------------------------------------------------- */
static bool measure_offset(ADC_HandleTypeDef *h, uint16_t *out)
{
    uint32_t saved = h->Instance->CFGR & ADC_CFGR_EXTEN;
    h->Instance->CFGR &= ~ADC_CFGR_EXTEN;      /* software trigger */

    uint32_t acc = 0u;
    for (uint32_t i = 0u; i < OFFSET_CAL_SAMPLES; i++) {
        if (HAL_ADC_Start(h) != HAL_OK) { return false; }
        if (HAL_ADC_PollForConversion(h, 10u) != HAL_OK) { return false; }
        acc += HAL_ADC_GetValue(h);
    }
    (void)HAL_ADC_Stop(h);

    h->Instance->CFGR |= saved;
    *out = (uint16_t)(acc / OFFSET_CAL_SAMPLES);
    return true;
}

/* -----------------------------------------------------------------------------
 * Arm one analog watchdog.
 *
 * Both limits matter.  Bidirectional sensing means a shoot-through can appear
 * as a large excursion in EITHER direction depending on which leg and which
 * device, so a high-only threshold leaves half the fault space uncovered.
 * -------------------------------------------------------------------------- */
static bool arm_watchdog(ADC_HandleTypeDef *h, uint32_t channel, uint16_t offset)
{
    ADC_AnalogWDGConfTypeDef w = {0};

    w.WatchdogNumber = ADC_ANALOGWATCHDOG_1;   /* AWD1: full-resolution
                                                * thresholds.  AWD2/AWD3
                                                * compare only the upper bits,
                                                * which would give phase A a
                                                * coarser trip than B and C. */
    w.WatchdogMode   = ADC_ANALOGWATCHDOG_SINGLE_REG;
    w.Channel        = channel;
    w.ITMode         = ENABLE;
    w.HighThreshold  = (uint32_t)(offset + AWD_LIMIT_HI_Q15);
    w.LowThreshold   = (uint32_t)(offset - AWD_LIMIT_HI_Q15);

    return HAL_ADC_AnalogWDGConfig(h, &w) == HAL_OK;
}

/* -----------------------------------------------------------------------------
 * Bring-up.
 * -------------------------------------------------------------------------- */
bool hal_init_adc(void)
{
    /* ---- 1. Calibrate ---------------------------------------------------
     * Offset and linearity, single-ended, before any conversion.  Must be
     * redone if VDDA or temperature moves substantially. */
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY,
                                    ADC_SINGLE_ENDED) != HAL_OK) { return false; }
    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET_LINEARITY,
                                    ADC_SINGLE_ENDED) != HAL_OK) { return false; }
    if (HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET_LINEARITY,
                                    ADC_SINGLE_ENDED) != HAL_OK) { return false; }

    /* ---- 2. FIX-UPS FOR TWO THINGS THE .ioc DOES NOT SET ----------------
     *
     * These belong in CubeMX, and this block should be deleted once they are
     * there.  They are done in code meanwhile because the consequence of
     * either being absent is a silently dead protection lane, and a dead lane
     * looks identical to a working one until a fault arrives.
     *
     * (a) ADC2 has no regular external trigger in its IPParameters, so it
     *     defaults to software start and never converts.  Phase C's watchdog
     *     would never see a sample.  Dual mode couples only the INJECTED
     *     groups; the regular groups stay independent and each needs its own
     *     trigger.
     *
     * (b) ADC3 enables AWD1 but has no WatchdogChannel, so nothing is
     *     assigned to it.  arm_watchdog() below sets it explicitly, which
     *     covers this -- noted here so the .ioc gets fixed too. */
    MODIFY_REG(hadc2.Instance->CFGR,
               ADC_CFGR_EXTSEL | ADC_CFGR_EXTEN,
               ADC_EXTERNALTRIG_T2_CC2 | ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING);

    /* ---- 3. Offsets, with the bridge off --------------------------------
     * hal_init_safety() ran before this and left MOE clear, so the phases are
     * floating and any current reading is the amplifier's resting point. */
    if (!measure_offset(&hadc1, &g.offset_b)) { return false; }
    if (!measure_offset(&hadc2, &g.offset_c)) { return false; }
    if (!measure_offset(&hadc3, &g.offset_a)) { return false; }

    /* ---- 4. Arm the watchdogs -------------------------------------------
     * Thresholds are relative to the measured offsets, which is why step 3
     * has to come first. */
    if (!arm_watchdog(&hadc1, ADC_CHANNEL_11, g.offset_b)) { return false; }
    if (!arm_watchdog(&hadc2, ADC_CHANNEL_10, g.offset_c)) { return false; }
    if (!arm_watchdog(&hadc3, ADC_CHANNEL_0,  g.offset_a)) { return false; }

    /* ---- 5. Start the regular groups ------------------------------------
     * HAL_ADC_Start, not _IT and not _DMA.  Nothing reads these results, so
     * there is no completion to handle: the watchdog comparison happens in
     * hardware and only a crossing produces anything.
     *
     * EOCIE, EOSIE and OVRIE must all stay off.  Any of them would give
     * 320,000 interrupts per second per converter. */
    if (HAL_ADC_Start(&hadc1) != HAL_OK) { return false; }
    if (HAL_ADC_Start(&hadc2) != HAL_OK) { return false; }
    if (HAL_ADC_Start(&hadc3) != HAL_OK) { return false; }

    /* ---- 6. Start the injected groups -----------------------------------
     * Slave before master: in injected-simultaneous dual mode ADC2 must be
     * armed and waiting when ADC1's trigger arrives, or the first conversion
     * of the pair is not actually simultaneous.
     *
     * Plain InjectedStart, then the interrupt enabled by hand.  HAL's _IT
     * variant would route completion through HAL_ADC_IRQHandler, which walks
     * every flag and evaluates callbacks before reaching our code -- hundreds
     * of cycles of variable overhead on the hardest deadline in the system
     * (arch section 6.1).  isr_vectors.c replaces that handler entirely. */
    if (HAL_ADCEx_InjectedStart(&hadc2) != HAL_OK) { return false; }
    if (HAL_ADCEx_InjectedStart(&hadc3) != HAL_OK) { return false; }
    if (HAL_ADCEx_InjectedStart(&hadc1) != HAL_OK) { return false; }

    /* ---- 7. Interrupts ---------------------------------------------------
     * JEOS on ADC1 only.  It is the master of the pair and its sequence is
     * the longer of the two, so when it completes every injected result in
     * the system is resident.  Enabling it on ADC2 or ADC3 as well would give
     * two or three ISR entries per period -- exactly what the stage 1 pin
     * toggle check exists to catch.
     *
     * JEOC would fire per rank rather than per sequence: two entries per
     * period from ADC1 alone. */
    __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOS);

    __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_AWD1);
    __HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_AWD1);
    __HAL_ADC_ENABLE_IT(&hadc3, ADC_IT_AWD1);

    return true;
}