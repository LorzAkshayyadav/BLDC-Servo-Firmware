/**
 * @file contracts.h
 * @brief Every datum that crosses a context boundary, and the mechanism it
 *        crosses by. Arch section 12, table 9.
 *
 * @details
 * PRINCIPLE P4, SINGLE WRITER PER FIELD: each shared datum has exactly one
 * writing context. Coherent multi-word sets cross domains by double buffer
 * with a single-word swap, never field by field.
 *
 * WHY A SINGLE-WORD SWAP IS ENOUGH: an aligned 32-bit store on Cortex-M7 is
 * atomic with respect to interrupts. So the producer fills the inactive
 * half, then publishes with one store to `active`. A consumer that reads
 * `active` either sees the old index or the new one, never a half-updated
 * set.
 *
 * THE RULE CONSUMERS MUST FOLLOW: take ONE copy of the index at entry and
 * use that copy throughout. Re-reading `active` mid-computation
 * reintroduces exactly the tear the double buffer was avoiding -- you can
 * end up computing Clarke and Park on one period's angle and the inverse
 * Park on the next.
 *
 * This header contains no hardware types, so it compiles for the host.
 */

#ifndef CONTRACTS_H
#define CONTRACTS_H

#include <stdint.h>
#include <stdbool.h>
#include "hal/hal.h"

/* -------------------------------------------------------------------------
 * The double-buffer pattern, written once.
 *
 * Producer:   T *w = CONTRACT_WRITE_SLOT(c);
 *             ... fill *w ...
 *             CONTRACT_PUBLISH(c);
 *
 * Consumer:   uint32_t idx = CONTRACT_TAKE(c);
 *             const T *r = &(c).slot[idx];
 *             ... use r for the whole computation ...
 * ------------------------------------------------------------------------- */
/**
 * @brief Declare a double-buffered contract type named @p name for @p type.
 * @param type Payload struct/union carried in each of the two slots.
 * @param name Name of the generated typedef.
 */
#define CONTRACT_DECLARE(type, name)                                          \
    typedef struct { volatile uint32_t active; type slot[2]; } name

/**
 * @brief Pointer to the inactive slot of @p c, for the producer to fill.
 * @param c A contract instance declared via @ref CONTRACT_DECLARE.
 * @return  Pointer to the slot not currently visible to consumers.
 */
#define CONTRACT_WRITE_SLOT(c)  (&(c).slot[1u - (c).active])

/**
 * @brief Publish the slot just filled via @ref CONTRACT_WRITE_SLOT.
 *
 * One aligned 32-bit store, atomic with respect to interrupts on Cortex-M7.
 *
 * @param c A contract instance declared via @ref CONTRACT_DECLARE.
 */
#define CONTRACT_PUBLISH(c)     do { (c).active = 1u - (c).active; } while (0)

/**
 * @brief Take the index of the currently published slot.
 *
 * Callers must store the result once and use that copy for the whole
 * computation -- re-reading mid-computation reintroduces the tear the
 * double buffer exists to avoid.
 *
 * @param c A contract instance declared via @ref CONTRACT_DECLARE.
 * @return  Index (0 or 1) of the slot currently visible to consumers.
 */
#define CONTRACT_TAKE(c)        ((c).active)

/* =========================================================================
 * 1.  FEEDBACK AND STATUS
 *     writer: torque (L2), at ISR exit
 *     readers: motion, supervisor, fieldbus
 * ========================================================================= */
typedef struct {
    uint32_t angle_electrical;
    uint32_t position_motor;
    uint32_t position_load;
    int32_t  i_d_ma;
    int32_t  i_q_ma;
    int32_t  v_bus_mv;
    uint32_t status_word;
    uint32_t stamp;
    uint16_t duty_max_applied;   /* for saturation reporting                 */
    bool     duty_clamped;
} feedback_t;

CONTRACT_DECLARE(feedback_t, feedback_contract_t);
extern feedback_contract_t g_feedback;

/* =========================================================================
 * 2.  CURRENT REFERENCE
 *     writer: motion (L3), 4 kHz
 *     reader: torque (L2), 16 kHz
 *     mechanism: single aligned word, atomic against interrupts
 *
 *     Two int16 packed into one word so the pair is coherent without a
 *     double buffer.  q and d are meaningless independently.
 * ========================================================================= */
typedef union {
    volatile uint32_t word;
    struct { int16_t i_q_ma; int16_t i_d_ma; } v;
} current_ref_t;
extern current_ref_t g_current_ref;

/* =========================================================================
 * 3.  VELOCITY ACCUMULATOR
 *     writer: torque (L2) accumulates every period, swaps every 4th
 *     reader: motion (L3), reads the completed set, writes nothing
 *
 *     WHY THIS IS DOUBLE BUFFERED and not a plain pair of counters:
 *     a plain accumulator cleared by the consumer has two writers, which
 *     violates P4 and lets the motion task zero an accumulator mid-period.
 *     Here the ISR owns both halves; the motion task only reads.
 *
 *     WHY ACCUMULATE AT ALL (see notes in motion/velocity_loop.c):
 *     differentiation amplifies quantisation noise in inverse proportion to
 *     the interval.  The four consecutive differences telescope to the total
 *     displacement, so accumulating loses no information and divides the
 *     noise floor by four.  At 19-bit, 1.83 rpm/LSB becomes 0.46 rpm/LSB.
 * ========================================================================= */
typedef struct {
    int32_t  d_position;   /* summed wrap-safe differences over the window   */
    uint32_t d_ticks;      /* summed measured intervals, NOT nominal         */
    uint16_t samples;      /* normally HAL_MOTION_DIVIDER; less means a
                            * period was rejected, so scale accordingly      */
} velocity_accum_t;

CONTRACT_DECLARE(velocity_accum_t, velocity_contract_t);
extern velocity_contract_t g_velocity;

/* =========================================================================
 * 4.  POSITION COMMAND
 *     writer: fieldbus SYNC0 context
 *     reader: motion (L3)
 *     mechanism: double buffer plus interpolator state
 *
 *     Setpoints arrive at the network cycle rate while the position loop runs
 *     faster.  Without interpolation the loop sees a staircase, which becomes
 *     a velocity step and then a torque step at the network rate -- audible,
 *     and mechanically excitatory in a geared joint (arch section 9.3).
 * ========================================================================= */
typedef struct {
    int32_t  position;
    int32_t  velocity_ff;    /* if the master provides it                    */
    int32_t  accel_ff;
    uint32_t stamp;
    bool     ff_valid;
} position_cmd_t;
typedef struct {
    int32_t  velocity_rpm;                   
    int32_t  accel_ff;
    uint32_t stamp;
    bool     ff_valid;
} velocity_cmd_t;
typedef enum {
    CTRL_MODE_IDLE = 0,          /* bridge enabled, zero duty              */
    CTRL_MODE_FIXED_DUTY,        /* stage 1: motor disconnected            */
    CTRL_MODE_OPEN_LOOP_VOLTAGE, /* stage 3: forced angle, no current loop */
    CTRL_MODE_CURRENT_FORCED,    /* stage 4: current loop, forced angle    */
    CTRL_MODE_FOC,               /* stage 5 onward                         */
} ctrl_mode_t;
CONTRACT_DECLARE(position_cmd_t, position_cmd_contract_t);
CONTRACT_DECLARE(velocity_cmd_t, velocity_cmd_contract_t);
extern position_cmd_contract_t g_position_cmd;
extern velocity_cmd_contract_t g_velocity_cmd;

/* =========================================================================
 * 5.  GAINS AND PARAMETERS
 *     writer: supervisor (L4/1 kHz)
 *     readers: torque, motion
 *     mechanism: double buffer plus swap flag -- NEVER field by field
 *
 *     Writing gains one field at a time means the current loop can run one
 *     period with a new Kp and an old Ki.  On a wound-up integrator that
 *     presents as a hardware fault and is not one.
 * ========================================================================= */
typedef struct {
    int32_t  kp_current_q15;
    int32_t  ki_current_q15;
    int32_t  kp_velocity_q15;
    int32_t  ki_velocity_q15;
    int32_t  kp_position_q15;
    uint32_t angle_advance_per_rpm;   /* tuned empirically, arch section 8.1 */
    int32_t  i_limit_ma;
    int32_t  v_bus_max_mv;
    int32_t  v_bus_min_mv;
    uint32_t speed_limit_rpm;         /* below the iC-MU 7 kHz track limit   */
    int32_t  current_sum_tol_ma;
    uint32_t transmission_tol;        /* built from 20 arcsec/encoder, not
                                       * from the 2.5 arcsec resolution      */
    uint32_t torque_path_tol_mnm;
    ctrl_mode_t ctrl_mode;
    int32_t     forced_speed_rpm;   /* CTRL_MODE_*_FORCED only */
} params_t;

CONTRACT_DECLARE(params_t, params_contract_t);
extern params_contract_t g_params;

/* =========================================================================
 * 6.  FAULT WORD
 *     writers: all tiers
 *     readers: supervisor, fieldbus
 *     mechanism: ATOMIC BIT-SET ONLY.  Cleared solely by an explicit reset
 *                command through the drive state machine.
 *
 *     Bit-set only is what makes multiple writers safe without a lock: two
 *     contexts setting different bits cannot lose each other's write, and no
 *     context ever needs to read-modify-write.
 * ========================================================================= */
extern volatile uint32_t g_fault_word;

/**
 * @brief Atomically OR one trip cause's bit into the fault word.
 *
 * Bit-set only, per the mechanism above: two contexts setting different
 * bits cannot lose each other's write, and no context ever needs to
 * read-modify-write. Never call this to clear a bit.
 *
 * @param cause Trip cause whose bit is to be set.
 */
static inline void fault_set(hal_trip_cause_t cause)
{
    __atomic_or_fetch(&g_fault_word, (1u << (uint32_t)cause), __ATOMIC_RELAXED);
}

/**
 * @brief Check whether any fault bit is currently latched.
 * @return true if @ref g_fault_word is nonzero.
 */
static inline bool fault_any(void) { return g_fault_word != 0u; }
void fault_clear_all(void);
#endif /* CONTRACTS_H */
