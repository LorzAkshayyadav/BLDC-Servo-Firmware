/* =============================================================================
 * build_config.h  --  feature selection, with guards on invalid combinations.
 *
 * Arch section 13 asks for guards, not just switches.  A feature matrix with
 * no guards eventually produces a build that compiles, runs, and is quietly
 * missing a protection tier.
 * ============================================================================= */
#ifndef CONFIG_BUILD_CONFIG_H
#define CONFIG_BUILD_CONFIG_H

/* -- Bring-up staging ----------------------------------------------------- */
/* Arch section 14: the system builds and runs deliberately incomplete, in a
 * defined order, each stage with a pass criterion.  This is the debugging
 * strategy, not a tutorial. */
#define STAGE_PROTECTION_ONLY   0   /* stage 0: no motor, watchdogs armed     */
#define STAGE_TRIGGERS          1   /* stage 1: fixed duty, scope everything  */
#define STAGE_ENCODERS          2   /* stage 2: hand rotation, no motion      */
#define STAGE_OPEN_LOOP         3   /* stage 3: forced angle                  */
#define STAGE_CURRENT_LOOP      4
#define STAGE_FOC               5
#define STAGE_MOTION            6
#define STAGE_FIELDBUS          7
#define STAGE_PRODUCTION        8

#ifndef BRINGUP_STAGE
#define BRINGUP_STAGE           STAGE_PROTECTION_ONLY
#endif

/* -- Instrumentation ------------------------------------------------------ */
#define FEATURE_SCOPE_BUFFER    1   /* build it early; it repays itself       */
#define FEATURE_PIN_TOGGLE      (BRINGUP_STAGE < STAGE_PRODUCTION)
#define FEATURE_CONSOLE         (BRINGUP_STAGE < STAGE_PRODUCTION)

/* -- Guards --------------------------------------------------------------- */
#if BRINGUP_STAGE >= STAGE_OPEN_LOOP && !defined(AWD_LIMIT_HI_Q15)
#error "Cannot connect a motor before the fast protection tier is armed (stage 0)"
#endif

#if BRINGUP_STAGE >= STAGE_PRODUCTION && FEATURE_CONSOLE
#error "Console UART must be off the deadline path in a production build"
#endif

/* The in-RAM scope is the mitigation recorded against risk R2, no trace
 * output, because an encoder interface occupies the trace pins.  Removing it
 * removes the mitigation. */
#if BRINGUP_STAGE >= STAGE_PRODUCTION && !FEATURE_SCOPE_BUFFER
#error "Scope buffer is the R2 mitigation and may not be disabled"
#endif

/* Worst-case ISR duration must be measured with all instrumentation ENABLED
 * (arch section 15), so a production build that silently drops it invalidates
 * the number. */
#if BRINGUP_STAGE == STAGE_PRODUCTION && !FEATURE_SCOPE_BUFFER
#error "Timing measurements were taken with the scope enabled"
#endif

#endif /* CONFIG_BUILD_CONFIG_H */
