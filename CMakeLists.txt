cmake_minimum_required(VERSION 3.22)

# =============================================================================
# EtherCAT Servo Joint Drive
#
# CHANGES FROM THE CUBEMX-GENERATED VERSION, AND WHY:
#
#  1. Directory case fixed.  The generated file globbed App/ FOC/ HAL/ Config/
#     while the tree has app/ foc/ hal/ config/.  file(GLOB) on a missing
#     directory returns EMPTY WITH NO ERROR, so on any case-sensitive
#     filesystem this project built successfully with none of the firmware
#     compiled in.  That is the worst class of build bug: silent success.
#
#  2. GLOB replaced with explicit lists.  CMake does not re-run when a new
#     file appears, so a newly added tier2_checks.c is silently not compiled.
#     On a project where a missing file means a missing protection tier that
#     is not acceptable.  Explicit lists fail loudly instead.
#
#  3. common/*.c added.  It was on the include path but its sources were never
#     compiled, so fixed.c and contracts.c would have linked as undefined.
#
#  4. Custom linker script moved to linker/, because CubeMX regenerates
#     platform/ and would clobber it, and nothing CubeMX-generated references
#     its path. The startup file COULD NOT move the same way: CubeMX's own
#     generated platform/cmake/stm32cubemx/CMakeLists.txt hardcodes
#     platform/startup_stm32h743xx.s into this target's sources, so it must
#     stay there or the executable ends up with startup_stm32h743xx.s
#     compiled in twice (from linker/ here and from platform/ there) and a
#     duplicate Reset_Handler/g_pfnVectors link error. Its ITCM/DTCM copy
#     loops (Reset_Handler) must therefore be manually re-applied after every
#     CubeMX "Generate Code" -- same manual-reconciliation category
#     CODE_LAYOUT.md already documents for the three build files that carry a
#     path into platform/, now with this as a fourth item.
#
#  5. Both build checks wired.  Arch section 13: layer discipline that exists
#     only in a document decays within months; the check that fails the build
#     is what actually holds the boundary.
# =============================================================================

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME BLDC_18A_STM32H743)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})
enable_language(C ASM)

add_executable(${CMAKE_PROJECT_NAME})
add_subdirectory(platform/cmake/stm32cubemx)

# -----------------------------------------------------------------------------
# Sources, explicit.  Files not yet written are listed and commented out, so
# the list doubles as a checklist and adding one is a single-character edit.
# -----------------------------------------------------------------------------
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # -- dependency-free -------------------------------------------------
    common/contracts.c          # MISSING: defines the g_* storage
    common/fixed.c              # MISSING: trig table

    # -- L1 --------------------------------------------------------------
    hal/hal_memory.c
    hal/hal_pwm.c
    hal/hal_sequencer.c
    hal/hal_adc.c
    hal/hal_encoder.c
    hal/hal_encoder_ll.c
    hal/hal_fieldbus.c
    hal/hal_torque_sensor.c
    hal/hal_safety.c
    hal/hal_watchdog.c
    hal/hal_time.c
    hal/isr_vectors.c

    # -- L2 --------------------------------------------------------------
    foc/foc_isr.c
    foc/transforms.c
    foc/angle.c
    foc/current_pi.c
    foc/svpwm.c
    foc/tier2_checks.c
    foc/scope_log.c

    # -- L3 --------------------------------------------------------------
    motion/motion.c
    motion/position_loop.c
    motion/velocity_loop.c
    motion/torque_loop.c
    motion/interpolator.c
    motion/feedforward.c
    motion/traj_limit.c
    motion/brake_seq.c

    # -- L4 --------------------------------------------------------------
    app/servo_main.c
    app/supervisor.c
    app/drive_profile.c
    app/fieldbus_app.c
    app/object_dict.c
    app/param_service.c
    app/clock_discipline.c
    app/scope_readout.c
    app/fw_update.c

    # startup_stm32h743xx.s is NOT listed here: it is added by
    # platform/cmake/stm32cubemx/CMakeLists.txt (MX_Application_Src), which
    # hardcodes platform/startup_stm32h743xx.s. Listing it again from another
    # path would compile it twice into this same target. See change #4 above.
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Project root FIRST: every file uses root-relative quoted includes
    # (#include "hal/hal.h", #include "foc/transforms.h", ...), not bare
    # filenames, so the subdirectories below exist only for files that use
    # the bare form (e.g. hal/*.c including its own directory's headers
    # directly, or hal_pwm.c's #include "hal_private.h").
    ${CMAKE_CURRENT_SOURCE_DIR}
    app
    motion
    foc
    config
    common
    hal
)

# -----------------------------------------------------------------------------
# Warnings.  -Wdouble-promotion is not cosmetic: the H743 FPU is
# double-precision capable, single-precision ops are one cycle and double are
# several, and a stray `1.0` instead of `1.0f` silently promotes an entire
# expression.  On a 16 kHz ISR this one flag is worth more than most
# optimisation work.
# -----------------------------------------------------------------------------
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    -Wall -Wextra
    -Wdouble-promotion -Werror=double-promotion
    -Wshadow
    -Wconversion                # catches silent Q15 truncation
    -Wundef                     # a typo'd BRINGUP_STAGE would evaluate to 0
    -ffp-contract=fast          # lets the compiler form VFMA, one cycle
)

# -----------------------------------------------------------------------------
# Linker script from linker/, not platform/.
# -----------------------------------------------------------------------------
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -T${CMAKE_CURRENT_SOURCE_DIR}/linker/stm32h743_servo.ld
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map,--cref
    -Wl,--print-memory-usage
)
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    LINK_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/linker/stm32h743_servo.ld)

# -----------------------------------------------------------------------------
# The two build checks.
# -----------------------------------------------------------------------------
find_package(Python3 REQUIRED COMPONENTS Interpreter)

add_custom_target(layer_check ALL
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/Tools/layer_check.py
            ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "layer_check: peripheral symbols must not escape hal/")
add_dependencies(${CMAKE_PROJECT_NAME} layer_check)

# Section attributes are silently ignored more often than you would expect,
# and a DMA buffer in TCM fails at runtime with no error and no transfer.
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/Tools/section_check.py
            ${CMAKE_PROJECT_NAME}.map
    COMMENT "section_check: verifying memory placement against the map")

# -----------------------------------------------------------------------------
# Host build.  Arch section 2: if L2 and L3 compile and run on a host with L1
# stubbed, the boundary is real and you can regression-test control logic
# against a plant model without hardware.  If they do not, the compiler errors
# are an exhaustive list of your layer violations.
#
#   cmake -B build-host -DBUILD_HOST_TESTS=ON && cmake --build build-host
# -----------------------------------------------------------------------------
if(BUILD_HOST_TESTS)
    add_executable(host_tests
        common/contracts.c
        common/fixed.c
        foc/transforms.c
        foc/angle.c
        foc/current_pi.c
        foc/svpwm.c
        foc/tier2_checks.c
        foc/scope_log.c
        motion/motion.c
        motion/position_loop.c
        motion/velocity_loop.c
        motion/torque_loop.c
        motion/interpolator.c
        motion/feedforward.c
        motion/traj_limit.c
        motion/brake_seq.c
        test/host/test_angle_wrap.c
        test/host/test_current_loop.c
        test/host/test_velocity_accum.c
        test/host/test_tier2.c)
    target_include_directories(host_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR} foc motion config common hal)
    target_compile_definitions(host_tests PRIVATE HOST_BUILD=1)
    target_compile_options(host_tests PRIVATE
        -Wall -Wextra -Wdouble-promotion -Werror=double-promotion)
endif()