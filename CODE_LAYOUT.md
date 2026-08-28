# Code Layout

```
BLDC Fimware/
├── CMakeLists.txt
├── CMakePresets.json
├── CODE_LAYOUT.md                  this file
│
├── App/                            L4 — main loop and SYNC0 context
│   ├── main.c                      init order, then the main loop
│   ├── drive_profile.c/.h          CiA 402 state machine (supervisory, 1 kHz)
│   ├── fieldbus_app.c/.h           EtherCAT slave stack glue
│   ├── object_dict.c/.h            CoE object dictionary
│   ├── param_service.c/.h          parameter read/write, publishes g_params
│   ├── clock_discipline.c/.h       SYNC0 phase loop → hal_carrier_trim()
│   ├── supervisor.c/.h             1 kHz task: cross-checks, IWDG refresh
│   ├── scope_readout.c/.h          drains the in-RAM scope over CoE
│   └── fw_update.c/.h
│
├── Motion/                         L3 — 4 kHz, pended.  NO REGISTERS.
│   ├── motion.c/.h                 task entry, runs the three loops in order
│   ├── position_loop.c/.h          on the LOAD encoder
│   ├── velocity_loop.c/.h          on the MOTOR encoder, accumulated dt
│   ├── torque_loop.c/.h            on the RS-485 sensor
│   ├── interpolator.c/.h           network cycle → 4 kHz, §9.3
│   ├── feedforward.c/.h
│   ├── traj_limit.c/.h
│   └── brake_seq.c/.h              engage-before-torque-removal sequencing
│
├── FOC/                            L2 — one ISR, ITCM resident.  NO REGISTERS.
│   ├── foc_isr.c                   THE control ISR.  Ordering is load-bearing.
│   ├── transforms.c/.h             Clarke (two-current), Park, inverse Park
│   ├── current_pi.c/.h             with anti-windup
│   ├── svpwm.c/.h                  modulator, dead-time comp, duty clamp report
│   ├── angle.c/.h                  phase accumulator, trig table, advance term
│   ├── tier2_checks.c/.h           sum, filtered overcurrent, bus, speed, thermal
│   └── scope_log.c/.h              8 channels at 16 kHz, circular, pre-trigger
│
├── Config/                         parameters only, no logic
│   ├── motor_params.h              per commissioned unit — tuned values land here
│   ├── control_params.h            gains, limits, rates, angle advance, trips
│   ├── board_limits.h              duty clamp, dead time, thresholds
│   └── build_config.h              feature selection, guards on invalid combos
│
├── common/
│   ├── contracts.h                 §12 table 9, single-writer discipline
│   ├── fixed.h                     q15/q31 helpers, no float anywhere
│   └── fault_codes.h
│
├── HAL/                            L1 — ALL device and board specifics
│   ├── hal.h                       THE public surface.  See the rule above.
│   ├── hal_sections.h              memory placement macros
│   ├── hal_pwm.c                   carrier, trigger placement, dead time,
│   │                               break, safe state, duty clamp
│   ├── hal_sequencer.c             TIM2/TIM4/TIM5/TIM6, clocks, monitor
│   ├── hal_adc.c                   injected + regular groups, watchdog limits,
│   │                               INA241 scaling and low-side sign inversion
│   ├── hal_encoder.c               iC-MBE register config, opcode 0x09 frames,
│   │                               DMA, buffers, status byte, PDVALID reset
│   ├── hal_fieldbus.c              LAN9253 map, single-owner SM, SYNC0 capture
│   ├── hal_torque_sensor.c         RS-485 protocol, DE turnaround, validity
│   ├── hal_safety.c                STO assert and monitor, brake, emergency kill
│   ├── hal_time.c                  TIM5 reads, carrier position
│   ├── hal_persist.c               backup SRAM, integrity, fault history
│   └── isr_vectors.c               hand-written handlers.  NOT HAL dispatchers.
│
├── platform/                       generated CubeMX output lives here, untouched
│   ├── BLDC_18A_STM32H743.ioc      CubeMX project — Generate Code writes here
│   ├── .mxproject
│   ├── Core/
│   ├── Drivers/
│   ├── cmake/
│   │   ├── gcc-arm-none-eabi.cmake
│   │   ├── starm-clang.cmake
│   │   └── stm32cubemx/CMakeLists.txt
│   ├── startup_stm32h743xx.s
│   └── STM32H743xx_FLASH.ld
│
├── test/host/                      stub L1, run L2 and L3 on a PC
│   ├── hal_stub.c                  implements every verb in hal.h
│   ├── plant_model.c               two-inertia motor + gearbox + encoder quant.
│   ├── test_current_loop.c
│   ├── test_angle_wrap.c
│   ├── test_velocity_accum.c
│   └── test_tier2.c
│
└── Tools/
    ├── layer_check.py              fail if peripheral symbols escape HAL/
    └── section_check.py            fail if memory placement is wrong
```

## Notes

- The tree is flat at the project root — no `src/` wrapper. `App`, `Motion`,
  `FOC`, `Config`, `common`, and `HAL` are the module directories;
  `torque/` from the original plan is named `FOC` here.
- `platform/` is the real, single copy of everything STM32CubeMX owns —
  `BLDC_18A_STM32H743.ioc`, `Core/`, `Drivers/`, `cmake/`, the startup file,
  and the linker script all live there now (moved from the project root).
  CubeMX generates `Core/`, `Drivers/`, `cmake/`, the startup file, and the
  `.ld` file as siblings of wherever the `.ioc` sits, so opening
  `platform/BLDC_18A_STM32H743.ioc` in CubeMX and hitting "Generate Code"
  writes back into `platform/` from now on. Nothing under `platform/` is
  ever hand-edited except the three build files below.
- Three build files carry a root-relative path into `platform/` and needed
  updating for the move: root `CMakeLists.txt`
  (`add_subdirectory(platform/cmake/stm32cubemx)`), `CMakePresets.json`
  (`toolchainFile` → `platform/cmake/gcc-arm-none-eabi.cmake`), and both
  `platform/cmake/gcc-arm-none-eabi.cmake` / `starm-clang.cmake` (linker
  script path → `${CMAKE_SOURCE_DIR}/platform/STM32H743xx_FLASH.ld`).
  `platform/cmake/stm32cubemx/CMakeLists.txt` needed no edit — its
  `../../Core/Inc`-style paths are relative to its own folder and resolve
  correctly at the new nesting depth; it's CubeMX-regenerated anyway.
- `CMakeLists.txt` sources/includes `App/`, `Motion/`, `FOC/`, `HAL/`
  (`*.c` glob) and adds `App/`, `Motion/`, `FOC/`, `Config/`, `common/`,
  `HAL/` to the include path.
- Still open: `App/main.c` and `Core/Src/main.c` both will want to own
  `int main(void)` once `App/main.c` gets real content — that collision is
  unresolved.
- Every file under `App/`, `Motion/`, `FOC/`, `Config/`, `common/`, `HAL/`,
  and `test/host/` is currently a skeleton: include guards and a one-line
  header comment only, no declarations or logic yet.
