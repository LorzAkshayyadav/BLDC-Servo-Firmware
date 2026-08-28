/*
******************************************************************************
**  File        : LinkerScript.ld
**  Target      : STM32H743VIHx  --  EtherCAT Servo Joint Drive
**
**  Derived from the STM32CubeMX template.  Every deviation is commented with
**  the reason, because a linker script is the one file where a wrong change
**  produces no build error and no runtime error -- just a peripheral that
**  transfers nothing.
*****************************************************************************
*/

ENTRY(Reset_Handler)

MEMORY
{
DTCMRAM (xrw)      : ORIGIN = 0x20000000, LENGTH = 128K
RAM (xrw)          : ORIGIN = 0x24000000, LENGTH = 512K
RAM_D2 (xrw)       : ORIGIN = 0x30000000, LENGTH = 288K
RAM_D3 (xrw)       : ORIGIN = 0x38000000, LENGTH = 64K
ITCMRAM (xrw)      : ORIGIN = 0x00000000, LENGTH = 64K
FLASH (rx)         : ORIGIN = 0x8000000,  LENGTH = 2048K
/* Backup SRAM: 4K, survives a power cycle, VBAT-domain backed.
 * NOT RAM_D3 above -- RAM_D3 is SRAM4, a normal D3-domain SRAM zeroed like
 * any other RAM on a cold boot.  Fault history and crash dumps need the true
 * backup domain (hal/hal_sections.h, BACKUP_DATA). */
BACKUP_SRAM (xrw)  : ORIGIN = 0x38800000, LENGTH = 4K
}

/* ORDER MATTERS HERE.
 * These three were previously written with _sstack using _Min_Stack_Size one
 * line BEFORE it was defined.  A forward reference in a linker script
 * expression evaluates to 0, so _sstack silently became equal to _estack --
 * a zero-length stack region.  Nothing uses _sstack today, which is why it
 * went unnoticed; an MPU stack-guard region or a stack-usage check would
 * have been configured over nothing. */
_Min_Heap_Size  = 0x200;    /* libc only; P5 forbids allocation above main() */
_Min_Stack_Size = 0x1000;   /* 4K.  See note at ._user_heap_stack.           */

_estack = ORIGIN(DTCMRAM) + LENGTH(DTCMRAM);
_sstack = _estack - _Min_Stack_Size;

SECTIONS
{
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)
    KEEP (*(.init))
    KEEP (*(.fini))
    . = ALIGN(4);
    _etext = .;
  } >FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  .ARM.extab (READONLY) : { . = ALIGN(4); *(.ARM.extab* .gnu.linkonce.armextab.*) . = ALIGN(4); } >FLASH
  .ARM (READONLY)       : { . = ALIGN(4); __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; . = ALIGN(4); } >FLASH

  .preinit_array (READONLY) : { . = ALIGN(4); PROVIDE_HIDDEN (__preinit_array_start = .); KEEP (*(.preinit_array*)) PROVIDE_HIDDEN (__preinit_array_end = .); . = ALIGN(4); } >FLASH
  .init_array (READONLY)    : { . = ALIGN(4); PROVIDE_HIDDEN (__init_array_start = .);    KEEP (*(SORT(.init_array.*))) KEEP (*(.init_array*)) PROVIDE_HIDDEN (__init_array_end = .); . = ALIGN(4); } >FLASH
  .fini_array (READONLY)    : { . = ALIGN(4); PROVIDE_HIDDEN (__fini_array_start = .);    KEEP (*(SORT(.fini_array.*))) KEEP (*(.fini_array*)) PROVIDE_HIDDEN (__fini_array_end = .); . = ALIGN(4); } >FLASH

  /* ---------------------------------------------------------------------
   * ITCM: control ISR, transforms, controllers, modulator, fault handlers
   * (hal_sections.h: ITCM_FUNC).  Zero wait state, never cached, so the ISR
   * duration histogram in arch section 15 shows near-zero spread.
   *
   * Separate from .text because .text executes in place from FLASH, while
   * this must be physically copied to ITCMRAM before the first call.  Hence
   * its own load/run address pair and the ONE copy loop that startup needs.
   * ------------------------------------------------------------------- */
  .itcm_text :
  {
    . = ALIGN(4);
    _sitcm = .;
    *(.itcm_text)
    *(.itcm_text*)
    . = ALIGN(4);
    _eitcm = .;
  } >ITCMRAM AT> FLASH
  _siitcm = LOADADDR(.itcm_text);

  _sidata = LOADADDR(.data);

  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    *(.RamFunc)
    *(.RamFunc*)

    /* DTCM initialised data and DTCM-resident constants (DTCM_DATA,
     * DTCM_CONST -- trig table, control state, integrators).  Folded in
     * here because both already target DTCMRAM AT> FLASH exactly like .data,
     * so they reuse the existing _sdata/_edata copy loop rather than needing
     * a second one.
     *
     * They keep their own input-section names, so the .map still shows
     * ".dtcm_data.g_foo" separately inside the .data output section and
     * Tools/section_check.py can still verify placement per symbol. */
    *(.dtcm_data)
    *(.dtcm_data*)
    *(.dtcm_rodata)
    *(.dtcm_rodata*)

    . = ALIGN(4);
  } >DTCMRAM AT> FLASH

  .tdata : ALIGN(4)
  {
    *(.tdata .tdata.* .gnu.linkonce.td.*)
    . = ALIGN(4);
    _edata = .;
    PROVIDE(__data_end = .);
    PROVIDE(__tdata_end = .);
  } >DTCMRAM AT> FLASH

  PROVIDE( __tdata_start = ADDR(.tdata) );
  PROVIDE( __tdata_size = __tdata_end - __tdata_start );
  PROVIDE( __data_start = ADDR(.data) );
  PROVIDE( __data_size = __data_end - __data_start );
  PROVIDE( __tdata_source = LOADADDR(.tdata) );
  PROVIDE( __tdata_source_end = LOADADDR(.tdata) + SIZEOF(.tdata) );
  PROVIDE( __tdata_source_size = __tdata_source_end - __tdata_source );
  PROVIDE( __data_source = LOADADDR(.data) );
  PROVIDE( __data_source_end = __tdata_source_end );
  PROVIDE( __data_source_size = __data_source_end - __data_source );

  .tbss (NOLOAD) : ALIGN(4)
  {
    _sbss = .;
    __bss_start__ = _sbss;
    *(.tbss .tbss.*)
    . = ALIGN(4);
    PROVIDE( __tbss_end = . );
  } >DTCMRAM

  PROVIDE( __tbss_start = ADDR(.tbss) );
  PROVIDE( __tbss_size = __tbss_end - __tbss_start );
  PROVIDE( __tbss_offset = ADDR(.tbss) - ADDR(.tdata) );
  PROVIDE( __tls_base = __tdata_start );
  PROVIDE( __tls_end = __tbss_end );
  PROVIDE( __tls_size = __tls_end - __tls_base );
  PROVIDE( __tls_align = MAX(ALIGNOF(.tdata), ALIGNOF(.tbss)) );
  PROVIDE( __tls_size_align = (__tls_size + __tls_align - 1) & ~(__tls_align - 1) );
  PROVIDE( __arm32_tls_tcb_offset = MAX(8, __tls_align) );
  PROVIDE( __arm64_tls_tcb_offset = MAX(16, __tls_align) );

  .bss (NOLOAD) : ALIGN(4)
  {
    *(.bss)
    *(.bss*)
    *(COMMON)

    /* DTCM-resident uninitialised state (DTCM_BSS), zeroed by the same loop
     * that zeroes .bss -- same region, same treatment, no second loop. */
    *(.dtcm_bss)
    *(.dtcm_bss*)

    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
    PROVIDE( __bss_end = .);
  } >DTCMRAM
  PROVIDE( __non_tls_bss_start = ADDR(.bss) );
  PROVIDE( __bss_start = __tbss_start );
  PROVIDE( __bss_size = __bss_end - __bss_start );

  /* Heap and stack both reserved in DTCM.  Stack raised from the CubeMX
   * default of 1K to 4K: the FOC ISR can be preempted by three priority-0
   * fault handlers, and the M7 FPU pushes an extra 18 words per nested frame
   * under lazy stacking.  1K leaves no room to be wrong about that, and a
   * stack overflow in DTCM runs straight into .bss with no fault. */
  ._user_heap_stack (NOLOAD) :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >DTCMRAM

  /* ---------------------------------------------------------------------
   * D2 SRAM: every DMA buffer (hal_sections.h: DMA_BUF).  Same domain as
   * DMA1/DMA2, so no bridge delay, and mapped non-cacheable by the MPU so a
   * stale cache line can never be read back after a peripheral writes here.
   *
   * ALIGN(32) at both ends is not cosmetic: the MPU works in 32-byte
   * granularity and a cache line straddling the boundary would make part of
   * an adjacent object non-cacheable too.
   *
   * NOLOAD, and the extent symbols below exist so hal_init_memory() can zero
   * the region at startup.  C guarantees uninitialised statics read as zero;
   * without that memset a DMA buffer contains boot garbage, which on the
   * very first period looks like a valid but wrong encoder frame.
   * ------------------------------------------------------------------- */
  .d2_sram (NOLOAD) :
  {
    . = ALIGN(32);
    _sd2_sram = .;
    *(.d2_sram)
    *(.d2_sram*)
    . = ALIGN(32);
    _ed2_sram = .;
  } >RAM_D2

  /* ---------------------------------------------------------------------
   * AXI SRAM: scope buffer, firmware staging, object dictionary (AXI_DATA).
   * Core + DMA, and CACHEABLE -- unlike D2.  Anything here that a peripheral
   * also writes needs explicit cache maintenance at every transfer site,
   * which is exactly the burden arch section 10 avoids by putting DMA
   * buffers in D2 instead.  Keep peripheral-written data out of this region.
   * ALIGN(32) for the same MPU/cache-line reason as above.
   * ------------------------------------------------------------------- */
  .axi_sram (NOLOAD) :
  {
    . = ALIGN(32);
    _saxi_sram = .;
    *(.axi_sram)
    *(.axi_sram*)
    . = ALIGN(32);
    _eaxi_sram = .;
  } >RAM

  /* ---------------------------------------------------------------------
   * Backup SRAM: fault history, crash dump, calibration validity
   * (BACKUP_DATA).  NEVER zeroed at startup -- it sits outside every
   * zero-fill loop in Reset_Handler, and that is what makes the guarantee
   * hold.  Merge this into .bss and backup SRAM starts being wiped on every
   * boot, which loses the fault record from precisely the power cycle a
   * field failure causes.
   *
   * The extent symbols are for hal_persist.c to CRC the region; it must not
   * hard-code the address.
   * ------------------------------------------------------------------- */
  .backup_sram (NOLOAD) :
  {
    . = ALIGN(4);
    _sbackup_sram = .;
    *(.backup_sram)
    *(.backup_sram*)
    . = ALIGN(4);
    _ebackup_sram = .;
  } >BACKUP_SRAM

  /* libgcc.a is NOT discarded: the compiler emits calls to its helpers for
   * 64-bit division and some conversions without any source file naming
   * them.
   *
   * libc.a stays discarded, but be aware GCC also emits memcpy and memset
   * for struct assignment and array initialisation.  If that link error
   * appears, the fix is to keep libc.a and rely on -ffreestanding plus the
   * layer check to catch genuine libc use, NOT to write a local memcpy.
   *
   * libm.a discarded on purpose: the control path is fixed-point
   * (common/fixed.h), so a stray sinf() should be a link error rather than a
   * silent flash-resident call -- see the trap in fixed.c about generating
   * the trig table at startup instead of at compile time. */
  /DISCARD/ :
  {
    libc.a:* ( * )
    libm.a:* ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}

/* ---------------------------------------------------------------------------
 * Link-time checks.
 *
 * Region overflow is already a linker error, so these cover only what ld
 * cannot see for itself.
 * ------------------------------------------------------------------------- */

/* The MPU non-cacheable region for D2 must start on a 32-byte boundary. */
ASSERT((_sd2_sram & 31) == 0, "D2 SRAM section is not 32-byte aligned")
ASSERT((_saxi_sram & 31) == 0, "AXI SRAM section is not 32-byte aligned")

/* Backup SRAM is 4K and holds the fault history ring.  Overflow here would
 * be caught by ld, but a near-full region means the ring has been sized
 * without leaving room for the crash dump. */
ASSERT(_ebackup_sram - _sbackup_sram <= 3K,
       "backup SRAM over 3K: leave room for the crash dump")
