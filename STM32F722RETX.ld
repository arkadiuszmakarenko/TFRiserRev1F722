

/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = 0x20008000;    /* end of RAM */
/* Generate a link error if heap and stack don't fit into RAM */
Heap_Size = 8000;      /* required amount of heap  */
Stack_Size = 4000; /* required amount of stack */

/* Specify the memory areas */
MEMORY
{
FLASHTCM (rx)        : ORIGIN = 0x00200000, LENGTH = 512K
RAMITCM (xrw)        : ORIGIN = 0x00000000, LENGTH = 16K
RAMDTCM_RW_ISR (xrw) : ORIGIN = 0x20000000, LENGTH = 512
RAMDTCM_RW_ZI (xrw)  : ORIGIN = 0x20000200, LENGTH = 0x3E00
RAMDTCM_STACK        : ORIGIN = 0x20004000, LENGTH = 16K
RAMDTCM_HEAP         : ORIGIN = 0x20008000, LENGTH = 32K
SRAM		         : ORIGIN = 0x20010000, LENGTH = 192K
}

/* Define output sections */
SECTIONS
{
_siISR = LOADADDR(.isr_vector);
  /* The startup code goes first into FLASHTCM */
  .isr_vector :
  {
    . = ALIGN(4);
    _sISR = .;    
    KEEP(*(.isr_vector)) /* Startup code */
    . = ALIGN(4);
    _eISR = .; 
  } >RAMDTCM_RW_ISR AT> FLASHTCM

  /* used by the startup to initialize data */
  _siITCM = LOADADDR(.ITCMSection);

 .ITCMSection : {
 
   . = ALIGN(4);
   _sITCM = .;        /* create a global symbol at ITCM start */
   *main.o (.text .text*)
   *amiga.o (.text .text*)
   *dwt_delay.o (.text .text*)
   *(.itcmtext .itcmtext*)
   *arm_bitreversal2.o (.text .text*)
   *arm_cfft_f32.o (.text .text*)
   *arm_cfft_radix8_f32.o (.text .text*) 
   *arm_cmplx_mag_f32.o (.text .text*) 
   *arm_max_f32.o (.text .text*)
   *main.o (.rodata .rodata*)
   *amiga.o (.rodata .rodata*)
   *dwt_delay.o (.rodata .rodata*)
   *(.itcmrodata .itcmrodata*)
   *arm_common_tables.o (.rodata .rodata*) 
   *arm_const_structs.o (.rodata .rodata*) 
   . = ALIGN(4); 
   _eITCM = .;        /* create a global symbol at ITCM end */
   
 }>RAMITCM AT> FLASHTCM
 
  /* The program code and other data goes into FLASHTCM */
  .text :
  {
    . = ALIGN(4);
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;        /* define a global symbols at end of code */
  } >FLASHTCM

  /* Constant data goes into FLASHTCM */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    . = ALIGN(4);
  } >FLASHTCM

  .ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASHTCM
  .ARM : {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASHTCM

  .preinit_array     :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASHTCM
  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASHTCM
  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASHTCM

  /* used by the startup to initialize data */
  _sidata = LOADADDR(.data);

  /* Initialized data sections goes into RAMDTCM_RW_ZI, load LMA copy after code */
  .data : 
  {
    . = ALIGN(4);
    _sdata = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */

    . = ALIGN(4);
    _edata = .;        /* define a global symbol at data end */
  } >RAMDTCM_RW_ZI AT> FLASHTCM

  
  /* Uninitialized data section */
  . = ALIGN(4);
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss secion */
    _sbss = .;         /* define a global symbol at bss start */
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    _ebss = .;         /* define a global symbol at bss end */
    __bss_end__ = _ebss;
  } >RAMDTCM_RW_ZI

   /* User_heap section, used to check that there is enough DTCMRAM left */
  ._user_heap :
  {
    . = ALIGN(8);
    PROVIDE ( s_heap = . );
    . = . + Heap_Size;
    . = ALIGN(8);
    PROVIDE ( e_heap = . );
  } >RAMDTCM_HEAP
  
  /* User_stack section, used to check that there is enough DTCMRAM left */
 ._user_stack :
  {
    . = ALIGN(8);
    . = . + Stack_Size;
    . = ALIGN(8);
  } >RAMDTCM_STACK
  
  /*  RAM-DTCM partitioning:
              ____________________    __ 0x20000000  RW/ZI region TOP
             |               |    |
             |               |    |
             |    RW & ZI   \ /   |   -> 16kb
             |               v    |
             |____________________|   __ 0x20004000
             |                    |
             |                    |
             |               ^    |   -> 16kb
             |      Stack   / \   |
             |               |    |
             |_______________|____|   __ 0x20008000 STACK region TOP / Heap region TOP 
             |   |                | 
             |   |                | 
             |  \ /   Heap        |  -> 32kb
             |   v                | 
             |____________________|   __ 0x20010000
*/	 

  /* Remove information from the standard libraries */
  
    _sisram = LOADADDR(.sram);
  
    .sram :
  {
    . = ALIGN(4);
    _ssram = .;               
    *(.sram_data)           
    *(.sram_data*)          
    . = ALIGN(4);
    _esram = .;        

  } >SRAM AT> FLASHTCM
  
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}