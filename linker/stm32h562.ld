ENTRY(Reset_Handler)

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 1024K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 512K
}

_estack = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{
    /* FIX ERRATA FLASH: Vetor de IRQs e handlers críticos em SRAM (.fastrun) */
    /* Previne latency spike de 120µs durante primeira operação Flash após power-on */
    .isr_vector :
    {
        KEEP(*(.isr_vector))
    } > RAM AT > FLASH
    _svector_ram = LOADADDR(.isr_vector);
    _evector_ram = LOADADDR(.isr_vector) + SIZEOF(.isr_vector);

    /* Seção .fastrun para código crítico (ISRs, handlers Flash) */
    .fastrun :
    {
        _sfastrun = .;
        *(.fastrun*)
        *(.fastrun.*)
        . = ALIGN(4);
        _efastrun = .;
    } > RAM AT > FLASH
    _sfastrun_load = LOADADDR(.fastrun);

    .text :
    {
        *(.text*)
        *(.rodata*)
        KEEP(*(.init))
        KEEP(*(.fini))
        . = ALIGN(4);
    } > FLASH

    .ARM.exidx :
    {
        *(.ARM.exidx*)
    } > FLASH

    _sidata = LOADADDR(.data);
    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } > RAM AT > FLASH

    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > RAM

    .stack (NOLOAD) :
    {
        . = ALIGN(8);
        _sstack = .;
        . = . + 0x2000;
        . = ALIGN(8);
        _estack_guard = .;
    } > RAM

    /DISCARD/ :
    {
        *(.comment*)
        *(.note*)
    }
}
