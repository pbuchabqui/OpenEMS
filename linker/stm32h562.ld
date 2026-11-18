ENTRY(Reset_Handler)

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 1024K
    /* O H562 tem 640 KB de SRAM contígua: SRAM1 256K @0x20000000 + SRAM2 64K @0x20040000 + SRAM3 320K @0x20050000
       (stm32h562xx.h). O projeto usa os primeiros 512K. Havia uma região SRAM3 em 0x20060000 que sobrepunha a RAM
       e não coincidia com a SRAM3 real; nenhuma seção a usava e foi removida. */
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 512K
}

_estack = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{
    /* Vector table e handlers críticos em Flash */
    .isr_vector :
    {
        KEEP(*(.isr_vector))
    } > FLASH

    .fastrun :
    {
        *(.fastrun*)
        *(.fastrun.*)
        . = ALIGN(4);
    } > FLASH

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
