#include <cstdint>

extern "C" int main();
extern "C" void __libc_init_array();

extern "C" uint32_t _estack;
extern "C" uint32_t _sidata;
extern "C" uint32_t _sdata;
extern "C" uint32_t _edata;
extern "C" uint32_t _sbss;
extern "C" uint32_t _ebss;
/* FIX ERRATA FLASH: símbolos do vetor de IRQs em SRAM */
extern "C" uint32_t _svector_ram;
extern "C" uint32_t _evector_ram;

extern "C" void Default_Handler();
extern "C" [[noreturn]] void Reset_Handler();
extern "C" void SysTick_Handler()            noexcept __attribute__((weak, alias("Default_Handler")));
extern "C" void ADC1_IRQHandler()            noexcept __attribute__((weak, alias("Default_Handler")));
extern "C" void ADC2_IRQHandler()            noexcept __attribute__((weak, alias("Default_Handler")));
extern "C" void GPDMA1_Channel0_IRQHandler() noexcept __attribute__((weak, alias("Default_Handler")));
extern "C" void GPDMA1_Channel1_IRQHandler() noexcept __attribute__((weak, alias("Default_Handler")));
extern "C" void TIM5_IRQHandler()            noexcept __attribute__((weak, alias("Default_Handler")));
// Knock detection uses ADC1_IN6 (PA5) + software threshold — no COMP IRQ needed.

extern "C" void _init() {}
extern "C" void _fini() {}

/* FIX ERRATA FLASH: Handlers críticos marcados com .fastrun para execução em SRAM */
extern "C" void Default_Handler() __attribute__((section(".fastrun")));
extern "C" [[noreturn]] void Reset_Handler() __attribute__((section(".fastrun")));

extern "C" void Default_Handler() {
    // Trigger SYSRESETREQ so an unhandled IRQ resets the MCU instead of hanging.
    *reinterpret_cast<volatile uint32_t*>(0xE000ED0Cu) = 0x05FA0004u;
    while (true) { }
}

/* FIX ERRATA FLASH: Reset_Handler copia vetor de IRQs para SRAM antes de habilitar ISRs */
extern "C" [[noreturn]] void Reset_Handler() {
    /* FIX ERRATA: Copiar vetor de IRQs para SRAM ANTES de qualquer operação Flash */
    /* Isso previne o latency spike de 120µs descrito na errata do STM32H562 */
    const uint32_t* src = &_svector_ram;
    volatile uint32_t* dst = reinterpret_cast<volatile uint32_t*>(0xE000E000); /* VTOR base */
    const uint32_t* end = &_evector_ram;
    while (src < end) {
        *dst++ = *src++;
    }
    /* Agora configura VTOR para apontar para o vetor em SRAM */
    *(volatile uint32_t*)0xE000ED08 = reinterpret_cast<uint32_t>(&_svector_ram);
    
    /* Copia .data para RAM */
    src = &_sidata;
    for (uint32_t* d = &_sdata; d < &_edata; ++d, ++src) {
        *d = *src;
    }
    /* Zera .bss */
    for (uint32_t* d = &_sbss; d < &_ebss; ++d) {
        *d = 0u;
    }
    __libc_init_array();
    static_cast<void>(main());
    while (true) { }
}

using Handler = void (*)();

extern "C" __attribute__((section(".isr_vector"), used))
Handler const g_vector_table[] = {
    reinterpret_cast<Handler>(&_estack),
    Reset_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    Default_Handler,
    nullptr, nullptr, nullptr, nullptr,
    Default_Handler,
    Default_Handler,
    nullptr,
    Default_Handler,
    SysTick_Handler,
    // IRQ0..IRQ15
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    // IRQ16..IRQ31
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler, GPDMA1_Channel0_IRQHandler,
    GPDMA1_Channel1_IRQHandler, Default_Handler, Default_Handler, Default_Handler,
    // IRQ32..IRQ47
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, ADC1_IRQHandler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    // IRQ48..IRQ63
    TIM5_IRQHandler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    // IRQ64..IRQ69  (64=LPTIM1, 65=TIM8_BRK, 66=TIM8_UP, 67=TIM8_TRG, 68=TIM8_CC, 69=ADC2)
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, ADC2_IRQHandler,
};

static_assert((sizeof(g_vector_table) / sizeof(g_vector_table[0])) >= (16u + 69u + 1u),
              "STM32H562 vector table must include ADC2 IRQ69");
