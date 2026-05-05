#include <cstdint>

extern "C" int main();
extern "C" void __libc_init_array();

extern "C" uint32_t _estack;
extern "C" uint32_t _sidata;
extern "C" uint32_t _sdata;
extern "C" uint32_t _edata;
extern "C" uint32_t _sbss;
extern "C" uint32_t _ebss;

extern "C" void Default_Handler();
extern "C" [[noreturn]] void Reset_Handler();
extern "C" void SysTick_Handler()     __attribute__((weak, alias("Default_Handler")));
extern "C" void ADC1_IRQHandler()     __attribute__((weak, alias("Default_Handler")));
extern "C" void ADC2_IRQHandler()     __attribute__((weak, alias("Default_Handler")));
extern "C" void GPDMA1_Channel0_IRQHandler() __attribute__((weak, alias("Default_Handler")));
extern "C" void GPDMA1_Channel1_IRQHandler() __attribute__((weak, alias("Default_Handler")));
extern "C" void TIM5_IRQHandler()     __attribute__((weak, alias("Default_Handler")));

extern "C" void _init() {}
extern "C" void _fini() {}

extern "C" void Default_Handler() {
    while (true) { }
}

extern "C" [[noreturn]] void Reset_Handler() {
    uint32_t* src = &_sidata;
    for (uint32_t* dst = &_sdata; dst < &_edata; ++dst, ++src) {
        *dst = *src;
    }
    for (uint32_t* dst = &_sbss; dst < &_ebss; ++dst) {
        *dst = 0u;
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
    // IRQ64..IRQ69
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, ADC2_IRQHandler,
};

static_assert((sizeof(g_vector_table) / sizeof(g_vector_table[0])) >= (16u + 69u + 1u),
              "STM32H562 vector table must include ADC2 IRQ69");
