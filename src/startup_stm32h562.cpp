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
extern "C" void SysTick_Handler()            noexcept __attribute__((weak, alias("Default_Handler")));
extern "C" void ADC1_IRQHandler()            noexcept __attribute__((weak, alias("Default_Handler")));
extern "C" void ADC2_IRQHandler()            noexcept __attribute__((weak, alias("Default_Handler")));
extern "C" void GPDMA1_Channel0_IRQHandler() noexcept __attribute__((weak, alias("Default_Handler")));
extern "C" void GPDMA1_Channel1_IRQHandler() noexcept __attribute__((weak, alias("Default_Handler")));
extern "C" void TIM5_IRQHandler()            noexcept __attribute__((weak, alias("Default_Handler")));
extern "C" void USB_IRQHandler()             noexcept __attribute__((weak, alias("Default_Handler")));
extern "C" void EXTI5_9_IRQHandler()        noexcept __attribute__((weak, alias("Default_Handler")));

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

extern "C" [[noreturn]] void Reset_Handler() {
    /* 1. Desactivar IRQs e corrigir VTOR — DFU jump pode deixar VTOR errado */
    __asm__ volatile ("cpsid i");
    *reinterpret_cast<volatile uint32_t*>(0xE000ED08u) = 0x08000000u;  // VTOR = Flash

    /* 2. Kick WWDG imediatamente (hardware WWDG activo desde reset) */
    *reinterpret_cast<volatile uint32_t*>(0x40002C00u) = 0x7Fu;  // WWDG_CR refresh

    /* 3. Forcar SYSCLK = HSI16
     * OBRIGATORIO: activar HSI16 primeiro (DFU pode ter desligado para usar PLL/HSE)
     * RCC_CR @ 0x44020C00: HSION=bit0, HSIRDY=bit1
     * RCC_CFGR1 @ 0x44020C1C: SW[1:0]=bits[1:0], SWS[1:0]=bits[4:3] */
    {
        volatile uint32_t* rcc_cr  = reinterpret_cast<volatile uint32_t*>(0x44020C00u);
        volatile uint32_t* cfgr1   = reinterpret_cast<volatile uint32_t*>(0x44020C1Cu);

        // 3a. Ligar HSI16 e aguardar pronto
        *rcc_cr |= (1u << 0u);  // HSION = 1
        for (volatile uint32_t t = 20000u; t > 0u; --t) {
            if (*rcc_cr & (1u << 1u)) break;  // aguardar HSIRDY
        }

        // 3b. Mudar SYSCLK para HSI16 e aguardar
        *cfgr1 = (*cfgr1 & ~0x3u);              // SW[1:0] = 00 (HSI16)
        for (volatile uint32_t t = 10000u; t > 0u; --t) {
            if ((*cfgr1 & (3u << 3)) == 0u) break; // aguardar SWS=00
        }
        // Desligar PLL1 (liberta energia, confirma HSI16 exclusivo)
        // NAO alterar Flash latency aqui: VOS3 (default reset) exige >=1WS a 16MHz
        *reinterpret_cast<volatile uint32_t*>(0x44020C00u) &= ~(1u << 24u); // PLL1ON=0

        // Reset prescalers AHB/APB1/APB2 para 1 (DFU pode ter configurado divisores)
        // CFGR2 @ 0x44020C20: HPRE[3:0]=0, PPRE1[6:4]=0, PPRE2[10:8]=0 -> todos /1
        volatile uint32_t* cfgr2 = reinterpret_cast<volatile uint32_t*>(0x44020C20u);
        *cfgr2 &= ~0x77Fu;  // limpar HPRE+PPRE1+PPRE2

        // USART1 clock = HSI16 directamente (independente de prescalers)
        // CCIPR1 @ 0x44020CD8: USART1SEL[2:0] = 011 = HSI16
        volatile uint32_t* ccipr1 = reinterpret_cast<volatile uint32_t*>(0x44020CD8u);
        *ccipr1 = (*ccipr1 & ~0x7u) | 0x3u;
    }

    /* 3. Copia .data para RAM */
    const uint32_t* src = &_sidata;
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
    Default_Handler, Default_Handler, Default_Handler, EXTI5_9_IRQHandler, // IRQ23=EXTI[9:5]
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
    // IRQ70..IRQ74  (73=TIM17, 74=USB DRD FS)
    // USB_DRD_FS_IRQn=74 confirmed from stm32h562xx.h
    Default_Handler, Default_Handler, Default_Handler, Default_Handler, USB_IRQHandler,
};

static_assert((sizeof(g_vector_table) / sizeof(g_vector_table[0])) >= (16u + 74u + 1u),
              "STM32H562 vector table must include USB IRQ74");
