// =============================================================================
// OpenEMS — src/main_stm32.cpp
// STM32H562RGT6 (ARM Cortex-M33 @ 250 MHz)
//
// Entry point STM32H562 do firmware.
//
// Configuracao principal:
//   - PLL 250 MHz via system_stm32_init()
//   - millis() provido por SysTick_Handler em system.cpp
//   - iwdg_kick() alimenta o watchdog IWDG
//   - SysTick fornece millis()/micros()
//   - NVIC setup usa IRQs do STM32H562 (TIM5 para CKP; TIM2/TIM1 são OC por hardware)
//   - SysTick e IWDG já inicializados em system_stm32_init()
//   - nvm_flush_adaptive_maps() no slot 500ms (Flash Bank2)
// =============================================================================

#if defined(EMS_HOST_TEST)

int main() { return 0; }

#elif defined(TARGET_STM32H562)  // ── target STM32H562 ─────────────────────

#include <cstdint>
#include <cstring>

#include "engine/math_utils.h"

#include "hal/stm32h562/system.h"
#include "hal/stm32h562/regs.h"
#include "hal/stm32h562/usb_cdc.h"
#include "hal/uart.h"

#include "app/can_stack.h"
#include "app/can_rx_map.h"
#include "app/nvm_boot.h"
#include "app/ui_protocol.h"
#include "drv/ckp.h"
#include "drv/sensors.h"
#include "engine/auxiliaries.h"
#include "engine/calibration.h"
#include "engine/constants.h"
#include "engine/cut_reason.h"
#include "engine/vehicle_inputs.h"
#include "engine/ecu_sched.h"
#include "engine/enc_cyl_setpoints.h"
#include "engine/engine_config.h"
#include "engine/etb_control.h"
#include "engine/etb_autocal.h"
#include "hal/etb_driver.h"
#include "engine/fuel_calc.h"
#include "engine/ign_calc.h"
#include "engine/knock.h"
#include "engine/map_estimator.h"
#include "engine/output_test.h"
#include "engine/diagnostic_manager.h"
#include "engine/misfire_encoder.h"
#include "engine/quick_crank.h"
#include "engine/torque_manager.h"
#include "engine/transient_fuel.h"
#include "engine/spark_skip.h"
#include "engine/xtau_autocalib.h"
#include "engine/ewg_control.h"
#include "engine/loop_2ms_fuel_ign.h"
#include "hal/adc.h"
#include "hal/can.h"
#include "hal/flash.h"
#include "hal/tle8888.h"
#include "hal/out_pins.h"
#include "hal/flex_fuel.h"
#include "hal/timer.h"
#include "hal/mt6835.h"
#include "hal/critical_section.h"
#include "drv/encoder_sync.h"

// =============================================================================
// Estado de background (do firmware)
// =============================================================================

static constexpr uint16_t kCalibPageBytes = 512u;
alignas(4) static uint8_t g_calib_page0[kCalibPageBytes];
static bool                g_calib_dirty  = false;

// g_datalog_us: no STM32 usa micros() de system.cpp em vez de SysTick
// Mantemos a variável para quadro CAN 0x400
volatile uint32_t g_datalog_us = 0u;
volatile uint32_t g_flash_write_faults = 0u; // FIX: fault counter para falhas de escrita NVM


// DIAG rev-limit: bordas de subida + rpm no trip (dump 'D'). Escritos em
// loop_2ms_fuel_ign.cpp.
uint32_t g_dbg_rev_limit_trips = 0u;
uint32_t g_dbg_rev_limit_rpm_x10 = 0u;
uint32_t g_dbg_rev_limit_rpm_max = 0u;
// Barometric correction: amostrar MAP quando motor parado por >300ms após key-on
static uint32_t g_baro_stopped_since_ms = 0u;
static bool     g_baro_sampled          = false;
static uint32_t g_loop2ms_last_us = 0u;
static uint32_t g_loop2ms_max_us = 0u;

static constexpr uint32_t kCalibSaveMinIntervalMs = 300000u;

// =============================================================================
// Estado ETB e Torque Manager
// =============================================================================

static bool g_etb_initialized = false;

// =============================================================================
// Utilitários
// =============================================================================

static inline bool elapsed(uint32_t now, uint32_t last, uint32_t period) noexcept {
    return static_cast<uint32_t>(now - last) >= period;
}

static inline uint16_t build_status_bits(const ems::drv::CkpSnapshot& snap,
                                           const ems::drv::SensorData& sensors) noexcept {
    uint16_t status = 0u;
    if (snap.state == ems::drv::SyncState::FULL_SYNC) {
        status |= ems::app::STATUS_SYNC_FULL;
    }
    if (snap.phase_A) {
        status |= ems::app::STATUS_PHASE_A;
    }
    if (sensors.fault_bits != 0u) {
        status |= ems::app::STATUS_SENSOR_FAULT;
    }
    if (g_rev_limit_active) {
        status |= ems::app::STATUS_REV_LIMIT;
    }
    if (ems::engine::torque_manager_get_launch_active() != 0u) {
        status |= ems::app::STATUS_LAUNCH_ACTIVE;
    }
    if (ems::engine::torque_manager_get_tc_reduction() > 0u) {
        status |= ems::app::STATUS_TC_ACTIVE;
    }
    return status;
}

using ems::engine::clamp_u16;
using ems::engine::clamp_i16;

static inline void comms_pump() noexcept {
    // Shuttle UART+USB↔protocolo a 2 ms. Ambos os transportes são servidos no
    // MESMO ritmo curto: o handshake de conexão do TunerStudio envia comandos
    // PLAIN (F, Q, ...) e, se a resposta demora, ele estoura o timeout curto e
    // envia o próximo comando antes de ler a resposta anterior — as duas
    // respostas saem então coladas ("001OpenEMS_v1.2") e o TS rejeita com
    // "Unsupported Controller Firmware". Servir o USB a 20 ms (como antes)
    // garantia esse gluing; a 2 ms cada comando é respondido a tempo, como nas
    // implementações de referência (Speeduino/rusEFI: request→resposta→flush
    // sub-ms). Nada aqui bloqueia (drena TX só com espaço no FIFO/rings).
    ems::hal::uart0_poll_rx(32u);
    {
        uint8_t b = 0u;
        while (ems::hal::uart0_rx_pop(b)) {
            ems::app::ui_rx_byte(b);
        }
    }
    ems::hal::usb_cdc_poll();
    if (ems::hal::usb_cdc_dtr()) {
        uint8_t rx_buf[64] = {};
        const uint16_t rx_n = ems::hal::usb_cdc_read_bytes(rx_buf, 64u);
        for (uint16_t i = 0u; i < rx_n; ++i) {
            ems::app::ui_rx_byte(rx_buf[i]);
        }
    }
    ems::app::ui_process();

    // Drena ui_tx → UART + USB (espelhado), limitado pelo MENOR espaço livre
    // entre os dois rings de destino — usb_cdc_send_bytes() descarta bytes
    // em silêncio se o seu próprio ring encher, então o orçamento tem de
    // respeitar ambos, não só o do UART, para nunca perder bytes de um frame.
    uint16_t budget = ems::hal::uart0_tx_free();
    const uint16_t usb_free = ems::hal::usb_cdc_tx_free();
    if (usb_free < budget) { budget = usb_free; }
    if (budget > 32u) { budget = 32u; }
    uint8_t tx_buf[32] = {};
    uint16_t tx_n = 0u;
    while (tx_n < budget && ems::app::ui_tx_pop(tx_buf[tx_n])) { ++tx_n; }
    if (tx_n != 0u) {
        for (uint16_t i = 0u; i < tx_n; ++i) {
            ems::hal::uart0_tx_push(tx_buf[i]);
        }
        ems::hal::usb_cdc_send_bytes(tx_buf, tx_n);
    }
    ems::hal::uart0_tx_poll_nb(16u);
}

// =============================================================================
// Inicialização — sequência idêntica ao main.cpp
// =============================================================================

// Modo diagnóstico: descomente para teste isolado de USB CDC (clock + usb_cdc_init + echo).
// Em produção fica DESATIVADO → usa o openems_init() real (ECU completa + usb_cdc_init).
// #define MINIMAL_BOOT 1  // uncomment for USB CDC echo-only diagnostic mode

#ifdef MINIMAL_BOOT
static void openems_init() noexcept {
    // ABSOLUTE MINIMUM TEST: kick WWDG + DPPU=1
    // Objectivo: confirmar que firmware executa (dmesg USB) ou detectar WWDG loop

    // 0. Kick WWDG imediatamente (hardware WWDG activo!)
    // WWDG_CR @ APB1_BASE + 0x2C00 = 0x40002C00
    // bit 7 (WDGA) e bits[6:0] T[6:0] = 0x7F: refresh com T=0x7F
    STM32_REG32(0x40002C00) = 0x7Fu;  // WWDG kick

    // 1. GPIOA clock — dummy read DEVE ser do GPIOA (nao do AHB2ENR!)
    STM32_REG32(0x44020C8C) |= (1u << 0);  // AHB2ENR GPIOAEN
    (void)STM32_REG32(0x42020000);  // dummy read GPIOA_MODER — garante clock propagado

    // 2. PA9 = OUTPUT para teste GPIO (MODER[19:18]=01)
    //    PA9 HIGH = 3.3V visivel no adaptador serie RXD
    STM32_REG32(0x42020000) = (STM32_REG32(0x42020000) & ~(3u<<18)) | (1u<<18);
    STM32_REG32(0x42020018) = (1u << 9);  // BSRR: set PA9 HIGH

    // 3. UART init PA9=AF7 USART1
    STM32_REG32(0x42020000) = (STM32_REG32(0x42020000) & ~(3u<<18)) | (2u<<18);
    STM32_REG32(0x42020024) = (STM32_REG32(0x42020024) & ~(0xFu<<4)) | (7u<<4);
    STM32_REG32(0x44020CA4) |= (1u << 14);  // APB2ENR USART1EN
    (void)STM32_REG32(0x40013800);  // dummy read USART1 — garante clock propagado

    auto uart_putc = [](char c) noexcept {
        for (volatile uint32_t t = 5000u; t > 0; --t) {
            STM32_REG32(0x40002C00) = 0x7Fu;  // WWDG kick
            if (STM32_REG32(0x40013800 + 0x1C) & (1u<<7)) break;
        }
        STM32_REG32(0x40013800 + 0x28) = static_cast<uint8_t>(c);
    };
    auto uart_puts = [&uart_putc](const char* s) noexcept {
        while (*s) uart_putc(*s++);
    };

    // SYSCLK = 64 MHz (confirmado por clock sweep: DFU exit deixa PLL @ 64 MHz)
    // PCLK2 = 64 MHz, BRR = 64e6/115200 = 556 = 0x22C
    STM32_REG32(0x40013800 + 0x0C) = 0x22Cu;
    STM32_REG32(0x40013800 + 0x00) = (1u<<3)|(1u<<0);   // CR1 TE+UE
    for (volatile uint32_t i = 0; i < 1000u; ++i) { __asm__("nop"); }

    uart_puts("\r\n=== OpenEMS BOOT64 ===\r\n");

    // Helper: print hex (4 digits)
    auto uart_hex16 = [&uart_putc](uint32_t v) noexcept {
        const char* hex = "0123456789ABCDEF";
        uart_putc(hex[(v >> 12) & 0xF]);
        uart_putc(hex[(v >> 8)  & 0xF]);
        uart_putc(hex[(v >> 4)  & 0xF]);
        uart_putc(hex[v & 0xF]);
    };
    auto uart_hex32 = [&uart_hex16](uint32_t v) noexcept {
        uart_hex16(v >> 16);
        uart_hex16(v);
    };
    auto delay_ms_64 = [](uint32_t ms) noexcept {
        // 64 MHz: ~6400 NOPs por ms (com pipeline ~2 ciclos/loop)
        for (uint32_t m = 0; m < ms; ++m) {
            STM32_REG32(0x40002C00) = 0x7Fu;
            for (volatile uint32_t i = 0; i < 6400u; ++i) { __asm__("nop"); }
        }
    };

    // Dump state inicial
    uart_puts("RCC_CR=");      uart_hex32(STM32_REG32(0x44020C00)); uart_puts("\r\n");
    uart_puts("RCC_CFGR1=");   uart_hex32(STM32_REG32(0x44020C1C)); uart_puts("\r\n");
    uart_puts("PWR_VOSCR=");   uart_hex32(STM32_REG32(0x44020810)); uart_puts("\r\n");
    uart_puts("PWR_USBSCR=");  uart_hex32(STM32_REG32(0x44020838)); uart_puts("\r\n");

    // 4. VDDUSB enable + delay longo (>= 1 ms)
    STM32_REG32(0x44020838) |= (1u << 25);
    delay_ms_64(5);
    uart_puts("VDDUSB_OK PWR_USBSCR="); uart_hex32(STM32_REG32(0x44020838)); uart_puts("\r\n");

    // 5. HSI48 (USB clock) — confirmar HSI48RDY
    STM32_REG32(0x44020C00) |= (1u << 12);
    {
        bool ready = false;
        for (uint32_t n = 0; n < 200000u; ++n) {
            STM32_REG32(0x40002C00) = 0x7Fu;
            if (STM32_REG32(0x44020C00) & (1u<<13)) { ready = true; break; }
        }
        uart_puts(ready ? "HSI48_RDY " : "HSI48_TIMEOUT ");
        uart_puts("RCC_CR="); uart_hex32(STM32_REG32(0x44020C00)); uart_puts("\r\n");
    }

    // 6. Inicialização USB CDC completa pelo driver REAL (ems::hal::usb_cdc_init):
    //    seleciona USBSEL=HSI48 (0b11, NÃO 00=NOCLOCK), configura PA11/PA12 AF10,
    //    power-up do transceiver, BDTable + descritores, NVIC e DPPU. Substitui os
    //    pokes crus anteriores (que usavam USBSEL=00 e nunca serviam descritores).
    STM32_REG32(0x40002C00) = 0x7Fu;  // kick WWDG antes da init
    ems::hal::usb_cdc_init();
    STM32_REG32(0x40002C00) = 0x7Fu;  // kick WWDG depois da init

    // CRÍTICO: o Reset_Handler faz cpsid i e nunca reabilita. O firmware completo
    // reabilita por acidente no 1º cpsie de uma seção crítica do openems_init; o caminho
    // MINIMAL não chama nenhuma → sem isto, PRIMASK=1 e a ISR do USB NUNCA dispara.
    __asm__ volatile("cpsie i" ::: "memory");
    uart_puts("usb_cdc_init OK CCIPR4="); uart_hex32(STM32_REG32(0x44020CE4));
    uart_puts(" CNTR=");  uart_hex32(STM32_REG32(0x40016040));
    uart_puts(" ISTR=");  uart_hex32(STM32_REG32(0x40016044));
    uart_puts(" BCDR=");  uart_hex32(STM32_REG32(0x40016058)); uart_puts("\r\n");

    delay_ms_64(100);  // dar tempo ao host de detectar/enumerar
    uart_puts("100ms ISTR="); uart_hex32(STM32_REG32(0x40016044));
    uart_puts(" DADDR=");      uart_hex32(STM32_REG32(0x4001604C)); uart_puts("\r\n");

    // Configura PB2 (LED da placa WeAct) como saída para o "ladder" de diagnóstico:
    // o LED pisca N vezes = maior estágio de enumeração alcançado (1..6), pausa, repete.
    STM32_REG32(0x44020C8C) |= (1u << 1);  // RCC AHB2ENR1 GPIOBEN
    (void)STM32_REG32(0x42020400);          // dummy read p/ propagar clock
    STM32_REG32(0x42020400) = (STM32_REG32(0x42020400) & ~(3u << 4)) | (1u << 4);  // PB2 output

    // AF brute-force test: cycle AF0-AF15 on PC6 with TIM3 FORCE_ACTIVE
    // Enable GPIOC + TIM3 clocks
    STM32_REG32(0x44020C8C) |= (1u << 2);   // GPIOCEN
    STM32_REG32(0x44020C9C) |= (1u << 1);   // TIM3EN (APB1LENR bit 1)
    (void)STM32_REG32(0x42020800);           // dummy read GPIOC

    // TIM3: PSC=24 (10MHz), FORCE_ACTIVE on CH1, CCER CC1E=1, CEN=1
    STM32_REG32(0x40000400 + 0x28) = 24u;   // TIM3_PSC
    STM32_REG32(0x40000400 + 0x18) = 0x50u; // TIM3_CCMR1 = FORCE_ACTIVE CH1
    STM32_REG32(0x40000400 + 0x20) = 1u;    // TIM3_CCER = CC1E
    STM32_REG32(0x40000400 + 0x00) = 1u;    // TIM3_CR1 = CEN

    auto delay_2s = [](void) {
        for (volatile uint32_t d = 0; d < 8000000u; ++d) {
            if ((d & 0xFFFu) == 0) STM32_REG32(0x40002C00) = 0x7Fu;
        }
    };

    auto blink_n = [](uint8_t n) {
        for (uint8_t i = 0; i < n; ++i) {
            STM32_REG32(0x42020414) |= (1u << 2);
            for (volatile uint32_t d = 0; d < 500000u; ++d) {
                if ((d & 0xFFFu) == 0) STM32_REG32(0x40002C00) = 0x7Fu;
            }
            STM32_REG32(0x42020414) &= ~(1u << 2);
            for (volatile uint32_t d = 0; d < 500000u; ++d) {
                if ((d & 0xFFFu) == 0) STM32_REG32(0x40002C00) = 0x7Fu;
            }
        }
    };

    while (true) {
        for (uint8_t af = 0u; af < 16u; ++af) {
            // Set PC6 to AF mode (MODER=10) with AFRL[27:24] = af
            uint32_t afrl = STM32_REG32(0x42020820);  // GPIOC_AFRL
            afrl = (afrl & ~(0xFu << 24u)) | ((uint32_t)af << 24u);
            STM32_REG32(0x42020820) = afrl;
            STM32_REG32(0x42020800) = (STM32_REG32(0x42020800) & ~(3u << 12u)) | (2u << 12u);

            blink_n(af + 1u);  // blink AF number (1-16)
            delay_2s();        // hold — measure PC6 with multimeter
        }
    }
}
#else
static void openems_init() noexcept {
    // 1) PLL → 250 MHz + SysTick 1ms + IWDG 100ms
    system_stm32_init();

    // 1a) CRITICAL: INJ/IGN LOW before any delay.
    // PA15 resets as JTDI with pull-up → HIGH → injectors on (active-high).
    // PC10/11 float; external pull-ups can also pull INJ high.
    // Must run before the USB 300 ms wait (was: ECU_Hardware_Init after that).
    ::ecu_sched_outputs_safe_early();
    ::ecu_sched_set_inj_inhibit_mask(0x0Fu);
    ::ecu_sched_set_inj_pw_ticks(0u);

    // 1b) Reabilitar IRQs globais EXPLICITAMENTE. O Reset_Handler faz cpsid i e nunca
    // reabilita; antes isto só acontecia por efeito colateral do 1º cpsie de uma seção
    // crítica adiante, o que deixava a ISR do USB (e outras) mascaradas se a ordem mudasse.
    // Com SysTick já configurado em system_stm32_init(), é seguro habilitar aqui.
    __asm__ volatile("cpsie i" ::: "memory");

    // 1c) PB2 (LED WeAct) como saída — heartbeat visível desde o boot.
    GPIOB_MODER = (GPIOB_MODER & ~(3u << 4u)) | (1u << 4u);

    // 1d) USB CDC CEDO: só depende de clock (HSI48/CRS já prontos) + IRQs. Subir aqui,
    // antes dos inits da ECU (ADC/CAN/CKP), garante enumeração mesmo que algum init
    // adiante demore/bloqueie numa placa de bancada sem motor — a ISR cuida do resto.
    ems::hal::usb_cdc_init();

    // 1e) Janela p/ a enumeração USB (ISR-driven) completar antes dos inits da ECU, que
    // podem entrar em seção crítica (cpsid i) e mascarar a ISR do USB por um tempo.
    // ~300 ms kicando o IWDG (timeout 100 ms) a cada ~1 ms @ 250 MHz.
    // Re-assert INJ/IGN LOW each 100 ms in case USB/other init touches GPIOA.
    for (uint32_t ms = 0u; ms < 300u; ++ms) {
        for (volatile uint32_t d = 0u; d < 60000u; ++d) { /* ~1ms */ }
        iwdg_kick();
        ems::hal::usb_cdc_poll();
        if ((ms % 100u) == 0u) {
            GPIOB_ODR ^= (1u << 2u);
            ::ecu_sched_outputs_safe_early();
        }
    }

    // 2) Timers (TIM5 freerun). g_cyl_window (misfire_encoder.cpp) parte de
    // BSS (int8_t 0 = cilindro 0, não o -1 sentinela de "sem cilindro"), e
    // tim2_heartbeat_start() já arma o 1º sub-tick — sem init aqui, todo o
    // ciclo seria atribuído ao cilindro 0 até o primeiro init() correr.
    ems::engine::misfire_encoder_init();
    // Encoder-only: TIM5 is the freerun timebase (dwell/inj watchdogs).
    // TIM2 AB = crank angle; TIM3_CH1 = CMP. No Hall/60-2 IC path.
    ems::hal::tim5_freerun_init();
    ems::hal::tim2_encoder_init();
    ems::hal::tim3_cmp_ic_init();
    ems::hal::mt6835_init();
    // Heartbeat TIM2_CH4 after SPI preload so the first CCR4 target
    // (now+256) starts from a real TIM2_CNT, not reset garbage.
    ems::hal::tim2_heartbeat_start();
    iwdg_kick();

    // 2a) Scheduler unificado (re-asserts pin safe + clears event queue)
    ::ECU_Hardware_Init();
    ::ecu_sched_set_presync_inj_auto(1u);  // auto-select SIMULTANEOUS/SEMI_SEQUENTIAL by cranking
    ::ecu_sched_set_inj_inhibit_mask(0x0Fu);
    ::ecu_sched_set_inj_pw_ticks(0u);
    iwdg_kick();

    // 3) ADC (ADC1/ADC2 + TIM6 trigger)
    ems::hal::adc_init();
    // Sem CKP físico, adc_trigger_on_tooth() nunca corre — TIM6 ficaria
    // parado e o ADC nunca converteria. Arranca livre-corrente uma vez.
    ems::hal::adc_start_free_running_encoder();
    iwdg_kick();

    // 4) CAN + bench communication. MVP transport: USART1 PA9/PA10.
    // (usb_cdc_init() já foi chamado cedo em 1b, antes dos inits da ECU.)
    ems::hal::can0_init();
    ems::hal::uart0_init(115200u);
    ems::hal::uart0_enable_rx();  // RX fica desligado por padrão (uart.cpp:61)
#if EMS_TLE8888_PRESENT
    ems::hal::tle8888_init();
    // INJEN/IGNEN só sobem se o CI confirmou SPI + configure().
    ems::hal::power_stage_enable(ems::hal::tle8888_ok());
#else
    // Sem TLE8888: INJ/IGN são GPIO directo. INJEN/IGNEN sobem sempre
    // (nascem LOW em out_pins_hw_init()).
    ems::hal::power_stage_enable(true);
#endif
    ems::engine::ewg_control_init();
    ems::hal::flex_fuel_init();
    iwdg_kick();

	// 5) Flash Bank2 → carrega calibrações persistidas
	if (!ems::hal::nvm_load_calibration(0u, g_calib_page0, kCalibPageBytes)) {
		++g_flash_write_faults; // FIX: rastrear falha de leitura NVM
	}
	ems::engine::cfg::engine_config_load(g_calib_page0, kCalibPageBytes);
	// Reconstrói g_cyl_window (misfire_encoder.cpp) com o trigger_tooth0_engine_deg
	// REAL agora que engine_config_load() o carregou — a chamada em ~480 só zerou
	// a tabela antes do 1º sub-tick armar (tim2_heartbeat_start), usando o valor
	// por omissão (BSS zero) de trigger_tooth0_engine_deg. misfire_encoder_init()
	// é idempotente (sem alloc/NVIC/MMIO — ver test_misfire_encoder.cpp/test_sched.cpp
	// que já a chamam múltiplas vezes), por isso repetir a chamada aqui é seguro.
	ems::engine::misfire_encoder_init();
	ems::engine::map_estimator_sync_engine_config();  // displacement → MAP model
	// Calibração de sensores persistida (página 0, bytes 16-55) → drivers
	ems::engine::apply_etb_calibration_from_page(g_calib_page0 + 16, 40u);
	ems::engine::push_sensor_calibration_to_drivers();
	// Trims / CMP window / anti-jerk / rev limiter / ckp skip (56-76).
	// Antes só a UI aplicava isto — reboot perdia a calibração.
	ems::engine::apply_page0_trims_driveability(g_calib_page0, kCalibPageBytes);
	// Polaridade CKP/CMP (page0[258]) — re-aplica TIM5 + pull (tim5_ic_init foi
	// antes da NVM, default subida/pull-down).
	ems::engine::apply_page0_capture_polarity(g_calib_page0, kCalibPageBytes);
	// Closed-loop / LEARN (page0[80-85])
	ems::engine::closed_loop_enable =
	    (g_calib_page0[80] != 0u) ? 1u : 0u;
	ems::engine::ltft_apply_burn_ve = (g_calib_page0[81] != 0u) ? 1u : 0u;
	std::memcpy(&ems::engine::closed_loop_post_start_s, g_calib_page0 + 82, 2u);
	std::memcpy(&ems::engine::ltft_adapt_min_rpm_x10,   g_calib_page0 + 84, 2u);
	// Authority LTFT (176-184) só se layout version actual — blob v2 tem lixo/zeros.
	if (g_calib_page0[ems::engine::kCalLayoutVersionOffset] ==
	    ems::engine::kCalLayoutVersion) {
		uint16_t mult_c = 0u, add_c = 0u, max_s = 0u;
		std::memcpy(&mult_c, g_calib_page0 + 176, 2u);
		std::memcpy(&add_c,  g_calib_page0 + 178, 2u);
		std::memcpy(&max_s,  g_calib_page0 + 182, 2u);
		if (mult_c != 0u) { ems::engine::ltft_mult_clamp_pct_x10 = mult_c; }
		if (add_c  != 0u) { ems::engine::ltft_add_clamp_us = add_c; }
		if (g_calib_page0[180] != 0u) { ems::engine::ltft_learn_div = g_calib_page0[180]; }
		if (g_calib_page0[181] != 0u) { ems::engine::ltft_commit_gain_pct = g_calib_page0[181]; }
		ems::engine::ltft_max_step_x10 = max_s;
		if (g_calib_page0[184] <= 1u) {
			ems::engine::ltft_adapt_enable = g_calib_page0[184];
		}
		{
			uint16_t hits = 0u;
			std::memcpy(&hits, g_calib_page0 + 185, 2u);
			if (hits != 0u) { ems::engine::ltft_learn_ready_hits = hits; }
			if (g_calib_page0[187] != 0u) {
				ems::engine::ltft_learn_max_err_x1000 = g_calib_page0[187];
			}
			if (g_calib_page0[188] != 0u) {
				ems::engine::ltft_learn_ready_max_mean_err = g_calib_page0[188];
			}
			if (g_calib_page0[189] != 0u) {
				ems::engine::ltft_learn_ready_min_stft_x10 = g_calib_page0[189];
			}
			if (g_calib_page0[190] != 0u) {
				ems::engine::ltft_learn_ready_max_stft_x10 = g_calib_page0[190];
			}
		}
		// Launch + TC knobs (page0 191-215, layout v5)
		ems::engine::launch_tc_apply_from_page0(g_calib_page0, kCalibPageBytes);
		// CAN RX map: gear / vehicle / driven wheel (216-245)
		ems::app::can_rx_map_apply_from_page0(g_calib_page0, kCalibPageBytes);
		// MAP janela angular (246-251); len=0 não substitui o default
		ems::engine::map_window_enable = (g_calib_page0[246] != 0u) ? 1u : 0u;
		{
			uint16_t od = 0u, wl = 0u;
			std::memcpy(&od, g_calib_page0 + 248, 2u);
			std::memcpy(&wl, g_calib_page0 + 250, 2u);
			ems::engine::map_window_open_deg =
			    (od >= 720u) ? static_cast<uint16_t>(od % 720u) : od;
			if (wl != 0u) {
				ems::engine::map_window_len_deg =
				    (wl < 10u) ? 10u : (wl > 180u) ? 180u : wl;
			}
		}
		// Protecção de duty INJ + gates DFCO + knock morto (252-257);
		// blob antigo = zeros = tudo off (tol=0 mantém default 300 ms).
		ems::engine::inj_duty_max_pct = g_calib_page0[252];
		if (g_calib_page0[253] != 0u) {
			ems::engine::inj_duty_tol_ms10 = g_calib_page0[253];
		}
		std::memcpy(&ems::engine::decel_cut_map_max_bar_x100,
		            g_calib_page0 + 254, 2u);
		ems::engine::decel_cut_gear_inhibit_ms10 = g_calib_page0[256];
		ems::engine::knock_dead_min_p2p = g_calib_page0[257];
		if (kCalibPageBytes > (ems::engine::kDecelCutRampMsPage0Off + 1u)) {
			std::memcpy(&ems::engine::decel_cut_ramp_ms,
			            g_calib_page0 + ems::engine::kDecelCutRampMsPage0Off, 2u);
		}
	}
	// Gate de layout: páginas de tabela só carregam se a versão gravada no
	// page0 (byte 175) bater com o firmware — um blob de dimensão antiga
	// lido com o tamanho novo ganharia cauda 0xFF (VE=255!). Sem versão →
	// defaults de compilação; um "burn all" no dashboard re-persiste tudo.
	const bool cal_layout_ok =
		g_calib_page0[ems::engine::kCalLayoutVersionOffset] ==
		ems::engine::kCalLayoutVersion;
	ems::app::nvm_boot_load_tables(cal_layout_ok);
	if (!ems::hal::nvm_load_adaptive_maps()) {
		++g_flash_write_faults; // FIX: rastrear falha de leitura NVM
	}
    // 6) Drivers
    ems::drv::sensors_init();
    iwdg_kick();

    // 6a) Inicializa sistemas "invisíveis" ao motorista
    ems::engine::DiagnosticManager::init();
    ems::engine::map_estimator_init();
    ems::engine::xtau_autocalib_init();

    // 6b) Inicializa ETB e Torque Manager (borboleta eletrônica)
    g_etb_initialized = etb_control_init();
    // Auto-cal dos limites TPS a cada power-on (varre batentes no tick 2ms;
    // pulada se harness ausente; falha mantém a calibração de flash).
    if (g_etb_initialized) {
        ems::engine::etb_autocal_start();
    }
    torque_manager_init();
    iwdg_kick();

    // 7) Engine
    ems::engine::fuel_reset_adaptives();
    ems::engine::auxiliaries_init();
    ems::engine::knock_init();
    ems::engine::quick_crank_reset();
    iwdg_kick();

    // 8) Aplicação
    ems::app::ui_init();
    ems::app::can_stack_init(ems::engine::wbo2_can_id);

    // 9) NVIC — CKP fica com prioridade máxima. Injeção/ignição em TIM2/TIM1
    //    usam output compare direto por hardware, sem ISR no caminho crítico.
    //    SysTick configurado em system_stm32_init() com prio 11.
    nvic_set_priority(IRQ_TIM5, 1u);
    nvic_enable_irq(IRQ_TIM5);

    // 10) Aguardar CKP sync (timeout 5 s), heartbeat 5 Hz
    {
        const uint32_t sync_deadline = millis() + 5000u;
        uint32_t next_toggle = millis() + 100u;
        while (millis() < sync_deadline) {
            iwdg_kick();
            if (millis() >= next_toggle) {
                GPIOB_ODR ^= (1u << 2u);
                next_toggle = millis() + 100u;
            }
            const auto snap = ems::drv::ckp_snapshot();
            if (snap.state == ems::drv::SyncState::FULL_SYNC) { break; }
        }
    }

    // Kick final antes de entrar no main loop — garante que openems_init()
    // não ultrapassa o timeout de 100 ms do IWDG.
    iwdg_kick();
}
#endif // MINIMAL_BOOT


// =============================================================================
// main() — substituição do setup()/loop() do STM32 runtime
// =============================================================================

int main() {
    openems_init();

    uint32_t g_t2ms_   = millis();
    uint32_t g_t10ms_  = g_t2ms_;
    uint32_t g_t20ms_  = g_t2ms_;
    uint32_t g_t50ms_  = g_t2ms_;
    uint32_t g_t100ms_ = g_t2ms_;
    uint32_t g_t500ms_ = g_t2ms_;
    uint32_t g_t_etb_ms = g_t2ms_;
    uint32_t g_t_comms_ms = g_t2ms_;

    // Boot IWDG is ~10 s (/256). Runtime is /32 + RLR=99 → ~100 ms.
    iwdg_enter_runtime();

    for (;;) {
        // ── Watchdog kick (primeiro statement) ───────────────────────────
        iwdg_kick();
        g_datalog_us = micros();

        const uint32_t now = millis();

        // ── 2ms: fuel + ign recalc + commit calibration ───────────────────
        if (elapsed(now, g_t2ms_, 2u)) {
            g_t2ms_ = now;
            const uint32_t loop2ms_start_us = micros();

            if (ems::drv::ckp_stall_poll_encoder(ems::hal::tim5_count()) ||
                !ems::drv::encoder_sync::health_ok()) {
                ecu_sched_on_encoder_stall();
            }

            // Watchdog do TIM3 CMP IC: se revoluções demais se passaram
            // sem um flanco CMP aceite (virabrequim vivo, CMP mudo — ver
            // ecu_sched_encoder_cmp_watchdog_poll_and_clear() em
            // ecu_sched_encoder_heartbeat.cpp), rearma o periférico. Fora de
            // contexto de ISR, mesmo padrão dos watchdogs abaixo.
            if (ecu_sched_encoder_cmp_watchdog_poll_and_clear() != 0U) {
                ems::hal::CriticalSectionGuard guard;
                ems::hal::tim3_cmp_ic_init();
            }

            // Dwell / injector open watchdogs (lost SPARK / lost INJ_OFF).
            ecu_sched_dwell_watchdog();
            ecu_sched_inj_watchdog();

            // Sem hook por-dente: refresca MAP/TPS/MAF/knock e a janela MAP
            // angular ANTES do sensors_get() para este ciclo já ver valores
            // frescos (docs/dev/mt6835_encoder_fork.md, Parte 3a/3b).
            ems::drv::sensors_sample_fast_channels_encoder(
                ems::drv::ckp_snapshot().rpm_x10);
            ems::drv::sensors_map_window_poll_encoder(ems::hal::tim2_encoder_count());

            const auto snap    = ems::drv::ckp_snapshot();
            const auto sensors = ems::drv::sensors_get();

            // Teste de saídas em bancada: aborto imediato se RPM > 0 e
            // timeout de keepalive (o dwell watchdog acima continua activo).
            ems::engine::output_test_poll(now, snap.rpm_x10);
            loop_2ms_fuel_ign(now, snap, sensors);

            // EWG position inner loop (2ms cadence)
            if (!ems::engine::output_test_active()) {
                const uint16_t demand = ems::engine::auxiliaries_ewg_position_demand_x10();
                const uint16_t pos = ems::engine::ewg_read_position_pct_x10();
                ems::engine::ewg_control_update(demand, pos);
            }

            g_loop2ms_last_us = micros() - loop2ms_start_us;
            if (g_loop2ms_last_us > g_loop2ms_max_us) {
                g_loop2ms_max_us = g_loop2ms_last_us;
            }
            ems::app::ui_update_loop_diag(g_loop2ms_last_us, g_loop2ms_max_us);
        }

        // ── 10ms: VVT, wastegate PID ─────────────────────────────────────
        if (elapsed(now, g_t10ms_, 10u)) {
            g_t10ms_ = now;
            ems::engine::auxiliaries_tick_10ms();
        }

        // ── 20ms: UI proprietaria + aux tasks ───────────────────────────
        if (elapsed(now, g_t20ms_, 20u)) {
            g_t20ms_ = now;
            const auto snap = ems::drv::ckp_snapshot();
            const auto sensors = ems::drv::sensors_get();
            ems::app::ui_update_rt_metrics(
	            // Formula result: flow + dead×openings (not flow/2+dead).
	            g_last_pw_ms_x10,
	            g_last_advance_deg, g_last_stft_pct,
                                           g_last_lambda_target_d4, g_last_ltft_pct);
            ems::app::ui_update_rt_sched_diag(
                ecu_sched_encoder_late_event_count(),
                g_cycle_schedule_drop_count,
                g_calibration_clamp_count,
                static_cast<uint8_t>(snap.state));
            // Transporte (UART+USB RX/TX/parse) vive em comms_pump() a 2 ms.
            ems::engine::auxiliaries_tick_20ms();
            ems::app::can_stack_process(now, snap, sensors,
                                        g_last_advance_deg,
                                        g_last_pw_ms_x10,
                                        g_last_stft_pct,
                                        0u, 0u,
                                        build_status_bits(snap, sensors));
        }

        // ── 50ms: sensores lentos ──────────────────────────────────────────
        if (elapsed(now, g_t50ms_, 50u)) {
            g_t50ms_ = now;
            ems::drv::sensors_tick_50ms();
        }

        // ── 100ms: sensores + STFT + misfire report + baro ──────────────
        if (elapsed(now, g_t100ms_, 100u)) {
            g_t100ms_ = now;
            ems::drv::sensors_tick_100ms();
#if EMS_TLE8888_PRESENT
            ems::hal::tle8888_poll_diag();
#endif

            // Poll de saúde do MT6835 a 100 ms (SPI, não 2 ms). O ângulo
            // lido é descartado — posição vem do TIM2. Sem hardware
            // (MT6835_HW_PRESENT=0) a leitura falharia sempre.
            if (ems::hal::mt6835_hw_present()) {
                uint32_t health_angle21_unused = 0u;
                uint8_t  health_status_unused  = 0u;
                const bool health_ok = ems::hal::mt6835_read_angle_raw21(
                    &health_angle21_unused, &health_status_unused);
                ems::drv::encoder_sync::set_health_ok(health_ok);
                if (!health_ok) {
                    ecu_sched_on_encoder_stall();
                }
            }

            // Knock sensor morto (FOME #578): report único na transição.
            {
                static bool s_knock_dead_reported = false;
                const bool dead = ems::engine::knock_sensor_dead();
                if (dead && !s_knock_dead_reported) {
                    ems::engine::DiagnosticManager::report_fault(
                        ems::engine::DiagnosticCode::KNOCK_SENSOR_FAULT,
                        ems::engine::FaultSeverity::WARNING);
                    s_knock_dead_reported = true;
                } else if (!dead) {
                    s_knock_dead_reported = false;
                }
            }

            // Flex fuel: update stoich AFR based on ethanol %
            // E0=14.7 (1470), E100=9.0 (900), linear
            if (ems::hal::flex_fuel_valid()) {
                const uint16_t eth = ems::hal::flex_fuel_ethanol_pct();
                ems::engine::cfg::g_eng_cfg.stoich_afr_x100 =
                    static_cast<uint16_t>(1470u - (eth * 570u) / 100u);
            }
            const auto snap    = ems::drv::ckp_snapshot();
            const auto sensors = ems::drv::sensors_get();

            // Compensação barométrica: amostrar MAP enquanto motor parado.
            // Aguarda 300ms estabilizado antes de aceitar a leitura (ADC settle).
            if (snap.rpm_x10 == 0u) {
                if (g_baro_stopped_since_ms == 0u) {
                    g_baro_stopped_since_ms = now;
                    g_baro_sampled = false;
                } else if (!g_baro_sampled &&
                           (now - g_baro_stopped_since_ms) >= 300u) {
                    const uint16_t map_baro = clamp_u16(
                        static_cast<uint16_t>(sensors.map_bar_x1000 / 10u),
                        70u, 110u);
                    ems::engine::fuel_set_baro_bar_x100(map_baro);
                    g_baro_sampled = true;
                }
            } else {
                g_baro_stopped_since_ms = 0u;
            }

            // Misfire: reporte de DTCs acumulados no período de 100ms
            if (snap.state == ems::drv::SyncState::FULL_SYNC) {
                constexpr ems::engine::DiagnosticCode kMisfireCodes[4] = {
                    ems::engine::DiagnosticCode::MISFIRE_CYLINDER_1,
                    ems::engine::DiagnosticCode::MISFIRE_CYLINDER_2,
                    ems::engine::DiagnosticCode::MISFIRE_CYLINDER_3,
                    ems::engine::DiagnosticCode::MISFIRE_CYLINDER_4,
                };
                for (uint8_t c = 0u; c < 4u; ++c) {
                    // Só o detector encoder: misfire_detect.cpp não está no
                    // firmware (janela-por-dente 60-2, sem dentes neste tree).
                    const uint8_t count =
                        ems::engine::misfire_encoder_get_event_count(c);
                    if (count >= ems::engine::kMisfireFaultThreshold) {
                        ems::engine::DiagnosticManager::report_fault(
                            kMisfireCodes[c],
                            ems::engine::FaultSeverity::WARNING);
                        ems::engine::misfire_encoder_clear_events(c);
                    }
                }
            }

            // HALF_SYNC: posição TIM2 fiável, falta só a metade de 720°.
            // STFT/λ também corre em HALF (excepto cranking).
            const bool stft_sync_ok = (snap.state == ems::drv::SyncState::FULL_SYNC) ||
                (snap.state == ems::drv::SyncState::HALF_SYNC && !ems::engine::is_cranking());
            if (stft_sync_ok) {
                // MAP fundido (mesma fonte do cálculo de combustível de 2ms):
                // o cru daqui divergia na fronteira de célula → alvo λ do
                // gauge oscilava sem o ponto de operação mudar.
                const uint16_t map_bar_x100 = clamp_u16(
                    g_last_map_fused_x100, kMapMinBarX100, kMapMaxBarX100);
                const uint16_t lambda_measured = clamp_u16(
                    ems::app::can_stack_lambda_milli_safe(now), kLambdaMinMilli, kLambdaMaxMilli);
                const bool lambda_valid = ems::app::can_stack_wbo2_fresh(now);
                const uint16_t lambda_target_x1000 =
                    ems::engine::get_lambda_target_x1000(snap.rpm_x10, map_bar_x100);
                const bool rev_cut = g_limp_active &&
                    (snap.rpm_x10 > kLimpRpmLimit_x10);
                const bool ae_active = ems::engine::fuel_ae_stft_freeze_active();
                // STFT congelado em qualquer condição de corte intencional de combustível:
                // - rev_cut: limp mode
                // - decel_cut: borboleta fechada em desaceleração
                // - inj_inhibit_mask != 0: rev limiter cortou injeção em ≥1 cilindro
                //   (lambda leria lean sem combustível → STFT aprenderia errado)
                const bool stft_inhibit = rev_cut ||
                    ems::engine::fuel_decel_cut_active() ||
                    (::ecu_sched_get_inj_inhibit_mask() != 0u);
                // APP (pedido do condutor) para estabilidade do acumulador LTFT:
                // ETB mexe sozinho em idle e rejeitaria hits sem o condutor mexer.
                // Célula continua a ser (MAP, RPM); APP só filtra regime.
                const int16_t stft = ems::engine::fuel_update_stft_delayed(
                    now, snap.rpm_x10, map_bar_x100,
                    static_cast<int16_t>(lambda_target_x1000),
                    static_cast<int16_t>(lambda_measured),
                    sensors.clt_degc_x10, lambda_valid,
                    ae_active, stft_inhibit, g_last_net_pw_us,
                    sensors.app_pct_x10);
                ems::engine::fuel_ae_stft_freeze_clear();
                g_last_stft_pct = clamp_i8(static_cast<int16_t>(stft / 10), -25, 25);
                // ÷5 (não ÷4): ÷4 saturava o u8 em 1020 — alvos 1.02-1.27 exibiam 1.02
                g_last_lambda_target_d4 = ems::engine::clamp_u8(lambda_target_x1000 / 5u);
                g_last_ltft_pct = clamp_i8(
                    ems::engine::fuel_get_ltft_at(snap.rpm_x10, map_bar_x100) / 10,
                    -25, 25);

                // X-τ learn (100ms): exige transiente real (tpsdot) E gates de
                // qualidade λ/STFT/RPM. Nunca tratar "learning_ok" sozinho como
                // is_transient — isso corrompia a tabela 2D em regime estável.
                const bool xtau_learning_ok = (stft >= -500 && stft <= 500 &&
                    lambda_valid && snap.rpm_x10 >= 20000u);
                const bool xtau_transient = ems::engine::map_is_transient() &&
                    xtau_learning_ok && !ae_active;
                ems::engine::xtau_autocalib_update(
                    snap.rpm_x10,
                    map_bar_x100,  // MAP fundido (mesma fonte do PW 2ms)
                    static_cast<int16_t>(lambda_target_x1000),
                    static_cast<int16_t>(ems::app::can_stack_lambda_milli_safe(now)),
                    sensors.clt_degc_x10,
                    xtau_transient);
            } else {
                g_last_stft_pct = 0;
            }

        }

        // ── 500ms: agenda flush Flash + LED heartbeat (PB2 WeAct blue LED) ─
        // PB2 = LED blue on-board da WeAct STM32H562 LQFP100.
        // FIX P0: Only allow flash writes when engine is stopped or below safe RPM
        static bool adaptive_flush_pending = false;
        static uint32_t last_calib_save_ms = 0u;
        static bool engine_was_running = false;
        ems::hal::nvm_set_now_ms(now);
        if (elapsed(now, g_t500ms_, 500u)) {
            g_t500ms_ = now;
            // LED heartbeat: toggle PB2 a cada 500ms (1 Hz)
            GPIOB_MODER = (GPIOB_MODER & ~(3u << 4u)) | (1u << 4u);
            GPIOB_ODR ^= (1u << 2u);  // toggle PB2
            const auto snap = ems::drv::ckp_snapshot();
            // Motor acabou de parar (RPM>0 → 0): força flush do LTFT/knock/etbcal
            // imediatamente, ignorando os 60s de kMinAdaptiveFlushIntervalMs.
            // Sem isto, nenhum caminho de produção chamava
            // nvm_request_adaptive_flush_now() fora do 'Z' manual — uma sessão
            // mais curta que 60s desde o último flush automático perdia
            // aprendizagem de LTFT em silêncio numa queda de energia pós key-off
            // (achado da revisão da mesa de 5 conselheiros, 2026-08-14).
            const bool engine_running_now = (snap.rpm_x10 > 0u);
            if (engine_was_running && !engine_running_now) {
                ems::hal::nvm_request_adaptive_flush_now();
            }
            engine_was_running = engine_running_now;
            // FIX: gate ALL flash writes behind the same RPM threshold — calibration
            // writes had no RPM check, creating a latent bug (see Blocker #3).
            const bool engine_running_fast = (snap.rpm_x10 > ems::engine::kFlashWriteSafeRpmX10);
            if (!engine_running_fast) {
                if (g_calib_dirty &&
                    (last_calib_save_ms == 0u ||
                     elapsed(now, last_calib_save_ms, kCalibSaveMinIntervalMs))) {
                    ems::engine::cfg::engine_config_serialize(g_calib_page0, kCalibPageBytes);
                    if (ems::hal::nvm_save_calibration(0u, g_calib_page0,
                                                       kCalibPageBytes)) {
                        g_calib_dirty = false;
                        last_calib_save_ms = now;
                    }
                }
                // Adaptive LTFT/knock: só agenda se dirty (rate-limit dentro do flush).
                if (ems::hal::nvm_adaptive_maps_dirty()) {
                    adaptive_flush_pending = true;
                }
            }
        }
        if (adaptive_flush_pending) {
            // Double-check RPM before actually writing (engine may have started)
            const auto snap = ems::drv::ckp_snapshot();
            const bool engine_running_fast = (snap.rpm_x10 > ems::engine::kFlashWriteSafeRpmX10);
            if (!engine_running_fast) {
                adaptive_flush_pending = !ems::hal::nvm_flush_adaptive_maps();
            }
        }

        // ── Comms pump UART (2ms cadence, fora da medição de loop2) ──────
        if (elapsed(now, g_t_comms_ms, 2u)) {
            g_t_comms_ms = now;
            comms_pump();
        }

        // ── ETB control (2ms cadence) ─────────────────────────────────────
        if (elapsed(now, g_t_etb_ms, 2u) && !ems::engine::output_test_active()) {
            g_t_etb_ms = now;
            const auto sensors_etb = ems::drv::sensors_get();
            const auto snap_etb = ems::drv::ckp_snapshot();
            // Auto-cal power-on em curso: varre batentes e pula torque/PID.
            if (ems::engine::etb_autocal_active()) {
                ems::engine::etb_autocal_tick(2u, snap_etb.rpm_x10);
            } else {
                const bool etb_rev_cut = g_limp_active && (snap_etb.rpm_x10 > kLimpRpmLimit_x10);
                const auto torque_out = ems::engine::torque_manager_update(
                    snap_etb, sensors_etb, true, g_limp_active, etb_rev_cut,
                    ems::engine::auxiliaries_idle_target_rpm_x10(sensors_etb.clt_degc_x10), 2u);
                g_torque_spark_retard_deg = torque_out.spark_retard_deg;
                const auto etb = ems::engine::etb_control_update(
                    torque_out.etb_target_pct_x10, sensors_etb.etb_tps_pct_x10,
                    torque_out.etb_enable_request, 2u);
                // Apply PID → H-bridge. On disable/fault/no-cal, spring-return safe.
                if (!torque_out.etb_enable_request || !etb.active || !g_etb_initialized) {
                    ::etb_driver_shutdown();
                } else {
                    // output_pct_x10 ∈ [-1000, 1000] → driver PWM ∈ [-1023, 1023]
                    int32_t pwm = (static_cast<int32_t>(etb.output_pct_x10) * 1023) / 1000;
                    if (pwm > 1023) { pwm = 1023; }
                    if (pwm < -1023) { pwm = -1023; }
                    if (!::etb_driver_set_motor_pwm(static_cast<int16_t>(pwm))) {
                        ::etb_driver_shutdown();
                    }
                }
            }
        }
    }
}

#endif  // TARGET_STM32H562
