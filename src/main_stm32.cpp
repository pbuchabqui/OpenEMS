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
//   - NVIC setup usa IRQs do STM32H562 (TIM5 para CKP; TIM2/TIM8 são OC por hardware)
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
#include "app/ui_protocol.h"
#include "drv/ckp.h"
#include "drv/sensors.h"
#include "engine/auxiliaries.h"
#include "engine/calibration.h"
#include "engine/constants.h"
#include "engine/ecu_sched.h"
#include "engine/engine_config.h"
#include "engine/etb_control.h"
#include "engine/fuel_calc.h"
#include "engine/ign_calc.h"
#include "engine/knock.h"
#include "engine/map_estimator.h"
#include "engine/diagnostic_manager.h"
#include "engine/misfire_detect.h"
#include "engine/quick_crank.h"
#include "engine/torque_manager.h"
#include "engine/transient_fuel.h"
#include "engine/xtau_autocalib.h"
#include "hal/adc.h"
#include "hal/can.h"
#include "hal/etb_driver.h"
#include "hal/flash.h"
#include "hal/runtime_seed.h"
#include "hal/timer.h"
#include "hal/etb_driver.h"

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


static int8_t  g_last_advance_deg = 0;
static uint8_t g_last_pw_ms_x10   = 0u;
static int8_t  g_last_stft_pct    = 0;

static constexpr uint32_t kLimpRpmLimit_x10 = 30000u;
static constexpr uint8_t  kFaultBitMap = (1u << 0u);
static constexpr uint8_t  kFaultBitClt = (1u << 3u);
static bool g_limp_active = false;
static bool g_engine_was_running = false;
static bool g_runtime_seed_saved_for_stop = false;
static bool g_runtime_seed_arm_window_active = false;
static bool     g_ae_active        = false;
static uint32_t g_last_net_pw_us   = 0u;
static int16_t  g_rev_spark_trim   = 0;   // calculado no bloco rev limiter, usado no advance
// Barometric correction: amostrar MAP quando motor parado por >300ms após key-on
static uint32_t g_baro_stopped_since_ms = 0u;
static bool     g_baro_sampled          = false;
static uint32_t g_zero_rpm_since_ms = 0u;
static uint32_t g_runtime_seed_arm_window_start_ms = 0u;
static uint16_t g_prev_tps_pct_x10 = 0u;
static bool g_have_last_full_sync = false;
static ems::drv::CkpSnapshot g_last_full_sync_snapshot = {
    0u, 0u, 0u, 0u, 0u, ems::drv::SyncState::WAIT_GAP, false
};
static bool g_have_last_gap_sync = false;
static ems::drv::CkpSnapshot g_last_gap_sync_snapshot = {
    0u, 0u, 0u, 0u, 0u, ems::drv::SyncState::WAIT_GAP, false
};
static uint32_t g_loop2ms_last_us = 0u;
static uint32_t g_loop2ms_max_us = 0u;

static constexpr uint32_t kRuntimeSeedSaveDelayMs = 100u;
static constexpr uint32_t kRuntimeSeedArmWindowMs = 300000u;  // 5 minutos para start-stop
static constexpr uint32_t kSchedulerTicksPerMs = 10000u;
static constexpr uint32_t kCalibSaveMinIntervalMs = 300000u;
static constexpr uint16_t kMapMinBarX100 = 10u;
static constexpr uint16_t kMapMaxBarX100 = 300u;
static constexpr uint16_t kLambdaMinMilli = 700u;
static constexpr uint16_t kLambdaMaxMilli = 1400u;
static constexpr uint16_t kAePeriodMs = 2u;

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
    return status;
}

using ems::engine::clamp_u16;
using ems::engine::clamp_i16;

static inline int8_t clamp_i8(int16_t v, int8_t lo, int8_t hi) noexcept {
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return static_cast<int8_t>(v);
}

static bool page_range_is_zero(const uint8_t* data, uint16_t off, uint16_t len) noexcept {
    for (uint16_t i = 0u; i < len; ++i) {
        if (data[off + i] != 0u) {
            return false;
        }
    }
    return true;
}

static bool page_range_is_erased(const uint8_t* data, uint16_t off, uint16_t len) noexcept {
    for (uint16_t i = 0u; i < len; ++i) {
        if (data[off + i] != 0xFFu) { return false; }
    }
    return true;
}

static bool page_is_erased(const uint8_t* data, uint16_t len) noexcept {
    return page_range_is_erased(data, 0u, len);
}

static void load_corr_calibration_from_nvm() noexcept {
    alignas(4) uint8_t page[256] = {};
    if (!ems::hal::nvm_load_calibration(4u, page, sizeof(page)) ||
        page_is_erased(page, sizeof(page))) {
        return;
    }

    const uint8_t* p = page;
    std::memcpy(ems::engine::clt_corr_axis_x10,          p +   0, 16u);
    std::memcpy(ems::engine::clt_corr_x256,              p +  16, 16u);
    std::memcpy(ems::engine::iat_corr_axis_x10,          p +  32, 16u);
    std::memcpy(ems::engine::iat_corr_x256,              p +  48, 16u);
    std::memcpy(ems::engine::warmup_corr_axis_x10,       p +  64, 16u);
    std::memcpy(ems::engine::warmup_corr_x256,           p +  80, 16u);
    std::memcpy(ems::engine::vbatt_corr_axis_mv,         p +  96, 16u);
    std::memcpy(ems::engine::injector_dead_time_us,      p + 112, 16u);
    std::memcpy(ems::engine::ae_clt_corr_axis_x10,       p + 128, 16u);
    std::memcpy(ems::engine::ae_clt_sens,                p + 144, 16u);
    std::memcpy(ems::engine::dwell_vbatt_axis_mv,        p + 160, 16u);
    std::memcpy(ems::engine::dwell_ms_x10_table,         p + 176, 16u);
    std::memcpy(ems::engine::lambda_delay_rpm_axis_x10,  p + 192, 12u);
    std::memcpy(ems::engine::lambda_delay_load_axis_bar_x100, p + 204, 12u);
    std::memcpy(ems::engine::lambda_delay_ms_table,      p + 216, 18u);
    if (!page_range_is_zero(page, 234u, 6u)) {
        std::memcpy(&ems::engine::ae_tpsdot_threshold_x10, p + 234, 2u);
        std::memcpy(&ems::engine::ae_taper_cycles,         p + 236, 2u);
        std::memcpy(&ems::engine::ae_max_pw_us,            p + 238, 2u);
    }
    if (page_range_is_zero(page, 240u, 16u)) {
        return;
    }
    std::memcpy(&ems::engine::idle_spark_tps_max_x10,             p + 240, 2u);
    std::memcpy(&ems::engine::idle_spark_map_max_bar_x100,             p + 242, 2u);
    std::memcpy(&ems::engine::idle_spark_rpm_min_x10,             p + 244, 2u);
    std::memcpy(&ems::engine::idle_spark_window_above_target_x10, p + 246, 2u);
    std::memcpy(&ems::engine::idle_spark_deadband_rpm_x10,        p + 248, 2u);
    std::memcpy(&ems::engine::idle_spark_rpm_per_deg_x10,         p + 250, 2u);
    std::memcpy(&ems::engine::idle_spark_retard_limit_deg,        p + 252, 2u);
    std::memcpy(&ems::engine::idle_spark_advance_limit_deg,       p + 254, 2u);
}

static void load_dwell2d_calibration_from_nvm() noexcept {
    alignas(4) uint8_t page[32] = {};
    if (!ems::hal::nvm_load_calibration(6u, page, sizeof(page)) ||
        page_is_erased(page, 16u)) {
        return;  // sem page 6 → mantém valores default (factor 1.0× a 4000 RPM)
    }
    std::memcpy(ems::engine::dwell_rpm_axis_rpm,  page + 0,  8u);
    std::memcpy(ems::engine::dwell_rpm_factor_q8, page + 8,  8u);
}

static void load_xtau_calibration_from_nvm() noexcept {
    alignas(4) uint8_t page[80] = {};
    if (!ems::hal::nvm_load_calibration(5u, page, sizeof(page)) ||
        page_is_erased(page, sizeof(page)) ||
        page_range_is_zero(page, 0u, 48u)) {
        return;
    }

    const uint8_t* p = page;
    std::memcpy(ems::engine::xtau_clt_axis_x10,  p +  0, 16u);
    std::memcpy(ems::engine::xtau_x_fraction_q8, p + 16, 16u);
    std::memcpy(ems::engine::xtau_tau_cycles,    p + 32, 16u);
    if (!page_range_is_zero(page, 48u, 16u)) {
        std::memcpy(ems::engine::ae_tpsdot_axis_x10, p + 48, 8u);
        std::memcpy(ems::engine::ae_pw_adder_us,     p + 56, 8u);
    }
    if (!page_range_is_zero(page, 64u, 12u) &&
        !page_range_is_erased(page, 64u, 12u)) {
        std::memcpy(&ems::engine::crank_enter_rpm_x10,   p + 64, 2u);
        std::memcpy(&ems::engine::crank_exit_rpm_x10,    p + 66, 2u);
        std::memcpy(&ems::engine::crank_spark_deg,       p + 68, 2u);
        std::memcpy(&ems::engine::crank_min_pw_us,       p + 70, 2u);
        std::memcpy(&ems::engine::crank_prime_tooth,     p + 72, 2u);
        std::memcpy(&ems::engine::crank_prime_max_pw_us, p + 74, 2u);
    }
}

struct CachedFuelCorrections {
    bool valid;
    int16_t clt_x10;
    int16_t iat_x10;
    uint16_t vbatt_mv;
    uint16_t corr_clt_x256;
    uint16_t corr_iat_x256;
    uint16_t dead_time_us;
    // dwell_ms_x10 não é cacheado: depende de RPM que varia a cada dente.
    // Calculado inline via dwell_ms_x10_from_vbatt_rpm() em cada slot de 2ms.
};

static CachedFuelCorrections g_fuel_corr_cache = {};

static inline const CachedFuelCorrections& fuel_corrections_for(
    const ems::drv::SensorData& sensors) noexcept {
    if (!g_fuel_corr_cache.valid ||
        g_fuel_corr_cache.clt_x10 != sensors.clt_degc_x10 ||
        g_fuel_corr_cache.iat_x10 != sensors.iat_degc_x10 ||
        g_fuel_corr_cache.vbatt_mv != sensors.vbatt_mv) {
        g_fuel_corr_cache.valid = true;
        g_fuel_corr_cache.clt_x10 = sensors.clt_degc_x10;
        g_fuel_corr_cache.iat_x10 = sensors.iat_degc_x10;
        g_fuel_corr_cache.vbatt_mv = sensors.vbatt_mv;
        g_fuel_corr_cache.corr_clt_x256 = ems::engine::corr_clt(sensors.clt_degc_x10);
        g_fuel_corr_cache.corr_iat_x256 = ems::engine::corr_iat(sensors.iat_degc_x10);
        g_fuel_corr_cache.dead_time_us = ems::engine::corr_vbatt(sensors.vbatt_mv);
    }
    return g_fuel_corr_cache;
}

static inline void ui_service() noexcept {
    // ── UART path (transporte primário: adaptador USB-UART em PA9/PA10) ──
    ems::hal::uart0_poll_rx(64u);
    {
        uint8_t b = 0u;
        while (ems::hal::uart0_rx_pop(b)) {
            ems::app::ui_rx_byte(b);
        }
    }

    // ── USB CDC path (stub; no-op até driver real) ─────────────────────────
    ems::hal::usb_cdc_poll();
    if (ems::hal::usb_cdc_dtr()) {
        uint8_t rx_buf[64] = {};
        const uint16_t rx_n = ems::hal::usb_cdc_read_bytes(rx_buf, 64u);
        for (uint16_t i = 0u; i < rx_n; ++i) {
            ems::app::ui_rx_byte(rx_buf[i]);
        }
    }

    // ── Processar protocolo ────────────────────────────────────────────────
    ems::app::ui_process();

    // ── Drenar TX para ambos os transportes ───────────────────────────────
    uint8_t tx_buf[96] = {};
    uint16_t tx_n = 0u;
    while (tx_n < 96u && ems::app::ui_tx_pop(tx_buf[tx_n])) { ++tx_n; }
    if (tx_n != 0u) {
        for (uint16_t i = 0u; i < tx_n; ++i) {
            ems::hal::uart0_tx_push(tx_buf[i]);
        }
        ems::hal::usb_cdc_send_bytes(tx_buf, tx_n);
    }
    ems::hal::uart0_tx_poll(32u);  // máx 32 bytes/ciclo de 20 ms (~230 B budget)
}

// =============================================================================
// Inicialização — sequência idêntica ao main.cpp
// =============================================================================

// Modo diagnóstico: descomente para teste isolado de USB CDC (clock + usb_cdc_init + echo).
// Em produção fica DESATIVADO → usa o openems_init() real (ECU completa + usb_cdc_init).
// #define MINIMAL_BOOT 1

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

    // Loop limpo: echo a cada iteração (sem bloqueio) + heartbeat de LED não-bloqueante.
    uint32_t hb = 0u;
    while (true) {
        STM32_REG32(0x40002C00) = 0x7Fu;  // kick WWDG

        // Echo: devolve cada byte recebido (valida EP2 OUT bulk + EP1 IN bulk + BDTable).
        ems::hal::usb_cdc_poll();
        while (ems::hal::usb_cdc_available()) {
            const uint8_t b = ems::hal::usb_cdc_read_byte();
            ems::hal::usb_cdc_send_byte(b);
        }

        // Heartbeat: toggle PB2 ~a cada N iterações, sem bloquear o echo.
        if (++hb >= 100000u) {
            STM32_REG32(0x42020414) ^= (1u << 2);  // GPIOB_ODR toggle PB2
            hb = 0u;
        }
    }
}
#else
static void openems_init() noexcept {
    // 1) PLL → 250 MHz + SysTick 1ms + IWDG 100ms
    system_stm32_init();

    // 1a) Reabilitar IRQs globais EXPLICITAMENTE. O Reset_Handler faz cpsid i e nunca
    // reabilita; antes isto só acontecia por efeito colateral do 1º cpsie de uma seção
    // crítica adiante, o que deixava a ISR do USB (e outras) mascaradas se a ordem mudasse.
    // Com SysTick já configurado em system_stm32_init(), é seguro habilitar aqui.
    __asm__ volatile("cpsie i" ::: "memory");

    // 1b) USB CDC CEDO: só depende de clock (HSI48/CRS já prontos) + IRQs. Subir aqui,
    // antes dos inits da ECU (ADC/CAN/CKP), garante enumeração mesmo que algum init
    // adiante demore/bloqueie numa placa de bancada sem motor — a ISR cuida do resto.
    ems::hal::usb_cdc_init();

    // 1c) Janela p/ a enumeração USB (ISR-driven) completar antes dos inits da ECU, que
    // podem entrar em seção crítica (cpsid i) e mascarar a ISR do USB por um tempo.
    // ~300 ms kicando o IWDG (timeout 100 ms) a cada ~1 ms @ 250 MHz.
    for (uint32_t ms = 0u; ms < 300u; ++ms) {
        for (volatile uint32_t d = 0u; d < 60000u; ++d) { /* ~1ms */ }
        iwdg_kick();
        ems::hal::usb_cdc_poll();
    }

    // 2) Timers (TIM5=CKP IC, TIM2/TIM8=OC injeção/ignição)
    // TIM3/TIM4 PWM auxiliares são inicializados em auxiliaries_init().
    // ECU_Hardware_Init() owns TIM2/TIM8 for injection/ignition scheduling.
    // misfire_init() DEVE preceder tim5_ic_init(): a tabela g_tooth_to_cyl parte de
    // BSS (zero), mas 0 é um índice de cilindro válido — o ISR do CKP leria cyl=0
    // para todos os dentes antes da tabela ser preenchida, gerando DTCs falsos.
    ems::engine::misfire_init();
    ems::hal::tim5_ic_init();   // → TIM5 input capture (CKP + CMP)
    iwdg_kick();

    // 2a) Scheduler unificado
    ::ECU_Hardware_Init();
    iwdg_kick();

    // 3) ADC (ADC1/ADC2 + TIM6 trigger)
    ems::hal::adc_init();
    iwdg_kick();

    // 4) CAN + bench communication. MVP transport: USART1 PA9/PA10.
    // (usb_cdc_init() já foi chamado cedo em 1b, antes dos inits da ECU.)
    ems::hal::can0_init();
    ems::hal::uart0_init(115200u);
    iwdg_kick();

	// 5) Flash Bank2 → carrega calibrações persistidas
	if (!ems::hal::nvm_load_calibration(0u, g_calib_page0, kCalibPageBytes)) {
		++g_flash_write_faults; // FIX: rastrear falha de leitura NVM
	}
	ems::engine::cfg::engine_config_load(g_calib_page0, kCalibPageBytes);
	load_corr_calibration_from_nvm();
	load_xtau_calibration_from_nvm();
	load_dwell2d_calibration_from_nvm();
	if (!ems::hal::nvm_load_adaptive_maps()) {
		++g_flash_write_faults; // FIX: rastrear falha de leitura NVM
	}
    {
        ems::hal::RuntimeSyncSeed seed = {};
        if (ems::hal::nvm_load_runtime_seed(&seed) &&
            ems::hal::runtime_seed_fast_reacquire_compatible_60_2(seed)) {
            const bool phase_a =
                ((seed.flags & ems::hal::RUNTIME_SYNC_SEED_FLAG_PHASE_A) != 0u);
            ems::drv::ckp_seed_arm(phase_a);
            g_runtime_seed_arm_window_active = true;
            g_runtime_seed_arm_window_start_ms = millis();
	if (!ems::hal::nvm_clear_runtime_seed()) {
		++g_flash_write_faults; // FIX: rastrear falha de limpeza NVM
	}
        }
    }

    // 6) Drivers
    ems::drv::sensors_init();
    iwdg_kick();

    // 6a) Inicializa sistemas "invisíveis" ao motorista
    ems::engine::map_estimator_init();
    ems::engine::xtau_autocalib_init();

    // 6b) Inicializa ETB e Torque Manager (borboleta eletrônica)
    g_etb_initialized = etb_control_init();
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
    ems::app::can_stack_init();

    // 9) NVIC — CKP fica com prioridade máxima. Injeção/ignição em TIM2/TIM8
    //    usam output compare direto por hardware, sem ISR no caminho crítico.
    //    SysTick configurado em system_stm32_init() com prio 11.
    nvic_set_priority(IRQ_TIM5, 1u);
    nvic_enable_irq(IRQ_TIM5);

    // 10) Aguardar CKP sync (timeout 5 s)
    const uint32_t sync_deadline = millis() + 5000u;
    while (millis() < sync_deadline) {
        iwdg_kick();
        const auto snap = ems::drv::ckp_snapshot();
        if (snap.state == ems::drv::SyncState::FULL_SYNC) { break; }
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

    // Estreitar IWDG de 10s (boot) para 100ms (runtime): o main loop kica a cada
    // ciclo; 100ms detecta travamento de runtime sem tolerar os inits longos do boot.
    // Nota: IWDG_PR segue /256 (boot) → RLR=99 dá ~0.8s efetivo; suficiente p/ runtime
    // e evita esperar PVU/RVU de novo no caminho crítico.
    IWDG_KR  = IWDG_KR_ACCESS;
    IWDG_RLR = IWDG_RLR_100MS;
    IWDG_KR  = IWDG_KR_REFRESH;

    for (;;) {
        // ── Watchdog kick (primeiro statement) ───────────────────────────
        iwdg_kick();
        g_datalog_us = micros();

        const uint32_t now = millis();

        // ── 2ms: fuel + ign recalc + commit calibration ───────────────────
        if (elapsed(now, g_t2ms_, 2u)) {
            g_t2ms_ = now;
            const uint32_t loop2ms_start_us = micros();

            // Stall watchdog: detecta virabrequim parado entre dentes.
            // Deve preceder ckp_snapshot() para que o snapshot deste ciclo
            // já reflicta LOSS_OF_SYNC se o motor parou.
            ems::drv::ckp_stall_poll(ems::hal::tim5_count());

            // Dwell watchdog: protege bobinas de ignição contra saturação.
            // Se SPARK não disparou dentro de 1.4 × dwell após DWELL_START,
            // força a saída LOW imediatamente (MS42 §2.2.2.1.3).
            ecu_sched_dwell_watchdog();

            const auto snap    = ems::drv::ckp_snapshot();
            const auto sensors = ems::drv::sensors_get();
            const bool full_sync = (snap.state == ems::drv::SyncState::FULL_SYNC);
            const bool sched_sync = (snap.state == ems::drv::SyncState::HALF_SYNC || full_sync);

            if (g_runtime_seed_arm_window_active) {
                if (elapsed(now, g_runtime_seed_arm_window_start_ms,
                            kRuntimeSeedArmWindowMs)) {
                    ems::drv::ckp_seed_disarm();
                    g_runtime_seed_arm_window_active = false;
                }
            }

            if (full_sync) {
                g_have_last_full_sync = true;
                g_last_full_sync_snapshot = snap;
            }
            if (sched_sync && snap.tooth_index == 0u) {
                g_have_last_gap_sync = true;
                g_last_gap_sync_snapshot = snap;
            }

            const uint16_t map_bar_x100_raw = static_cast<uint16_t>(sensors.map_bar_x1000 / 10u);
            const uint16_t map_bar_x100_sensor = clamp_u16(map_bar_x100_raw, kMapMinBarX100, kMapMaxBarX100);
            
            // Atualiza estimador de MAP com sensor fusion para resposta transiente rápida
            const uint16_t map_bar_x100 = ems::engine::map_estimator_update(
                map_bar_x100_sensor,
                sensors.tps_pct_x10,
                kAePeriodMs,
                snap.rpm_x10);
            
            // Loop ETB (1kHz = 1ms) - controle de borboleta eletrônica
            const bool map_fault = (sensors.fault_bits & kFaultBitMap) != 0u;
            const bool clt_fault = (sensors.fault_bits & kFaultBitClt) != 0u;
            g_limp_active = map_fault || clt_fault;
            const bool rev_cut = g_limp_active &&
                (snap.rpm_x10 > kLimpRpmLimit_x10);
            const CachedFuelCorrections& fuel_corr = fuel_corrections_for(sensors);
            // Dwell 2D: tensão × RPM (MS42 §2.2.2.2.1).
            // Calculado fora do cache porque depende de RPM que varia a cada dente.
            const uint16_t dwell_ms_x10 = ems::engine::dwell_ms_x10_from_vbatt_rpm(
                sensors.vbatt_mv, snap.rpm_x10);
            const uint32_t dwell_ticks =
                (static_cast<uint32_t>(dwell_ms_x10) * kSchedulerTicksPerMs) / 10u;

            // Multi-spark (MS42 §2.2.3): habilita/desabilita conforme RPM gate.
            // O dwell inter-spark é mais curto (tabela dedicada mspark_inter_dwell_ms_x10).
            // Limite 18°ATDC garante que o último spark contribui para a combustão.
            if (snap.rpm_x10 < ems::engine::mspark_max_rpm_x10 &&
                ems::engine::mspark_count > 0u) {
                const uint32_t inter_dwell_ticks =
                    (static_cast<uint32_t>(ems::engine::mspark_inter_dwell_ms_x10)
                     * kSchedulerTicksPerMs) / 10u;
                ::ecu_sched_set_mspark(ems::engine::mspark_count, inter_dwell_ticks, 18u);
            } else {
                ::ecu_sched_set_mspark(0u, 0u, 18u);
            }
            const bool crank_rpm_window =
                sched_sync && snap.rpm_x10 > 0u &&
                snap.rpm_x10 < ems::engine::crank_exit_rpm_x10;
            ems::engine::quick_crank_set_prime_context(sensors.clt_degc_x10,
                                                       fuel_corr.dead_time_us);

            // Limitador de RPM por faísca + injeção (MS42 §2.2.5)
            // Camada 1 (larga, ~500 RPM): retardo progressivo de ignição
            // Camada 2 (estreita, ~200 RPM): corte progressivo de injeção
            // Camada 3 (no limite): corte total de ambos
            {
                const uint32_t hard       = ems::engine::rev_limit_rpm_x10;
                const uint32_t inj_window = ems::engine::rev_limit_soft_window_x10;

                // Injeção — corte progressivo em 3 níveis:
                //   25% (cil 0) na banda inferior, 50% (cil 0+2) na superior, 100% no limite.
                // Saturating subtractions prevent unsigned underflow if a misconfigured
                // calibration sets inj_window > hard, which would wrap to ~4B RPM
                // and disable the rev limiter entirely.
                const uint32_t band75  = inj_window * 3u / 4u;
                const uint32_t thresh_50 = (hard > band75)      ? hard - band75      : 0u;
                const uint32_t thresh_25 = (hard > inj_window)  ? hard - inj_window  : 0u;
                uint8_t inj_mask = 0u;
                if (rev_cut || snap.rpm_x10 >= hard) {
                    inj_mask = 0x0Fu;
                } else if (inj_window > 0u && snap.rpm_x10 >= thresh_50) {
                    inj_mask = 0x05u;  // 50%: cil 0 + 2
                } else if (inj_window > 0u && snap.rpm_x10 >= thresh_25) {
                    inj_mask = 0x01u;  // 25%: apenas cil 0
                }
                ::ecu_sched_set_inj_inhibit_mask(inj_mask);

                // Ignição — retardo progressivo + corte alternado no limite
                g_rev_spark_trim =
                    rev_cut ? INT16_MIN :
                    ems::engine::calc_rev_limit_spark_trim(snap.rpm_x10);
                if (g_rev_spark_trim == INT16_MIN) {
                    if (rev_cut) {
                        // Limp mode (falha de sensor): corte total de ignição
                        ::ecu_sched_set_ign_inhibit_mask(0x0Fu);
                    } else {
                        // Limitador normal: alternado 0+2 / 1+3 para transição mais suave
                        static bool s_ign_toggle = false;
                        s_ign_toggle = !s_ign_toggle;
                        ::ecu_sched_set_ign_inhibit_mask(s_ign_toggle ? 0x05u : 0x0Au);
                    }
                } else {
                    ::ecu_sched_set_ign_inhibit_mask(0u);
                }
            }

            if (sched_sync && !rev_cut) {
                const ems::engine::Table2dLookup fuel_lookup =
                    ems::engine::table3d_prepare_lookup(ems::engine::kRpmAxisX10,
                                                        ems::engine::kLoadAxisBarX100,
                                                        snap.rpm_x10,
                                                        map_bar_x100);
                const uint8_t  ve = ems::engine::get_ve_prepared(fuel_lookup);
                const uint16_t lambda_target_x1000 =
                    ems::engine::get_lambda_target_x1000_prepared(fuel_lookup);
                const int16_t fuel_trim_pct_x10 = crank_rpm_window ? 0 : clamp_i16(
                    static_cast<int16_t>(ems::engine::fuel_get_stft_pct_x10() +
                                         ems::engine::fuel_get_ltft_pct_x10(fuel_lookup.yi, fuel_lookup.xi)),
                    -500, 500);
                const int32_t ae_pw_us = crank_rpm_window ? 0 : ems::engine::calc_ae_pw_us(
                    sensors.tps_pct_x10,
                    g_prev_tps_pct_x10,
                    kAePeriodMs,
                    sensors.clt_degc_x10);
                uint32_t final_pw_us_base =
                    ems::engine::calc_fuel_pw_us_default_fast(ve,
                                                               map_bar_x100,
                                                               lambda_target_x1000,
                                                               fuel_trim_pct_x10,
                                                               fuel_corr.corr_clt_x256,
                                                               fuel_corr.corr_iat_x256,
                                                               fuel_corr.dead_time_us);
                // Corte de combustível na desaceleração (MS42 TI_PUR).
                // Avaliado ANTES do X-Tau: evita alimentar o modelo de parede com PW
                // real e depois descartar o resultado, contaminando a auto-calibração.
                const bool decel_cut_active = !crank_rpm_window &&
                    ems::engine::fuel_decel_cut_update(
                        snap.rpm_x10, sensors.tps_pct_x10, sensors.clt_degc_x10);
                ems::engine::misfire_set_all_inhibit(decel_cut_active || crank_rpm_window);
                if (decel_cut_active) {
                    g_last_net_pw_us = 0u;
                    g_ae_active = false;
                    ems::engine::transient_fuel_reset();
                } else if (final_pw_us_base > fuel_corr.dead_time_us) {
                    uint32_t fuel_pw_us =
                        final_pw_us_base - static_cast<uint32_t>(fuel_corr.dead_time_us);

                    // LTFT aditivo (MS42 TI_AD_ADD_MMV): aplica offset no PW líquido
                    if (!crank_rpm_window) {
                        const int16_t ltft_add = ems::engine::fuel_get_ltft_add_us(
                            fuel_lookup.yi, fuel_lookup.xi);
                        const int32_t pw_adj = static_cast<int32_t>(fuel_pw_us) + ltft_add;
                        fuel_pw_us = (pw_adj <= 0) ? 0u
                                   : (pw_adj > 100000) ? 100000u
                                   : static_cast<uint32_t>(pw_adj);
                    }
                    g_last_net_pw_us = fuel_pw_us;

                    const bool xtau_enabled = !crank_rpm_window && (snap.rpm_x10 >= 7000u);

                    // Detecta transiente para auto-calibração X-τ
                    const bool is_transient = ems::engine::map_is_transient();

                    // Auto-calibração X-τ baseada em erro de lambda.
                    // Lambda vem exclusivamente do WBO2 via CAN (mesma fonte do
                    // STFT em 100ms); SensorData não carrega mais o2/lambda.
                    const bool lambda_valid = ems::app::can_stack_wbo2_fresh(now);
                    if (full_sync && !crank_rpm_window && lambda_valid) {
                        const uint16_t lambda_measured = clamp_u16(
                            ems::app::can_stack_lambda_milli_safe(now),
                            kLambdaMinMilli, kLambdaMaxMilli);
                        ems::engine::xtau_autocalib_update(
                            snap.rpm_x10,
                            map_bar_x100,
                            lambda_target_x1000,
                            static_cast<int16_t>(lambda_measured),
                            sensors.clt_degc_x10,
                            is_transient);
                    }

                    // Usa modelo X-τ com parâmetros aprendidos
                    const uint32_t xtau_fuel_pw_us =
                        ems::engine::transient_fuel_xtau_with_autocalib(fuel_pw_us,
                                                                        sensors.clt_degc_x10,
                                                                        xtau_enabled);
                    final_pw_us_base = xtau_fuel_pw_us + static_cast<uint32_t>(fuel_corr.dead_time_us);
                } else {
                    ems::engine::transient_fuel_reset();
                }
                if (!decel_cut_active && ae_pw_us > 0) {
                    const uint32_t ae_add = static_cast<uint32_t>(ae_pw_us);
                    final_pw_us_base = (ae_add >= 100000u || final_pw_us_base > (100000u - ae_add))
                        ? 100000u
                        : (final_pw_us_base + ae_add);
                    g_ae_active = true;
                }
                const int16_t base_advance_deg = ems::engine::get_advance_prepared(fuel_lookup);
                const uint16_t knock_retard_x10 = ems::engine::knock_get_retard_x10(0u);
                const uint16_t idle_target_rpm_x10 =
                    ems::engine::auxiliaries_idle_target_rpm_x10(sensors.clt_degc_x10);
                const int16_t idle_spark_corr_deg = crank_rpm_window ? 0 :
                    ems::engine::calc_idle_spark_correction_deg(snap.rpm_x10,
                                                                idle_target_rpm_x10,
                                                                sensors.tps_pct_x10,
                                                                map_bar_x100);
                const int16_t iat_spark_deg = crank_rpm_window ? 0 :
                    ems::engine::calc_ign_iat_correction_deg(sensors.iat_degc_x10);
                const int16_t clt_spark_deg = crank_rpm_window ? 0 :
                    ems::engine::calc_ign_clt_correction_deg(sensors.clt_degc_x10);
                const bool ae_now = (ae_pw_us > 0);
                const int16_t antijerk_retard = crank_rpm_window ? 0 :
                    ems::engine::calc_antijerk_retard_deg(ae_now);
                int16_t advance_deg = ems::engine::calc_total_advance(
                    base_advance_deg,
                    {iat_spark_deg, clt_spark_deg,
                     static_cast<int16_t>(knock_retard_x10 / 10u),
                     idle_spark_corr_deg, antijerk_retard});
                // Rev limiter spark retard: aplicado após todas as outras correções.
                // Se INT16_MIN (spark cut), ignição já foi suprimida via ign_inhibit_mask;
                // aqui apenas garante que o advance não seja avançado nesse ciclo.
                if (g_rev_spark_trim != INT16_MIN && g_rev_spark_trim != 0) {
                    advance_deg = ems::engine::clamp_advance_deg(
                        static_cast<int16_t>(advance_deg + g_rev_spark_trim));
                }

                const auto qc = ems::engine::quick_crank_update(
                    now, snap.rpm_x10, sched_sync, sensors.clt_degc_x10, advance_deg);
                // Decel cut: força PW=0 sem passar pelo quick_crank (que pode adicionar min_pw)
                const uint32_t final_pw_us = decel_cut_active ? 0u :
                    ems::engine::quick_crank_apply_pw_us(final_pw_us_base,
                                                         qc.fuel_mult_x256,
                                                         qc.min_pw_us);
                const uint32_t pw_100 = final_pw_us / 100u;
                g_last_pw_ms_x10 = static_cast<uint8_t>(pw_100 > 255u ? 255u : pw_100);
                g_last_advance_deg = clamp_i8(qc.spark_deg, -10, 40);

                const uint32_t inj_pw_ticks = ems::engine::inj_pw_us_to_scheduler_ticks(final_pw_us);

                ::ecu_sched_commit_calibration(
                    static_cast<uint32_t>(qc.spark_deg < 0 ? 0 : qc.spark_deg),
                    dwell_ticks,
                    inj_pw_ticks,
                    ems::engine::cfg::kDefaultSoiLeadDeg);
            } else if (sched_sync && rev_cut) {
                const int16_t base_advance_deg = ems::engine::get_advance(snap.rpm_x10, map_bar_x100);
                ::ecu_sched_commit_calibration(
                    static_cast<uint32_t>(base_advance_deg < 0 ? 0 : base_advance_deg),
                    dwell_ticks,
                    0u,
                    ems::engine::cfg::kDefaultSoiLeadDeg);
                g_last_pw_ms_x10 = 0u;
                g_ae_active = false;
            } else if (snap.rpm_x10 == 0u) {
                static_cast<void>(ems::engine::quick_crank_update(
                    now, snap.rpm_x10, false, sensors.clt_degc_x10, 0));
            }
            g_prev_tps_pct_x10 = sensors.tps_pct_x10;

            const uint32_t prime_pw = ems::engine::quick_crank_consume_prime();
            if (prime_pw != 0u) {
                ::ecu_sched_fire_prime_pulse(prime_pw);
            }

            g_loop2ms_last_us = micros() - loop2ms_start_us;
            if (g_loop2ms_last_us > g_loop2ms_max_us) {
                g_loop2ms_max_us = g_loop2ms_last_us;
            }
            ems::app::ui_update_loop_diag(g_loop2ms_last_us, g_loop2ms_max_us);
        }

        // ── 10ms: IACV, VVT, wastegate PID ───────────────────────────────
        if (elapsed(now, g_t10ms_, 10u)) {
            g_t10ms_ = now;
            ems::engine::auxiliaries_tick_10ms();
        }

        // ── 20ms: UI proprietaria + aux tasks ───────────────────────────
        if (elapsed(now, g_t20ms_, 20u)) {
            g_t20ms_ = now;
            const auto snap = ems::drv::ckp_snapshot();
            const auto sensors = ems::drv::sensors_get();
            ems::app::ui_update_rt_metrics(g_last_pw_ms_x10, g_last_advance_deg, g_last_stft_pct);
            ems::app::ui_update_rt_sched_diag(
                g_late_event_count,
                g_cycle_schedule_drop_count,
                g_calibration_clamp_count,
                ems::drv::ckp_seed_loaded_count(),
                ems::drv::ckp_seed_confirmed_count(),
                ems::drv::ckp_seed_rejected_count(),
                static_cast<uint8_t>(snap.state));
            ems::app::ui_update_ivc_diag(::ecu_sched_ivc_clamp_count());
            ui_service();
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
                    if (ems::engine::misfire_get_event_count(c) >=
                        ems::engine::kMisfireFaultThreshold) {
                        ems::engine::DiagnosticManager::report_fault(
                            kMisfireCodes[c],
                            ems::engine::FaultSeverity::WARNING);
                        ems::engine::misfire_clear_events(c);
                    }
                }
            }

            if (snap.state == ems::drv::SyncState::FULL_SYNC) {
                const uint16_t map_bar_x100 = clamp_u16(
                    static_cast<uint16_t>(sensors.map_bar_x1000 / 10u), kMapMinBarX100, kMapMaxBarX100);
                const uint16_t lambda_measured = clamp_u16(
                    ems::app::can_stack_lambda_milli_safe(now), kLambdaMinMilli, kLambdaMaxMilli);
                const bool lambda_valid = ems::app::can_stack_wbo2_fresh(now);
                const uint16_t lambda_target_x1000 =
                    ems::engine::get_lambda_target_x1000(snap.rpm_x10, map_bar_x100);
                const bool rev_cut = g_limp_active &&
                    (snap.rpm_x10 > kLimpRpmLimit_x10);
                const bool ae_active = g_ae_active;
                // STFT congelado em qualquer condição de corte intencional de combustível:
                // - rev_cut: limp mode
                // - decel_cut: borboleta fechada em desaceleração
                // - inj_inhibit_mask != 0: rev limiter cortou injeção em ≥1 cilindro
                //   (lambda leria lean sem combustível → STFT aprenderia errado)
                const bool stft_inhibit = rev_cut ||
                    ems::engine::fuel_decel_cut_active() ||
                    (::ecu_sched_get_inj_inhibit_mask() != 0u);
                const int16_t stft = ems::engine::fuel_update_stft_delayed(
                    now, snap.rpm_x10, map_bar_x100,
                    static_cast<int16_t>(lambda_target_x1000),
                    static_cast<int16_t>(lambda_measured),
                    sensors.clt_degc_x10, lambda_valid,
                    ae_active, stft_inhibit, g_last_net_pw_us);
                g_ae_active = false;
                g_last_stft_pct = clamp_i8(static_cast<int16_t>(stft / 10), -25, 25);

                // X-tau autocalibration (100ms lambda-gated)
                const bool xtau_learning_ok = (stft >= -500 && stft <= 500 &&
                    lambda_valid && snap.rpm_x10 >= 2000u);
                ems::engine::xtau_autocalib_update(
                    snap.rpm_x10,
                    clamp_u16(static_cast<uint16_t>(sensors.map_bar_x1000 / 10u),
                              kMapMinBarX100, kMapMaxBarX100),
                    static_cast<int16_t>(lambda_target_x1000),
                    static_cast<int16_t>(ems::app::can_stack_lambda_milli_safe(now)),
                    sensors.clt_degc_x10,
                    xtau_learning_ok);
            } else {
                g_last_stft_pct = 0;
            }

            // Runtime seed — salva posição para re-sincronização rápida
            const uint32_t rpm = snap.rpm_x10;
            if (rpm > 0u) {
                g_engine_was_running = true;
                g_zero_rpm_since_ms  = 0u;
                g_runtime_seed_saved_for_stop = false;
            } else {
                if (g_engine_was_running && g_zero_rpm_since_ms == 0u) {
                    g_zero_rpm_since_ms = now;
                }
                if (g_engine_was_running && !g_runtime_seed_saved_for_stop &&
                    g_zero_rpm_since_ms != 0u &&
                    elapsed(now, g_zero_rpm_since_ms, kRuntimeSeedSaveDelayMs) &&
                    g_have_last_gap_sync) {
                    const auto seed_snap = g_last_gap_sync_snapshot;
                    ems::hal::RuntimeSyncSeed seed = {};
                    seed.flags = static_cast<uint8_t>(
                        ems::hal::RUNTIME_SYNC_SEED_FLAG_VALID |
                        ems::hal::RUNTIME_SYNC_SEED_FLAG_FULL_SYNC |
                        (seed_snap.phase_A
                             ? ems::hal::RUNTIME_SYNC_SEED_FLAG_PHASE_A : 0u));
                    seed.tooth_index = seed_snap.tooth_index;
                    seed.decoder_tag =
                        ems::hal::RUNTIME_SYNC_SEED_DECODER_TAG_60_2;
	if (!ems::hal::nvm_save_runtime_seed(&seed)) {
		// FIX: não descartar retorno — falha de flash deve ser rastreada
		++g_flash_write_faults; // fault counter para diagnóstico
	}
	g_runtime_seed_saved_for_stop = true;
                }
            }
        }

        // ── 500ms: agenda flush Flash + LED heartbeat (PB2 WeAct) ──────────
        // PB2 = LED da placa WeAct STM32H562; toggle confirma firmware a correr.
        // GPIOB já tem clock habilitado em system_stm32_init().
        // FIX P0: Only allow flash writes when engine is stopped or below safe RPM
        static bool adaptive_flush_pending = false;
        static uint32_t last_calib_save_ms = 0u;
        if (elapsed(now, g_t500ms_, 500u)) {
            g_t500ms_ = now;
            // LED heartbeat: toggle PB2 a cada 500ms (1 Hz)
            // MODER PB2 = output (01b at bits [5:4])
            GPIOB_MODER = (GPIOB_MODER & ~(3u << 4u)) | (1u << 4u);
            GPIOB_ODR ^= (1u << 2u);  // toggle PB2
            const auto snap = ems::drv::ckp_snapshot();
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
                adaptive_flush_pending = true;
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

        // ── ETB control (2ms cadence) ─────────────────────────────────────
        if (elapsed(now, g_t_etb_ms, 2u)) {
            g_t_etb_ms = now;
            const auto sensors_etb = ems::drv::sensors_get();
            const auto snap_etb = ems::drv::ckp_snapshot();
            const bool etb_rev_cut = g_limp_active && (snap_etb.rpm_x10 > kLimpRpmLimit_x10);
            const auto torque_out = ems::engine::torque_manager_update(
                snap_etb, sensors_etb, true, g_limp_active, etb_rev_cut,
                ems::engine::auxiliaries_idle_target_rpm_x10(sensors_etb.clt_degc_x10), 2u);
            (void)ems::engine::etb_control_update(
                torque_out.etb_target_pct_x10, sensors_etb.etb_tps_pct_x10,
                torque_out.etb_enable_request, 2u);
        }
    }
}

#endif  // TARGET_STM32H562
