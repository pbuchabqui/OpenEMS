/**
 * @file engine/knock.cpp
 * @brief Knock detection via ADC (PA5 / ADC1_IN6).
 *
 * O2 sensor input foi migrado para CAN exclusivamente, liberando o canal
 * ADC1_IN6 (PA5) para o sensor de knock (piezo + filtro passa-banda externo).
 *
 * Hardware: sensor piezo → filtro BP externo → PA5 / ADC1_IN6.
 * Software: sample_fast_channels() chama knock_adc_update(raw) 12×/rev
 * (acumulador kFastSamplesPerRev, não a cada dente) enquanto a janela
 * estiver ativa; knock_cycle_complete() aplica retard/recovery.
 *
 * Sem periférico COMP interno (STM32H562 não o possui) — detecção 100% em
 * software a partir das amostras ADC.
 */

#include "engine/knock.h"

#include <cstdint>

#include "engine/calibration.h"
#include "engine/math_utils.h"
#include "engine/ms42_cal.h"
#include "drv/ckp.h"
#include "hal/flash.h"

namespace {

using ems::engine::clamp_u16;

// ── Constantes do algoritmo ───────────────────────────────────────────────────
constexpr uint8_t  kDefaultEventThreshold  = 3u;    // amostras acima do threshold por janela
// Passo, máximo, recuperação e ciclos limpos: bloco MS42 (ms42.knock_*),
// defaults 2,0° / 10,0° / 0,1° / 10 ciclos.
// Modo relativo (MS42 S10): ruído p2p por cilindro aprendido nas janelas sem
// detonação; só detecta após kRelLearnWindows janelas de aprendizagem.
constexpr uint8_t  kRelLearnWindows        = 16u;
constexpr uint16_t kAdcThresholdDefault    = 2048u; // 12-bit mid-range; ajustar em bancada
constexpr uint16_t kAdcThresholdMin        = 256u;  // evita falsos positivos por ruído de offset
constexpr uint16_t kAdcThresholdMax        = 4000u; // abaixo de 4095 para margem

// ── Estado ────────────────────────────────────────────────────────────────────
struct KnockState {
    uint8_t  knock_count[ems::engine::kKnockCylinders]; // amostras acima do threshold na janela atual
    uint8_t  clean_cycles[ems::engine::kKnockCylinders];
    uint8_t  event_threshold;
    uint8_t  global_clean_cycles;
    uint8_t  window_cyl;
    bool     window_active;
    uint16_t adc_threshold; // 12-bit: amostra acima disto conta como evento de knock
    uint16_t base_threshold; // calibrated threshold; adc_threshold returns to it
    // Sensor morto (FOME #578): pico-a-pico do raw por janela; um piezo vivo
    // tem sempre ruído de fundo — EMA do p2p abaixo do piso por muitas
    // janelas = sensor desligado/curto.
    uint16_t win_min;         // min do raw na janela corrente
    uint16_t win_max;         // max do raw na janela corrente
    uint16_t noise_p2p_ema;   // EMA (α=1/8) do p2p por janela
    uint16_t dead_windows;    // janelas consecutivas abaixo do piso
    uint32_t cyl_noise_x16[ems::engine::kKnockCylinders];  // ruído p2p ×16
    uint8_t  cyl_noise_n[ems::engine::kKnockCylinders];    // janelas aprendidas
};

constexpr uint16_t kDeadWindowLimit = 100u;  // ~100 eventos de combustão

static KnockState g = {};

// ── Utilitários ───────────────────────────────────────────────────────────────

}  // namespace

namespace ems::engine {

volatile uint16_t knock_retard_x10[kKnockCylinders] = {};

// ── API pública ───────────────────────────────────────────────────────────────

void knock_init() noexcept {
    g = {};
    g.event_threshold = kDefaultEventThreshold;
    g.adc_threshold   = kAdcThresholdDefault;
    g.base_threshold  = kAdcThresholdDefault;

    // Carrega retard persistido do NVM (rpm_i=0, load_i=cyl, 4 células)
    for (uint8_t i = 0u; i < kKnockCylinders; ++i) {
        const int8_t stored = ems::hal::nvm_read_knock(0u, i);
        knock_retard_x10[i] = (stored > 0)
            ? clamp_u16(static_cast<uint16_t>(stored), 0u, ems::engine::ms42.knock_max_x10)
            : 0u;
    }
    // The ADC threshold is calibration, not adaptive state: it is not
    // persisted (older firmware stored a value that drifted to the maximum).
}

void knock_save_to_nvm() noexcept {
    for (uint8_t i = 0u; i < kKnockCylinders; ++i) {
        const int8_t val = static_cast<int8_t>(
            knock_retard_x10[i] > 127u ? 127u : knock_retard_x10[i]);
        ems::hal::nvm_write_knock(0u, i, val);
    }
}

void knock_set_event_threshold(uint8_t threshold) noexcept {
    g.event_threshold = threshold;
}

void knock_set_adc_threshold(uint16_t threshold) noexcept {
    g.adc_threshold = clamp_u16(threshold, kAdcThresholdMin, kAdcThresholdMax);
    g.base_threshold = g.adc_threshold;
}

uint16_t knock_get_adc_threshold() noexcept {
    return g.adc_threshold;
}

void knock_window_open(uint8_t cyl) noexcept {
    g.window_cyl    = static_cast<uint8_t>(cyl & 0x3u);
    g.window_active = true;
    // Reset counter for this cylinder so we start clean each window
    g.knock_count[g.window_cyl] = 0u;
    g.win_min = 0xFFFFu;
    g.win_max = 0u;
}

void knock_window_close(uint8_t cyl) noexcept {
    if ((g.window_cyl == static_cast<uint8_t>(cyl & 0x3u)) && g.window_active) {
        g.window_active = false;
    }
}

void knock_adc_update(uint16_t raw) noexcept {
    // Called from sample_fast_channels() (CKP tooth ISR) on every tooth while
    // window is active. Counts samples that exceed the detection threshold.
    if (!g.window_active) { return; }
    if (raw < g.win_min) { g.win_min = raw; }
    if (raw > g.win_max) { g.win_max = raw; }
    if (raw > g.adc_threshold) {
        const uint8_t c = g.window_cyl;
        if (g.knock_count[c] < 255u) {
            ++g.knock_count[c];
        }
    }
}

void knock_cycle_complete(uint8_t cyl) noexcept {
    const uint8_t c = static_cast<uint8_t>(cyl & 0x3u);

    // Read and zero atomically — knock_adc_update() can run from CKP ISR
    // concurrently; CPSID prevents a torn read/zero on non-atomic uint8_t.
#if defined(__arm__) || defined(__thumb__)
    __asm__ volatile("cpsid i" ::: "memory");
#endif
    const uint8_t count = g.knock_count[c];
    g.knock_count[c] = 0u;
#if defined(__arm__) || defined(__thumb__)
    __asm__ volatile("cpsie i" ::: "memory");
#endif

    // p2p da janela que fechou (só se houve amostras; sentinela reposta para
    // não reavaliar a mesma janela).
    const Ms42Cal& cal = ems::engine::ms42;
    const bool have_p2p = (g.win_min <= g.win_max);
    const uint16_t p2p = have_p2p ? static_cast<uint16_t>(g.win_max - g.win_min) : 0u;
    g.win_min = 0xFFFFu;
    g.win_max = 0u;

    // Sensor morto: EMA do p2p abaixo do piso por muitas janelas.
    if (ems::engine::knock_dead_min_p2p != 0u && have_p2p) {
        g.noise_p2p_ema = static_cast<uint16_t>(
            static_cast<int32_t>(g.noise_p2p_ema) +
            (static_cast<int32_t>(p2p) -
             static_cast<int32_t>(g.noise_p2p_ema)) / 8);
        if (g.noise_p2p_ema < ems::engine::knock_dead_min_p2p) {
            if (g.dead_windows < 0xFFFFu) { ++g.dead_windows; }
        } else {
            g.dead_windows = 0u;
        }
    }

    bool knocked = false;
    if (cal.knock_rel_enable != 0u) {
        if (have_p2p) {
            const uint32_t p_x16 = static_cast<uint32_t>(p2p) * 16u;
            if (g.cyl_noise_n[c] >= kRelLearnWindows) {
                const uint16_t rpm100 = static_cast<uint16_t>(
                    ems::drv::ckp_snapshot().rpm_x10 / 1000u);
                const uint32_t gain_x10 = ems::engine::ms42_interp_u8(
                    cal.knock_gain_rpm_axis, cal.knock_gain_x10, kKnockGainPts, rpm100);
                knocked = (p_x16 * 10u) > (gain_x10 * g.cyl_noise_x16[c]);
            }
            if (!knocked) {
                if (g.cyl_noise_n[c] == 0u) {
                    g.cyl_noise_x16[c] = p_x16;
                } else {
                    const int32_t diff = static_cast<int32_t>(p_x16) -
                                         static_cast<int32_t>(g.cyl_noise_x16[c]);
                    g.cyl_noise_x16[c] = static_cast<uint32_t>(
                        static_cast<int32_t>(g.cyl_noise_x16[c]) +
                        diff / (1 << cal.knock_noise_shift));
                }
                if (g.cyl_noise_n[c] < 255u) { ++g.cyl_noise_n[c]; }
            }
        }
    } else {
        knocked = (count > g.event_threshold);
    }

    if (knocked) {
        // Knock detected: add retard, reset clean cycle counter
        const uint16_t next = static_cast<uint16_t>(knock_retard_x10[c] + cal.knock_step_x10);
        knock_retard_x10[c]  = clamp_u16(next, 0u, cal.knock_max_x10);
        g.clean_cycles[c]    = 0u;
        g.global_clean_cycles = 0u;

        // Slightly lower threshold to stay sensitive during knock conditions
        if (cal.knock_rel_enable == 0u && g.adc_threshold > kAdcThresholdMin + 64u) {
            g.adc_threshold = static_cast<uint16_t>(g.adc_threshold - 64u);
        }
    } else {
        // Clean cycle: accumulate toward recovery
        if (g.clean_cycles[c] < 255u) { ++g.clean_cycles[c]; }

        if (g.clean_cycles[c] >= cal.knock_clean_cycles) {
            knock_retard_x10[c] = (knock_retard_x10[c] > cal.knock_recovery_x10)
                ? static_cast<uint16_t>(knock_retard_x10[c] - cal.knock_recovery_x10)
                : 0u;
        }

        if (g.global_clean_cycles < 255u) { ++g.global_clean_cycles; }
        if (g.global_clean_cycles >= 100u) {
            // After a knock episode lowered it, step back toward the
            // calibrated threshold — never above it (detection must not
            // silently turn itself off during long clean running).
            if (g.adc_threshold < g.base_threshold) {
                const uint16_t up = static_cast<uint16_t>(g.adc_threshold + 32u);
                g.adc_threshold = (up > g.base_threshold) ? g.base_threshold : up;
            }
            g.global_clean_cycles = 0u;
        }
    }
}

void knock_window_cycle_end() noexcept {
    if (g.window_active) {
        g.window_active = false;
        knock_cycle_complete(g.window_cyl);
    }
}

bool knock_window_open_for(uint8_t cyl) noexcept {
    return g.window_active && (g.window_cyl == static_cast<uint8_t>(cyl & 0x3u));
}

void knock_window_cycle_end_if_cyl_mask(uint8_t cyl_mask) noexcept {
    if (g.window_active &&
        ((static_cast<uint8_t>(1u << g.window_cyl) & cyl_mask) != 0u)) {
        knock_window_cycle_end();
    }
}

bool knock_sensor_dead() noexcept {
    return (ems::engine::knock_dead_min_p2p != 0u) &&
           (g.dead_windows >= kDeadWindowLimit);
}

uint16_t knock_get_retard_x10(uint8_t cyl) noexcept {
    return knock_retard_x10[static_cast<uint8_t>(cyl & 0x3u)];
}

#if defined(EMS_HOST_TEST)
uint8_t knock_test_get_knock_count(uint8_t cyl) noexcept {
    return g.knock_count[static_cast<uint8_t>(cyl & 0x3u)];
}
bool knock_test_window_active() noexcept { return g.window_active; }
uint8_t knock_test_window_cyl() noexcept { return g.window_cyl; }
void knock_test_set_adc_raw(uint16_t raw) noexcept { knock_adc_update(raw); }
uint16_t knock_test_get_noise_p2p_ema() noexcept { return g.noise_p2p_ema; }
uint32_t knock_test_get_cyl_noise_x16(uint8_t cyl) noexcept {
    return g.cyl_noise_x16[static_cast<uint8_t>(cyl & 0x3u)];
}
#endif

}  // namespace ems::engine
