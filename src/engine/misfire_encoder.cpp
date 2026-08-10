#include "engine/misfire_encoder.h"
#include "engine/misfire_detect.h"
#include "engine/engine_config.h"
#include "engine/ecu_sched.h"
#include "hal/board_pinout.h"

namespace ems::engine {

namespace {

constexpr uint32_t kMisfireEncoderWindowDeg      = 62u;  // ~10/58×360°, paridade com a janela CKP (10 dentes)
constexpr uint32_t kMisfireEncoderSubticksPerRev = 64u;  // 64×256 = 16384, exacto
constexpr uint32_t kSubTickCounts                = 16384u / kMisfireEncoderSubticksPerRev;  // 256

// [phase][sub_tick_idx] → cilindro (0-3) ou -1 (fora de qualquer janela).
// phase: 0=ECU_PHASE_A, 1=ECU_PHASE_B.
int8_t g_cyl_window[2][kMisfireEncoderSubticksPerRev];

int32_t  g_active_cyl      = -1;  // -1 = nenhuma janela activa
uint32_t g_power_ticks_sum = 0u;  // Δticks TIM5 reais acumulados na janela activa
uint32_t g_pred_ticks_sum  = 0u;  // Δticks previstos (tendência) acumulados

uint32_t g_prev_tim2        = 0u;
uint32_t g_prev_tim5        = 0u;
bool     g_have_prev        = false;

// Histórico de Δticks para a previsão por tendência — precisa de DOIS
// pontos anteriores (t-1, t-2) para extrapolar uma tendência e prever t
// ANTES de ver a amostra real de t, exactamente como
// predict_next_period_ticks() (ckp.cpp) prevê o próximo dente a partir dos
// dois últimos, não do que está a chegar agora.
uint32_t g_delta_t1          = 0u;  // Δticks em t-1
uint32_t g_delta_t2          = 0u;  // Δticks em t-2
uint8_t  g_delta_hist_count  = 0u;  // 0, 1 ou 2 (satura)

uint8_t g_event_count[4] = {};
uint8_t g_debounce[4]    = {};
bool    g_all_inhibit    = false;

// Previsão do Δticks da amostra actual, a partir SÓ do histórico anterior
// (t-1, t-2) — nunca do valor actual, senão deixaria de ser previsão.
// Tendência linear análoga a predict_next_period_ticks() (ckp.cpp), mas
// nova e local (domínio sub-tick, não dente). Mesmo clamp ±12.5%
// (kPredictionClampDen=8 lá).
uint32_t predict_current_delta_ticks() noexcept {
    if (g_delta_hist_count < 1u) { return 0u; }       // sem histórico: sem previsão útil
    if (g_delta_hist_count < 2u) { return g_delta_t1; }  // 1 ponto: previsão de ordem zero
    int32_t trend = static_cast<int32_t>(g_delta_t1) - static_cast<int32_t>(g_delta_t2);
    const int32_t clamp = static_cast<int32_t>(g_delta_t1 / 8u);
    if (trend > clamp) { trend = clamp; }
    if (trend < -clamp) { trend = -clamp; }
    const int32_t predicted = static_cast<int32_t>(g_delta_t1) + trend;
    return (predicted > 0) ? static_cast<uint32_t>(predicted) : g_delta_t1;
}

// Avalia a janela do cilindro `cyl` que acabou de fechar (saída detectada).
// O incremento do contador DTC-facing fica atrás de phase_valid() E
// EMS_MISFIRE_ENCODER_ENABLE — a lógica de threshold/debounce em si corre
// sempre, testável em host sem build especial (mesmo padrão de
// drv/encoder_sync.cpp e da fase do dispatcher encoder).
void evaluate_window(uint8_t cyl) noexcept {
    if (g_pred_ticks_sum == 0u) { return; }  // sem amostras suficientes
    const uint64_t threshold =
        (static_cast<uint64_t>(g_pred_ticks_sum) * kMisfireThresholdQ8) >> 8u;
    if (static_cast<uint64_t>(g_power_ticks_sum) <= threshold) {
        g_debounce[cyl] = 0u;
        return;
    }
    uint8_t d = static_cast<uint8_t>(g_debounce[cyl] + 1u);
    if (d >= kMisfireDebounceCycles) {
        d = 0u;
        if (ecu_sched_encoder_phase_valid() != 0u && EMS_MISFIRE_ENCODER_ENABLE) {
            if (g_event_count[cyl] < 255u) { ++g_event_count[cyl]; }
        }
    }
    g_debounce[cyl] = d;
}

}  // namespace

void misfire_encoder_init() noexcept {
    for (auto& row : g_cyl_window) {
        for (auto& v : row) { v = -1; }
    }
    const uint32_t origin_mod360 =
        static_cast<uint32_t>(cfg::g_eng_cfg.trigger_tooth0_engine_deg) % 360u;
    const uint32_t window_span_counts = (kMisfireEncoderWindowDeg * 16384u) / 360u;
    const uint32_t span_buckets =
        (window_span_counts + kSubTickCounts - 1u) / kSubTickCounts;

    for (uint8_t cyl = 0u; cyl < 4u; ++cyl) {
        const uint16_t tdc_deg = cfg::cyl_tdc_deg(cyl);  // 0..719
        const uint8_t phase_idx = (tdc_deg < 360u) ? 0u : 1u;
        const uint32_t crank_deg =
            (static_cast<uint32_t>(tdc_deg) % 360u + 360u - origin_mod360) % 360u;
        const uint32_t start_counts = (crank_deg * 16384u) / 360u;
        const uint32_t start_bucket = start_counts / kSubTickCounts;
        for (uint32_t b = 0u; b < span_buckets; ++b) {
            const uint32_t bucket = (start_bucket + b) % kMisfireEncoderSubticksPerRev;
            g_cyl_window[phase_idx][bucket] = static_cast<int8_t>(cyl);
        }
    }
    misfire_encoder_reset();
}

void misfire_encoder_reset() noexcept {
    g_active_cyl        = -1;
    g_power_ticks_sum   = 0u;
    g_pred_ticks_sum    = 0u;
    g_have_prev         = false;
    g_delta_t1          = 0u;
    g_delta_t2          = 0u;
    g_delta_hist_count  = 0u;
    g_all_inhibit       = false;
    for (uint8_t i = 0u; i < 4u; ++i) {
        g_event_count[i] = 0u;
        g_debounce[i]    = 0u;
    }
}

void misfire_encoder_set_all_inhibit(bool inhibit) noexcept { g_all_inhibit = inhibit; }

uint8_t misfire_encoder_get_event_count(uint8_t cyl) noexcept {
    return (cyl < 4u) ? g_event_count[cyl] : 0u;
}

void misfire_encoder_clear_events(uint8_t cyl) noexcept {
    if (cyl < 4u) { g_event_count[cyl] = 0u; }
}

void misfire_encoder_on_sample(uint32_t tim2_now, uint32_t tim5_now) noexcept {
    if (!g_have_prev) {
        g_prev_tim2 = tim2_now;
        g_prev_tim5 = tim5_now;
        g_have_prev = true;
        return;
    }
    const uint32_t delta_ticks = tim5_now - g_prev_tim5;  // wrap-safe (unsigned)
    g_prev_tim2 = tim2_now;
    g_prev_tim5 = tim5_now;
    if (delta_ticks == 0u) { return; }

    // Previsão calculada ANTES de incorporar a amostra actual no histórico —
    // senão estaríamos a "prever" com o próprio valor que queremos avaliar.
    const uint32_t predicted_ticks = predict_current_delta_ticks();
    g_delta_t2 = g_delta_t1;
    g_delta_t1 = delta_ticks;
    if (g_delta_hist_count < 2u) { ++g_delta_hist_count; }

    if (ecu_sched_encoder_phase_valid() == 0u) { return; }  // sem fase, sem cilindro identificável

    const uint8_t phase_idx = (ecu_sched_encoder_phase_at(tim2_now) == ECU_PHASE_A) ? 0u : 1u;
    const uint32_t bucket = (tim2_now & 0x3FFFu) / kSubTickCounts;
    const int8_t cyl = g_cyl_window[phase_idx][bucket];

    if (static_cast<int32_t>(cyl) != g_active_cyl) {
        if (g_active_cyl >= 0 && !g_all_inhibit) {
            evaluate_window(static_cast<uint8_t>(g_active_cyl));
        }
        g_active_cyl      = cyl;
        g_power_ticks_sum = 0u;
        g_pred_ticks_sum  = 0u;
    }

    if (cyl >= 0 && !g_all_inhibit) {
        g_power_ticks_sum += delta_ticks;
        g_pred_ticks_sum  += predicted_ticks;
    }
}

#if defined(EMS_HOST_TEST)
uint8_t misfire_encoder_test_get_debounce(uint8_t cyl) noexcept {
    return (cyl < 4u) ? g_debounce[cyl] : 0u;
}

int8_t misfire_encoder_test_cyl_at(uint8_t phase_idx, uint32_t tim2_pos) noexcept {
    if (phase_idx > 1u) { return -1; }
    const uint32_t bucket = (tim2_pos & 0x3FFFu) / kSubTickCounts;
    return g_cyl_window[phase_idx][bucket];
}
#endif

}  // namespace ems::engine
