/**
 * @file etb_autocal.cpp
 * @brief Auto-calibração dos limites do ETB TPS a cada power-on.
 */

#include "etb_autocal.h"
#include "engine/calibration.h"
#include "drv/sensors.h"
#include "hal/etb_driver.h"
#include "hal/flash.h"

namespace ems::engine {

namespace {

// PWM da varredura: ~39% de 1023 — vence a mola de retorno sem bater com
// força nos batentes (output test usa 30% para pulsos manuais).
constexpr int16_t  kSweepPwm          = 400;
// Batente considerado atingido: raw varia ≤ kSettleDelta por kSettleTicks
// ticks consecutivos (25 × 2 ms = 50 ms estável).
constexpr uint16_t kSettleDeltaCounts = 6u;
constexpr uint16_t kSettleTicks       = 25u;
// Tempo mínimo de fase antes de aceitar settle: o motor pode demorar a
// arrancar do repouso e a estabilidade inicial seria um falso batente.
constexpr uint16_t kMinPhaseMs        = 400u;
constexpr uint16_t kPhaseTimeoutMs    = 1500u;
constexpr uint16_t kReleaseMs         = 200u;
// Curso mínimo plausível entre batentes (defaults manuais: ~3700 counts).
constexpr uint16_t kMinSpanCounts     = 800u;
// Só regrava o registro NVM se algum limite mudou mais que isto (wear).
constexpr uint16_t kPersistDeltaCounts = 12u;

EtbAutocalState g_state = EtbAutocalState::Idle;
EtbAutocalFail  g_fail  = EtbAutocalFail::None;

uint16_t g_phase_ms     = 0u;
uint16_t g_settle_count = 0u;
uint16_t g_prev_raw1    = 0u;

uint16_t g_closed_raw1 = 0u, g_closed_raw2 = 0u;
uint16_t g_open_raw1   = 0u, g_open_raw2   = 0u;

void finish_fail(EtbAutocalFail reason) noexcept {
    etb_driver_shutdown();
    g_fail  = reason;
    g_state = EtbAutocalState::Failed;
}

void enter_phase(EtbAutocalState st) noexcept {
    g_state        = st;
    g_phase_ms     = 0u;
    g_settle_count = 0u;
    g_prev_raw1    = 0u;
}

// Amostra os dois TPS; devolve false (e aborta) em fault do driver.
bool sample(uint16_t& raw1, uint16_t& raw2) noexcept {
    etb_driver_data_t data{};
    if (etb_driver_read_sensors(&data) != ETB_DRV_OK) {
        finish_fail(EtbAutocalFail::DriverFault);
        return false;
    }
    raw1 = data.tps1_raw;
    raw2 = data.tps2_raw;
    return true;
}

// true quando o raw estabilizou (batente mecânico atingido).
bool settled(uint16_t raw1) noexcept {
    const uint16_t delta = (raw1 > g_prev_raw1) ? (raw1 - g_prev_raw1)
                                                : (g_prev_raw1 - raw1);
    if (g_settle_count > 0u && delta <= kSettleDeltaCounts) {
        ++g_settle_count;
    } else {
        g_settle_count = 1u;   // 1ª amostra da janela (ou movimento retomou)
    }
    g_prev_raw1 = raw1;
    return g_settle_count >= kSettleTicks;
}

void apply_limits(uint16_t t1_min, uint16_t t1_max,
                  uint16_t t2_min, uint16_t t2_max) noexcept {
    etb_tps1_raw_min = t1_min;
    etb_tps1_raw_max = t1_max;
    etb_tps2_raw_min = t2_min;
    etb_tps2_raw_max = t2_max;
    etb_cal_valid    = 1u;
    ems::drv::sensors_set_etb_tps_cal(t1_min, t1_max, t2_min, t2_max);
}

inline uint16_t abs_delta(uint16_t a, uint16_t b) noexcept {
    return (a > b) ? static_cast<uint16_t>(a - b) : static_cast<uint16_t>(b - a);
}

void apply_calibration() noexcept {
    apply_limits(g_closed_raw1, g_open_raw1, g_closed_raw2, g_open_raw2);

    // Persiste como fallback do próximo power-on (partida antes da varredura
    // terminar cai nesta última cal, não nos defaults de flash). Só regrava
    // se algo mudou além do ruído — poupa erases do setor adaptativo.
    ems::hal::EtbCalRecord last{};
    const bool have_last = ems::hal::nvm_load_etb_cal(&last);
    if (!have_last ||
        abs_delta(last.tps1_min, g_closed_raw1) > kPersistDeltaCounts ||
        abs_delta(last.tps1_max, g_open_raw1)   > kPersistDeltaCounts ||
        abs_delta(last.tps2_min, g_closed_raw2) > kPersistDeltaCounts ||
        abs_delta(last.tps2_max, g_open_raw2)   > kPersistDeltaCounts) {
        ems::hal::EtbCalRecord rec{};
        rec.tps1_min = g_closed_raw1;
        rec.tps1_max = g_open_raw1;
        rec.tps2_min = g_closed_raw2;
        rec.tps2_max = g_open_raw2;
        (void)ems::hal::nvm_save_etb_cal(&rec);
    }
}

}  // namespace

void etb_autocal_start() noexcept {
    g_fail = EtbAutocalFail::None;
    if (etb_harness_present == 0u ||
        etb_driver_get_state() != ETB_DRV_STATE_READY) {
        g_state = EtbAutocalState::Skipped;
        return;
    }
    // Fallback imediato: aplica a última auto-cal persistida ANTES de varrer.
    // Se a partida interromper a varredura, a ECU já opera com a cal do
    // power-on anterior em vez dos valores de flash da página 0.
    ems::hal::EtbCalRecord last{};
    if (ems::hal::nvm_load_etb_cal(&last)) {
        apply_limits(last.tps1_min, last.tps1_max, last.tps2_min, last.tps2_max);
    }
    enter_phase(EtbAutocalState::Closing);
}

bool etb_autocal_active() noexcept {
    return g_state == EtbAutocalState::Closing ||
           g_state == EtbAutocalState::Opening ||
           g_state == EtbAutocalState::Release;
}

void etb_autocal_tick(uint16_t period_ms, uint32_t rpm_x10) noexcept {
    if (!etb_autocal_active()) {
        return;
    }
    // Segurança: nunca varrer com o motor girando.
    if (rpm_x10 != 0u) {
        finish_fail(EtbAutocalFail::EngineTurning);
        return;
    }

    g_phase_ms = static_cast<uint16_t>(g_phase_ms + period_ms);

    switch (g_state) {
    case EtbAutocalState::Closing: {
        etb_driver_set_motor_pwm(static_cast<int16_t>(-kSweepPwm));
        uint16_t raw1, raw2;
        if (!sample(raw1, raw2)) { return; }
        if (settled(raw1) && g_phase_ms >= kMinPhaseMs) {
            g_closed_raw1 = raw1;
            g_closed_raw2 = raw2;
            enter_phase(EtbAutocalState::Opening);
        } else if (g_phase_ms >= kPhaseTimeoutMs) {
            finish_fail(EtbAutocalFail::Timeout);
        }
        return;
    }
    case EtbAutocalState::Opening: {
        etb_driver_set_motor_pwm(kSweepPwm);
        uint16_t raw1, raw2;
        if (!sample(raw1, raw2)) { return; }
        if (settled(raw1) && g_phase_ms >= kMinPhaseMs) {
            g_open_raw1 = raw1;
            g_open_raw2 = raw2;
            etb_driver_shutdown();   // solta — mola volta ao repouso
            enter_phase(EtbAutocalState::Release);
        } else if (g_phase_ms >= kPhaseTimeoutMs) {
            finish_fail(EtbAutocalFail::Timeout);
        }
        return;
    }
    case EtbAutocalState::Release: {
        if (g_phase_ms < kReleaseMs) {
            return;
        }
        // Validação: aberto > fechado com curso plausível nos dois sensores.
        if (g_open_raw1 <= g_closed_raw1 || g_open_raw2 <= g_closed_raw2) {
            finish_fail(EtbAutocalFail::Inverted);
            return;
        }
        if ((g_open_raw1 - g_closed_raw1) < kMinSpanCounts ||
            (g_open_raw2 - g_closed_raw2) < kMinSpanCounts) {
            finish_fail(EtbAutocalFail::SpanTooSmall);
            return;
        }
        apply_calibration();
        g_state = EtbAutocalState::Done;
        return;
    }
    default:
        return;
    }
}

EtbAutocalState etb_autocal_state() noexcept { return g_state; }
EtbAutocalFail  etb_autocal_fail_reason() noexcept { return g_fail; }

#if defined(EMS_HOST_TEST)
void etb_autocal_test_reset() noexcept {
    g_state = EtbAutocalState::Idle;
    g_fail  = EtbAutocalFail::None;
    g_phase_ms = 0u;
    g_settle_count = 0u;
    g_prev_raw1 = 0u;
    g_closed_raw1 = g_closed_raw2 = 0u;
    g_open_raw1 = g_open_raw2 = 0u;
}
#endif

}  // namespace ems::engine
