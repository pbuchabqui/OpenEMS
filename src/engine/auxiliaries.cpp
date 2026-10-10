#include "engine/auxiliaries.h"

#include <cstdint>

#include "engine/output_test.h"

#include "engine/math_utils.h"
#include "engine/table3d.h"
#include "engine/calibration.h"
#include "engine/vehicle_inputs.h"
#include "engine/ms42_cal.h"
#include "engine/diagnostic_manager.h"

#if __has_include("drv/ckp.h")
#include "drv/ckp.h"
#elif __has_include("ckp.h")
#include "ckp.h"
#endif

#if __has_include("drv/sensors.h")
#include "drv/sensors.h"
#elif __has_include("sensors.h")
#include "sensors.h"
#endif

#include "hal/timer.h"
#include "hal/regs.h"
#include "hal/board_pinout.h"  // EMS_BOARD_IS_VGT6 — selecciona os pinos de relé

#if defined(EMS_HOST_TEST)
volatile uint32_t ems_test_aux_rcc_ahb2enr1 = 0u;
volatile uint32_t ems_test_aux_gpiob_moder = 0u;
volatile uint32_t ems_test_aux_gpiob_bsrr = 0u;
#define RCC_AHB2ENR1 ems_test_aux_rcc_ahb2enr1
#define RCC_AHB2ENR1_GPIOBEN (1u << 1u)
#define GPIOB_MODER ems_test_aux_gpiob_moder
#define GPIOB_BSRR ems_test_aux_gpiob_bsrr
// No VGT6 os relés vivem em GPIOE; os testes de host observam as mesmas
// variáveis, por isso o alias aponta para os mesmos registos simulados.
#define RCC_AHB2ENR1_GPIOEEN (1u << 4u)
#define GPIOE_MODER ems_test_aux_gpiob_moder
#define GPIOE_BSRR ems_test_aux_gpiob_bsrr
#endif

namespace {

using ems::engine::clamp_i16;

constexpr uint32_t kTick10ms = 10u;
constexpr uint32_t kTick20ms = 20u;
constexpr uint32_t kAuxTim3PwmHz = 15u;
constexpr uint32_t kAuxTim4PwmHz = 15u;

constexpr int16_t kDrpmEnableMaxX10PerSec = 2000;

constexpr uint32_t kOverboostDurationMs = 500u;
constexpr uint16_t kOverboostMarginBarX1000 = 200u;

// VVT: sem borda de came nova há mais do que isto → solenoide em repouso.
constexpr uint32_t kVvtEdgeTimeoutMs = 200u;
// Aprendizagem do ângulo de repouso: só perto da marcha lenta, com o
// solenoide sem corrente há pelo menos kVvtParkSettleMs (came encostado).
constexpr uint32_t kVvtLearnMaxRpmX10 = 25000u;
constexpr uint32_t kVvtParkSettleMs = 1000u;
constexpr int32_t kVvtIntegratorMax = 300;   // ‰ duty
// Variação máxima do alvo por tick de 10 ms (0,1° × 10 = 1°/10 ms).
constexpr int16_t kVvtTargetSlewX10 = 10;

constexpr uint32_t kPumpPrimeMs = 2000u;
// 2 s (era 3 s): corta a bomba mais cedo num acidente sem cortar prematuramente
// num calo momentâneo do motor.
constexpr uint32_t kPumpOffDelayMs = 2000u;

// Idle target RPM vs CLT — curva compartilhada com ETB idle spark
#define kWarmupPts         ems::engine::kIacWarmupPts
#define kWarmupCltAxisX10  ems::engine::iac_clt_axis_x10
#define kIdleTargetRpmX10  ems::engine::iac_idle_target_rpm_x10

// Boost target: usa global calibrável boost_target_bar_x1000[7][8] de calibration.h
// Eixo RPM fixo (abaixo). Eixo Y = marcha inteira — índice direto sem interpolação.
constexpr uint8_t kBoostRpmPts = 8u;
constexpr uint8_t kBoostGears  = 7u;
constexpr uint32_t kBoostRpmAxisX10[kBoostRpmPts] = {
    15000u, 20000u, 25000u, 30000u, 40000u, 50000u, 65000u, 80000u
};

// ── Bomba de combustível e ventoinha ────────────────────────────────────────
// ⚠️ VGT6: PE12 = ventoinha, PE10 = bomba.
// RGT6 (sem GPIOE): PB12 = ventoinha, PB13 = bomba.
// Escrita por BSRR, que é set/reset atómico por bit e não perturba os canais
// de INJ/IGN no mesmo porto.
#if EMS_BOARD_IS_VGT6
constexpr uint8_t kFanPin  = 12u;  // PE12
constexpr uint8_t kPumpPin = 10u;  // PE10
#define EMS_AUX_RELAY_BSRR  GPIOE_BSRR
#define EMS_AUX_RELAY_MODER GPIOE_MODER
#define EMS_AUX_RELAY_RCC_EN() (RCC_AHB2ENR1 |= RCC_AHB2ENR1_GPIOEEN)
#else
constexpr uint8_t kFanPin  = 12u;  // PB12
constexpr uint8_t kPumpPin = 13u;  // PB13
#define EMS_AUX_RELAY_BSRR  GPIOB_BSRR
#define EMS_AUX_RELAY_MODER GPIOB_MODER
#define EMS_AUX_RELAY_RCC_EN() (RCC_AHB2ENR1 |= RCC_AHB2ENR1_GPIOBEN)
#endif

constexpr uint32_t kFanBit = (1u << kFanPin);
constexpr uint32_t kPumpBit = (1u << kPumpPin);

struct AuxState {
    bool key_on;
    bool engine_running;
    uint32_t running_since_ms;
    bool fan_on;
    bool pump_on;

    // FIX-9: volatile — incrementado nos slots periódicos do background loop;
    // sem volatile, o compilador pode elevar leituras para fora de estruturas
    // de controle, observando sempre o mesmo valor em comparações de timeout.
    volatile uint32_t time_ms;
    uint32_t key_on_ms;
    uint32_t rpm_zero_since_ms;

    uint16_t wg_duty_x10;
    int16_t wg_integrator_x10;
    uint32_t wg_overboost_ms;
    bool wg_failsafe;
    uint16_t ewg_position_demand_x10;

    uint16_t vvt_esc_duty_x10;
    uint16_t vvt_adm_duty_x10;
    int32_t vvt_adm_integrator_x10;
    uint32_t vvt_last_seq;
    uint32_t vvt_last_edge_ms;
    uint32_t vvt_parked_since_ms;
    uint16_t vvt_ref_learned_x10;      // 0 = ainda não aprendido
    int16_t vvt_adv_x10;               // avanço medido filtrado (° vir. ×10)
    int16_t vvt_target_x10;            // alvo após slew
    bool vvt_have_meas;
    bool vvt_active;

    uint16_t vbatt_low_ticks;          // ticks de 10 ms consecutivos fora da faixa
    uint16_t vbatt_high_ticks;
    uint16_t vbatt_ok_ticks;
};

static AuxState g = {};

// gear: 0=neutro/desconhecido, 1-6; índice direto na tabela (sem interpolação no eixo Y)
uint16_t lookup_boost_target(uint32_t rpm_x10, uint8_t gear) noexcept {
    const uint8_t g  = (gear >= kBoostGears) ? (kBoostGears - 1u) : gear;
    const uint8_t xi = ems::engine::table_axis_index(kBoostRpmAxisX10, kBoostRpmPts, rpm_x10);
    const uint8_t fx = ems::engine::table_axis_frac_q8(kBoostRpmAxisX10, xi, rpm_x10);

    const int32_t v0 = static_cast<int32_t>(ems::engine::boost_target_bar_x1000[g][xi]);
    const int32_t v1 = static_cast<int32_t>(ems::engine::boost_target_bar_x1000[g][xi + 1u]);
    const int32_t v  = ems::engine::lerp_q8_s32(v0, v1, fx);

    if (v <= 0) {
        return 0u;
    }
    if (v >= 65535) {
        return 65535u;
    }
    return static_cast<uint16_t>(v);
}

// Alvo de avanço (° virabrequim ×10) da tabela 6×6 calibrável [carga][rpm].
int16_t lookup_vvt_target_x10(uint32_t rpm_x10, uint16_t map_kpa) noexcept {
    const ems::engine::Ms42Cal& c = ems::engine::ms42;
    const uint16_t rpm100 = static_cast<uint16_t>(rpm_x10 / 1000u);
    // Interpolação em rpm em cada linha de carga, depois em carga.
    uint8_t col[ems::engine::kVvtCalPts];
    for (uint8_t y = 0u; y < ems::engine::kVvtCalPts; ++y) {
        col[y] = static_cast<uint8_t>(ems::engine::ms42_interp_u8(
            c.vvt_rpm_axis, c.vvt_target_deg[y], ems::engine::kVvtCalPts, rpm100));
    }
    // ×10 antes de interpolar em carga (resolução de 0,1°).
    const uint8_t* ax = c.vvt_load_axis;
    int32_t v;
    if (map_kpa <= ax[0]) {
        v = col[0] * 10;
    } else if (map_kpa >= ax[ems::engine::kVvtCalPts - 1u]) {
        v = col[ems::engine::kVvtCalPts - 1u] * 10;
    } else {
        uint8_t i = 1u;
        while (map_kpa > ax[i]) { ++i; }
        const int32_t x0 = ax[i - 1u];
        const int32_t x1 = ax[i];
        v = col[i - 1u] * 10 +
            ((col[i] - col[i - 1u]) * 10 * (static_cast<int32_t>(map_kpa) - x0)) / (x1 - x0);
    }
    const int32_t vmax = static_cast<int32_t>(c.vvt_max_adv_deg) * 10;
    if (v > vmax) { v = vmax; }
    if (v < 0) { v = 0; }
    return static_cast<int16_t>(v);
}

void set_fan(bool on) noexcept {
    g.fan_on = on;
    if (on) {
        EMS_AUX_RELAY_BSRR = kFanBit;
    } else {
        EMS_AUX_RELAY_BSRR = (kFanBit << 16u);
    }
}

void set_pump(bool on) noexcept {
    g.pump_on = on;
    if (on) {
        EMS_AUX_RELAY_BSRR = kPumpBit;
    } else {
        EMS_AUX_RELAY_BSRR = (kPumpBit << 16u);
    }
}

uint16_t iac_target_rpm_x10(int16_t clt_x10) noexcept {
    return ems::engine::interp_u16_8pt(kWarmupCltAxisX10, kIdleTargetRpmX10, kWarmupPts, clt_x10);
}

void run_wastegate_control(const ems::drv::CkpSnapshot& snap,
                           const ems::drv::SensorData& s) noexcept {
    uint8_t gear = 0u;
    (void)ems::engine::vehicle_gear(gear, g.time_ms);  // fallback 0 se CAN off
    const uint16_t target_bar_x1000 = lookup_boost_target(snap.rpm_x10, gear);

    if (s.map_bar_x1000 > static_cast<uint16_t>(target_bar_x1000 + kOverboostMarginBarX1000)) {
        g.wg_overboost_ms += kTick20ms;
    } else {
        g.wg_overboost_ms = 0u;
        g.wg_failsafe = false;
    }

    if (g.wg_overboost_ms >= kOverboostDurationMs) {
        g.wg_failsafe = true;
    }

    if (g.wg_failsafe) {
        g.wg_duty_x10 = 1000u;  // full open on overboost
        g.wg_integrator_x10 = 0;
        g.ewg_position_demand_x10 = 1000u;
        return;
    }

    const int32_t error = static_cast<int32_t>(target_bar_x1000) - static_cast<int32_t>(s.map_bar_x1000);
    const int32_t p_x10 = (error * 8) / 100;
    g.wg_integrator_x10 = clamp_i16(
        static_cast<int16_t>(g.wg_integrator_x10 + static_cast<int16_t>(error / 100)),
        -250,
        250);

    int32_t out = p_x10 + g.wg_integrator_x10;
    if (out < 0) {
        out = 0;
    }
    if (out > 1000) {
        out = 1000;
    }

    g.wg_duty_x10 = static_cast<uint16_t>(out);
    // EWG cascata: outer loop output = position demand for inner PID (2ms loop)
    g.ewg_position_demand_x10 = g.wg_duty_x10;
}

// Avanço = referência − ângulo medido, em ±180°: borda mais cedo = came
// avançado (MS42 S15: ângulo de referência + desvio).
int16_t wrap_adv_x10(int32_t d) noexcept {
    while (d > 1800) { d -= 3600; }
    while (d < -1800) { d += 3600; }
    return static_cast<int16_t>(d);
}

void vvt_park() noexcept {
    if (g.vvt_active) {
        g.vvt_parked_since_ms = g.time_ms;
    }
    g.vvt_active = false;
    g.vvt_adm_duty_x10 = 0u;
    g.vvt_adm_integrator_x10 = 0;
    ems::hal::tim4_set_duty(1u, 0u);
}

// VVT só de admissão (uma entrada de came). A borda do CMP medida pelo driver
// CKP dá a fase real; o PI corre sobre o avanço medido, não sobre o ângulo do
// virabrequim. Sem borda nova, sincronismo, óleo frio ou VVT desligado → duty
// 0 (repouso mecânico, MS42 c_cam_ini).
void run_vvt_control(const ems::drv::CkpSnapshot& snap,
                     const ems::drv::SensorData& s) noexcept {
    const ems::engine::Ms42Cal& c = ems::engine::ms42;

    // Escape: sem sensor de came de escape nesta placa → sempre desligado.
    g.vvt_esc_duty_x10 = 0u;
    ems::hal::tim4_set_duty(0u, 0u);

    uint16_t meas_x10 = 0u;
    const uint32_t seq = ems::drv::ckp_cam_edge_angle(meas_x10);
    const bool fresh = (seq != g.vvt_last_seq);
    if (fresh) {
        g.vvt_last_seq = seq;
        g.vvt_last_edge_ms = g.time_ms;
    }
    const bool signal_ok =
        (snap.state == ems::drv::SyncState::FULL_SYNC) && (seq != 0u) &&
        ((g.time_ms - g.vvt_last_edge_ms) <= kVvtEdgeTimeoutMs);
    if (!signal_ok) {
        g.vvt_have_meas = false;
        vvt_park();
        return;
    }

    // Referência: calibrada ou aprendida com o solenoide em repouso.
    if (fresh && !g.vvt_active && snap.rpm_x10 <= kVvtLearnMaxRpmX10 &&
        (g.time_ms - g.vvt_parked_since_ms) >= kVvtParkSettleMs) {
        if (g.vvt_ref_learned_x10 == 0u) {
            g.vvt_ref_learned_x10 = (meas_x10 == 0u) ? 1u : meas_x10;
        } else {
            // EMA 1/8 no domínio circular.
            const int16_t d = wrap_adv_x10(static_cast<int32_t>(meas_x10) -
                                           static_cast<int32_t>(g.vvt_ref_learned_x10));
            int32_t r = static_cast<int32_t>(g.vvt_ref_learned_x10) + d / 8;
            if (r <= 0) { r += 3600; }
            if (r >= 3600) { r -= 3600; }
            g.vvt_ref_learned_x10 = static_cast<uint16_t>(r == 0 ? 1 : r);
        }
    }
    const uint16_t ref_x10 = (c.vvt_cam_ref_x10 != 0u) ? c.vvt_cam_ref_x10
                                                       : g.vvt_ref_learned_x10;
    if (ref_x10 == 0u) {
        // Ainda sem referência: não há como medir o avanço.
        g.vvt_have_meas = false;
        vvt_park();
        return;
    }
    if (fresh) {
        const int16_t adv = wrap_adv_x10(static_cast<int32_t>(ref_x10) -
                                         static_cast<int32_t>(meas_x10));
        if (!g.vvt_have_meas) {
            g.vvt_adv_x10 = adv;
            g.vvt_have_meas = true;
        } else {
            g.vvt_adv_x10 = static_cast<int16_t>(
                g.vvt_adv_x10 + (adv - g.vvt_adv_x10) / 4);
        }
    }

    if (c.vvt_enable == 0u || s.clt_degc_x10 < c.vvt_min_clt_x10) {
        vvt_park();
        g.vvt_target_x10 = 0;
        return;
    }

    // Alvo com slew (evita degrau no óleo do atuador).
    const uint16_t map_kpa = static_cast<uint16_t>(s.map_bar_x1000 / 10u);
    const int16_t want = lookup_vvt_target_x10(snap.rpm_x10, map_kpa);
    if (want > g.vvt_target_x10 + kVvtTargetSlewX10) {
        g.vvt_target_x10 = static_cast<int16_t>(g.vvt_target_x10 + kVvtTargetSlewX10);
    } else if (want < g.vvt_target_x10 - kVvtTargetSlewX10) {
        g.vvt_target_x10 = static_cast<int16_t>(g.vvt_target_x10 - kVvtTargetSlewX10);
    } else {
        g.vvt_target_x10 = want;
    }

    const int32_t err = static_cast<int32_t>(g.vvt_target_x10) - g.vvt_adv_x10;
    const int32_t p = (err * static_cast<int32_t>(c.vvt_kp_x10)) / 10;
    int32_t integ = g.vvt_adm_integrator_x10 +
                    (err * static_cast<int32_t>(c.vvt_ki_x100)) / 100;
    if (integ > kVvtIntegratorMax) { integ = kVvtIntegratorMax; }
    if (integ < -kVvtIntegratorMax) { integ = -kVvtIntegratorMax; }
    g.vvt_adm_integrator_x10 = integ;

    int32_t out = static_cast<int32_t>(c.vvt_hold_duty_pct) * 10 + p + integ;
    if (out < 0) { out = 0; }
    if (out > 1000) { out = 1000; }
    g.vvt_active = true;
    g.vvt_adm_duty_x10 = static_cast<uint16_t>(out);
    ems::hal::tim4_set_duty(1u, g.vvt_adm_duty_x10);
}

void run_fan_control(int16_t clt_x10) noexcept {
    if (!g.fan_on && clt_x10 >= ems::engine::ms42.fan_on_x10) {
        set_fan(true);
    } else if (g.fan_on && clt_x10 <= ems::engine::ms42.fan_off_x10) {
        set_fan(false);
    }
}

// DTC de tensão (MS42 S19): baixa só com o motor a trabalhar (na partida a
// queda é normal), alta sempre; 2 s fora da faixa para ativar, 2 s dentro
// para limpar.
constexpr uint16_t kVbattDebounceTicks = 200u;
constexpr uint32_t kVbattRunRpmX10 = 5000u;

void run_vbatt_diag(uint32_t rpm_x10, uint16_t vbatt_mv) noexcept {
    using ems::engine::DiagnosticCode;
    using ems::engine::DiagnosticManager;
    const uint16_t low_mv = ems::engine::ms42.vbatt_low_mv;
    const uint16_t high_mv = ems::engine::ms42.vbatt_high_mv;
    const bool low = (low_mv != 0u) && (rpm_x10 >= kVbattRunRpmX10) && (vbatt_mv < low_mv);
    const bool high = (high_mv != 0u) && (vbatt_mv > high_mv);

    g.vbatt_low_ticks = low ? static_cast<uint16_t>(g.vbatt_low_ticks + (g.vbatt_low_ticks < 0xFFFFu)) : 0u;
    g.vbatt_high_ticks = high ? static_cast<uint16_t>(g.vbatt_high_ticks + (g.vbatt_high_ticks < 0xFFFFu)) : 0u;
    g.vbatt_ok_ticks = (!low && !high)
        ? static_cast<uint16_t>(g.vbatt_ok_ticks + (g.vbatt_ok_ticks < 0xFFFFu)) : 0u;

    if (g.vbatt_low_ticks == kVbattDebounceTicks) {
        DiagnosticManager::report_fault(DiagnosticCode::VBATT_LOW,
                                        ems::engine::FaultSeverity::WARNING, vbatt_mv);
    }
    if (g.vbatt_high_ticks == kVbattDebounceTicks) {
        DiagnosticManager::report_fault(DiagnosticCode::VBATT_HIGH,
                                        ems::engine::FaultSeverity::WARNING, vbatt_mv);
    }
    if (g.vbatt_ok_ticks == kVbattDebounceTicks) {
        DiagnosticManager::clear_fault(DiagnosticCode::VBATT_LOW);
        DiagnosticManager::clear_fault(DiagnosticCode::VBATT_HIGH);
    }
}

void run_pump_control(uint32_t rpm_x10) noexcept {
    if (!g.key_on) {
        set_pump(false);
        g.rpm_zero_since_ms = g.time_ms;
        return;
    }

    if ((g.time_ms - g.key_on_ms) < kPumpPrimeMs) {
        set_pump(true);
        return;
    }

    if (rpm_x10 > 0u) {
        set_pump(true);
        g.rpm_zero_since_ms = g.time_ms;
        return;
    }

    if ((g.time_ms - g.rpm_zero_since_ms) >= kPumpOffDelayMs) {
        set_pump(false);
    } else {
        set_pump(true);
    }
}

void reset_state() noexcept {
    g = AuxState{};
}

}  // namespace

namespace ems::engine {

// Alvo de marcha lenta + acréscimo para aquecer o catalisador (MS42 E17E):
// cat_heat_rpm logo após a partida, a decair linearmente até 0 em cat_heat_s.
uint16_t auxiliaries_idle_target_rpm_x10(int16_t clt_x10) noexcept {
    uint32_t target = iac_target_rpm_x10(clt_x10);
    const uint32_t dur_ms = static_cast<uint32_t>(ms42.cat_heat_s) * 1000u;
    if (g.engine_running && ms42.cat_heat_rpm_x10 != 0u && dur_ms != 0u) {
        const uint32_t t = g.time_ms - g.running_since_ms;
        if (t < dur_ms) {
            target += (static_cast<uint32_t>(ms42.cat_heat_rpm_x10) * (dur_ms - t)) / dur_ms;
        }
    }
    return static_cast<uint16_t>(target > 0xFFFFu ? 0xFFFFu : target);
}

void auxiliaries_init() noexcept {
    reset_state();

    // NÃO inicializar o TIM3 aqui: ele é dedicado à injeção (OC em PC6-9).
    // O motor EWG (wastegate) usa o TIM2_CH3/PB10 via ewg_driver. Antes, este
    // tim3_pwm_init reescrevia o ARR do TIM3 e quebrava o timing dos injetores.
    ems::hal::tim4_pwm_init(kAuxTim4PwmHz);   // TIM4: VVT (CH1 exhaust, CH2 intake)
    ems::hal::tim4_set_duty(0u, 0u);
    ems::hal::tim4_set_duty(1u, 0u);

    EMS_AUX_RELAY_RCC_EN();
    EMS_AUX_RELAY_MODER =
        (EMS_AUX_RELAY_MODER & ~(3u << (kFanPin * 2u))) | (1u << (kFanPin * 2u));
    EMS_AUX_RELAY_MODER =
        (EMS_AUX_RELAY_MODER & ~(3u << (kPumpPin * 2u))) | (1u << (kPumpPin * 2u));

    set_fan(false);
    set_pump(false);
}

void auxiliaries_set_key_on(bool key_on) noexcept {
    if (key_on && !g.key_on) {
        g.key_on = true;
        g.key_on_ms = g.time_ms;
        g.rpm_zero_since_ms = g.time_ms;
        return;
    }

    if (!key_on && g.key_on) {
        g.key_on = false;
        g.key_on_ms = 0u;
        g.rpm_zero_since_ms = g.time_ms;
        set_pump(false);
    }
}

void auxiliaries_tick_10ms() noexcept {
    g.time_ms += kTick10ms;

    // Teste de saídas em bancada é dono de VVT/fan/pump — só o relógio corre.
    if (output_test_active()) { return; }

    const ems::drv::CkpSnapshot snap = ems::drv::ckp_snapshot();
    const ems::drv::SensorData s = ems::drv::sensors_get();  // cópia atômica

    run_vvt_control(snap, s);
    run_fan_control(s.clt_degc_x10);
    run_pump_control(snap.rpm_x10);
    run_vbatt_diag(snap.rpm_x10, s.vbatt_mv);

    // Motor a trabalhar (≥ 500 rpm) desde quando — para o aquecimento do cat.
    if (snap.rpm_x10 == 0u) {
        g.engine_running = false;
    } else if (!g.engine_running && snap.rpm_x10 >= 5000u) {
        g.engine_running = true;
        g.running_since_ms = g.time_ms;
    }
}

void auxiliaries_tick_20ms() noexcept {
    if (output_test_active()) { return; }

    const ems::drv::CkpSnapshot snap = ems::drv::ckp_snapshot();
    const ems::drv::SensorData s = ems::drv::sensors_get();  // cópia atômica

    run_wastegate_control(snap, s);
    run_fan_control(s.clt_degc_x10);
    run_pump_control(snap.rpm_x10);
}

void auxiliaries_force_pump(bool on) noexcept { set_pump(on); }
void auxiliaries_force_fan(bool on) noexcept { set_fan(on); }

uint16_t auxiliaries_ewg_position_demand_x10() noexcept {
    return g.ewg_position_demand_x10;
}

int16_t auxiliaries_vvt_advance_x10() noexcept {
    return g.vvt_have_meas ? g.vvt_adv_x10 : 0;
}

int16_t auxiliaries_vvt_target_x10() noexcept {
    return g.vvt_active ? g.vvt_target_x10 : 0;
}

uint16_t auxiliaries_vvt_ref_x10() noexcept {
    return (ems::engine::ms42.vvt_cam_ref_x10 != 0u)
        ? ems::engine::ms42.vvt_cam_ref_x10 : g.vvt_ref_learned_x10;
}

#if defined(EMS_HOST_TEST)
void auxiliaries_test_reset() noexcept {
    auxiliaries_init();
}

uint16_t auxiliaries_test_get_wg_duty() noexcept {
    return g.wg_duty_x10;
}

uint16_t auxiliaries_test_get_vvt_esc_duty() noexcept {
    return g.vvt_esc_duty_x10;
}

uint16_t auxiliaries_test_get_vvt_adm_duty() noexcept {
    return g.vvt_adm_duty_x10;
}

bool auxiliaries_test_get_fan_state() noexcept {
    return g.fan_on;
}

bool auxiliaries_test_get_pump_state() noexcept {
    return g.pump_on;
}

bool auxiliaries_test_get_wg_failsafe() noexcept {
    return g.wg_failsafe;
}
#endif

}  // namespace ems::engine
