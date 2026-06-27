#include "engine/auxiliaries.h"

#include <cstdint>

#include "engine/table3d.h"
#include "engine/calibration.h"
#include "app/can_rx_map.h"

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

#if defined(EMS_HOST_TEST)
volatile uint32_t ems_test_aux_rcc_ahb2enr1 = 0u;
volatile uint32_t ems_test_aux_gpiob_moder = 0u;
volatile uint32_t ems_test_aux_gpiob_bsrr = 0u;
#define RCC_AHB2ENR1 ems_test_aux_rcc_ahb2enr1
#define RCC_AHB2ENR1_GPIOBEN (1u << 1u)
#define GPIOB_MODER ems_test_aux_gpiob_moder
#define GPIOB_BSRR ems_test_aux_gpiob_bsrr
#endif

namespace {

constexpr uint32_t kTick10ms = 10u;
constexpr uint32_t kTick20ms = 20u;
constexpr uint32_t kAuxTim3PwmHz = 15u;
constexpr uint32_t kAuxTim4PwmHz = 15u;

constexpr int16_t kDrpmEnableMaxX10PerSec = 2000;

constexpr uint32_t kOverboostDurationMs = 500u;
constexpr uint16_t kOverboostMarginBarX1000 = 200u;

constexpr uint32_t kVvtConfirmTimeoutMs = 200u;

constexpr int16_t kFanOnDegCX10 = 950;
constexpr int16_t kFanOffDegCX10 = 900;

constexpr uint32_t kPumpPrimeMs = 2000u;
constexpr uint32_t kPumpOffDelayMs = 3000u;

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

constexpr uint8_t kVvtPts = 12u;
constexpr uint32_t kVvtRpmAxisX10[kVvtPts] = {10000u, 15000u, 20000u, 25000u, 30000u, 35000u, 40000u, 45000u, 50000u, 60000u, 70000u, 80000u};
constexpr uint16_t kVvtLoadAxisBarX1000[kVvtPts] = {300u, 400u, 500u, 600u, 700u, 800u, 900u, 1000u, 1100u, 1200u, 1400u, 1700u};

constexpr int16_t kVvtAdmTargetDegX10[kVvtPts][kVvtPts] = {
    {180, 180, 170, 160, 150, 140, 130, 120, 110, 100, 90, 80},
    {200, 200, 190, 180, 170, 160, 150, 140, 130, 120, 100, 90},
    {220, 220, 210, 200, 190, 180, 170, 160, 145, 130, 115, 100},
    {240, 240, 230, 220, 210, 200, 185, 170, 155, 140, 120, 105},
    {260, 260, 250, 240, 225, 210, 195, 180, 165, 145, 125, 110},
    {280, 280, 270, 255, 240, 225, 210, 195, 175, 155, 130, 115},
    {300, 300, 285, 270, 255, 240, 225, 205, 185, 165, 140, 120},
    {315, 315, 300, 285, 270, 250, 230, 210, 190, 170, 145, 125},
    {320, 320, 305, 290, 275, 255, 235, 215, 195, 175, 150, 130},
    {325, 325, 310, 295, 280, 260, 240, 220, 200, 180, 155, 135},
    {330, 330, 315, 300, 285, 265, 245, 225, 205, 185, 160, 140},
    {330, 330, 315, 300, 285, 265, 245, 225, 205, 185, 160, 140},
};

constexpr int16_t kVvtEscTargetDegX10[kVvtPts][kVvtPts] = {
    {60, 60, 70, 80, 90, 100, 105, 110, 115, 120, 125, 130},
    {70, 70, 80, 90, 100, 110, 115, 120, 125, 130, 135, 140},
    {80, 80, 90, 100, 110, 120, 125, 130, 135, 140, 145, 150},
    {90, 90, 100, 110, 120, 130, 135, 140, 145, 150, 155, 160},
    {100, 100, 110, 120, 130, 140, 145, 150, 155, 160, 165, 170},
    {110, 110, 120, 130, 140, 150, 155, 160, 165, 170, 175, 180},
    {120, 120, 130, 140, 150, 160, 165, 170, 175, 180, 185, 190},
    {130, 130, 140, 150, 160, 170, 175, 180, 185, 190, 195, 200},
    {140, 140, 150, 160, 170, 180, 185, 190, 195, 200, 205, 210},
    {150, 150, 160, 170, 180, 190, 195, 200, 205, 210, 215, 220},
    {160, 160, 170, 180, 190, 200, 205, 210, 215, 220, 225, 230},
    {160, 160, 170, 180, 190, 200, 205, 210, 215, 220, 225, 230},
};

constexpr uint8_t kFanPin = 12u;
constexpr uint8_t kPumpPin = 13u;
constexpr uint32_t kFanBit = (1u << kFanPin);
constexpr uint32_t kPumpBit = (1u << kPumpPin);

struct AuxState {
    bool key_on;
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
    int16_t vvt_esc_integrator_x10;
    int16_t vvt_adm_integrator_x10;

    bool phase_prev;
    uint32_t vvt_last_phase_toggle_ms;
};

static AuxState g = {};

int16_t clamp_i16(int16_t v, int16_t lo, int16_t hi) noexcept {
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

uint8_t axis_index_u16(const uint16_t* axis, uint8_t size, uint16_t x) noexcept {
    if (size < 2u) {
        return 0u;
    }
    if (x <= axis[0]) {
        return 0u;
    }
    const uint8_t last = static_cast<uint8_t>(size - 1u);
    if (x >= axis[last]) {
        return static_cast<uint8_t>(last - 1u);
    }
    uint8_t idx = 0u;
    while (idx < static_cast<uint8_t>(size - 2u) && x > axis[idx + 1u]) { ++idx; }
    return idx;
}

uint8_t axis_frac_q8_u16(const uint16_t* axis, uint8_t idx, uint16_t x) noexcept {
    const uint16_t x0 = axis[idx];
    const uint16_t x1 = axis[idx + 1u];

    if (x <= x0) {
        return 0u;
    }
    if (x >= x1) {
        return 255u;
    }

    const uint16_t span = static_cast<uint16_t>(x1 - x0);
    if (span == 0u) {
        return 0u;
    }

    const uint32_t num = static_cast<uint32_t>(x - x0) << 8u;
    uint32_t frac = num / span;
    if (frac > 255u) {
        frac = 255u;
    }
    return static_cast<uint8_t>(frac);
}


int32_t lerp_q8_s32(int32_t a, int32_t b, uint8_t fq8) noexcept {
    return a + (((b - a) * static_cast<int32_t>(fq8)) >> 8u);
}

uint16_t interp1_u16_8(const int16_t* axis, const uint16_t* values, int16_t x) noexcept {
    if (x <= axis[0]) {
        return values[0];
    }
    if (x >= axis[kWarmupPts - 1u]) {
        return values[kWarmupPts - 1u];
    }

    uint8_t idx = 0u;
    while (idx < (kWarmupPts - 2u) && x > axis[idx + 1u]) { ++idx; }

    const int16_t x0 = axis[idx];
    const int16_t x1 = axis[idx + 1u];
    const uint16_t y0 = values[idx];
    const uint16_t y1 = values[idx + 1u];

    const int32_t dx = static_cast<int32_t>(x) - static_cast<int32_t>(x0);
    const int32_t span = static_cast<int32_t>(x1) - static_cast<int32_t>(x0);
    if (span <= 0) {
        return y0;
    }

    const int32_t y = static_cast<int32_t>(y0) +
                      ((static_cast<int32_t>(y1) - static_cast<int32_t>(y0)) * dx) / span;
    if (y <= 0) {
        return 0u;
    }
    if (y >= 65535) {
        return 65535u;
    }
    return static_cast<uint16_t>(y);
}

// gear: 0=neutro/desconhecido, 1-6; índice direto na tabela (sem interpolação no eixo Y)
uint16_t lookup_boost_target(uint32_t rpm_x10, uint8_t gear) noexcept {
    const uint8_t g  = (gear >= kBoostGears) ? (kBoostGears - 1u) : gear;
    const uint8_t xi = ems::engine::table_axis_index(kBoostRpmAxisX10, kBoostRpmPts, rpm_x10);
    const uint8_t fx = ems::engine::table_axis_frac_q8(kBoostRpmAxisX10, xi, rpm_x10);

    const int32_t v0 = static_cast<int32_t>(ems::engine::boost_target_bar_x1000[g][xi]);
    const int32_t v1 = static_cast<int32_t>(ems::engine::boost_target_bar_x1000[g][xi + 1u]);
    const int32_t v  = lerp_q8_s32(v0, v1, fx);

    if (v <= 0) {
        return 0u;
    }
    if (v >= 65535) {
        return 65535u;
    }
    return static_cast<uint16_t>(v);
}

int16_t lookup_vvt_target(const int16_t table[kVvtPts][kVvtPts],
                          uint32_t rpm_x10,
                          uint16_t load_bar_x1000) noexcept {
    const uint8_t xi = ems::engine::table_axis_index(kVvtRpmAxisX10, kVvtPts, rpm_x10);
    const uint8_t yi = axis_index_u16(kVvtLoadAxisBarX1000, kVvtPts, load_bar_x1000);
    const uint8_t fx = ems::engine::table_axis_frac_q8(kVvtRpmAxisX10, xi, rpm_x10);
    const uint8_t fy = axis_frac_q8_u16(kVvtLoadAxisBarX1000, yi, load_bar_x1000);

    const int32_t v00 = table[yi][xi];
    const int32_t v10 = table[yi][xi + 1u];
    const int32_t v01 = table[yi + 1u][xi];
    const int32_t v11 = table[yi + 1u][xi + 1u];

    const int32_t v0 = lerp_q8_s32(v00, v10, fx);
    const int32_t v1 = lerp_q8_s32(v01, v11, fx);
    const int32_t v = lerp_q8_s32(v0, v1, fy);

    return clamp_i16(static_cast<int16_t>(v), -1000, 3600);
}

void set_fan(bool on) noexcept {
    g.fan_on = on;
    if (on) {
        GPIOB_BSRR = kFanBit;
    } else {
        GPIOB_BSRR = (kFanBit << 16u);
    }
}

void set_pump(bool on) noexcept {
    g.pump_on = on;
    if (on) {
        GPIOB_BSRR = kPumpBit;
    } else {
        GPIOB_BSRR = (kPumpBit << 16u);
    }
}

uint16_t calc_cam_pos_est_x10(const ems::drv::CkpSnapshot& snap) noexcept {
    // tooth_index × 6,0° × 10 = tooth_index × 60 (roda 60-2: 360°/60 posições = 6°/dente)
    const uint16_t crank_deg_x10 = static_cast<uint16_t>(static_cast<uint32_t>(snap.tooth_index) * 60u);
    const uint16_t cycle_deg_x10 = snap.phase_A ? crank_deg_x10 : static_cast<uint16_t>(crank_deg_x10 + 3600u);
    return static_cast<uint16_t>(cycle_deg_x10 / 2u);
}

uint16_t iac_target_rpm_x10(int16_t clt_x10) noexcept {
    return interp1_u16_8(kWarmupCltAxisX10, kIdleTargetRpmX10, clt_x10);
}

void run_wastegate_control(const ems::drv::CkpSnapshot& snap,
                           const ems::drv::SensorData& s) noexcept {
    uint8_t gear = 0u;
    ems::app::can_rx_gear(gear, g.time_ms);  // fallback para 0 se CAN não configurado
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

uint16_t run_vvt_pid(int16_t target_deg_x10,
                     uint16_t pos_deg_x10,
                     int16_t& integrator_x10) noexcept {
    const int32_t error = static_cast<int32_t>(target_deg_x10) - static_cast<int32_t>(pos_deg_x10);
    const int32_t p = (error * 12) / 10;

    integrator_x10 = clamp_i16(
        static_cast<int16_t>(integrator_x10 + static_cast<int16_t>(error / 20)),
        -300,
        300);

    int32_t out = 500 + p + integrator_x10;
    if (out < 0) {
        out = 0;
    }
    if (out > 1000) {
        out = 1000;
    }
    return static_cast<uint16_t>(out);
}

void run_vvt_control(const ems::drv::CkpSnapshot& snap,
                     const ems::drv::SensorData& s) noexcept {
    if (snap.phase_A != g.phase_prev) {
        g.phase_prev = snap.phase_A;
        g.vvt_last_phase_toggle_ms = g.time_ms;
    }

    const bool confirmed =
        (snap.state == ems::drv::SyncState::FULL_SYNC) &&
        ((g.time_ms - g.vvt_last_phase_toggle_ms) <= kVvtConfirmTimeoutMs);

    if (!confirmed) {
        g.vvt_esc_duty_x10 = 0u;
        g.vvt_adm_duty_x10 = 0u;
        g.vvt_esc_integrator_x10 = 0;
        g.vvt_adm_integrator_x10 = 0;
        ems::hal::tim4_set_duty(0u, 0u);
        ems::hal::tim4_set_duty(1u, 0u);
        return;
    }

    const uint16_t pos_deg_x10 = calc_cam_pos_est_x10(snap);
    const int16_t target_esc = lookup_vvt_target(kVvtEscTargetDegX10, snap.rpm_x10, s.map_bar_x1000);
    const int16_t target_adm = lookup_vvt_target(kVvtAdmTargetDegX10, snap.rpm_x10, s.map_bar_x1000);

    g.vvt_esc_duty_x10 = run_vvt_pid(target_esc, pos_deg_x10, g.vvt_esc_integrator_x10);
    g.vvt_adm_duty_x10 = run_vvt_pid(target_adm, pos_deg_x10, g.vvt_adm_integrator_x10);

    ems::hal::tim4_set_duty(0u, g.vvt_esc_duty_x10);
    ems::hal::tim4_set_duty(1u, g.vvt_adm_duty_x10);
}

void run_fan_control(int16_t clt_x10) noexcept {
    if (!g.fan_on && clt_x10 >= kFanOnDegCX10) {
        set_fan(true);
    } else if (g.fan_on && clt_x10 <= kFanOffDegCX10) {
        set_fan(false);
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

uint16_t auxiliaries_idle_target_rpm_x10(int16_t clt_x10) noexcept {
    return iac_target_rpm_x10(clt_x10);
}

void auxiliaries_init() noexcept {
    reset_state();

    const ems::drv::CkpSnapshot snap = ems::drv::ckp_snapshot();
    g.phase_prev = snap.phase_A;

    // NÃO inicializar o TIM3 aqui: ele é dedicado à injeção (OC em PC6-9).
    // O motor EWG (wastegate) usa o TIM2_CH3/PB10 via ewg_driver. Antes, este
    // tim3_pwm_init reescrevia o ARR do TIM3 e quebrava o timing dos injetores.
    ems::hal::tim4_pwm_init(kAuxTim4PwmHz);   // TIM4: VVT (CH1 exhaust, CH2 intake)
    ems::hal::tim4_set_duty(0u, 0u);
    ems::hal::tim4_set_duty(1u, 0u);

    RCC_AHB2ENR1 |= RCC_AHB2ENR1_GPIOBEN;
    GPIOB_MODER = (GPIOB_MODER & ~(3u << (kFanPin * 2u))) | (1u << (kFanPin * 2u));
    GPIOB_MODER = (GPIOB_MODER & ~(3u << (kPumpPin * 2u))) | (1u << (kPumpPin * 2u));

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

    const ems::drv::CkpSnapshot snap = ems::drv::ckp_snapshot();
    const ems::drv::SensorData s = ems::drv::sensors_get();  // cópia atômica

    run_vvt_control(snap, s);
    run_fan_control(s.clt_degc_x10);
    run_pump_control(snap.rpm_x10);
}

void auxiliaries_tick_20ms() noexcept {
    const ems::drv::CkpSnapshot snap = ems::drv::ckp_snapshot();
    const ems::drv::SensorData s = ems::drv::sensors_get();  // cópia atômica

    run_wastegate_control(snap, s);
    run_fan_control(s.clt_degc_x10);
    run_pump_control(snap.rpm_x10);
}

uint16_t auxiliaries_ewg_position_demand_x10() noexcept {
    return g.ewg_position_demand_x10;
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
