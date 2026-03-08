#include "engine/auxiliaries.h"

#include <cstdint>

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

#if __has_include("hal/ftm.h")
#include "hal/ftm.h"
#elif __has_include("ftm.h")
#include "ftm.h"
#endif

namespace {

constexpr uint32_t kTick10ms = 10u;
constexpr uint32_t kTick20ms = 20u;
constexpr uint32_t kAuxFtm1PwmHz = 15u;
constexpr uint32_t kAuxFtm2PwmHz = 15u;

constexpr int16_t kCltPidEnableX10 = 600;
constexpr int16_t kDrpmEnableMaxX10PerSec = 2000;

constexpr int16_t kIacKp_num = 2;
constexpr int16_t kIacKd_num = 5;
constexpr int16_t kIacKd_den = 2;
constexpr int16_t kIacIClampX10 = 300;

constexpr uint32_t kOverboostDurationMs = 500u;
constexpr uint16_t kOverboostMarginKpaX10 = 200u;

constexpr uint32_t kVvtConfirmTimeoutMs = 200u;

constexpr int16_t kFanOnDegCX10 = 950;
constexpr int16_t kFanOffDegCX10 = 900;

constexpr uint32_t kPumpPrimeMs = 2000u;
constexpr uint32_t kPumpOffDelayMs = 3000u;

constexpr uint8_t kWarmupPts = 8u;
constexpr int16_t kWarmupCltAxisX10[kWarmupPts] = {-400, -100, 100, 300, 500, 700, 900, 1100};
constexpr uint16_t kIacWarmupDutyX10[kWarmupPts] = {620u, 560u, 500u, 440u, 360u, 280u, 220u, 180u};
constexpr uint16_t kIdleTargetRpmX10[kWarmupPts] = {12000u, 11500u, 10800u, 10000u, 9200u, 8500u, 8200u, 8000u};

constexpr uint8_t kBoostPts = 8u;
constexpr uint16_t kBoostRpmAxisX10[kBoostPts] = {1500u, 2000u, 2500u, 3000u, 4000u, 5000u, 6500u, 8000u};
constexpr uint16_t kBoostTpsAxisX10[kBoostPts] = {100u, 200u, 300u, 450u, 600u, 750u, 900u, 1000u};
constexpr uint16_t kBoostTargetKpaX10[kBoostPts][kBoostPts] = {
    {1050u, 1100u, 1150u, 1200u, 1250u, 1300u, 1350u, 1400u},
    {1050u, 1100u, 1150u, 1220u, 1270u, 1320u, 1380u, 1430u},
    {1060u, 1120u, 1180u, 1250u, 1300u, 1360u, 1420u, 1480u},
    {1080u, 1140u, 1210u, 1280u, 1340u, 1400u, 1460u, 1520u},
    {1100u, 1170u, 1240u, 1320u, 1390u, 1460u, 1530u, 1600u},
    {1120u, 1200u, 1270u, 1350u, 1430u, 1510u, 1590u, 1670u},
    {1140u, 1220u, 1300u, 1390u, 1470u, 1560u, 1650u, 1740u},
    {1150u, 1240u, 1330u, 1420u, 1510u, 1610u, 1700u, 1800u},
};

constexpr uint8_t kVvtPts = 12u;
constexpr uint16_t kVvtRpmAxisX10[kVvtPts] = {1000u, 1500u, 2000u, 2500u, 3000u, 3500u, 4000u, 4500u, 5000u, 6000u, 7000u, 8000u};
constexpr uint16_t kVvtLoadAxisKpaX10[kVvtPts] = {300u, 400u, 500u, 600u, 700u, 800u, 900u, 1000u, 1100u, 1200u, 1400u, 1700u};

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

#if defined(EMS_HOST_TEST)
volatile uint32_t ems_test_portb_pddr = 0u;
volatile uint32_t ems_test_fgpiob_psor = 0u;
volatile uint32_t ems_test_fgpiob_pcor = 0u;
volatile uint32_t ems_test_sim_scgc5 = 0u;
volatile uint32_t ems_test_portb_pcr16 = 0u;
volatile uint32_t ems_test_portb_pcr17 = 0u;
#define GPIOB_PDDR ems_test_portb_pddr
#define FGPIOB_PSOR ems_test_fgpiob_psor
#define FGPIOB_PCOR ems_test_fgpiob_pcor
#define SIM_SCGC5 ems_test_sim_scgc5
#define PORTB_PCR16 ems_test_portb_pcr16
#define PORTB_PCR17 ems_test_portb_pcr17
#else
#define GPIOB_PDDR (*reinterpret_cast<volatile uint32_t*>(0xF80FF054u))
#define FGPIOB_PSOR (*reinterpret_cast<volatile uint32_t*>(0xF80FF044u))
#define FGPIOB_PCOR (*reinterpret_cast<volatile uint32_t*>(0xF80FF048u))
#define SIM_SCGC5 (*reinterpret_cast<volatile uint32_t*>(0x40048038u))
#define PORTB_PCR16 (*reinterpret_cast<volatile uint32_t*>(0x4004A040u))
#define PORTB_PCR17 (*reinterpret_cast<volatile uint32_t*>(0x4004A044u))
#endif

constexpr uint32_t SIM_SCGC5_PORTB_MASK = (1u << 10u);
constexpr uint32_t PCR_MUX_GPIO = (1u << 8u);
constexpr uint32_t kFanBit = (1u << 16u);
constexpr uint32_t kPumpBit = (1u << 17u);

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

    uint16_t iac_duty_x10;
    int16_t iac_integrator_x10;
    int16_t iac_prev_error_x10;
    int32_t iac_last_rpm_x10;
    bool iac_have_prev_rpm;

    uint16_t wg_duty_x10;
    int16_t wg_integrator_x10;
    uint32_t wg_overboost_ms;
    bool wg_failsafe;

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
    for (uint8_t i = 0u; i < last; ++i) {
        if (x <= axis[i + 1u]) {
            return i;
        }
    }
    return static_cast<uint8_t>(size - 2u);
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
    for (uint8_t i = 0u; i < (kWarmupPts - 1u); ++i) {
        if (x <= axis[i + 1u]) {
            idx = i;
            break;
        }
    }

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

uint16_t lookup_boost_target(uint16_t rpm_x10, uint16_t tps_x10) noexcept {
    const uint8_t xi = axis_index_u16(kBoostRpmAxisX10, kBoostPts, rpm_x10);
    const uint8_t yi = axis_index_u16(kBoostTpsAxisX10, kBoostPts, tps_x10);
    const uint8_t fx = axis_frac_q8_u16(kBoostRpmAxisX10, xi, rpm_x10);
    const uint8_t fy = axis_frac_q8_u16(kBoostTpsAxisX10, yi, tps_x10);

    const int32_t v00 = static_cast<int32_t>(kBoostTargetKpaX10[yi][xi]);
    const int32_t v10 = static_cast<int32_t>(kBoostTargetKpaX10[yi][xi + 1u]);
    const int32_t v01 = static_cast<int32_t>(kBoostTargetKpaX10[yi + 1u][xi]);
    const int32_t v11 = static_cast<int32_t>(kBoostTargetKpaX10[yi + 1u][xi + 1u]);

    const int32_t v0 = lerp_q8_s32(v00, v10, fx);
    const int32_t v1 = lerp_q8_s32(v01, v11, fx);
    const int32_t v = lerp_q8_s32(v0, v1, fy);

    if (v <= 0) {
        return 0u;
    }
    if (v >= 65535) {
        return 65535u;
    }
    return static_cast<uint16_t>(v);
}

int16_t lookup_vvt_target(const int16_t table[kVvtPts][kVvtPts],
                          uint16_t rpm_x10,
                          uint16_t load_kpa_x10) noexcept {
    const uint8_t xi = axis_index_u16(kVvtRpmAxisX10, kVvtPts, rpm_x10);
    const uint8_t yi = axis_index_u16(kVvtLoadAxisKpaX10, kVvtPts, load_kpa_x10);
    const uint8_t fx = axis_frac_q8_u16(kVvtRpmAxisX10, xi, rpm_x10);
    const uint8_t fy = axis_frac_q8_u16(kVvtLoadAxisKpaX10, yi, load_kpa_x10);

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
        FGPIOB_PSOR = kFanBit;
    } else {
        FGPIOB_PCOR = kFanBit;
    }
}

void set_pump(bool on) noexcept {
    g.pump_on = on;
    if (on) {
        FGPIOB_PSOR = kPumpBit;
    } else {
        FGPIOB_PCOR = kPumpBit;
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

uint16_t iac_warmup_duty_x10(int16_t clt_x10) noexcept {
    return interp1_u16_8(kWarmupCltAxisX10, kIacWarmupDutyX10, clt_x10);
}

void run_iac_control(const ems::drv::CkpSnapshot& snap,
                     const ems::drv::SensorData& s) noexcept {
    const int32_t rpm_now = static_cast<int32_t>(snap.rpm_x10);
    int32_t drpm_x10 = 0;
    if (g.iac_have_prev_rpm) {
        drpm_x10 = rpm_now - g.iac_last_rpm_x10;
    } else {
        g.iac_have_prev_rpm = true;
    }
    g.iac_last_rpm_x10 = rpm_now;
    const int32_t drpm_dt_x10_per_sec = drpm_x10 * static_cast<int32_t>(1000u / kTick20ms);

    uint16_t duty_base = 0u;
    if (s.clt_degc_x10 < kCltPidEnableX10) {
        duty_base = iac_warmup_duty_x10(s.clt_degc_x10);
    } else {
        duty_base = 300u;
    }

    const uint16_t rpm_target = iac_target_rpm_x10(s.clt_degc_x10);
    const int32_t error_x10 = static_cast<int32_t>(rpm_target) - rpm_now;

    bool pid_enabled = false;
    if (s.clt_degc_x10 > kCltPidEnableX10) {
        const int32_t abs_drpm = (drpm_dt_x10_per_sec < 0) ? -drpm_dt_x10_per_sec : drpm_dt_x10_per_sec;
        pid_enabled = (abs_drpm < kDrpmEnableMaxX10PerSec);
    }

    int32_t out_x10 = static_cast<int32_t>(duty_base);
    if (g.iac_duty_x10 > static_cast<uint16_t>(out_x10)) {
        out_x10 = g.iac_duty_x10;
    }

    if (pid_enabled) {
        const int32_t p_x10 = static_cast<int32_t>(kIacKp_num) * error_x10;
        g.iac_integrator_x10 = clamp_i16(
            static_cast<int16_t>(g.iac_integrator_x10 + static_cast<int16_t>((error_x10 * 6) / 1000)),
            -kIacIClampX10,
            kIacIClampX10);

        const int32_t de_x10 = error_x10 - static_cast<int32_t>(g.iac_prev_error_x10);
        const int32_t d_x10 = (de_x10 * kIacKd_num) / kIacKd_den;

        g.iac_prev_error_x10 = static_cast<int16_t>(clamp_i16(static_cast<int16_t>(error_x10), -12000, 12000));
        out_x10 += p_x10 + g.iac_integrator_x10 + d_x10;
    }

    if (out_x10 < 0) {
        out_x10 = 0;
    }
    if (out_x10 > 1000) {
        out_x10 = 1000;
    }

    g.iac_duty_x10 = static_cast<uint16_t>(out_x10);
    ems::hal::ftm1_set_duty(0u, g.iac_duty_x10);
}

void run_wastegate_control(const ems::drv::CkpSnapshot& snap,
                           const ems::drv::SensorData& s) noexcept {
    const uint16_t target_kpa_x10 = lookup_boost_target(snap.rpm_x10, s.tps_pct_x10);

    if (s.map_kpa_x10 > static_cast<uint16_t>(target_kpa_x10 + kOverboostMarginKpaX10)) {
        g.wg_overboost_ms += kTick20ms;
    } else {
        g.wg_overboost_ms = 0u;
        g.wg_failsafe = false;
    }

    if (g.wg_overboost_ms >= kOverboostDurationMs) {
        g.wg_failsafe = true;
    }

    if (g.wg_failsafe) {
        g.wg_duty_x10 = 0u;
        g.wg_integrator_x10 = 0;
        ems::hal::ftm1_set_duty(1u, 0u);
        return;
    }

    const int32_t error = static_cast<int32_t>(target_kpa_x10) - static_cast<int32_t>(s.map_kpa_x10);
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
    ems::hal::ftm1_set_duty(1u, g.wg_duty_x10);
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
        ems::hal::ftm2_set_duty(0u, 0u);
        ems::hal::ftm2_set_duty(1u, 0u);
        return;
    }

    const uint16_t pos_deg_x10 = calc_cam_pos_est_x10(snap);
    const int16_t target_esc = lookup_vvt_target(kVvtEscTargetDegX10, snap.rpm_x10, s.map_kpa_x10);
    const int16_t target_adm = lookup_vvt_target(kVvtAdmTargetDegX10, snap.rpm_x10, s.map_kpa_x10);

    g.vvt_esc_duty_x10 = run_vvt_pid(target_esc, pos_deg_x10, g.vvt_esc_integrator_x10);
    g.vvt_adm_duty_x10 = run_vvt_pid(target_adm, pos_deg_x10, g.vvt_adm_integrator_x10);

    ems::hal::ftm2_set_duty(0u, g.vvt_esc_duty_x10);
    ems::hal::ftm2_set_duty(1u, g.vvt_adm_duty_x10);
}

void run_fan_control(int16_t clt_x10) noexcept {
    if (!g.fan_on && clt_x10 >= kFanOnDegCX10) {
        set_fan(true);
    } else if (g.fan_on && clt_x10 <= kFanOffDegCX10) {
        set_fan(false);
    }
}

void run_pump_control(uint16_t rpm_x10) noexcept {
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

void auxiliaries_init() noexcept {
    reset_state();

    const ems::drv::CkpSnapshot snap = ems::drv::ckp_snapshot();
    g.phase_prev = snap.phase_A;

    // FTM1_CH0 (IACV) e FTM1_CH1 (Wastegate) compartilham o mesmo MOD:
    // não é possível 12.5 Hz e 15 Hz simultâneos em hardware no mesmo FTM.
    // Estratégia: fixa FTM1 em 15 Hz (prioridade para WG) e calibra IACV por duty.
    ems::hal::ftm1_pwm_init(kAuxFtm1PwmHz);
    ems::hal::ftm2_pwm_init(kAuxFtm2PwmHz);

    ems::hal::ftm1_set_duty(0u, 0u);
    ems::hal::ftm1_set_duty(1u, 0u);
    ems::hal::ftm2_set_duty(0u, 0u);
    ems::hal::ftm2_set_duty(1u, 0u);

    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
    PORTB_PCR16 = PCR_MUX_GPIO;
    PORTB_PCR17 = PCR_MUX_GPIO;
    GPIOB_PDDR |= (kFanBit | kPumpBit);

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
    g.time_ms += kTick20ms;

    const ems::drv::CkpSnapshot snap = ems::drv::ckp_snapshot();
    const ems::drv::SensorData s = ems::drv::sensors_get();  // cópia atômica

    run_iac_control(snap, s);
    run_wastegate_control(snap, s);
    run_fan_control(s.clt_degc_x10);
    run_pump_control(snap.rpm_x10);
}

#if defined(EMS_HOST_TEST)
void auxiliaries_test_reset() noexcept {
    auxiliaries_init();
}

uint16_t auxiliaries_test_get_iac_duty() noexcept {
    return g.iac_duty_x10;
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
