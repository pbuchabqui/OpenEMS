#include "drv/sensors.h"

#include <cstdint>

#if __has_include("hal/adc.h")
#include "hal/adc.h"
#elif __has_include("adc.h")
#include "adc.h"
#endif

namespace {

using ems::drv::SensorData;
using ems::drv::SensorId;
using ems::drv::SensorRange;

constexpr uint8_t kFaultLimit = 3u;
constexpr uint16_t kRealTeethPerRev = 58u;
constexpr uint16_t kFastSamplesPerRev = 12u;

constexpr uint16_t kFallbackMapKpaX10 = 1010u;
constexpr uint16_t kFallbackTpsPctX10 = 0u;
constexpr int16_t kFallbackCltDegcX10 = 900;
constexpr int16_t kFallbackIatDegcX10 = 250;

struct FaultTracker {
    SensorRange range;
    uint8_t consecutive_bad;
    bool active;
};

static SensorData g_data = {0u, 0u, 0u, 0, 0, 0u, 0u, 0u, 0u, 0u};

static FaultTracker g_fault[8] = {
    {{50u, 4050u}, 0u, false},
    {{10u, 4095u}, 0u, false},
    {{50u, 4050u}, 0u, false},
    {{100u, 3800u}, 0u, false},
    {{100u, 3900u}, 0u, false},
    {{10u, 4095u}, 0u, false},
    {{50u, 4050u}, 0u, false},
    {{50u, 4050u}, 0u, false},
};

static uint16_t g_tps_raw_min = 200u;
static uint16_t g_tps_raw_max = 3895u;

static uint16_t g_map_filt = 0u;
static uint16_t g_mafv_filt = 0u;
static uint16_t g_o2_filt = 0u;

static uint16_t g_tps_buf[4] = {0u, 0u, 0u, 0u};
static uint8_t g_tps_pos = 0u;

static uint16_t g_clt_buf[8] = {0u};
static uint8_t g_clt_pos = 0u;
static uint16_t g_iat_buf[8] = {0u};
static uint8_t g_iat_pos = 0u;

static uint16_t g_fuel_buf[4] = {0u};
static uint8_t g_fuel_pos = 0u;
static uint16_t g_oil_buf[4] = {0u};
static uint8_t g_oil_pos = 0u;

static uint16_t g_maf_period_buf[4] = {0u};
static uint8_t g_maf_period_pos = 0u;

static int16_t g_clt_table[128] = {};
static int16_t g_iat_table[128] = {};

static uint16_t g_fast_sample_accum = 0u;

inline void reset_state() noexcept {
    g_data = SensorData{0u, 0u, 0u, 0, 0, 0u, 0u, 0u, 0u, 0u};

    g_map_filt = 0u;
    g_mafv_filt = 0u;
    g_o2_filt = 0u;

    for (uint8_t i = 0u; i < 4u; ++i) {
        g_tps_buf[i] = 0u;
        g_fuel_buf[i] = 0u;
        g_oil_buf[i] = 0u;
        g_maf_period_buf[i] = 0u;
    }
    for (uint8_t i = 0u; i < 8u; ++i) {
        g_clt_buf[i] = 0u;
        g_iat_buf[i] = 0u;
    }

    g_tps_pos = 0u;
    g_clt_pos = 0u;
    g_iat_pos = 0u;
    g_fuel_pos = 0u;
    g_oil_pos = 0u;
    g_maf_period_pos = 0u;
    g_fast_sample_accum = 0u;

    g_tps_raw_min = 200u;
    g_tps_raw_max = 3895u;

    g_fault[0] = FaultTracker{{50u, 4095u}, 0u, false};
    g_fault[1] = FaultTracker{{10u, 4095u}, 0u, false};
    g_fault[2] = FaultTracker{{50u, 4095u}, 0u, false};
    g_fault[3] = FaultTracker{{100u, 3800u}, 0u, false};
    g_fault[4] = FaultTracker{{100u, 3900u}, 0u, false};
    g_fault[5] = FaultTracker{{10u, 4095u}, 0u, false};
    g_fault[6] = FaultTracker{{50u, 4050u}, 0u, false};
    g_fault[7] = FaultTracker{{50u, 4050u}, 0u, false};
}

inline uint16_t iir_alpha_03(uint16_t y, uint16_t x) noexcept {
    const int32_t delta = static_cast<int32_t>(x) - static_cast<int32_t>(y);
    int32_t step = (delta * 3) / 10;
    if (step == 0 && delta != 0) {
        step = (delta > 0) ? 1 : -1;
    }
    const int32_t out = static_cast<int32_t>(y) + step;
    if (out <= 0) {
        return 0u;
    }
    if (out >= 4095) {
        return 4095u;
    }
    return static_cast<uint16_t>(out);
}

inline uint16_t iir_alpha_01(uint16_t y, uint16_t x) noexcept {
    const int32_t delta = static_cast<int32_t>(x) - static_cast<int32_t>(y);
    int32_t step = delta / 10;
    if (step == 0 && delta != 0) {
        step = (delta > 0) ? 1 : -1;
    }
    const int32_t out = static_cast<int32_t>(y) + step;
    if (out <= 0) {
        return 0u;
    }
    if (out >= 4095) {
        return 4095u;
    }
    return static_cast<uint16_t>(out);
}

inline uint8_t sensor_bit(SensorId id) noexcept {
    return static_cast<uint8_t>(id);
}

inline uint16_t avg4(const uint16_t* v) noexcept {
    return static_cast<uint16_t>((static_cast<uint32_t>(v[0]) + v[1] + v[2] + v[3]) / 4u);
}

inline uint16_t avg8(const uint16_t* v) noexcept {
    uint32_t sum = 0u;
    for (uint8_t i = 0u; i < 8u; ++i) {
        sum += v[i];
    }
    return static_cast<uint16_t>(sum / 8u);
}

inline void apply_fault(SensorId id, uint16_t raw) noexcept {
    FaultTracker& f = g_fault[static_cast<uint8_t>(id)];
    const bool bad = (raw < f.range.min_raw) || (raw > f.range.max_raw);

    if (bad) {
        if (f.consecutive_bad < 255u) {
            ++f.consecutive_bad;
        }
        if (f.consecutive_bad >= kFaultLimit) {
            f.active = true;
        }
    } else {
        f.consecutive_bad = 0u;
        f.active = false;
    }

    const uint8_t bit = sensor_bit(id);
    if (f.active) {
        g_data.fault_bits = static_cast<uint8_t>(g_data.fault_bits | static_cast<uint8_t>(1u << bit));
    } else {
        g_data.fault_bits = static_cast<uint8_t>(g_data.fault_bits & static_cast<uint8_t>(~(1u << bit)));
    }
}

inline int16_t lut128(const int16_t* table, uint16_t raw) noexcept {
    const uint8_t idx = static_cast<uint8_t>(raw >> 5u);
    return table[idx];
}

inline void init_tables() noexcept {
    for (uint16_t i = 0u; i < 128u; ++i) {
        const int32_t t = -400 + static_cast<int32_t>((1900u * i) / 127u);
        g_clt_table[i] = static_cast<int16_t>(t);
        g_iat_table[i] = static_cast<int16_t>(t);
    }
}

inline uint16_t map_raw_to_kpa_x10(uint16_t raw) noexcept {
    return static_cast<uint16_t>((static_cast<uint32_t>(raw) * 2500u) / 4095u);
}

inline uint16_t raw_to_mv(uint16_t raw) noexcept {
    return static_cast<uint16_t>((static_cast<uint32_t>(raw) * 1000u) / 4095u);
}

inline uint16_t tps_raw_to_pct_x10(uint16_t raw) noexcept {
    if (g_tps_raw_max <= g_tps_raw_min) {
        return 0u;
    }
    if (raw <= g_tps_raw_min) {
        return 0u;
    }
    if (raw >= g_tps_raw_max) {
        return 1000u;
    }
    const uint32_t num = static_cast<uint32_t>(raw - g_tps_raw_min) * 1000u;
    const uint32_t den = static_cast<uint32_t>(g_tps_raw_max - g_tps_raw_min);
    return static_cast<uint16_t>(num / den);
}

inline uint16_t maf_period_avg4() noexcept {
    return avg4(g_maf_period_buf);
}

inline void sample_fast_channels() noexcept {
    const uint16_t map_raw = ems::hal::adc0_read(ems::hal::Adc0Channel::MAP_SE10);
    const uint16_t mafv_raw = ems::hal::adc0_read(ems::hal::Adc0Channel::MAF_V_SE11);
    const uint16_t tps_raw = ems::hal::adc0_read(ems::hal::Adc0Channel::TPS_SE12);
    const uint16_t o2_raw = ems::hal::adc0_read(ems::hal::Adc0Channel::O2_SE4B);

    g_map_filt = iir_alpha_03(g_map_filt, map_raw);
    g_mafv_filt = iir_alpha_03(g_mafv_filt, mafv_raw);
    g_o2_filt = iir_alpha_01(g_o2_filt, o2_raw);

    g_tps_buf[g_tps_pos] = tps_raw;
    g_tps_pos = static_cast<uint8_t>((g_tps_pos + 1u) & 0x3u);

    apply_fault(SensorId::MAP, map_raw);
    apply_fault(SensorId::MAF, mafv_raw);
    apply_fault(SensorId::TPS, tps_raw);
    apply_fault(SensorId::O2, o2_raw);

    g_data.map_kpa_x10 = g_fault[static_cast<uint8_t>(SensorId::MAP)].active
                             ? kFallbackMapKpaX10
                             : map_raw_to_kpa_x10(g_map_filt);

    if (g_fault[static_cast<uint8_t>(SensorId::TPS)].active) {
        g_data.tps_pct_x10 = kFallbackTpsPctX10;
    } else {
        g_data.tps_pct_x10 = tps_raw_to_pct_x10(avg4(g_tps_buf));
    }

    g_data.o2_mv = raw_to_mv(g_o2_filt);

    const uint16_t maf_avg_period = maf_period_avg4();
    uint32_t maf_from_freq_x100 = 0u;
    if (maf_avg_period > 0u) {
        maf_from_freq_x100 = 24000000u / maf_avg_period;
    }
    const uint32_t maf_from_v_x100 = (static_cast<uint32_t>(g_mafv_filt) * 10000u) / 4095u;
    g_data.maf_gps_x100 = static_cast<uint16_t>((maf_from_freq_x100 + maf_from_v_x100) / 2u);
}

}  // namespace

namespace ems::drv {

void sensors_init() noexcept {
    ems::hal::adc_init();
    init_tables();
    reset_state();
}

void sensors_on_tooth(const CkpSnapshot& snap) noexcept {
    const uint32_t tooth_ticks_32 = (snap.tooth_period_ns * 120u) / 1000u;
    const uint16_t tooth_ticks = static_cast<uint16_t>(tooth_ticks_32 & 0xFFFFu);
    ems::hal::adc_pdb_on_tooth(tooth_ticks);

    g_fast_sample_accum = static_cast<uint16_t>(g_fast_sample_accum + kFastSamplesPerRev);
    if (g_fast_sample_accum >= kRealTeethPerRev) {
        g_fast_sample_accum = static_cast<uint16_t>(g_fast_sample_accum - kRealTeethPerRev);
        sample_fast_channels();
    }
}

void sensors_tick_50ms() noexcept {
    const uint16_t fuel_raw = ems::hal::adc1_read(ems::hal::Adc1Channel::FUEL_PRESS_SE5B);
    const uint16_t oil_raw = ems::hal::adc1_read(ems::hal::Adc1Channel::OIL_PRESS_SE6B);

    g_fuel_buf[g_fuel_pos] = fuel_raw;
    g_fuel_pos = static_cast<uint8_t>((g_fuel_pos + 1u) & 0x3u);

    g_oil_buf[g_oil_pos] = oil_raw;
    g_oil_pos = static_cast<uint8_t>((g_oil_pos + 1u) & 0x3u);

    apply_fault(SensorId::FUEL_PRESS, fuel_raw);
    apply_fault(SensorId::OIL_PRESS, oil_raw);

    g_data.fuel_press_kpa_x10 = static_cast<uint16_t>((static_cast<uint32_t>(avg4(g_fuel_buf)) * 2500u) / 4095u);
    g_data.oil_press_kpa_x10 = static_cast<uint16_t>((static_cast<uint32_t>(avg4(g_oil_buf)) * 2500u) / 4095u);
}

void sensors_tick_100ms() noexcept {
    const uint16_t clt_raw = ems::hal::adc1_read(ems::hal::Adc1Channel::CLT_SE14);
    const uint16_t iat_raw = ems::hal::adc1_read(ems::hal::Adc1Channel::IAT_SE15);

    g_clt_buf[g_clt_pos] = clt_raw;
    g_clt_pos = static_cast<uint8_t>((g_clt_pos + 1u) & 0x7u);

    g_iat_buf[g_iat_pos] = iat_raw;
    g_iat_pos = static_cast<uint8_t>((g_iat_pos + 1u) & 0x7u);

    apply_fault(SensorId::CLT, clt_raw);
    apply_fault(SensorId::IAT, iat_raw);

    const uint16_t clt_avg = avg8(g_clt_buf);
    const uint16_t iat_avg = avg8(g_iat_buf);

    g_data.clt_degc_x10 = g_fault[static_cast<uint8_t>(SensorId::CLT)].active
                              ? kFallbackCltDegcX10
                              : lut128(g_clt_table, clt_avg);
    g_data.iat_degc_x10 = g_fault[static_cast<uint8_t>(SensorId::IAT)].active
                              ? kFallbackIatDegcX10
                              : lut128(g_iat_table, iat_avg);

    // TODO: VBATT via canal dedicado quando mapeamento elétrico for definido.
    g_data.vbatt_mv = 12000u;
}

void sensors_maf_freq_capture_isr(uint16_t period_ticks) noexcept {
    g_maf_period_buf[g_maf_period_pos] = period_ticks;
    g_maf_period_pos = static_cast<uint8_t>((g_maf_period_pos + 1u) & 0x3u);
}

void sensors_set_tps_cal(uint16_t raw_min, uint16_t raw_max) noexcept {
    g_tps_raw_min = raw_min;
    g_tps_raw_max = raw_max;
}

void sensors_set_range(SensorId id, SensorRange range) noexcept {
    g_fault[static_cast<uint8_t>(id)].range = range;
}

const SensorData& sensors_get() noexcept {
    return g_data;
}

#if defined(EMS_HOST_TEST)
void sensors_test_reset() noexcept {
    reset_state();
}

void sensors_test_set_clt_table_entry(uint8_t idx, int16_t degc_x10) noexcept {
    if (idx < 128u) {
        g_clt_table[idx] = degc_x10;
    }
}

void sensors_test_set_iat_table_entry(uint8_t idx, int16_t degc_x10) noexcept {
    if (idx < 128u) {
        g_iat_table[idx] = degc_x10;
    }
}
#endif

}  // namespace ems::drv
