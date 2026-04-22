#include <cstdint>
#include <cstdio>
#include <cstring>

#define EMS_HOST_TEST 1
#include "app/tuner_studio.h"
#include "drv/ckp.h"
#include "drv/sensors.h"

namespace ems::drv {

static CkpSnapshot g_ckp = {0u, 0u, 0u, 0u, SyncState::WAIT_GAP, false};
static SensorData g_sensors = {0u, 0u, 0u, 0, 0, 0u, 0u, 0u, 0u, 0u};

CkpSnapshot ckp_snapshot() noexcept {
    return g_ckp;
}

SensorData sensors_get() noexcept {
    return g_sensors;
}

}  // namespace ems::drv

namespace {

int g_tests_run = 0;
int g_tests_failed = 0;

#define TEST_ASSERT_TRUE(cond) do { \
    ++g_tests_run; \
    if (!(cond)) { \
        ++g_tests_failed; \
        std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    } \
} while (0)

#define TEST_ASSERT_EQ_U32(exp, act) do { \
    ++g_tests_run; \
    const uint32_t _e = static_cast<uint32_t>(exp); \
    const uint32_t _a = static_cast<uint32_t>(act); \
    if (_e != _a) { \
        ++g_tests_failed; \
        std::printf("FAIL %s:%d: expected %u got %u\n", __FILE__, __LINE__, (unsigned)_e, (unsigned)_a); \
    } \
} while (0)

void ts_send_bytewise(const uint8_t* frame, uint16_t len) {
    for (uint16_t i = 0u; i < len; ++i) {
        ems::app::ts_uart0_rx_isr_byte(frame[i]);
        ems::app::ts_process();
    }
}

uint16_t ts_drain(uint8_t* out, uint16_t max_len) {
    uint16_t n = 0u;
    uint8_t b = 0u;
    while (n < max_len && ems::app::ts_tx_pop(b)) {
        out[n++] = b;
    }
    return n;
}

void test_q_returns_signature_cstr() {
    ems::app::ts_test_reset();

    const uint8_t cmd = static_cast<uint8_t>('Q');
    ts_send_bytewise(&cmd, 1u);

    uint8_t out[32] = {};
    const uint16_t n = ts_drain(out, sizeof(out));
    const char exp[] = "OpenEMS_v1.1";
    TEST_ASSERT_EQ_U32(std::strlen(exp), n);
    TEST_ASSERT_TRUE(std::memcmp(out, exp, std::strlen(exp)) == 0);
}

void test_write_then_read_page_roundtrip() {
    ems::app::ts_test_reset();

    const uint8_t w_frame[] = {
        static_cast<uint8_t>('w'),
        0x01u,
        0x10u, 0x00u,
        0x04u, 0x00u,
        0x12u, 0x34u, 0x56u, 0x78u,
    };
    ts_send_bytewise(w_frame, sizeof(w_frame));

    uint8_t out[32] = {};
    uint16_t n = ts_drain(out, sizeof(out));
    TEST_ASSERT_EQ_U32(1u, n);
    TEST_ASSERT_EQ_U32(0u, out[0]);

    const uint8_t r_frame[] = {
        static_cast<uint8_t>('r'),
        0x01u,
        0x10u, 0x00u,
        0x04u, 0x00u,
    };
    ts_send_bytewise(r_frame, sizeof(r_frame));

    n = ts_drain(out, sizeof(out));
    TEST_ASSERT_EQ_U32(4u, n);
    TEST_ASSERT_EQ_U32(0x12u, out[0]);
    TEST_ASSERT_EQ_U32(0x34u, out[1]);
    TEST_ASSERT_EQ_U32(0x56u, out[2]);
    TEST_ASSERT_EQ_U32(0x78u, out[3]);
}

void test_a_returns_64_bytes_realtime() {
    ems::app::ts_test_reset();

    ems::drv::g_ckp = ems::drv::CkpSnapshot{
        0u,
        0u,
        0u,
        12340u,
        ems::drv::SyncState::FULL_SYNC,
        true,
    };
    ems::drv::g_sensors = ems::drv::SensorData{
        987u,  // map_kpa_x10
        0u,    // maf_gps_x100
        321u,  // tps_pct_x10
        850,   // clt_degc_x10
        250,   // iat_degc_x10
        // o2_mv REMOVIDO
        0u,    // fuel_press_kpa_x10
        0u,    // oil_press_kpa_x10
        0u,    // vbatt_mv
        1u,    // fault_bits
    };

    const uint8_t cmd = static_cast<uint8_t>('A');
    ts_send_bytewise(&cmd, 1u);

    uint8_t out[128] = {};
    const uint16_t n = ts_drain(out, sizeof(out));
    TEST_ASSERT_EQ_U32(64u, n);

    const uint16_t rpm = static_cast<uint16_t>(out[0] | (static_cast<uint16_t>(out[1]) << 8u));
    TEST_ASSERT_EQ_U32(1234u, rpm);
    TEST_ASSERT_EQ_U32(98u, out[2]);
    TEST_ASSERT_EQ_U32(32u, out[3]);
    TEST_ASSERT_EQ_U32(125u, out[4]);
    TEST_ASSERT_EQ_U32(65u, out[5]);
    // o2_mv_d4 agora vem do WBO2 via CAN: WBO2_SAFE_LAMBDA_MILLI(1050)/4 = 262 → clamp → 255
    TEST_ASSERT_EQ_U32(255u, out[6]);
    TEST_ASSERT_EQ_U32(40u, out[8]);
    // ve = g_page1_ve[0] inicializado de ve_table[0][0] = 50 (P1: páginas ligadas às tabelas reais)
    TEST_ASSERT_EQ_U32(50u, out[9]);
    TEST_ASSERT_EQ_U32(0u, static_cast<int8_t>(out[10]));
    TEST_ASSERT_EQ_U32(0x87u, out[11]);
    TEST_ASSERT_EQ_U32(0u, out[12]);  // late_events LSB
    TEST_ASSERT_EQ_U32(0u, out[13]);
    TEST_ASSERT_EQ_U32(0u, out[14]);
    TEST_ASSERT_EQ_U32(0u, out[15]);  // late_events MSB
}

void test_a_packs_scheduler_diag_in_reserved() {
    ems::app::ts_test_reset();
    ems::drv::g_ckp = ems::drv::CkpSnapshot{
        0u,
        0u,
        0u,
        0u,
        ems::drv::SyncState::WAIT_GAP,
        false,
    };
    ems::drv::g_sensors = ems::drv::SensorData{
        0u, 0u, 0u, 0, 0, 0u, 0u, 0u, 0u, 0u,
    };
    ems::app::ts_update_rt_sched_diag(
        3u,          // late events
        1500u,       // late max delay ticks
        18u,         // queue depth peak
        12u,         // queue depth last-cycle peak
        2u,          // cycle schedule drops
        1u,          // calibration clamps
        7u,          // seed loaded
        5u,          // seed confirmed
        2u,          // seed rejected
        3u);         // sync state raw (FULL_SYNC)

    const uint8_t cmd = static_cast<uint8_t>('A');
    ts_send_bytewise(&cmd, 1u);

    uint8_t out[128] = {};
    const uint16_t n = ts_drain(out, sizeof(out));
    TEST_ASSERT_EQ_U32(64u, n);
    TEST_ASSERT_EQ_U32(0xF0u, out[11]);  // WBO2 + scheduler diag flags

    // reserved[0..3] late_events = 3
    TEST_ASSERT_EQ_U32(3u, out[12]);
    TEST_ASSERT_EQ_U32(0u, out[13]);
    TEST_ASSERT_EQ_U32(0u, out[14]);
    TEST_ASSERT_EQ_U32(0u, out[15]);
    // reserved[4..7] late_max_delay_ticks = 1500 (0x05DC)
    TEST_ASSERT_EQ_U32(0xDCu, out[16]);
    TEST_ASSERT_EQ_U32(0x05u, out[17]);
    TEST_ASSERT_EQ_U32(0u, out[18]);
    TEST_ASSERT_EQ_U32(0u, out[19]);
    // reserved[8..9] queue peaks
    TEST_ASSERT_EQ_U32(18u, out[20]);
    TEST_ASSERT_EQ_U32(12u, out[21]);
    // reserved[10..13] cycle drops = 2
    TEST_ASSERT_EQ_U32(2u, out[22]);
    TEST_ASSERT_EQ_U32(0u, out[23]);
    TEST_ASSERT_EQ_U32(0u, out[24]);
    TEST_ASSERT_EQ_U32(0u, out[25]);
    // reserved[14..17] clamp count = 1
    TEST_ASSERT_EQ_U32(1u, out[26]);
    TEST_ASSERT_EQ_U32(0u, out[27]);
    TEST_ASSERT_EQ_U32(0u, out[28]);
    TEST_ASSERT_EQ_U32(0u, out[29]);
    // reserved[18..21] seed_loaded = 7
    TEST_ASSERT_EQ_U32(7u, out[30]);
    TEST_ASSERT_EQ_U32(0u, out[31]);
    TEST_ASSERT_EQ_U32(0u, out[32]);
    TEST_ASSERT_EQ_U32(0u, out[33]);
    // reserved[22..25] seed_confirmed = 5
    TEST_ASSERT_EQ_U32(5u, out[34]);
    TEST_ASSERT_EQ_U32(0u, out[35]);
    TEST_ASSERT_EQ_U32(0u, out[36]);
    TEST_ASSERT_EQ_U32(0u, out[37]);
    // reserved[26..29] seed_rejected = 2
    TEST_ASSERT_EQ_U32(2u, out[38]);
    TEST_ASSERT_EQ_U32(0u, out[39]);
    TEST_ASSERT_EQ_U32(0u, out[40]);
    TEST_ASSERT_EQ_U32(0u, out[41]);
    // reserved[30] sync state raw
    TEST_ASSERT_EQ_U32(3u, out[42]);
}

void test_h_returns_signature() {
    ems::app::ts_test_reset();

    const uint8_t cmd = static_cast<uint8_t>('H');
    ts_send_bytewise(&cmd, 1u);

    uint8_t out[32] = {};
    const uint16_t n = ts_drain(out, sizeof(out));
    const char exp[] = "OpenEMS_v1.1";
    TEST_ASSERT_EQ_U32(std::strlen(exp), n);
    TEST_ASSERT_TRUE(std::memcmp(out, exp, std::strlen(exp)) == 0);
}

void test_o_aliases_a_realtime() {
    ems::app::ts_test_reset();

    const uint8_t cmd = static_cast<uint8_t>('O');
    ts_send_bytewise(&cmd, 1u);

    uint8_t out[128] = {};
    const uint16_t n = ts_drain(out, sizeof(out));
    TEST_ASSERT_EQ_U32(64u, n);
}

void test_write_realtime_page_rejected() {
    ems::app::ts_test_reset();

    const uint8_t w_frame[] = {
        static_cast<uint8_t>('w'),
        0x03u,
        0x00u, 0x00u,
        0x01u, 0x00u,
        0xAAu,
    };
    ts_send_bytewise(w_frame, sizeof(w_frame));

    uint8_t out[8] = {};
    const uint16_t n = ts_drain(out, sizeof(out));
    TEST_ASSERT_EQ_U32(1u, n);
    TEST_ASSERT_EQ_U32(1u, out[0]);
}

void test_f_returns_protocol_version_cstr() {
    ems::app::ts_test_reset();

    const uint8_t cmd = static_cast<uint8_t>('F');
    ts_send_bytewise(&cmd, 1u);

    uint8_t out[16] = {};
    const uint16_t n = ts_drain(out, sizeof(out));
    const char exp[] = "001";
    TEST_ASSERT_EQ_U32(std::strlen(exp), n);
    TEST_ASSERT_TRUE(std::memcmp(out, exp, std::strlen(exp)) == 0);
}

void test_c_returns_comms_ack_and_magic() {
    ems::app::ts_test_reset();

    const uint8_t cmd = static_cast<uint8_t>('C');
    ts_send_bytewise(&cmd, 1u);

    uint8_t out[8] = {};
    const uint16_t n = ts_drain(out, sizeof(out));
    TEST_ASSERT_EQ_U32(2u, n);
    TEST_ASSERT_EQ_U32(0u, out[0]);
    TEST_ASSERT_EQ_U32(0xAAu, out[1]);
}

void test_ignores_reset_probe_byte_f0() {
    ems::app::ts_test_reset();

    const uint8_t cmd = 0xF0u;
    ts_send_bytewise(&cmd, 1u);

    uint8_t out[8] = {};
    const uint16_t n = ts_drain(out, sizeof(out));
    TEST_ASSERT_EQ_U32(0u, n);
}

}  // namespace

int main() {
    test_q_returns_signature_cstr();
    test_h_returns_signature();
    test_f_returns_protocol_version_cstr();
    test_c_returns_comms_ack_and_magic();
    test_ignores_reset_probe_byte_f0();
    test_write_then_read_page_roundtrip();
    test_a_returns_64_bytes_realtime();
    test_a_packs_scheduler_diag_in_reserved();
    test_o_aliases_a_realtime();
    test_write_realtime_page_rejected();

    std::printf("tests=%d failed=%d\n", g_tests_run, g_tests_failed);
    return (g_tests_failed == 0) ? 0 : 1;
}
