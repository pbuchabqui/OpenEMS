#include <cstdint>
#include <cstdio>
#include <cstring>

#define EMS_HOST_TEST 1
#include "app/tuner_studio.h"
#include "drv/ckp.h"
#include "drv/sensors.h"

namespace ems::drv {

static CkpSnapshot g_ckp = {0u, 0u, 0u, 0u, SyncState::WAIT, false};
static SensorData g_sensors = {0u, 0u, 0u, 0, 0, 0u, 0u, 0u, 0u, 0u};

CkpSnapshot ckp_snapshot() noexcept {
    return g_ckp;
}

const SensorData& sensors_get() noexcept {
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
        ems::drv::SyncState::SYNCED,
        true,
    };
    ems::drv::g_sensors = ems::drv::SensorData{
        987u,
        0u,
        321u,
        850,
        250,
        800u,
        0u,
        0u,
        0u,
        1u,
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
    TEST_ASSERT_EQ_U32(200u, out[6]);
    TEST_ASSERT_EQ_U32(40u, out[8]);
    TEST_ASSERT_EQ_U32(100u, out[9]);
    TEST_ASSERT_EQ_U32(0u, static_cast<int8_t>(out[10]));
    TEST_ASSERT_EQ_U32(0x07u, out[11]);
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
    test_o_aliases_a_realtime();
    test_write_realtime_page_rejected();

    std::printf("tests=%d failed=%d\n", g_tests_run, g_tests_failed);
    return (g_tests_failed == 0) ? 0 : 1;
}
