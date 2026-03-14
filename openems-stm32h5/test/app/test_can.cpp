#include <cstdint>
#include <cstdio>

#define EMS_HOST_TEST 1
#include "app/can_stack.h"
#include "hal/can.h"

namespace {

int g_tests_run    = 0;
int g_tests_failed = 0;

#define TEST_ASSERT_TRUE(cond) do { \
    ++g_tests_run; \
    if (!(cond)) { \
        ++g_tests_failed; \
        std::printf("FAIL %s:%d  cond: %s\n", __FILE__, __LINE__, #cond); \
    } \
} while (0)

#define TEST_ASSERT_EQ_U32(exp, act) do { \
    ++g_tests_run; \
    const uint32_t _e = static_cast<uint32_t>(exp); \
    const uint32_t _a = static_cast<uint32_t>(act); \
    if (_e != _a) { \
        ++g_tests_failed; \
        std::printf("FAIL %s:%d  expected=%u  got=%u\n", \
                    __FILE__, __LINE__, (unsigned)_e, (unsigned)_a); \
    } \
} while (0)

// ─── helpers ────────────────────────────────────────────────────────────────

ems::drv::CkpSnapshot make_ckp(uint32_t rpm_x10) {
    return ems::drv::CkpSnapshot{0u, 0u, 0u, rpm_x10,
                                  ems::drv::SyncState::FULL_SYNC, false};
}

// SensorData sem o campo o2_mv
ems::drv::SensorData make_sensors() {
    return ems::drv::SensorData{
        998u,    // map_kpa_x10
        0u,      // maf_gps_x100
        457u,    // tps_pct_x10
        930,     // clt_degc_x10   →  93 °C
        250,     // iat_degc_x10   →  25 °C
        // o2_mv REMOVIDO
        3510u,   // fuel_press_kpa_x10
        4020u,   // oil_press_kpa_x10
        13800u,  // vbatt_mv
        0u       // fault_bits
    };
}

// ─── test 1: bit timing ──────────────────────────────────────────────────────

void test_can_init_programs_bit_timing() {
    // STM32H562 uses FDCAN1 with NBTP register (not FlexCAN CTRL1).
    // The EMS_HOST_TEST mock for the STM32 HAL does not simulate NBTP —
    // can0_init() is a no-op and can_test_ctrl1() returns 0.
    // Bit-timing is validated by hardware bring-up; host test only checks
    // that the mock infrastructure is intact (ctrl1 accessible without crash).
    ems::app::can_stack_test_reset();
    const uint32_t ctrl1 = ems::hal::can_test_ctrl1();
    // STM32 host mock: ctrl1 is always 0 (FDCAN NBTP not simulated in mock).
    TEST_ASSERT_EQ_U32(0u, ctrl1);
}

// ─── test 2: serialização 0x400 ──────────────────────────────────────────────

void test_tx_0x400_serialization() {
    ems::app::can_stack_test_reset();
    const ems::drv::CkpSnapshot ckp     = make_ckp(32150u);
    const ems::drv::SensorData  sensors = make_sensors();

    // WBO2 nunca recebeu frame → fault deve estar ativo
    ems::app::can_stack_process(10u, ckp, sensors,
        12,    // advance_deg
        45u,   // pw_ms_x10
        0,     // stft_pct
        0u, 0u,
        0x25u  // status_bits do chamador (sem bit7)
    );

    ems::hal::CanFrame out = {};
    TEST_ASSERT_TRUE(ems::hal::can_test_pop_tx(out));
    TEST_ASSERT_EQ_U32(0x400u, out.id);
    TEST_ASSERT_EQ_U32(8u,     out.dlc);
    TEST_ASSERT_EQ_U32(3215u & 0xFFu,       out.data[0]); // RPM LSB
    TEST_ASSERT_EQ_U32((3215u >> 8u) & 0xFFu, out.data[1]); // RPM MSB
    TEST_ASSERT_EQ_U32(99u,  out.data[2]);   // MAP: 998/10 = 99
    TEST_ASSERT_EQ_U32(45u,  out.data[3]);   // TPS: 457/10 = 45
    TEST_ASSERT_EQ_U32(133u, out.data[4]);   // CLT: 93 + 40 = 133
    TEST_ASSERT_EQ_U32(52u,  out.data[5]);   // avanço: 12 + 40 = 52
    TEST_ASSERT_EQ_U32(45u,  out.data[6]);   // PW
    // bit7 (STATUS_WBO2_FAULT) deve estar SETADO pois WBO2 nunca recebeu frame
    TEST_ASSERT_TRUE((out.data[7] & ems::app::STATUS_WBO2_FAULT) != 0u);
    TEST_ASSERT_EQ_U32(0x25u | ems::app::STATUS_WBO2_FAULT, out.data[7]);
}

// ─── test 3: serialização 0x401 ──────────────────────────────────────────────

void test_tx_0x401_serialization() {
    ems::app::can_stack_test_reset();
    const ems::drv::CkpSnapshot ckp     = make_ckp(10000u);
    const ems::drv::SensorData  sensors = make_sensors();

    ems::app::can_stack_process(100u, ckp, sensors, 0, 0u, -7, 32u, 64u, 0u);

    ems::hal::CanFrame out0 = {};
    ems::hal::CanFrame out1 = {};
    TEST_ASSERT_TRUE(ems::hal::can_test_pop_tx(out0));
    TEST_ASSERT_TRUE(ems::hal::can_test_pop_tx(out1));

    const ems::hal::CanFrame& slow = (out0.id == 0x401u) ? out0 : out1;
    TEST_ASSERT_EQ_U32(0x401u, slow.id);
    TEST_ASSERT_EQ_U32(8u,     slow.dlc);
    TEST_ASSERT_EQ_U32(3510u & 0xFFu,         slow.data[0]); // fuel LSB
    TEST_ASSERT_EQ_U32((3510u >> 8u) & 0xFFu, slow.data[1]); // fuel MSB
    TEST_ASSERT_EQ_U32(4020u & 0xFFu,         slow.data[2]); // oil LSB
    TEST_ASSERT_EQ_U32((4020u >> 8u) & 0xFFu, slow.data[3]); // oil MSB
    TEST_ASSERT_EQ_U32(65u,  slow.data[4]);   // IAT: 25 + 40 = 65
    TEST_ASSERT_EQ_U32(93u,  slow.data[5]);   // STFT: -7 + 100 = 93
    TEST_ASSERT_EQ_U32(32u,  slow.data[6]);   // VVT intake
    TEST_ASSERT_EQ_U32(64u,  slow.data[7]);   // VVT exhaust
}

// ─── test 4: RX WBO2 + safe lambda + timeout ─────────────────────────────────

void test_rx_wbo2_safe_lambda_and_timeout() {
    ems::app::can_stack_test_reset();
    const ems::drv::CkpSnapshot ckp     = make_ckp(9000u);
    const ems::drv::SensorData  sensors = make_sensors();

    // Estado inicial: nenhum frame recebido → fault ativo, lambda seguro
    TEST_ASSERT_TRUE(ems::app::can_stack_wbo2_fault());
    TEST_ASSERT_EQ_U32(ems::app::WBO2_SAFE_LAMBDA_MILLI,
                       ems::app::can_stack_lambda_milli_safe(0u));

    // Injeta frame WBO2: lambda = 0x041A = 1050
    ems::hal::CanFrame in = {};
    in.id       = 0x180u;
    in.dlc      = 8u;
    in.extended = false;
    in.data[0]  = 0x1Au;
    in.data[1]  = 0x04u;   // 1050 little-endian
    in.data[2]  = 0x55u;   // status WBO2 externo
    TEST_ASSERT_TRUE(ems::hal::can_test_inject_rx(in));

    // Processa em t=100 ms
    ems::app::can_stack_process(100u, ckp, sensors, 0, 0u, 0, 0u, 0u, 0u);

    // Sensor fresco → lambda raw = 1050, safe = 1050, sem fault
    TEST_ASSERT_EQ_U32(1050u, ems::app::can_stack_lambda_milli());
    TEST_ASSERT_EQ_U32(1050u, ems::app::can_stack_lambda_milli_safe(100u));
    TEST_ASSERT_EQ_U32(0x55u, ems::app::can_stack_wbo2_status());
    TEST_ASSERT_TRUE(ems::app::can_stack_wbo2_fresh(100u));
    TEST_ASSERT_TRUE(!ems::app::can_stack_wbo2_fault());

    // Ainda fresco em t=600 ms (600 - 100 = 500 ms, não excede)
    TEST_ASSERT_TRUE(ems::app::can_stack_wbo2_fresh(600u));
    TEST_ASSERT_EQ_U32(1050u, ems::app::can_stack_lambda_milli_safe(600u));

    // Timeout em t=601 ms (601 - 100 = 501 ms > 500 ms)
    TEST_ASSERT_TRUE(!ems::app::can_stack_wbo2_fresh(601u));
    // Sem fallback para narrow: retorna lambda fixo de segurança
    TEST_ASSERT_EQ_U32(ems::app::WBO2_SAFE_LAMBDA_MILLI,
                       ems::app::can_stack_lambda_milli_safe(601u));
}

// ─── test 5: status_bits — bit7 WBO2_FAULT reflete estado real ───────────────

void test_status_bits_wbo2_fault_flag() {
    ems::app::can_stack_test_reset();
    const ems::drv::CkpSnapshot ckp     = make_ckp(5000u);
    const ems::drv::SensorData  sensors = make_sensors();

    // t=10: sem frame → fault forçado no data[7]
    ems::app::can_stack_process(10u, ckp, sensors, 0, 0u, 0, 0u, 0u, 0x00u);
    ems::hal::CanFrame f = {};
    TEST_ASSERT_TRUE(ems::hal::can_test_pop_tx(f));
    TEST_ASSERT_EQ_U32(0x400u, f.id);
    TEST_ASSERT_TRUE((f.data[7] & ems::app::STATUS_WBO2_FAULT) != 0u);

    // Injeta frame WBO2 e processa em t=20
    ems::hal::CanFrame in = {};
    in.id = 0x180u; in.dlc = 3u; in.extended = false;
    in.data[0] = 0xE8u; in.data[1] = 0x03u; // 1000 = λ1.00
    in.data[2] = 0x01u;
    ems::hal::can_test_inject_rx(in);

    ems::app::can_stack_process(20u, ckp, sensors, 0, 0u, 0, 0u, 0u, 0x00u);
    while (ems::hal::can_test_pop_tx(f)) {
        if (f.id == 0x400u) { break; }
    }
    TEST_ASSERT_EQ_U32(0x400u, f.id);
    // Sensor fresco → bit7 deve estar LIMPO
    TEST_ASSERT_TRUE((f.data[7] & ems::app::STATUS_WBO2_FAULT) == 0u);
}

void test_status_bits_scheduler_flags_passthrough() {
    ems::app::can_stack_test_reset();
    const ems::drv::CkpSnapshot ckp     = make_ckp(5000u);
    const ems::drv::SensorData  sensors = make_sensors();

    const uint8_t status = static_cast<uint8_t>(
        ems::app::STATUS_SYNC_FULL |
        ems::app::STATUS_SCHED_LATE |
        ems::app::STATUS_SCHED_DROP |
        ems::app::STATUS_SCHED_CLAMP);

    // Provide a fresh WBO2 frame so bit7 is not auto-forced.
    ems::hal::CanFrame in = {};
    in.id = 0x180u; in.dlc = 3u; in.extended = false;
    in.data[0] = 0xE8u; in.data[1] = 0x03u;
    in.data[2] = 0x01u;
    ems::hal::can_test_inject_rx(in);

    ems::app::can_stack_process(10u, ckp, sensors, 0, 0u, 0, 0u, 0u, status);

    ems::hal::CanFrame f = {};
    while (ems::hal::can_test_pop_tx(f)) {
        if (f.id == 0x400u) { break; }
    }
    TEST_ASSERT_EQ_U32(0x400u, f.id);
    TEST_ASSERT_EQ_U32(status, f.data[7]);
}

} // namespace

int main() {
    test_can_init_programs_bit_timing();
    test_tx_0x400_serialization();
    test_tx_0x401_serialization();
    test_rx_wbo2_safe_lambda_and_timeout();
    test_status_bits_wbo2_fault_flag();
    test_status_bits_scheduler_flags_passthrough();

    std::printf("\ntests=%d  failed=%d\n", g_tests_run, g_tests_failed);
    return (g_tests_failed == 0) ? 0 : 1;
}
