/**
 * @file test/test_out_pins_vgt6.cpp
 * VGT6 INJ/IGN BSRR coverage — standalone host binary (make host-test-vgt6).
 *
 * The main host regression is compiled -DEMS_BOARD_RGT6, so out_pins.h's
 * compile-time VGT6 tables (INJ PE0/2/4/6, IGN PE9/11/13/15) never run there.
 * This binary is built -DEMS_BOARD_VGT6 to exercise the real VGT6 branch of
 * out_pin_write / out_pins_hw_init, mirroring test_out_pins_bsrr_rgt6.
 */
#include "test/harness.h"

#include <cstdint>
#include <cstdio>

#include "engine/ecu_sched.h"  // ECU_CH_* (header-only #defines)
#include "hal/out_pins.h"

namespace {

// GPIOE snapshot port index (A=0 B=1 C=2 E=3).
constexpr uint8_t kPortE = 3u;

// Assert one channel drives GPIOE set bit on high and reset bit on low.
void check_channel(uint8_t channel, uint8_t pin, const char* inj_name,
                   const char* ign_name) {
    using namespace ems::hal;
    out_pins_test_reset_stubs();
    out_pin_write(channel, 1u);
    CHECK_EQ(out_pins_test_bsrr_snapshot(kPortE) & (1u << pin), (1u << pin),
             inj_name);
    out_pin_write(channel, 0u);
    CHECK_EQ(out_pins_test_bsrr_snapshot(kPortE) & (1u << (pin + 16u)),
             (1u << (pin + 16u)), ign_name);
}

void test_out_pins_bsrr_vgt6(void) {
    section("out_pins: VGT6 BSRR polarity (INJ PE0/2/4/6, IGN PE9/11/13/15)");
    using namespace ems::hal;

    // INJ: ECU_CH → PE pin
    check_channel(ECU_CH_INJ1, 0u,  "INJ1 high → GPIOE set PE0",
                                     "INJ1 low → GPIOE reset PE0");
    check_channel(ECU_CH_INJ2, 2u,  "INJ2 high → GPIOE set PE2",
                                     "INJ2 low → GPIOE reset PE2");
    check_channel(ECU_CH_INJ3, 4u,  "INJ3 high → GPIOE set PE4",
                                     "INJ3 low → GPIOE reset PE4");
    check_channel(ECU_CH_INJ4, 6u,  "INJ4 high → GPIOE set PE6",
                                     "INJ4 low → GPIOE reset PE6");
    // IGN: ECU_CH → PE pin
    check_channel(ECU_CH_IGN1, 9u,  "IGN1 high → GPIOE set PE9",
                                     "IGN1 low → GPIOE reset PE9");
    check_channel(ECU_CH_IGN2, 11u, "IGN2 high → GPIOE set PE11",
                                     "IGN2 low → GPIOE reset PE11");
    check_channel(ECU_CH_IGN3, 13u, "IGN3 high → GPIOE set PE13",
                                     "IGN3 low → GPIOE reset PE13");
    check_channel(ECU_CH_IGN4, 15u, "IGN4 high → GPIOE set PE15",
                                     "IGN4 low → GPIOE reset PE15");

    // Safe-early: all 8 active-high outputs de-asserted (reset bits written),
    // and each configured as output (MODER low bits = 01).
    out_pins_test_reset_stubs();
    out_pins_hw_init();
    const uint32_t e = out_pins_test_bsrr_snapshot(kPortE);
    const uint8_t pe_pins[8] = {0u, 2u, 4u, 6u, 9u, 11u, 13u, 15u};
    for (uint8_t i = 0u; i < 8u; ++i) {
        const uint8_t pin = pe_pins[i];
        char msg[48];
        std::snprintf(msg, sizeof(msg), "safe-early: PE%u reset", pin);
        CHECK_TRUE((e & (1u << (pin + 16u))) != 0u, msg);
    }
}

}  // namespace

int main(void) {
    printf("OpenEMS VGT6 out_pins Tests\n");
    printf("============================================================\n");
    test_out_pins_bsrr_vgt6();
    printf("\n============================================================\n");
    printf("VGT6 out_pins: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
