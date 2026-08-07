/**
 * @file test/test_out_pins_mre.cpp
 * MRE (microRusEFI copper) INJ/IGN BSRR — make host-test-mre.
 *
 * INJ PE14/13/12/11 · IGN PD12/13/14/15 · EN PD11/PD10
 * docs/hw/pinout_mre_bringup.md
 */
#include "test/harness.h"

#include <cstdint>
#include <cstdio>

#include "engine/ecu_sched.h"
#include "hal/out_pins.h"

namespace {

constexpr uint8_t kPortD = 3u;
constexpr uint8_t kPortE = 4u;

void check_ch(uint8_t channel, uint8_t port, uint8_t pin, const char* hi,
              const char* lo) {
    using namespace ems::hal;
    out_pins_test_reset_stubs();
    out_pin_write(channel, 1u);
    CHECK_EQ(out_pins_test_bsrr_snapshot(port) & (1u << pin), (1u << pin), hi);
    out_pin_write(channel, 0u);
    CHECK_EQ(out_pins_test_bsrr_snapshot(port) & (1u << (pin + 16u)),
             (1u << (pin + 16u)), lo);
}

void test_out_pins_bsrr_mre(void) {
    section("out_pins: MRE BSRR (INJ PE14-11, IGN PD12-15)");
    using namespace ems::hal;

    check_ch(ECU_CH_INJ1, kPortE, 14u, "INJ1 high PE14", "INJ1 low PE14");
    check_ch(ECU_CH_INJ2, kPortE, 13u, "INJ2 high PE13", "INJ2 low PE13");
    check_ch(ECU_CH_INJ3, kPortE, 12u, "INJ3 high PE12", "INJ3 low PE12");
    check_ch(ECU_CH_INJ4, kPortE, 11u, "INJ4 high PE11", "INJ4 low PE11");
    check_ch(ECU_CH_IGN1, kPortD, 12u, "IGN1 high PD12", "IGN1 low PD12");
    check_ch(ECU_CH_IGN2, kPortD, 13u, "IGN2 high PD13", "IGN2 low PD13");
    check_ch(ECU_CH_IGN3, kPortD, 14u, "IGN3 high PD14", "IGN3 low PD14");
    check_ch(ECU_CH_IGN4, kPortD, 15u, "IGN4 high PD15", "IGN4 low PD15");

    out_pins_test_reset_stubs();
    out_pins_hw_init();
    const uint32_t e = out_pins_test_bsrr_snapshot(kPortE);
    const uint32_t d = out_pins_test_bsrr_snapshot(kPortD);
    const uint8_t pe_safe[] = {11u, 12u, 13u, 14u};
    for (uint8_t i = 0u; i < 4u; ++i) {
        const uint8_t pin = pe_safe[i];
        char msg[48];
        std::snprintf(msg, sizeof(msg), "safe-early: PE%u reset", pin);
        CHECK_TRUE((e & (1u << (pin + 16u))) != 0u, msg);
    }
    const uint8_t pd_safe[] = {10u, 11u, 12u, 13u, 14u, 15u};
    for (uint8_t i = 0u; i < 6u; ++i) {
        const uint8_t pin = pd_safe[i];
        char msg[48];
        std::snprintf(msg, sizeof(msg), "safe-early: PD%u reset", pin);
        CHECK_TRUE((d & (1u << (pin + 16u))) != 0u, msg);
    }

    out_pins_test_reset_stubs();
    power_stage_enable(true);
    CHECK_TRUE((out_pins_test_bsrr_snapshot(kPortD) & ((1u << 10u) | (1u << 11u)))
                   == ((1u << 10u) | (1u << 11u)),
               "power_stage_enable high → PD10|PD11 set");
    power_stage_enable(false);
    CHECK_TRUE((out_pins_test_bsrr_snapshot(kPortD)
                & ((1u << (10u + 16u)) | (1u << (11u + 16u))))
                   != 0u,
               "power_stage_enable low → PD10|PD11 reset");
}

}  // namespace

int main(void) {
    printf("OpenEMS MRE out_pins Tests\n");
    printf("============================================================\n");
    test_out_pins_bsrr_mre();
    printf("\n============================================================\n");
    printf("MRE out_pins: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
