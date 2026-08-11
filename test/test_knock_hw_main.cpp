/**
 * @file test/test_knock_hw_main.cpp
 * @brief main() do binário standalone `make host-test-knock-hw`
 *        (-DEMS_KNOCK_HW_PRESENT=1). Mesmo padrão de test_out_pins_vgt6.cpp —
 *        binário próprio, contagem PASS/FAIL da suite principal intocada.
 *        Lógica de teste em test_knock_hw_wiring.cpp (partilhada com a
 *        suite principal, que a compila com EMS_KNOCK_HW_PRESENT=0).
 */
#include "test/harness.h"

#include <cstdio>

void test_knock_window_scheduler_wiring(void);
void test_knock_window_encoder_arm_wiring(void);

int main(void) {
    printf("OpenEMS Knock HW-Present Wiring Test\n");
    printf("============================================================\n");
    test_knock_window_scheduler_wiring();
    test_knock_window_encoder_arm_wiring();
    printf("\n============================================================\n");
    printf("Knock HW wiring: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
