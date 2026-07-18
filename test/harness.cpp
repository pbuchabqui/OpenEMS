#include "test/harness.h"
#include <cstdint>

int g_pass = 0;
int g_fail = 0;

// Host stubs: contadores diag do rev-limit definidos em main_stm32.cpp (só target).
// O handler do dump 'D' em ui_protocol.cpp referencia-os; no host o main não é
// linkado, por isso fornecemos as definições aqui (no target não há duplicação).
uint32_t g_dbg_rev_limit_trips = 0u;
uint32_t g_dbg_rev_limit_rpm_x10 = 0u;
uint32_t g_dbg_rev_limit_rpm_max = 0u;

void section(const char* name) {
    printf("\n[%s]\n", name);
}
