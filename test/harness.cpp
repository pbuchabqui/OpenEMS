#include "test/harness.h"

int g_pass = 0;
int g_fail = 0;

void section(const char* name) {
    printf("\n[%s]\n", name);
}
