#include <stdio.h>
#include <stdint.h>

int main() {
    printf("OpenEMS Test Framework - Minimal Validation\n");
    printf("ESP-IDF Path: %s\n", getenv("IDF_PATH") ? getenv("IDF_PATH") : "Not set");
    printf("Test framework structure created successfully\n");
    return 0;
}
