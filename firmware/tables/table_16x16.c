#include table_16x16.h"
#include <string.h>

static uint8_t find_bin_index(const uint16_t *bins, uint16_t value) {
    for (uint8_t i = 0; i < 15; i++) {
        if (value < bins[i + 1]) {
            return i;
        }
    }
    return 14;
}

void table_16x16_init(table_16x16_t *table,
                      const uint16_t *rpm_bins,
                      const uint16_t *load_bins,
                      uint16_t default_value) {
    if (!table) {
        return;
    }

    if (rpm_bins) {
        memcpy(table->rpm_bins, rpm_bins, sizeof(table->rpm_bins));
    } else {
        memcpy(table->rpm_bins, DEFAULT_RPM_BINS, sizeof(table->rpm_bins));
    }

    if (load_bins) {
        memcpy(table->load_bins, load_bins, sizeof(table->load_bins));
    } else {
        memcpy(table->load_bins, DEFAULT_LOAD_BINS, sizeof(table->load_bins));
    }

    for (int y = 0; y < 16; y++) {
        for (int x = 0; x < 16; x++) {
            table->values[y][x] = default_value;
        }
    }

    table->checksum = table_16x16_checksum(table);
}

uint16_t table_16x16_interpolate(const table_16x16_t *table, uint16_t rpm, uint16_t load) {
    if (!table) {
        return 0;
    }

    uint8_t x = find_bin_index(table->rpm_bins, rpm);
    uint8_t y = find_bin_index(table->load_bins, load);

    uint16_t x0 = table->rpm_bins[x];
    uint16_t x1 = table->rpm_bins[x + 1];
    uint16_t y0 = table->load_bins[y];
    uint16_t y1 = table->load_bins[y + 1];

    float dx = 0.0f;
    float dy = 0.0f;
    if (x1 > x0) {
        dx = (float)(rpm - x0) / (float)(x1 - x0);
    }
    if (y1 > y0) {
        dy = (float)(load - y0) / (float)(y1 - y0);
    }

    float v00 = (float)table->values[y][x];
    float v10 = (float)table->values[y][x + 1];
    float v01 = (float)table->values[y + 1][x];
    float v11 = (float)table->values[y + 1][x + 1];

    float v0 = v00 + dx * (v10 - v00);
    float v1 = v01 + dx * (v11 - v01);
    float v = v0 + dy * (v1 - v0);

    if (v < 0.0f) {
        v = 0.0f;
    }
    if (v > 65535.0f) {
        v = 65535.0f;
    }

    return (uint16_t)(v + 0.5f);
}

uint16_t table_16x16_checksum(const table_16x16_t *table) {
    if (!table) {
        return 0;
    }

    uint32_t sum = 0;
    for (int i = 0; i < 16; i++) {
        sum += table->rpm_bins[i];
        sum += table->load_bins[i];
    }
    for (int y = 0; y < 16; y++) {
        for (int x = 0; x < 16; x++) {
            sum += table->values[y][x];
        }
    }
    return (uint16_t)(sum & 0xFFFF);
}

bool table_16x16_validate(const table_16x16_t *table) {
    if (!table) {
        return false;
    }
    return table->checksum == table_16x16_checksum(table);
}
