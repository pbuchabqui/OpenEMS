#ifndef TABLE_16X16_H
#define TABLE_16X16_H

#include <stdint.h>
#include <stdbool.h>
#include "config/engine_config.h"

#ifdef __cplusplus
extern "C" {
#endif

void table_16x16_init(table_16x16_t *table,
                      const uint16_t *rpm_bins,
                      const uint16_t *load_bins,
                      uint16_t default_value);

uint16_t table_16x16_interpolate(const table_16x16_t *table, uint16_t rpm, uint16_t load);

uint16_t table_16x16_checksum(const table_16x16_t *table);

bool table_16x16_validate(const table_16x16_t *table);

#ifdef __cplusplus
}
#endif

#endif // TABLE_16X16_H
