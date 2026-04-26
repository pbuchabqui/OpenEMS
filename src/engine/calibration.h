#pragma once

#include <cstdint>

#include "engine/table3d.h"

namespace ems::engine {

extern uint8_t ve_table[kTableAxisSize][kTableAxisSize];
extern int16_t lambda_target_table_x1000[kTableAxisSize][kTableAxisSize];
extern int8_t spark_table[kTableAxisSize][kTableAxisSize];

}  // namespace ems::engine
