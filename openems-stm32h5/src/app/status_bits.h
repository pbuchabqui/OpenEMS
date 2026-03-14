#pragma once

#include <cstdint>

namespace ems::app {

static constexpr uint8_t STATUS_SYNC_FULL = (1u << 0);
static constexpr uint8_t STATUS_PHASE_A = (1u << 1);
static constexpr uint8_t STATUS_SENSOR_FAULT = (1u << 2);
static constexpr uint8_t STATUS_LIMP_MODE = (1u << 3);
static constexpr uint8_t STATUS_SCHED_LATE = (1u << 4);
static constexpr uint8_t STATUS_SCHED_DROP = (1u << 5);
static constexpr uint8_t STATUS_SCHED_CLAMP = (1u << 6);
static constexpr uint8_t STATUS_WBO2_FAULT = (1u << 7);

}  // namespace ems::app
