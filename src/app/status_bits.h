#pragma once

#include <cstdint>

namespace ems::app {

static constexpr uint16_t STATUS_SYNC_FULL = (1u << 0);
static constexpr uint16_t STATUS_PHASE_A = (1u << 1);
static constexpr uint16_t STATUS_SENSOR_FAULT = (1u << 2);
static constexpr uint16_t STATUS_LIMP_MODE = (1u << 3);
static constexpr uint16_t STATUS_ETB_LIMP = (1u << 4);
static constexpr uint16_t STATUS_XTAU_LEARN = (1u << 5);
static constexpr uint16_t STATUS_SCHED_LATE = (1u << 6);
static constexpr uint16_t STATUS_SCHED_DROP = (1u << 7);
static constexpr uint16_t STATUS_SCHED_CLAMP = (1u << 8);
static constexpr uint16_t STATUS_WBO2_FAULT = (1u << 9);
static constexpr uint16_t STATUS_TLE8888_FAULT = (1u << 10);
static constexpr uint16_t STATUS_IGN_SEQUENTIAL = (1u << 11);
static constexpr uint16_t STATUS_REV_LIMIT     = (1u << 12);  // fuel cut active
static constexpr uint16_t STATUS_LAUNCH_ACTIVE = (1u << 13);  // launch control holding
static constexpr uint16_t STATUS_TC_ACTIVE     = (1u << 14);  // traction control reducing

}  // namespace ems::app
