/**
 * @file app/datalog.cpp
 * @brief Ring buffer datalog with SD card flush via SDMMC1.
 *
 * Raw LBA append (no filesystem). Each ignition cycle starts at the
 * next LBA after the previous session. A header block at LBA 0 tracks
 * the current write position.
 */

#ifndef EMS_HOST_TEST

#include "app/datalog.h"
#include "hal/sdmmc.h"
#include <cstring>

namespace {

static constexpr uint32_t kRingSize     = 4096u;
static constexpr uint32_t kBlockSize    = 512u;
static constexpr uint32_t kHeaderLba    = 0u;
static constexpr uint32_t kDataStartLba = 1u;

alignas(4) static uint8_t g_ring[kRingSize];
static volatile uint32_t g_head     = 0u;
static volatile uint32_t g_tail     = 0u;
static volatile uint32_t g_next_lba = kDataStartLba;
static volatile uint32_t g_dropped  = 0u;
static volatile bool     g_active   = false;

static uint32_t ring_used() noexcept {
    uint32_t h = g_head;
    uint32_t t = g_tail;
    return (h >= t) ? (h - t) : (kRingSize - t + h);
}

}  // namespace

namespace ems::app {

void datalog_init() noexcept {
    g_head = 0u;
    g_tail = 0u;
    g_dropped = 0u;
    g_active = ems::hal::sdmmc_card_present();
}

void datalog_append(const DatalogEntry& entry) noexcept {
    if (!g_active) { return; }

    uint32_t h = g_head;
    uint32_t free = kRingSize - ring_used();
    if (free < sizeof(DatalogEntry)) {
        ++g_dropped;
        return;
    }

    const auto* src = reinterpret_cast<const uint8_t*>(&entry);
    for (uint32_t i = 0u; i < sizeof(DatalogEntry); ++i) {
        g_ring[h] = src[i];
        h = (h + 1u) % kRingSize;
    }
    g_head = h;
}

void datalog_flush() noexcept {
    if (!g_active) { return; }

    while (ring_used() >= kBlockSize) {
        alignas(4) uint8_t block[kBlockSize];
        uint32_t t = g_tail;
        for (uint32_t i = 0u; i < kBlockSize; ++i) {
            block[i] = g_ring[t];
            t = (t + 1u) % kRingSize;
        }

        if (!ems::hal::sdmmc_write_block(g_next_lba, block)) {
            g_active = false;
            return;
        }
        g_tail = t;
        ++g_next_lba;
    }
}

bool datalog_is_active() noexcept {
    return g_active;
}

uint32_t datalog_dropped_count() noexcept {
    return g_dropped;
}

}  // namespace ems::app

#else  // EMS_HOST_TEST

#include "app/datalog.h"

namespace ems::app {

void     datalog_init() noexcept {}
void     datalog_append(const DatalogEntry&) noexcept {}
void     datalog_flush() noexcept {}
bool     datalog_is_active() noexcept { return false; }
uint32_t datalog_dropped_count() noexcept { return 0u; }

}  // namespace ems::app

#endif  // EMS_HOST_TEST
