#pragma once

#include <cstdint>

namespace ems::hal {

struct RuntimeSyncSeed {
    uint16_t magic;
    uint8_t version;
    uint8_t flags;
    uint16_t crank_deg;   // 0–359 (was 60-2 tooth_index; same slot)
    uint16_t decoder_tag;
    uint32_t sequence;
    uint32_t crc32;
};

static constexpr uint16_t RUNTIME_SYNC_SEED_MAGIC = 0x5343u; /* "SC" */
static constexpr uint8_t RUNTIME_SYNC_SEED_VERSION = 1u;
static constexpr uint8_t RUNTIME_SYNC_SEED_FLAG_VALID = (1u << 0u);
static constexpr uint8_t RUNTIME_SYNC_SEED_FLAG_FULL_SYNC = (1u << 1u);
static constexpr uint8_t RUNTIME_SYNC_SEED_FLAG_PHASE_A = (1u << 2u);
static constexpr uint16_t RUNTIME_SYNC_SEED_DECODER_TAG_ENCODER = 0x4000u;  // TIM2 16384
static constexpr uint16_t RUNTIME_SYNC_SEED_MAX_CRANK_DEG = 359u;
// Legacy 60-2 tag — rejected on load (this tree is encoder-only).
static constexpr uint16_t RUNTIME_SYNC_SEED_DECODER_TAG_60_2 = 0x3C02u;
static constexpr uint16_t RUNTIME_SYNC_SEED_MAX_TOOTH_INDEX_60_2 = 57u;

bool nvm_save_runtime_seed(const RuntimeSyncSeed* seed) noexcept;
bool nvm_load_runtime_seed(RuntimeSyncSeed* seed_out) noexcept;
bool nvm_clear_runtime_seed() noexcept;

inline bool runtime_seed_boot_compatible(const RuntimeSyncSeed& seed) noexcept {
    if ((seed.flags & RUNTIME_SYNC_SEED_FLAG_FULL_SYNC) == 0u) {
        return false;
    }
    if (seed.decoder_tag != RUNTIME_SYNC_SEED_DECODER_TAG_ENCODER) {
        return false;
    }
    return (seed.crank_deg <= RUNTIME_SYNC_SEED_MAX_CRANK_DEG);
}

inline bool runtime_seed_boot_compatible_60_2(const RuntimeSyncSeed& seed) noexcept {
    return runtime_seed_boot_compatible(seed);
}

inline bool runtime_seed_fast_reacquire_compatible_60_2(const RuntimeSyncSeed& seed) noexcept {
    if (!runtime_seed_boot_compatible(seed)) {
        return false;
    }
    return (seed.crank_deg == 0u);
}

#if defined(EMS_HOST_TEST)
bool nvm_test_runtime_seed_inject_slot(uint8_t slot,
                                       const RuntimeSyncSeed* seed,
                                       bool recompute_crc) noexcept;
uint8_t nvm_test_runtime_seed_slot_count() noexcept;
#endif

}  // namespace ems::hal
