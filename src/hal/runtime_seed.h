#pragma once

#include <cstdint>

namespace ems::hal {

struct RuntimeSyncSeed {
    uint16_t magic;
    uint8_t version;
    uint8_t flags;
    uint16_t tooth_index;
    uint16_t _pad0;
    uint32_t sequence;
    uint32_t crc32;
};

static constexpr uint16_t RUNTIME_SYNC_SEED_MAGIC = 0x5343u; /* "SC" */
static constexpr uint8_t RUNTIME_SYNC_SEED_VERSION = 1u;
static constexpr uint8_t RUNTIME_SYNC_SEED_FLAG_VALID = (1u << 0u);
static constexpr uint8_t RUNTIME_SYNC_SEED_FLAG_FULL_SYNC = (1u << 1u);
static constexpr uint8_t RUNTIME_SYNC_SEED_FLAG_PHASE_A = (1u << 2u);

bool nvm_save_runtime_seed(const RuntimeSyncSeed* seed) noexcept;
bool nvm_load_runtime_seed(RuntimeSyncSeed* seed_out) noexcept;
bool nvm_clear_runtime_seed() noexcept;

}  // namespace ems::hal
