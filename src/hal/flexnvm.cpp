#include "hal/flexnvm.h"
#include "hal/runtime_seed.h"

#include <cstdint>
#include <cstring>

namespace {

constexpr uint8_t kLtftRows = 16u;
constexpr uint8_t kLtftCols = 16u;
constexpr uint16_t kLtftSize = static_cast<uint16_t>(kLtftRows * kLtftCols);
constexpr uint8_t kKnockRows = 8u;
constexpr uint8_t kKnockCols = 8u;
constexpr uint16_t kKnockSize = static_cast<uint16_t>(kKnockRows * kKnockCols);
// knock_map offset in FlexRAM: logo após os 256 bytes do LTFT
constexpr uint16_t kKnockEeeOffset = kLtftSize;
constexpr uint16_t kRuntimeSeedOffset = static_cast<uint16_t>(kKnockEeeOffset + kKnockSize);
constexpr uint8_t kRuntimeSeedSlots = 8u;
constexpr uint16_t kRuntimeSeedSlotBytes = 32u;
constexpr uint16_t kRuntimeSeedStorageBytes = static_cast<uint16_t>(
    static_cast<uint16_t>(kRuntimeSeedSlots) * kRuntimeSeedSlotBytes);
constexpr uint16_t kCalPageSize = 4096u;
constexpr uint8_t kCalPages = 32u;
constexpr uint32_t kCcifTimeoutPolls = 1'200'000u;  // ~10 ms @ 120 MHz

#if defined(EMS_HOST_TEST)

constexpr uint8_t kFstatCcif = (1u << 7u);
static uint8_t g_fstat = kFstatCcif;
static int8_t g_ltft[kLtftSize] = {};
static int8_t g_knock[kKnockSize] = {};
static uint8_t g_cal[kCalPages][kCalPageSize] = {};
static uint8_t g_runtime_seed[kRuntimeSeedStorageBytes] = {};
static uint32_t g_busy_polls_remaining = 0u;
static uint32_t g_erase_count = 0u;
static uint32_t g_program_count = 0u;

static bool wait_ccif_ready() noexcept {
    for (uint32_t i = 0u; i < kCcifTimeoutPolls; ++i) {
        if ((g_fstat & kFstatCcif) != 0u) {
            return true;
        }
        if (g_busy_polls_remaining > 0u) {
            --g_busy_polls_remaining;
            if (g_busy_polls_remaining == 0u) {
                g_fstat |= kFstatCcif;
            }
        }
    }
    return false;
}

static inline uint16_t ltft_index(uint8_t rpm_i, uint8_t load_i) noexcept {
    return static_cast<uint16_t>(static_cast<uint16_t>(rpm_i) * kLtftCols + load_i);
}

static inline uint16_t knock_index(uint8_t rpm_i, uint8_t load_i) noexcept {
    return static_cast<uint16_t>(static_cast<uint16_t>(rpm_i) * kKnockCols + load_i);
}

#else

// FTFE (Flash memory module)
#define FTFE_FSTAT (*reinterpret_cast<volatile uint8_t*>(0x40020000u))
#define FTFE_FCCOB3 (*reinterpret_cast<volatile uint8_t*>(0x40020004u))
#define FTFE_FCCOB2 (*reinterpret_cast<volatile uint8_t*>(0x40020005u))
#define FTFE_FCCOB1 (*reinterpret_cast<volatile uint8_t*>(0x40020006u))
#define FTFE_FCCOB0 (*reinterpret_cast<volatile uint8_t*>(0x40020007u))
#define FTFE_FCCOB7 (*reinterpret_cast<volatile uint8_t*>(0x40020008u))
#define FTFE_FCCOB6 (*reinterpret_cast<volatile uint8_t*>(0x40020009u))
#define FTFE_FCCOB5 (*reinterpret_cast<volatile uint8_t*>(0x4002000Au))
#define FTFE_FCCOB4 (*reinterpret_cast<volatile uint8_t*>(0x4002000Bu))

constexpr uint8_t kFstatCcif = (1u << 7u);
constexpr uint8_t kFstatRdcolerr = (1u << 6u);
constexpr uint8_t kFstatAccerr = (1u << 5u);
constexpr uint8_t kFstatFpviol = (1u << 4u);
constexpr uint8_t kFstatErrorMask = kFstatRdcolerr | kFstatAccerr | kFstatFpviol;

constexpr uint8_t kCmdEraseSector = 0x09u;
constexpr uint8_t kCmdProgramLongword = 0x06u;

constexpr uintptr_t kEeeBase = 0x14000000u;      // FlexRAM mapped as emulated EEPROM
constexpr uintptr_t kFlexNvmBase = 0x10000000u;  // FlexNVM (D-Flash)

static bool wait_ccif_ready() noexcept {
    for (uint32_t i = 0u; i < kCcifTimeoutPolls; ++i) {
        if ((FTFE_FSTAT & kFstatCcif) != 0u) {
            return true;
        }
    }
    return false;
}

static bool ftfe_launch_command() noexcept {
    if (!wait_ccif_ready()) {
        return false;
    }

    // Clear sticky access/protection errors before command launch.
    FTFE_FSTAT = kFstatErrorMask;
    FTFE_FSTAT = kFstatCcif;

    if (!wait_ccif_ready()) {
        return false;
    }
    return (FTFE_FSTAT & (kFstatErrorMask | 0x01u)) == 0u;
}

static bool ftfe_erase_sector(uint32_t addr) noexcept {
    FTFE_FCCOB0 = kCmdEraseSector;
    FTFE_FCCOB1 = static_cast<uint8_t>((addr >> 16u) & 0xFFu);
    FTFE_FCCOB2 = static_cast<uint8_t>((addr >> 8u) & 0xFFu);
    FTFE_FCCOB3 = static_cast<uint8_t>(addr & 0xFFu);
    return ftfe_launch_command();
}

static bool ftfe_program_longword(uint32_t addr, const uint8_t* data) noexcept {
    FTFE_FCCOB0 = kCmdProgramLongword;
    FTFE_FCCOB1 = static_cast<uint8_t>((addr >> 16u) & 0xFFu);
    FTFE_FCCOB2 = static_cast<uint8_t>((addr >> 8u) & 0xFFu);
    FTFE_FCCOB3 = static_cast<uint8_t>(addr & 0xFFu);
    FTFE_FCCOB4 = data[0];
    FTFE_FCCOB5 = data[1];
    FTFE_FCCOB6 = data[2];
    FTFE_FCCOB7 = data[3];
    return ftfe_launch_command();
}

static inline uint16_t ltft_index(uint8_t rpm_i, uint8_t load_i) noexcept {
    return static_cast<uint16_t>(static_cast<uint16_t>(rpm_i) * kLtftCols + load_i);
}

static inline uint16_t knock_index(uint8_t rpm_i, uint8_t load_i) noexcept {
    return static_cast<uint16_t>(static_cast<uint16_t>(rpm_i) * kKnockCols + load_i);
}

#endif

uint32_t crc32_update(uint32_t crc, uint8_t data) noexcept {
    crc ^= data;
    for (uint8_t i = 0u; i < 8u; ++i) {
        const uint32_t mask = static_cast<uint32_t>(-(static_cast<int32_t>(crc & 1u)));
        crc = (crc >> 1u) ^ (0xEDB88320u & mask);
    }
    return crc;
}

uint32_t runtime_seed_crc32(const ems::hal::RuntimeSyncSeed& seed) noexcept {
    uint32_t crc = 0xFFFFFFFFu;
    const uint8_t* p = reinterpret_cast<const uint8_t*>(&seed);
    const uint16_t sz = static_cast<uint16_t>(sizeof(seed) - sizeof(seed.crc32));
    for (uint16_t i = 0u; i < sz; ++i) {
        crc = crc32_update(crc, p[i]);
    }
    return ~crc;
}

bool runtime_seed_is_valid(const ems::hal::RuntimeSyncSeed& seed) noexcept {
    if (seed.magic != ems::hal::RUNTIME_SYNC_SEED_MAGIC) {
        return false;
    }
    if (seed.version != ems::hal::RUNTIME_SYNC_SEED_VERSION) {
        return false;
    }
    if ((seed.flags & ems::hal::RUNTIME_SYNC_SEED_FLAG_VALID) == 0u) {
        return false;
    }
    return (runtime_seed_crc32(seed) == seed.crc32);
}

bool runtime_seed_sequence_newer(uint32_t seq_a, uint32_t seq_b) noexcept {
    return static_cast<int32_t>(seq_a - seq_b) > 0;
}

bool runtime_seed_read_slot(uint8_t slot, ems::hal::RuntimeSyncSeed* out) noexcept {
    if (out == nullptr || slot >= kRuntimeSeedSlots) {
        return false;
    }
#if defined(EMS_HOST_TEST)
    const uint16_t off = static_cast<uint16_t>(slot * kRuntimeSeedSlotBytes);
    std::memcpy(out, &g_runtime_seed[off], sizeof(ems::hal::RuntimeSyncSeed));
    return true;
#else
    const volatile uint8_t* const eee = reinterpret_cast<const volatile uint8_t*>(
        kEeeBase + kRuntimeSeedOffset + static_cast<uint16_t>(slot * kRuntimeSeedSlotBytes));
    uint8_t* dst = reinterpret_cast<uint8_t*>(out);
    for (uint16_t i = 0u; i < sizeof(ems::hal::RuntimeSyncSeed); ++i) {
        dst[i] = eee[i];
    }
    return true;
#endif
}

bool runtime_seed_write_slot(uint8_t slot, const ems::hal::RuntimeSyncSeed& seed) noexcept {
    if (slot >= kRuntimeSeedSlots) {
        return false;
    }
    if (!wait_ccif_ready()) {
        return false;
    }
#if defined(EMS_HOST_TEST)
    const uint16_t off = static_cast<uint16_t>(slot * kRuntimeSeedSlotBytes);
    std::memset(&g_runtime_seed[off], 0xFF, kRuntimeSeedSlotBytes);
    std::memcpy(&g_runtime_seed[off], &seed, sizeof(seed));
    return true;
#else
    volatile uint8_t* const eee = reinterpret_cast<volatile uint8_t*>(
        kEeeBase + kRuntimeSeedOffset + static_cast<uint16_t>(slot * kRuntimeSeedSlotBytes));
    const uint8_t* src = reinterpret_cast<const uint8_t*>(&seed);
    for (uint16_t i = 0u; i < kRuntimeSeedSlotBytes; ++i) {
        eee[i] = (i < sizeof(seed)) ? src[i] : 0xFFu;
    }
    return true;
#endif
}

}  // namespace

namespace ems::hal {

bool nvm_write_ltft(uint8_t rpm_i, uint8_t load_i, int8_t val) noexcept {
    if (rpm_i >= kLtftRows || load_i >= kLtftCols) {
        return false;
    }
    if (!wait_ccif_ready()) {
        return false;
    }

    const uint16_t idx = ltft_index(rpm_i, load_i);
#if defined(EMS_HOST_TEST)
    g_ltft[idx] = val;
#else
    volatile int8_t* const eee = reinterpret_cast<volatile int8_t*>(kEeeBase);
    eee[idx] = val;
#endif
    return true;
}

int8_t nvm_read_ltft(uint8_t rpm_i, uint8_t load_i) noexcept {
    if (rpm_i >= kLtftRows || load_i >= kLtftCols) {
        return 0;
    }

    const uint16_t idx = ltft_index(rpm_i, load_i);
#if defined(EMS_HOST_TEST)
    return g_ltft[idx];
#else
    const volatile int8_t* const eee = reinterpret_cast<const volatile int8_t*>(kEeeBase);
    return eee[idx];
#endif
}

// ---------------------------------------------------------------------------
// knock_map[8×8] — FlexRAM (EEPROM emulada), offset 256 bytes após LTFT
// Unidade: 0,1° de avanço (valor negativo = retardo de ignição)
// Range: –12,7° .. +12,7° (int8_t: –127 .. +127 × 0,1°)
// ---------------------------------------------------------------------------

bool nvm_write_knock(uint8_t rpm_i, uint8_t load_i, int8_t retard_deci_deg) noexcept {
    if (rpm_i >= kKnockRows || load_i >= kKnockCols) {
        return false;
    }
    if (!wait_ccif_ready()) {
        return false;
    }

    const uint16_t idx = knock_index(rpm_i, load_i);
#if defined(EMS_HOST_TEST)
    g_knock[idx] = retard_deci_deg;
#else
    // FlexRAM mapeado como EEPROM: offset kKnockEeeOffset (256) após LTFT
    volatile int8_t* const eee = reinterpret_cast<volatile int8_t*>(kEeeBase + kKnockEeeOffset);
    eee[idx] = retard_deci_deg;
#endif
    return true;
}

int8_t nvm_read_knock(uint8_t rpm_i, uint8_t load_i) noexcept {
    if (rpm_i >= kKnockRows || load_i >= kKnockCols) {
        return 0;
    }

    const uint16_t idx = knock_index(rpm_i, load_i);
#if defined(EMS_HOST_TEST)
    return g_knock[idx];
#else
    const volatile int8_t* const eee =
        reinterpret_cast<const volatile int8_t*>(kEeeBase + kKnockEeeOffset);
    return eee[idx];
#endif
}

void nvm_reset_knock_map() noexcept {
#if defined(EMS_HOST_TEST)
    std::memset(g_knock, 0, sizeof(g_knock));
#else
    // Escreve zero célula a célula via EEPROM emulada (wear-leveling por HW)
    volatile int8_t* const eee = reinterpret_cast<volatile int8_t*>(kEeeBase + kKnockEeeOffset);
    for (uint16_t i = 0u; i < kKnockSize; ++i) {
        eee[i] = 0;
    }
#endif
}

bool nvm_save_calibration(uint8_t page, const uint8_t* data, uint16_t len) noexcept {
    if (data == nullptr || len == 0u || len > kCalPageSize || page >= kCalPages) {
        return false;
    }

    if (!wait_ccif_ready()) {
        return false;
    }

#if defined(EMS_HOST_TEST)
    ++g_erase_count;
    std::memset(g_cal[page], 0xFF, kCalPageSize);
    std::memcpy(g_cal[page], data, len);
    ++g_program_count;
    return true;
#else
    const uint32_t base_addr = static_cast<uint32_t>(kFlexNvmBase + static_cast<uintptr_t>(page) * kCalPageSize);
    if (!ftfe_erase_sector(base_addr)) {
        return false;
    }

    uint8_t block[4] = {0xFFu, 0xFFu, 0xFFu, 0xFFu};
    for (uint16_t off = 0u; off < len; off = static_cast<uint16_t>(off + 4u)) {
        const uint16_t remaining = static_cast<uint16_t>(len - off);
        const uint8_t n = (remaining >= 4u) ? 4u : static_cast<uint8_t>(remaining);
        block[0] = 0xFFu;
        block[1] = 0xFFu;
        block[2] = 0xFFu;
        block[3] = 0xFFu;
        for (uint8_t i = 0u; i < n; ++i) {
            block[i] = data[static_cast<uint16_t>(off + i)];
        }
        if (!ftfe_program_longword(base_addr + off, block)) {
            return false;
        }
    }
    return true;
#endif
}

bool nvm_load_calibration(uint8_t page, uint8_t* data, uint16_t len) noexcept {
    if (data == nullptr || len == 0u || len > kCalPageSize || page >= kCalPages) {
        return false;
    }

#if defined(EMS_HOST_TEST)
    std::memcpy(data, g_cal[page], len);
#else
    const volatile uint8_t* const base = reinterpret_cast<const volatile uint8_t*>(
        kFlexNvmBase + static_cast<uintptr_t>(page) * kCalPageSize);
    for (uint16_t i = 0u; i < len; ++i) {
        data[i] = base[i];
    }
#endif
    return true;
}

bool nvm_save_runtime_seed(const RuntimeSyncSeed* seed) noexcept {
    if (seed == nullptr) {
        return false;
    }
    RuntimeSyncSeed slot_seed{};
    RuntimeSyncSeed write = *seed;
    uint8_t target_slot = 0u;
    uint8_t best_slot = 0u;
    bool have_best = false;
    uint32_t last_seq = 0u;
    for (uint8_t s = 0u; s < kRuntimeSeedSlots; ++s) {
        if (!runtime_seed_read_slot(s, &slot_seed) || !runtime_seed_is_valid(slot_seed)) {
            continue;
        }
        if (!have_best || runtime_seed_sequence_newer(slot_seed.sequence, last_seq)) {
            have_best = true;
            last_seq = slot_seed.sequence;
            best_slot = s;
        }
    }
    if (have_best) {
        target_slot = static_cast<uint8_t>((best_slot + 1u) % kRuntimeSeedSlots);
    } else {
        target_slot = 0u;
    }

    write.magic = RUNTIME_SYNC_SEED_MAGIC;
    write.version = RUNTIME_SYNC_SEED_VERSION;
    write.flags = static_cast<uint8_t>(write.flags | RUNTIME_SYNC_SEED_FLAG_VALID);
    write.sequence = last_seq + 1u;
    write.crc32 = runtime_seed_crc32(write);
    return runtime_seed_write_slot(target_slot, write);
}

bool nvm_load_runtime_seed(RuntimeSyncSeed* seed_out) noexcept {
    if (seed_out == nullptr) {
        return false;
    }
    RuntimeSyncSeed slot_seed{};
    RuntimeSyncSeed best{};
    bool have_best = false;
    for (uint8_t s = 0u; s < kRuntimeSeedSlots; ++s) {
        if (!runtime_seed_read_slot(s, &slot_seed) || !runtime_seed_is_valid(slot_seed)) {
            continue;
        }
        if (!have_best || runtime_seed_sequence_newer(slot_seed.sequence, best.sequence)) {
            have_best = true;
            best = slot_seed;
        }
    }
    if (!have_best) {
        std::memset(seed_out, 0, sizeof(*seed_out));
        return false;
    }
    *seed_out = best;
    return true;
}

bool nvm_clear_runtime_seed() noexcept {
    RuntimeSyncSeed cleared{};
    cleared.magic = RUNTIME_SYNC_SEED_MAGIC;
    cleared.version = RUNTIME_SYNC_SEED_VERSION;
    cleared.flags = 0u;
    cleared.tooth_index = 0u;
    cleared.sequence = 0u;
    cleared.crc32 = runtime_seed_crc32(cleared);
    bool ok = true;
    for (uint8_t s = 0u; s < kRuntimeSeedSlots; ++s) {
        ok = runtime_seed_write_slot(s, cleared) && ok;
    }
    return ok;
}

#if defined(EMS_HOST_TEST)
bool nvm_test_runtime_seed_inject_slot(uint8_t slot,
                                       const RuntimeSyncSeed* seed,
                                       bool recompute_crc) noexcept {
    if (seed == nullptr || slot >= kRuntimeSeedSlots) {
        return false;
    }
    RuntimeSyncSeed write = *seed;
    if (recompute_crc) {
        write.crc32 = runtime_seed_crc32(write);
    }
    return runtime_seed_write_slot(slot, write);
}

uint8_t nvm_test_runtime_seed_slot_count() noexcept {
    return kRuntimeSeedSlots;
}

void nvm_test_reset() noexcept {
    g_fstat = kFstatCcif;
    g_busy_polls_remaining = 0u;
    g_erase_count = 0u;
    g_program_count = 0u;
    std::memset(g_ltft, 0, sizeof(g_ltft));
    std::memset(g_knock, 0, sizeof(g_knock));
    std::memset(g_runtime_seed, 0xFF, sizeof(g_runtime_seed));
    std::memset(g_cal, 0xFF, sizeof(g_cal));
}

void nvm_test_set_ccif_busy_polls(uint32_t polls) noexcept {
    g_busy_polls_remaining = polls;
    if (polls == 0u) {
        g_fstat |= kFstatCcif;
    } else {
        g_fstat &= static_cast<uint8_t>(~kFstatCcif);
    }
}

uint32_t nvm_test_erase_count() noexcept {
    return g_erase_count;
}

uint32_t nvm_test_program_count() noexcept {
    return g_program_count;
}
#endif

}  // namespace ems::hal
