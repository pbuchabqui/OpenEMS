#include "hal/flexnvm.h"

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
constexpr uint16_t kCalPageSize = 4096u;
constexpr uint8_t kCalPages = 32u;
constexpr uint32_t kCcifTimeoutPolls = 1'200'000u;  // ~10 ms @ 120 MHz

#if defined(EMS_HOST_TEST)

constexpr uint8_t kFstatCcif = (1u << 7u);
static uint8_t g_fstat = kFstatCcif;
static int8_t g_ltft[kLtftSize] = {};
static int8_t g_knock[kKnockSize] = {};
static uint8_t g_cal[kCalPages][kCalPageSize] = {};
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

#if defined(EMS_HOST_TEST)
void nvm_test_reset() noexcept {
    g_fstat = kFstatCcif;
    g_busy_polls_remaining = 0u;
    g_erase_count = 0u;
    g_program_count = 0u;
    std::memset(g_ltft, 0, sizeof(g_ltft));
    std::memset(g_knock, 0, sizeof(g_knock));
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
