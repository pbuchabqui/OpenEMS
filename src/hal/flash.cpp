/**
 * @file hal/stm32h562/flash.cpp
 * @brief Emulação de EEPROM via Flash Bank2 para STM32H562RGT6
 *
 * Layout da Flash Bank2 (base 0x08080000 — H562RG = 1MB, 512K/bank):
 *   Setor 0 (0x08080000, 8 KB): LTFT map + Knock map (página quente)
 *   Setor 1 (0x08082000, 8 KB): Calibração página 0 (512 bytes)
 *   Setor 2 (0x08084000, 8 KB): Calibração página 1 (256 bytes)
 *   Setor 3 (0x08086000, 8 KB): Calibração página 2 (256 bytes)
 *   Setores 4-6: Calibração páginas 3-5
 *   Setor 7: reservado
 *
 * Emulação de SRAM:
 *   LTFT e knock maps usam buffer em SRAM (g_ltft_ram / g_knock_ram)
 *   e escrevemos na Flash periodicamente (quando dirty bit ativo).
 *
 * Procedimento de escrita:
 *   1. Aguardar BSY
 *   2. Desbloquear Flash Bank2 (KEYR2 com sequência)
 *   3. Apagar setor (SER + SNB + STRT)
 *   4. Programar em palavras de 32 bits (PG mode)
 *   5. Re-travar
 */

#include "hal/flash.h"
#include "hal/runtime_seed.h"
#include <cstring>

// ── CRC-32 (ISO 3309 / Ethernet) — shared between production and host-test ───
static uint32_t crc32_update(uint32_t crc, uint8_t data) noexcept {
    crc ^= data;
    for (uint8_t i = 0u; i < 8u; ++i) {
        const uint32_t mask = static_cast<uint32_t>(-(static_cast<int32_t>(crc & 1u)));
        crc = (crc >> 1u) ^ (0xEDB88320u & mask);
    }
    return crc;
}

static uint32_t runtime_seed_crc32(const ems::hal::RuntimeSyncSeed& seed) noexcept {
    uint32_t crc = 0xFFFFFFFFu;
    const uint8_t* p = reinterpret_cast<const uint8_t*>(&seed);
    const uint16_t sz = static_cast<uint16_t>(sizeof(seed) - sizeof(seed.crc32));
    for (uint16_t i = 0u; i < sz; ++i) {
        crc = crc32_update(crc, p[i]);
    }
    return ~crc;
}

#ifndef EMS_HOST_TEST

#include "hal/regs.h"

// ── Buffers SRAM para LTFT e Knock maps ─────────────────────────────────────
// Espelham os dados da Flash; modificados em RAM e flushed periodicamente.
static int8_t  g_ltft_ram[16][16] = {};     // 256 bytes  @ offset 0
static int8_t  g_knock_ram[8][8]  = {};     // 64 bytes   @ offset 256
static int8_t  g_ltft_add_ram[8][8] = {};   // 64 bytes   @ offset 320 (50µs/count)
static bool    g_ltft_dirty     = false;
static bool    g_knock_dirty    = false;
static bool    g_ltft_add_dirty = false;

// ── Endereços dos setores Bank2 ───────────────────────────────────────────────
static constexpr uint32_t kSectorLtft  = 0u;   // Setor 0: LTFT + knock
static constexpr uint32_t kSectorCal0  = 1u;   // Setor 1: Cal page 0
static constexpr uint32_t kSectorCal1  = 2u;   // Setor 2: Cal page 1
static constexpr uint32_t kSectorCal2  = 3u;   // Setor 3: Cal page 2
static constexpr uint32_t kSectorCal6  = 7u;   // Setor 7: Cal page 6 (ETB)

static constexpr uint32_t kBank2Base   = FLASH_BANK2_BASE;
static constexpr uint32_t kSectorSize  = FLASH_SECTOR_SIZE;

// Chaves de desbloqueio da Flash (RM0481 §7.4)
static constexpr uint32_t kFlashKey1 = 0x45670123u;
static constexpr uint32_t kFlashKey2 = 0xCDEF89ABu;
static constexpr uint32_t kFlashErrorMask = FLASH_SR_PGSERR | FLASH_SR_WRPERR;
static constexpr uint32_t kFlashBusyMask = FLASH_SR_BSY | FLASH_SR_WBNE | FLASH_SR_DBNE;
static constexpr uint32_t kFlashWordsPerStep = 16u;
static volatile uint32_t g_flash_wait_timeouts = 0u;

// ── Funções auxiliares ───────────────────────────────────────────────────────

static void flash_unlock_bank2() noexcept {
    if (FLASH_CR2 & FLASH_CR_LOCK) {
        FLASH_KEYR2 = kFlashKey1;
        FLASH_KEYR2 = kFlashKey2;
    }
}

static void flash_lock_bank2() noexcept {
    FLASH_CR2 |= FLASH_CR_LOCK;
}

static void flash_wait_ready() noexcept {
    // Timeout defensivo: evita busy-wait infinito se hardware de flash falhar.
    // ~300k iterações ≈ vários ms de margem para operações de erase/program.
    constexpr uint32_t kFlashWaitTimeout = 300000u;
    for (uint32_t i = 0u; i < kFlashWaitTimeout; ++i) {
        if ((FLASH_SR2 & kFlashBusyMask) == 0u) { return; }
    }
    // Timeout: contabiliza e prossegue — caller verifica FLASH_SR2 por erros.
    ++g_flash_wait_timeouts;
}

static bool flash_erase_sector(uint32_t sector_num) noexcept {
    flash_wait_ready();
    FLASH_CCR2 = 0xFFFFFFFFu;  // limpa todos os flags de erro

    FLASH_CR2 = FLASH_CR_SER
              | ((sector_num & 0xFu) << FLASH_CR_SNB_SHIFT)
              | FLASH_CR_STRT;

    flash_wait_ready();
    FLASH_CR2 &= ~(FLASH_CR_SER | (0xFu << FLASH_CR_SNB_SHIFT));

    return (FLASH_SR2 & kFlashErrorMask) == 0u;
}

static bool flash_write_words(uint32_t dest_addr,
                              const uint8_t* src,
                              uint32_t len_bytes) noexcept {
    // len_bytes deve ser múltiplo de 4
    flash_wait_ready();
    FLASH_CR2 |= FLASH_CR_PG;

    const uint32_t* src32 = reinterpret_cast<const uint32_t*>(src);
    volatile uint32_t* dst32 = reinterpret_cast<volatile uint32_t*>(dest_addr);
    const uint32_t nwords = len_bytes / 4u;

    for (uint32_t i = 0u; i < nwords; ++i) {
        dst32[i] = src32[i];
        flash_wait_ready();
        if (FLASH_SR2 & kFlashErrorMask) {
            FLASH_CR2 &= ~FLASH_CR_PG;
            return false;
        }
    }

    FLASH_CR2 &= ~FLASH_CR_PG;
    return true;
}

namespace ems::hal {

// ── LTFT map ─────────────────────────────────────────────────────────────────

bool nvm_write_ltft(uint8_t rpm_i, uint8_t load_i, int8_t val) noexcept {
    if (rpm_i >= 16u || load_i >= 16u) { return false; }
    if (g_ltft_ram[rpm_i][load_i] != val) {
        g_ltft_ram[rpm_i][load_i] = val;
        g_ltft_dirty = true;
    }
    return true;
}

int8_t nvm_read_ltft(uint8_t rpm_i, uint8_t load_i) noexcept {
    if (rpm_i >= 16u || load_i >= 16u) { return 0; }
    return g_ltft_ram[rpm_i][load_i];
}

bool nvm_load_adaptive_maps() noexcept {
    std::memcpy(g_ltft_ram,
                reinterpret_cast<const void*>(kBank2Base),
                sizeof(g_ltft_ram));
    std::memcpy(g_knock_ram,
                reinterpret_cast<const void*>(kBank2Base + 256u),
                sizeof(g_knock_ram));
    std::memcpy(g_ltft_add_ram,
                reinterpret_cast<const void*>(kBank2Base + 320u),
                sizeof(g_ltft_add_ram));

    bool erased = true;
    const uint8_t* ltft_bytes = reinterpret_cast<const uint8_t*>(g_ltft_ram);
    for (uint16_t i = 0u; i < sizeof(g_ltft_ram); ++i) {
        if (ltft_bytes[i] != 0xFFu) { erased = false; break; }
    }
    if (erased) {
        std::memset(g_ltft_ram, 0, sizeof(g_ltft_ram));
        std::memset(g_knock_ram, 0, sizeof(g_knock_ram));
    }

    bool add_erased = true;
    const uint8_t* add_bytes = reinterpret_cast<const uint8_t*>(g_ltft_add_ram);
    for (uint16_t i = 0u; i < sizeof(g_ltft_add_ram); ++i) {
        if (add_bytes[i] != 0xFFu) { add_erased = false; break; }
    }
    if (add_erased) {
        std::memset(g_ltft_add_ram, 0, sizeof(g_ltft_add_ram));
    }

    g_ltft_dirty     = false;
    g_knock_dirty    = false;
    g_ltft_add_dirty = false;
    return true;
}

bool nvm_write_ltft_add(uint8_t rpm_i, uint8_t load_i, int8_t val_50us) noexcept {
    if (rpm_i >= 8u || load_i >= 8u) { return false; }
    if (g_ltft_add_ram[rpm_i][load_i] != val_50us) {
        g_ltft_add_ram[rpm_i][load_i] = val_50us;
        g_ltft_add_dirty = true;
    }
    return true;
}

int8_t nvm_read_ltft_add(uint8_t rpm_i, uint8_t load_i) noexcept {
    if (rpm_i >= 8u || load_i >= 8u) { return 0; }
    return g_ltft_add_ram[rpm_i][load_i];
}

// ── Knock map ─────────────────────────────────────────────────────────────────

bool nvm_write_knock(uint8_t rpm_i, uint8_t load_i, int8_t retard_deci_deg) noexcept {
    if (rpm_i >= 8u || load_i >= 8u) { return false; }
    if (g_knock_ram[rpm_i][load_i] != retard_deci_deg) {
        g_knock_ram[rpm_i][load_i] = retard_deci_deg;
        g_knock_dirty = true;
    }
    return true;
}

int8_t nvm_read_knock(uint8_t rpm_i, uint8_t load_i) noexcept {
    if (rpm_i >= 8u || load_i >= 8u) { return 0; }
    return g_knock_ram[rpm_i][load_i];
}

void nvm_reset_knock_map() noexcept {
    std::memset(g_knock_ram, 0, sizeof(g_knock_ram));
    g_knock_dirty = true;
}

// ── Calibração (páginas) ──────────────────────────────────────────────────────

bool nvm_save_calibration(uint8_t page, const uint8_t* data, uint16_t len) noexcept {
    if (page > 6u || data == nullptr || len == 0u) { return false; }

    const uint32_t sector = (page == 6u) ? kSectorCal6 : (kSectorCal0 + page);
    const uint32_t dest   = kBank2Base + sector * kSectorSize;

    // Arredondar len para múltiplo de 4
    const uint32_t len32 = (static_cast<uint32_t>(len) + 3u) & ~3u;
    if (len32 > kSectorSize) { return false; }

    const uint32_t whole_len = static_cast<uint32_t>(len) & ~3u;
    const uint32_t tail_len = static_cast<uint32_t>(len) - whole_len;

    flash_unlock_bank2();
    bool ok = flash_erase_sector(sector);
    if (ok && whole_len != 0u) {
        ok = flash_write_words(dest, data, whole_len);
    }
    if (ok && tail_len != 0u) {
        uint8_t tail[4] = {0xFFu, 0xFFu, 0xFFu, 0xFFu};
        std::memcpy(tail, data + whole_len, tail_len);
        ok = flash_write_words(dest + whole_len, tail, sizeof(tail));
    }
    flash_lock_bank2();
    return ok;
}

bool nvm_load_calibration(uint8_t page, uint8_t* data, uint16_t len) noexcept {
    if (page > 6u || data == nullptr || len == 0u) { return false; }

    const uint32_t sector = (page == 6u) ? kSectorCal6 : (kSectorCal0 + page);
    const uint32_t src    = kBank2Base + sector * kSectorSize;

    // Leitura direta da Flash (mapeada em memória)
    std::memcpy(data, reinterpret_cast<const void*>(src), len);
    return true;
}

// ── Flush LTFT + Knock para Flash ─────────────────────────────────────────────
// Chamado a cada 500 ms do loop principal (flash flush do STM32)

bool nvm_flush_adaptive_maps() noexcept {
    enum class FlushState : uint8_t {
        Idle,
        WaitErase,
        Program,
        WaitFinal,
    };
    static uint8_t sector_buf[FLASH_SECTOR_SIZE] = {};
    static FlushState state = FlushState::Idle;
    static uint32_t word_i = 0u;
    static uint32_t bsy_stall_count = 0u;  // FIX C8: detect permanently stuck BSY

    const auto fail = []() noexcept {
        FLASH_CR2 &= ~(FLASH_CR_PG | FLASH_CR_SER | (0xFu << FLASH_CR_SNB_SHIFT));
        flash_lock_bank2();
        g_ltft_dirty     = true;
        g_knock_dirty    = true;
        g_ltft_add_dirty = true;
        return false;
    };

    if (state == FlushState::Idle) {
        if (!g_ltft_dirty && !g_knock_dirty && !g_ltft_add_dirty) { return true; }

        // Setor 0 contém LTFT, Knock, LTFT-add e RuntimeSyncSeed. Preserva o restante.
        std::memcpy(sector_buf,
                    reinterpret_cast<const void*>(kBank2Base),
                    sizeof(sector_buf));

        std::memcpy(sector_buf + 0,   g_ltft_ram,     256u);
        std::memcpy(sector_buf + 256, g_knock_ram,     64u);
        std::memcpy(sector_buf + 320, g_ltft_add_ram,  64u);
        g_ltft_dirty     = false;
        g_knock_dirty    = false;
        g_ltft_add_dirty = false;

        flash_unlock_bank2();
        FLASH_CCR2 = 0xFFFFFFFFu;
        FLASH_CR2 = FLASH_CR_SER
                  | ((kSectorLtft & 0xFu) << FLASH_CR_SNB_SHIFT)
                  | FLASH_CR_STRT;
        state = FlushState::WaitErase;
        return false;
    }

    if (FLASH_SR2 & kFlashBusyMask) { return false; }
    if (FLASH_SR2 & kFlashErrorMask) {
        state = FlushState::Idle;
        return fail();
    }

    if (state == FlushState::WaitErase) {
        FLASH_CR2 &= ~(FLASH_CR_SER | (0xFu << FLASH_CR_SNB_SHIFT));
        FLASH_CCR2 = 0xFFFFFFFFu;
        FLASH_CR2 |= FLASH_CR_PG;
        word_i = 0u;
        state = FlushState::Program;
    }

    if (state == FlushState::Program) {
        volatile uint32_t* dst32 = reinterpret_cast<volatile uint32_t*>(kBank2Base);
        const uint32_t* src32 = reinterpret_cast<const uint32_t*>(sector_buf);
        const uint32_t nwords = sizeof(sector_buf) / sizeof(uint32_t);
        uint32_t budget = kFlashWordsPerStep;
        while ((word_i < nwords) && (budget-- != 0u)) {
            if (FLASH_SR2 & kFlashBusyMask) {
                // FIX C8: if BSY never clears, the state machine is permanently stuck
                // (word_i doesn't advance, fail() is never called, data is lost forever).
                // Abort and re-mark dirty after a threshold so the next flush cycle
                // can attempt recovery.
                constexpr uint32_t kBsyStallLimit = 50000u;
                if (++bsy_stall_count >= kBsyStallLimit) {
                    bsy_stall_count = 0u;
                    state = FlushState::Idle;
                    return fail();
                }
                return false;
            }
            bsy_stall_count = 0u;  // Reset on any successful progress
            if (FLASH_SR2 & kFlashErrorMask) {
                state = FlushState::Idle;
                return fail();
            }
            dst32[word_i] = src32[word_i];
            ++word_i;
        }
        if (word_i < nwords) { return false; }
        state = FlushState::WaitFinal;
    }

    if (state == FlushState::WaitFinal) {
        if (FLASH_SR2 & kFlashBusyMask) { return false; }
        if (FLASH_SR2 & kFlashErrorMask) {
            state = FlushState::Idle;
            return fail();
        }
        FLASH_CR2 &= ~FLASH_CR_PG;
        flash_lock_bank2();
        state = FlushState::Idle;
        return !g_ltft_dirty && !g_knock_dirty && !g_ltft_add_dirty;
    }

    return false;
}

// ── RuntimeSyncSeed (boot rápido) ────────────────────────────────────────────
// Armazena seed na região final do Setor 0 (bytes 512-543 = 32 bytes)

static constexpr uint32_t kSeedOffset = 512u;

bool nvm_save_runtime_seed(const RuntimeSyncSeed* seed) noexcept {
    if (seed == nullptr) { return false; }

    static uint8_t sector_buf[FLASH_SECTOR_SIZE] = {};
    std::memcpy(sector_buf,
                reinterpret_cast<const void*>(kBank2Base),
                sizeof(sector_buf));
    std::memcpy(sector_buf + kSeedOffset, seed, sizeof(RuntimeSyncSeed));

    flash_unlock_bank2();
    const bool ok = flash_erase_sector(kSectorLtft) &&
                    flash_write_words(kBank2Base, sector_buf, sizeof(sector_buf));
    flash_lock_bank2();
    return ok;
}

bool nvm_load_runtime_seed(RuntimeSyncSeed* seed_out) noexcept {
    if (seed_out == nullptr) { return false; }
    const uint32_t addr = kBank2Base + kSeedOffset;
    std::memcpy(seed_out, reinterpret_cast<const void*>(addr),
                sizeof(RuntimeSyncSeed));
    if (seed_out->crc32 != runtime_seed_crc32(*seed_out)) { return false; }
    return runtime_seed_boot_compatible_60_2(*seed_out);
}

bool nvm_clear_runtime_seed() noexcept {
    RuntimeSyncSeed blank{};
    return nvm_save_runtime_seed(&blank);
}

} // namespace ems::hal

#else  // EMS_HOST_TEST ─────────────────────────────────────────────────────

namespace ems::hal {

static constexpr uint8_t kTestSeedSlots = 8u;

static int8_t g_ltft[16][16]     = {};
static int8_t g_knock[8][8]      = {};
static int8_t g_ltft_add[8][8]   = {};
static uint8_t g_cal[6][512]     = {};
static uint32_t g_erase_cnt   = 0u, g_prog_cnt = 0u;
static bool     g_flash_busy      = false;  // simulates flash BSY timeout when set
static uint32_t g_flash_busy_polls = 0u;     // non-zero → simulate timeout on next op

// Runtime seed: slot array (mirrors the STM32 flash-backed slot layout)
static RuntimeSyncSeed g_seed_slots[kTestSeedSlots] = {};
static bool g_seed_slot_valid[kTestSeedSlots] = {};

bool nvm_write_ltft(uint8_t r, uint8_t l, int8_t v) noexcept {
    if (r >= 16u || l >= 16u) { return false; }
    if (g_flash_busy) { return false; }
    g_ltft[r][l] = v; return true;
}
int8_t nvm_read_ltft(uint8_t r, uint8_t l) noexcept {
    if (r >= 16u || l >= 16u) { return 0; }
    return g_ltft[r][l];
}
bool nvm_load_adaptive_maps() noexcept { return true; }
bool nvm_write_knock(uint8_t r, uint8_t l, int8_t v) noexcept {
    if (r >= 8u || l >= 8u) { return false; }
    if (g_flash_busy) { return false; }
    g_knock[r][l] = v; return true;
}
int8_t nvm_read_knock(uint8_t r, uint8_t l) noexcept {
    if (r >= 8u || l >= 8u) { return 0; }
    return g_knock[r][l];
}
void nvm_reset_knock_map() noexcept { std::memset(g_knock, 0, sizeof(g_knock)); }
bool nvm_write_ltft_add(uint8_t r, uint8_t l, int8_t v) noexcept {
    if (r >= 8u || l >= 8u) { return false; }
    g_ltft_add[r][l] = v; return true;
}
int8_t nvm_read_ltft_add(uint8_t r, uint8_t l) noexcept {
    if (r >= 8u || l >= 8u) { return 0; }
    return g_ltft_add[r][l];
}

bool nvm_save_calibration(uint8_t pg, const uint8_t* d, uint16_t l) noexcept {
    if (pg > 5u || d == nullptr || l == 0u) return false;
    if (g_flash_busy) { return false; }
    ++g_erase_cnt; ++g_prog_cnt;
    std::memcpy(g_cal[pg], d, l); return true;
}
bool nvm_load_calibration(uint8_t pg, uint8_t* d, uint16_t l) noexcept {
    if (pg > 5u || d == nullptr || l == 0u) return false;
    std::memcpy(d, g_cal[pg], l); return true;
}
bool nvm_flush_adaptive_maps() noexcept { return true; }

bool nvm_save_runtime_seed(const RuntimeSyncSeed* s) noexcept {
    if (!s) { return false; }
    // Find slot with highest sequence number to determine next write slot
    uint32_t max_seq = 0u;
    uint8_t write_slot = 0u;
    bool found_any = false;
    for (uint8_t i = 0u; i < kTestSeedSlots; ++i) {
        if (!g_seed_slot_valid[i]) {
            if (!found_any) { write_slot = i; }
            break;
        }
        if (g_seed_slots[i].sequence >= max_seq) {
            max_seq = g_seed_slots[i].sequence;
            write_slot = static_cast<uint8_t>((i + 1u) % kTestSeedSlots);
            found_any = true;
        }
    }
    RuntimeSyncSeed w = *s;
    w.magic   = RUNTIME_SYNC_SEED_MAGIC;
    w.version = RUNTIME_SYNC_SEED_VERSION;
    w.sequence = found_any ? max_seq + 1u : 0u;
    w.crc32 = runtime_seed_crc32(w);
    g_seed_slots[write_slot] = w;
    g_seed_slot_valid[write_slot] = true;
    return true;
}

bool nvm_load_runtime_seed(RuntimeSyncSeed* s) noexcept {
    if (!s) { return false; }
    // Find valid slot with highest sequence (wrap-aware)
    bool found = false;
    uint32_t best_seq = 0u;
    const RuntimeSyncSeed* best = nullptr;
    for (uint8_t i = 0u; i < kTestSeedSlots; ++i) {
        if (!g_seed_slot_valid[i]) { continue; }
        const RuntimeSyncSeed& sl = g_seed_slots[i];
        if (sl.crc32 != runtime_seed_crc32(sl)) { continue; }
        if (!runtime_seed_boot_compatible_60_2(sl)) { continue; }
        if (!found || static_cast<int32_t>(sl.sequence - best_seq) > 0) {
            best_seq = sl.sequence;
            best = &sl;
            found = true;
        }
    }
    if (!found || best == nullptr) { return false; }
    *s = *best;
    return true;
}

bool nvm_clear_runtime_seed() noexcept {
    std::memset(g_seed_slots, 0, sizeof(g_seed_slots));
    std::memset(g_seed_slot_valid, 0, sizeof(g_seed_slot_valid));
    return true;
}

void nvm_test_reset() noexcept {
    std::memset(g_ltft, 0, sizeof(g_ltft));
    std::memset(g_knock, 0, sizeof(g_knock));
    std::memset(g_cal, 0, sizeof(g_cal));
    g_erase_cnt = g_prog_cnt = 0u;
    g_flash_busy = false;
    g_flash_busy_polls = 0u;
    std::memset(g_seed_slots, 0, sizeof(g_seed_slots));
    std::memset(g_seed_slot_valid, 0, sizeof(g_seed_slot_valid));
}
void flash_test_set_busy_polls(uint32_t polls) noexcept {
    g_flash_busy_polls = polls;
    g_flash_busy = (polls > 0u);
}
uint32_t nvm_test_erase_count() noexcept { return g_erase_cnt; }
uint32_t nvm_test_program_count() noexcept { return g_prog_cnt; }

bool nvm_test_runtime_seed_inject_slot(uint8_t slot,
                                       const RuntimeSyncSeed* seed,
                                       bool recompute_crc) noexcept {
    if (seed == nullptr || slot >= kTestSeedSlots) { return false; }
    RuntimeSyncSeed w = *seed;
    if (recompute_crc) { w.crc32 = runtime_seed_crc32(w); }
    g_seed_slots[slot] = w;
    g_seed_slot_valid[slot] = true;
    return true;
}

uint8_t nvm_test_runtime_seed_slot_count() noexcept {
    return kTestSeedSlots;
}

} // namespace ems::hal

#endif  // EMS_HOST_TEST
