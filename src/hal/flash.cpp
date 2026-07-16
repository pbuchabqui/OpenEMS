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
#include "hal/crc32.h"
#include "hal/runtime_seed.h"
#include "hal/critical_section.h"
#include <cstring>

static uint32_t runtime_seed_crc32(const ems::hal::RuntimeSyncSeed& seed) noexcept {
    const uint8_t* p = reinterpret_cast<const uint8_t*>(&seed);
    const uint16_t sz = static_cast<uint16_t>(sizeof(seed) - sizeof(seed.crc32));
    return ems::hal::crc32_calc(p, sz);
}

namespace ems::hal {

// Pura e compilada em ambos os alvos (target + host-test): o gate de layout
// do setor adaptativo é testável alimentando imagens sintéticas.
uint32_t nvm_adaptive_maps_crc(const uint8_t* sector) noexcept {
    if (sector == nullptr) { return 0u; }
    return crc32_calc(sector, kNvmOffLayoutMagic);
}

bool nvm_adaptive_sector_valid(const uint8_t* sector) noexcept {
    if (sector == nullptr) { return false; }
    uint32_t magic = 0u;
    // memcpy: o offset é 16-alinhado, mas não assumir alinhamento do buffer.
    __builtin_memcpy(&magic, sector + kNvmOffLayoutMagic, sizeof(magic));
    if (magic != kNvmLayoutMagic) { return false; }
    uint32_t stored_crc = 0u;
    __builtin_memcpy(&stored_crc, sector + kNvmOffMapsCrc, sizeof(stored_crc));
    return stored_crc == nvm_adaptive_maps_crc(sector);
}

// Stamp magic + maps CRC into a sector image (payload must already be filled).
inline void nvm_stamp_adaptive_header(uint8_t* sector) noexcept {
    if (sector == nullptr) { return; }
    uint32_t magic = kNvmLayoutMagic;
    __builtin_memcpy(sector + kNvmOffLayoutMagic, &magic, sizeof(magic));
    const uint32_t crc = nvm_adaptive_maps_crc(sector);
    __builtin_memcpy(sector + kNvmOffMapsCrc, &crc, sizeof(crc));
}

}  // namespace ems::hal

#ifndef EMS_HOST_TEST

#include "hal/regs.h"

// ── Buffers SRAM para LTFT e Knock maps ─────────────────────────────────────
// Espelham os dados da Flash; modificados em RAM e flushed periodicamente.
// Layout do Setor 0: ver kNvmOff* em flash.h (offsets derivados das dimensões).
static int8_t  g_ltft_ram[ems::hal::kNvmLtftDim][ems::hal::kNvmLtftDim] = {};
static int8_t  g_knock_ram[8][8]  = {};     // 8×8 fixo (por-cilindro, não segue o grid)
static int8_t  g_ltft_add_ram[ems::hal::kNvmLtftAddDim][ems::hal::kNvmLtftAddDim] = {};  // 50µs/count
static bool    g_ltft_dirty     = false;
static bool    g_knock_dirty    = false;
static bool    g_ltft_add_dirty = false;
static uint32_t g_nvm_now_ms              = 0u;
static uint32_t g_last_adaptive_flush_ms  = 0u;
static bool     g_adaptive_flush_asap     = false;
// True while nvm_flush_adaptive_maps SM holds sector 0 (erase/program in flight).
static bool     g_sector0_flush_active    = false;
// Seed lives in the same sector as adaptive maps — never erase independently.
// g_seed_ram is always the source of truth for the next flush; g_seed_dirty
// forces a sector rewrite even when LTFT/knock/add are clean.
static ems::hal::RuntimeSyncSeed g_seed_ram{};
static bool g_seed_dirty = false;

// ── Endereços dos setores Bank2 ───────────────────────────────────────────────
static constexpr uint32_t kSectorLtft  = 0u;   // Setor 0: LTFT + knock
static constexpr uint32_t kSectorCal0  = 1u;   // Setor 1: Cal page 0
static constexpr uint32_t kSectorCal1  = 2u;   // Setor 2: Cal page 1
static constexpr uint32_t kSectorCal2  = 3u;   // Setor 3: Cal page 2
static constexpr uint32_t kSectorCal6  = 7u;   // Setor 7: Cal page 6 (ETB)
static constexpr uint32_t kSectorCal7  = 8u;   // Setor 8: Cal page 7 (Pedal map)
static constexpr uint32_t kSectorCal8  = 9u;   // Setor 9: Cal page 8 (Boost map)
static constexpr uint32_t kSectorCal9  = 10u;  // Setor 10: Cal page 9 (eixos tabela)

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
    if (FLASH_NSCR & FLASH_CR_LOCK) {
        FLASH_NSKEYR = kFlashKey1;
        FLASH_NSKEYR = kFlashKey2;
    }
}

static void flash_lock_bank2() noexcept {
    FLASH_NSCR |= FLASH_CR_LOCK;
}

static void flash_wait_ready() noexcept {
    // Timeout defensivo: evita busy-wait infinito se hardware de flash falhar.
    // ~300k iterações ≈ vários ms de margem para operações de erase/program.
    constexpr uint32_t kFlashWaitTimeout = 300000u;
    for (uint32_t i = 0u; i < kFlashWaitTimeout; ++i) {
        if ((FLASH_NSSR & kFlashBusyMask) == 0u) { return; }
    }
    // Timeout: contabiliza e prossegue — caller verifica FLASH_NSSR por erros.
    ++g_flash_wait_timeouts;
}

static bool flash_erase_sector(uint32_t sector_num) noexcept {
    flash_wait_ready();
    FLASH_NSCCR = 0xFFFFFFFFu;  // limpa todos os flags de erro

    // BKSEL=1: toda a calibração vive no Bank2 (0x08080000+); sector_num é
    // relativo ao banco (0-63), igual em ambos os bancos — só o bit BKSEL
    // desambigua qual metade física da flash o número de sector referencia.
    FLASH_NSCR = FLASH_CR_SER
              | FLASH_CR_BKSEL
              | ((sector_num << FLASH_CR_SNB_SHIFT) & FLASH_CR_SNB_MASK)
              | FLASH_CR_STRT;

    flash_wait_ready();
    FLASH_NSCR &= ~(FLASH_CR_SER | FLASH_CR_BKSEL | FLASH_CR_SNB_MASK);

    return (FLASH_NSSR & kFlashErrorMask) == 0u;
}

// STM32H5 (RM0481): a Flash programa-se em "flash words" de 128 bits
// (4×32-bit). O write buffer interno (SR.WBNE/DBNE) só comita as 4 palavras
// para a célula não-volátil quando chegam em sucessão imediata; fazer
// flash_wait_ready() (poll de BSY/WBNE/DBNE) ENTRE palavras do MESMO
// quad-word deixa o buffer preso — BSY nunca chega a subir porque a escrita
// nunca se completa, o timeout defensivo de flash_wait_ready() expira e
// "segue em frente" sem erro reportado (nenhum bit de kFlashErrorMask é
// setado), então flash_write_words devolve true mas os dados nunca saem do
// buffer volátil: sobrevive a leituras da mesma sessão (RAM/SRAM cache) mas
// perde-se num power-cycle. Corresponde exactamente a FLASH_Program_QuadWord
// do HAL oficial da ST (stm32h5xx_hal_flash.c), que também desabilita IRQs
// durante o loop das 4 palavras — uma ISR longa a meio do quad-word pode
// violar o timing exigido pelo write buffer.
// len_bytes DEVE ser múltiplo de 16 (4 words); ver nvm_save_calibration.
static bool flash_write_words(uint32_t dest_addr,
                              const uint8_t* src,
                              uint32_t len_bytes) noexcept {
    flash_wait_ready();
    FLASH_NSCR |= FLASH_CR_PG;

    const uint32_t* src32 = reinterpret_cast<const uint32_t*>(src);
    volatile uint32_t* dst32 = reinterpret_cast<volatile uint32_t*>(dest_addr);
    const uint32_t nwords = len_bytes / 4u;

    for (uint32_t qw = 0u; qw < nwords; qw += 4u) {
        const uint32_t n = (nwords - qw < 4u) ? (nwords - qw) : 4u;
        {
            // As (até) 4 palavras do quad-word em sucessão imediata, sem
            // poll de status nem IRQs no meio — só assim o hardware comita
            // o quad-word (ver comentário acima).
            ems::hal::CriticalSectionGuard guard;
            for (uint32_t i = 0u; i < n; ++i) {
                dst32[qw + i] = src32[qw + i];
            }
        }
        flash_wait_ready();
        if (FLASH_NSSR & kFlashErrorMask) {
            FLASH_NSCR &= ~FLASH_CR_PG;
            return false;
        }
    }

    FLASH_NSCR &= ~FLASH_CR_PG;
    return true;
}

namespace ems::hal {

// ── LTFT map ─────────────────────────────────────────────────────────────────

bool nvm_write_ltft(uint8_t rpm_i, uint8_t load_i, int8_t val) noexcept {
    if (rpm_i >= kNvmLtftDim || load_i >= kNvmLtftDim) { return false; }
    if (g_ltft_ram[rpm_i][load_i] != val) {
        g_ltft_ram[rpm_i][load_i] = val;
        g_ltft_dirty = true;
    }
    return true;
}

int8_t nvm_read_ltft(uint8_t rpm_i, uint8_t load_i) noexcept {
    if (rpm_i >= kNvmLtftDim || load_i >= kNvmLtftDim) { return 0; }
    return g_ltft_ram[rpm_i][load_i];
}

bool nvm_load_adaptive_maps() noexcept {
    // Magic+CRC gate: setor apagado, layout antigo (LTF2), ou bit-rot nos
    // mapas → zera e marca dirty; o flush regrava LTF3 + CRC + seed.
    const uint8_t* sector = reinterpret_cast<const uint8_t*>(kBank2Base);
    if (!nvm_adaptive_sector_valid(sector)) {
        std::memset(g_ltft_ram, 0, sizeof(g_ltft_ram));
        std::memset(g_knock_ram, 0, sizeof(g_knock_ram));
        std::memset(g_ltft_add_ram, 0, sizeof(g_ltft_add_ram));
        std::memset(&g_seed_ram, 0, sizeof(g_seed_ram));
        g_ltft_dirty     = true;
        g_knock_dirty    = true;
        g_ltft_add_dirty = true;
        g_seed_dirty     = false;  // blank seed need not force rewrite alone
        return true;
    }

    std::memcpy(g_ltft_ram,
                reinterpret_cast<const void*>(kBank2Base + kNvmOffLtft),
                sizeof(g_ltft_ram));
    std::memcpy(g_knock_ram,
                reinterpret_cast<const void*>(kBank2Base + kNvmOffKnock),
                sizeof(g_knock_ram));
    std::memcpy(g_ltft_add_ram,
                reinterpret_cast<const void*>(kBank2Base + kNvmOffLtftAdd),
                sizeof(g_ltft_add_ram));
    std::memcpy(&g_seed_ram,
                reinterpret_cast<const void*>(kBank2Base + kNvmSeedOffset),
                sizeof(g_seed_ram));

    g_ltft_dirty     = false;
    g_knock_dirty    = false;
    g_ltft_add_dirty = false;
    g_seed_dirty     = false;
    return true;
}

bool nvm_write_ltft_add(uint8_t rpm_i, uint8_t load_i, int8_t val_50us) noexcept {
    if (rpm_i >= kNvmLtftAddDim || load_i >= kNvmLtftAddDim) { return false; }
    if (g_ltft_add_ram[rpm_i][load_i] != val_50us) {
        g_ltft_add_ram[rpm_i][load_i] = val_50us;
        g_ltft_add_dirty = true;
    }
    return true;
}

int8_t nvm_read_ltft_add(uint8_t rpm_i, uint8_t load_i) noexcept {
    if (rpm_i >= kNvmLtftAddDim || load_i >= kNvmLtftAddDim) { return 0; }
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
    if (page > 9u || data == nullptr || len == 0u) { return false; }

    const uint32_t sector = (page == 9u) ? kSectorCal9 :
                            (page == 8u) ? kSectorCal8 :
                            (page == 7u) ? kSectorCal7 :
                            (page == 6u) ? kSectorCal6 : (kSectorCal0 + page);
    const uint32_t dest   = kBank2Base + sector * kSectorSize;

    // flash_write_words exige múltiplo de 16 (quad-word de 128 bits, ver
    // comentário no seu topo) — arredondar para cima ao verificar limites.
    const uint32_t len16 = (static_cast<uint32_t>(len) + 15u) & ~15u;
    if (len16 > kSectorSize) { return false; }

    const uint32_t whole_len = static_cast<uint32_t>(len) & ~15u;
    const uint32_t tail_len = static_cast<uint32_t>(len) - whole_len;  // 0..15

    flash_unlock_bank2();
    bool ok = flash_erase_sector(sector);
    if (ok && whole_len != 0u) {
        ok = flash_write_words(dest, data, whole_len);
    }
    if (ok && tail_len != 0u) {
        uint8_t tail[16];
        std::memset(tail, 0xFFu, sizeof(tail));
        std::memcpy(tail, data + whole_len, tail_len);
        ok = flash_write_words(dest + whole_len, tail, sizeof(tail));
    }
    flash_lock_bank2();
    // DIAG (2026-07-09): burn reportava OK mas não sobrevivia a power-cycle.
    // Suspeita: write buffer de 128 bits (WBNE/DBNE) do STM32H5 não é
    // forçado a comitar para NV storage nas mesmas condições em que
    // flash_wait_ready() considera "pronto". Readback imediato expõe
    // isto sem esperar por reboot — se falhar aqui, confirma a hipótese.
    if (ok) {
        ok = std::memcmp(reinterpret_cast<const void*>(dest), data, len) == 0;
    }
    return ok;
}

bool nvm_load_calibration(uint8_t page, uint8_t* data, uint16_t len) noexcept {
    if (page > 9u || data == nullptr || len == 0u) { return false; }

    const uint32_t sector = (page == 9u) ? kSectorCal9 :
                            (page == 8u) ? kSectorCal8 :
                            (page == 7u) ? kSectorCal7 :
                            (page == 6u) ? kSectorCal6 : (kSectorCal0 + page);
    const uint32_t src    = kBank2Base + sector * kSectorSize;

    // Leitura direta da Flash (mapeada em memória)
    std::memcpy(data, reinterpret_cast<const void*>(src), len);
    return true;
}

// ── Flush LTFT + Knock para Flash ─────────────────────────────────────────────
// Poll do main (ex. 500 ms). Rate-limit: no máximo 1 erase/program completo
// por kMinAdaptiveFlushIntervalMs, salvo nvm_request_adaptive_flush_now().

void nvm_set_now_ms(uint32_t now_ms) noexcept {
    g_nvm_now_ms = now_ms;
}

void nvm_request_adaptive_flush_now() noexcept {
    g_adaptive_flush_asap = true;
}

bool nvm_adaptive_maps_dirty() noexcept {
    return g_ltft_dirty || g_knock_dirty || g_ltft_add_dirty || g_seed_dirty;
}

// Pack RAM maps + seed + LTF3 header into sector_buf (full FLASH_SECTOR_SIZE).
static void pack_adaptive_sector(uint8_t* sector_buf) noexcept {
    std::memcpy(sector_buf,
                reinterpret_cast<const void*>(kBank2Base),
                FLASH_SECTOR_SIZE);
    std::memcpy(sector_buf + kNvmOffLtft,    g_ltft_ram,     sizeof(g_ltft_ram));
    std::memcpy(sector_buf + kNvmOffKnock,   g_knock_ram,    sizeof(g_knock_ram));
    std::memcpy(sector_buf + kNvmOffLtftAdd, g_ltft_add_ram, sizeof(g_ltft_add_ram));
    nvm_stamp_adaptive_header(sector_buf);
    std::memcpy(sector_buf + kNvmSeedOffset, &g_seed_ram, sizeof(g_seed_ram));
}

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
    static bool held_seed_dirty = false;

    const auto fail = []() noexcept {
        FLASH_NSCR &= ~(FLASH_CR_PG | FLASH_CR_SER | FLASH_CR_BKSEL | FLASH_CR_SNB_MASK);
        flash_lock_bank2();
        g_ltft_dirty     = true;
        g_knock_dirty    = true;
        g_ltft_add_dirty = true;
        g_seed_dirty     = true;  // re-attempt seed with maps
        g_sector0_flush_active = false;
        return false;
    };

    if (state == FlushState::Idle) {
        if (!g_ltft_dirty && !g_knock_dirty && !g_ltft_add_dirty && !g_seed_dirty) {
            return true;
        }

        // Rate-limit: adia flush se ainda dentro do intervalo (mantém dirty).
        // Seed-only writes (engine stop) always bypass rate-limit via asap or
        // when only g_seed_dirty — still respect asap flag from request_now.
        if (!g_adaptive_flush_asap && g_last_adaptive_flush_ms != 0u) {
            const uint32_t age = g_nvm_now_ms - g_last_adaptive_flush_ms;
            // Seed-only: allow without waiting 60 s (stop-sync must persist).
            const bool seed_only = g_seed_dirty &&
                !g_ltft_dirty && !g_knock_dirty && !g_ltft_add_dirty;
            if (!seed_only && age < kMinAdaptiveFlushIntervalMs) {
                return true;  // defer — main re-agenda no próximo tick
            }
        }
        g_adaptive_flush_asap = false;

        pack_adaptive_sector(sector_buf);
        held_seed_dirty  = g_seed_dirty;
        g_ltft_dirty     = false;
        g_knock_dirty    = false;
        g_ltft_add_dirty = false;
        g_seed_dirty     = false;

        flash_unlock_bank2();
        FLASH_NSCCR = 0xFFFFFFFFu;
        FLASH_NSCR = FLASH_CR_SER
                  | FLASH_CR_BKSEL
                  | ((kSectorLtft << FLASH_CR_SNB_SHIFT) & FLASH_CR_SNB_MASK)
                  | FLASH_CR_STRT;
        g_sector0_flush_active = true;
        state = FlushState::WaitErase;
        return false;
    }

    if (FLASH_NSSR & kFlashBusyMask) { return false; }
    if (FLASH_NSSR & kFlashErrorMask) {
        state = FlushState::Idle;
        return fail();
    }

    if (state == FlushState::WaitErase) {
        FLASH_NSCR &= ~(FLASH_CR_SER | FLASH_CR_BKSEL | FLASH_CR_SNB_MASK);
        FLASH_NSCCR = 0xFFFFFFFFu;
        FLASH_NSCR |= FLASH_CR_PG;
        word_i = 0u;
        state = FlushState::Program;
    }

    if (state == FlushState::Program) {
        // DIAG (2026-07-09): escrever 1 palavra/iteração e verificar BSY antes
        // da SEGUINTE fazia o loop ler SR.WBNE=1 (buffer com 1-3 palavras do
        // quad-word ainda por completar) como "ocupado" — exactamente o
        // sintoma que o "FIX C8" abaixo mascarava (BSY nunca liberta,
        // word_i nunca avança) sem resolver a causa: as 4 palavras de um
        // flash-word de 128 bits têm de ser escritas em sucessão imediata,
        // sem poll de status entre elas (ver flash_write_words). kFlashWordsPerStep
        // é múltiplo de 4 — orçamento agora em quad-words, não em palavras soltas.
        volatile uint32_t* dst32 = reinterpret_cast<volatile uint32_t*>(kBank2Base);
        const uint32_t* src32 = reinterpret_cast<const uint32_t*>(sector_buf);
        const uint32_t nwords = sizeof(sector_buf) / sizeof(uint32_t);
        uint32_t budget_qw = kFlashWordsPerStep / 4u;
        while ((word_i < nwords) && (budget_qw-- != 0u)) {
            if (FLASH_NSSR & kFlashBusyMask) {
                // FIX C8: se BSY nunca limpar entre quad-words (ex.: hardware
                // preso), aborta e remarca dirty para o próximo ciclo tentar.
                constexpr uint32_t kBsyStallLimit = 50000u;
                if (++bsy_stall_count >= kBsyStallLimit) {
                    bsy_stall_count = 0u;
                    state = FlushState::Idle;
                    return fail();
                }
                return false;
            }
            bsy_stall_count = 0u;  // Reset on any successful progress
            if (FLASH_NSSR & kFlashErrorMask) {
                state = FlushState::Idle;
                return fail();
            }
            const uint32_t n = (nwords - word_i < 4u) ? (nwords - word_i) : 4u;
            {
                // As (até) 4 palavras do quad-word em sucessão imediata, sem
                // IRQs no meio (ver flash_write_words).
                ems::hal::CriticalSectionGuard guard;
                for (uint32_t i = 0u; i < n; ++i) {
                    dst32[word_i + i] = src32[word_i + i];
                }
            }
            word_i += n;
        }
        if (word_i < nwords) { return false; }
        state = FlushState::WaitFinal;
    }

    if (state == FlushState::WaitFinal) {
        if (FLASH_NSSR & kFlashBusyMask) { return false; }
        if (FLASH_NSSR & kFlashErrorMask) {
            state = FlushState::Idle;
            return fail();
        }
        FLASH_NSCR &= ~FLASH_CR_PG;
        flash_lock_bank2();
        state = FlushState::Idle;
        g_sector0_flush_active = false;
        g_last_adaptive_flush_ms = g_nvm_now_ms;
        (void)held_seed_dirty;
        return !g_ltft_dirty && !g_knock_dirty && !g_ltft_add_dirty && !g_seed_dirty;
    }

    return false;
}

// ── RuntimeSyncSeed (boot rápido) ────────────────────────────────────────────
// Sempre via shadow RAM + flush SM do setor 0 (nunca erase independente).
// Layout antigo / LTF2 / CRC maps inválido → seed rejeitado no load.

bool nvm_save_runtime_seed(const RuntimeSyncSeed* seed) noexcept {
    if (seed == nullptr) { return false; }

    // Finalize header fields (main only fills flags/tooth/decoder_tag).
    RuntimeSyncSeed w = *seed;
    w.magic   = RUNTIME_SYNC_SEED_MAGIC;
    w.version = RUNTIME_SYNC_SEED_VERSION;
    // Bump sequence so load prefers the newest seed if multi-slot ever returns.
    w.sequence = g_seed_ram.sequence + 1u;
    w.crc32 = 0u;
    w.crc32 = runtime_seed_crc32(w);
    g_seed_ram = w;
    g_seed_dirty = true;
    g_adaptive_flush_asap = true;  // stop-sync must not wait 60 s rate-limit

    // If flush SM already owns the sector, RAM shadow is enough — SM will
    // re-read dirty flags after completion or on next Idle entry.
    if (g_sector0_flush_active) { return true; }

    // Idle: pack + blocking erase/program so seed survives power-cycle even
    // if main loop does not re-enter flush before shutdown.
    static uint8_t sector_buf[FLASH_SECTOR_SIZE] = {};
    pack_adaptive_sector(sector_buf);
    flash_unlock_bank2();
    const bool ok = flash_erase_sector(kSectorLtft) &&
                    flash_write_words(kBank2Base, sector_buf, sizeof(sector_buf));
    flash_lock_bank2();
    if (ok) {
        g_seed_dirty = false;
        // Maps were co-written; clear dirty if they were only dirty because of
        // a concurrent edit — actually maps may still be dirty in RAM if we
        // packed current RAM. Clear all dirty on success (sector matches RAM).
        g_ltft_dirty = false;
        g_knock_dirty = false;
        g_ltft_add_dirty = false;
        g_last_adaptive_flush_ms = g_nvm_now_ms;
    }
    return ok;
}

bool nvm_load_runtime_seed(RuntimeSyncSeed* seed_out) noexcept {
    if (seed_out == nullptr) { return false; }
    // Prefer RAM shadow if already loaded / recently saved.
    if (g_seed_ram.crc32 == runtime_seed_crc32(g_seed_ram) &&
        runtime_seed_boot_compatible_60_2(g_seed_ram)) {
        *seed_out = g_seed_ram;
        return true;
    }
    const uint32_t addr = kBank2Base + kNvmSeedOffset;
    std::memcpy(seed_out, reinterpret_cast<const void*>(addr),
                sizeof(RuntimeSyncSeed));
    if (seed_out->crc32 != runtime_seed_crc32(*seed_out)) { return false; }
    if (!runtime_seed_boot_compatible_60_2(*seed_out)) { return false; }
    g_seed_ram = *seed_out;
    return true;
}

bool nvm_clear_runtime_seed() noexcept {
    RuntimeSyncSeed blank{};
    return nvm_save_runtime_seed(&blank);
}

} // namespace ems::hal

#else  // EMS_HOST_TEST ─────────────────────────────────────────────────────

namespace ems::hal {

static constexpr uint8_t kTestSeedSlots = 8u;

static int8_t g_ltft[kNvmLtftDim][kNvmLtftDim] = {};
static int8_t g_knock[8][8]      = {};
static int8_t g_ltft_add[kNvmLtftAddDim][kNvmLtftAddDim] = {};
static uint8_t g_cal[10][1024]   = {};  // linha ≥ maior página (lambda 2×kTableCells)
static uint32_t g_erase_cnt   = 0u, g_prog_cnt = 0u;
static bool     g_flash_busy      = false;  // simulates flash BSY timeout when set
static uint32_t g_flash_busy_polls = 0u;     // non-zero → simulate timeout on next op

// Runtime seed: slot array (mirrors the STM32 flash-backed slot layout)
static RuntimeSyncSeed g_seed_slots[kTestSeedSlots] = {};
static bool g_seed_slot_valid[kTestSeedSlots] = {};

bool nvm_write_ltft(uint8_t r, uint8_t l, int8_t v) noexcept {
    if (r >= kNvmLtftDim || l >= kNvmLtftDim) { return false; }
    if (g_flash_busy) { return false; }
    g_ltft[r][l] = v; return true;
}
int8_t nvm_read_ltft(uint8_t r, uint8_t l) noexcept {
    if (r >= kNvmLtftDim || l >= kNvmLtftDim) { return 0; }
    return g_ltft[r][l];
}
bool nvm_load_adaptive_maps() noexcept { return true; }
void nvm_set_now_ms(uint32_t) noexcept {}
void nvm_request_adaptive_flush_now() noexcept {}
bool nvm_adaptive_maps_dirty() noexcept { return false; }
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
    if (r >= kNvmLtftAddDim || l >= kNvmLtftAddDim) { return false; }
    g_ltft_add[r][l] = v; return true;
}
int8_t nvm_read_ltft_add(uint8_t r, uint8_t l) noexcept {
    if (r >= kNvmLtftAddDim || l >= kNvmLtftAddDim) { return 0; }
    return g_ltft_add[r][l];
}

bool nvm_save_calibration(uint8_t pg, const uint8_t* d, uint16_t l) noexcept {
    if (pg > 9u || d == nullptr || l == 0u) return false;
    if (g_flash_busy) { return false; }
    ++g_erase_cnt; ++g_prog_cnt;
    std::memcpy(g_cal[pg], d, l); return true;
}
bool nvm_load_calibration(uint8_t pg, uint8_t* d, uint16_t l) noexcept {
    if (pg > 9u || d == nullptr || l == 0u) return false;
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
