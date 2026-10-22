#include "app/datalog.h"

#include "hal/board_pinout.h"
#include "hal/crc32.h"
#include "hal/sd.h"

#include <cstring>

// Só a VGT6 tem o SDMMC1 livre (docs/datalog.md); o host testa com o cartão simulado.
#if EMS_BOARD_IS_VGT6 || defined(EMS_HOST_TEST)

namespace ems::app {

namespace {

using ems::hal::SdOpStatus;
using ems::hal::SdState;

constexpr uint16_t kBlockBytes = ems::hal::kSdBlockBytes;
constexpr uint8_t  kWriteRetries = 3u;

// Cabeçalho (16 B): magic, sessão, sequência, CRC-32 do bloco com o campo CRC = 0.
constexpr uint8_t kOffMagic = 0u;
constexpr uint8_t kOffSession = 4u;
constexpr uint8_t kOffSeq = 8u;
constexpr uint8_t kOffCrc = 12u;

alignas(4) uint8_t g_ring[kDatalogRingBlocks][kBlockBytes];
uint8_t g_ring_tail = 0u;    // bloco mais antigo (o que está sendo gravado)
uint8_t g_ring_count = 0u;   // blocos na fila, contando o em gravação

alignas(4) uint8_t g_fill[kBlockBytes];
uint8_t g_fill_n = 0u;

alignas(4) uint8_t g_search_buf[kBlockBytes];

DatalogState g_state = DatalogState::OFF;
uint32_t g_region_blocks = kDatalogRegionBlocks;
uint32_t g_session = 0u;
uint32_t g_seq = 0u;
uint32_t g_next_lba = kDatalogStartLba;
uint32_t g_blocks_written = 0u;
uint32_t g_records_dropped = 0u;
uint32_t g_write_errors = 0u;
uint8_t g_retries = 0u;
bool g_op_pending = false;

// Busca binária do primeiro bloco inválido na região (os válidos formam um prefixo).
uint32_t g_lo = 0u;
uint32_t g_hi = 0u;
bool g_read_last = false;   // última leitura: bloco anterior ao fim, para ler a sessão

void put16(uint8_t* p, uint16_t v) noexcept {
    p[0] = static_cast<uint8_t>(v & 0xFFu);
    p[1] = static_cast<uint8_t>(v >> 8u);
}

void put32(uint8_t* p, uint32_t v) noexcept {
    put16(p, static_cast<uint16_t>(v & 0xFFFFu));
    put16(p + 2, static_cast<uint16_t>(v >> 16u));
}

uint32_t get32(const uint8_t* p) noexcept {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8u) |
           (static_cast<uint32_t>(p[2]) << 16u) | (static_cast<uint32_t>(p[3]) << 24u);
}

uint32_t block_crc(const uint8_t* b) noexcept {
    uint32_t crc = 0xFFFFFFFFu;
    for (uint16_t i = 0u; i < kBlockBytes; ++i) {
        const bool in_crc_field = (i >= kOffCrc) && (i < kOffCrc + 4u);
        crc = ems::hal::crc32_update(crc, in_crc_field ? 0u : b[i]);
    }
    return ~crc;
}

void serialize(uint8_t* p, const DatalogSample& s) noexcept {
    put32(p + 0, s.t_ms);
    put16(p + 4, s.rpm_x10);
    put16(p + 6, s.map_bar_x100);
    put16(p + 8, s.tps_x10);
    put16(p + 10, static_cast<uint16_t>(s.clt_x10));
    put16(p + 12, static_cast<uint16_t>(s.iat_x10));
    put16(p + 14, s.lambda_x1000);
    put16(p + 16, s.lambda_target_x1000);
    put16(p + 18, s.pw_us);
    put16(p + 20, static_cast<uint16_t>(s.advance_x10));
    put16(p + 22, static_cast<uint16_t>(s.stft_x10));
    put16(p + 24, static_cast<uint16_t>(s.ltft_x10));
    put16(p + 26, s.status_bits);
    put16(p + 28, s.cut_reasons);
    p[30] = s.sync_state;
    p[31] = s.flags;
}

void ring_clear() noexcept {
    g_ring_tail = 0u;
    g_ring_count = 0u;
    g_op_pending = false;
}

// Fecha o bloco em preenchimento: numera (a sequência avança mesmo se descartar,
// para a lacuna aparecer na decodificação) e enfileira.
void finalize_fill() noexcept {
    put32(g_fill + kOffMagic, kDatalogMagic);
    put32(g_fill + kOffSession, g_session);
    put32(g_fill + kOffSeq, g_seq);
    put32(g_fill + kOffCrc, 0u);
    put32(g_fill + kOffCrc, block_crc(g_fill));
    ++g_seq;

    if (g_ring_count >= kDatalogRingBlocks) {
        g_records_dropped += kDatalogRecordsPerBlock;
    } else {
        const uint8_t head = static_cast<uint8_t>((g_ring_tail + g_ring_count) % kDatalogRingBlocks);
        std::memcpy(g_ring[head], g_fill, kBlockBytes);
        ++g_ring_count;
    }
    std::memset(g_fill, 0, sizeof(g_fill));
    g_fill_n = 0u;
}

void search_start() noexcept {
    g_lo = 0u;
    g_hi = g_region_blocks;
    g_read_last = false;
    g_op_pending = false;
    g_state = DatalogState::SEARCH;
}

void search_finish(uint32_t session_of_last) noexcept {
    g_session = session_of_last + 1u;
    g_seq = 0u;
    g_next_lba = kDatalogStartLba + g_lo;
    g_fill_n = 0u;
    std::memset(g_fill, 0, sizeof(g_fill));
    ring_clear();
    g_state = (g_lo >= g_region_blocks) ? DatalogState::FULL : DatalogState::ACTIVE;
}

void poll_search() noexcept {
    if (g_op_pending) {
        const SdOpStatus st = ems::hal::sd_op_status();
        if (st == SdOpStatus::BUSY) {
            return;
        }
        g_op_pending = false;
        ems::hal::sd_op_ack();
        if (st == SdOpStatus::ERROR) {
            ++g_write_errors;
            g_state = DatalogState::FAULT;
            return;
        }
        uint32_t session = 0u;
        uint32_t seq = 0u;
        const bool valid = datalog_block_valid(g_search_buf, session, seq);
        if (g_read_last) {
            search_finish(valid ? session : 0u);
            return;
        }
        const uint32_t mid = g_lo + (g_hi - g_lo) / 2u;
        if (valid) {
            g_lo = mid + 1u;
        } else {
            g_hi = mid;
        }
    }
    if (g_lo < g_hi) {
        const uint32_t mid = g_lo + (g_hi - g_lo) / 2u;
        g_op_pending = ems::hal::sd_read_start(kDatalogStartLba + mid, g_search_buf);
        return;
    }
    if (g_lo == 0u) {
        search_finish(0u);   // cartão sem log: sessão 1
        return;
    }
    g_read_last = true;
    g_op_pending = ems::hal::sd_read_start(kDatalogStartLba + g_lo - 1u, g_search_buf);
}

void poll_write() noexcept {
    if (g_op_pending) {
        const SdOpStatus st = ems::hal::sd_op_status();
        if (st == SdOpStatus::BUSY) {
            return;
        }
        g_op_pending = false;
        ems::hal::sd_op_ack();
        if (st == SdOpStatus::OK) {
            g_retries = 0u;
            g_ring_tail = static_cast<uint8_t>((g_ring_tail + 1u) % kDatalogRingBlocks);
            --g_ring_count;
            ++g_next_lba;
            ++g_blocks_written;
        } else {
            ++g_write_errors;
            if (++g_retries >= kWriteRetries) {
                ring_clear();
                g_state = DatalogState::FAULT;
                return;
            }
        }
    }
    if (g_ring_count == 0u) {
        return;
    }
    if (g_next_lba >= kDatalogStartLba + g_region_blocks) {
        ring_clear();
        g_state = DatalogState::FULL;
        return;
    }
    g_op_pending = ems::hal::sd_write_start(g_next_lba, g_ring[g_ring_tail]);
}

}  // namespace

bool datalog_block_valid(const uint8_t* block, uint32_t& session, uint32_t& seq) noexcept {
    if (get32(block + kOffMagic) != kDatalogMagic) {
        return false;
    }
    if (get32(block + kOffCrc) != block_crc(block)) {
        return false;
    }
    session = get32(block + kOffSession);
    seq = get32(block + kOffSeq);
    return true;
}

void datalog_init() noexcept {
    g_state = DatalogState::OFF;
    g_region_blocks = kDatalogRegionBlocks;
    g_session = 0u;
    g_seq = 0u;
    g_next_lba = kDatalogStartLba;
    g_blocks_written = 0u;
    g_records_dropped = 0u;
    g_write_errors = 0u;
    g_retries = 0u;
    g_fill_n = 0u;
    std::memset(g_fill, 0, sizeof(g_fill));
    ring_clear();
}

void datalog_push(const DatalogSample& s) noexcept {
    if (g_state != DatalogState::ACTIVE) {
        return;
    }
    serialize(g_fill + kDatalogHeaderBytes + static_cast<uint16_t>(g_fill_n) * kDatalogRecordBytes, s);
    if (++g_fill_n >= kDatalogRecordsPerBlock) {
        finalize_fill();
    }
}

void datalog_poll() noexcept {
    switch (g_state) {
        case DatalogState::OFF:
            if (ems::hal::sd_state() == SdState::READY) {
                search_start();
            }
            break;
        case DatalogState::SEARCH:
            poll_search();
            break;
        case DatalogState::ACTIVE:
            poll_write();
            break;
        case DatalogState::FULL:
        case DatalogState::FAULT:
            break;
    }
}

DatalogStats datalog_stats() noexcept {
    return DatalogStats{g_state, g_session, g_blocks_written, g_records_dropped, g_write_errors};
}

#if defined(EMS_HOST_TEST)
void datalog_test_set_region_blocks(uint32_t blocks) noexcept { g_region_blocks = blocks; }
#endif

}  // namespace ems::app

#endif  // EMS_BOARD_IS_VGT6 || EMS_HOST_TEST
