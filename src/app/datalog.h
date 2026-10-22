#pragma once

#include <cstdint>

namespace ems::app {

// Datalog em cartão SD, blocos brutos de 512 B sem sistema de arquivos
// (formato em docs/datalog.md). Nada aqui bloqueia nem roda em ISR:
// datalog_push() só copia 32 B; datalog_poll() avança uma operação por chamada.

// Amostra de 32 B. Na mídia é little-endian, campo a campo, nesta ordem
// (offsets em datalog.cpp e tools/datalog/sdlog.py).
struct DatalogSample {
    uint32_t t_ms;
    uint16_t rpm_x10;
    uint16_t map_bar_x100;
    uint16_t tps_x10;
    int16_t  clt_x10;
    int16_t  iat_x10;
    uint16_t lambda_x1000;
    uint16_t lambda_target_x1000;
    uint16_t pw_us;
    int16_t  advance_x10;
    int16_t  stft_x10;
    int16_t  ltft_x10;
    uint16_t status_bits;
    uint16_t cut_reasons;
    uint8_t  sync_state;
    uint8_t  flags;
};

enum class DatalogState : uint8_t {
    OFF    = 0,  // sem cartão pronto
    SEARCH = 1,  // achando o fim do log anterior
    ACTIVE = 2,  // gravando
    FULL   = 3,  // região esgotada
    FAULT  = 4,  // gravação falhou 3 vezes seguidas
};

struct DatalogStats {
    DatalogState state;
    uint32_t session;
    uint32_t blocks_written;
    uint32_t records_dropped;   // descartados com o anel cheio
    uint32_t write_errors;
};

constexpr uint32_t kDatalogMagic         = 0x31474C4Fu;  // "OLG1"
constexpr uint32_t kDatalogStartLba      = 2048u;        // 1 MiB: fora de tabela de partição
constexpr uint32_t kDatalogRegionBlocks  = 262144u;      // 128 MiB
constexpr uint16_t kDatalogHeaderBytes   = 16u;
constexpr uint16_t kDatalogRecordBytes   = 32u;
constexpr uint8_t  kDatalogRecordsPerBlock = 15u;
constexpr uint8_t  kDatalogRingBlocks    = 16u;

void datalog_init() noexcept;
void datalog_push(const DatalogSample& s) noexcept;
void datalog_poll() noexcept;
DatalogStats datalog_stats() noexcept;

// Confere magic e CRC-32 de um bloco; devolve sessão e sequência.
bool datalog_block_valid(const uint8_t* block, uint32_t& session, uint32_t& seq) noexcept;

#if defined(EMS_HOST_TEST)
void datalog_test_set_region_blocks(uint32_t blocks) noexcept;
#endif

}  // namespace ems::app
