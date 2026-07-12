#pragma once

#include <cstdint>

namespace ems::engine {

constexpr uint8_t kTableAxisSize = 16u;
// Derivados — usar SEMPRE estes em serialização/buffers em vez de literais:
// células por tabela (u8/i8 = kTableCells bytes; i16 = 2×kTableCells) e a
// dimensão do sub-grid LTFT aditivo (indexado por idx>>1 do grid principal).
constexpr uint16_t kTableCells = static_cast<uint16_t>(kTableAxisSize) * kTableAxisSize;
constexpr uint8_t  kLtftAddAxisSize = (kTableAxisSize + 1u) / 2u;

// Versão do layout de calibração (byte 175 da página 0, área livre).
// Páginas de tabela gravadas em flash só são carregadas no boot se a versão
// bater — evita que um blob antigo menor seja lido com cauda 0xFF (ex.: VE
// de 256B interpretado como 400B = células a 255). Incrementar sempre que
// kTableAxisSize (ou o layout serializado das tabelas) mudar.
constexpr uint8_t kCalLayoutVersion       = 2u;
constexpr uint16_t kCalLayoutVersionOffset = 175u;

// Eixos das tabelas 16×16. Editáveis em runtime via protocolo (página 11);
// mutação apenas por table_axes_set(), que valida monotonicidade estrita.
extern uint32_t kRpmAxisX10[kTableAxisSize];
extern uint32_t kLoadAxisBarX100[kTableAxisSize];

// Aplica novos eixos a partir da forma serializada da página 11
// (RPM cru em u16, load em bar×100 u16). Rejeita (sem alterar nada) se
// qualquer eixo não for estritamente crescente ou se rpm[0] == 0.
bool table_axes_set(const uint16_t rpm[kTableAxisSize],
                    const uint16_t load_bar_x100[kTableAxisSize]) noexcept;

// Serializa os eixos atuais para a forma da página 11 (RPM cru, load bar×100).
void table_axes_get(uint16_t rpm[kTableAxisSize],
                    uint16_t load_bar_x100[kTableAxisSize]) noexcept;

struct Table2dLookup {
    uint8_t xi;
    uint8_t yi;
    uint8_t fx_q8;
    uint8_t fy_q8;
};

uint8_t table_axis_index(const uint32_t* axis, uint8_t size, uint32_t value) noexcept;
uint8_t table_axis_frac_q8(const uint32_t* axis, uint8_t idx, uint32_t value) noexcept;

Table2dLookup table3d_prepare_lookup(const uint32_t* x_axis,
                                     const uint32_t* y_axis,
                                     uint32_t x,
                                     uint32_t y) noexcept;

uint8_t table3d_lookup_u8_prepared(const uint8_t table[kTableAxisSize][kTableAxisSize],
                                   const Table2dLookup& lookup) noexcept;

int16_t table3d_lookup_i8_prepared(const int8_t table[kTableAxisSize][kTableAxisSize],
                                   const Table2dLookup& lookup) noexcept;

int16_t table3d_lookup_s16_prepared(const int16_t table[kTableAxisSize][kTableAxisSize],
                                    const Table2dLookup& lookup) noexcept;

uint8_t table3d_lookup_u8(const uint8_t table[kTableAxisSize][kTableAxisSize],
                          const uint32_t* x_axis,
                          const uint32_t* y_axis,
                          uint32_t x,
                          uint32_t y) noexcept;

int16_t table3d_lookup_s16(const int16_t table[kTableAxisSize][kTableAxisSize],
                           const uint32_t* x_axis,
                           const uint32_t* y_axis,
                           uint32_t x,
                           uint32_t y) noexcept;

// Funções otimizadas para VE e Advance lookup com fixed-point arithmetic
uint16_t table3d_lookup_ve_q8(const uint8_t ve_table[kTableAxisSize][kTableAxisSize],
                             const uint32_t* x_axis,
                             const uint32_t* y_axis,
                             uint32_t x,
                             uint32_t y) noexcept;

int32_t table3d_lookup_advance_q10(const int16_t advance_table[kTableAxisSize][kTableAxisSize],
                                 const uint32_t* x_axis,
                                 const uint32_t* y_axis,
                                 uint32_t x,
                                 uint32_t y) noexcept;

}  // namespace ems::engine
