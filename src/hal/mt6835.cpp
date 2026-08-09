/**
 * @file mt6835.cpp
 * @brief Driver SPI do MT6835 — protocolo verificado contra o datasheet
 *        primário (MagnTek Rev.1.3); pinagem/barramento ainda placeholder.
 *
 * ⚠️ MT6835_HW_PRESENT = 0 — MESMO PADRÃO DO EWG (ewg_driver.cpp)
 * O MT6835 não tem footprint em nenhuma PCB do OpenEMS hoje; este fork é uma
 * exploração pré-hardware (docs/dev/mt6835_encoder_fork.md). Enquanto a flag
 * estiver em 0, nenhuma função abaixo toca em GPIO/SPI de verdade — só o
 * protocolo (frame de 24 bits, CRC-8, mapa de registradores) está implementado
 * e testável, porque essa parte não depende de nenhuma decisão de hardware
 * ainda por tomar. Ao popular o footprint: (1) confirmar CS real (hoje
 * placeholder PC13 — só verificado como "não usado em lado nenhum do código",
 * NÃO verificado contra layout/AF real); (2) por a flag a 1.
 *
 * BARRAMENTO: partilha o SPI2 já usado pelo tle8888.cpp (PB13/14/15
 * SCK/MISO/MOSI, já configurados por tle8888_init()) em vez de reclamar um
 * periférico novo — evita depender de um endereço-base de SPI3 não verificado
 * neste HAL. Custo: o MT6835 usa SPI modo 3 (CPOL=1/CPHA=1) e frame de 8 bits
 * partido em 3 bytes, diferente do modo do TLE8888 (CPOL=0/CPHA=1, 16 bits) —
 * cada transação do MT6835 salva/restaura CFG1/CFG2 para não quebrar o
 * TLE8888. SPE tem de estar a 0 para mudar CFG1/CFG2 (RM0481).
 */

#include "hal/mt6835.h"

#ifdef TARGET_STM32H562
#include "hal/board_pinout.h"
#include "hal/stm32h562/regs.h"
#include "hal/mt6835_regs.h"

namespace {

namespace R = ems::hal::mt6835;

#define MT6835_HW_PRESENT 0

volatile bool     g_ok           = false;
volatile uint16_t g_fault_count  = 0u;
volatile uint8_t  g_last_status  = 0u;

#if MT6835_HW_PRESENT

// ── Placeholder de pinagem — ver aviso no topo do ficheiro ─────────────────
// CS: PC13. Verificado apenas "não referenciado em nenhum outro ficheiro de
// src/" nesta sessão — NÃO é uma decisão de hardware confirmada. PC13 é
// alimentado via VSW (power switch) no H562: nota do datasheet limita-o a
// <2 MHz / <30 pF como fonte de corrente (LED etc.) — um CS de lógica a
// poucos kHz de toggle não esbarra nisso, mas confirmar antes de layout real.
constexpr uint8_t kCsPin = 13u;  // GPIOC

inline void cs_low()  noexcept { GPIOC_BSRR = (1u << (kCsPin + 16u)); }
inline void cs_high() noexcept { GPIOC_BSRR = (1u << kCsPin); }

// Guarda o modo do TLE8888 para restaurar depois de cada transação MT6835.
uint32_t g_saved_cfg1 = 0u;
uint32_t g_saved_cfg2 = 0u;

void spi2_enter_mt6835_mode() noexcept {
    g_saved_cfg1 = SPI2_CFG1;
    g_saved_cfg2 = SPI2_CFG2;
    SPI2_CR1 &= ~SPI_CR1_SPE;  // SPE=0 obrigatório antes de mudar CFG1/CFG2
    SPI2_CFG1 = SPI_CFG1_DSIZE_8BIT | (5u << 28u);  // MBR=101b → /64, margem extra
    // Modo 3: CPOL=1, CPHA=1 (datasheet MT6835 §7.6.2).
    SPI2_CFG2 = SPI_CFG2_MASTER | SPI_CFG2_SSM | SPI_CFG2_CPHA | SPI_CFG2_CPOL
              | SPI_CFG2_COMM_FULLDUPLEX;
    SPI2_CR1 |= SPI_CR1_SPE;
}

void spi2_restore_tle8888_mode() noexcept {
    SPI2_CR1 &= ~SPI_CR1_SPE;
    SPI2_CFG1 = g_saved_cfg1;
    SPI2_CFG2 = g_saved_cfg2;
    SPI2_CR1 |= SPI_CR1_SPE;
}

constexpr uint32_t kSpiTimeout = 50000u;  // ~500 µs @250 MHz, mesmo orçamento do tle8888.cpp

/// Transfere 1 byte (8 bits) em modo full-duplex. CS já deve estar LOW.
uint8_t spi2_byte(uint8_t tx) noexcept {
    uint32_t tries = kSpiTimeout;
    while (!(SPI2_SR & SPI_SR_TXP) && --tries) {}
    if (!tries) { return 0xFFu; }
    SPI2_TXDR = tx;

    SPI2_CR1 |= SPI_CR1_CSTART;

    tries = kSpiTimeout;
    while (!(SPI2_SR & SPI_SR_RXP) && --tries) {}
    if (!tries) { SPI2_IFCR = SPI_IFCR_EOTC | SPI_IFCR_TXTFC; return 0xFFu; }
    const uint8_t rx = static_cast<uint8_t>(SPI2_RXDR);

    tries = kSpiTimeout;
    while (!(SPI2_SR & SPI_SR_EOT) && --tries) {}
    SPI2_IFCR = SPI_IFCR_EOTC | SPI_IFCR_TXTFC;

    return rx;
}

/// Frame de 24 bits do MT6835 (§7.6.3): byte0=cmd+addr[11:8], byte1=addr[7:0],
/// byte2=dado (escrita) ou dummy (leitura — MISO devolve o dado real).
/// Os 3 bytes têm de sair na mesma sessão CS-low.
bool mt6835_xfer(R::Cmd cmd, uint16_t addr, uint8_t data_out, uint8_t* data_in) noexcept {
    spi2_enter_mt6835_mode();
    cs_low();
    (void)spi2_byte(static_cast<uint8_t>((static_cast<uint8_t>(cmd) << 4u)
                                        | ((addr >> 8u) & 0x0Fu)));
    (void)spi2_byte(static_cast<uint8_t>(addr & 0xFFu));
    const uint8_t rx = spi2_byte(data_out);
    cs_high();
    spi2_restore_tle8888_mode();
    if (data_in != nullptr) { *data_in = rx; }
    return true;
}

bool mt6835_write_reg(uint16_t addr, uint8_t data) noexcept {
    return mt6835_xfer(R::Cmd::kWrite, addr, data, nullptr);
}

bool mt6835_read_reg(uint16_t addr, uint8_t* out) noexcept {
    // Comando de leitura: byte2 é dummy (0x00) — MISO devolve o dado real
    // durante esse mesmo byte (§7.6.4, Figura 19 — diferente do TLE8888, que
    // precisa de 2 transações porque o dado só chega no frame seguinte).
    return mt6835_xfer(R::Cmd::kRead, addr, 0x00u, out);
}

bool mt6835_write_verify(uint16_t addr, uint8_t data) noexcept {
    if (!mt6835_write_reg(addr, data)) { return false; }
    uint8_t readback = 0u;
    if (!mt6835_read_reg(addr, &readback)) { return false; }
    return readback == data;
}

/// CRC-8, poly 0x07, MSB primeiro — ver ressalva sobre o valor inicial em
/// mt6835_regs.h.
uint8_t crc8(const uint8_t* data, uint8_t len) noexcept {
    uint8_t crc = R::kCrc8Init;
    for (uint8_t i = 0u; i < len; ++i) {
        crc = static_cast<uint8_t>(crc ^ data[i]);
        for (uint8_t b = 0u; b < 8u; ++b) {
            crc = (crc & 0x80u)
                ? static_cast<uint8_t>(static_cast<uint8_t>(crc << 1u) ^ R::kCrc8Poly)
                : static_cast<uint8_t>(crc << 1u);
        }
    }
    return crc;
}

bool configure_ppr_4096() noexcept {
    // ABZ_RES[13:0] = 4095 (0x0FFF): reg 0x007 = bits[13:6], reg 0x008 = bits[5:0]
    // nos bits [7:2], com ABZ_OFF=0 (saída ligada) e AB_SWAP=0 nos bits[1:0].
    constexpr uint16_t kAbzRes = R::kAbzResPpr4096;
    const uint8_t hi = static_cast<uint8_t>((kAbzRes >> 6u) & 0xFFu);
    const uint8_t lo = static_cast<uint8_t>(((kAbzRes & 0x3Fu) << 2u));  // ABZ_OFF=0, AB_SWAP=0
    // Escreve no register map (volátil) — NÃO grava em EEPROM de propósito:
    // reconfigurar a cada boot evita depender de estado não-volátil e evita
    // desgaste de EEPROM (mesma filosofia do tle8888.cpp: configure() a cada
    // init, não confiar em estado persistido no CI).
    if (!mt6835_write_verify(R::kRegAbzResHi, hi)) { return false; }
    if (!mt6835_write_verify(R::kRegAbzResLo, lo)) { return false; }
    return true;
}

#endif  // MT6835_HW_PRESENT

}  // namespace

namespace ems::hal {

bool mt6835_init() noexcept {
#if !MT6835_HW_PRESENT
    return false;
#else
    RCC_AHB2ENR1 |= RCC_AHB2ENR1_GPIOCEN;
    gpio_set_output(&GPIOC_MODER, &GPIOC_OSPEEDR, kCsPin);
    cs_high();

    g_ok = configure_ppr_4096();
    if (!g_ok) {
        ++g_fault_count;
        return false;
    }

    uint32_t angle21 = 0u;
    uint8_t status = 0u;
    g_ok = mt6835_read_angle_raw21(&angle21, &status);
    if (!g_ok) { ++g_fault_count; }
    return g_ok;
#endif
}

bool mt6835_read_angle_raw21(uint32_t* out_angle21, uint8_t* out_status) noexcept {
#if !MT6835_HW_PRESENT
    (void)out_angle21; (void)out_status;
    return false;
#else
    // Leitura single-byte dos 4 registradores (§7.6.8): o CSN a descer no
    // 1º read já faz o sensor "congelar" 0x003-0x006 até os 4 terem sido
    // lidos — não é preciso usar o comando burst (0xA) para isto ser
    // consistente entre si; burst é só uma otimização de taxa de transferência
    // que não interessa para uma leitura de baixa frequência (key-on/poll).
    uint8_t reg003 = 0u, reg004 = 0u, reg005 = 0u, reg006 = 0u;
    if (!mt6835_read_reg(R::kRegAngle2013,   &reg003)) { return false; }
    if (!mt6835_read_reg(R::kRegAngle125,    &reg004)) { return false; }
    if (!mt6835_read_reg(R::kRegAngle40Stat, &reg005)) { return false; }
    if (!mt6835_read_reg(R::kRegCrc,         &reg006)) { return false; }

    const uint8_t crc_data[3] = { reg003, reg004, reg005 };
    if (crc8(crc_data, 3u) != reg006) {
        ++g_fault_count;
        return false;
    }

    const uint32_t angle21 = (static_cast<uint32_t>(reg003) << 13u)
                            | (static_cast<uint32_t>(reg004) << 5u)
                            | (static_cast<uint32_t>(reg005) >> 3u);
    const uint8_t status = static_cast<uint8_t>(reg005 & 0x07u);

    if (out_angle21 != nullptr) { *out_angle21 = angle21; }
    if (out_status  != nullptr) { *out_status  = status;  }
    g_last_status = status;
    return true;
#endif
}

uint32_t mt6835_angle21_to_tim2_counts(uint32_t angle21) noexcept {
    // (angle21 * 16384) / 2097152 = angle21 >> 7, com arredondamento (+64 =
    // metade do LSB descartado) e máscara para o domínio de 14 bits (16384
    // contagens/volta = 4096 PPR × X4, decisão travada da arquitetura base).
    return ((angle21 + 64u) >> 7u) & 0x3FFFu;
}

bool mt6835_ok() noexcept { return g_ok; }
uint16_t mt6835_fault_count() noexcept { return g_fault_count; }
uint8_t mt6835_last_status() noexcept { return g_last_status; }

}  // namespace ems::hal

#else  // host test stub

namespace ems::hal {
bool mt6835_init() noexcept { return true; }
bool mt6835_read_angle_raw21(uint32_t* out_angle21, uint8_t* out_status) noexcept {
    if (out_angle21 != nullptr) { *out_angle21 = 0u; }
    if (out_status  != nullptr) { *out_status  = 0u; }
    return true;
}
uint32_t mt6835_angle21_to_tim2_counts(uint32_t angle21) noexcept {
    return ((angle21 + 64u) >> 7u) & 0x3FFFu;
}
bool mt6835_ok() noexcept { return true; }
uint16_t mt6835_fault_count() noexcept { return 0u; }
uint8_t mt6835_last_status() noexcept { return 0u; }
}  // namespace ems::hal

#endif  // TARGET_STM32H562
