/**
 * @file tle8888.cpp
 * @brief Driver do TLE8888 (-2QK na v1) — reescrito contra o datasheet Rev. 1.2.
 *
 * ⚠️ REESCRITA COMPLETA (2026-07-20). A versão anterior deste ficheiro não podia
 * funcionar: tinha o **mapa de registadores inventado** *e* o **frame SPI
 * invertido**. Detalhe da comparação registador a registador em
 * `hal/tle8888_regs.h`. Os dois erros mais graves eram:
 *   - "trigger de watchdog" escrevia 0x01 em 0x57, que é **DDConfig0** — ligava
 *     o direct-drive do injector 1 por acidente a cada poll de diagnóstico;
 *   - "chip ID" lia 0x7E, que é **Cont3** (registador de controlo de saídas),
 *     à espera de 0x88 — não existe registador de chip ID no dispositivo.
 * Nunca clockou silício (o conflito PB12/PB13 mata o SPI2_SCK no arranque), pelo
 * que os erros nunca foram expostos.
 *
 * ARQUITECTURA (ver `docs/hw/interface_board_v1.md`)
 * Injecção e ignição são accionadas por **direct drive**, não por SPI: os pinos
 * IN1–IN4 → OUT1–OUT4 (injectores) e IN5–IN8 → IGN1–IGN4 (ignição), com
 * atribuição fixa (Tab. 24), activos a nível alto e com pull-down interno. O
 * escalonador continua a escrever GPIOE_BSRR exactamente como antes — este
 * driver só **configura e diagnostica**, nunca está no caminho crítico de
 * temporização.
 *
 * FRAME SPI (cap. 15.2): 16 bits, máx 5 MHz
 *   bit    0  → C0, R/W (1 = escrita, 0 = leitura)
 *   bits 7:1  → endereço do registador (offset de 7 bits)
 *   bits 15:8 → dados
 * Leitura: o registador é carregado no shift register de saída na borda de
 * subida de CSN, pelo que a resposta chega no **frame seguinte** (2 transacções).
 *
 * ⚠️ WATCHDOG NÃO IMPLEMENTADO — ver nota no fim do ficheiro.
 */

#include "hal/tle8888.h"

#ifdef TARGET_STM32H562
#include "hal/board_pinout.h"
#include "hal/stm32h562/regs.h"
#include "hal/tle8888_regs.h"

namespace {

namespace R = ems::hal::tle;

// ── Estado ───────────────────────────────────────────────────────────────────
volatile uint16_t g_fault_count = 0u;
volatile bool     g_comms_ok    = false;
volatile bool     g_configured  = false;

/// Diagnóstico por canal: [0-3] = INJ1-4 (OUT1-4), [4-7] = IGN1-4.
/// Codificação do datasheet (OutDiag0 / IgnDiag, 2 bits por canal):
///   0 = sem falha
///   1 = curto ao positivo (sobrecorrente) OU sobretemperatura
///   2 = open load (em off)
///   3 = curto à massa (em off)
/// ⚠️ Difere do driver antigo, que usava 1=open-load / 2=short-GND / 3=short-BAT.
volatile uint8_t g_channel_faults[8] = {};

/// VRSDiag0 mais recente (short-GND / short-BAT / open-load do sensor CKP).
volatile uint8_t g_vrs_diag = 0u;

/// Estado bruto do módulo de watchdog — ver nota no fim do ficheiro.
/// [0]=WdStat0  [1]=WWDStat  [2]=FWDStat0  [3]=TECStat  [4]=WdDiag
volatile uint8_t g_wd_status[5] = {};

/// Bitmask do fingerprint (consultivo): bit N = entrada N divergiu.
/// 0 = valores de reset batem. NÃO é gate de segurança — ver eco abaixo.
volatile uint8_t g_map_mismatch = 0xFFu;

/// Eco de endereço SPI (gate contínuo, estilo rusEFI spi_validate).
/// A resposta devolve o endereço do frame *anterior*. 0x00 = REG_INVALID
/// (Cmd0 é 0x01; 0x00 não é registador real).
constexpr uint8_t kRegInvalid = 0x00u;
volatile uint8_t g_last_tx_reg   = kRegInvalid;
volatile uint8_t g_echo_por_cnt  = 0u;   // resposta OpStat0 → power-on reset
volatile uint8_t g_echo_wdr_cnt  = 0u;   // FWDStat1 → watchdog reset
volatile uint8_t g_echo_comfe_cnt = 0u;  // Diag0 → frame inválido
volatile uint8_t g_echo_fail_cnt = 0u;   // divergência genérica
volatile bool    g_echo_ok       = true;
volatile bool    g_need_reinit   = false;

// ── Transporte SPI ───────────────────────────────────────────────────────────
// VGT6 / default: SPI2 on PB12(CSN)+PB13/14/15
// MRE copper:     SPI1 on PD5(CSN)+PB3/4/5  (docs/hw/pinout_mre_bringup.md)
constexpr uint32_t kSpiTimeout = 50000u;  // ~500 µs @250 MHz

#if EMS_BOARD_IS_MRE
inline void cs_low()  noexcept { GPIOD_BSRR = (1u << (5u + 16u)); }  // PD5 = CSN
inline void cs_high() noexcept { GPIOD_BSRR = (1u << 5u); }
#define TLE_SPI_CR1   SPI1_CR1
#define TLE_SPI_CFG1  SPI1_CFG1
#define TLE_SPI_CFG2  SPI1_CFG2
#define TLE_SPI_CR2   SPI1_CR2
#define TLE_SPI_SR    SPI1_SR
#define TLE_SPI_IFCR  SPI1_IFCR
#define TLE_SPI_TXDR  SPI1_TXDR
#define TLE_SPI_RXDR  SPI1_RXDR
#else
inline void cs_low()  noexcept { GPIOB_BSRR = (1u << (12u + 16u)); }  // PB12 = CSN
inline void cs_high() noexcept { GPIOB_BSRR = (1u << 12u); }
#define TLE_SPI_CR1   SPI2_CR1
#define TLE_SPI_CFG1  SPI2_CFG1
#define TLE_SPI_CFG2  SPI2_CFG2
#define TLE_SPI_CR2   SPI2_CR2
#define TLE_SPI_SR    SPI2_SR
#define TLE_SPI_IFCR  SPI2_IFCR
#define TLE_SPI_TXDR  SPI2_TXDR
#define TLE_SPI_RXDR  SPI2_RXDR
#endif

/// Monta o frame: dados em [15:8], endereço em [7:1], R/W no bit 0.
inline uint16_t frame(uint16_t addr, uint8_t data, bool write) noexcept {
    return static_cast<uint16_t>((static_cast<uint16_t>(data) << 8u)
                               | ((addr & 0x7Fu) << 1u)
                               | (write ? 1u : 0u));
}

inline uint8_t addr_from_frame(uint16_t word) noexcept {
    return static_cast<uint8_t>((word >> 1u) & 0x7Fu);
}

uint16_t spi_txrx_raw(uint16_t tx) noexcept {
    uint32_t tries = kSpiTimeout;
    while (!(TLE_SPI_SR & SPI_SR_TXP) && --tries) {}
    if (!tries) { return 0xFFFFu; }
    TLE_SPI_TXDR = tx;

    TLE_SPI_CR1 |= SPI_CR1_CSTART;

    tries = kSpiTimeout;
    while (!(TLE_SPI_SR & SPI_SR_RXP) && --tries) {}
    if (!tries) { TLE_SPI_IFCR = SPI_IFCR_EOTC | SPI_IFCR_TXTFC; return 0xFFFFu; }
    const uint16_t rx = static_cast<uint16_t>(TLE_SPI_RXDR);

    tries = kSpiTimeout;
    while (!(TLE_SPI_SR & SPI_SR_EOT) && --tries) {}
    TLE_SPI_IFCR = SPI_IFCR_EOTC | SPI_IFCR_TXTFC;

    return rx;
}

/// Transacção com validação de eco (resposta = reg do frame anterior).
uint16_t spi_xfer(uint16_t tx) noexcept {
    cs_low();
    const uint16_t rx = spi_txrx_raw(tx);
    cs_high();

    if (g_last_tx_reg != kRegInvalid) {
        const uint8_t got = addr_from_frame(rx);
        if (got != g_last_tx_reg) {
            // Eventos especiais do datasheet / rusEFI (não são “mapa errado”):
            if (got == static_cast<uint8_t>(R::OP_STAT0)) {
                ++g_echo_por_cnt;
                g_need_reinit = true;
            } else if (got == static_cast<uint8_t>(R::FWD_STAT1)) {
                ++g_echo_wdr_cnt;
                g_need_reinit = true;
            } else if (got == static_cast<uint8_t>(R::DIAG0)) {
                ++g_echo_comfe_cnt;
                g_need_reinit = true;
            } else {
                ++g_echo_fail_cnt;
            }
            g_echo_ok = false;
        }
    }
    g_last_tx_reg = addr_from_frame(tx);
    return rx;
}

void tle_write(uint16_t addr, uint8_t data) noexcept {
    (void)spi_xfer(frame(addr, data, true));
}

uint8_t tle_read(uint16_t addr) noexcept {
    // 1ª transacção: comando de leitura. Conteúdo carrega no SDO na subida CSN.
    (void)spi_xfer(frame(addr, 0u, false));
    // 2ª: recolhe a resposta (eco do 1º comando = addr + data).
    const uint16_t resp = spi_xfer(frame(addr, 0u, false));
    return static_cast<uint8_t>((resp >> 8u) & 0xFFu);
}

bool write_verify(uint16_t addr, uint8_t data) noexcept {
    tle_write(addr, data);
    return tle_read(addr) == data;
}

// ── Fingerprint CONSULTIVO (não bloqueante) ──────────────────────────────────
// Reduzido a endereços que o rusEFI também usa (mapa confirmado offline).
// Valores de reset NÃO têm segunda fonte — um typo latched mataria INJ num CI
// saudável. O gate de segurança é o eco de endereço + write_verify.
// Ver docs/hw/tle8888_crosscheck.md §3 e política no interface_board_v1.md.
struct ResetFingerprint {
    uint16_t addr;
    uint8_t  expected;
};

constexpr ResetFingerprint kResetFingerprint[] = {
    { R::OP_CONFIG0,  0x09u },
    { R::OUT_CONFIG3, 0x30u },
};
constexpr uint8_t kFingerprintCount =
    static_cast<uint8_t>(sizeof(kResetFingerprint) / sizeof(kResetFingerprint[0]));
static_assert(kFingerprintCount <= 8u, "g_map_mismatch só tem 8 bits");

void verify_register_map_advisory() noexcept {
    uint8_t mismatch = 0u;
    for (uint8_t i = 0u; i < kFingerprintCount; ++i) {
        if (tle_read(kResetFingerprint[i].addr) != kResetFingerprint[i].expected) {
            mismatch = static_cast<uint8_t>(mismatch | (1u << i));
        }
    }
    g_map_mismatch = mismatch;
}

// ── Configuração ─────────────────────────────────────────────────────────────
// Ordem alinhada com rusEFI chip_init(): unlock → InConfig → DD/OE → OE_SET.
// Ver docs/hw/tle8888_crosscheck.md §5.
bool configure() noexcept {
    // 0. Unlock (comando — sem write_verify). Sem isto as escritas de config
    //    ficam protegidas e não pegam.
    tle_write(R::CMD_LOCK, R::CMD_CHIP_UNLOCK_DATA);

    // 1. Mapa IN9..IN12 → saídas remapeáveis (bomba/fan/VVT).
    //    Encoding: valor = (índice 0-based de OUTn) − 4 (rusEFI / Tab. 24).
    if (!write_verify(R::IN_CONFIG0, R::IN_CONFIG0_OPENEMS)) { return false; }
    if (!write_verify(R::IN_CONFIG1, R::IN_CONFIG1_OPENEMS)) { return false; }
    if (!write_verify(R::IN_CONFIG2, R::IN_CONFIG2_OPENEMS)) { return false; }
    if (!write_verify(R::IN_CONFIG3, R::IN_CONFIG3_OPENEMS)) { return false; }

    // 2. Direct drive: INJ1-4 + VVT OUT5/6; bomba/fan OUT14/15; IGN1-4.
    if (!write_verify(R::DD_CONFIG0, R::DD_CONFIG0_OPENEMS)) { return false; }
    if (!write_verify(R::DD_CONFIG1, R::DD_CONFIG1_OPENEMS)) { return false; }
    if (!write_verify(R::DD_CONFIG3, R::DD_CONFIG3_OPENEMS)) { return false; }

    // 3. Interface VR do CKP: modo auto-detecção, diagnóstico em modo normal.
    //    (VRSM=00 é o reset value; escrevemos explicitamente para não depender
    //     do estado de arranque. Em modo auto o CI ignora VRSPV/VRSPT.)
    if (!write_verify(R::VRS_CONFIG1, R::VRS_CONFIG1_OPENEMS)) { return false; }

    // 4. Output enable por canal. Estes bits são repostos a 0 pela protecção —
    //    daí a sequência de recuperação em poll_diag.
    if (!write_verify(R::OE_CONFIG0, R::OE_CONFIG0_OPENEMS)) { return false; }
    if (!write_verify(R::OE_CONFIG1, R::OE_CONFIG1_OPENEMS)) { return false; }
    if (!write_verify(R::OE_CONFIG3, R::OE_CONFIG3_OPENEMS)) { return false; }

    // 5. Enable central (comando — sem write_verify). Sem isto as saídas
    //    plausivelmente nunca habilitam mesmo com OEConfig correcto.
    tle_write(R::CMD_OE, R::CMD_OE_SET_DATA);
    return true;
}

void rearm_output_enables() noexcept {
    (void)write_verify(R::OE_CONFIG0, R::OE_CONFIG0_OPENEMS);
    (void)write_verify(R::OE_CONFIG1, R::OE_CONFIG1_OPENEMS);
    (void)write_verify(R::OE_CONFIG3, R::OE_CONFIG3_OPENEMS);
    tle_write(R::CMD_OE, R::CMD_OE_SET_DATA);
}

/// Descodifica um registador de diagnóstico com 4 canais × 2 bits.
void decode_quad(uint8_t reg, uint8_t first_ch) noexcept {
    for (uint8_t i = 0u; i < 4u; ++i) {
        g_channel_faults[first_ch + i] =
            static_cast<uint8_t>((reg >> (i * 2u)) & 0x03u);
    }
}

}  // namespace

namespace ems::hal {

void tle8888_init() noexcept {
#if EMS_BOARD_IS_MRE
    // ── 1–2. mRE: PD5 CSN, PB3/4/5 = SPI1 SCK/MISO/MOSI (AF5) ──────────────
    RCC_AHB2ENR1 |= RCC_AHB2ENR1_GPIOBEN | RCC_AHB2ENR1_GPIODEN;
    RCC_APB2ENR  |= RCC_APB2ENR_SPI1EN;
    gpio_set_output(&GPIOD_MODER, &GPIOD_OSPEEDR, 5u);
    cs_high();
    gpio_set_af(&GPIOB_MODER, &GPIOB_AFRL, &GPIOB_AFRH, &GPIOB_OSPEEDR, 3u, GPIO_AF5);
    gpio_set_af(&GPIOB_MODER, &GPIOB_AFRL, &GPIOB_AFRH, &GPIOB_OSPEEDR, 4u, GPIO_AF5);
    gpio_set_af(&GPIOB_MODER, &GPIOB_AFRL, &GPIOB_AFRH, &GPIOB_OSPEEDR, 5u, GPIO_AF5);
#else
    // ── 1–2. VGT6: PB12 CSN, PB13/14/15 = SPI2 SCK/MISO/MOSI (AF5) ─────────
    // ⚠️ auxiliaries_init() não pode reclamar PB12/PB13 (bomba/fan → PE10/12).
    RCC_AHB2ENR1 |= RCC_AHB2ENR1_GPIOBEN;
    RCC_APB1LENR |= RCC_APB1LENR_SPI2EN;
    gpio_set_output(&GPIOB_MODER, &GPIOB_OSPEEDR, 12u);
    cs_high();
    gpio_set_af(&GPIOB_MODER, &GPIOB_AFRL, &GPIOB_AFRH, &GPIOB_OSPEEDR, 13u, GPIO_AF5);
    gpio_set_af(&GPIOB_MODER, &GPIOB_AFRL, &GPIOB_AFRH, &GPIOB_OSPEEDR, 14u, GPIO_AF5);
    gpio_set_af(&GPIOB_MODER, &GPIOB_AFRL, &GPIOB_AFRH, &GPIOB_OSPEEDR, 15u, GPIO_AF5);
#endif

    // ── 3. SPI mestre, 16 bits, CPOL=0/CPHA=1, ~3.9 MHz (< 5 MHz máx) ───────
    TLE_SPI_CR1  = 0u;
    TLE_SPI_CFG1 = SPI_CFG1_DSIZE_16BIT | (4u << 28u);  // MBR=100b → /32
    TLE_SPI_CFG2 = SPI_CFG2_MASTER | SPI_CFG2_SSM | SPI_CFG2_CPHA
                 | SPI_CFG2_COMM_FULLDUPLEX;
    TLE_SPI_CR2  = 1u;
    TLE_SPI_CR1  = SPI_CR1_SPE;

    // ── 4. Fingerprint CONSULTIVO (não bloqueia) ────────────────────────────
    // Antes de qualquer escrita (valores de reset). Falhar aqui NÃO aborta:
    // 5/7 entradas antigas usavam regs sem 2ª fonte (rusEFI). Telemetria só.
    g_last_tx_reg = kRegInvalid;
    g_echo_ok     = true;
    g_need_reinit = false;
    verify_register_map_advisory();

    // ── 5. Unlock + verificação de comunicação (caminho de ESCRITA + eco) ──
    // Unlock tem de preceder qualquer write_verify. O eco de endereço (spi_xfer)
    // é o gate contínuo: prova link, frame e round-trip sem depender dos resets.
    tle_write(R::CMD_LOCK, R::CMD_CHIP_UNLOCK_DATA);
    g_comms_ok = write_verify(R::DD_CONFIG0, R::DD_CONFIG0_OPENEMS) && g_echo_ok;
    if (!g_comms_ok) {
        ++g_fault_count;
        g_configured = false;
        return;
    }

    g_configured = configure() && g_echo_ok;
    if (!g_configured) {
        g_comms_ok = false;
        ++g_fault_count;
    }
}

void tle8888_poll_diag() noexcept {
    // Eco partido / POR / WDR / COMFE → reconfigurar (não latch de fingerprint).
    if (g_need_reinit || !g_echo_ok) {
        g_comms_ok    = false;
        g_configured  = false;
        g_need_reinit = false;
        g_echo_ok     = true;
        g_last_tx_reg = kRegInvalid;
    }

    if (!g_comms_ok || !g_configured) {
        // Tenta reestabelecer: unlock → escrita → configuração completa.
        tle_write(R::CMD_LOCK, R::CMD_CHIP_UNLOCK_DATA);
        g_comms_ok = write_verify(R::DD_CONFIG0, R::DD_CONFIG0_OPENEMS) && g_echo_ok;
        if (!g_comms_ok) { ++g_fault_count; return; }
        g_configured = configure() && g_echo_ok;
        if (!g_configured) {
            g_comms_ok = false;
            ++g_fault_count;
            return;
        }
    }

    // ── Diagnóstico por canal ───────────────────────────────────────────────
    decode_quad(tle_read(R::OUT_DIAG0), 0u);  // OUT1-4 = injectores
    decode_quad(tle_read(R::IGN_DIAG),  4u);  // IGN1-4 = bobinas
    g_vrs_diag = tle_read(R::VRS_DIAG0);      // sensor CKP: short / open-load

    // ── Estado do watchdog (leitura apenas) ─────────────────────────────────
    // Não servimos o watchdog (ver nota no fim), mas lemos o estado para que a
    // entrada em Safe State seja *visível* em vez de misteriosa: nesse estado o
    // CI desliga os power stages e força O1E..O24E/IGN1E a 0 — injecção e
    // ignição morrem e, sem esta leitura, pareceria falha mecânica.
    g_wd_status[0] = tle_read(R::WD_STAT0);
    g_wd_status[1] = tle_read(R::WWD_STAT);
    g_wd_status[2] = tle_read(R::FWD_STAT0);
    g_wd_status[3] = tle_read(R::TEC_STAT);
    g_wd_status[4] = tle_read(R::WD_DIAG);

    bool any_fault = false;
    for (uint8_t i = 0u; i < 8u; ++i) {
        if (g_channel_faults[i] != 0u) { any_fault = true; break; }
    }

    if (any_fault) {
        ++g_fault_count;
        // Sequência de recuperação do datasheet (cap. 9.2): a protecção repõe
        // os bits de enable a 0. É preciso reler o diagnóstico para confirmar
        // que a falha já não persiste e só então re-armar o enable — caso
        // contrário o canal fica morto até ao próximo arranque.
        decode_quad(tle_read(R::OUT_DIAG0), 0u);
        decode_quad(tle_read(R::IGN_DIAG),  4u);

        bool still_faulted = false;
        for (uint8_t i = 0u; i < 8u; ++i) {
            if (g_channel_faults[i] != 0u) { still_faulted = true; break; }
        }
        if (!still_faulted) {
            rearm_output_enables();
        }
    }
}

bool tle8888_ok() noexcept {
    return g_comms_ok && g_configured;
}

uint16_t tle8888_fault_count() noexcept {
    return g_fault_count;
}

uint8_t tle8888_channel_fault(uint8_t ch) noexcept {
    if (ch >= 8u) { return 0u; }
    return g_channel_faults[ch];
}

uint8_t tle8888_fault_bitmap() noexcept {
    uint8_t bm = 0u;
    for (uint8_t i = 0u; i < 8u; ++i) {
        if (g_channel_faults[i] != 0u) { bm = static_cast<uint8_t>(bm | (1u << i)); }
    }
    return bm;
}

uint8_t tle8888_vrs_diag() noexcept {
    return g_vrs_diag;
}

uint8_t tle8888_wd_status(uint8_t idx) noexcept {
    if (idx >= 5u) { return 0u; }
    return g_wd_status[idx];
}

uint8_t tle8888_map_mismatch() noexcept { return g_map_mismatch; }

uint8_t tle8888_echo_status(uint8_t idx) noexcept {
    // 0=ok(1/0) 1=por 2=wdr 3=comfe 4=fail  — telemetria / bring-up
    switch (idx) {
    case 0: return g_echo_ok ? 1u : 0u;
    case 1: return g_echo_por_cnt;
    case 2: return g_echo_wdr_cnt;
    case 3: return g_echo_comfe_cnt;
    case 4: return g_echo_fail_cnt;
    default: return 0u;
    }
}

}  // namespace ems::hal

/*
 * ⚠️ WATCHDOG — DELIBERADAMENTE NÃO IMPLEMENTADO
 *
 * O TLE8888 tem um módulo de monitorização em duas partes (cap. 6) — activo
 * na variante -1QK; a v1 usa -2QK com watchdog desactivado de fábrica:
 *   - Window Watchdog (WWD): verificação temporal, por comando de serviço;
 *   - Functional Watchdog (FWD): verificação lógica por **pergunta/resposta**,
 *     em que o microcontrolador tem de executar rotinas de auto-teste e devolver
 *     quatro bytes de resposta correctos.
 * Ambos alimentam contadores de erro que, por overflow, actuam o reset do
 * watchdog, o power-down counter e o **secure shut off timer** — ou seja, podem
 * desligar as saídas e actuar os pinos MON/RST.
 *
 * Implementar isto pela metade é pior do que não implementar: dá a ilusão de
 * cobertura e pode desligar injecção e ignição com o motor a trabalhar. Foi
 * exactamente o que a versão anterior deste ficheiro fez — "alimentava" o
 * watchdog escrevendo num registador de configuração ao acaso.
 *
 * ⚠️ NÃO É OPCIONAL NO -1QK. Table 11/12 do datasheet:
 *     Safe State  ⇐ WWDEC > 32  OU  FWDPC > 32  OU  TEC > 32
 *     Safe State  ⇒ "Power stages: **disabled**", "O1E..O24E, IGN1E: **0**"
 *   Ou seja: num TLE8888-1QK, se o firmware não servir o watchdog, os
 *   contadores sobem e o CI **desliga injecção e ignição** sozinho. Durante o
 *   arranque de um motor isso parece falha mecânica ou de combustível — é o
 *   pior modo de falha possível para diagnosticar.
 *
 * ⚠️ CORRECÇÃO a uma ideia anterior: pôr **WDREN = 0 NÃO resolve**. Esse bit só
 *   controla se o *watchdog reset* ocorre no overflow dos contadores (cap. 6.4);
 *   o Safe State acontece antes disso, aos 32, e já desliga os power stages.
 *
 * Restam por isso duas opções reais:
 *   (a) implementar WWD + FWD a sério. O WWD é simples (comando periódico
 *       WWDServiceCmd dentro da janela aberta); o FWD é pergunta/resposta com
 *       quatro bytes e rotinas de auto-teste do MCU — software de segurança que
 *       merece esforço e validação próprios. Implementar só o WWD não chega:
 *       o FWDPC sobe na mesma e leva a Safe State.
 *   (b) **usar a variante TLE8888-2QK**, em que o watchdog vem desactivado de
 *       fábrica. Mesmo package LQFP-100, mesma pinagem — difere só nisso.
 *
 * RECOMENDADO PARA A v1: **(b) TLE8888-2QK**. O watchdog é valioso, mas errá-lo
 * mata o motor de forma imprevisível e difícil de diagnosticar exactamente na
 * fase de primeira partida; e o MCU já tem o seu IWDG independente. Fazer (a)
 * depois, com validação dedicada, e migrar para -1QK numa placa de produção.
 *
 * Entretanto este driver **lê** o estado do watchdog (WdStat0, WWDStat,
 * FWDStat0, TECStat, WdDiag → tle8888_wd_status()) para que, se um -1QK for
 * montado por engano, a entrada em Safe State apareça na telemetria em vez de
 * se manifestar como "o motor morreu sem razão".
 * Ver docs/hw/interface_board_v1.md.
 */

#else  // host test stub

namespace ems::hal {
void tle8888_init() noexcept {}
void tle8888_poll_diag() noexcept {}
bool tle8888_ok() noexcept { return true; }
uint16_t tle8888_fault_count() noexcept { return 0u; }
uint8_t tle8888_channel_fault(uint8_t) noexcept { return 0u; }
uint8_t tle8888_fault_bitmap() noexcept { return 0u; }
uint8_t tle8888_vrs_diag() noexcept { return 0u; }
uint8_t tle8888_wd_status(uint8_t) noexcept { return 0u; }
uint8_t tle8888_map_mismatch() noexcept { return 0u; }
uint8_t tle8888_echo_status(uint8_t) noexcept { return 0u; }
}

#endif
