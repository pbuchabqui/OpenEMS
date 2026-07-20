#pragma once
/**
 * @file tle8888_regs.h
 * @brief Mapa de registradores do TLE8888-1QK — VERIFICADO contra o datasheet.
 *
 * Fonte: Infineon TLE8888-1QK "Engine Machine System IC", Data Sheet **Rev. 1.2**
 * (2017-02-10), Table 50 "Register Overview" (cap. 14.1) e cap. 14.1.4/14.1.5.
 *
 * ⚠️ MOTIVO DESTE ARQUIVO
 * O mapa de registradores embutido em `tle8888.cpp` **não corresponde ao
 * datasheet**. O comentário de lá cita "rev 2.1" e os nomes usados espelham a
 * narrativa (legada) de `docs/wiring_diagram.md`. Como o conflito de pinos
 * PB12/PB13 (bomba/ventoinha vs SPI2) mata o SPI2_SCK no boot, aquele driver
 * nunca clockou silício — o mapa nunca foi confrontado com a realidade.
 *
 * Comparação (endereço do driver antigo → o que REALMENTE existe ali):
 *
 *   antigo REG_OPMODE     0x01 → Cmd0            (colisão fortuita)
 *   antigo REG_INCONFIG0  0x04 → MSCReadDiag0EOT   (InConfig0 é 0x053)
 *   antigo REG_INCONFIG1  0x05 → MSCReadDiag1      (InConfig1 é 0x054)
 *   antigo REG_IGNCONFIG  0x08 → MSCReadConfig1    (IGNConfig é 0x048)
 *   antigo REG_OC_THRESH  0x0A → MSCReadOEConfig   (não existe tal registrador;
 *                                                   sobrecorrente vive em OutConfig*)
 *   antigo REG_SLEW_RATE  0x0B → MSCReadMain       (idem)
 *   antigo REG_VRS_CTRL   0x0C → MSCReadWd1        (VRSConfig0 é 0x049)
 *   antigo REG_VRS_THRESH 0x0D → (VRSConfig1 é 0x04A)
 *   antigo REG_CHIP_ID    0x7E → **Cont3** — registrador de CONTROLE de saídas.
 *                                Não existe "chip ID" nesse endereço; o
 *                                handshake do driver antigo é sem sentido.
 *   antigo REG_OP_STAT    0x34 → OpStat0           ✅ único correto
 *   antigo REG_DIAG_OUT0..3 0x38..0x3B → FWDStat1/TECStat/WdStat0/WdStat1
 *                                (OutDiag0..4 são 0x025..0x029)
 *   antigo REG_WD_CTRL    0x56 → InConfig3
 *   antigo REG_WD_TRIG    0x57 → **DDConfig0** — ou seja, o "trigger de watchdog"
 *                                do driver antigo escrevia 0x01 no registrador de
 *                                direct-drive, ligando O1DD por acidente a cada
 *                                poll de diagnóstico.
 *
 * Registradores endereçados por word ("The registers are addressed wordwise").
 */

#include <cstdint>

namespace ems::hal::tle {

// ─── Comando ────────────────────────────────────────────────────────────────
constexpr uint16_t CMD0            = 0x001u;  ///< reset 0x00
constexpr uint16_t WWD_SERVICE_CMD = 0x015u;
constexpr uint16_t FWD_RESP_CMD    = 0x016u;
constexpr uint16_t CMD_SR          = 0x01Au;
constexpr uint16_t CMD_OE          = 0x01Cu;  ///< enable central das saídas
constexpr uint16_t CMD_LOCK        = 0x01Eu;

// ─── Diagnóstico ────────────────────────────────────────────────────────────
constexpr uint16_t DIAG0     = 0x020u;
constexpr uint16_t DIAG1     = 0x021u;
constexpr uint16_t VRS_DIAG0 = 0x022u;  ///< VRSG / VRSB / VRSOL (short, open-load)
constexpr uint16_t VRS_DIAG1 = 0x023u;  ///< VRSD — leitura ADC da entrada VR
constexpr uint16_t COM_DIAG  = 0x024u;
constexpr uint16_t OUT_DIAG0 = 0x025u;
constexpr uint16_t OUT_DIAG1 = 0x026u;
constexpr uint16_t OUT_DIAG2 = 0x027u;
constexpr uint16_t OUT_DIAG3 = 0x028u;
constexpr uint16_t OUT_DIAG4 = 0x029u;
constexpr uint16_t PPOV_DIAG = 0x02Au;
constexpr uint16_t BRI_DIAG0 = 0x02Bu;
constexpr uint16_t BRI_DIAG1 = 0x02Cu;
constexpr uint16_t IGN_DIAG  = 0x02Du;
constexpr uint16_t WD_DIAG   = 0x02Eu;

// ─── Status ─────────────────────────────────────────────────────────────────
constexpr uint16_t OP_STAT0  = 0x034u;
constexpr uint16_t OP_STAT1  = 0x035u;
constexpr uint16_t WWD_STAT  = 0x036u;  ///< reset 0x30
constexpr uint16_t FWD_STAT0 = 0x037u;  ///< reset 0x30
constexpr uint16_t FWD_STAT1 = 0x038u;  ///< reset 0x30
constexpr uint16_t TEC_STAT  = 0x039u;  ///< reset 0x30
constexpr uint16_t WD_STAT0  = 0x03Au;
constexpr uint16_t WD_STAT1  = 0x03Bu;

// ─── Configuração ───────────────────────────────────────────────────────────
constexpr uint16_t OUT_CONFIG0 = 0x040u;  ///< reset 0xFF
constexpr uint16_t OUT_CONFIG1 = 0x041u;  ///< reset 0x3F
constexpr uint16_t OUT_CONFIG2 = 0x042u;  ///< reset 0x3F
constexpr uint16_t OUT_CONFIG3 = 0x043u;  ///< reset 0x30
constexpr uint16_t OUT_CONFIG4 = 0x044u;  ///< reset 0x3F
constexpr uint16_t OUT_CONFIG5 = 0x045u;  ///< reset 0x3F
constexpr uint16_t BRI_CONFIG0 = 0x046u;  ///< modo/freewheeling das meias-pontes
constexpr uint16_t BRI_CONFIG1 = 0x047u;
constexpr uint16_t IGN_CONFIG  = 0x048u;
constexpr uint16_t VRS_CONFIG0 = 0x049u;  ///< VRSPV / VRSPT / VRSF
constexpr uint16_t VRS_CONFIG1 = 0x04Au;  ///< VRSI_SC / VRSM / VRSDIAGM
constexpr uint16_t VRS_CONFIG2 = 0x04Bu;  ///< VRSI_OL / VRSI_ADC
constexpr uint16_t OP_CONFIG0  = 0x04Eu;  ///< reset 0x09
constexpr uint16_t COM_CONFIG0 = 0x04Fu;  ///< reset 0xA4
constexpr uint16_t COM_CONFIG1 = 0x050u;  ///< reset 0x0D
constexpr uint16_t IN_CONFIG0  = 0x053u;  ///< atribuição IN9..IN12 → OUT5..OUT24
constexpr uint16_t IN_CONFIG1  = 0x054u;
constexpr uint16_t IN_CONFIG2  = 0x055u;
constexpr uint16_t IN_CONFIG3  = 0x056u;
constexpr uint16_t DD_CONFIG0  = 0x057u;  ///< O1DD..O8DD
constexpr uint16_t DD_CONFIG1  = 0x058u;
constexpr uint16_t DD_CONFIG2  = 0x059u;
constexpr uint16_t DD_CONFIG3  = 0x05Au;  ///< IGN1DD..IGN4DD
constexpr uint16_t OE_CONFIG0  = 0x05Bu;  ///< O1E..O8E
constexpr uint16_t OE_CONFIG1  = 0x05Cu;
constexpr uint16_t OE_CONFIG2  = 0x05Du;
constexpr uint16_t OE_CONFIG3  = 0x05Eu;
constexpr uint16_t WWD_CONFIG0 = 0x05Fu;  ///< reset 0xFF
constexpr uint16_t WWD_CONFIG1 = 0x060u;  ///< reset 0x77
constexpr uint16_t FWD_CONFIG  = 0x061u;  ///< reset 0xF7
constexpr uint16_t TEC_CONFIG  = 0x062u;  ///< reset 0x77
constexpr uint16_t WD_CONFIG0  = 0x063u;  ///< reset 0x47
constexpr uint16_t WD_CONFIG1  = 0x064u;  ///< reset 0x03

// ─── Controle de saída (via SPI, alternativa ao direct drive) ───────────────
constexpr uint16_t CONT0 = 0x07Bu;
constexpr uint16_t CONT1 = 0x07Cu;
constexpr uint16_t CONT2 = 0x07Du;
constexpr uint16_t CONT3 = 0x07Eu;

// ─── DDConfig0 (0x057): bit N-1 = OutputN direct drive ──────────────────────
// Tab. 24: IN1..IN4 → OUT1..OUT4 (injetores, atribuição fixa);
//          IN9..IN12 → OUT5..OUT24 (só 4 saídas podem ser diretas).
constexpr uint8_t O1DD = 1u << 0;  ///< INJ1
constexpr uint8_t O2DD = 1u << 1;  ///< INJ2
constexpr uint8_t O3DD = 1u << 2;  ///< INJ3
constexpr uint8_t O4DD = 1u << 3;  ///< INJ4
constexpr uint8_t O5DD = 1u << 4;  ///< VVT escape  (OUT5, 4.5 A, clamp activo)
constexpr uint8_t O6DD = 1u << 5;  ///< VVT admissão (OUT6, 4.5 A, clamp activo)
constexpr uint8_t O7DD = 1u << 6;
constexpr uint8_t O8DD = 1u << 7;

/// Direct drive dos 4 injectores + 2 solenoides de VVT.
constexpr uint8_t DD_CONFIG0_OPENEMS = O1DD | O2DD | O3DD | O4DD | O5DD | O6DD;

// ─── DDConfig3 (0x05A): bits 0..3 = IGN1..IGN4 direct drive ────────────────
constexpr uint8_t IGN1DD = 1u << 0;
constexpr uint8_t IGN2DD = 1u << 1;
constexpr uint8_t IGN3DD = 1u << 2;
constexpr uint8_t IGN4DD = 1u << 3;

/// Direct drive das 4 bobinas.
constexpr uint8_t DD_CONFIG3_OPENEMS = IGN1DD | IGN2DD | IGN3DD | IGN4DD;

// ─── OEConfig0 (0x05B): bit N-1 = OutputN enable ───────────────────────────
// ⚠️ Estes bits são RESETADOS pela função de protecção do canal. Após falha, a
// sequência do datasheet (cap. 9.2) é obrigatória: ler diagnóstico, ler outra
// vez para confirmar que a falha não persiste, re-armar o bit, então ligar.
constexpr uint8_t O1E = 1u << 0;
constexpr uint8_t O2E = 1u << 1;
constexpr uint8_t O3E = 1u << 2;
constexpr uint8_t O4E = 1u << 3;
constexpr uint8_t O5E = 1u << 4;
constexpr uint8_t O6E = 1u << 5;
constexpr uint8_t O7E = 1u << 6;
constexpr uint8_t O8E = 1u << 7;

constexpr uint8_t OE_CONFIG0_OPENEMS = O1E | O2E | O3E | O4E | O5E | O6E;

// ─── VRSConfig1 (0x04A) ────────────────────────────────────────────────────
// [7:4] VRSI_SC — corrente do diagnóstico short-to-GND/Bat
// [3:2] VRSM    — modo de detecção
// [1:0] VRSDIAGM— modo de diagnóstico
constexpr uint8_t VRSM_SHIFT = 2u;
constexpr uint8_t VRSM_AUTO      = 0u << VRSM_SHIFT;  ///< **reset value** — o melhor modo já é o default
constexpr uint8_t VRSM_SEMI_AUTO = 1u << VRSM_SHIFT;  ///< datasheet: "less accurate than the auto mode"
constexpr uint8_t VRSM_MANUAL    = 2u << VRSM_SHIFT;
constexpr uint8_t VRSM_HALL      = 3u << VRSM_SHIFT;

constexpr uint8_t VRSDIAGM_NORMAL   = 0u;  ///< detecção normal (operação)
constexpr uint8_t VRSDIAGM_SHORT    = 1u;  ///< diagnóstico short-to-GND / short-to-Bat
constexpr uint8_t VRSDIAGM_OPENLOAD = 2u;  ///< diagnóstico open-load
constexpr uint8_t VRSDIAGM_ADC      = 3u;  ///< medida ADC da tensão de entrada

/// CKP: VR em auto-detecção, diagnóstico em modo normal.
/// Nota: em auto, escritas a VRSPV/VRSPT (VRSConfig0) são ignoradas pelo CI.
constexpr uint8_t VRS_CONFIG1_OPENEMS = VRSM_AUTO | VRSDIAGM_NORMAL;

// ─── Ainda POR VERIFICAR no datasheet antes de usar ────────────────────────
// - OutConfig0..5: limiares de sobrecorrente e slew rate por canal.
// - InConfig0..3 : atribuição concreta de IN9..IN12 às saídas escolhidas
//                  (bomba, ventoinha, VVT×2).
// - BriConfig0/1 : só se as meias-pontes forem usadas (não são, no plano v1).
// - Cmd0 / CmdOE : sequência exacta de enable central.
// - Watchdog     : WWDConfig0/1, FWDConfig, WDConfig0/1 e o protocolo de
//                  serviço (janela vs pergunta-resposta). O driver antigo
//                  "alimentava" o watchdog escrevendo em DDConfig0.

}  // namespace ems::hal::tle
