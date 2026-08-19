#pragma once

#include <cstdint>

namespace ems::engine::cfg {

inline constexpr uint8_t kCylinderCount = 4u;
inline constexpr uint16_t kDisplacementCc = 2000u;
inline constexpr uint16_t kInjectorFlowCcMin = 450u;

// E30: lambda 1.00 equivale aproximadamente a AFR 13.0.
inline constexpr uint16_t kStoichAfrX100 = 1300u;
inline constexpr uint16_t kFuelDensityMgPerCc = 755u;
inline constexpr uint16_t kAirDensityMgPerCcX1000 = 1184u;

inline constexpr uint16_t kMapRefBarX100 = 100u;
// EOI: ângulo (° BTDC de combustão) em que a injecção TERMINA (SOI é
// derivado para trás, SOI = EOI − PW°). 2026-08-16: deixou de ser um único
// valor de compilação aqui — passou a uma tabela 2D RPM×CLT
// (eoi_rpm_axis_x10/eoi_clt_axis_x10/eoi_table_deg, engine/calibration.h),
// substituindo o antigo default_eoi_lead_deg+blend 1D só-RPM.

// Default para g_eng_cfg.trigger_tooth0_engine_deg (usado antes de NVM válida).
// NOTA: este valor de compilação é apenas o default inicial.
// O scheduler usa g_eng_cfg.trigger_tooth0_engine_deg em runtime (NVM/UART).
//
// Como calibrar:
//   1. Com dial indicator no cil.0, marcar TDC na polia.
//   2. Ligar oscilóscopo ao pino CKP (PA0). Identificar o dente 0
//      (primeiro dente após o gap de 3 períodos).
//   3. Medir o ângulo entre o dente 0 e a marca de TDC.
//   4. Calcular: trigger_tooth0_engine_deg = (720 - offset_graus_antes_TDC) % 720
//      Exemplo: dente 0 está 84° antes do TDC do cil.0 → valor = 636.
//   5. Escrever via comando UART SET_CONFIG antes do primeiro arranque.
//
// Modo encoder MT6835 (EMS_MT6835_ENCODER=1, hal/board_pinout.h): este mesmo
// campo é REAPROVEITADO com o mesmo papel físico — "que ângulo de motor
// corresponde à posição bruta zero" — só que a posição bruta já não é o
// dente 0 da roda 60-2, é TIM2_CNT==0 (ver ecu_sched_encoder_builders.cpp,
// engine_deg_to_counts_in_rev()). Só a resídua MOD 360 é significativa nesse
// caminho (TIM2 embrulha a cada 16384 contagens = 1 volta, não 720° como o
// campo sugere pelo nome) — um calibrador escrevendo 400 ou 40 produz o
// MESMO timing em modo encoder. O default abaixo é um artefacto do bring-up
// da roda 60-2 e TEM de ser remedido (mesmo procedimento, dial indicator +
// leitura de TIM2_CNT em vez do osciloscópio no dente 0) antes de um motor
// girar com uma placa encoder — não é intercambiável entre os dois modos.
inline constexpr uint16_t kTriggerTooth0EngineDeg = 0u;  // MEDIR NO MOTOR REAL

// Campo dedicado ao caminho encoder (EMS_MT6835_ENCODER=1) — decisão do
// utilizador (2026-08-13) de NÃO reaproveitar trigger_tooth0_engine_deg
// (que fica exclusivo do caminho roda-dentada): permite trocar entre Hall e
// encoder na mesma placa sem perder/sobrescrever a calibração do outro modo.
// Mesmo papel físico ("que ângulo de motor corresponde à posição bruta
// zero"), mas domínio de embrulho é 360° (1 volta de TIM2), não 720° como o
// nome de trigger_tooth0_engine_deg sugere — só valores 0-359 fazem sentido
// aqui (ver validação em engine_config.cpp).
//
// Como calibrar (mesmo procedimento físico de trigger_tooth0_engine_deg —
// dial indicator / roda de graus na polia — mas lendo TIM2_CNT em vez de
// osciloscópio no dente 0; ver docs/dev/mt6835_encoder_fork.md,
// "Procedimento de bancada"):
//   1. Colocar o cilindro 1 no PMS de compressão (referência mecânica).
//   2. Nessa posição exacta, chamar ecu_sched_encoder_tdc1_calibrate_from_raw()
//      com o valor cru de tim2_encoder_count() — devolve o valor pronto a
//      escrever aqui (ver ecu_sched.h). Exposto via UI protocol (comando 'X'),
//      não precisa de calcular à mão.
//   3. Escrever o valor devolvido neste campo e fazer burn.
inline constexpr uint16_t kEncoderTdc1OriginDeg = 0u;  // MEDIR NO MOTOR REAL

// Fase CMP do caminho encoder (EMS_MT6835_ENCODER=1) — qual das duas janelas
// de 360° do ciclo de 720° o flanco do CMP (Hall na came, PC6/TIM3) marca.
// Substitui EMS_MT6835_CMP_PHASE_CALIBRATED/EMS_MT6835_CMP_PHASE_VALUE
// (board_pinout.h, removidos — eram compile-time, sem persistência nem
// interface) por um campo NVM calibrável ao vivo, mesmo espírito de
// encoder_tdc1_origin_deg acima. 3 valores em vez de "calibrado + valor"
// separados: evita a combinação inválida "calibrado=1, valor=lixo".
//
// Como calibrar (depois de encoder_tdc1_origin_deg já medido — mesma
// referência física, mesma sessão de bancada):
//   1. Rodar o motor por ≥1 ciclo completo de 720° (garante que
//      ems::hal::cmp_edge_count() > 0 — pelo menos um flanco CMP visto).
//   2. Colocar o cilindro 1 no PMS de COMPRESSÃO (não escape — mesma
//      referência mecânica do TDC1).
//   3. Nessa posição exacta, chamar
//      ecu_sched_encoder_cmp_phase_calibrate_from_raw() com
//      tim2_encoder_count() e ems::hal::cmp_angle_snapshot() — devolve o
//      valor pronto a escrever aqui. Exposto via UI protocol (comando 'M'),
//      não precisa de calcular à mão.
//   4. Escrever o valor devolvido neste campo e fazer burn ('b').
inline constexpr uint8_t kCmpPhaseUncalibrated = 0u;  // default — presync sempre (nunca adivinha)
inline constexpr uint8_t kCmpPhaseCalibratedA  = 1u;
inline constexpr uint8_t kCmpPhaseCalibratedB  = 2u;

// Convenção de canal: ECU_CH_IGNn/ECU_CH_INJn = cilindro físico n−1, SEMPRE.
// A ordem de disparo entra apenas via kFiringOrder/cyl_tdc_deg — nunca na
// escolha do canal. Invariante partilhado por Calculate_Sequential_Cycle,
// pares presync (companheiros 0↔3, 2↔1), ign_ch_to_cyl_bit e misfire_encoder.
// kFiringOrder={0,2,3,1} = ordem de ignição física 1-3-4-2.
inline constexpr uint8_t kFiringOrder[kCylinderCount] = {0u, 2u, 3u, 1u};

// CMP reference half: which 360° half the cam rising edge marks.
// Convention: CMP rises at kCmpTooth of the 1st revolution of the 720° pair.
// That revolution is PHASE_A → kCmpRefHalf=0 (snap.phase_A=true).
// Adjust empirically if the physical cam sensor marks the other revolution.
inline constexpr uint8_t kCmpRefHalf = 0u;

constexpr uint16_t cyl_tdc_deg(uint8_t cyl) noexcept {
    // TDC baseado na POSIÇÃO do cilindro na ordem de disparo, não no número.
    // Ex: kFiringOrder={0,2,3,1} (= físicos 1-3-4-2) → cyl 0/físico 1 na
    //     pos 0 (0°), cyl 2/físico 3 na pos 1 (180°), cyl 3/físico 4 na
    //     pos 2 (360°), cyl 1/físico 2 na pos 3 (540°).
    uint8_t pos = 0u;
    for (; pos < kCylinderCount; ++pos) {
        if (kFiringOrder[pos] == cyl) { break; }
    }
    return static_cast<uint16_t>(pos * (720u / kCylinderCount));
}

// =============================================================================
// Runtime-configurable engine parameters (stored in Flash page 0)
// =============================================================================

struct EngineConfigRam {
    uint16_t displacement_cc;
    uint16_t injector_flow_cc_min;
    uint16_t stoich_afr_x100;
    uint16_t map_ref_bar_x100;
    uint16_t trigger_tooth0_engine_deg;
    uint16_t encoder_tdc1_origin_deg;
    uint8_t  cmp_phase_state;  // kCmpPhaseUncalibrated/CalibratedA/CalibratedB
};

// Runtime config — initialized to compile-time defaults at startup.
// Overwritten by engine_config_load() if valid data found in Flash.
extern EngineConfigRam g_eng_cfg;

// Comprimento mínimo de page0_buf exigido por load()/serialize() — cobre o
// bloco contíguo [0-15] (inclui cmp_phase_state em [12], 2026-08-16: já
// não é um byte isolado noutro gap de page0 — ver engine_config.cpp para o
// achado da colisão que motivou isto). Todos os chamadores devem passar
// isto, não um literal.
inline constexpr uint16_t kEngineConfigMinPageLen = 16u;

// Call at boot after nvm_load_calibration(0, page0_buf, 512).
// If page0 magic is valid, populates g_eng_cfg from page0_buf offsets 2-15.
// Otherwise keeps compile-time defaults.
void engine_config_load(const uint8_t* page0_buf, uint16_t len) noexcept;

// Call when UI writes page 0 and requests burn: updates g_eng_cfg from buf.
void engine_config_apply(const uint8_t* page0_buf, uint16_t len) noexcept;

// Validates runtime config: returns false if any value is out of safe range.
bool engine_config_valid(const EngineConfigRam& cfg) noexcept;

// Write current g_eng_cfg into page0_buf at the correct offsets.
// (for reading back via UI protocol)
void engine_config_serialize(uint8_t* page0_buf, uint16_t len) noexcept;

}  // namespace ems::engine::cfg
