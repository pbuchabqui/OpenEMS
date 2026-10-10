#pragma once

// Calibração das estratégias portadas da Siemens MS42 (VVT de admissão com
// fase medida, VBATT filtrada, DFCO por CLT, corte rotativo, gradiente de
// avanço, partida/pós-partida, knock relativo ao ruído, marcha lenta P+FF,
// ganhos do STFT por rpm×carga, ventoinha).
//
// Vive na page0 a partir de kMs42Page0Off, em ordem fixa (ver ms42_cal.cpp),
// guardada por um magic: blob antigo sem o magic → defaults de compilação.
// Os defaults preservam o comportamento anterior sempre que já existia uma
// estratégia equivalente (as novas ficam desligadas até serem calibradas).

#include <cstdint>

namespace ems::engine {

constexpr uint16_t kMs42Page0Off   = 276u;
constexpr uint16_t kMs42Magic      = 0x344Du;  // "M4"
constexpr uint8_t  kMs42BlockVer   = 1u;

constexpr uint8_t kVvtCalPts       = 6u;
constexpr uint8_t kDfcoCltPts      = 4u;
constexpr uint8_t kCrankCalPts     = 7u;
constexpr uint8_t kKnockGainPts    = 4u;
constexpr uint8_t kIdleFfPts       = 8u;   // eixo = iac_clt_axis_x10
constexpr uint8_t kStftGainPts     = 4u;

struct Ms42Cal {
    // ── A1: VVT de admissão (fase medida pelo CMP) ──────────────────────
    uint8_t  vvt_enable;           // 0 = solenoide em repouso (duty 0)
    uint8_t  vvt_kp_x10;           // ‰ duty por 0,1° de erro, ×10
    uint8_t  vvt_ki_x100;          // ‰ duty por 0,1° de erro por 10 ms, ×100
    uint8_t  vvt_hold_duty_pct;    // duty de retenção (base do PI)
    uint16_t vvt_cam_ref_x10;      // ângulo de repouso da borda; 0 = aprende
    int16_t  vvt_min_clt_x10;      // abaixo disto: repouso (óleo frio)
    uint8_t  vvt_max_adv_deg;      // limite de avanço pedido (° virabrequim)
    uint8_t  vvt_rpm_axis[kVvtCalPts];    // rpm/100
    uint8_t  vvt_load_axis[kVvtCalPts];   // kPa
    uint8_t  vvt_target_deg[kVvtCalPts][kVvtCalPts];  // [carga][rpm], ° avanço

    // ── A3: VBATT ────────────────────────────────────────────────────────
    uint8_t  vbatt_filter_shift;   // IIR α = 1/2^n (0 = sem filtro)
    uint16_t vbatt_low_mv;         // DTC baixa (motor a trabalhar); 0 = off
    uint16_t vbatt_high_mv;        // DTC alta; 0 = off

    // ── B4: DFCO por CLT ─────────────────────────────────────────────────
    int16_t  dfco_clt_axis_x10[kDfcoCltPts];
    uint16_t dfco_entry_rpm_x10[kDfcoCltPts];  // todos 0 = escalares antigos
    uint16_t dfco_hyst_rpm_x10;    // saída = entrada − histerese (só c/ curva)
    uint16_t dfco_entry_delay_ms;  // condições estáveis antes de cortar

    // ── B5: corte rotativo de injeção ────────────────────────────────────
    uint8_t  rev_roll_enable;
    uint16_t rev_roll_window_rpm_x10;  // janela abaixo do limite duro

    // ── B6: gradiente do avanço (°×10 por volta; 0 = sem limite) ─────────
    uint8_t  spark_grad_inc_x10;
    uint8_t  spark_grad_dec_x10;

    // ── B7/B8: partida e pós-partida ─────────────────────────────────────
    uint8_t  crank_taper_cycles;   // 0 = off; ciclos até taper_end_pct
    uint8_t  crank_taper_end_pct;
    int16_t  hot_restart_clt_x10;
    uint8_t  hot_restart_pct;      // 100 = sem correção
    uint8_t  crank_baro_enable;    // escala o pulso de partida por baro
    uint8_t  afterstart_by_cycles; // 0 = decai em ms; 1 = em ciclos do motor
    int16_t  crank_clt_axis_x10[kCrankCalPts];
    uint16_t crank_mult_x256[kCrankCalPts];
    uint16_t afterstart_start_x256[kCrankCalPts];
    uint16_t afterstart_ms[kCrankCalPts];
    uint16_t afterstart_cycles[kCrankCalPts];

    // ── C9: knock ────────────────────────────────────────────────────────
    uint8_t  knock_step_x10;
    uint8_t  knock_max_x10;
    uint8_t  knock_recovery_x10;
    uint8_t  knock_clean_cycles;
    uint8_t  knock_rel_enable;     // limiar = ganho(rpm) × ruído do cilindro
    uint8_t  knock_noise_shift;    // EMA do ruído α = 1/2^n
    uint8_t  knock_gain_rpm_axis[kKnockGainPts];  // rpm/100
    uint8_t  knock_gain_x10[kKnockGainPts];

    // ── C10: marcha lenta ────────────────────────────────────────────────
    uint8_t  idle_kp_x10;          // ‰ lâmina por 10 rpm de erro, ×10; 0 = off
    uint8_t  idle_persist;         // aplica o aprendido retido (NVM, S20)
    uint16_t idle_ff_x10[kIdleFfPts];  // abertura-base por CLT; 0 = off
    uint16_t cat_heat_rpm_x10;     // acréscimo ao alvo após a partida
    uint16_t cat_heat_s;           // duração do acréscimo

    // ── C11: STFT ────────────────────────────────────────────────────────
    uint8_t  stft_rpm_axis[kStftGainPts];   // rpm/100
    uint8_t  stft_load_axis[kStftGainPts];  // kPa
    uint8_t  stft_gain_pct[kStftGainPts][kStftGainPts];  // 100 = 1,0
    uint16_t stft_min_rpm_x10;     // 0 = sem mínimo

    // ── Ventoinha ────────────────────────────────────────────────────────
    int16_t  fan_on_x10;
    int16_t  fan_off_x10;
};

extern Ms42Cal ms42;

// ── Extensão na page5 (bytes 64–95, antiga curva de warmup) ─────────────
// A page0 não tem mais espaço. Bloco de 32 bytes com magic próprio: sem o
// magic (blob antigo, área zerada ou com a curva removida) → defaults, que
// reproduzem o comportamento anterior.
constexpr uint16_t kMs42ExtPage5Off = 64u;
constexpr uint16_t kMs42ExtLen      = 32u;
constexpr uint16_t kMs42ExtMagic    = 0x354Du;  // "M5"
constexpr uint8_t  kMisfireCalPts   = 4u;

struct Ms42Ext {
    // ── Retardo de aquecimento do catalisador (MS42 0x4F291) ─────────────
    uint8_t  cat_heat_retard_x10;  // 0,1°; decai com cat_heat_s; 0 = off
    int8_t   cat_heat_clt_max_c;   // só abaixo desta CLT (°C)

    // ── Limite de queda do pulso na pós-partida (ip_ti_lgrd_ast__tco) ────
    uint8_t  as_pw_fall_cycles;    // ciclos do motor após a partida; 0 = off
    uint8_t  as_pw_fall_cold_pct;  // queda máx. por ciclo, % do pulso anterior,
    uint8_t  as_pw_fall_hot_pct;   //   no 1º / último ponto de crank_clt_axis

    // ── Limiar de misfire por rpm × MAP (Q8 − 256: 31 = 1,12×) ───────────
    uint8_t  misfire_rpm_axis[kMisfireCalPts];  // rpm/100
    uint8_t  misfire_map_axis[kMisfireCalPts];  // kPa
    uint8_t  misfire_excess_q8[kMisfireCalPts][kMisfireCalPts];  // [MAP][rpm]
};

extern Ms42Ext ms42x;

void ms42_ext_defaults() noexcept;
void ms42_ext_serialize_to_page5(uint8_t* page5, uint16_t len) noexcept;
void ms42_ext_apply_page5(const uint8_t* page5, uint16_t len) noexcept;

void ms42_cal_defaults() noexcept;
// Bytes ocupados na page0 (magic incluído) — para testes e static checks.
uint16_t ms42_cal_page0_len() noexcept;
void ms42_serialize_to_page0(uint8_t* page0, uint16_t len) noexcept;
// Sem o magic/versão → defaults. Valores fora de faixa são saneados.
void ms42_apply_page0(const uint8_t* page0, uint16_t len) noexcept;

// Interpolação linear 1-D em eixo u8 (rpm/100 ou kPa) com valores u8.
uint16_t ms42_interp_u8(const uint8_t* axis, const uint8_t* vals, uint8_t n,
                        uint16_t x) noexcept;

// Bilinear numa tabela n×n de u8 [y][x] (linhas = y_axis), eixos u8.
uint16_t ms42_interp_u8_2d(const uint8_t* x_axis, const uint8_t* y_axis,
                           const uint8_t* table, uint8_t n,
                           uint16_t x, uint16_t y) noexcept;

}  // namespace ems::engine
