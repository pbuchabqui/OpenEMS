#pragma once
/**
 * @file encoder_sync.h
 * @brief Validação de flancos CMP e detecção de staleness para o rastreador
 *        de fase do fork MT6835/TIM2 (ver ecu_sched_encoder_heartbeat.cpp e
 *        docs/dev/mt6835_encoder_fork.md).
 *
 * Funções puras — parâmetros já lidos, sem MMIO directo aqui — mesma
 * disciplina de host-testabilidade do resto do fork. Substitui, no domínio
 * de posição absoluta, os dois gates que o caminho CKP aplica a cada flanco
 * CMP (ckp.cpp:1046-1124): aqui colapsam num só, porque em contagens
 * absolutas "o span é um múltiplo plausível de 32768" e "a posição mod 16384
 * bate com a referência anterior" são a MESMA afirmação — um múltiplo de
 * 32768 é necessariamente côngruo mod 16384. O CKP precisa dos dois porque o
 * primeiro gate ali é temporal (período reconstruído, não posição) — aqui
 * ambas as entradas já são posição absoluta do mesmo contador de hardware,
 * não há segunda fonte de informação independente a validar.
 *
 * O CMP é 1 lóbulo Hall por volta de came = 1 flanco aceite por 720° de
 * cambota = 32768 counts TIM2 fixos (verificado directamente em
 * ckp.cpp:1046-1063, `expected = 2 × kTeethPositionsPerRev × período` — não
 * confundir com uma memória de sessão anterior que descrevia um esquema de
 * 2 flancos/720° com espaçamento alternado; esse esquema não existe no
 * código actual, ver docs/dev/mt6835_encoder_fork.md).
 *
 * Ao contrário do CKP, este span NÃO depende de RPM: é uma medição de
 * posição pura entre dois pontos do mesmo contador absoluto, não um período
 * reconstruído a partir de contagem de dentes — por isso não há aqui um
 * equivalente ao "±25% normal / ±50% baixo RPM" do CKP. A tolerância abaixo
 * é um orçamento de folga MECÂNICA (corrente/correia de distribuição, folga
 * de engrenagem) — não ruído electrónico: o filtro digital do TIM3 (N=8 @
 * fDTS/8, mesmo ajuste do TIM5, hal/stm32h562/timer.cpp) atrasa bem menos de
 * 1 count a qualquer RPM realista (ver a tabela de ω em
 * ecu_sched_encoder_omega.cpp — mesmo no redline o atraso do filtro não
 * chega a 1 count).
 */

#include <cstdint>

namespace ems::drv::encoder_sync {

/// 1 flanco CMP aceite por 720° de cambota (2 voltas de TIM2 a 16384/volta).
constexpr uint32_t kCmpSpanCounts = 32768u;

/// Orçamento de folga mecânica (correia/corrente, folga de engrenagem) — NÃO
/// ruído electrónico (esse é sub-count, ver comentário acima). Valor de
/// arranque conservador (~4,4° de cambota); ajustar em bancada se necessário.
constexpr uint32_t kCmpSpanToleranceCounts = 200u;

/// Múltiplo máximo de kCmpSpanCounts aceite como "flanco(s) perdido(s)" antes
/// de tratar o delta como não-plausível — além disto, mais provável ser lixo
/// do que uma sequência real de flancos perdidos (a via de staleness, não
/// esta, é quem decide sobre ausência prolongada de CMP).
constexpr uint32_t kCmpMaxAcceptedMultiple = 4u;

/// Rejeições temporais CONSECUTIVAS antes de descartar a referência e exigir
/// reestabelecer — mesmo valor do CKP (ckp.cpp kCmpRejectResync).
constexpr uint8_t kCmpRejectResyncThreshold = 3u;

/// Heartbeats (1×/revolução) sem um flanco CMP aceite antes do fallback
/// FULL_SYNC→HALF_SYNC — mesmos valores do CKP (kMaxRevsWithoutCmp/Bench),
/// mapeamento directo porque aqui 1 heartbeat já É 1 revolução.
constexpr uint16_t kMaxHeartbeatsWithoutCmp      = 6u;
constexpr uint16_t kMaxHeartbeatsWithoutCmpBench = 60u;

struct CmpEdgeResult {
    bool    accepted;       ///< passou o gate de span
    uint8_t multiple;       ///< N aceite (1=normal; 2+=flanco(s) perdido(s)); 0 se rejeitado
    uint8_t reject_streak;  ///< streak actualizado após esta avaliação
    bool    streak_resync;  ///< true se o streak atingiu o limite (chamador deve descartar a referência)
};

/// Avalia um novo flanco CMP contra o anterior aceite.
/// - cmp_angle_now: TIM2 raw count (32-bit) no instante do novo flanco.
/// - has_prev: false no primeiro flanco após boot/perda — arma referência,
///   não valida (mesma semântica do CKP: primeiro flanco só arma timestamp).
/// - prev_cmp_angle: TIM2 raw count do último flanco aceite.
/// - reject_streak_in: streak actual de rejeições consecutivas.
CmpEdgeResult evaluate_cmp_edge(uint32_t cmp_angle_now, bool has_prev,
                                uint32_t prev_cmp_angle,
                                uint8_t reject_streak_in) noexcept;

/// true se o número de heartbeats desde o último flanco CMP aceite excedeu o
/// limite (fallback FULL_SYNC→HALF_SYNC) — bench_mode reutiliza
/// ems::drv::sensors_is_bench_mode() no chamador, não decidido aqui.
bool staleness_exceeded(uint32_t heartbeats_since_accepted, bool bench_mode) noexcept;

/// Saúde do sensor MT6835 — separado do gate de fase/CMP acima porque tem
/// origem diferente (poll periódico de mt6835_read_angle_raw21(),
/// atrás de MT6835_HW_PRESENT, não a cada heartbeat) e cadência diferente
/// (~100ms, não 1×/revolução). Default true: sem hardware populado
/// (MT6835_HW_PRESENT=0) o poll nunca corre e este flag nunca é escrito —
/// ausência de hardware não deve ler como falha de sensor (ver
/// docs/dev/mt6835_encoder_fork.md). O heartbeat (ecu_sched_encoder_heartbeat.cpp)
/// lê isto a cada tick para compor o SyncState publicado; só um escritor
/// (o poll de 100ms em main_stm32.cpp) evita que dois publicadores
/// independentes de CkpSnapshot se pisem.
void set_health_ok(bool ok) noexcept;
bool health_ok() noexcept;

// ── Mitigação de drift TIM2(AB) vs. MT6835(SPI) ───────────────────────────
// docs/dev/mt6835_encoder_fork.md, "Mitigação de drift silencioso". Três
// camadas: (1) blindagem física do wiring, fora do escopo deste header;
// (2) correção contínua de software (ecu_sched_encoder_phase_correction_update(),
// engine/ecu_sched.h) — nunca escreve em TIM2_CNT/CCR3/CCR4; (3) detecção
// grosseira + corte via DiagnosticManager/limp_gating, abaixo.

/// Distância mínima com sinal de `b` para `a` no anel de 16384 contagens
/// (domínio TIM2, 1 volta), em (-8192, 8192]. Wrap-safe — mesma disciplina do
/// resto do fork (aritmética unsigned + correção de faixa, não módulo signed).
int32_t circular_diff16384(uint32_t a, uint32_t b) noexcept;

/// Comparador de plausibilidade GROSSEIRA entre a contagem TIM2 (crua, mod
/// 16384) e o ângulo absoluto do MT6835 já convertido para a mesma escala
/// (ems::hal::mt6835_angle21_to_tim2_counts()). NÃO é a correção contínua
/// (essa é a Camada 2, engine layer) — esta função só diz se o desvio cabe no
/// orçamento tolerado; o orçamento (escalado por ω) é decidido pelo chamador
/// (ver ecu_sched_encoder_gross_drift_tolerance_counts(), engine/ecu_sched.h).
/// Deliberadamente insensível a drift fino — esse já é absorvido pela Camada 2.
bool evaluate_angle_plausibility(uint32_t tim2_counts_mod16384,
                                  uint32_t spi_counts_mod16384,
                                  uint32_t tolerance_counts) noexcept;

// ── Correção de drift via Z (índice) do MT6835 ────────────────────────────
// docs/dev/mt6835_encoder_fork.md, "Correção de drift via Z". Espelha
// evaluate_cmp_edge() acima, mas o span esperado é 1 volta de TIM2 (Z
// dispara 1×/volta, sempre na mesma posição física do ímã), não 2 voltas
// como o CMP. Ao contrário do CMP (referência mecânica independente, folga
// de corrente/correia), Z sai do MESMO encoder que AB — a tolerância aqui
// é orçamento de RUÍDO/glitch de captura (filtro TIM3 + jitter), não folga
// mecânica; por isso um valor bem mais apertado que kCmpSpanToleranceCounts.

/// 1 flanco Z aceite por volta de TIM2 (mesma posição física do ímã a cada
/// volta, por construção do sensor).
constexpr uint32_t kZSpanCounts = 16384u;

/// Tolerância de span entre flancos Z consecutivos — ordem de grandeza
/// recomendada pela mesa de powertrain (±16 a ±32 counts); usamos o teto
/// dessa faixa para não gerar rejeições espúrias antes de ter dado real de
/// bancada.
constexpr uint32_t kZSpanToleranceCounts = 32u;

/// Múltiplo máximo aceite como "flanco(s) Z perdido(s)" antes de tratar como
/// não-plausível — mesmo papel de kCmpMaxAcceptedMultiple. Correção só é
/// aplicada pelo chamador quando multiple==1 (ambiguidade com múltiplas
/// voltas normais acima disso — ver ecu_sched_encoder_heartbeat_z_tick()).
constexpr uint32_t kZMaxAcceptedMultiple = 4u;

/// Rejeições consecutivas antes de descartar a referência de Z — mesmo valor
/// do CMP.
constexpr uint8_t kZRejectResyncThreshold = 3u;

struct ZEdgeResult {
    bool    accepted;
    uint8_t multiple;       ///< N aceite (1=normal; 2+=flanco(s) perdido(s)); 0 se rejeitado
    uint8_t reject_streak;
    bool    streak_resync;  ///< true se o chamador deve descartar a referência (e o alvo fixo)
};

/// Avalia um novo flanco Z contra o anterior aceite. Mesma semântica de
/// has_prev/reject_streak_in de evaluate_cmp_edge() acima.
ZEdgeResult evaluate_z_edge(uint32_t z_angle_now, bool has_prev,
                            uint32_t prev_z_angle,
                            uint8_t reject_streak_in) noexcept;

/// Reação ao poll de 100 ms do MT6835 (main_stm32.cpp), chamada só depois de
/// uma leitura SPI bem-sucedida (health_ok()==true já garantido pelo
/// chamador antes de invocar isto). Faz, nesta ordem:
///  1. Camada 2 — corrige o offset de software via
///     ecu_sched_encoder_phase_correction_update() (engine/ecu_sched.h).
///  2. Camada 3 — checa os bits de status do sensor (overspeed/weak-field/
///     undervoltage, kStatusOverspeedBit/... em hal/mt6835_regs.h) e a
///     plausibilidade grosseira; escada de severidade WARNING→CRITICAL via
///     DiagnosticCode::CKP_SIGNAL_FAULT (engine/diagnostic_manager.h) — uma
///     falta CRITICAL já corta fuel/ignição via limp_gating (diag_critical),
///     sem nenhuma mudança em limp_gating.cpp.
/// `angle21`/`status`: saída de ems::hal::mt6835_read_angle_raw21().
/// `tim2_raw_now`: TIM2_CNT lido pelo chamador na MESMA amostra que `angle21`
/// (minimiza o próprio skew que a Camada 2 tenta respeitar).
void poll_100ms(uint32_t angle21, uint8_t status, uint32_t tim2_raw_now) noexcept;

#if defined(EMS_HOST_TEST)
/// Reseta o estado de escalada (strikes/active/critical) de poll_100ms() e
/// health_ok() para true. Não mexe em DiagnosticManager — o teste chama
/// DiagnosticManager::init() separadamente, mesmo padrão do resto da suite.
void test_reset(void) noexcept;
#endif

}  // namespace ems::drv::encoder_sync
