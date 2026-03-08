// =============================================================================
// engine/cycle_sched.cpp — Agendamento por ângulo de virabrequim
//
// Substitui o loop de 5 ms para agendamento de injeção/ignição por um
// mecanismo disparado por dente do CKP (via schedule_on_tooth), eliminando
// o atraso de até 5 ms entre o recálculo e o disparo real dos eventos.
//
// ARQUITETURA (duplo buffer):
//   Loop background:
//     Calcula PW e ângulos para todos os cilindros → escreve em g_pending[cyl]
//
//   ISR CKP (a cada dente, via schedule_on_tooth):
//     Encaminha para o backend unificado ecu_sched.
//
// LIMITAÇÃO CONHECIDA — EOI e ignição:
//   O FTM0 opera a 60 MHz com contador de 16 bits (overflow ~1,09 ms).
//   Pulsos de injeção > 1 ms sofrem erro de temporização no INJ_CLR porque
//   o comparador FTM0 dispara na PRIMEIRA ocorrência do valor alvo.
//   Solução futura: usar PIT em modo one-shot ou FTM0 com prescaler 128.
//   O agendamento de ignição (dwell + faísca) requer a mesma correção de HAL.
// =============================================================================

#include "engine/cycle_sched.h"
#include "engine/ecu_sched.h"
#include "engine/ign_calc.h"
#include "drv/ckp.h"

namespace ems::engine {
#if defined(__GNUC__)
void ecu_sched_on_tooth_hook(const ems::drv::CkpSnapshot& snap) noexcept __attribute__((weak));
#else
void ecu_sched_on_tooth_hook(const ems::drv::CkpSnapshot& snap) noexcept;
#endif
}

namespace {

static constexpr uint8_t kNCyl = 4u;

// Dentes de lead do trigger antes do SOI
// (tempo necessário para a ISR processar o evento)
static constexpr uint8_t kInjLeadTeeth = 4u;

// SOI N dentes antes do TDC compressão
static constexpr uint8_t kSoiTeeth = 10u;

// Total de dentes antes do TDC no qual o trigger de injeção ocorre
static constexpr uint8_t kTrigLeadTeeth = kSoiTeeth + kInjLeadTeeth;

// Dentes de lead do trigger de ignição antes do evento alvo.
// LIMITAÇÃO: mesmo que a injeção, o delta em ticks excede 65535 do FTM0
// a baixas rotações (< ~1500 RPM). Solução futura: PIT one-shot ou prescaler 128.
static constexpr uint8_t kIgnLeadTeeth = 4u;

// kToothMillideg: ângulo inter-dente em miligraus para roda 60-2.
// Roda 60-2: 60 posições × 6°/posição → 1 dente normal = 6,0° = 6000 mg.
// Deve coincidir com kToothAngleX1000 em ckp.cpp (ambos = 6000).
static constexpr uint16_t kToothMillideg = 6000u;

static bool g_enabled = false;

struct TriggerPoint {
    uint8_t tooth;   // 0..57
    bool    phase;   // false = 1ª rev, true = 2ª rev (cam phase_A)
};

// Dente-gatilho de injeção por slot de disparo (0..3 em firing_order)
static TriggerPoint g_inj_trigger[kNCyl];

// Dente-gatilho de ignição: SET = kIgnLeadTeeth antes do dwell_start,
//                            CLR = kIgnLeadTeeth antes da faísca.
// Actualizados em cycle_sched_update() dentro da janela valid=false.
static TriggerPoint g_ign_set_trigger[kNCyl];
static TriggerPoint g_ign_clr_trigger[kNCyl];

// Parâmetros pré-calculados por cilindro (cyl_idx 0..3).
// Escrito pelo loop de background; lido pela ISR do CKP.
struct CylPending {
    uint32_t pw_ticks;       // FTM0 ticks de largura de pulso
    uint16_t dead_ticks;     // FTM0 ticks de dead-time
    uint16_t soi_abs_x10;    // ângulo absoluto SOI em graus × 10 (0-7199)
    uint16_t spark_abs_x10;  // ângulo absoluto faísca em graus × 10 (reservado)
    uint16_t dwell_abs_x10;  // ângulo absoluto início de dwell em graus × 10 (reservado)
    bool     valid;
};

static volatile CylPending g_pending[kNCyl];
static volatile uint32_t g_last_pw_ticks = 1000u;
static volatile uint16_t g_last_dead_ticks = 100u;
static volatile uint16_t g_last_soi_lead_x10 = 100u;
static volatile int16_t g_last_advance_x10 = 100;
static volatile uint16_t g_last_dwell_x10 = 1000u;

// Subtrai N dentes do ângulo absoluto no domínio de 720°.
// 1 dente = 360/60 = 6°. N dentes = N×6°.
static uint16_t sub_teeth(uint16_t abs_deg, uint8_t teeth) noexcept {
    const uint16_t offset = static_cast<uint16_t>(
        (static_cast<uint32_t>(teeth) * 360u) / 60u);
    return (abs_deg >= offset)
        ? static_cast<uint16_t>(abs_deg - offset)
        : static_cast<uint16_t>(abs_deg + 720u - offset);
}

// Converte ângulo absoluto (0-719°) → (tooth, phase).
static TriggerPoint angle_to_tp(uint16_t abs_deg) noexcept {
    const bool    phase = (abs_deg >= 360u);
    const uint16_t local = abs_deg % 360u;
    const uint8_t  tooth = static_cast<uint8_t>(
        (static_cast<uint32_t>(local) * 60u) / 360u);
    return TriggerPoint{tooth, phase};
}

}  // namespace

// =============================================================================
// API pública — engine layer
// =============================================================================
namespace ems::engine {

void cycle_sched_init() noexcept {
    g_enabled = false;
    for (uint8_t slot = 0u; slot < kNCyl; ++slot) {
        const uint8_t  cyl_idx  = static_cast<uint8_t>(firing_order[slot] - 1u);
        const uint16_t tdc_deg  = cylinder_offset_deg[cyl_idx];
        const uint16_t trig_deg = sub_teeth(tdc_deg, kTrigLeadTeeth);
        g_inj_trigger[slot]     = angle_to_tp(trig_deg);
        g_ign_set_trigger[slot] = TriggerPoint{0u, false};
        g_ign_clr_trigger[slot] = TriggerPoint{0u, false};
        g_pending[cyl_idx].valid = false;
    }
}

void cycle_sched_enable(bool en) noexcept {
    g_enabled = en;
}

void cycle_sched_update(uint32_t pw_ticks,
                        uint16_t dead_ticks,
                        uint16_t soi_lead_x10,
                        int16_t  advance_x10,
                        uint16_t dwell_x10) noexcept {
    // CYC-04: proteção de saturação para pw_ticks.
    // FTM0 opera com contador de 16 bits (PS=8, 15 ticks/µs).
    // Pulsos > 65535 ticks (≈4,369 ms) causam o comparador INJ_OFF a disparar
    // na PRIMEIRA correspondência após o wrap, injectando por uma revolução inteira.
    // Mitigação interim: saturar a 65000 ticks (≈4,333 ms) e incrementar o
    // contador de saturação de calibração para diagnóstico.
    // Solução definitiva pendente: migrar INJ_OFF para PIT one-shot (ver LIMITAÇÃO
    // CONHECIDA no topo deste ficheiro).
    static constexpr uint32_t kMaxPwTicks = 65000u;
    if (pw_ticks > kMaxPwTicks) {
        pw_ticks = kMaxPwTicks;
        ++g_calibration_clamp_count;
    }

    g_last_pw_ticks = pw_ticks;
    g_last_dead_ticks = dead_ticks;
    g_last_soi_lead_x10 = soi_lead_x10;
    g_last_advance_x10 = advance_x10;
    g_last_dwell_x10 = dwell_x10;

    for (uint8_t slot = 0u; slot < kNCyl; ++slot) {
        const uint8_t  cyl_idx  = static_cast<uint8_t>(firing_order[slot] - 1u);
        const uint16_t tdc_x10  = static_cast<uint16_t>(cylinder_offset_deg[cyl_idx] * 10u);

        // SOI: TDC - soi_lead (domínio 0-7199, graus × 10)
        const uint16_t soi_x10 = static_cast<uint16_t>(
            (tdc_x10 + 7200u - soi_lead_x10) % 7200u);

        // Faísca: TDC - advance (advance pode ser negativo = retardo)
        const int32_t spark_raw = static_cast<int32_t>(tdc_x10)
                                  - static_cast<int32_t>(advance_x10);
        const uint16_t spark_x10 = static_cast<uint16_t>(
            ((spark_raw % 7200) + 7200) % 7200);

        // Dwell start: antes da faísca pelo ângulo de dwell
        const uint16_t dwell_x10_abs = static_cast<uint16_t>(
            (spark_x10 + 7200u - dwell_x10) % 7200u);

        // Protocolo de publicação: invalida primeiro, escreve campos, valida por último.
        // volatile garante ordenação de acesso sem barreiras de memória explícitas
        // (suficiente em single-core Cortex-M4 sem cache de escrita fora de ordem).
        g_pending[cyl_idx].valid        = false;
        g_pending[cyl_idx].pw_ticks     = pw_ticks;
        g_pending[cyl_idx].dead_ticks   = dead_ticks;
        g_pending[cyl_idx].soi_abs_x10  = soi_x10;
        g_pending[cyl_idx].spark_abs_x10 = spark_x10;
        g_pending[cyl_idx].dwell_abs_x10 = dwell_x10_abs;

        // Dentes-gatilho de ignição (actualizados dentro da janela valid=false).
        // Se a ISR ler g_ign_{set,clr}_trigger e encontrar valid=false, irá ignorar.
        const uint16_t dwell_deg = dwell_x10_abs / 10u;
        const uint16_t spark_deg = spark_x10       / 10u;
        g_ign_set_trigger[slot] = angle_to_tp(sub_teeth(dwell_deg, kIgnLeadTeeth));
        g_ign_clr_trigger[slot] = angle_to_tp(sub_teeth(spark_deg, kIgnLeadTeeth));

        g_pending[cyl_idx].valid        = true;
    }
}

#if defined(EMS_HOST_TEST)
void cycle_sched_test_reset() noexcept {
    g_enabled = false;
    for (uint8_t i = 0u; i < kNCyl; ++i) {
        g_pending[i].valid      = false;
        g_ign_set_trigger[i]    = TriggerPoint{0u, false};
        g_ign_clr_trigger[i]    = TriggerPoint{0u, false};
    }
}

bool cycle_sched_test_trigger(uint8_t slot, uint8_t& tooth, bool& phase) noexcept {
    if (slot >= kNCyl) { return false; }
    tooth = g_inj_trigger[slot].tooth;
    phase = g_inj_trigger[slot].phase;
    return true;
}

bool cycle_sched_test_ign_set_trigger(uint8_t slot, uint8_t& tooth, bool& phase) noexcept {
    if (slot >= kNCyl) { return false; }
    tooth = g_ign_set_trigger[slot].tooth;
    phase = g_ign_set_trigger[slot].phase;
    return true;
}

bool cycle_sched_test_ign_clr_trigger(uint8_t slot, uint8_t& tooth, bool& phase) noexcept {
    if (slot >= kNCyl) { return false; }
    tooth = g_ign_clr_trigger[slot].tooth;
    phase = g_ign_clr_trigger[slot].phase;
    return true;
}
#endif

}  // namespace ems::engine

// =============================================================================
// Override do símbolo fraco schedule_on_tooth (drv/ckp.cpp)
// Executado dentro da ISR do CKP (prioridade 1 — mais alta do sistema).
// Deve ser determinístico e de execução rápida.
// =============================================================================
namespace ems::drv {

void schedule_on_tooth(const CkpSnapshot& snap) noexcept {
    if (ems::engine::ecu_sched_on_tooth_hook != nullptr) {
        ems::engine::ecu_sched_on_tooth_hook(snap);
    }
}

}  // namespace ems::drv

// ============================================================================
// Angular Execution Loop - Forced Update Support
// ============================================================================

// ============================================================================
// cycle_sched_force_update / cycle_sched_update_with_calculated
// Estas funções são chamadas APENAS do loop background (nunca da ISR).
// Ver CYC-01 na revisão de código — ciclo_sched_force_update foi removido
// do caminho da ISR em hal/ftm.cpp para evitar race condition.
// ============================================================================

namespace ems::engine {

void cycle_sched_force_update() noexcept {
    // Chamada apenas do loop background (5 ms). Thread-safe porque o loop
    // background não é preemptido por si mesmo e g_pending usa protocolo
    // valid=false → escreve → valid=true para proteção face à ISR CKP.
    cycle_sched_update(
        g_last_pw_ticks,
        g_last_dead_ticks,
        g_last_soi_lead_x10,
        g_last_advance_x10,
        g_last_dwell_x10);
}

bool cycle_sched_should_update_on_gap() noexcept {
    return g_enabled;
}

void cycle_sched_update_with_calculated(uint32_t pw_ticks,
                                        uint16_t dead_ticks,
                                        uint16_t soi_lead_x10,
                                        int16_t  advance_x10,
                                        uint16_t dwell_x10) noexcept {
    cycle_sched_update(pw_ticks, dead_ticks, soi_lead_x10, advance_x10, dwell_x10);
}

// CYC-03: cycle_sched_acceleration_compensation() removida.
// Era um stub sem implementação que criava falsa expectativa de comportamento.
// Compensação de aceleração será implementada como feature real quando necessário,
// com algoritmo documentado e testado (derivada de RPM entre gaps + ajuste de advance).

}  // namespace ems::engine
