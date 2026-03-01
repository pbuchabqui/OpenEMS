// test_cycle_sched.cpp — testes do agendador de ciclo por ângulo de CKP

#include <cstdint>
#include <cstdio>

#define EMS_HOST_TEST 1
#include "engine/cycle_sched.h"
#include "engine/ign_calc.h"
#include "drv/ckp.h"
#include "drv/scheduler.h"

// ── Variáveis de teste exportadas por ckp.cpp e scheduler.cpp ────────────────
extern volatile uint32_t ems_test_ftm3_c0v;
extern volatile uint32_t ems_test_ftm3_c1v;
extern volatile uint32_t ems_test_gpiod_pdir;
extern volatile uint32_t ems_test_fgpio_c_psor;
extern volatile uint32_t ems_test_fgpio_c_pcor;
extern volatile uint32_t ems_test_fgpio_d_psor;
extern volatile uint32_t ems_test_fgpio_d_pcor;

// ── Stubs HAL ────────────────────────────────────────────────────────────────
namespace ems::hal {
static uint16_t g_ftm0_count = 0u;

uint16_t ftm0_count() noexcept { return g_ftm0_count; }
void ftm0_set_compare(uint8_t, uint16_t) noexcept {}
void ftm0_clear_chf(uint8_t) noexcept {}
}  // namespace ems::hal

// ── Infra de teste ────────────────────────────────────────────────────────────
namespace {

int g_tests_run = 0;
int g_tests_failed = 0;

#define TEST_ASSERT_TRUE(cond) do { \
    ++g_tests_run; \
    if (!(cond)) { \
        ++g_tests_failed; \
        std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    } \
} while (0)

#define TEST_ASSERT_EQ_U32(exp, act) do { \
    ++g_tests_run; \
    const uint32_t _e = static_cast<uint32_t>(exp); \
    const uint32_t _a = static_cast<uint32_t>(act); \
    if (_e != _a) { \
        ++g_tests_failed; \
        std::printf("FAIL %s:%d: expected %u got %u\n", \
                    __FILE__, __LINE__, (unsigned)_e, (unsigned)_a); \
    } \
} while (0)

// ── Helpers de alimentação do CKP ─────────────────────────────────────────────

static uint16_t g_capture = 0u;

// Alimenta um dente normal ao CKP (período de 1000 ticks)
void feed_ckp(uint16_t period_ticks = 1000u) {
    ems_test_gpiod_pdir = (1u << 0u);
    g_capture = static_cast<uint16_t>(g_capture + period_ticks);
    ems_test_ftm3_c0v = g_capture;
    ems::drv::ckp_ftm3_ch0_isr();
}

// Alimenta um pulso de cam (toggles phase_A)
void feed_cam() {
    ems_test_gpiod_pdir = (1u << 1u);
    ems::drv::ckp_ftm3_ch1_isr();
}

// Sincroniza com dois gaps (requer 2 × (58 dentes normais + 1 gap))
void sync_with_two_gaps() {
    for (int i = 0; i < 58; ++i) { feed_ckp(1000u); }
    feed_ckp(1600u);   // gap 1 (período 1,6× maior que a média)
    for (int i = 0; i < 58; ++i) { feed_ckp(1000u); }
    feed_ckp(1600u);   // gap 2 → state = SYNCED
}

// Reset completo entre testes
void test_reset() {
    ems::drv::ckp_test_reset();
    ems::drv::sched_test_reset();
    ems::engine::cycle_sched_test_reset();
    ems::hal::g_ftm0_count = 0u;
    g_capture = 0u;
    ems_test_ftm3_c0v = 0u;
    ems_test_ftm3_c1v = 0u;
    ems_test_gpiod_pdir = 0u;
}

// ── Cálculo dos dentes-gatilho esperados ─────────────────────────────────────
// Replica a lógica de cycle_sched_init() para verificação independente.
// kTrigLeadTeeth = kSoiTeeth(10) + kInjLeadTeeth(4) = 14 dentes antes do TDC.

struct ExpTrigger {
    uint8_t tooth;
    bool    phase;
};

static ExpTrigger expected_trigger(uint8_t slot) {
    const uint8_t  cyl_idx  = static_cast<uint8_t>(ems::engine::firing_order[slot] - 1u);
    const uint16_t tdc_deg  = ems::engine::cylinder_offset_deg[cyl_idx];
    const uint16_t offset   = static_cast<uint16_t>((14u * 360u) / 58u);  // 14 dentes → 86°
    uint16_t trig_deg;
    if (tdc_deg >= offset) {
        trig_deg = tdc_deg - offset;
    } else {
        trig_deg = tdc_deg + 720u - offset;
    }
    const bool    phase = (trig_deg >= 360u);
    const uint16_t local = trig_deg % 360u;
    const uint8_t  tooth = static_cast<uint8_t>((static_cast<uint32_t>(local) * 58u) / 360u);
    return ExpTrigger{tooth, phase};
}

// ── Casos de teste ────────────────────────────────────────────────────────────

// T1: cycle_sched_init calcula dentes-gatilho corretos para todos os slots
void test_trigger_teeth_computed() {
    test_reset();
    ems::engine::cycle_sched_init();

    for (uint8_t slot = 0u; slot < 4u; ++slot) {
        uint8_t tooth = 0u;
        bool    phase = false;
        TEST_ASSERT_TRUE(ems::engine::cycle_sched_test_trigger(slot, tooth, phase));

        const ExpTrigger exp = expected_trigger(slot);
        if (tooth != exp.tooth || phase != exp.phase) {
            ++g_tests_failed;
            std::printf("FAIL %s:%d: slot %u: expected (tooth=%u,phase=%d) got (tooth=%u,phase=%d)\n",
                        __FILE__, __LINE__,
                        (unsigned)slot,
                        (unsigned)exp.tooth, (int)exp.phase,
                        (unsigned)tooth,     (int)phase);
        }
        ++g_tests_run;
    }
}

// T2: sem habilitar, nenhum evento é agendado mesmo no dente-gatilho
void test_no_schedule_when_disabled() {
    test_reset();
    ems::engine::cycle_sched_init();
    // NÃO chama cycle_sched_enable(true)

    ems::engine::cycle_sched_update(
        60000u,  // pw_ticks (1 ms)
        600u,    // dead_ticks (10 µs)
        620u,    // soi_lead_x10 (62°)
        300,     // advance_x10 (30°)
        1080u);  // dwell_x10 (108°)

    sync_with_two_gaps();
    // Alimenta um cam para definir phase_A = true
    feed_cam();
    // Alimenta dentes até o trigger do slot 0 (tooth=44, phase=true)
    for (int i = 0; i < 44; ++i) { feed_ckp(); }

    TEST_ASSERT_EQ_U32(0u, ems::drv::sched_test_size());
}

// T3: sem parâmetros válidos, nenhum evento é agendado
void test_no_schedule_invalid_params() {
    test_reset();
    ems::engine::cycle_sched_init();
    ems::engine::cycle_sched_enable(true);
    // NÃO chama cycle_sched_update → g_pending[cyl].valid = false

    sync_with_two_gaps();
    feed_cam();  // phase_A = true
    for (int i = 0; i < 44; ++i) { feed_ckp(); }

    TEST_ASSERT_EQ_U32(0u, ems::drv::sched_test_size());
}

// T4: no primeiro dente-gatilho (slot 3 / cyl2, tooth=15, phase=false) com
//     parâmetros válidos, dois eventos devem ser agendados: INJ2 SET e INJ2 CLEAR.
//     Usamos pw=1000 ticks para que o EOI não ultrapasse 65535 e não inverta a
//     ordem de inserção na fila do scheduler.
void test_schedule_fires_at_trigger_tooth() {
    test_reset();
    ems::engine::cycle_sched_init();
    ems::engine::cycle_sched_enable(true);

    // Parâmetros: pw = 1000 ticks (~17 µs), dead = 600 ticks (10 µs)
    ems::engine::cycle_sched_update(
        1000u,    // pw_ticks (pequeno para não causar wrapping de uint16 no EOI)
        600u,     // dead_ticks
        620u,     // soi_lead_x10 (62°)
        300,      // advance_x10 (30°)
        1080u);   // dwell_x10

    sync_with_two_gaps();

    // Após sync, tooth_index=0, phase_A=false.
    // Sequência de disparo: o primeiro trigger é slot 3 (cyl2, TDC=180°):
    //   tooth=15, phase=false → INJ2 = kInjCh[cyl_idx=1].
    // Sem cam pulse, phase permanece false. Alimenta exatamente 15 dentes.
    for (int i = 0; i < 15; ++i) { feed_ckp(); }

    // Verifica que 2 eventos foram agendados (INJ_SET e INJ_CLR para cyl2)
    TEST_ASSERT_EQ_U32(2u, ems::drv::sched_test_size());

    // Verifica que o primeiro evento é INJ2 SET
    uint16_t ticks = 0u;
    ems::drv::Channel ch = ems::drv::Channel::INJ1;
    ems::drv::Action  act = ems::drv::Action::CLEAR;
    bool valid = false;
    TEST_ASSERT_TRUE(ems::drv::sched_test_event(0u, ticks, ch, act, valid));
    TEST_ASSERT_TRUE(valid);
    TEST_ASSERT_TRUE(ch == ems::drv::Channel::INJ2);
    TEST_ASSERT_TRUE(act == ems::drv::Action::SET);

    // Verifica que o segundo evento é INJ2 CLEAR, com ticks diferente do SOI
    uint16_t ticks2 = 0u;
    ems::drv::Channel ch2 = ems::drv::Channel::INJ1;
    ems::drv::Action  act2 = ems::drv::Action::SET;
    bool valid2 = false;
    TEST_ASSERT_TRUE(ems::drv::sched_test_event(1u, ticks2, ch2, act2, valid2));
    TEST_ASSERT_TRUE(valid2);
    TEST_ASSERT_TRUE(ch2 == ems::drv::Channel::INJ2);
    TEST_ASSERT_TRUE(act2 == ems::drv::Action::CLEAR);
    // EOI deve ser agendado após SOI
    TEST_ASSERT_TRUE(ticks2 != ticks);
}

// T5: dentes antes do primeiro trigger (tooth=15) não disparam eventos
void test_no_spurious_events_between_triggers() {
    test_reset();
    ems::engine::cycle_sched_init();
    ems::engine::cycle_sched_enable(true);

    ems::engine::cycle_sched_update(60000u, 600u, 620u, 300, 1080u);

    sync_with_two_gaps();
    // tooth_index=0, phase=false. Alimenta 10 dentes (antes do primeiro trigger em 15)
    for (int i = 0; i < 10; ++i) { feed_ckp(); }

    // Ainda não chegou ao dente 15 → sem eventos
    TEST_ASSERT_EQ_U32(0u, ems::drv::sched_test_size());
}

}  // namespace

int main() {
    test_trigger_teeth_computed();
    test_no_schedule_when_disabled();
    test_no_schedule_invalid_params();
    test_schedule_fires_at_trigger_tooth();
    test_no_spurious_events_between_triggers();

    std::printf("tests=%d failed=%d\n", g_tests_run, g_tests_failed);
    return (g_tests_failed == 0) ? 0 : 1;
}
