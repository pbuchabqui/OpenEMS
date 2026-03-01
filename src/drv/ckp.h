#pragma once

#include <cstdint>

namespace ems::drv {

enum class SyncState : uint8_t { WAIT, SYNCING, SYNCED };

struct CkpSnapshot {
    uint32_t tooth_period_ns;
    uint16_t tooth_index;
    uint16_t last_ftm3_capture;
    uint32_t rpm_x10;
    SyncState state;
    bool phase_A;
};

CkpSnapshot ckp_snapshot() noexcept;
uint16_t ckp_angle_to_ticks(uint16_t angle_x10, uint16_t ref_capture) noexcept;

// Hooks chamados a cada dente pela ISR do CKP (símbolos fracos — sobrescreva para adicionar
// comportamento). sensors_on_tooth: amostragem de sensores (drv/sensors.cpp).
// schedule_on_tooth: agendamento de injeção/ignição (engine/cycle_sched.cpp).
void sensors_on_tooth(const CkpSnapshot& snap) noexcept;
void schedule_on_tooth(const CkpSnapshot& snap) noexcept;

void ckp_ftm3_ch0_isr() noexcept;
void ckp_ftm3_ch1_isr() noexcept;

#if defined(EMS_HOST_TEST)
void ckp_test_reset() noexcept;
uint32_t ckp_test_rpm_x10_from_period_ns(uint32_t period_ns) noexcept;
#endif

}  // namespace ems::drv
