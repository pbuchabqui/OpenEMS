#pragma once

#include <cstdint>

#include "drv/ckp.h"

namespace ems::drv {

// 8 canais: 4 injeção (INJ1-4 = FTM0_CH0-3) + 4 ignição (IGN1-4 = FTM0_CH4-7)
enum class Channel : uint8_t {
    INJ3 = 0,
    INJ4 = 1,
    INJ1 = 2,
    INJ2 = 3,
    IGN4 = 4,
    IGN3 = 5,
    IGN2 = 6,
    IGN1 = 7,
};

enum class Action : uint8_t { SET, CLEAR };

// Retorna false se fila cheia ou evento já passou.
bool sched_event(Channel ch, uint16_t ftm0_ticks, Action act) noexcept;
void sched_cancel(Channel ch) noexcept;
void sched_cancel_all() noexcept;
void sched_isr() noexcept;
void sched_recalc(const CkpSnapshot& snap) noexcept;

#if defined(EMS_HOST_TEST)
void sched_test_reset() noexcept;
uint8_t sched_test_size() noexcept;
bool sched_test_event(uint8_t index, uint16_t& ticks, Channel& ch, Action& act, bool& valid) noexcept;
#endif

}  // namespace ems::drv
