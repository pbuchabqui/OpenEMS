#include "drv/scheduler.h"

#include <cstddef>
#include <cstdint>

#if __has_include("hal/ftm.h")
#include "hal/ftm.h"
#elif __has_include("ftm.h")
#include "ftm.h"
#endif

#if defined(EMS_HOST_TEST)
volatile uint32_t ems_test_fgpio_c_psor = 0u;
volatile uint32_t ems_test_fgpio_c_pcor = 0u;
volatile uint32_t ems_test_fgpio_d_psor = 0u;
volatile uint32_t ems_test_fgpio_d_pcor = 0u;
#endif

namespace {

static constexpr uint8_t QUEUE_SIZE = 16u;
static constexpr uint16_t kPastThresholdTicks = 60000u;

struct Event {
    uint16_t ftm0_ticks;
    ems::drv::Channel channel;
    ems::drv::Action action;
    bool valid;
};

static volatile Event queue[QUEUE_SIZE] = {};
static volatile uint8_t q_size = 0u;
static ems::drv::CkpSnapshot g_last_snap = {0u, 0u, 0u, 0u, ems::drv::SyncState::WAIT, false};
static bool g_have_last_snap = false;

#if defined(EMS_HOST_TEST)
#define FGPIOC_PSOR ems_test_fgpio_c_psor
#define FGPIOC_PCOR ems_test_fgpio_c_pcor
#define FGPIOD_PSOR ems_test_fgpio_d_psor
#define FGPIOD_PCOR ems_test_fgpio_d_pcor
#else
#define FGPIOC_PSOR (*reinterpret_cast<volatile uint32_t*>(0xF80FF084u))
#define FGPIOC_PCOR (*reinterpret_cast<volatile uint32_t*>(0xF80FF088u))
#define FGPIOD_PSOR (*reinterpret_cast<volatile uint32_t*>(0xF80FF0C4u))
#define FGPIOD_PCOR (*reinterpret_cast<volatile uint32_t*>(0xF80FF0C8u))
#endif

#define FTM0_BASE (0x40038000u)
#define FTM_CnSC(base, n) (*reinterpret_cast<volatile uint32_t*>((base) + 0x0Cu + (uint32_t)(n) * 8u))
#define FTM_CnSC_CHIE (1u << 6)

inline void enter_critical() noexcept {
#if defined(__arm__) || defined(__thumb__)
    asm volatile("cpsid i" ::: "memory");
#endif
}

inline void exit_critical() noexcept {
#if defined(__arm__) || defined(__thumb__)
    asm volatile("cpsie i" ::: "memory");
#endif
}

inline uint8_t to_ch(ems::drv::Channel ch) noexcept {
    return static_cast<uint8_t>(ch);
}

inline bool is_event_in_past(uint16_t target, uint16_t now) noexcept {
    const uint16_t delta = static_cast<uint16_t>(target - now);
    return (delta > kPastThresholdTicks);
}

inline Event queue_read(uint8_t idx) noexcept {
    return Event{
        queue[idx].ftm0_ticks,
        queue[idx].channel,
        queue[idx].action,
        queue[idx].valid,
    };
}

inline void queue_write(uint8_t idx, const Event& ev) noexcept {
    queue[idx].ftm0_ticks = ev.ftm0_ticks;
    queue[idx].channel = ev.channel;
    queue[idx].action = ev.action;
    queue[idx].valid = ev.valid;
}

inline void disable_compare_irq(uint8_t ch) noexcept {
#if defined(EMS_HOST_TEST)
    (void)ch;
#else
    FTM_CnSC(FTM0_BASE, ch) &= ~FTM_CnSC_CHIE;
#endif
}

inline void execute_gpio(ems::drv::Channel ch, ems::drv::Action action) noexcept {
    if (to_ch(ch) <= 3u) {
        const uint32_t bit = 1u << (to_ch(ch) + 1u);
        if (action == ems::drv::Action::SET) {
            FGPIOC_PSOR = bit;
        } else {
            FGPIOC_PCOR = bit;
        }
        return;
    }

    const uint32_t bit = 1u << to_ch(ch);
    if (action == ems::drv::Action::SET) {
        FGPIOD_PSOR = bit;
    } else {
        FGPIOD_PCOR = bit;
    }
}

inline void compact_and_sort() noexcept {
    Event tmp[QUEUE_SIZE];
    uint8_t n = 0u;

    for (uint8_t i = 0u; i < q_size; ++i) {
        const Event ev = queue_read(i);
        if (ev.valid) {
            tmp[n++] = Event{ev.ftm0_ticks, ev.channel, ev.action, true};
        }
    }

    for (uint8_t i = 1u; i < n; ++i) {
        Event key = tmp[i];
        int8_t j = static_cast<int8_t>(i) - 1;
        while (j >= 0 && tmp[(uint8_t)j].ftm0_ticks > key.ftm0_ticks) {
            tmp[(uint8_t)j + 1u] = tmp[(uint8_t)j];
            --j;
        }
        tmp[(uint8_t)j + 1u] = key;
    }

    for (uint8_t i = 0u; i < n; ++i) {
        queue_write(i, tmp[i]);
    }
    for (uint8_t i = n; i < QUEUE_SIZE; ++i) {
        queue_write(i, Event{0u, ems::drv::Channel::INJ3, ems::drv::Action::SET, false});
    }

    q_size = n;
}

inline void program_next_compare() noexcept {
    if (q_size == 0u) {
        for (uint8_t ch = 0u; ch < 8u; ++ch) {
            disable_compare_irq(ch);
        }
        return;
    }

    const Event ev = queue_read(0u);
    ems::hal::ftm0_set_compare(to_ch(ev.channel), ev.ftm0_ticks);
}

}  // namespace

namespace ems::drv {

bool sched_event(Channel ch, uint16_t ftm0_ticks, Action act) noexcept {
    enter_critical();

    if (q_size >= QUEUE_SIZE) {
        exit_critical();
        return false;
    }

    const uint16_t now = ems::hal::ftm0_count();
    if (is_event_in_past(ftm0_ticks, now)) {
        exit_critical();
        return false;
    }

    Event ev = {ftm0_ticks, ch, act, true};
    uint8_t idx = q_size;
    while (idx > 0u && queue_read(idx - 1u).ftm0_ticks > ev.ftm0_ticks) {
        queue_write(idx, queue_read(idx - 1u));
        --idx;
    }
    queue_write(idx, ev);
    ++q_size;

    program_next_compare();

    exit_critical();
    return true;
}

void sched_cancel(Channel ch) noexcept {
    enter_critical();

    for (uint8_t i = 0u; i < q_size; ++i) {
        const Event ev = queue_read(i);
        if (ev.valid && ev.channel == ch) {
            queue[i].valid = false;
            disable_compare_irq(to_ch(ch));
        }
    }

    compact_and_sort();
    program_next_compare();

    exit_critical();
}

void sched_cancel_all() noexcept {
    enter_critical();

    for (uint8_t i = 0u; i < q_size; ++i) {
        queue[i].valid = false;
    }

    compact_and_sort();
    program_next_compare();

    exit_critical();
}

void sched_isr() noexcept {
    const uint16_t now = ems::hal::ftm0_count();

    while (q_size > 0u) {
        const Event ev = queue_read(0u);
        if (!ev.valid || ev.ftm0_ticks > now) {
            break;
        }

        execute_gpio(ev.channel, ev.action);
        ems::hal::ftm0_clear_chf(to_ch(ev.channel));
        disable_compare_irq(to_ch(ev.channel));

        queue[0].valid = false;
        compact_and_sort();
    }

    program_next_compare();
}

void sched_recalc(const CkpSnapshot& snap) noexcept {
    enter_critical();

    const uint16_t now = ems::hal::ftm0_count();

    if (!g_have_last_snap) {
        g_last_snap = snap;
        g_have_last_snap = true;
        exit_critical();
        return;
    }

    if (g_last_snap.rpm_x10 == 0u || snap.rpm_x10 == 0u) {
        g_last_snap = snap;
        exit_critical();
        return;
    }

    for (uint8_t i = 0u; i < q_size; ++i) {
        const Event ev = queue_read(i);
        if (!ev.valid || is_event_in_past(ev.ftm0_ticks, now)) {
            continue;
        }

        const uint16_t old_delta = static_cast<uint16_t>(ev.ftm0_ticks - g_last_snap.last_ftm3_capture);
        const uint32_t scaled = (static_cast<uint32_t>(old_delta) * g_last_snap.rpm_x10) / snap.rpm_x10;
        queue[i].ftm0_ticks = static_cast<uint16_t>(snap.last_ftm3_capture + static_cast<uint16_t>(scaled));
    }

    compact_and_sort();
    program_next_compare();

    g_last_snap = snap;
    exit_critical();
}

#if defined(EMS_HOST_TEST)
void sched_test_reset() noexcept {
    enter_critical();
    for (uint8_t i = 0u; i < QUEUE_SIZE; ++i) {
        queue_write(i, Event{0u, Channel::INJ3, Action::SET, false});
    }
    q_size = 0u;
    g_last_snap = CkpSnapshot{0u, 0u, 0u, 0u, SyncState::WAIT, false};
    g_have_last_snap = false;
    ems_test_fgpio_c_psor = 0u;
    ems_test_fgpio_c_pcor = 0u;
    ems_test_fgpio_d_psor = 0u;
    ems_test_fgpio_d_pcor = 0u;
    exit_critical();
}

uint8_t sched_test_size() noexcept {
    return q_size;
}

bool sched_test_event(uint8_t index, uint16_t& ticks, Channel& ch, Action& act, bool& valid) noexcept {
    if (index >= q_size) {
        return false;
    }
    ticks = queue[index].ftm0_ticks;
    ch = queue[index].channel;
    act = queue[index].action;
    valid = queue[index].valid;
    return true;
}
#endif

}  // namespace ems::drv
