#include "engine/quick_crank.h"
#include "drv/ckp.h"

#include <cstdint>

namespace {

struct P2 {
    int16_t x;
    uint16_t y;
};

constexpr uint32_t kCrankEnterRpmX10 = 4500u;
constexpr uint32_t kCrankExitRpmX10 = 7000u;
constexpr int16_t kCrankSparkDeg = 8;
constexpr uint32_t kCrankMinPwUs = 2500u;
constexpr uint32_t kDefaultReqFuelUs = 8000u;
constexpr uint32_t kPrimePwMaxUs = 30000u;
constexpr uint8_t  kPrimeToothTarget = 5u;  // 5º dente desde o início do cranking

constexpr P2 kCrankFuelMult[] = {
    {-400, 768},  // 3.00x
    {0, 614},     // 2.40x
    {200, 512},   // 2.00x
    {400, 435},   // 1.70x
    {700, 358},   // 1.40x
    {900, 320},   // 1.25x
    {1100, 294},  // 1.15x
};

constexpr P2 kAfterstartMultStart[] = {
    {-400, 346},  // 1.35x
    {0, 333},     // 1.30x
    {200, 320},   // 1.25x
    {400, 307},   // 1.20x
    {700, 294},   // 1.15x
    {900, 281},   // 1.10x
    {1100, 269},  // 1.05x
};

constexpr P2 kAfterstartDurationMs[] = {
    {-400, 2400},
    {0, 2000},
    {200, 1700},
    {400, 1400},
    {700, 1000},
    {900, 700},
    {1100, 500},
};

bool g_prev_cranking = false;
uint32_t g_afterstart_start_ms = 0u;
uint32_t g_afterstart_duration_ms = 0u;

// ── Estado do prime pulse (ISR-safe) ──────────────────────────────────────────
volatile uint8_t  g_prime_tooth_count = 0u;   ///< Dentes contados desde cranking
volatile bool     g_prime_done        = false; ///< Já disparado neste ciclo de partida
volatile bool     g_prime_pending     = false; ///< Sinaliza loop de fundo
volatile uint32_t g_prime_pw_us       = 0u;   ///< PW calculada para disparo
int16_t           g_prime_clt_x10    = 900;   ///< CLT mais recente (do loop de fundo)

uint16_t interp_u16(const P2* table, uint8_t n, int16_t x) noexcept {
    if (x <= table[0].x) {
        return table[0].y;
    }
    if (x >= table[n - 1u].x) {
        return table[n - 1u].y;
    }
    for (uint8_t i = 0u; i < (n - 1u); ++i) {
        if (x <= table[i + 1u].x) {
            const int32_t x0 = table[i].x;
            const int32_t x1 = table[i + 1u].x;
            const int32_t y0 = table[i].y;
            const int32_t y1 = table[i + 1u].y;
            const int32_t dx = static_cast<int32_t>(x) - x0;
            const int32_t span = x1 - x0;
            return static_cast<uint16_t>(y0 + ((y1 - y0) * dx) / span);
        }
    }
    return table[n - 1u].y;
}

uint16_t clamp_u16(uint32_t v, uint16_t lo, uint16_t hi) noexcept {
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return static_cast<uint16_t>(v);
}

bool detect_cranking(uint32_t rpm_x10, bool full_sync) noexcept {
    if (!full_sync || rpm_x10 == 0u) {
        return false;
    }
    if (g_prev_cranking) {
        return rpm_x10 < kCrankExitRpmX10;
    }
    return rpm_x10 <= kCrankEnterRpmX10;
}

uint16_t afterstart_mult_x256(uint32_t now_ms, int16_t clt_x10) noexcept {
    if (g_afterstart_duration_ms == 0u) {
        return 256u;
    }
    const uint32_t elapsed = now_ms - g_afterstart_start_ms;
    if (elapsed >= g_afterstart_duration_ms) {
        return 256u;
    }
    const uint16_t start = interp_u16(
        kAfterstartMultStart,
        static_cast<uint8_t>(sizeof(kAfterstartMultStart) / sizeof(kAfterstartMultStart[0])),
        clt_x10);
    const uint32_t decay = static_cast<uint32_t>(start - 256u) * elapsed;
    const uint32_t mult = static_cast<uint32_t>(start) - (decay / g_afterstart_duration_ms);
    return clamp_u16(mult, 256u, 512u);
}

static inline void enter_critical() noexcept {
#if defined(__arm__) || defined(__thumb__)
    asm volatile("cpsid i" ::: "memory");
#endif
}

static inline void exit_critical() noexcept {
#if defined(__arm__) || defined(__thumb__)
    asm volatile("cpsie i" ::: "memory");
#endif
}

}  // namespace

// ── Override do hook de dente (ISR de CKP, prioridade 1) ────────────────────
// Conta os primeiros kPrimeToothTarget dentes recebidos durante cranking.
// Quando o 5º dente chega, calcula a PW e sinaliza o loop de fundo via
// g_prime_pending. Não requer sincronização — opera em qualquer SyncState.
namespace ems::drv {

void prime_on_tooth(const CkpSnapshot& snap) noexcept {
    using namespace ems::engine;  // acessa g_prime_* e constantes

    if (g_prime_done) { return; }

    // Só conta enquanto RPM indica cranking (< 700 RPM)
    if (snap.rpm_x10 == 0u || snap.rpm_x10 >= kCrankExitRpmX10) { return; }

    ++g_prime_tooth_count;
    if (g_prime_tooth_count < kPrimeToothTarget) { return; }

    // 5º dente: calcula PW usando a tabela de enriquecimento de cranking e
    // o CLT mais recente atualizado pelo loop de fundo.
    const uint32_t mult = interp_u16(
        kCrankFuelMult,
        static_cast<uint8_t>(sizeof(kCrankFuelMult) / sizeof(kCrankFuelMult[0])),
        g_prime_clt_x10);
    uint32_t pw = (kDefaultReqFuelUs * static_cast<uint32_t>(mult)) >> 8u;
    if (pw > kPrimePwMaxUs) { pw = kPrimePwMaxUs; }

    g_prime_pw_us  = pw;
    g_prime_pending = true;
    g_prime_done   = true;
}

}  // namespace ems::drv

namespace ems::engine {

void quick_crank_reset() noexcept {
    g_prev_cranking = false;
    g_afterstart_start_ms = 0u;
    g_afterstart_duration_ms = 0u;
    g_prime_tooth_count = 0u;
    g_prime_done        = false;
    g_prime_pending     = false;
    g_prime_pw_us       = 0u;
}

QuickCrankOutput quick_crank_update(uint32_t now_ms,
                                    uint32_t rpm_x10,
                                    bool full_sync,
                                    int16_t clt_x10,
                                    int16_t base_spark_deg) noexcept {
    QuickCrankOutput out{};
    out.spark_deg = base_spark_deg;
    out.fuel_mult_x256 = 256u;
    out.min_pw_us = 0u;
    out.prime_pw_us = g_prime_pw_us;  // informativo — disparo é feito via consume_prime()

    const bool cranking = detect_cranking(rpm_x10, full_sync);
    out.cranking = cranking;

    if (cranking) {
        out.spark_deg = kCrankSparkDeg;
        out.min_pw_us = kCrankMinPwUs;
        out.fuel_mult_x256 = interp_u16(
            kCrankFuelMult,
            static_cast<uint8_t>(sizeof(kCrankFuelMult) / sizeof(kCrankFuelMult[0])),
            clt_x10);
        g_afterstart_duration_ms = 0u;
    } else {
        if (g_prev_cranking && rpm_x10 >= kCrankExitRpmX10) {
            g_afterstart_start_ms = now_ms;
            g_afterstart_duration_ms = interp_u16(
                kAfterstartDurationMs,
                static_cast<uint8_t>(sizeof(kAfterstartDurationMs) / sizeof(kAfterstartDurationMs[0])),
                clt_x10);
        }
        const uint16_t as_mult = afterstart_mult_x256(now_ms, clt_x10);
        out.afterstart_active = (as_mult > 256u);
        out.fuel_mult_x256 = as_mult;
    }

    g_prev_cranking = cranking;
    return out;
}

uint32_t quick_crank_apply_pw_us(uint32_t base_pw_us,
                                 uint16_t fuel_mult_x256,
                                 uint32_t min_pw_us) noexcept {
    uint32_t out = static_cast<uint32_t>(
        (static_cast<uint64_t>(base_pw_us) * fuel_mult_x256) / 256u);
    if (out < min_pw_us) {
        out = min_pw_us;
    }
    if (out > 100000u) {
        out = 100000u;
    }
    return out;
}

void quick_crank_set_clt(int16_t clt_x10) noexcept {
    // Escrita atômica de int16_t em Cortex-M4 (single instrução STRH).
    g_prime_clt_x10 = clt_x10;
}

uint32_t quick_crank_consume_prime() noexcept {
    enter_critical();
    uint32_t pw = 0u;
    if (g_prime_pending) {
        pw = g_prime_pw_us;
        g_prime_pending = false;
    }
    exit_critical();
    return pw;
}

}  // namespace ems::engine
