/**
 * @file test_safety_metrics.cpp
 * @brief Concrete timing/angle numbers for the FOME safety gauntlet.
 *
 * Encoder: 16384 counts/rev. FOME 60-2: 6°/tooth + scheduleByAngle
 * (delayUs = oneDegreeUs * angle) holding last-tooth RPM.
 */
#include "test/harness.h"

#include "engine/ecu_sched.h"

#include <cstdint>
#include <cstdio>

namespace {

constexpr uint32_t kCountsPerRev = 16384u;
constexpr double kDegPerCount = 360.0 / static_cast<double>(kCountsPerRev);
constexpr uint32_t kTim5Hz = 62500000u;

// Seed ω so x65536 matches a physical RPM (heavy-tick formula inverted).
// rpm_x10 = 600e9 * ω / (16 * 16384 * 65536)
// Choose ΔTIM2, ΔTIM5 with ΔTIM2/ΔTIM5 = 16384 / ticks_per_rev.
void seed_omega_for_rpm(uint32_t rpm) noexcept
{
    ecu_sched_test_reset();
    const uint64_t ticks_per_rev =
        (static_cast<uint64_t>(kTim5Hz) * 60ull) / static_cast<uint64_t>(rpm);
    ecu_sched_encoder_omega_sample(0u, 0u);
    ecu_sched_encoder_omega_sample(kCountsPerRev,
                                   static_cast<uint32_t>(ticks_per_rev));
}

double deg_from_us_at_rpm(double us, uint32_t rpm) noexcept
{
    return (static_cast<double>(rpm) / 60.0) * 360.0 * (us * 1.0e-6);
}

}  // namespace

void test_safety_metrics_encoder_quantisation(void)
{
    section("safety metrics: encoder quantisation vs 60-2 tooth");
    ecu_sched_test_reset();

    uint32_t max_int_err_deg = 0u;
    uint32_t out_of_range = 0u;
    for (uint32_t deg = 0u; deg < 360u; ++deg) {
        const uint32_t counts = ecu_sched_encoder_test_engine_deg_to_counts(deg);
        if (counts >= kCountsPerRev) {
            ++out_of_range;
        }
        const uint32_t recon = (counts * 360u) / kCountsPerRev;
        const uint32_t err = (recon > deg) ? (recon - deg) : (deg - recon);
        if (err > max_int_err_deg) {
            max_int_err_deg = err;
        }
    }
    CHECK_EQ(out_of_range, 0u, "all integer-deg conversions in 0..16383");
    CHECK_TRUE(max_int_err_deg <= 1u, "integer-deg round-trip error <= 1°");
    CHECK_TRUE(kDegPerCount < 0.022, "encoder step 360/16384 < 0.022°");
    CHECK_TRUE(kDegPerCount * 6.0 < 0.14, "6 encoder counts << one 60-2 tooth");
    std::printf("    encoder step = %.5f deg/count (FOME 60-2 tooth = 6.00000 deg)\n",
                kDegPerCount);
}

void test_safety_metrics_isr_lag_degrees(void)
{
    section("safety metrics: 1 us / 3 us ISR lag in degrees at 600/2000/6000 rpm");

    struct Row {
        uint32_t rpm;
        double max_lag_us;
        double max_err_deg;
    };
    const Row rows[] = {
        {600u, 3.0, 0.011},    // 600 rpm * 360/60 * 3e-6 = 0.0108°
        {2000u, 3.0, 0.037},   // 0.036°
        {6000u, 3.0, 0.110},   // 0.108°
    };
    for (const Row& r : rows) {
        const double err1 = deg_from_us_at_rpm(1.0, r.rpm);
        const double err3 = deg_from_us_at_rpm(r.max_lag_us, r.rpm);
        CHECK_TRUE(err1 < 0.04, "1 us lag < 0.04° even at 6000 rpm");
        CHECK_TRUE(err3 < r.max_err_deg + 0.002, "3 us dispatch margin under bound");
        std::printf("    %u rpm: 1us=%.5f°  3us=%.5f°  (FOME 60-2 tooth=6°)\n",
                    r.rpm, err1, err3);
        (void)err1;
    }
}

void test_safety_metrics_min_lead_counts(void)
{
    section("safety metrics: 2 us min-lead in counts at representative rpm");

    // 2 us floor is converted via ω. At 6000 rpm one count is
    // 60/6000/16384 s = 0.610 us, so 2 us ≈ 3.3 counts.
    seed_omega_for_rpm(6000u);
    CHECK_EQ(ecu_sched_encoder_omega_valid(), 1u, "omega valid at 6000 rpm seed");

    const int32_t omega = ecu_sched_encoder_omega_x65536();
    CHECK_TRUE(omega > 0, "omega positive at 6000 rpm");
    // min_lead_counts is not exported; arm a target==now and see clamp.
    // At 6000 rpm, 2 us of counts = duration_ticks_to_span(125 ticks).
    // ticks=125, span = 125 * omega / 65536.
    const uint32_t span_2us =
        static_cast<uint32_t>((static_cast<uint64_t>(125u) *
                               static_cast<uint64_t>(omega)) /
                              65536ull);
    CHECK_TRUE(span_2us >= 1u && span_2us <= 8u,
               "2 us min-lead at 6000 rpm is a handful of counts, not 36 ms");
    std::printf("    6000 rpm: omega_x65536=%d  2us_lead=%u counts (%.4f°)\n",
                omega, span_2us, static_cast<double>(span_2us) * kDegPerCount);

    seed_omega_for_rpm(600u);
    const int32_t omega_idle = ecu_sched_encoder_omega_x65536();
    const uint32_t span_idle =
        static_cast<uint32_t>((static_cast<uint64_t>(125u) *
                               static_cast<uint64_t>(omega_idle)) /
                              65536ull);
    std::printf("    600 rpm: omega_x65536=%d  2us_lead=%u counts (%.4f°)\n",
                omega_idle, span_idle, static_cast<double>(span_idle) * kDegPerCount);
    CHECK_TRUE(span_idle <= span_2us, "min-lead counts grows with rpm, not a fixed angle");
}

void test_safety_metrics_fome_tooth_hold(void)
{
    section("safety metrics: FOME scheduleByAngle hold vs encoder compare");
    // FOME delayUs = oneDegreeUs * angle, frozen from the last tooth.
    // Worst case inside a 60-2 6° gap: the whole 6° is timed from stale RPM.
    // A 5% RPM error over that gap is 0.30° of spark/inj error.
    const double fome_gap_deg = 6.0;
    const double five_pct = fome_gap_deg * 0.05;
    CHECK_TRUE(five_pct > 0.2, "FOME 5% RPM-hold error over one tooth > 0.2°");
    CHECK_TRUE(kDegPerCount < five_pct / 10.0,
               "encoder step is >10x finer than FOME 5% tooth-hold error");
    std::printf("    FOME 60-2 5%% RPM-hold error over 6° = %.3f°; encoder step = %.5f°\n",
                five_pct, kDegPerCount);
}
