/**
 * @file app/nvm_boot.cpp
 * Boot-time NVM → RAM calibration loaders (relocated from main_stm32.cpp).
 */
#include "app/nvm_boot.h"

#include <cstring>

#include "engine/calibration.h"
#include "engine/engine_config.h"
#include "engine/etb_control.h"
#include "engine/table3d.h"
#include "hal/flash.h"

namespace ems::app {

bool page_range_is_zero(const uint8_t* data, uint16_t off, uint16_t len) noexcept {
    for (uint16_t i = 0u; i < len; ++i) {
        if (data[off + i] != 0u) {
            return false;
        }
    }
    return true;
}

bool page_range_is_erased(const uint8_t* data, uint16_t off, uint16_t len) noexcept {
    for (uint16_t i = 0u; i < len; ++i) {
        if (data[off + i] != 0xFFu) { return false; }
    }
    return true;
}

bool page_is_erased(const uint8_t* data, uint16_t len) noexcept {
    return page_range_is_erased(data, 0u, len);
}

// BUG (encontrado 2026-07-10): burn_page_to_flash grava VE/Spark/Lambda
// (páginas 1/2/4, índices internos 1/2/3) correctamente, mas nunca havia
// loader correspondente aqui — reset_pages() (ui_protocol.cpp) copia
// SEMPRE ve_table/spark_table/lambda_target_table_x1000 (defaults de
// compilação, nunca actualizados a partir da flash) para os buffers da UI
// no boot, apagando silenciosamente qualquer tabela editada e gravada.
void load_ve_table_from_nvm() noexcept {
    alignas(4) uint8_t page[sizeof(ems::engine::ve_table)] = {};
    if (!ems::hal::nvm_load_calibration(1u, page, sizeof(page)) ||
        page_is_erased(page, sizeof(page))) {
        return;  // flash apagada → tabela default de compilação
    }
    std::memcpy(ems::engine::ve_table, page, sizeof(ems::engine::ve_table));
}

void load_spark_table_from_nvm() noexcept {
    alignas(4) uint8_t page[sizeof(ems::engine::spark_table)] = {};
    if (!ems::hal::nvm_load_calibration(2u, page, sizeof(page)) ||
        page_is_erased(page, sizeof(page))) {
        return;
    }
    std::memcpy(ems::engine::spark_table, page, sizeof(ems::engine::spark_table));
}

void load_lambda_target_table_from_nvm() noexcept {
    alignas(4) uint8_t page[sizeof(ems::engine::lambda_target_table_x1000)] = {};
    if (!ems::hal::nvm_load_calibration(3u, page, sizeof(page)) ||
        page_is_erased(page, sizeof(page))) {
        return;
    }
    std::memcpy(ems::engine::lambda_target_table_x1000, page,
                sizeof(ems::engine::lambda_target_table_x1000));
}

void load_corr_calibration_from_nvm() noexcept {
    alignas(4) uint8_t page[256] = {};
    if (!ems::hal::nvm_load_calibration(4u, page, sizeof(page)) ||
        page_is_erased(page, sizeof(page))) {
        return;
    }

    const uint8_t* p = page;
    std::memcpy(ems::engine::clt_corr_axis_x10,          p +   0, 16u);
    std::memcpy(ems::engine::clt_corr_x256,              p +  16, 16u);
    std::memcpy(ems::engine::iat_corr_axis_x10,          p +  32, 16u);
    std::memcpy(ems::engine::iat_corr_x256,              p +  48, 16u);
    std::memcpy(ems::engine::warmup_corr_axis_x10,       p +  64, 16u);
    std::memcpy(ems::engine::warmup_corr_x256,           p +  80, 16u);
    std::memcpy(ems::engine::vbatt_corr_axis_mv,         p +  96, 16u);
    std::memcpy(ems::engine::injector_dead_time_us,      p + 112, 16u);
    std::memcpy(ems::engine::ae_clt_corr_axis_x10,       p + 128, 16u);
    std::memcpy(ems::engine::ae_clt_sens,                p + 144, 16u);
    std::memcpy(ems::engine::dwell_vbatt_axis_mv,        p + 160, 16u);
    std::memcpy(ems::engine::dwell_ms_x10_table,         p + 176, 16u);
    std::memcpy(ems::engine::lambda_delay_rpm_axis_x10,  p + 192, 12u);
    std::memcpy(ems::engine::lambda_delay_load_axis_bar_x100, p + 204, 12u);
    std::memcpy(ems::engine::lambda_delay_ms_table,      p + 216, 18u);
    if (!page_range_is_zero(page, 234u, 6u)) {
        std::memcpy(&ems::engine::ae_tpsdot_threshold_x10, p + 234, 2u);
        std::memcpy(&ems::engine::ae_taper_cycles,         p + 236, 2u);
        std::memcpy(&ems::engine::ae_max_pw_us,            p + 238, 2u);
    }
    if (page_range_is_zero(page, 240u, 16u)) {
        return;
    }
    std::memcpy(&ems::engine::idle_spark_tps_max_x10,             p + 240, 2u);
    std::memcpy(&ems::engine::idle_spark_map_max_bar_x100,             p + 242, 2u);
    std::memcpy(&ems::engine::idle_spark_rpm_min_x10,             p + 244, 2u);
    std::memcpy(&ems::engine::idle_spark_window_above_target_x10, p + 246, 2u);
    std::memcpy(&ems::engine::idle_spark_deadband_rpm_x10,        p + 248, 2u);
    std::memcpy(&ems::engine::idle_spark_rpm_per_deg_x10,         p + 250, 2u);
    std::memcpy(&ems::engine::idle_spark_retard_limit_deg,        p + 252, 2u);
    std::memcpy(&ems::engine::idle_spark_advance_limit_deg,       p + 254, 2u);
}

void load_dwell2d_calibration_from_nvm() noexcept {
    alignas(4) uint8_t page[32] = {};
    if (!ems::hal::nvm_load_calibration(6u, page, sizeof(page)) ||
        page_is_erased(page, 16u)) {
        return;  // sem page 6 → mantém valores default (factor 1.0× a 4000 RPM)
    }
    std::memcpy(ems::engine::dwell_rpm_axis_rpm,  page + 0,  8u);
    std::memcpy(ems::engine::dwell_rpm_factor_q8, page + 8,  8u);
}

void load_boost_map_from_nvm() noexcept {
    alignas(4) uint8_t page[112] = {};
    if (!ems::hal::nvm_load_calibration(8u, page, sizeof(page)) ||
        page_is_erased(page, sizeof(page))) {
        return;
    }
    std::memcpy(ems::engine::boost_target_bar_x1000, page,
                sizeof(ems::engine::boost_target_bar_x1000));
}

void load_table_axes_from_nvm() noexcept {
    alignas(4) uint8_t page[64] = {};
    if (!ems::hal::nvm_load_calibration(9u, page, sizeof(page)) ||
        page_is_erased(page, sizeof(page))) {
        return;  // flash apagada → eixos default de compilação
    }
    uint16_t rpm[ems::engine::kTableAxisSize] = {};
    uint16_t load[ems::engine::kTableAxisSize] = {};
    std::memcpy(rpm,  page +  0, 32u);
    std::memcpy(load, page + 32, 32u);
    // table_axes_set valida monotonicidade — conteúdo corrompido é rejeitado
    // e os defaults de compilação permanecem.
    (void)ems::engine::table_axes_set(rpm, load);
}

void load_pedal_map_from_nvm() noexcept {
    alignas(4) uint8_t page[80] = {};
    if (!ems::hal::nvm_load_calibration(7u, page, sizeof(page)) ||
        page_is_erased(page, sizeof(page))) {
        return;  // flash apagada → usa defaults de compilação
    }
    std::memcpy(ems::engine::etb_pedal_map, page, sizeof(ems::engine::etb_pedal_map));
}

void load_xtau_calibration_from_nvm() noexcept {
    alignas(4) uint8_t page[80] = {};
    if (!ems::hal::nvm_load_calibration(5u, page, sizeof(page)) ||
        page_is_erased(page, sizeof(page)) ||
        page_range_is_zero(page, 0u, 48u)) {
        return;
    }

    const uint8_t* p = page;
    std::memcpy(ems::engine::xtau_clt_axis_x10,  p +  0, 16u);
    std::memcpy(ems::engine::xtau_x_fraction_q8, p + 16, 16u);
    std::memcpy(ems::engine::xtau_tau_cycles,    p + 32, 16u);
    if (!page_range_is_zero(page, 48u, 16u)) {
        std::memcpy(ems::engine::ae_tpsdot_axis_x10, p + 48, 8u);
        std::memcpy(ems::engine::ae_pw_adder_us,     p + 56, 8u);
    }
    if (!page_range_is_zero(page, 64u, 12u) &&
        !page_range_is_erased(page, 64u, 12u)) {
        std::memcpy(&ems::engine::crank_enter_rpm_x10,   p + 64, 2u);
        std::memcpy(&ems::engine::crank_exit_rpm_x10,    p + 66, 2u);
        std::memcpy(&ems::engine::crank_spark_deg,       p + 68, 2u);
        std::memcpy(&ems::engine::crank_min_pw_us,       p + 70, 2u);
        std::memcpy(&ems::engine::crank_prime_tooth,     p + 72, 2u);
        std::memcpy(&ems::engine::crank_prime_max_pw_us, p + 74, 2u);
    }
}


void nvm_boot_load_tables(bool cal_layout_ok) noexcept {
    // Preserve exact former main_stm32 call order and layout gating.
    if (cal_layout_ok) {
        load_ve_table_from_nvm();
        load_spark_table_from_nvm();
        load_lambda_target_table_from_nvm();
    }
    load_corr_calibration_from_nvm();
    load_xtau_calibration_from_nvm();
    load_dwell2d_calibration_from_nvm();
    load_pedal_map_from_nvm();
    load_boost_map_from_nvm();
    if (cal_layout_ok) {
        load_table_axes_from_nvm();
    }
}

}  // namespace ems::app
