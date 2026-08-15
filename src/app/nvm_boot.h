#pragma once
/**
 * @file app/nvm_boot.h
 * Relocate-only NVM calibration loaders (hygiene PR-09).
 * Called from openems_init() after page0 load; behaviour preserved.
 */
#include <cstdint>
#include <span>

namespace ems::app {

// std::span em vez de ponteiro cru: off+len deixam de ser confiança cega no
// chamador — a função vê o tamanho real do buffer e pode recusar um range
// fora dos limites em vez de ler lixo. Chamadas com um array (ex.
// `page_is_erased(page, sizeof(page))`) continuam a compilar sem alteração
// — `page` converte implicitamente para span, só o ponteiro cru desaparece.
bool page_range_is_zero(std::span<const uint8_t> data, uint16_t off, uint16_t len) noexcept;
bool page_range_is_erased(std::span<const uint8_t> data, uint16_t off, uint16_t len) noexcept;
bool page_is_erased(std::span<const uint8_t> data, uint16_t len) noexcept;

void load_ve_table_from_nvm() noexcept;
void load_spark_table_from_nvm() noexcept;
void load_lambda_target_table_from_nvm() noexcept;
void load_corr_calibration_from_nvm() noexcept;
void load_dwell2d_calibration_from_nvm() noexcept;
void load_boost_map_from_nvm() noexcept;
void load_table_axes_from_nvm() noexcept;
void load_pedal_map_from_nvm() noexcept;
void load_xtau_calibration_from_nvm() noexcept;

/**
 * Load map/corr pages at boot (same order as former main_stm32).
 * @param cal_layout_ok when false, skip VE/spark/lambda/axes (layout gate).
 */
void nvm_boot_load_tables(bool cal_layout_ok) noexcept;

}  // namespace ems::app
