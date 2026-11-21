#pragma once
/**
 * @file app/nvm_boot.h
 * Relocate-only NVM calibration loaders (hygiene PR-09).
 * Called from openems_init() after page0 load; behaviour preserved.
 */
#include <cstdint>

namespace ems::app {

bool page_range_is_zero(const uint8_t* data, uint16_t off, uint16_t len) noexcept;
bool page_range_is_erased(const uint8_t* data, uint16_t off, uint16_t len) noexcept;
bool page_is_erased(const uint8_t* data, uint16_t len) noexcept;
// Página em branco: apagada (0xFF, placa nova) OU toda zero. Uma página de zeros com CRC válido (gravada de
// um buffer vazio, ou o NVM de host) não é calibração: aplicá-la zera parâmetros e o limitador de rotação
// cai no mínimo de 1000 RPM. O boot só aplica a página 0 se ela não estiver em branco.
bool page_is_blank(const uint8_t* data, uint16_t len) noexcept;

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
