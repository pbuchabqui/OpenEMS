/**
 * @file test/run_all.cpp
 * Host-test entry: fixed suite order (hygiene PR-06). PASS count must stay exact.
 */
#include <cstdio>
#include "test/harness.h"
#include "test/suite_registry.h"

int main(void) {
    printf("OpenEMS Host Regression Tests\n");
    printf("============================================================\n");

    // ── ETB Driver ────────────────────────────────────────────────────────────
    printf("\n=== ETB DRIVER ===");
    test_etb_driver_adc_to_percent();
    test_etb_driver_init_and_state();
    test_etb_driver_init_fault_tps1_open();
    test_etb_driver_init_fault_tps2_short();
    test_etb_driver_read_sensors_valid();
    test_etb_driver_read_sensors_mismatch();
    test_etb_driver_read_sensors_null();
    test_etb_driver_set_motor_pwm();
    test_etb_driver_shutdown();
    test_etb_driver_clear_fault();

    // ── ETB Control ───────────────────────────────────────────────────────────
    printf("\n=== ETB CONTROL ===");
    test_etb_set_get_drive_mode();
    test_etb_is_ready();
    test_etb_enter_limp_mode();
    test_etb_set_idle_control_and_spark_trim();
    test_etb_get_throttle_position();
    test_etb_control_loop_rpm_cutoff();
    test_etb_control_loop_sensor_fault_triggers_limp();

    // ── Torque Manager ────────────────────────────────────────────────────────
    printf("\n=== TORQUE MANAGER ===");
    test_torque_manager_init();
    test_torque_manager_enter_limp();
    test_torque_manager_set_get_config();
    test_torque_manager_set_config_null();
    test_torque_manager_loop_normal_pedal();
    test_torque_manager_loop_rpm_hard_cut();
    test_torque_manager_loop_rpm_progressive_cut();
    test_torque_manager_loop_limp_via_input();
    test_torque_manager_loop_idle_mode();
    test_torque_manager_loop_traction_control();
    test_torque_manager_loop_null_guards();
    test_torque_manager_loop_speed_limiter();

    // ── CKP Decoder ───────────────────────────────────────────────────────────
    printf("\n=== CKP DECODER / SYNC ===");
    test_ckp_rpm_math();
    test_ckp_initial_state();
    test_ckp_half_sync();
    test_ckp_full_sync();
    test_ckp_tooth_index_increments();
    test_ckp_instant_rpm_360();
    test_ckp_loss_of_sync_too_many_teeth();
    test_ckp_loss_of_sync_early_gap();
    test_ckp_noise_rejection();
    test_ckp_stall_poll();
    test_ckp_phantom_rpm_unsync();
    test_ckp_rpm_jump_recovery();
    test_ckp_stall_poll_no_false_positive();
    test_ckp_seed_arm_disarm();

    // ── Sensors ───────────────────────────────────────────────────────────────
    printf("\n=== SENSORS ===");
    test_sensors_validate_range();
    test_sensors_validate_values();
    test_sensors_health_status();
    test_sensors_calibration();
    test_sensors_tick_100ms_clt_iat();
    test_sensors_maf_freq_capture();

    // ── Fuel Calc ─────────────────────────────────────────────────────────────
    printf("\n=== FUEL CALC ===");
    test_fuel_calc_req_fuel_us();
    test_fuel_calc_base_pw();
    test_fuel_apply_lambda_target();
    test_fuel_apply_trim();
    test_fuel_calc_final_pw();
    test_fuel_corr_functions();
    test_fuel_decel_cut();
    test_fuel_baro();

    // ── Ignition Calc ─────────────────────────────────────────────────────────
    printf("\n=== IGN CALC ===");
    test_ign_iat_correction();
    test_ign_clt_correction();
    test_ign_antijerk();
    test_ign_clamp_and_total_advance();
    test_ign_dwell();

    // ── Auxiliaries ───────────────────────────────────────────────────────────
    printf("\n=== AUXILIARIES ===");
    test_aux_init_and_idle();
    test_aux_pump_prime();
    test_aux_ticks_no_crash();

    // ── Knock ─────────────────────────────────────────────────────────────────
    printf("\n=== KNOCK ===");
    test_knock_init_and_threshold();
    test_knock_window();
    test_knock_detection_and_recovery();

    // ── Fuel Calc — Segunda Fase ──────────────────────────────────────────────
    printf("\n=== FUEL CALC (fase 2) ===");
    test_fuel_table_lookups();
    test_fuel_default_req_and_base_default();
    test_fuel_default_fast();
    test_fuel_corr_warmup();
    test_fuel_ae();
    test_fuel_adaptives_reset();
    test_fuel_lambda_delay();
    test_fuel_stft();
    test_fuel_stft_delayed();
    test_injector_scurve();
    test_fuel_delta_p_compensation();
    test_fuel_ltft();
    test_ltft_adapt_enable();
    test_fuel_trim_dtcs();
    test_fuel_ltft_authority();
    test_fuel_closed_loop_gates();
    test_fuel_ltft_apply_nearest();
    test_fuel_ltft_accum();
    test_fuel_ltft_accum_commit_ve();
    test_fuel_ltft_center_gate();
    test_fuel_inj_two_slope();

    // ── Ign Calc — Segunda Fase ───────────────────────────────────────────────
    printf("\n=== IGN CALC (fase 2) ===");
    test_ign_get_advance();
    test_ign_dwell_vbatt_rpm();
    test_ign_idle_spark_correction();

    // ── ETB Control C++ ns ───────────────────────────────────────────────────
    printf("\n=== ETB CONTROL (C++ ns) ===");
    test_etb_cpp_update();

    // ── Torque Manager C++ ns ──────────────────────────────────────────────
    printf("\n=== TORQUE MANAGER (C++ ns) ===");
    test_torque_manager_cpp_update();
    test_launch_tc_page0_roundtrip();

    // ── CKP — Segunda Fase ───────────────────────────────────────────────────
    printf("\n=== CKP (fase 2) ===");
    test_ckp_seed_confirmed();
    test_ckp_seed_rejected();
    test_ckp_cmp_glitch_count();

    // ── Sensors — Segunda Fase ───────────────────────────────────────────────
    printf("\n=== SENSORS (fase 2) ===");
    test_sensors_on_tooth();
    test_sensors_tick_50ms();
    test_sensors_set_range();
    test_sensors_etb_harness_present();
    test_sensors_table_entry_setters();

    // ── Knock — Segunda Fase ──────────────────────────────────────────────────
    printf("\n=== KNOCK (fase 2) ===");
    test_knock_window_cycle_end();
    test_knock_save_to_nvm();

    // ── Auxiliaries — Segunda Fase ────────────────────────────────────────────
    printf("\n=== AUXILIARIES (fase 2) ===");
    test_aux_test_getters();

    // ── Timer HAL ────────────────────────────────────────────────────────────
    printf("\n=== TIMER HAL ===");
    test_timer_stubs();
    test_out_pins_bsrr_rgt6();

    // ── TABLE3D ──────────────────────────────────────────────────────────
    printf("\n=== TABLE3D ===");
    test_table3d_all();

    // ── ECU SCHED ───────────────────────────────────────────────────────
    printf("\n=== ECU SCHED ===");
    test_ecu_sched_setters();
    test_ecu_sched_angle_table();
    test_ecu_sched_wasted_to_sequential();
    test_ecu_sched_cmp_revalidation_after_sync_loss();
    test_ecu_sched_noise_rejects_sequential();
    test_ecu_sched_recovers_after_fallback();
    test_ecu_sched_inhibit_masks();
    test_ecu_sched_mspark();
    test_ecu_sched_eoi_targeting();
    test_eoi_blend();
    test_ecu_sched_presync();
    test_ecu_sched_dwell_watchdog();

    // ── QUICK CRANK ─────────────────────────────────────────────────────
    printf("\n=== QUICK CRANK ===");
    test_quick_crank_all();

    // ── TRANSIENT FUEL ──────────────────────────────────────────────────
    printf("\n=== TRANSIENT FUEL ===");
    test_transient_fuel_all();

    // ── MAP ESTIMATOR ───────────────────────────────────────────────────
    printf("\n=== MAP ESTIMATOR ===");
    test_map_estimator_all();

    // ── MISFIRE DETECT ──────────────────────────────────────────────────
    printf("\n=== MISFIRE DETECT ===");
    test_misfire_all();

    // ── DIAGNOSTIC MANAGER ──────────────────────────────────────────────
    printf("\n=== DIAGNOSTIC MANAGER ===");
    test_diagnostic_manager_all();

    // ── HAL ADC ───────────────────────────────────────────────────────────
    printf("\n=== HAL ADC ===");
    test_hal_adc_all();

    // ── HAL FLASH (NVM) ─────────────────────────────────────────────────
    printf("\n=== HAL FLASH (NVM) ===");
    test_hal_flash_all();

    // ── XTAU AUTOCALIB ──────────────────────────────────────────────────
    printf("\n=== XTAU AUTOCALIB ===");
    test_xtau_autocalib_all();

    // ── ECU SCHED FASE 2 ────────────────────────────────────────────────
    printf("\n=== ECU SCHED (fase 2) ===");
    test_ecu_sched_hardware_init();
    test_ecu_sched_ccr_write();
    test_ecu_sched_late_events();
    test_ecu_sched_golden_min_lead_timestamp();
    test_ecu_sched_golden_far_target_timestamp();
    test_ecu_sched_golden_dispatch_past_counts_late();
    test_ecu_sched_golden_queue_sorted();
    test_ecu_sched_golden_seq_angle_table_size();
    test_ecu_sched_mspark_angle_table_margin();
    test_ecu_sched_golden_dispatch_identity();
    test_ecu_sched_dwell_watchdog_fires();
    test_ecu_sched_inj_watchdog_fires();
    test_ecu_sched_presync_table();

    // ── VERIFICAÇÃO MATEMÁTICA ─────────────────────────────────────────────
    printf("\n=== VERIFICAÇÃO MATEMÁTICA ===");
    test_math_req_fuel();
    test_math_base_pw();
    test_math_lambda_pw();
    test_math_table3d_bilinear();
    test_math_corrections();
    test_math_stft_gains();
    test_math_inj_scheduler_ticks();
    test_math_xtau_convergence();
    test_math_production_tables();
    test_math_misfire_threshold();
    test_trigger_offset();

    // ── CKP FASE 2 (snap fields, prime, phase_A, tooth_index) ─────────────
    printf("\n=== CKP (fase 3) ===");
    test_ckp_prime_on_tooth();
    test_ckp_snap_fields();
    test_ckp_tooth_index_progression();
    test_ckp_phase_toggle();

    // ── UI PROTOCOL / TUNERSTUDIO ENVELOPE ────────────────────────────────
    printf("\n=== UI PROTOCOL / TS ENVELOPE ===");
    test_crc32_vectors();
    test_legacy_protocol_regression();
    test_ts_envelope_basic();
    test_ts_envelope_crc_reject();
    test_ts_envelope_read_write_burn();
    test_ts_envelope_burn_gate();
    test_ts_axes_page();
    test_ts_envelope_canid_forms();
    test_och_launch_tc_status();
    test_ts_envelope_signature_via_r();
    test_ts_whole_page_800();
    test_adaptives_reset_cmd_z();
    test_ltft_apply_cmd_y();
    test_ltft_hit_matches_ve_dominant_cell();
    test_ltft_accum_page12();
    test_ltft_page_offsets_20();

    printf("\n=== OUTPUT TEST (teste de saídas) ===");
    test_output_test_enter_gate();
    test_output_test_fire_inj();
    test_output_test_busy_window();
    test_output_test_fire_ign_watchdog();
    test_output_test_rpm_abort();
    test_output_test_keepalive_timeout();
    test_output_test_suspends_aux();

    // ── Summary ───────────────────────────────────────────────────────────────
    printf("\n============================================================\n");
    printf("Results: %d PASS  %d FAIL\n", g_pass, g_fail);

    return (g_fail == 0) ? 0 : 1;
}
