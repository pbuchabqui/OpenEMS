# Graph Report - .  (2026-06-18)

## Corpus Check
- 81 files · ~81,906 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 1002 nodes · 1442 edges · 73 communities (42 shown, 31 thin omitted)
- Extraction: 96% EXTRACTED · 4% INFERRED · 0% AMBIGUOUS · INFERRED: 51 edges (avg confidence: 0.8)
- Token cost: 0 input · 0 output

## Community Hubs (Navigation)
- [[_COMMUNITY_ECU Scheduler & Timing|ECU Scheduler & Timing]]
- [[_COMMUNITY_Fuel Calculation & Lambda|Fuel Calculation & Lambda]]
- [[_COMMUNITY_Crankshaft Position Sensor|Crankshaft Position Sensor]]
- [[_COMMUNITY_Sensor Drivers & Faults|Sensor Drivers & Faults]]
- [[_COMMUNITY_ETB Control & PID|ETB Control & PID]]
- [[_COMMUNITY_USB CDC Driver|USB CDC Driver]]
- [[_COMMUNITY_Dashboard Protocol|Dashboard Protocol]]
- [[_COMMUNITY_Engine Config & Math|Engine Config & Math]]
- [[_COMMUNITY_Knock Detection & Correction|Knock Detection & Correction]]
- [[_COMMUNITY_Dashboard Frontend (JS)|Dashboard Frontend (JS)]]
- [[_COMMUNITY_UI Protocol & Pages|UI Protocol & Pages]]
- [[_COMMUNITY_Dashboard Server (Python)|Dashboard Server (Python)]]
- [[_COMMUNITY_Auxiliaries & Idle|Auxiliaries & Idle]]
- [[_COMMUNITY_Flash HAL & NVM|Flash HAL & NVM]]
- [[_COMMUNITY_ADC Drivers|ADC Drivers]]
- [[_COMMUNITY_Community 15|Community 15]]
- [[_COMMUNITY_Community 16|Community 16]]
- [[_COMMUNITY_Community 17|Community 17]]
- [[_COMMUNITY_Community 18|Community 18]]
- [[_COMMUNITY_Community 19|Community 19]]
- [[_COMMUNITY_Community 20|Community 20]]
- [[_COMMUNITY_Community 21|Community 21]]
- [[_COMMUNITY_Community 22|Community 22]]
- [[_COMMUNITY_Community 23|Community 23]]
- [[_COMMUNITY_Community 25|Community 25]]
- [[_COMMUNITY_Community 26|Community 26]]
- [[_COMMUNITY_Community 27|Community 27]]
- [[_COMMUNITY_Community 28|Community 28]]
- [[_COMMUNITY_Community 29|Community 29]]
- [[_COMMUNITY_Community 30|Community 30]]
- [[_COMMUNITY_Community 34|Community 34]]
- [[_COMMUNITY_Community 35|Community 35]]
- [[_COMMUNITY_Community 36|Community 36]]
- [[_COMMUNITY_Community 37|Community 37]]
- [[_COMMUNITY_Community 38|Community 38]]
- [[_COMMUNITY_Community 39|Community 39]]
- [[_COMMUNITY_Community 40|Community 40]]
- [[_COMMUNITY_Community 41|Community 41]]
- [[_COMMUNITY_Community 42|Community 42]]
- [[_COMMUNITY_Community 43|Community 43]]
- [[_COMMUNITY_Community 44|Community 44]]
- [[_COMMUNITY_Community 45|Community 45]]
- [[_COMMUNITY_Community 46|Community 46]]
- [[_COMMUNITY_Community 47|Community 47]]
- [[_COMMUNITY_Community 48|Community 48]]
- [[_COMMUNITY_Community 49|Community 49]]
- [[_COMMUNITY_Community 50|Community 50]]
- [[_COMMUNITY_Community 51|Community 51]]
- [[_COMMUNITY_Community 52|Community 52]]
- [[_COMMUNITY_Community 53|Community 53]]
- [[_COMMUNITY_Community 54|Community 54]]
- [[_COMMUNITY_Community 55|Community 55]]
- [[_COMMUNITY_Community 56|Community 56]]
- [[_COMMUNITY_Community 57|Community 57]]
- [[_COMMUNITY_Community 58|Community 58]]
- [[_COMMUNITY_Community 59|Community 59]]
- [[_COMMUNITY_Community 60|Community 60]]
- [[_COMMUNITY_Community 61|Community 61]]
- [[_COMMUNITY_Community 63|Community 63]]
- [[_COMMUNITY_Community 71|Community 71]]
- [[_COMMUNITY_Community 72|Community 72]]

## God Nodes (most connected - your core abstractions)
1. `$()` - 38 edges
2. `AuxState` - 17 edges
3. `clamp_i16()` - 17 edges
4. `handle_ctr()` - 16 edges
5. `OpenEMSLink` - 14 edges
6. `parse_byte()` - 12 edges
7. `ckp_tim5_ch1_isr()` - 12 edges
8. `main()` - 12 edges
9. `openems_init()` - 12 edges
10. `DecoderState` - 11 edges

## Surprising Connections (you probably didn't know these)
- `calc_idle_spark_correction_deg()` --calls--> `clamp_i16()`  [INFERRED]
  src/engine/ign_calc.cpp → src/engine/auxiliaries.cpp
- `sync_table_from_page()` --calls--> `etb_apply_idle_calibration()`  [INFERRED]
  src/app/ui_protocol.cpp → src/engine/etb_control.cpp
- `etb_control_update()` --calls--> `clamp_i16()`  [INFERRED]
  src/engine/etb_control.cpp → src/engine/auxiliaries.cpp
- `apply_fuel_trim_pw_us()` --calls--> `clamp_i16()`  [INFERRED]
  src/engine/fuel_calc.cpp → src/engine/auxiliaries.cpp
- `clamp_advance_deg()` --calls--> `clamp_i16()`  [INFERRED]
  src/engine/ign_calc.cpp → src/engine/auxiliaries.cpp

## Import Cycles
- None detected.

## Communities (73 total, 31 thin omitted)

### Community 0 - "ECU Scheduler & Timing"
Cohesion: 0.05
Nodes (45): angle_to_tooth_event(), arm_channel(), calculate_presync_revolution(), Calculate_Sequential_Cycle(), clamp_inj_pw_to_ivc(), clear_all_events_and_drive_safe_outputs(), ECU_Hardware_Init(), ecu_sched_commit_calibration() (+37 more)

### Community 1 - "Fuel Calculation & Lambda"
Cohesion: 0.05
Nodes (50): clamp_i16(), apply_fuel_trim_pw_us(), calc_ae_pw_us(), calc_base_pw_us_default(), calc_final_pw_us(), calc_fuel_pw_us_default_fast(), calc_req_fuel_us(), closed_loop_allowed() (+42 more)

### Community 2 - "Crankshaft Position Sensor"
Cohesion: 0.07
Nodes (38): ckp_snapshot(), ckp_stall_poll(), ckp_test_rpm_x10_from_period_ns(), ckp_tim5_ch1_isr(), ckp_tim5_ch2_isr(), classify_tooth(), DecoderState, cmp_confirms (+30 more)

### Community 3 - "Sensor Drivers & Faults"
Cohesion: 0.07
Nodes (34): apply_fault(), avg_n(), FaultTracker, active, consecutive_bad, range, iir(), init_tables() (+26 more)

### Community 4 - "ETB Control & PID"
Cohesion: 0.06
Nodes (23): apply_ramp_limit(), calculate_pid(), etb_apply_idle_calibration(), etb_control_init(), etb_control_loop(), etb_control_reset(), etb_control_update(), etb_enter_limp_mode() (+15 more)

### Community 5 - "USB CDC Driver"
Cohesion: 0.10
Nodes (39): bdtable_count_rx(), bdtable_set(), bdtable_set_count_rx_cfg(), bdtable_set_count_tx(), build_str_desc(), dbg_stage(), ep0_send(), ep0_send_zlp() (+31 more)

### Community 6 - "Dashboard Protocol"
Cohesion: 0.06
Nodes (23): _apply_scale(), autodetect_port(), decode_fields(), decode_grid_i16(), decode_grid_i8(), decode_grid_u8(), decode_ltft(), decode_pedal_maps() (+15 more)

### Community 7 - "Engine Config & Math"
Cohesion: 0.07
Nodes (36): engine(), engine(), torque_manager_init(), ems(), ems(), namespace, namespace, namespace (+28 more)

### Community 8 - "Knock Detection & Correction"
Cohesion: 0.07
Nodes (24): corr_vbatt(), clamp_u16(), knock_adc_update(), knock_cycle_complete(), knock_init(), knock_set_adc_threshold(), knock_test_set_adc_raw(), knock_window_cycle_end() (+16 more)

### Community 9 - "Dashboard Frontend (JS)"
Cohesion: 0.07
Nodes (30): $(), api(), axisLookup(), BOOST_DEFAULTS, BOOST_GEAR_COLORS, BOOST_GEAR_LABELS, BOOST_RPM_AXIS, buildCanRxUI() (+22 more)

### Community 10 - "UI Protocol & Pages"
Cohesion: 0.14
Nodes (29): burn_page_to_flash(), clear_page_dirty(), command_bounds_ok(), editable_page_bit(), enter_critical(), exit_critical(), handle_read_done(), handle_write_done() (+21 more)

### Community 11 - "Dashboard Server (Python)"
Cohesion: 0.09
Nodes (21): api_burn(), api_dirty(), api_log_export(), api_log_start(), api_log_stop(), api_read_page(), api_wbo2_can_id_get(), api_wbo2_can_id_set() (+13 more)

### Community 12 - "Auxiliaries & Idle"
Cohesion: 0.12
Nodes (24): auxiliaries_idle_target_rpm_x10(), auxiliaries_init(), auxiliaries_set_key_on(), auxiliaries_test_reset(), auxiliaries_tick_10ms(), auxiliaries_tick_20ms(), axis_frac_q8_u16(), axis_index_u16() (+16 more)

### Community 13 - "Flash HAL & NVM"
Cohesion: 0.12
Nodes (14): crc32_update(), flash_erase_sector(), flash_lock_bank2(), flash_unlock_bank2(), flash_wait_ready(), flash_write_words(), nvm_clear_runtime_seed(), nvm_flush_adaptive_maps() (+6 more)

### Community 14 - "ADC Drivers"
Cohesion: 0.09
Nodes (13): AdcPrimaryChannel, AdcSecondaryChannel, adc_enable(), adc_init(), adc_prepare_for_config(), adc_primary_read(), adc_secondary_read(), adc_test_set_raw_primary() (+5 more)

### Community 15 - "Community 15"
Cohesion: 0.10
Nodes (16): AdvanceCorrections, build_ign_schedule(), calc_dwell_angle_x10(), calc_dwell_start_deg_x10(), calc_idle_spark_correction_deg(), calc_total_advance(), clamp_advance_deg(), dwell_ms_x10_from_vbatt() (+8 more)

### Community 16 - "Community 16"
Cohesion: 0.16
Nodes (16): can_stack_init(), can_stack_lambda_milli_safe(), can_stack_process(), can_stack_test_reset(), can_stack_wbo2_fresh(), clamp_u8(), clamp_u8_i32(), elapsed() (+8 more)

### Community 17 - "Community 17"
Cohesion: 0.16
Nodes (17): afterstart_mult_x256(), detect_cranking(), enter_critical(), exit_critical(), interp_u16(), P2, x, y (+9 more)

### Community 18 - "Community 18"
Cohesion: 0.13
Nodes (19): transient_fuel_reset(), transient_fuel_xtau_update(), calculate_ideal_xtau(), lambda_error_get_average(), lambda_error_push(), lambda_error_reset(), LambdaErrorSample, error_x1000 (+11 more)

### Community 19 - "Community 19"
Cohesion: 0.12
Nodes (14): etb_get_drive_mode(), etb_set_drive_mode(), pedal_map_lookup(), torque_manager_get_config(), torque_manager_loop(), torque_manager_set_config(), torque_manager_update(), etb_drive_mode_t (+6 more)

### Community 20 - "Community 20"
Cohesion: 0.16
Nodes (13): DiagnosticCode, DiagnosticEvent, clear_fault(), get_event(), get_highest_severity(), get_recovery_state(), is_fault_active(), is_system_ready() (+5 more)

### Community 21 - "Community 21"
Cohesion: 0.12
Nodes (17): AuxState, fan_on, key_on, key_on_ms, phase_prev, pump_on, rpm_zero_since_ms, time_ms (+9 more)

### Community 22 - "Community 22"
Cohesion: 0.12
Nodes (17): App.js Main Script, Charts Container, Diagnostics Panel, Gauges Panel, OpenEMS Dashboard, Sidebar Navigation, Status LEDs, Status Bar (+9 more)

### Community 23 - "Community 23"
Cohesion: 0.21
Nodes (14): can_rx_gear(), can_rx_map_get(), can_rx_map_process(), can_rx_map_set(), can_rx_speed_kmh(), extract(), SignalState, def (+6 more)

### Community 25 - "Community 25"
Cohesion: 0.26
Nodes (7): CanFrame, can0_rx_pop(), can0_tx(), can_test_inject_rx(), can_test_pop_tx(), rx_fifo_pop(), rx_fifo_push()

### Community 26 - "Community 26"
Cohesion: 0.20
Nodes (8): CylTdcPos, phase_A, tdc_tooth, evaluate_window(), misfire_init(), misfire_on_tooth(), misfire_reset(), CkpSnapshot

### Community 27 - "Community 27"
Cohesion: 0.31
Nodes (9): analyse_csv(), main(), parse_edge_row(), parse_live_row(), parse_pulse_row(), print_live_header(), print_live_row(), Lê um CSV gravado em modo 'pulse' e verifica se os valores estão     dentro dos (+1 more)

### Community 28 - "Community 28"
Cohesion: 0.42
Nodes (7): cs_high(), cs_low(), spi2_txrx(), tle8888_init(), tle8888_poll_diag(), tle_read(), tle_write()

### Community 29 - "Community 29"
Cohesion: 0.39
Nodes (7): engine_config_apply(), engine_config_load(), engine_config_serialize(), engine_config_valid(), read_u16_le(), write_u16_le(), EngineConfigRam

### Community 30 - "Community 30"
Cohesion: 0.29
Nodes (7): ckp_gen_micropython.py — Gerador CKP (60-2) + CMP para bancada OpenEMS ━━━━━━━━━, Sinaliza paragem (eficaz no próximo ciclo)., Período de um dente em µs: 60 000 000 / (rpm × 60)., Inicia a geração do sinal CKP + CMP em modo bloqueante.      Parâmetros     ----, start(), stop(), _tooth_period_us()

## Knowledge Gaps
- **157 isolated node(s):** `recommendations`, `arduino-cli`, `namespace`, `def`, `value` (+152 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **31 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `clamp_i16()` connect `Fuel Calculation & Lambda` to `ETB Control & PID`, `Engine Config & Math`, `Knock Detection & Correction`, `Auxiliaries & Idle`, `Community 15`?**
  _High betweenness centrality (0.100) - this node is a cross-community bridge._
- **Why does `main()` connect `Engine Config & Math` to `ECU Scheduler & Timing`, `Fuel Calculation & Lambda`, `Community 18`, `Knock Detection & Correction`?**
  _High betweenness centrality (0.098) - this node is a cross-community bridge._
- **Why does `openems_init()` connect `Engine Config & Math` to `Community 18`, `ETB Control & PID`?**
  _High betweenness centrality (0.044) - this node is a cross-community bridge._
- **Are the 13 inferred relationships involving `clamp_i16()` (e.g. with `etb_control_update()` and `apply_fuel_trim_pw_us()`) actually correct?**
  _`clamp_i16()` has 13 INFERRED edges - model-reasoned connections that need verification._
- **What connects `recommendations`, `arduino-cli`, `namespace` to the rest of the system?**
  _177 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `ECU Scheduler & Timing` be split into smaller, more focused modules?**
  _Cohesion score 0.05010351966873706 - nodes in this community are weakly interconnected._
- **Should `Fuel Calculation & Lambda` be split into smaller, more focused modules?**
  _Cohesion score 0.0506558118498417 - nodes in this community are weakly interconnected._