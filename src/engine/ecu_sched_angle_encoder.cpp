/**
 * @file ecu_sched_angle_encoder.cpp
 * @brief Não compilado. O dispatcher TIM2 foi partido (move-only) em:
 *
 *   ecu_sched_encoder_omega.cpp      — estimador ω
 *   ecu_sched_encoder_phase.cpp      — âncora CMP / phase_valid
 *   ecu_sched_encoder_queue.cpp      — fila TIM2/CH3
 *   ecu_sched_encoder_heartbeat.cpp  — heavy tick + subtick
 *   ecu_sched_encoder_builders.cpp   — presync + try_arm + °→counts
 *   ecu_sched_encoder_priv.h         — estado partilhado interno
 *
 * API pública: ecu_sched.h
 */
