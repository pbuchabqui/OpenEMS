/**
 * @file ecu_sched_encoder_priv.h
 * @brief Estado partilhado entre os .cpp do dispatcher TIM2 (encoder-only).
 *
 * API pública continua em ecu_sched.h. Não incluir fora de
 * ecu_sched_encoder_*.cpp.
 *
 * Partes: omega / phase / queue / heartbeat / builders.
 */
#pragma once

#include "engine/ecu_sched_internal.h"

#include <stdint.h>

// TIM_SR_CC3IF / TIM_DIER_CC3IE (mocks de host-test) vêm de
// ecu_sched_internal.h, incluído acima — definição única.

namespace ems::engine::sched_internal::encoder {

void recompute_presync(uint32_t now_raw) noexcept;
void try_arm_sequential_due(uint32_t now_raw) noexcept;
uint32_t seq_arm_success_count(void) noexcept;
void seq_arm_success_count_test_reset(void) noexcept;
void refresh_pending_omega_spans(uint32_t now_raw) noexcept;
void clear_cyl_arm_latches(void) noexcept;
uint32_t min_lead_counts(void) noexcept;
uint32_t duration_ticks_to_span_counts(uint32_t duration_ticks) noexcept;

#if defined(EMS_HOST_TEST)
uint32_t engine_deg_to_counts_in_rev(uint32_t engine_angle_deg) noexcept;
uint32_t rev_target_to_absolute(uint32_t target_counts_in_rev,
                                uint32_t now_raw) noexcept;
uint32_t engine_deg720_to_absolute(uint32_t engine_angle_deg,
                                   uint32_t now_raw) noexcept;
#endif

}  // namespace ems::engine::sched_internal::encoder

// Queue internals used by builders (arm / retarget / pending-cyl scan).
#define ENC_EVT_QUEUE_SIZE 48U

struct EncSchedEvent {
    uint32_t timestamp;
    uint8_t  channel;
    uint8_t  high;
};

extern EncSchedEvent g_enc_evt_queue[ENC_EVT_QUEUE_SIZE];
extern volatile uint8_t g_enc_evt_count;
extern volatile uint32_t g_enc_dbg_insert_count;
extern volatile uint32_t g_enc_dbg_execute_count;

void enc_evt_insert(uint32_t ts, uint8_t channel, uint8_t high) noexcept;
void arm_channel_with_lead(uint8_t ch, uint32_t target_counts,
                           uint8_t action, uint32_t min_lead) noexcept;

// Shared between phase (invalidate/reset) and heartbeat (confirm gate).
extern uint8_t g_cmp_confirm_count;

// Shared between heartbeat (handoff) and builders (recompute_presync).
extern uint8_t g_enc_last_builder_was_sequential;

// Written by builders, reset/read by heartbeat test hooks.
extern volatile uint32_t g_enc_seq_min_lead_skip_count;
extern volatile uint32_t g_enc_omega_refresh_count;
