#pragma once

#include <cstdint>

namespace ems::engine {

// ── Spark-skip soft limiter (estilo rusEFI SoftSparkLimiter) ─────────────────
//
// Redução progressiva de torque por proporção de faíscas saltadas, aplicada
// VIA a máscara de inibição IGN já existente do scheduler (purge seguro +
// bobina LOW) — o núcleo de timing não é tocado: saltar = não armar o evento.
//
// Padrão Bresenham por ciclo (720°): num 4-cil 4T cada cilindro dispara 1x por
// ciclo; a cada 2 voltas o acumulador soma ratio×4 e o overflow (0..4) define
// quantos cilindros entram na máscara, que fica valendo pelo ciclo inteiro, com
// rotação para distribuir o salto entre cilindros. Como cada cilindro mascarado
// perde exatamente uma faísca por ciclo, o ratio entregue é o pedido.
//
// Ratio clampado a 50% (128 Q8): acima disso o corte duro de combustível do
// rev-limiter é o mecanismo correcto.

// Alvo de salto em Q8 (0 = off, 128 = 50% máx; valores acima clampam a 128).
void spark_skip_set_ratio_q8(uint8_t ratio_q8) noexcept;
uint8_t spark_skip_get_ratio_q8() noexcept;

// Chamar UMA vez por revolução (edge de wrap do tooth_index, contexto main).
// `cycle_start` = a revolução que abre o ciclo de 720° (fase do came, ckp phase_A):
// só na borda de subida dele a máscara é recalculada (fase congelada não recalcula
// a cada volta; após 4 chamadas sem borda a máscara é zerada); na outra é mantida. Preso à fase, não a uma
// contagem de chamadas, para não derivar após perda de sync ou dentes espúrios.
// Actualiza a máscara devolvida por spark_skip_mask().
void spark_skip_on_rev(bool cycle_start) noexcept;

// Sem sync pleno: zera a máscara (não há fase confiável) até o próximo cycle_start.
void spark_skip_clear_mask() noexcept;

// Máscara de cilindros a inibir nesta revolução (bit0=cyl0..bit3=cyl3).
// OR-ar com as outras fontes antes de ecu_sched_set_ign_inhibit_mask().
uint8_t spark_skip_mask() noexcept;

// Zera acumulador/máscara/rotação (ratio inclusive).
void spark_skip_reset() noexcept;

}  // namespace ems::engine
