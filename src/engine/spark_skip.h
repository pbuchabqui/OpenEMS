#pragma once

#include <cstdint>

namespace ems::engine {

// ── Spark-skip soft limiter (estilo rusEFI SoftSparkLimiter) ─────────────────
//
// Redução progressiva de torque por proporção de faíscas saltadas, aplicada
// VIA a máscara de inibição IGN já existente do scheduler (purge seguro +
// bobina LOW) — o núcleo de timing não é tocado: saltar = não armar o evento.
//
// Padrão Bresenham por revolução: num 4-cil 4T disparam ~2 cilindros por
// volta; a cada volta o acumulador soma ratio×2 e o overflow (0..2) define
// quantos cilindros entram na máscara, com rotação para distribuir o salto
// entre cilindros (sem aquecer sempre o mesmo). A atribuição por cilindro é
// aproximada (a máscara não conhece a ordem de disparo da meia-volta), mas o
// RATIO médio é exacto — suficiente para limitador/TC suave.
//
// Ratio clampado a 50% (128 Q8): acima disso o corte duro de combustível do
// rev-limiter é o mecanismo correcto.

// Alvo de salto em Q8 (0 = off, 128 = 50% máx; valores acima clampam a 128).
void spark_skip_set_ratio_q8(uint8_t ratio_q8) noexcept;
uint8_t spark_skip_get_ratio_q8() noexcept;

// Chamar UMA vez por revolução (edge de wrap do tooth_index, contexto main).
// Actualiza a máscara devolvida por spark_skip_mask().
void spark_skip_on_rev() noexcept;

// Máscara de cilindros a inibir nesta revolução (bit0=cyl0..bit3=cyl3).
// OR-ar com as outras fontes antes de ecu_sched_set_ign_inhibit_mask().
uint8_t spark_skip_mask() noexcept;

// Zera acumulador/máscara/rotação (ratio inclusive).
void spark_skip_reset() noexcept;

}  // namespace ems::engine
