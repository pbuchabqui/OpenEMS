#pragma once
// Diagnóstico dos sensores (DTCs) FORA da ISR do dente.
//
// Antes, drv/sensors.cpp reportava/limpava DTCs (MAP/TPS fora de faixa, plausibilidade MAP×TPS, ADC em
// recuperação) dentro da ISR do dente (TIM5, ~12×/rev). Cada report/clear varre a tabela de eventos sob
// CPSID, somando latência ao caminho que agenda INJ/IGN. Agora a ISR só mantém o que é crítico e barato
// (bits de falha, valores de fallback, commit do snapshot) e este poll, chamado do laço principal,
// traduz o estado COMMITADO em DTCs. A latência do DTC passa a ser de até um período do poll (50 ms);
// o corte/limp continua imediato porque é guiado pelos fault_bits.
#include <cstdint>

namespace ems::engine {

// Chamar do laço principal (slot de 50 ms), depois de sensors_tick_50ms().
void sensors_diag_poll() noexcept;

}  // namespace ems::engine
