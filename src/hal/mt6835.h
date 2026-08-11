#pragma once
/**
 * @file mt6835.h
 * @brief Driver SPI do encoder magnético absoluto MagnTek MT6835 — leitura de
 *        ângulo no key-on + configuração de PPR. NÃO é o caminho de leitura em
 *        tempo real (isso é hardware puro, ver tim2_encoder_init() em
 *        hal/timer.h) — este driver só fala com o sensor por SPI, de baixa
 *        frequência (key-on + poll de saúde ocasional).
 *
 * Ver docs/dev/mt6835_encoder_fork.md ("Arquitetura base") para as decisões
 * que este driver implementa: PPR=4096, leitura absoluta no key-on pré-carrega
 * TIM2->CNT.
 *
 * ⚠️ SEM HARDWARE REAL AINDA. O MT6835 não tem footprint na PCB do OpenEMS
 * hoje — este fork é uma exploração pré-hardware. `MT6835_HW_PRESENT` (em
 * mt6835.cpp) fica em 0, mesmo padrão que ewg_driver.cpp usa para o EWG
 * diferido: as funções abaixo compilam e têm assinatura estável, mas não
 * tocam pino nenhum enquanto a flag estiver em 0. O protocolo SPI (frame,
 * CRC, mapa de registradores) está implementado e é a parte que não depende
 * de decisão de hardware — a pinagem/barramento é que fica pendente.
 */

#include <cstdint>

namespace ems::hal {

/// Inicializa o driver: escreve ABZ_RES=4096 PPR (write+readback) e faz uma
/// primeira leitura de ângulo com verificação de CRC. Idempotente.
/// Retorna false se MT6835_HW_PRESENT=0 (ver nota acima) ou se a comunicação
/// falhar.
bool mt6835_init() noexcept;

/// Lê o ângulo absoluto atual (21 bits, 0..2097151 representando 0..360°) via
/// leitura single-byte dos registradores 0x003-0x006 (§7.6.8 do datasheet),
/// com verificação de CRC-8. `out_status` recebe STATUS[2:0] (bit0=overspeed,
/// bit1=weak field, bit2=undervoltage). Retorna false se a comunicação falhar
/// ou o CRC não bater — nesse caso os valores de saída não devem ser usados.
bool mt6835_read_angle_raw21(uint32_t* out_angle21, uint8_t* out_status) noexcept;

/// Converte um ângulo de 21 bits (formato do sensor) para a escala de
/// TIM2->CNT (16384 contagens/volta = 4096 PPR × X4). Uso: pré-carregar o
/// "virabrequim virtual" no key-on, decisão 3 da arquitetura base.
uint32_t mt6835_angle21_to_tim2_counts(uint32_t angle21) noexcept;

/// Espelha MT6835_HW_PRESENT (compile-time, privado a mt6835.cpp) como
/// runtime bool — permite a um chamador (poll de saúde 100ms,
/// main_stm32.cpp) distinguir "sem hardware populado" (skip, não é falha)
/// de "hardware populado mas leitura falhou" (falha real). Host-test: true.
bool mt6835_hw_present() noexcept;

}  // namespace ems::hal
