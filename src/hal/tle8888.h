#pragma once
#include <cstdint>

namespace ems::hal {

void tle8888_init() noexcept;
void tle8888_poll_diag() noexcept;
bool tle8888_ok() noexcept;
uint16_t tle8888_fault_count() noexcept;

// ch 0-3 = INJ1-4 (OUT1-4), 4-7 = IGN1-4.
// Codificação do datasheet Rev 1.2 (OutDiag0 / IgnDiag, 2 bits por canal):
//   0 = sem falha
//   1 = curto ao positivo (sobrecorrente) ou sobretemperatura
//   2 = open load (em off)
//   3 = curto à massa (em off)
// ⚠️ Mudou na reescrita de 2026-07-20: a versão anterior documentava
// 1=open-load / 2=short-GND / 3=short-VBAT, que não corresponde ao datasheet.
uint8_t tle8888_channel_fault(uint8_t ch) noexcept;

// Bitmap: bit N set = canal N com falha
uint8_t tle8888_fault_bitmap() noexcept;

// VRSDiag0 cru: diagnóstico do sensor VR do CKP (curto à massa, curto ao
// positivo, open load directamente nos pinos). Útil na telemetria para
// distinguir "não arranca" de "não arranca PORQUE o CKP está em circuito
// aberto".
uint8_t tle8888_vrs_diag() noexcept;

// Estado bruto do módulo de watchdog (leitura apenas — o driver NÃO serve o
// watchdog; ver a nota no fim de tle8888.cpp).
//   idx 0 = WdStat0   1 = WWDStat   2 = FWDStat0   3 = TECStat   4 = WdDiag
// Num TLE8888-1QK sem serviço de watchdog, os contadores sobem e o CI entra em
// Safe State, desligando injecção e ignição. Expor isto torna essa falha
// diagnosticável em vez de misteriosa.
uint8_t tle8888_wd_status(uint8_t idx) noexcept;

}  // namespace ems::hal
