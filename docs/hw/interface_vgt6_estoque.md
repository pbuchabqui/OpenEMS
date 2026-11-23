# Placa de interface VGT6 — consulta de estoque

> **Status:** vigente · validado 2026-11-22 (consulta por busca na web em 2026-11-22; **instantâneo perecível**, não é estoque ao vivo)

Disponibilidade dos candidatos de `interface_vgt6.md`. Os números vêm de páginas em cache de distribuidores e agregadores, sem horário, e **divergem entre fontes e regiões** (a mesma peça aparece com 0 e com milhares de unidades). Serve para achar risco de BOM, não para comprar: confirme na página do distribuidor antes do pedido e refaça a consulta quando a BOM for fechada.

## Por risco

| Risco | Peça | Observação |
|---|---|---|
| **Alto** | MAX9924UAUB (VR do CKP) | fornecedor único (Analog Devices); ~250 no LCSC (C143633 tem 18 a 251 em páginas diferentes; C5145181, em fita, de 0 a 1.374), de 150 a 2.900 no DigiKey conforme a região; compre reserva cedo |
| **Alto** | MAX9926UAEE (dual, alternativa) | LCSC C1523548 aparece esgotado numa página; DigiKey com ~234 |
| **Alto** | IRLR3110Z (FET de injetor, 100 V) | prazo de 27 semanas no DigiKey; preferir outro FET de 60–100 V ou o IRLR2905 |
| **Médio** | AOD4189 (inversão de polaridade) | AOS original sem estoque visível e prazo de 16–20 semanas no DigiKey; a cópia UMW (LCSC C5346824) tem ~3.700 |
| **Médio** | TPS2553DBVR (limitador de 5 V) | DigiKey México ~21 mil, Canadá 0, Europa esgotado; prever substituto |
| **Médio** | DRV8873HPWPR (se o ETB exigir 10 A) | LCSC C2150604 de 226 a 742 por página; DigiKey Canadá ~1.700 (prazo 6 semanas); não confundir com a variante Q1 |
| **Médio** | AP63205WU-7 (buck 5 V) | LCSC C2071056 de 1.744 a ~68 mil conforme a página; DigiKey com prazo de 12 semanas |
| **Médio** | ADuM3160BRWZ (módulo de isolação USB) | DigiKey US ~8.200 (variante -RL, prazo 10 semanas), bom; sem consulta no Mouser |
| **Baixo** | DRV8871DDAR (ponte do ETB) | LCSC C75864 de 4.300 a 6.500; DigiKey ~5.400 |
| **Baixo** | TJA1051T/3 (CAN) | /1J: ~40 mil no LCSC (C38695), ~6.700 no DigiKey; Mouser /3,118 ~90 mil |
| **Baixo** | SN74AHCT125PWR | LCSC C36365, ~1.000 |
| **Baixo** | SN74AHCT08PWR | LCSC C155175, de 105 a 2.685 por página; DigiKey ~13 mil |
| **Baixo** | SN74LVC1G17DBVR | LCSC C7836, ~91 mil; DigiKey US sem estoque (backorder), outras regiões com 55 a 72 mil |
| **Baixo** | SN74LVC1G86DBVR | DigiKey de 58 a 66 mil nas regiões consultadas; a TI mostra sem estoque na compra direta; **não usar a variante DBVRG4** (descontinuada); sem consulta no LCSC |
| **Baixo** | SMBJ24A (TVS, SMB) | LCSC com várias marcas: MDD C123819 5 mil a 28 mil, Littelfuse C224017 ~19.900, TWGMC C726830 ~64 mil |
| **Baixo** | BAT54S | muitas marcas no LCSC, MDD C408389 ~1,4 milhão |

SMBJ24A (SMB) e SMDJ24A (SMC) têm encapsulamentos diferentes. A especificação usa SMBJ/SMDJ como alternativas; fixe o footprint antes de escolher.

## Não consultado

2N7002 e AO3400A (a busca não achou página de estoque), IRLR2905 com distribuidor e contagem confirmados (só um agregador, ~30 mil), porta-fusível lâmina ATO/ATM, bornes plugáveis (a consulta só listou páginas da RS, sem estoque; Phoenix MSTB 2,5 de 5,08 mm tem numerosas variantes, e algumas são de 5 mm), choke de modo comum e TVS de CAN, ferrites, capacitores de reservatório, MAX9924 em Mouser.

## Ações sugeridas

1. Definir o FET de injetor sem depender do IRLR3110Z (60–100 V, lógico, DPAK) e conferir estoque.
2. Comprar MAX9924 de reserva antes da fabricação, e manter pegada que aceite o MAX9926 como alternativa.
3. Escolher um substituto para o TPS2553 e um para o AOD4189 (a cópia UMW funciona como alternativa só se a ficha técnica dela bater com a da AOS).
4. Refazer esta consulta na hora de fechar a BOM, com a pesquisa direta nas páginas do LCSC, Mouser e DigiKey.
