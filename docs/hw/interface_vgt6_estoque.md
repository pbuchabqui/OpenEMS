# Placa de interface VGT6 — consulta de estoque

> **Status:** vigente · validado 2026-11-23 (consultas por busca na web em 2026-11-22 e 2026-11-23; **instantâneo perecível**, não é estoque ao vivo)

Disponibilidade dos candidatos de `interface_vgt6.md`. Os números vêm de páginas em cache de distribuidores e agregadores, sem horário, e **divergem entre fontes e regiões** (a mesma peça aparece com 0 e com milhares de unidades). Serve para achar risco de BOM, não para comprar: confirme na página do distribuidor antes do pedido e refaça a consulta quando a BOM for fechada.

## Por risco

| Risco | Peça | Observação |
|---|---|---|
| **Alto** | MAX9924UAUB (VR do CKP) | fornecedor único (Analog Devices); Mouser ~277 (+) e ~2.100 (+T); ~250 no LCSC (C143633 tem 18 a 251 em páginas diferentes; C5145181, em fita, de 0 a 1.374), de 150 a 2.900 no DigiKey conforme a região; compre reserva cedo |
| **Alto** | MAX9926UAEE (dual, alternativa) | LCSC C1523548 aparece esgotado numa página; DigiKey com ~234 |
| **Alto** | ACT45B-510-2P (choke de CAN, TDK) | Futura: produção sai da Alemanha para a China, com descontinuação lá até o fim de 2026-11; LCSC (C76584, ~141 mil) marca "não recomendado para novos projetos"; DigiKey ~37.900 com prazo de 20 semanas. Trate como obsoleto e escolha outro choke |
| **Alto** | PESD2CANFD24V/24L (TVS de CAN, Nexperia) | esgotados com backorder no DigiKey; o 27V aparece com 2.753 numa página e 3 em outra; usar equivalente de outro fabricante |
| **Baixo–médio** | IRLR3110Z (FET de injetor, 100 V) | **Revisado em 2026-11-23:** há estoque (DigiKey ≈ 9,3 mil, Arrow ≈ 152 mil, Newark ≈ 25 mil, RS ≈ 5 mil); os 27 semanas e os 12–52 semanas são prazos de fábrica, não de estoque. Risco baixo a médio se comprar do estoque. O IRLR2905 (55 V) não é equivalente com o clamp de ≈ 70 V; o IRLR3410 (100 V) tem RDS(on) de 105 mΩ e a RS o descontinuou |
| **Médio** | IRLR2905TRPBF (FET de injetor, 55 V) | DigiKey ~44 mil, mas uma página dá prazo de 26 semanas e a do Canadá mostra esgotado; Mouser de 6 mil a 11 mil (agregadores); LCSC C3010 só ~290 |
| **Médio** | Porta-fusível Keystone 3544-2 / 3549-2 (mini ATM) | 3549-2: DigiKey 9.400 de fábrica, prazo de 14 semanas; 3544-2: ~228 mil globais na Future, 0 de fábrica; a família 3522-2 é ATO padrão, não mini; confirmar o encaixe do fusível |
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
| **Baixo** | 2N7002 (SOT-23) | LCSC com centenas de listagens (Nexperia C65189 ~633 mil); as marcas diferem em corrente (115–380 mA): confirme a ficha do C escolhido |
| **Baixo** | AO3400A | LCSC C20917 de 148 mil a 319 mil; JLCPCB ~1,98 milhão; DigiKey ~135 mil (prazo de 20 semanas); outras marcas com o mesmo nome têm especificação diferente |
| **Baixo** | BAT54S | muitas marcas no LCSC, MDD C408389 ~1,4 milhão |

SMBJ24A (SMB) e SMDJ24A (SMC) têm encapsulamentos diferentes. A especificação usa SMBJ/SMDJ como alternativas; fixe o footprint antes de escolher.

## Revisão de 2026-11-23 (linhas de maior custo da BOM)

Páginas em cache, sem data, divergentes entre fontes; confirmar ao vivo. Preços para 1 unidade.

| Peça | Estoque e preço | Observação |
|---|---|---|
| MAX9924UAUB+ | DigiKey US$ 10,91; Mouser ≈ 1,2 mil; LCSC C5145181 (variante em fita "+T") US$ 1,89, 482 em estoque | confirmar o part number: "+", "+T" e "/V+" (automotiva, ≈ US$ 18,68 no Mouser) são itens diferentes; fornecedor único |
| DRV8871DDAR | LCSC C75864 US$ 1,04–1,77 (≈ 4,4 mil); Mouser US$ 2,16–2,38 (≈ 7–9 mil); DigiKey ≈ US$ 3 (≈ 5,4 mil) | estoque bom |
| AOD4189 | DigiKey ≈ US$ 0,85, 52–57 mil em estoque (prazo de fábrica de 20 semanas); LCSC C485685 | a linha "sem estoque visível" acima está superada. Ficha AOS (rev. 2008): 40 V, −40 A, RDS(on) ≤ 22 mΩ a −10 V e ≤ 29 mΩ a −4,5 V, limiar máx. 3 V a 250 µA. O VGS máximo da peça não foi conferido (o zener de 15 V assume ±20 V) |
| TPS2553DBVR | LCSC C55266 US$ 0,23 (≈ 25 mil); Mouser e DigiKey ≈ US$ 0,80 (≈ 8–9 mil) | **não usar o TPS2553-1 (LCSC C111738):** trava em sobrecorrente em vez de tentar de novo, e é "não recomendado para novos projetos" |
| IRLR3110ZTRPBF | DigiKey ≈ 9,3 mil; Arrow ≈ 152 mil; Newark ≈ 25 mil; RS ≈ 5 mil | prazo de fábrica de 12 a 52 semanas; sem Mouser e LCSC consultados |

Ordem de grandeza de uma unidade de cada (MAX9924, DRV8871, 2 × AOD4189, TPS2553, sem os FETs de injetor): ≈ US$ 12–17. Não é cotação.

## Não consultado

Bornes plugáveis (a consulta só listou páginas da RS, sem estoque; a família Phoenix MSTB 2,5 de 5,08 mm tem numerosas variantes e algumas são de 5 mm), DLW5BTM (sem resultado), ferrites e capacitores de reservatório.

## Ações sugeridas

1. Trocar o choke ACT45B-510 e o TVS PESD2CANFD por equivalentes disponíveis.
2. Definir o FET de injetor sem depender do IRLR3110Z (60–100 V, lógico, DPAK) e conferir estoque.
3. Comprar MAX9924 de reserva antes da fabricação, e manter pegada que aceite o MAX9926 como alternativa.
4. TPS2553 e AOD4189 têm estoque na revisão de 2026-11-23; a cópia UMW do AOD4189 só vale se a ficha dela bater com a da AOS.
5. Refazer esta consulta na hora de fechar a BOM, com a pesquisa direta nas páginas do LCSC, Mouser e DigiKey.
