<!--
  Origem: plano de design elaborado e aprovado em 2026-07-20.
  Persistido no repositório porque ~/.claude/plans/ é área efémera.
  Fontes primárias verificadas nesta análise:
    - Infineon TLE8888-1QK Data Sheet Rev. 1.2 (2017-02-10)
    - Analog Devices MAX9924–MAX9927 Rev 5
    - STM32H562 (VREF+ só existe em LQFP100/UFBGA100)
    - Código do próprio repo (ver referências file:line ao longo do texto)
  ⚠️ Onde este documento contradiz docs/wiring_diagram.md, ESTE prevalece:
     aquele está desactualizado (mapas RGT6/TIM-OC e números do TLE8888 errados).
-->

# Plano de Hardware — Placa de Interface OpenEMS v1 (VGT6) rumo ao motor

## Context

O firmware OpenEMS está maduro em bancada (1200+ host-tests, scheduler congelado, ETB autocal, LTFT/STFT
validados com estimulador ESP32), mas nunca acionou um motor. Todo o hardware atual é um coreboard WeAct
STM32H562 com fios soltos para um estimulador ESP32 — não há estágio de potência, condicionamento de
sinal, proteção de entradas nem conector.

O objetivo é sair da bancada para a **primeira partida em motor real** (4 cilindros, roda 60-2, injeção e
ignição sequenciais, borboleta eletrônica), projetando uma **placa carrier/interface** que receba o
coreboard.

**Critério de projeto declarado pelo usuário: escolher componentes por desempenho — menor latência, menor
jitter, máxima imunidade a ruído.** Isso reordena a BOM inteira e está formalizado no orçamento de erro
abaixo.

### Decisões fechadas
- Alvo **VGT6 (LQFP100)** — GPIOE inteiro para INJ/IGN, sem os conflitos SDMMC/PB10-11 do RGT6.
- **Estágio de potência: TLE8888-1QK na v1** (LQFP-100). Injeção e ignição por **direct drive** →
  scheduler intacto. ⚠️ Exige **reescrever o driver** — ver a seção de arquitetura.
- **CKP: VR confirmado, pela interface VR do TLE8888** (zero-crossing + armamento por pico, clamp e
  diagnóstico integrados). **MAX9924 sai da BOM.** **CMP: Hall** direto ao `PA1`.
- **Knock diferido para a v2** — só footprint na v1.
- **USB com isolador galvânico**; **KiCad, 4 camadas**.
- **Injetores alta impedância** → acionamento saturado, sem peak-and-hold. Casa com o firmware atual.
- **Bobinas com ignitor integrado** (smart coils, entrada lógica) → sem IGBTs discretos.
- **VBATT real** num ADC dedicado (`PC3`, EWG diferido).

### Fonte de verdade — ler só estes
`docs/hw/pinout.md` e `src/hal/out_pins.h`.
⚠️ `docs/wiring_diagram.md` (ASCII) e `tools/esp32_combined/README.md` mostram RGT6/TIM-OC/pinos PA para
coisas que no VGT6 são PE*/BSRR/TIM15. **Não usar como referência de projeto.**

---

## Orçamento de erro de timing — e por que ele deixou de ser o eixo do projeto

O firmware entrega, medido: TIM5 quantiza a **16 ns**, jitter de escalonamento **~0,4 µs**
(≈**0,019° a 8000 rpm**), PW ±2 µs (memória `sched-inj-ign-precision`).

**Relação que governa tudo:** o atraso de um condicionador é constante em **tempo**, mas
`engine_angle_to_trigger_angle()` (`src/engine/ecu_sched_angle.cpp:47-53`) compensa com
`trigger_tooth0_engine_deg` — **um único offset em graus**. Como

```
erro_graus = atraso_µs × RPM × 6 × 1e-6        →  0,048°/µs a 8000 rpm
```

um offset angular só zera o erro **num RPM**. Atraso constante em tempo vira, portanto, erro angular
proporcional à rotação — não é "absorvido pela calibração", como é fácil supor.

### O balanço final, com números reais

| Fonte de erro | Valor | @8000 rpm |
|---|---|---|
| Quantização TIM5 (16 ns) | 16 ns | 0,0008° |
| **MAX9924 zero-crossing** (`tPDZ`, datasheet) | **50 ns** | **0,0024°** |
| **MAX9924 jitter** (`tPD-JITTER`) | **20 ns** | **0,001°** |
| Jitter do scheduler | ~0,4 µs | 0,019° |
| **Runout da roda fônica / gap do sensor** | — | **0,1–0,5°** ← **piso** |

**A conclusão que reordenou o projeto:** o front-end escolhido contribui **0,0024°** — 8× abaixo do
jitter do scheduler e **40–200× abaixo do piso mecânico** da roda fônica. Timing de CKP **não é um
problema neste projeto**, e não há trade-off a fazer na escolha do condicionador.

Três coisas caem disso:

1. **Compensação de atraso em firmware: CANCELADA.** O critério era "multiplicar o atraso real por
   0,048°/µs e comparar com 0,1–0,5°". Deu 0,0024°. Não implementar, nem na v2.
2. **O MAX9924 se justifica por CORREÇÃO, não por precisão** — imunidade a ruído, CMRR, e decodificar o
   dente faltante com sinal fraco no cranking. Não por ser rápido (é rápido de sobra, e isso não importa).
   Zero-crossing continua o modo certo porque a *variação* de threshold com amplitude seria o único termo
   eletrônico capaz de estourar o piso mecânico — mas o zero-crossing simplesmente elimina esse termo.
3. **O refinamento que ainda paga é de AMPLITUDE, não de tempo.** MAP e TPS ratiométricos alimentam
   diretamente as tabelas de combustível e avanço, e **não têm piso mecânico nenhum**. Se houver orçamento
   para exatamente um refinamento, é o LDO limpo e a referência estável do bloco 1 — não o front-end do
   CKP, que já está ordens de grandeza abaixo do que a mecânica permite enxergar.

⚠️ A relação 0,048°/µs fica registrada porque vale para **qualquer** atraso no caminho: se alguém propuser
opto-acoplador, filtro RC agressivo ou MCU intermediário no CKP, a conta volta a morder.

## Escopo v1 — placa completa, só o EWG fica de fora

**Populado na v1:** sync CKP/CMP · 4× INJ · 4× IGN · MAP/CLT/IAT/TPS · APP1/APP2 · ETB · VBATT ·
**pressão de combustível · pressão de óleo · flex fuel · 2× VVT** · CAN (WBO2 + telemetria) ·
alimentação · relés bomba/ventoinha · USB isolado.

**Diferidos (footprint sim, popular não):**
- **EWG/boost** — turbo-específico, segundo pino DIR ainda é `TODO(VGT6)`. É o que libera `PC3` para o
  VBATT (bloco 7).
- **Knock** — decisão de reduzir risco na v1: é o bloco analógico mais difícil de acertar, não contribui
  para a primeira partida, e o retard **mascara problema mecânico** justo quando se precisa enxergá-lo.
  Footprint reservado para o TPIC8101 na v2 (bloco 13).

Verifiquei os cinco recém-incluídos no firmware: **todos têm implementação real**, nenhum é stub.
Mas dois trazem limitações de arquitetura que mudam o hardware — blocos 13 (knock) e 15 (VVT). Ler antes
de comprar.

### ⚠️ Risco de escopo — dizer isto em voz alta
Cada bloco além do core-start é mais uma fonte de ruído e mais um jeito de a placa falhar no seu trabalho
principal, que é **fazer o motor pegar**. O knock já saiu por isso. Resta o **VVT** (bloco 15), que adiciona dois
solenoides comutando em PWM perto do par CKP.

**Mitigação: separar "popular" de "comissionar".** Montar na v1 (a placa fica pronta, evita respin), mas
comissionar em ordem: core-start primeiro, motor pegando e estável, **só então** VVT e flex. A sequência
de verificação reflete isso.

---

## Contratos elétricos impostos pelo firmware

1. **INJ/IGN = GPIO 3,3 V via BSRR, active-high, safe = LOW no boot.**
   INJ1–4 = `PE0/PE2/PE4/PE6`; IGN1–4 = `PE9/PE11/PE13/PE15`. Boot seguro por
   `ecu_sched_outputs_safe_early()` → `out_pins_hw_init()`.
   → Driver deve ser lógica 3,3 V, **ON em HIGH**, OFF em LOW/flutuante.
2. **ETB = TIM15_CH1 AF4 @ `PE5`, DIR `PE7`/`PE8`.** Não é PA6/TIM3/PA8 (isso é RGT6/doc stale).
3. **O2 exclusivamente CAN** (`FDCAN1` PB8/PB9, RX 0x180) — sem TLE8888 na v1, precisa transceiver discreto.
4. **CKP/CMP** em `PA0`/`PA1` (TIM5_CH1/CH2 AF2), pull-down interno + filtro IC ~256 ns
   (`src/hal/stm32h562/timer.cpp:41-64`).

---

## ⭐ Arquitetura v1 — TLE8888-1QK como hub (decidido 2026-07-20)

O TLE8888 entra na **v1**, não na v2. Isto substitui os blocos de estágio de potência discreto.

### O que torna isto compatível com o firmware congelado

A pergunta que decidia tudo era se as saídas aceitam **pinos diretos** ou só SPI. Resposta do datasheet
(cap. 9.1, Tab. 24):

| Entrada direta | Saída | Atribuição |
|---|---|---|
| **IN1–IN4** | **OUT1–OUT4** (injetores) | **fixa** — bits O1DD–O4DD em `DDConfig0` |
| **IN5–IN8** | **IGN1–IGN4** (ignição) | **fixa** — bits IGN1DD–IGN4DD em `DDConfig3` |
| IN9–IN12 | OUT5–OUT24 | configurável (`InConfig0-3`); **só 4 saídas** podem ser diretas |

*"A '1' in the control register/data frame bit or a **'high' at the direct drive inputs** switches on the
corresponding output."* E: *"All direct drive inputs have implemented a **pull down current source**."*

**Resultado: o scheduler congelado não muda em nada.** `PE0/2/4/6` e `PE9/11/13/15` deixam de ir a gates
de MOSFET e vão a `IN1–IN8`. Ativo-alto ✅, safe=LOW no boot ✅ (pull-down interno do próprio CI, o que
**dispensa** os pull-downs externos de gate que o plano discreto exigia). Os três contratos elétricos do
firmware são satisfeitos nativamente.

### Alocação de pinos

| Função | Pino MCU | Destino |
|---|---|---|
| INJ1–4 | `PE0/PE2/PE4/PE6` | IN1–IN4 (**inalterado**) |
| IGN1–4 | `PE9/PE11/PE13/PE15` | IN5–IN8 (**inalterado**) |
| **INJEN** | `PE1` | pino 24 — enable de hardware dos 4 injetores |
| **IGNEN** | `PE3` | pino 27 — enable de hardware das 4 bobinas |
| Bomba / ventoinha | `PE10` / `PE12` | IN9/IN10 → 2 saídas de relé (OUT14–20, 0,6 A) |
| **VVT escape / admissão** | `PB6` / `PB7` (TIM4_CH1/CH2, **15 Hz**) | IN11/IN12 → **OUT5/OUT6** (4,5 A, clamp ativo 50–60 V) |
| SPI2 | `PB12–PB15` | CS/SCK/MISO/MOSI — **agora livres**, resolvido o conflito |
| CKP | `PA0` ← VROUT | interface VR do TLE8888 (push-pull) |
| CMP | `PA1` | Hall, caminho próprio (o CI tem **um** canal VR) |
| CAN | `PB8/PB9` | interface HS-CAN integrada |

Sobra `PE14`. ⚠️ **Os 4 slots de direct drive do grupo OUT5–OUT24 ficam exatamente esgotados**
(bomba, ventoinha, VVT×2) — o datasheet limita a *"only 4 of this output stages can be switched
directly"*. Se algo mais precisar de direct drive, mover bomba e ventoinha para SPI (são lentas e não
têm requisito de forma de onda), o que libera IN9/IN10 — ao custo de depender do SPI para comandá-las.

**`INJEN`/`IGNEN` em pinos dedicados é ganho de segurança real**: caminho de corte de
injeção e ignição em hardware, independente do SPI *e* do scheduler — coisa que o estágio discreto não
oferecia.

### O que sai da BOM
MAX9924 · 4 MOSFETs de injetor + clamps · drivers de bobina · TJA1051 · drivers de relé de bomba/ventoinha
· driver de relé principal · regulador 5 V · trackers de 5 V dos sensores ratiométricos · **drivers de solenoide de VVT e seus diodos de roda-livre**.

### O que permanece externo
- **Ponte-H do ETB** (DRV8701 + FETs) — meias-pontes do CI são 0,6 A, falta uma ordem de grandeza.
- **EWG** (diferido de qualquer forma).
- **LDO 3,3 V** para o MCU (o CI entrega 5 V).
- Condicionamento analógico, divisor de VBATT, isolador USB.

### 🚨 O bloqueio real: o driver NÃO está pronto (eu disse que estava — errado)

Comparei o mapa de registradores do `src/hal/tle8888.cpp` com o datasheet Rev 1.2. **Não batem.**

| Datasheet Rev 1.2 | Nosso driver |
|---|---|
| `DDConfig0–3` (direct drive) | **ausente** |
| `OEConfig0–3` (output enable) | **ausente** |
| `InConfig0–3` (atribuição IN9–12) | **ausente** |
| `VRSConfig0/1/2` (bits VRSPV/VRSPT/VRSF/VRSM) | `REG_VRS_CTRL`+`REG_VRS_THRESH` com "histerese 20 mV" |
| `Cont0–3`, `Cmd0`, `BriConfig0`, `VRSDiag0/1` | ausentes |

O driver diz no comentário *"datasheet rev 2.1"*, mas os nomes que usa (`REG_OC_THRESH`, `REG_SLEW_RATE`,
`REG_VRS_CTRL`…) espelham a narrativa do **`wiring_diagram.md` stale**, não o datasheet. Somado ao facto
de o conflito `PB12`/`PB13` garantir que **nunca clockou hardware nenhum**, a conclusão é dura:
**o driver foi escrito contra um mapa de registradores inventado.**

⚠️ É exatamente o padrão de [[flash-nscr-nssr-register-map-bug]] — mapa errado, operação "sucede",
nada acontece no silício. Consequências concretas:
- Sem `DDConfig`, as saídas **não entram em direct drive** — os pinos IN1–IN8 não fazem nada.
- Sem `OEConfig`/enable central, os canais **nunca habilitam**; e como a proteção **reseta** esses bits,
  após qualquer falha o canal fica morto até rodar a sequência de recuperação (ler diag 2×, re-habilitar)
  — que o `poll_diag()` também não implementa.
- A configuração VRS atual quase certamente não corresponde a registrador nenhum.

**Tratar o driver como não escrito.** É o item de maior risco desta decisão: o trabalho migrou de
hardware para firmware. Reescrever `tle8888.cpp` contra o Rev 1.2 é **pré-requisito da placa**, não
tarefa posterior.

### ✅ Mitigação implementada (2026-07-20) — fingerprint do mapa de registadores

O driver foi reescrito, mas **isso não prova nada**: o mapa novo continua a ser a *minha leitura* da
Table 50, e nenhum dos 1234 host-tests lhe toca (mockam o SPI). O risco não é "o driver está errado" —
é **"o driver está errado e ninguém dá por isso"**, exatamente como em
[[flash-nscr-nssr-register-map-bug]].

`write_verify()` **não** fecha esse buraco: valida o caminho de escrita, mas se um endereço errado
calhar noutro registador escrevível, a escrita "sucede", a releitura confere, e o CI fica configurado
noutra coisa qualquer.

**O que fecha: ler os valores de reset ANTES de qualquer escrita.** Os registadores de configuração
têm reset documentado; lê-los e comparar prova de uma vez só que (a) o CI está presente, (b) o SPI está
vivo, (c) o formato do frame está certo — ordem de bits, largura, R/W — e (d) os endereços apontam para
os registadores que julgamos. Qualquer um destes errado faz **todas** as leituras divergirem.

Implementado como `verify_register_map()`, com 8 entradas escolhidas por serem **distintivas**:

| Registador | Reset |
|---|---|
| `ComConfig0` | `0xA4` |
| `ComConfig1` | `0x0D` |
| `OpConfig0` | `0x09` |
| `WdConfig0` | `0x47` |
| `WdConfig1` | `0x03` |
| `FWDConfig` | `0xF7` |
| `OutConfig3` | `0x30` |

⚠️ **A escolha não é arbitrária**, e três critérios governam-na:
1. Só registadores de **configuração**. Os de estado/contador (`WWDStat`, `TECStat`, ambos reset `0x30`)
   derivam com o estado do CI → dariam falso negativo.
2. Valores **distintos entre si**. Um conjunto cheio de `0x3F` (`OutConfig1/2/4/5`) não discrimina: um
   deslocamento de endereço que caia noutro `0x3F` passaria despercebido.
3. **Nenhum valor de fronteira** (`0x00` ou `0xFF`) — subtil, e é o que torna o diagnóstico legível.
   Num barramento morto o MISO flutua para um extremo e todas as leituras dão `0x00` **ou** `0xFF`. Com
   `OutConfig0` (reset `0xFF`) no conjunto, essa entrada passaria **por coincidência** num flutuar-alto e
   a máscara viria `0xBF` em vez de cheia — a parecer "mapa parcialmente errado" quando o problema é SPI
   mudo. Foi removido por isso. Sem valores de fronteira, **barramento morto ⇒ sempre todas as entradas
   divergem**.

**É bloqueante, não cosmético:** falhar impede `configure()`, deixa `tle8888_ok()` a false e, por
consequência, `power_stage_enable(false)` — arranca sem injeção nem ignição, em vez de arrancar com o CI
num estado desconhecido. E fica **latched**: `poll_diag()` não tenta recuperar (só usa `write_verify`,
que marcaria o CI como bom e mascararia justamente esta falha). Exposto na telemetria em
`reserved[49]`.

⚠️ **Isto NÃO fecha o risco antes do hardware.** Converte "mapa errado, silencioso, o motor não pega sem
razão visível" em "mapa errado, alto e específico, no primeiro power-on". É a resolução disponível sem
silício — o clock-out em bancada continua a ser o teste que decide.

---

## Blocos do PCB

### 1. Alimentação — re-especificada pelo critério de ruído
Topologia base de `docs/wiring_diagram.md` §Power Supply: P-MOSFET reverso + fusível 30 A, TVS
**SMBJ24CA** no VBAT, bulk 100 µF. **Mas as peças concretas daquele doc são incompatíveis com o critério
de precisão e ficam explicitamente rejeitadas:**

| Peça do doc legado | Problema | Substituir por |
|---|---|---|
| **LM2596-HV** | 150 kHz, ripple ~150 mV — dos switchers mais ruidosos em uso | **TPS54302** ou switcher ≥500 kHz de baixo ripple |
| **AMS1117-3.3** | Dropout alto e **PSRR fraco em alta frequência** — não rejeita o ripple que chega | LDO **low-noise / high-PSRR** nos rails de medição |

⚠️ **Parcialmente supersedido:** o TLE8888 traz **pré-regulador + regulador 5 V + 2 trackers de 5 V**
(saídas de tracking feitas exatamente para sensor ratiométrico, `DVT5Vx` ±10 mV) + regulador standby e
monitor de tensão. Isso cobre o 5 V dos sensores e tira o buck do caminho de medição.
**Ainda é preciso um LDO 3,3 V** para o MCU — e é aí que a tabela acima continua valendo.

Os sensores MAP/TPS/APP são **ratiométricos**: a saída é proporção da alimentação.
- **5 V dos sensores: usar os trackers do TLE8888.**
  ⚠️ O LDO sozinho não salva se for alimentado com 150 kHz de ripple e tiver PSRR ruim naquela frequência
  — **as duas trocas da tabela andam juntas**, trocar só uma não resolve.
- **VDDA/VREF+** com filtro próprio (ferrite + 1 µF + 100 nF) e pour AGND dedicado no VSSA.
- Chaveamento do buck **acima da banda dos filtros RC dos sensores** e longe de harmônicas da frequência
  de dente do CKP (60 dentes × 8000 rpm ≈ 8 kHz — folgado, mas verificar as harmônicas).

**VREF+ — ✅ DECIDIDO (2026-07-20): (a) VDDA 3,3 V filtrado, com (c) reservado como DNP.**
Ferrite + 1 µF + 100 nF, pour AGND dedicado no VSSA, alimentado pelo LDO low-noise/high-PSRR.
Fundamentação completa na pendência 3.

### 2. Front-end CKP — interface VR do TLE8888

**Decidido: o CKP usa a interface VR integrada** (cap. 10 do datasheet). O MAX9924 sai da BOM.

**Arquitetura — a mesma que o plano perseguia desde o início:**
> *"To achieve the best accuracy... **the switching point is the zero crossing**. For robustness against
> disturbances the next zero crossing is enabled only if a signal peak... is detected."*
> *"The zero crossing detection ensures that **the influence of the input signal slope is eliminated for
> both edges**."*

Zero-crossing como borda de tempo, detecção de pico como armamento — e a segunda frase é exatamente a
invariância com amplitude que o orçamento de erro exigia.

**Configuração (registradores `VRSConfig0/1/2`):**
- **Modo de detecção** (`VRSM`): manual / **auto adaptativo** / semi-auto / Hall.
  **Alvo: auto**, que o datasheet descreve como *"an adaptive algorithm to ensure best detection
  performance"* (o semi-auto é explicitamente *"less accurate"*). Em auto, escritas a `VRSPV`/`VRSPT`
  são ignoradas — o CI cuida sozinho.
- Threshold de pico (`VRSPV`): 50 / 150 / 350 / 550 mV. Tempo mínimo de pico (`VRSPT`): 10 µs ou 250 µs.
  Filtro de saída (`VRSF`): `tof,1..4`. Todos só relevantes nos modos manual/semi-auto.
- **Diagnóstico** (`VRSDIAGM` + `VDIAGS` em `Cmd0`): short-to-GND, short-to-BAT, open-load **e leitura ADC
  da tensão de entrada** (`VRSDiag1`/`VRSD`, `tconv` ≈ 7 µs). **Expor isto na telemetria** — num veículo é
  a diferença entre "não pega" e "não pega *porque* o CKP está em circuito aberto".

**Especificações que importam:**
- `VVR,th` (threshold de zero-crossing): **±30 mV**. Traduzido em ângulo (erro = Voffset ÷ 2πfA):
  **0,030° no cranking**, **0,0015° a 8000 rpm** — ambos abaixo do piso mecânico de 0,1–0,5°.
- ⚠️ **Atraso de propagação não é especificado** (o MAX9924 garantia 50 ns/20 ns). Pelas contas acima não
  muda nada, mas é **dado ausente, não dado bom** — medir na bancada (verificação passo 3).
- **Clamp de entrada integrado**: 50 mA, 2–3 V entre `VRIN1`/`VRIN2`. Dispensa a rede externa de
  resistor série + clamp que o MAX9924 exigia — **menos componentes na rede mais sensível da placa**.
- Resistência de carga interna `RVR,Load` 50/75/110 kΩ; tensão média 2,25 V típ. aplicada internamente
  para não deixar a entrada flutuando.
- **`VROUT` é push-pull** (ao contrário do open-drain do MAX9924) → **sem pull-up externo**, e o
  pull-down interno de `PA0` pode ficar como segunda camada anti falso-sync sem disputar nada.

**Cabeamento e imunidade** (inalterado):
- Par **trançado e blindado**, blindagem aterrada **só no lado da ECU**.
- Sem filtro RC agressivo na entrada — o clamp e o `RVR,Load` já definem a rede.
- Choke de modo comum a considerar: ataca ruído de modo comum da ignição sem tocar o diferencial.
- Manter o filtro IC de 256 ns do TIM5 como camada final (memória `ckp-noise-false-sync-injectors`).

**Pontos de injeção de bancada — continuam necessários:**
- **TP-DIG + jumper 0 Ω** entre `VROUT` e `PA0`: removido, libera o nó para o estimulador ESP32 injetar
  digital (sync e varredura de RPM). Sem isso a saída push-pull do CI disputa com o estimulador.
- **TP-VR** no par diferencial antes do CI: para caracterização com fonte analógica VR real
  (gerador, ou ESP32-DAC + atenuador — memória `esp32-dac-map-tps`).

### 3. Front-end CMP — Hall (confirmado)
Sem MAX9924 neste canal. Sensor Hall open-collector → **pull-up externo** (10 k → 5 V) + divisor para
3,3 V + RC leve + clamp. Timing do CMP é muito menos crítico que o do CKP: ele só resolve fase (qual
volta do ciclo), não o ângulo de faísca — tolera dezenas de µs sem consequência.

⚠️ **Mudança de firmware confirmada, não mais hipotética:** `PA1` tem **pull-down interno** configurado em
`src/hal/stm32h562/timer.cpp:41-64`, que conflita com saída open-collector (o pull-down luta contra o
pull-up externo, degradando nível e borda). Trocar `PA1` para pull-up interno ou nenhum pull, mantendo o
filtro IC. **O pull-down do `PA0` fica** enquanto o CKP for VR.
Cuidado: esse pull-down foi parte do fix de falso-sync; ao removê-lo do `PA1`, o pull-up externo passa a
ser a defesa contra flutuação com sensor desligado — dimensionar com firmeza (memória
`wasted-sequential-transition-state`).

### 4–5. Injetores e bobinas — SUPERSEDIDOS pelo TLE8888

Os FETs de injetor, clamps de flyback, resistores/pull-downs de gate e drivers de bobina **saem**:
OUT1–OUT4 (2,2 A low-side, adequado a injetor de **alta impedância**) e IGN1–IGN4 (push-pull 20 mA,
que é exatamente drive lógico de **smart coil**) fazem o trabalho, com clamp, proteção de sobrecorrente,
sobretemperatura e diagnóstico open-load / short-GND / short-BAT por canal — tudo integrado.

**O que sobrevive do raciocínio de slew (antigo bloco 4b):** o TLE8888 tem **`SLEW_RATE` configurável por
SPI**, então o trade EMI × dissipação continua existindo — só que agora é **parâmetro de software**, não
resistor de gate. Sintonizar na bancada no teste de ruído sob carga (verificação passo 4), lembrando que:
- **Injeção** tolera atraso simétrico (a PW se preserva); assimetria entre bordas **não**.
- **Ignição** é ângulo: 0,048°/µs a 8000 rpm. Slew da ignição mais rápido que o da injeção.

⚠️ Continua valendo a memória `ckp-noise-false-sync-injectors`: o acoplamento de ruído das chaves no CKP
já derrubou este projeto uma vez.

### 6. Analógicos — condicionamento e proteção
Por canal: divisor **10 k / 15 k** (0,5–4,5 V → 0,3–2,7 V) + RC **1 k + 100 nF** + **TVS 3,3 V**.
- O RC dá fc ≈ 1,6 kHz — confortável como anti-aliasing para grandezas lentas (CLT/IAT/TPS/APP), mas
  **verificar contra a taxa de amostragem do ADC** e contra a banda desejada de MAP (MAP tem conteúdo
  rápido por pulsação de coletor; filtrar demais atrasa a resposta transitória de carga).
- Popular v1: MAP `PA3`, TPS `PA4`, APP1 `PC0`, APP2 `PC2`, ETB_TPS1 `PA2`, ETB_TPS2 `PC5`, VBATT `PC3`.
  NTC com pull-up 2,49 k → 5 V: CLT `PB0`, IAT `PB1`.
- **Pressão de combustível `PC4`/INP4 e pressão de óleo `PC1`/INP11** — sensores ratiométricos 0,5–4,5 V,
  mesma rede padrão acima, sem nada especial. O firmware já converte
  (`src/drv/sensors.cpp:726-728`) e já valida plausibilidade (fuel > 5 bar e oil > 10 bar viram falha,
  linhas 636/641). Alimentar do **mesmo 5 V limpo do LDO** — são ratiométricos como o MAP.
- Knock `PA5`/INP19 tem bloco próprio (13) — **não** é a rede padrão.

### 7. VBATT dedicado — requer mudança de firmware
`src/drv/sensors.cpp:844-847`: com `etb_harness_present=1`, `PC5` vira ETB_TPS2 e **`vbatt_mv` é fixado em
12000**. Mas `corr_vbatt()` (`fuel_calc.cpp:266`) e `dwell_ms_x10_from_vbatt_rpm()` (`ign_calc.cpp:156`)
consomem esse valor — num cranking real a bateria cai a 9–10 V, o dead-time fica subestimado e o dwell
curto exatamente quando falta energia.

**Solução:** divisor 0–18 V → 0–3,3 V alimentando **`PC3` / ADC2 / INP13** (canal EWG_POS, livre com o EWG
diferido), com RC generoso (VBATT é lenta; filtrar forte é grátis e rejeita transientes de bobina). No
firmware, usar `vbatt_raw_to_mv()` (`sensors.cpp:332`, já existe) em vez do literal.

⚠️ **Ponta solta a amarrar junto:** o feedback de posição do EWG (`src/engine/ewg_control.cpp:59-64`,
`ewg_pos_min_raw`/`ewg_pos_max_raw`) consome o raw de `PC3`. Assim que `PC3` virar VBATT, esse caminho lê
lixo — inofensivo só porque a saída EWG não está populada. **Guardar/desabilitar o caminho de posição do
EWG enquanto o EWG estiver diferido**, na mesma mudança de `sensors.cpp`/`adc.cpp`, para liberar o canal
de forma limpa. (O `run_wastegate_control()` em si usa MAP e RPM, não `PC3` — não é afetado.)

### 8. ETB — ponte-H + failsafe mecânico obrigatório

✅ **DECIDIDO (2026-07-20): BTS7960 a 10 kHz.** PWM ← `PE5` (TIM15_CH1 AF4), IN1 ← `PE7`, IN2 ← `PE8`,
12 V, feedback TPS1/TPS2 pelo bloco 6.

O firmware pedia **20 kHz**, contra o máximo de **25 kHz** do BTS7960 — 20% de margem, com perdas de
comutação altas e FETs internos ao módulo, logo sem como aliviar. **Baixado para 10 kHz**
(`etb_pwm_init(10000u)`, `src/hal/stm32h562/timer.cpp`). A resolução do duty até melhora: ARR = 6250
passos (era 3125), bem acima dos 1000 que `etb_pwm_set_duty_x10()` precisa. **Custo aceite: chiado
audível**, já que 10 kHz está dentro da banda audível.

**O DRV8701 + 4 MOSFETs foi considerado e recusado.** Ganhava numa coisa real — o pino `IDRIVE`
(6/12,5/25/100/150 mA de corrente de gate) é um botão de EMI no ponto exato onde este projeto já falhou
(ruído de comutação → falso-sync → batch-fire, `ckp-noise-false-sync-injectors`). Perdeu por duas razões:
1. **Descasamento de interface.** `etb_driver.cpp:128-145` aciona **3 pinos** (PWM + IN1 + IN2, com
   travagem em ambos a 0) — convenção nativa do BTS7960, que mapeia **1:1 hoje**. O DRV8701E é PH/EN e o
   DRV8701P é IN1/IN2, ambos de 2 pinos: nenhum é drop-in, e a mudança cairia num subsistema que **nunca
   correu em hardware**.
2. **Responsabilidade de layout.** O módulo traz os loops de comutação resolvidos; 4 FETs externos mal
   dispostos ao lado do par CKP são **piores** que o módulo — controlo de slew não compensa área de laço.

Se o ETB vier a acoplar ruído no CKP, os remédios são layout, blindagem e filtro — não há botão de slew.
**Vigiar no passo 4 da verificação** (ruído sob carga).
Motor do ETB é fonte pesada de ruído — alimentação separada, retorno próprio até o ponto estrela, e o par
de feedback TPS longe do par de potência.

⚠️ **Gate de segurança.** O IACV foi removido — o ETB é a **única** autoridade sobre a borboleta, e o
autocal + PID **nunca rodaram em hardware** (memória `etb-autocal-power-on`). Travado aberto = motor em
disparada.
1. Mola de retorno **default-closed** verificada à mão.
2. Corte de energia duro do estágio ETB, independente do MCU e ao alcance.
3. Batente mecânico limitando a abertura máxima na primeira partida.
4. **ETB só toca o motor depois de validado em bancada** com chicote real.

### 9. CAN — integrado no TLE8888
**TJA1051 sai da BOM.** Interface HS-CAN integrada com wake-up por barramento, direto em `PB8`/`PB9`.
Terminação 120 Ω jumpeável na placa. Serve WBO2 (RX 0x180) e telemetria (0x400/0x401/0x402).

### 10. Relés — bomba e ventoinha, via TLE8888
Lógica de firmware **já existe e está correta** (`auxiliaries.cpp`): `run_pump_control()` faz prime de
2 s no key-on, mantém com RPM > 0 e **corta 3 s após RPM = 0** (considerar reduzir p/ 1–2 s);
`run_fan_control()` com histerese 95/90 °C. Ambos suspensos em `output_test_active()`.

**Hardware:** saídas de relé do TLE8888 (OUT14–20, 0,6 A — folgado para bobina de relé de ~200 mA), mais
o **driver de relé principal** integrado. Drivers discretos e diodos de roda-livre saem da BOM.

**Controle:** duas opções — (a) **direct drive** por `PE10`/`PE12` → IN9/IN10, gastando 2 das 4 saídas
direct-drive disponíveis no grupo OUT5–OUT24; ou (b) **via SPI**, já que relé é lento — libera `PE10`/`PE12`
mas exige uma função `tle8888_set_output()` que **não existe** no driver. Como o driver vai ser reescrito
de qualquer forma, (b) é defensável. **Recomendo (a)** para a v1: mantém o controle fora do caminho do SPI,
que também carrega o watchdog.

⚠️ **A migração de `PB12`/`PB13` é obrigatória de qualquer modo** — sem ela o SPI2 não existe e o
TLE8888 inteiro não funciona. Ver o bug em [[pb12-pb13-pump-fan-vs-spi2-tle8888]].

### 11. Relé principal — v1 sem controle por MCU
⚠️ **Rever:** o TLE8888 traz **driver de relé principal**, detecção de chave e *engine off timer*
integrados — feitos para exatamente este problema (manter o rail vivo até terminar as escritas em flash).
A razão original de evitar o relé comandado deixa de valer; avaliar usar o mecanismo do CI.

**Decisão anterior (a rever): a ECU é alimentada direto pelo key-on.** O firmware grava LTFT, calibração e `EtbCalRecord` em
flash, e esse caminho já mostrou fragilidade (memória `flash-nscr-nssr-register-map-bug`). Relé cortado
pelo MCU no key-off pode matar o rail no meio de uma escrita e corromper o setor. Fazer certo exige
**power-latch** — complexidade desnecessária agora. Footprint reservado para a v2.

### 12. — (movido) O inventário do TLE8888 está na seção de arquitetura, no topo.

### 13. Knock — o bloco que exige envelope em hardware (não é só um bandpass)

**Como o firmware detecta:** `knock_adc_update(raw)` é chamado de `sample_fast_channels()`
(`src/drv/sensors.cpp:474-477`) **uma vez por dente do CKP**, dentro da janela angular; conta amostras
acima de um threshold e aplica retard (+2,0° por evento, até 10,0°, recovery -0,1°/ciclo limpo).
O cabeçalho de `knock.cpp:13` é explícito: *"Sem periférico COMP interno (STM32H562 não o possui) —
detecção 100% em software a partir das amostras ADC."* **Não há filtro digital nem FFT.**

**A consequência dura — Nyquist.** A taxa de amostragem é a taxa de dente, e a roda 60-2 tem 58 dentes
por volta:

| RPM | Amostras/s |
|---|---|
| 1500 | ~1,45 kHz |
| 3000 | ~2,9 kHz |
| 6000 | ~5,8 kHz |
| 8000 | ~7,7 kHz |

Detonação vive em **6–8 kHz**, que exigiria amostrar acima de ~12–16 kHz. **Em nenhum RPM a taxa de dente
alcança isso.** Ou seja: entregar a onda AC filtrada crua ao ADC produz aliasing, e a contagem de amostras
acima do threshold vira ruído sem significado.

**Portanto o front-end analógico precisa entregar um envelope, não a portadora:**
`piezo → bandpass 6–8 kHz → retificação de onda completa → integrador/peak-hold com decaimento
controlado → bias em meio-rail → PA5`.
Só o envelope é lento o bastante para a taxa de dente ler com sentido. O default
`kAdcThresholdDefault = 2048` (meio de escala 12-bit) confirma que o firmware espera exatamente um sinal
polarizado em meio-rail.

**Decisão: knock fica para a v2 — só footprint na v1.** Motivos: é o bloco analógico mais difícil de
acertar, sintonizado numa frequência de detonação que ainda não se conhece; não contribui em nada para a
primeira partida; e o retard **mascara problema mecânico** justo na fase em que se precisa enxergar tudo.

**Preparar na v1, sem popular:**
- Footprint do **TPIC8101** (bandpass e ganho programáveis por SPI, retificação de onda completa e
  integrador — o caminho do rusEFI, `set_knock_ic 1`; coerente com a memória
  `always-check-speeduino-rusefi-ms`), junto da área do TLE8888. Exigirá driver SPI novo (o do TLE8888
  não serve) e entra na mesma conta de chip selects do bloco 10.
- Trilha de `PA5` roteada até o footprint, e a via do piezo reservada no conector.
- Entrada **longe das linhas de bobina** desde já (o knock ouve exatamente a banda que a ignição gera) —
  reservar o espaço no layout agora custa nada e evita respin.

Quando for populado: cabo do piezo blindado; `knock_sensor_dead()` já detecta sensor mudo.

### 14. Flex fuel — `PB5`
Implementado em `src/hal/flex_fuel.cpp` (EXTI). Sensor GM/Continental: **50 Hz = 0 % etanol, 150 Hz =
100 %**; duty 10–90 % = -40..+125 °C de temperatura do combustível.
Hardware: alimentação 12 V do sensor, divisor **10 k / 3,3 k** (0–12 V → ~0–3,0 V), pull-up 10 k → 5 V se
a saída for coletor aberto, TVS 3,3 V. Sinal é onda quadrada lenta — **filtrar com folga**, é entrada
digital e não há requisito de timing fino aqui.

### 15. VVT — 2 solenoides pelo TLE8888 (OUT5–OUT7)

**Podem ser acionados por meia-ponte? Sim — mas não é a escolha certa aqui.**

A meia-ponte traria **freewheeling ativo** (o datasheet oferece *"passive or active freewheeling...
actively with switching on the freewheeling path"*), que num solenoide sob PWM dissipa menos e dá melhor
controle do decaimento de corrente que um diodo. Dois motivos afastam a opção:
1. **Corrente.** As meias-pontes (OUT21–24) são de **0,6 A**. Uma OCV de VVT típica tem bobina de
   6,9–12 Ω → **1,1–2 A** a 13,5 V. Falta 2–3×.
2. **A OCV é unidirecional** — solenoide proporcional contra mola, o duty define a posição. Nunca precisa
   inverter corrente, então a bidirecionalidade da ponte fica sem uso.

**Escolha: OUT5–OUT7** (os 3 estágios de maior corrente, pensados para aquecedor de O2 e afins):

| Parâmetro | Valor |
|---|---|
| Corrente de operação | **4,5 A** |
| Limitação em sobrecorrente | 4,5–8 A |
| `ROUTn,on` | 350 mΩ @ 3 A |
| **Clamping** | **ativo, 50–60 V** |
| Energia de clamp repetitiva | 22 mJ (1×10⁹ ciclos, 125 °C) |

O datasheet descreve a topologia como *"a reverse protected low-side switch with **active clamping
freewheeling**"* — ou seja, **low-side com clamp ativo integrado**, exatamente o que uma OCV quer.
**Saem da BOM os drivers de solenoide e os diodos de roda-livre.**

**PWM: 15 Hz** (`kAuxTim4PwmHz = 15u`, `auxiliaries.cpp:41`) — frequência baixíssima, nenhuma preocupação
de comutação ou de atraso do CI. Vem de TIM4_CH1/CH2 em `PB6`/`PB7` → `IN11`/`IN12`.

⚠️ **Ressalva de arquitetura (inalterada):** em `auxiliaries.cpp:379-384` os **dois** PIDs consomem o
**mesmo** `pos_deg_x10` de `calc_cam_pos_est_x10(snap)`, derivado do **único** CMP. O came instrumentado
fica em malha fechada; o outro persegue a posição do came errado. Controle dual real pede 2º sensor de
came + firmware. **v1: montar os dois, comissionar só o instrumentado, reservar via no conector.**

### 16. Lacunas que o plano não cobria — mecânica, ambiente e USB

Levantadas na revisão crítica. Nenhuma é de precisão, mas todas podem matar a placa no motor:

- **O coreboard WeAct é grau consumidor.** Vai para um compartimento de motor com vibração, ciclo térmico
  e umidade, montado em barras de pino. Definir: soquete com retenção mecânica (ou solda direta),
  travamento do conjunto, e faixa de temperatura aceita. O cristal e os eletrolíticos do coreboard não
  foram feitos para isso — é um risco assumido conscientemente, não ignorado.
- **Caixa, vedação e alívio de tração** — não existem no plano. Definir grau de proteção, entrada do
  chicote com alívio de tração, e se haverá *conformal coating* na placa de interface.
- **USB com isolador galvânico (decidido).** Ligar o notebook com o motor rodando criaria laço de terra
  entre a massa do veículo e a do laptop — clássico matador de ECU e injetor de ruído justo durante a
  calibração. O isolador preserva o fluxo de tuning que já funciona (dash + USB CDC) e corta o laço.
  Requer **alimentação isolada no lado do veículo** (DC-DC isolado ou o próprio CI com isolação
  integrada) — dimensionar para a corrente do CDC. Colocar o isolador **junto ao conector**, não junto
  ao MCU.
- **Chicote é parte do projeto, não acessório.** Bitola por circuito (injetor e bobina puxam corrente),
  pares trançados para CKP/CMP/CAN, blindagens com dreno num ponto só, fusíveis por ramo. O conector de
  ~47 vias precisa disso definido para escolher o modelo.

### 17. Logística do projeto
- **KiCad, 4 camadas (decidido).** 4 camadas é o mínimo honesto para os pours PGND/SGND/AGND separados em
  que todo o bloco 18 se apoia — 2 camadas anularia boa parte do critério de imunidade a ruído.
  Empilhamento sugerido: sinal / **terra contínuo** / alimentação / sinal, com o par CKP na camada
  superior sobre terra ininterrupto.
- **Ordem de execução:** fechar as pendências restantes → esquemático por blocos → **revisão do
  esquemático antes do layout** (é onde erro custa barato) → layout → revisão → protótipo → sequência de
  verificação.
- **Estimativa e custo** ainda não levantados. Fazer antes de comprar.

### 18. Layout — onde a precisão é ganha ou perdida
Nenhuma escolha de componente sobrevive a um layout ruim:
- **Terra em estrela**: pours separados PGND / SGND / AGND / Shield, unidos num único ponto.
  Correntes de injetor e bobina **nunca** atravessam o retorno de sinal.
- **Par CKP** é a rede mais sensível: curto, blindado, longe de INJ/IGN/ETB/relés, com plano de terra
  contínuo por baixo. Sem via desnecessária, sem passar sob indutor do buck.
- Loops de comutação (FET–bico–bulk, e agora também os solenoides de VVT) **fisicamente pequenos** —
  área de laço é o que irradia.
- **Cabo de knock** é a segunda rede mais sensível depois do CKP: blindado, longe das linhas de bobina e
  dos solenoides de VVT — ele escuta justamente a banda que a ignição emite.
- 100 nF em cada pino VDD; bulk 100 µF no estágio de potência; VDDA isolado por ferrite.
- Blindagem de CKP e CAN aterrada só no lado da ECU.

---

## Arquivos a alterar

| Arquivo | Mudança |
|---|---|
| **`src/hal/tle8888.cpp` — REESCREVER (bloqueio da placa)** | Mapa de registradores atual não corresponde ao datasheet Rev 1.2. Implementar `DDConfig0-3` (direct drive INJ/IGN), `OEConfig0-3` + enable central, `InConfig0-3` (relés), `VRSConfig0/1/2` (modo auto), sequência de recuperação pós-falha, `VRSDiag0/1` na telemetria, e `tle8888_set_output()` se os relés forem por SPI |
| `src/engine/auxiliaries.cpp` (103-106) | Bomba/ventoinha `PB12`/`PB13` → `PE10`/`PE12`, condicional por board. **Obrigatório**: sem isso o SPI2 não existe |
| `src/hal/stm32h562/timer.cpp` ou `out_pins` | Adicionar `INJEN`=`PE1` e `IGNEN`=`PE3` como saídas, altas após init seguro |
| `src/drv/sensors.cpp` (~832–848) | Ler VBATT de `PC3`/INP13 via `vbatt_raw_to_mv()`; remover o literal 12000 |
| `src/hal/adc.cpp` | Reatribuir INP13/`PC3` de EWG_POS → VBATT |
| `src/engine/ewg_control.cpp` (59-64) | Guardar/desabilitar o feedback de posição do EWG enquanto o canal `PC3` for VBATT |
| `src/hal/stm32h562/timer.cpp` (41-64) | ⚠️ **CORRIGIDO — só o `PA1` muda.** A versão anterior desta linha mandava remover o pull-down de `PA0` **e** `PA1`, mas era texto da era MAX9924. Com o CKP na interface VR do TLE8888, `VROUT` é **push-pull** (não open-drain) → **o pull-down de `PA0` FICA**, e removê-lo desfaz o fix de falso-sync (`ckp-noise-false-sync-injectors`). Só o `PA1` (CMP Hall, open-collector) passa a pull-**up** interno. Manter o filtro IC 256 ns nos dois. **Ver a questão de polaridade de borda abaixo — não é só trocar o resistor.** |
| ~~`src/engine/auxiliaries.cpp` (`kPumpOffDelayMs`)~~ ✅ | **Feito** — corte da bomba 3 s → **2 s** (bloco 10) |
| ~~`src/hal/stm32h562/timer.cpp:283`~~ ✅ | **Feito** — `etb_pwm_init()` 20 kHz → **10 kHz** para a BTS7960 |
| `src/engine/auxiliaries.cpp` (103-106) | Só **se** bomba/ventoinha migrarem de `PB12`/`PB13` p/ liberar o SPI2 |
| `docs/hw/pinout.md` | VBATT em `PC3`; bomba/ventoinha `PB13`/`PB12`; nota do pull do `PA1` |
| ~~`docs/wiring_diagram.md`~~ ✅ | **Feito** — purgado o mapa de pinos ASCII e a narrativa de registadores do TLE8888 (era a origem do mapa inventado); ficou só alimentação/condicionamento/atuadores/conector/terra, com ponteiros para as fontes de verdade. Conector fechado em **55 vias** |
| `docs/hw/vr_input_conditioning.md` | CKP é VR-only com zero-crossing (e o porquê: variação de atraso é o único termo eletrônico acima do piso mecânico); CMP agora é Hall; jumper 0 Ω de bancada |
| Novo `docs/hw/interface_board_v1.md` | Esquemático, BOM, orçamento de erro, notas de layout |
| `test/` | Teste host do caminho de VBATT (padrão de `test_sensors_etb_harness_present`) |

---

## Montagem, formato da placa e conector — ✅ fechado (2026-07-20)

### ⭐ Recomendação primária: montar na CABINE, não no compartimento do motor

É a decisão com melhor relação benefício/custo de todo este documento, e resolve de uma vez três
problemas que de outro modo se pagam em componentes:

1. **O coreboard WeAct é grau consumidor** (o próprio plano diz isto no bloco 16). O cristal e os
   eletrolíticos não foram feitos para ciclo térmico de compartimento de motor. Nenhuma caixa resolve
   isso — só o ambiente resolve.
2. **Põe a antepara aterrada entre a ECU e a ignição.** Ataca exatamente o acoplamento de EMI que já
   derrubou este projeto uma vez (`ckp-noise-false-sync-injectors`), e de graça.
3. **Colapsa o requisito de caixa** de "die-cast selada IP67" para "qualquer caixa decente", o que por
   sua vez permite usar caixa de catálogo em vez de desenhar uma.

Custo: o chicote atravessa a antepara por passa-cabos vedado, e a corrida fica mais longa. O par CKP
aguenta bem — é par trançado e blindado, e o orçamento de erro mostrou que o timing tem margem de ordens
de grandeza. Local típico: painel lateral dos pés (*kick panel*), atrás do painel.

⚠️ **Isto implica furar a antepara — confirmar antes de assumir.** Se a montagem tiver mesmo de ser no
compartimento do motor, a decisão de caixa muda para die-cast selada com passa-cabos, *conformal
coating* obrigatório, e o coreboard passa a ser risco assumido (ou soldado direto em vez de em barra de
pinos).

### Conector: TE **AMPSEAL** — 776164-1 (35 vias) + 770680-1 (23 vias)

**Porquê AMPSEAL:** é o padrão de facto do mundo EMS aftermarket (o que significa precedente, chicotes,
tutoriais e ferramenta de crimpar acessíveis), selado, e barato. Especificações confirmadas: **4 mm de
centerline, 3 filas, IP67, −40…+125 °C, fio 0,5–1,25 mm²** (≈20–16 AWG).

**Porquê dois tamanhos diferentes, e não 2×35:** tamanhos distintos são **fisicamente impossíveis de
trocar**. Com dois conectores iguais, um chicote mal ligado é uma questão de tempo — e aqui isso
significa 12 V numa entrada de sensor. Isto também realiza a regra que o `wiring_diagram` já impunha:
separar fisicamente potência de sinal.

| | Conector | Vias | Grupo |
|---|---|---|---|
| **A** | `776164-1` (35 vias) | 35 | **Sinais** — sync, analógicos, CAN, flex |
| **B** | `770680-1` (23 vias) | 23 | **Potência/atuadores** — INJ, IGN, VVT, relés, ETB |

⚠️ **A alocação só fecha por causa de uma correção de encaminhamento** (ver abaixo): a potência das
bobinas e dos injetores **não passa pela ECU**. Sem essa correção, o grupo de potência não cabia em 23
vias — e, pior, teria contactos acima do que aguentam.

### 🚨 Correção: potência de bobinas e injetores NÃO atravessa a ECU

A tabela anterior deste documento levava `+12V das bobinas`, `PGND das bobinas` e `+12V dos injetores`
pelo conector. **Está errado, e não é só desperdício de vias:**

- **Bobinas com ignitor integrado** só precisam do **trigger lógico (~20 mA)** vindo da ECU. A corrente
  primária (7–10 A de pico) vem do relé/bateria **no chicote**, e nunca deve atravessar um contacto de
  8 A.
- **Injetores:** o *low-side* (~1 A) passa mesmo pela ECU — `INJ1–4` ficam. Mas o **+12 V** vem do relé
  no chicote.

Isto tira 3 vias e, mais importante, tira do conector uma corrente que ele não suportaria.

⚠️ **O que ainda esbarra no limite: o motor do ETB.** Puxa ~2–3 A em regime mas **8–10 A em stall**, e o
contacto AMPSEAL dá **8 A em estanho** (17 A em ouro). **Duplicar os pinos do motor (2+2)** ou
especificar contactos dourados. Não deixar em pino único.

### Alocação final

**Conector B — `770680-1`, 23 vias (potência/atuadores)**

| Sinais | Vias |
|---|---|
| `VBAT+` (alimentação da placa) | 2 |
| `PGND` | 3 |
| `INJ1–4` low-side (OUT1–OUT4) | 4 |
| `IGN1–4` trigger lógico (20 mA) | 4 |
| VVT escape / admissão (LS, OUT5/OUT6) | 2 |
| Bobinas de relé: bomba, ventoinha, principal | 3 |
| ETB `Motor+` ×2, `Motor−` ×2 (**duplicados**, ver acima) | 4 |
| **Usadas / livres** | **22 / 1** |

**Conector A — `776164-1`, 35 vias (sinais)**

| Sinais | Vias |
|---|---|
| CKP+ (`VRIN1`), CKP− (`VRIN2`), blindagem | 3 |
| CMP sinal, CMP +5 V, CMP GND | 3 |
| MAP, CLT, IAT, APP1, APP2, P.combustível, P.óleo | 7 |
| ETB `TPS1`, `TPS2` | 2 |
| `5V_A`, `5V_B` (trackers do TLE8888) | 2 |
| `SGND` | 2 |
| CANH, CANL, blindagem | 3 |
| Flex fuel: +12 V, sinal, GND | 3 |
| *Knock: sinal, blindagem* — **reservado, não cablar na v1** | 2 |
| *2º sensor de came* (VVT dual) — reservado | 3 |
| TPS independente (`PA4`) — reservado | 1 |
| Livres | 4 |
| **Total** | **35** |

**57 vias provisionadas** (22 + 35), das quais ~10 são reserva deliberada. Substitui a estimativa de 55:
aquela era uma contagem de sinais, esta é uma alocação a conectores reais.

### Formato da placa

**Regra, não número** — as dimensões dependem de duas medidas que **ainda não estão verificadas**:

⚠️ **Medir antes de desenhar:**
1. **O coreboard.** Não consegui obter as dimensões de fonte fiável, e a repo pública da WeAct que
   aparece é a de **64 pinos** — que não é a placa que temos (o alvo é **VGT6/LQFP100**). Tirar
   comprimento, largura, posição e passo dos headers **da placa física**.
2. **As caixas dos AMPSEAL.** Puxar os desenhos dimensionais da TE para as duas referências — a largura
   da aresta de conector é a soma das duas caixas mais folga de manobra.

**Diretrizes de formato:**
- **Os dois conectores na MESMA aresta**, lado a lado. Define a largura mínima da placa e concentra a
  entrada do chicote num ponto — bom para vedação e alívio de tração.
- **Zonas, na ordem em que a aresta manda:** entrada de potência e TLE8888 junto do conector B; coreboard
  ao centro; **front-end de CKP/CMP junto do conector A e o mais longe possível do TLE8888, da ponte-H do
  ETB e dos relés**. É a regra do bloco 18, agora vinculando o formato.
- **Isolador USB junto ao seu próprio conector**, não junto ao MCU.
- **4 camadas** (sinal / terra contínuo / alimentação / sinal), já decidido.
- **Dimensionar para caixa de catálogo** — extrudido de alumínio (ex. Hammond 1455) ou die-cast. Escolher
  a caixa **primeiro** e desenhar a placa para ela; o contrário custa um respin. O alumínio também dá
  blindagem, o que num projeto com este histórico de EMI não é acessório.

⚠️ **Não coberto aqui** (resto do item 8, precisa de alvo de custo teu): *conformal coating*, grau de
proteção final, retenção mecânica do coreboard e orçamento da BOM.

---

## Conector — contagem original (superseded pela alocação acima)

Tabela completa em `docs/wiring_diagram.md` §"Conector do chicote do motor", expressa no que **sai da
placa** (saídas do TLE8888 + entradas de sensor + sync), não em pinos do MCU.

⚠️ **Não são ~47.** A estimativa antiga vinha de uma tabela que:
- contava `EWG pos` — diferido, e o pino do potenciómetro externo vai com ele;
- **omitia pressão de combustível e de óleo**, ambas populadas na v1;
- listava VVT por pinos do MCU (`PB6`/`PB7`) em vez das saídas OUT5/OUT6 que realmente vão ao chicote.

**Escolher o modelo de conector contra 55, com margem.** VBATT não gasta via (mede o rail interno via
divisor → `PC3`). Knock leva 2 vias **reservadas mas não cabladas** na v1. A reserva inclui a via do
2º sensor de came, para o caso do VVT dual.

---

## Verificação — ordem inegociável: bancada → ETB validado → motor
Nunca ligar injetores ou bobinas nas etapas 1–3.

0. **Fingerprint do TLE8888 — o PRIMEIRO teste com a placa alimentada, antes de tudo.**
   Ler `reserved[49]` na telemetria assim que o CI tiver alimentação e o SPI clocar.
   - `0x00` → o mapa de registadores está confirmado contra o silício. **É o único momento em que este
     teste é possível** (os valores de reset desaparecem na primeira escrita).
   - **Qualquer valor diferente de zero é bloqueante.** A leitura da máscara orienta o diagnóstico:
     - **Todas as 7 entradas set (`0x7F`)** → quase de certeza **barramento mudo**, não mapa errado.
       Verificar `PB12–PB15` e sobretudo que `auxiliaries_init()` não voltou a reclamar `PB12`/`PB13`.
       (O conjunto não tem valores de fronteira justamente para que um MISO a flutuar — alto ou baixo —
       dê sempre máscara cheia, em vez de um padrão que se confunde com mapa parcialmente errado.)
     - **Padrão parcial** → mapa efetivamente errado nessas entradas: comparar uma a uma com a Table 50.
     - ⚠️ **Máscara cheia num CI sabidamente bom** → suspeitar de **corrida de arranque**, não do mapa:
       se `tle8888_init()` correr antes de o CI estar pronto a responder, o fingerprint falha e **fica
       latched até reset**. Confirmar que a alimentação/ready do CI precede a init.
   - **Não prosseguir** com máscara diferente de zero; injeção e ignição estarão inibidas de propósito.
1. **Host** — `make host-test` e `make host-test-vgt6` verdes, incluindo o novo teste de VBATT.
2. **Lacunas pendentes do README §P2** (ambas abertas, ambas pré-requisito de partida):
   - Scope de INJ/IGN — latência e jitter contra o esperado (~0,4 µs / ~0,019° @8000), nos pinos
     `PE0/2/4/6` e `PE9/11/13/15` **com driver montado** e carga dummy resistiva.
   - Sync CKP/CMP de 200 a 8500 rpm com o estimulador (`tools/esp32_combined/`) **por TP-DIG** — a saída
     digital do ESP32 não excita a entrada diferencial do MAX9924 —, verificando fase sequencial em todo
     o range.
3. **Caracterizar o front-end CKP — a medição que valida o critério de precisão.**
   Não basta "sai pulso". **Por TP-VR**, com fonte analógica de verdade (gerador ou ESP32-DAC +
   atenuador): injetar seno VR de amplitude variável e medir o atraso entrada→`PA0` **em função da
   amplitude e da frequência**. Registrar **duas** grandezas, ambas importam (ver a correção do orçamento
   de erro): o **atraso médio** (vira a constante de compensação em firmware) e o **espalhamento**
   (parcela irrecuperável).
   ⚠️ **Rebaixado a sanity check.** O datasheet dá `tPDZ` = 50 ns e `tPD-JITTER` = 20 ns — 0,0024° e
   0,001° @8000 rpm, ordens de grandeza abaixo do piso mecânico. Não há o que qualificar aqui. O teste
   serve só para **confirmar que a montagem está correta** (modo A2 ativo, pull-up certo, sem filtro
   parasita) — se medir microssegundos em vez de dezenas de nanossegundos, há erro de montagem.
   O que realmente importa medir nesta etapa é o **comportamento em amplitude baixa** (cranking): que o
   threshold adaptativo arme e o dente faltante seja decodificado.
4. **Ruído sob carga — o teste que fecha o loop do bloco 4b.** Com injetores e bobinas dummy comutando,
   observar o par CKP no scope e confirmar sync estável (`ckp_isr`, estado de sync via
   `/api/debug/counters`). Sintonizar os resistores de gate aqui. Este é o teste que teria pegado o
   falso-sync original.
5. **Analógicos** — varrer cada divisor com fonte de bancada (incluindo fuel press e oil press);
   `adc_init_faults=0`; VBATT contra multímetro em 9 / 12 / 14,5 V. Medir ruído do 5 V dos sensores com o
   buck sob carga.
5b. **Flex** — injetar 50 Hz e 150 Hz em `PB5` e conferir 0 % e 100 % de etanol; varrer duty p/ a
   temperatura. (Comissionar **depois** do motor estável.)
5c. **VVT** — acionar cada solenoide pela aba TESTES com o motor parado, confirmar corrente e o
   flyback no scope. O gate de `FULL_SYNC` impede o PID de agir sem sync — confirmar que as duas saídas
   ficam em zero sem estimulador. (Comissionar **depois** do motor estável.)
6. **Gate ETB** — chicote real, `etb_harness_present=1`, **desacoplado do motor**: rodar o autocal
   (primeira execução em HW real), verificar limites gravados, exercitar o PID em toda a faixa, provocar
   falha (desligar feedback) e confirmar fechamento. **Só depois montar no motor.**
7. **Motor, escalonado:** cranking sem faísca nem combustível (só sync) → faísca sem combustível →
   partida com bomba armada, extintor à mão, corte do ETB ao alcance.

---

## Pendências — estado após a rodada de resolução (2026-07-20)

### ✅ Resolvidas

**1. Sensor CKP.** É **VR**. MAX9924 confirmado, sem risco de respin.

**2. Datasheet MAX9924.** Modo **A2** (`ZERO_EN`=GND, `INT_THRS`=GND, `BIAS`→GND, ref. interna 2,46 V),
peça **MAX9924 single**, saída **open-drain** (pull-up 3,3 V, remover pull-down de `PA0`),
`tPDZ`=50 ns / jitter 20 ns, desacoplamento 10 nF∥100 nF∥1 µF.

**3. VREF+ — ✅ RESOLVIDO (2026-07-20): (a) VDDA 3,3 V filtrado, com (c) como DNP.**

⚠️ **Correção a este próprio documento.** A versão anterior recomendava **(c)** e afirmava, com aspas de
datasheet, que *"The VREF+ pin is available only in LQFP100 and UFBGA100 packages"*. **Essa citação não
foi reconfirmada:** as três tentativas de fetch dos PDFs primários da ST deram timeout, e fontes
secundárias divergem até nas tensões do VREFBUF (1,8/2,048/2,4 numas, 1,65/1,8/2,048/2,5 noutras). A
afirmação traça à mesma leitura que produziu os erros do TLE8888 e do pull-down do `PA0`.
**Tratar como não verificada.** Não afeta a decisão — em (a) o VREF+ liga ao VDDA filtrado, exista pino
dedicado ou não. **Se algum dia se for para (c) ou (d), confirmar o pino na tabela de pinout primeiro.**

**A física:** `código = 4095 × Vin / VREF+` — o VREF+ é a régua, e qualquer erro nele é erro proporcional
em todas as leituras. Os sensores são ratiométricos (`V_out = k × V_alim`), portanto derivar o VREF+ do
mesmo 5 V faria `V5` cancelar-se algebricamente. **O cancelamento de (c) é real.**

**Porque (a) mesmo assim, e o argumento vem do firmware, não do hardware:**
1. **A malha fechada já absorve.** Erro de escala no MAP → erro no combustível → erro de lambda → o STFT
   corrige e o LTFT aprende. Deriva de referência é **lenta** (térmica), exatamente a escala de tempo que
   os trims tratam.
2. **TPS/APP/ETB nem precisam de exatidão absoluta.** `tps_raw_to_pct_x10()` dá uma **percentagem entre
   extremos calibrados**, e o ETB recalibra os batentes a cada power-on (`etb_autocal`). Um erro de escala
   comum desloca calibração e leitura na mesma proporção — **cancela-se sozinho, sem op amp**.

Sobra beneficiar de (c) só o conjunto absoluto (MAP, pressão de combustível e de óleo) — e o MAP, único
que alimenta as tabelas, é justamente o que a malha fechada corrige.

⭐ **O reframe que decide: os trims absorvem DERIVA, não absorvem RUÍDO.** Ruído no 3,3 V aparece como
dispersão amostra-a-amostra; o MAP é lido 1×/dente e perturba o combustível *desse ciclo*. Nenhum trim
apanha isso — corrigem a média, não a variância. Logo o esforço rende no **LDO high-PSRR, na filtragem do
VDDA e no layout**, não em tornar a referência absolutamente exata. **(c) ataca o termo já coberto e
deixa o que não está.**

**Custos de (c) que não se veem à primeira:** o offset e a deriva térmica do op amp entram em série com
todas as medições (trocas uma fonte de erro por outra, sem ganho líquido garantido); VREF+ ≤ VDDA obriga
a dividir 5 V → ~3,0 V, perdendo ~9% da escala; mais componentes na rede mais crítica; e se o divisor do
VREF+ e o *tracker* do TLE8888 não virem **exatamente o mesmo nó** de 5 V, o cancelamento é imperfeito.

**(b) VREFBUF é a pior das três aqui:** obriga a redimensionar todos os divisores (2,5 V de fundo de
escala estoura os atuais 0,3–2,7 V) **e também não dá cancelamento ratiométrico**.

**Consequência prática:** (a) é o que o código já assume, incluindo o `18000` de `vbatt_raw_to_mv()`.
**Zero rework.** Reservar footprint do divisor + buffer como **DNP**, para que (c) continue disponível
sem respin se o passo 5 da verificação mostrar que a exatidão de amplitude é limitante.

**4. Colisão `PB12`/`PB13` — ⚠️ NÃO era risco futuro, é BUG ATIVO no firmware.**
`tle8888_init()` (`main_stm32.cpp:500`) põe `PB12`=CS e `PB13/14/15`=AF5/SPI2. Depois
`auxiliaries_init()` (`main_stm32.cpp:638`) **sobrescreve o `GPIOB_MODER`** de `PB12` (`kFanPin`) e
`PB13` (`kPumpPin`) para GPIO comum. **Auxiliaries roda por último e vence.**

Consequências, hoje: **`SPI2_SCK` morre no boot** → o TLE8888 nunca é clockado, `tle8888_ok()` fica false
permanentemente e o bit `STATUS_TLE8888_FAULT` fica preso na telemetria. E `set_pump()`/`set_fan()`
escrevem `GPIOB_BSRR` nos bits 12/13, ou seja **chacoalham CS e SCK** do CI.
⚠️ **Severidade se um TLE8888 for montado:** `tle8888_poll_diag()` termina em
`tle_write(REG_WD_TRIG, 0x01u)` — o **watchdog do CI**. Sem SCK o watchdog nunca é alimentado e o
TLE8888 **desliga as saídas**: sem injeção e sem ignição. Deixa de ser cosmético e vira "o motor não pega".

**Resolução: mover bomba e ventoinha para `GPIOE`.** Pinos livres confirmados no VGT6 (o porto está
todo bondado, já que PE0–PE15 são usados até o PE15): **`PE1`, `PE3`, `PE10`, `PE12`, `PE14`**.
- **Bomba → `PE10`, ventoinha → `PE12`** (sobram PE1/PE3/PE14).
- Seguro conviver com INJ/IGN no mesmo porto: o acionamento é por **BSRR**, que é set/reset atômico por
  bit e **não** faz read-modify-write — escrever o bit 10 não perturba os bits 0/2/4/6. Só o `MODER` é
  RMW, e roda uma vez no init, antes do scheduler.
- ⚠️ **Tem de ser condicional por board** (`board_pinout.h`, como INJ/IGN já são): o RGT6 não tem GPIOE.
  No RGT6 mantêm-se `PB12`/`PB13` — lá o conflito com o SPI2 continua e precisa de decisão separada,
  mas o RGT6 não é o alvo desta placa.

**5. Impedância dos injetores e trigger das bobinas.** ~~Pendente~~ — **já decidido**: injetores de
**alta impedância** (acionamento saturado, casa com o firmware) e **bobinas com ignitor integrado**.
Falta só escolher os part numbers para a BOM, o que **não trava esquemático**: o level-shift 3,3→5 V já
está previsto como DNP no bloco 5, cobrindo bobinas que exijam 5 V no trigger.

**7. ETB a 20 kHz.** ✅ **RESOLVIDO — BTS7960 a 10 kHz** (decidido 2026-07-20, ver bloco 8).
Firmware alterado: `etb_pwm_init(20000u)` → `etb_pwm_init(10000u)`. O DRV8701 foi considerado e recusado
por descasamento de interface (o firmware é PWM+IN1+IN2 de 3 pinos, o DRV8701 é de 2) e por transferir a
responsabilidade do layout de potência. Chiado audível aceite.

### ⚠️ Reaberta pela investigação

**A. O driver do TLE8888 EXISTE — a premissa do "híbrido v1 discreto" caiu.**
O plano afirmava duas vezes que o driver SPI não existia. **Errado.** `src/hal/tle8888.cpp` tem 252
linhas, está no Makefile, é chamado de `main_stm32.cpp:500` e `:1234`, e é um driver completo: SPI2 a
3,9 MHz, handshake por chip ID, `configure_channels()` (INJ low-side, IGN push-pull, thresholds de OC,
slew), decodificação de diagnóstico por canal e trigger de watchdog. O fault bitmap já vai na telemetria.

Razões que **caíram**: "não existe driver", "é trabalho de firmware" e **"package difícil de soldar"**
(é LQFP-100, igual ao MCU). Razões que **permanecem**: disponibilidade/custo, driver nunca testado em
hardware, e discreto é mais fácil de depurar canal a canal.

⚠️ O inventário real do datasheet (bloco 12) mostra que ele eliminaria **muito mais** do que eu supunha:
FETs de injetor, drivers de bobina, **relés de bomba e ventoinha** (matando a colisão `PB12`/`PB13`),
relé principal, transceiver CAN e o regulador/trackers de 5 V dos sensores.
**E também elimina o MAX9924** — a interface VR dele é zero-crossing com armamento por pico, mesma
arquitetura, com clamp integrado e diagnóstico de sensor por cima (eu tinha afirmado o contrário, errado).
**O que NÃO elimina é a ponte-H do ETB** (meias-pontes de só 0,6 A).
**Decisão a rever conscientemente, não por premissa errada.**

**B. SDMMC também está compilado** (`src/hal/sdmmc.cpp` no Makefile) usando `PC8`, `PC12`, `PD2` —
e `PC8` é **IGN3 no RGT6**. Não achei chamada a `sdmmc_init()` em `main_stm32.cpp`, então parece
compilado mas não ativado. Confirmar antes de assumir que os pinos estão livres.

### Ainda dependem de ti

**Estado 2026-07-20: só o item 8 continua aberto.** Os itens 3 (VREF+), 6 (VVT) e 7 (ponte-H do ETB)
foram fechados — ver as respetivas secções.

**6. VVT com um came só.** ✅ **RESOLVIDO — aceite na v1** (2026-07-20). Montar os dois drivers,
**comissionar só o came instrumentado**, e a via do 2º sensor de came já está reservada na tabela de 55
vias do conector (grupo "Reserva"). Controle dual real exige 2º sensor **e** mudança de firmware (os dois
PIDs partilham o `pos_deg_x10` do único CMP, `auxiliaries.cpp:379-384`), e nada disso é pré-requisito de
primeira partida. **Comissionar depois do motor estável**, não antes — os solenoides comutam perto do par
CKP.

**8. Caixa, vedação, coating e orçamento.** Precisa de decisão tua sobre grau de proteção e custo alvo.

Sources:
- [MAX9924–MAX9927 datasheet (Analog Devices)](https://www.analog.com/MAX9924/datasheet)
- [TPIC8101 Knock Sensor Interface — TI](https://www.ti.com/lit/gpn/tpic8101)
- [HIP9011 Engine Knock Signal Processor — Renesas](https://www.renesas.com/en/products/automotive-products/automotive-sensors/automotive-sensor-signal-conditioners-sscafe/hip9011-engine-knock-signal-processor)
- [rusEFI knock sensing wiki](https://github.com/rusefi/rusefi/wiki/knock-sensing/989a497d9b9f999e3880fa458349d277e33c7104)
- [MAX9926 product page](https://www.analog.com/en/products/max9926.html)
- [Comparison of VR Options — BrickEMS](http://brickems.com/brickrpm/comparison/)
- [NCV1124 VR interface — rusEFI forum](https://rusefi.com/forum/viewtopic.php?t=993)
- [Crankshaft Position Sensors: VR or Hall? — rusEFI forum](https://rusefi.com/forum/viewtopic.php?t=737)
- [How to Use Slew Rate for EMI Control — TI](https://www.ti.com/lit/pdf/ssztbi0)
- [How to use slew-rate control for EMI reduction — EDN](https://www.edn.com/how-to-use-slew-rate-control-for-emi-reduction/)
- [Ratiometric 0.5–4.5 V output pressure transducers — SensorsONE](https://www.sensorsone.com/ratiometric-0-5-to-4-5vdc-pressure-transducers/)

---

## Adenda 2026-07-20 — implementação (itens 1, 2 e 3)

### Watchdog do TLE8888 — decisão fundamentada: **usar a variante -2QK na v1**

⚠️ **Não é opcional no -1QK.** Table 11/12 do datasheet:

```
Safe State  ⇐  WWDEC > 32  OU  FWDPC > 32  OU  TEC > 32
Safe State  ⇒  "Power stages: disabled"  ·  "O1E..O24E, IGN1E: 0"
```

Num TLE8888-**1QK**, se o firmware não servir o watchdog os contadores sobem e o CI **desliga injecção e
ignição sozinho**. Durante a primeira partida isso parece falha mecânica ou de combustível — o pior modo
de falha possível para diagnosticar.

⚠️ **Correcção a uma sugestão anterior deste plano:** pôr `WDREN = 0` **não resolve**. Esse bit só
controla se ocorre o *watchdog reset* no overflow dos contadores (cap. 6.4); o Safe State acontece antes,
aos 32, e já desliga os power stages. A opção "(c) WDREN permissivo" fica **retirada**.

Restam duas opções reais:
- **(a)** implementar WWD + FWD a sério. O WWD é simples (`WWDServiceCmd` periódico dentro da janela
  aberta); o **FWD é pergunta/resposta de 4 bytes com rotinas de auto-teste do MCU** — software de
  segurança com validação própria. **Implementar só o WWD não chega:** o `FWDPC` sobe na mesma.
- **(b)** usar **TLE8888-2QK**, com watchdog desactivado de fábrica. Mesmo LQFP-100, mesma pinagem —
  as variantes diferem *só* no watchdog.

**Recomendado para a v1: (b).** O watchdog é valioso, mas errá-lo mata o motor de forma imprevisível
exactamente na fase de bring-up, e o MCU já tem o seu `IWDG` independente. Fazer (a) depois, com
validação dedicada, e migrar para -1QK numa placa de produção.

**Mitigação implementada:** o driver **lê** `WdStat0`, `WWDStat`, `FWDStat0`, `TECStat` e `WdDiag`
(`tle8888_wd_status()`), sem servir o watchdog. Se um -1QK for montado por engano, a entrada em Safe
State aparece na telemetria em vez de se manifestar como "o motor morreu sem razão".

### INJEN / IGNEN — implementados

`INJEN`=**PE1** (pino 24) e `IGNEN`=**PE3** (pino 27), configurados em `out_pins_hw_init()` na **mesma
escrita BSRR atómica** dos 8 canais de INJ/IGN, e a **LOW = desabilitado**. Sobem via
`power_stage_enable()`, chamada em `main_stm32.cpp` logo após `tle8888_init()` e **condicionada a
`tle8888_ok()`** — se o CI não respondeu, injecção e ignição ficam inibidas por hardware.

É um caminho de corte independente do SPI *e* do escalonador, que o estágio discreto não oferecia.
No RGT6 é no-op. Sobra `PE14` livre em GPIOE.

### SDMMC — sem conflito hoje, agora impossível amanhã

`sdmmc_init()` **nunca é chamado**; `datalog_init()` só consulta `sdmmc_card_present()`, que devolve
false, deixando o datalog inerte. A configuração dos GPIO vive só dentro de `sdmmc_init()`, portanto
`PC8`/`PC12`/`PD2` nunca são reclamados — **não havia conflito real**.

Mas era armadilha latente: no **RGT6**, `PC8` é **IGN3**. Ligar o datalog no futuro reconfiguraria o pino
de uma bobina para AF12 — ignição morta, sem aviso. `sdmmc_init()` passa a **retornar false
imediatamente em RGT6**, sem tocar em GPIO. No VGT6 não há conflito (INJ/IGN vivem em GPIOE), por isso
o driver só é permitido nesse package.

---

## Adenda 2026-07-20 (cont.) — bloco 7 implementado

### VBATT em canal dedicado + EWG guardado — **feito**

`PC3`/INP13 deixou de ser `EWG_POS` e passou a **VBATT** (`AdcSecondaryChannel::VBATT`). O literal
`12000 mV` saiu de `sensors.cpp`: a tensão vem do ADC via `vbatt_raw_to_mv()`, com fallback de 12 V fora
da janela plausível de 6–18 V. **A dependência de `g_etb_harness_present` desapareceu** — era ela que
fixava 12 V com o chicote do ETB ligado, subestimando o dead-time e encurtando o dwell no cranking.

**Guard do EWG na mesma mudança** (como o plano exigia): `ewg_driver_read_position_raw()` devolve **0**
em vez de ler o canal, que agora é VBATT. Sem isso o PID do EWG passaria a perseguir a tensão da bateria.

**Bancada preservada:** não há divisor em `PC3` na bancada atual — o pino flutua. O modo de bancada
(`sensors_set_bench_clt_iat`) continua a fixar 12 V, então o setup HIL existente não regride.

Cobertura: `test_sensors_vbatt_dedicated_channel` (12 V, cranking a 9,5 V, independência do chicote,
fallback de raw implausível, modo de bancada). **1234 PASS / 0 FAIL**, `host-test-vgt6` 24/24,
`firmware-vgt6` linka.

⚠️ **Duas amarras que este teste NÃO prova:**
1. **`vbatt_raw_to_mv()` = `raw × 18000 / 4095` embute VREF+ = 3,3 V** (opção (a) do bloco 1), coerente
   com todos os outros conversores do código. Mas a decisão de VREF+ continua **aberta**: escolher (b)
   2,048/2,5 V ou (c) ~3,0 V derivado do 5 V obriga a redimensionar o divisor de `PC3` **e** esta
   constante, junto com todos os outros divisores.
2. O teste é de lógica, não de razão de divisor. A calibração real é o **passo 5 da verificação** (VBATT
   contra multímetro em 9 / 12 / 14,5 V): o `18000` tem de bater com os resistores efetivamente montados.

### ⚠️ ADIADA (2026-07-20) — polaridade da borda do CMP e do CKP

**Estado: pendente da escolha do sensor Hall.** A decisão foi conscientemente adiada porque depende do
part number (nem todo Hall é ativo-LOW). **Nada foi alterado no firmware** — `PA1` mantém o pull-down e
`TIM5` mantém captura só na subida nos dois canais. Retomar **antes de fechar o esquemático**, decidindo
CMP e CKP juntos. Análise abaixo.



Trocar o pull do `PA1` **não é a edição toda.** O CMP passa a ser Hall **open-collector**: idle HIGH,
pulso ativo LOW. Logo o *início* do dente do came é uma borda de **descida** — mas `TIM5_CCER` tem
`CC2E` sem `CC2P` (`timer.cpp:64`), ou seja **captura só a subida**. Trocar apenas o resistor faria o
firmware cronometrar o *fim* do pulso, não o começo.

Como a ISR (`ckp.cpp:1024+`) valida o delta entre bordas contra ~2 revoluções de CKP, **as duas
polaridades passam no gate temporal** (há uma borda de cada tipo por ciclo de came) — o erro não
apareceria como falha, e sim como um **deslocamento angular igual à largura do pulso**, absorvível pela
janela de dente do CMP. Por isso é decisão de projeto, não bug:

- **Capturar a descida** (setar `CC2P`) — segue a geometria do alvo do came; a largura do pulso e a sua
  tolerância não entram na fase.
- **Manter a subida** e calibrar a janela — zero mudança de firmware, mas a fase passa a carregar a
  tolerância da largura do pulso.

Recomendação: **descida**, pela mesma razão que o CKP usa zero-crossing — tirar da medida de tempo tudo
o que varia com amplitude/largura. Confirmar contra a folha do sensor Hall escolhido (nem todo Hall é
ativo-LOW) **antes** de fechar o esquemático.

⚠️ **O CKP tem exatamente a mesma pergunta em aberto.** `CC1E` também é **só subida**, e o front-end do
`PA0` também mudou (MAX9924 → `VROUT` do TLE8888, push-pull). "Qual borda do `VROUT` corresponde ao
dente" é questão de esquemático idêntica à do CMP — o sync é baseado em razão de períodos, então não
quebra, mas entra na mesma classe de deslocamento angular constante. **Decidir as duas juntas e
confirmar ambas na caracterização de bancada (passo 3).** Não fechar a do CMP deixando a do CKP por
examinar.
