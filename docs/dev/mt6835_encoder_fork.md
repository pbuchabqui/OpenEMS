# Fork MT6835 / TIM2 encoder — objetivo e estado

Branch: `feat/mt6835-encoder`, criado a partir de `main` (58c71d9), isolado em
worktree separado (`~/openems-mt6835-encoder`) para não tocar em
`hw/v1-clean-board`, que tinha alterações não commitadas no momento da criação
deste fork.

## Objetivo

Explorar a substituição do CKP/CMP atual (Hall + roda 60-2, `TIM5` input
capture) por um encoder magnético absoluto **MT6835** (SPI + saída ABZ), lido
em **modo encoder de hardware** no `TIM2`, com disparo de eventos de
ignição/injeção em **domínio de ângulo** em vez do domínio de tempo que o
scheduler usa hoje.

Isto nasceu da avaliação de um documento externo (revisão multi-rodada de um
pseudocódigo hipotético de firmware usando MT6835) que não correspondia à
arquitetura real do OpenEMS. A avaliação completa está preservada no histórico
de conversa que originou este fork; o resumo técnico relevante está abaixo.

## Mecanismo proposto

- `TIM2` em modo encoder: `CH1`/`CH2` decodificam quadratura ABZ do MT6835
  (posição angular absoluta, contada em hardware, sem ISR por borda).
- `CH3` do mesmo `TIM2` fica livre (não usado pelo modo encoder, que só
  consome `CH1`/`CH2`) e passa a gerar compare-match diretamente contra
  `TIM2->CNT` — ou seja, disparo por **ângulo alvo**, não por timestamp
  previsto. `CH4` fica livre também, sem uso atribuído por agora.
- Isto elimina a conversão ângulo→tempo que hoje é feita uma vez por gap em
  `ecu_sched_angle.cpp` (aproximação que assume RPM ~constante até o próximo
  dente) — o disparo por ângulo é imune a essa aproximação porque compara
  contra posição real, não contra um tempo previsto.

## Restrição de segurança que NÃO desaparece

O dwell watchdog real do código atual (`ecu_sched_dwell_watchdog()`,
`src/engine/ecu_sched.cpp:548`, ~linha 548 em `hw/v1-clean-board` — conferir
offset neste branch) depende de `TIM5_CNT` continuar avançando **mesmo com o
motor parado**, porque o cenário que protege é exatamente esse: motor cala
com a bobina a meio da carga, sem giro nenhum para gerar novos pulsos de
ângulo. Um contador de ângulo (`TIM2` em modo encoder) **congela** nesse
cenário — não serve como base de tempo para o watchdog.

Consequência de projeto: mesmo adotando disparo em ângulo via `TIM2`, é
obrigatório manter uma referência de tempo **independente da rotação**, só
para:
- watchdog de over-dwell,
- lógica de "motor parou → forçar saídas seguras".

**Decisão (revista, ver "Arquitetura base" abaixo): não é `TIM6`.** `TIM6` é
timer básico, sem canal de captura — não serviria nem para o CMP se
quiséssemos. `TIM5` **não desaparece, encolhe**: mantém a captura do CMP
(`CH2`) e continua sendo a referência de tempo sempre-corrente que o
watchdog já usa hoje — é o papel que `TIM5` já exerce, sem peça nova.

## Gates — FECHADOS, com fonte primária (2026-08-08)

Os dois gates que bloqueavam código de driver foram fechados contra fontes
primárias reais (datasheet do fabricante + reference manual/datasheet
STM32H562, não análises de IA). Nota de método: o ficheiro `MT6835.md`
encontrado na mesma pen drive do documento original **não é um datasheet** —
é outra especificação hipotética gerada por IA, repetindo os mesmos dois
erros já sinalizados (teto de "12 MHz" inexistente; MT6816 descrito como
substituto TSSOP-16, quando é SOP-8). Não foi usado como fonte aqui.

### Gate 1 — Datasheet primário MT6835 (MagnTek, Rev.1.3, 2022.12)

Fonte: `MT6835_Rev.1.3.pdf`, publicado em `magntek.com.cn` (site oficial do
fabricante).

- **Footprint confirmado:** TSSOP-16, part number `MT6835GT-STD` (§1, Figura 1).
- **ABFreq confirmado:** 2,048 MHz máx (§ ABZ Output Characteristics), com a
  nota do próprio fabricante, quase palavra-por-palavra igual ao que a
  revisão anterior tinha reconstruído: *"Even >2MHz ABFreq is available, but
  the INL could not be guaranteed when ABFreq is >2MHz."*
  **Não existe nenhum "12 MHz" em lugar nenhum do datasheet** — confirmado
  por busca no texto extraído do PDF inteiro. A crítica da revisão anterior
  estava certa; o número "12 MHz" é fabricado (aparece só nos dois documentos
  gerados por IA, nunca na fonte primária).
- **Achado novo e importante — teto de RPM à resolução máxima:** o próprio
  datasheet resolve o exemplo para 16.384 PPR (resolução máxima):
  `RS_MAX = 2,048 MHz / 16.384 = 125 Hz = 7.500 RPM`. A faixa alvo do motor
  vai a 9.000 RPM — ou seja, **em resolução máxima, o INL garantido do
  próprio fabricante já não cobre o topo de RPM do motor**. Isto não é uma
  preferência de projeto, é um limite documentado: resolução máxima e RPM de
  redline são mutuamente exclusivos aqui, reforçando com número oficial a
  bifurcação A/B já identificada (PPR baixo = compatível com o topo de RPM
  E com o ISR atual; PPR alto = melhor resolução, mas fora do INL garantido
  acima de ~7.500 RPM independente de qual timer decodifica o sinal).
  A 4.096 PPR (valor usado na revisão anterior), `RS_MAX` sobe para
  30.000 RPM — folga real, não hipotética.
- **Atraso de propagação, achado novo:** `TDelay` (Propagation Delay,
  Constant Speed) = **10 µs típico** (Electrical Characteristics). Isto
  alimenta diretamente o problema de [[trigger-offset-angular-vs-time-delay]]
  discutido na conversa que originou este fork: `10 µs × 9.000 RPM × 6 ×
  1e-6 ≈ 0,54°` — **acima do piso mecânico de 0,1–0,5°** que discutimos
  (runout + folga de montagem). Ou seja, ao contrário do exemplo ilustrativo
  de 1–5 µs usado na conversa, o atraso real e documentado do MT6835
  **ultrapassa** o piso mecânico em RPM alto — compensar isso (subtrair uma
  constante em µs do timestamp/ângulo capturado, não um offset fixo em graus)
  deixa de ser opcional e passa a gate de projeto, não só nota de rodapé.
- **Especificações magnéticas/mecânicas** (§6, Magnetic Input Specifications):
  `Bpk` (campo no plano do IC) 30–1.000 mT; `AG` (air gap) típico 1,0 mm,
  máx 3,0 mm; `DISP` (desalinhamento fora do eixo) máx 0,3 mm; ímã
  recomendado diametral Ø10 mm × 2,5 mm, NdFeB (coef. térmico −0,12 %/°C) ou
  SmCo (−0,035 %/°C). **O datasheet não especifica nenhum grau de ímã tipo
  "N45SH"** — essa especificação (e o piso térmico de 150 °C associado) não
  tem base na fonte primária; foi outra invenção dos documentos de IA. Se o
  projeto quiser um grau de ímã específico, é decisão de engenharia a parte,
  não algo "confirmado pelo fabricante".
- **SPI:** 4 fios, leitura de 21 bits de ângulo absoluto (não achamos
  necessidade de verificar largura de frame aqui — só relevante para o
  driver SPI, fora de escopo nesta etapa).

### Gate 1b — MT6816 vs MT6825 (comparação de footprint)

Confirmado via datasheets dos próprios produtos (busca web, fontes MagnTek/
distribuidoras):
- **MT6816 = SOP-8, 8 pinos.** Não é TSSOP-16. A afirmação "substituto
  drop-in do MT6816 em TSSOP-16" (presente nos dois documentos de IA) está
  errada — não é diferença de pinagem dentro do mesmo footprint, é
  footprint inteiro diferente (8 vs. 16 pinos).
- **MT6825 = TSSOP-16, 16 pinos, núcleo 18 bits**, mesma família de ABZ/UVW/
  PWM/SPI, mesmo teto de ABFreq (2,048 MHz) com a mesma ressalva de INL. É o
  candidato correto para qualquer discussão de compatibilidade de footprint
  com o MT6835 — a correção que a revisão anterior já apontava está certa.

### Gate 2 — Mapeamento PA0/PA1 → TIM2 no STM32H562 (LQFP100)

Fonte: datasheet STM32H562/H563, **DS14258 Rev 6**, Tabela 15 ("Alternate
functions AF0 to AF7"), extraído de `docs/alternatefunctions2.pdf` (já
presente no repo).

```
PA0   AF0=-   AF1=TIM2_CH1   AF2=TIM5_CH1   AF3=TIM8_ETR   AF4=TIM15_BKIN  ...
PA1   AF0=-   AF1=TIM2_CH2   AF2=TIM5_CH2   AF3=-          AF4=TIM15_CH1N ...
```

Confirmado pino a pino: os mesmos pinos físicos hoje usados para
`TIM5_CH1/CH2` (CKP/CMP) alcançam `TIM2_CH1/CH2` via seleção de função
alternativa (`GPIOx_AFRL`), mutuamente exclusivo com o uso atual em TIM5 (só
uma AF ativa por vez no mesmo pino). Não é necessário mudar pinos de
hardware para esta troca — só reconfigurar o registrador AF no firmware.

**Achado lateral, fora do escopo imediato mas registado:** PA0 também expõe
`TIM15_BKIN` em AF4 — ou seja, há break input disponível nesse mesmo pino via
TIM15 (não TIM1/TIM8 como o documento revisado assumia). Isto não muda a
conclusão já registada sobre BKIN ser incompatível com o mecanismo BSRR do
scheduler atual — só é uma nota para se essa discussão for reaberta depois.

**⚠️ Correção (encontrada só ao escrever `tim2_encoder_init()`, não durante o
fechamento da arquitetura): PA1 não pode ir para TIM2.** A leitura acima
mostra o mapeamento AF correto, mas a decisão original ("CH1/CH2 em
PA0/PA1") ignorava que **PA1 continua sendo `TIM5_CH2`/CMP** (decisão já
travada — "TIM5 encolhe, mantém CH2 = CMP"). Um pino físico só tem uma AF
ativa por vez: não dá para PA1 ser `TIM2_CH2` (canal B do MT6835) e
`TIM5_CH2` (CMP) ao mesmo tempo. Resolvido usando **PB3** (também
`TIM2_CH2`, AF1, confirmado na mesma Tabela 15 — livre na VGT6, é `INJ2`
só na variante RGT6) para o canal B, deixando PA1 intocado. `CH1` (canal A)
fica em PA0 normalmente, já que nada mais precisa desse pino uma vez que o
Hall CKP for retirado nesta hipótese. Lição: a checagem de conflito de pino
tem de ser feita pino a pino contra TODOS os usos simultâneos, não só
contra o mapeamento AF em isolado — o gate 2 confirmou que o AF existe,
não que o pino estava livre para esse uso específico.

## Arquitetura base — FECHADA (2026-08-08)

Decisões travadas antes de qualquer código de driver, para que a implementação
não fique "descobrindo" a arquitetura no processo.

### Verificação do mecanismo central — FECHADA (3 fontes independentes)

A peça mais crítica do desenho — `CH1`/`CH2` em modo encoder e `CH3`/`CH4`
livres para compare-match independente, no **mesmo** `TIM2` — foi
verificada, porque se estivesse errada a arquitetura toda cairia.

Três fontes independentes da comunidade ST convergem: o modo encoder (bits
SMS) só governa o roteamento de TI1/TI2 para o contador via `CCMR1`;
`CCMR2`/`CCR3`/`CCR4` são circuitos de compare independentes, não afetados
pelo SMS. A mais forte das três é uma resposta de um moderador técnico ST,
citando a figura exata do reference manual da família STM32H7 (mesmo bloco
de IP de timer que o H5 herda): *"The encoder interface is only connected
to the internal signals TI1FP1 and TI2FP2, which in turn are only routed to
CH1 and CH2 (see RM0468, e.g. fig. 373)"* — e conclui explicitamente que
essa restrição **é só do modo encoder em si**; `CH3`/`CH4` continuam
plenamente capazes de output compare ou input capture no mesmo timer. Uma
segunda fonte (outro thread ST) dá a receita concreta: *"set up CH3 or CH4
for the output compare (you don't need to actually enable it as output, nor
set up a GPIO pin for it) and bind the interrupt to the chosen channel in
DIER."*

**Ressalva de método:** não é uma citação literal do texto do RM0481
(capítulo TIM2) — duas tentativas de baixar o RM0481/AN4776 completos via
WebFetch deram timeout (documentos de milhares/dezenas de páginas). A
citação de figura é da família RM0468 (H7), não RM0481 (H5) — mesmo bloco
de IP, alta confiança de que a numeração de figura/nota é equivalente, mas
não confirmado byte a byte. Tratar como **fechado por convergência de 3
fontes técnicas independentes, com uma delas citando figura do RM da
família irmã** — não como citação primária H5 word-for-word. Se algo não
bater na bancada, este é o primeiro ponto a reabrir.

### Decisões locked

1. **Papel dos timers:**
   - `TIM2` (32-bit): `CH1`/`CH2` modo encoder (**PA0 + PA1**, ambos AF1,
     confirmados no gate 2) = ângulo do CKP a partir do ABZ do MT6835. `CH3`
     = compare-match em domínio de ângulo, reaproveitando o mesmo padrão
     fila-ordenada + rearmar-um-canal-HW que `TIM5_CH3` já usa hoje — só
     troca o "relógio" de base (tempo→ângulo); a lógica da fila não muda de
     forma. ⚠️ Revisão 2026-08-09: PA1 volta a ser canal B (decisão do
     utilizador, reverte a correção anterior) — ver seção "Revisão
     2026-08-09" abaixo para o CMP, que sai de PA1/TIM5 e passa a PB3/EXTI.
   - `TIM5` (32-bit): encolhe mais do que o previsto aqui originalmente —
     ver "Revisão 2026-08-09": CMP também sai de `TIM5_CH2`, então `TIM5`
     fica só com o papel de referência de tempo livre-corrente para o
     watchdog de dwell/stall (nenhum canal de captura em uso).
   - Saídas INJ/IGN: **inalteradas** — GPIO BSRR por software na ISR,
     mecanismo congelado preservado. Sem output compare físico do `TIM2` nos
     pinos de saída: com só 2 canais livres (`CH3`/`CH4`) para até 8 saídas
     lógicas com fases dinâmicas (dwell recalculado por VBat/RPM), OC
     reintroduziria exatamente a classe de falha (evento perdido, não só
     jitter) que já derrubou essa abordagem no scheduler atual — comparação
     detalhada de latência/jitter OC vs. BSRR na conversa que originou este
     documento.
2. **PPR programado: 4.096**, não os 16.384 máximos. Gate 1: a 16.384 PPR o
   fabricante só garante INL até 7.500 RPM (abaixo do redline de 9.000 RPM);
   a 4.096 PPR o teto sobe para ~30.000 RPM, com folga de ~3,3× a 9.000 RPM.
   Resolução resultante: 16.384 CPR pós-decodificação X4 → 0,0219°/passo.
3. **Leitura absoluta no key-on**: SPI lê os 21 bits de ângulo do MT6835
   antes do primeiro movimento, converte para a escala de `TIM2->CNT` e
   pré-carrega o contador. CMP continua obrigatório para desambiguar qual
   metade do ciclo de 720° (compressão vs. exaustão) — papel inalterado, só
   ancora contra o ângulo do `TIM2` em vez da contagem de dentes de hoje.
4. **Dwell**: continua exigindo conversão duração(tempo)→janela(ângulo) via
   RPM instantâneo — não desaparece, só encolhe de horizonte (hoje é
   recalculado 1×/gap inteiro; no novo desenho é 1×/cálculo de dwell,
   tipicamente 2–5 ms de antecedência). Mesma classe de erro de
   [[trigger-offset-angular-vs-time-delay]], horizonte menor.

### Itens antes abertos — agora resolvidos por design (2026-08-08)

1. ✅ **CH3/CH4 independentes do SMS** — fechado acima (3 fontes
   convergentes). Ver ressalva de método: revisitar se bancada discordar.

2. ✅ **Compensação do atraso de propagação — ponto de aplicação derivado.**
   `TDelay` = 10 µs significa que o ângulo reportado pelo sensor está
   atrasado em relação ao ângulo mecânico real: `ângulo_reportado(t) ≈
   ângulo_real(t − TDelay)`. Como `TIM2->CNT` é incrementado diretamente
   pelas bordas ABZ do sensor, o próprio `CNT` **já é** a leitura atrasada —
   não há como "corrigir o passado" nele. A correção certa é **subtrativa,
   no alvo do evento** (`CCR` de disparo), não na leitura de posição atual:
   ```
   correção_counts(RPM) = round(RPM × 6 × 10e-6 × (16384/360))
   target_cnt_compensado = target_cnt_desejado − correção_counts(RPM)
   ```
   A 9.000 RPM isso é ≈25 contagens (de 16.384/volta) — pequeno mas não
   desprezível (~0,15% da volta). O RPM usado na fórmula vem da mesma
   fonte que hoje alimenta `ecu_sched_angle.cpp`: amostrar `TIM2->CNT`
   junto com `TIM5->CNT` (que continua sendo a base de tempo) em cada
   volta/gap para estimar RPM instantâneo. **Isto cai exatamente no mesmo
   lugar arquitetural que `trigger_tooth0_engine_deg` ocupa hoje**
   (`engine_angle_to_trigger_angle()`, `ecu_sched_angle.cpp:47-53`) — só
   que passa de uma constante fixa em graus para uma correção
   dependente de RPM. Não é um mecanismo novo, é o mesmo mecanismo generalizado.

3. ✅ **Rotação reversa durante cranking — resolvido por composição das
   decisões já travadas, resíduo marcado para bancada.**
   Dois efeitos protegem contra o pior caso, nenhum deles novo:
   - **Disparo duplicado no mesmo alvo (bounce cruza o CCR duas vezes):**
     o padrão de fila já existente (mesmo em uso hoje no `TIM5`) rearma o
     canal de HW para o **próximo** alvo da fila imediatamente ao disparar
     o atual — então, no instante em que um segundo cruzamento do mesmo
     valor antigo pudesse ocorrer, o `CCR` já não aponta mais para ele.
     Proteção existente, não precisa de código novo.
   - **Dwell alongado/encurtado por oscilação do virabrequim durante o
     bounce:** o watchdog de over-dwell continua vivendo em `TIM5`
     (tempo real, independente de rotação — decisão já travada acima),
     então mesmo que a duração real do dwell em ângulo fique imprecisa
     durante um bounce de compressão, o limite de segurança (tempo
     máximo com a bobina ligada) continua valendo sem depender do
     `TIM2`. Isto não é uma proteção nova — é a mesma razão pela qual
     `TIM5` foi mantido em vez de mover tudo para `TIM2`.
   - **Resíduo não fechável por raciocínio, só por bancada:** a
     *precisão* do dwell (não a segurança) durante cranking com bounce
     real ainda precisa de validação empírica — osciloscópio no pino da
     bobina durante partida com compressão alta. Marcar como item de
     bancada, não de código.

## Diferença conhecida entre este fork e `hw/v1-clean-board`

Este branch parte de `main`, que está **15 commits atrás** de
`hw/v1-clean-board` e não tem:

- `docs/hw/architecture_v2.md` (decisão CKP=Hall, TLE8888 → CIs dedicados
  MC33810/TPS65381A/L9960T/CJ125).
- Versões atualizadas de `src/hal/board_pinout.h`, `src/hal/out_pins.{h,cpp}`,
  `src/hal/stm32h562/regs.h`, `src/hal/tle8888.cpp`.

O núcleo do scheduler (`src/engine/ecu_sched.cpp` e afins — dispatcher `TIM5`,
fila de eventos, BSRR, dwell watchdog) é **idêntico** nos dois branches; a
divergência está isolada em HAL/pinout/hardware. Quem for portar trabalho
deste fork de volta para `hw/v1-clean-board` precisa revisar esses ficheiros
manualmente — não é um merge direto.

## Estado: driver SPI + tim2_encoder_init() implementados (2026-08-08)

Gates, arquitetura base e os 3 itens que estavam em aberto (CH3/CH4,
compensação de atraso, rotação reversa) fechados — ver seções acima.
Implementado nesta etapa, compilado e testado (VGT6/RGT6/MRE + host-test +
host-test-vgt6, todos limpos — ver verificação abaixo):

- **`src/hal/mt6835_regs.h`**: mapa de registradores + protocolo SPI (frame
  de 24 bits, comandos, CRC-8), verificado byte a byte contra o datasheet
  primário (§7.6, §10) nesta sessão.
- **`src/hal/mt6835.{h,cpp}`**: driver SPI. Configura ABZ_RES=4096 PPR
  (write+readback a cada boot, não grava EEPROM — mesma filosofia do
  `tle8888.cpp`), lê ângulo de 21 bits com verificação de CRC-8, converte
  para escala de `TIM2->CNT`.
  - **`MT6835_HW_PRESENT = 0`** — mesmo padrão do EWG diferido
    (`ewg_driver.cpp`): o MT6835 não tem footprint em nenhuma PCB ainda,
    então nenhuma função toca GPIO/SPI de verdade enquanto a flag estiver
    em 0. O protocolo (a parte que não depende de hardware) está
    implementado e correto contra a fonte primária; a pinagem é
    placeholder.
  - **Barramento: SPI2, dono exclusivo** — ver "Revisão 2026-08-09" abaixo
    (o desenho original partilhava com o TLE8888; deixou de fazer sentido).
  - **CS: PC13, placeholder** — só verificado como "não referenciado em
    nenhum outro ficheiro de `src/`" nesta sessão, não é uma decisão de
    hardware confirmada (ver aviso no topo de `mt6835.cpp`).
- **`src/hal/timer.{h,cpp}`**: `tim2_encoder_init()` + `tim2_encoder_count()`
  + `tim2_encoder_set_count()` + `tim2_encoder_arm_next()` + `TIM2_IRQHandler`
  (limpa `CC3IF`, dispatcher em domínio de ângulo ainda não ligado — TODO
  explícito no código, fora de escopo). **CH1=PA0, CH2=PA1** — ver "Revisão
  2026-08-09" abaixo para o histórico (foi PB3, depois voltou a PA1).
- **`src/hal/stm32h562/regs.h`**: `TIM2_SMCR` + `TIM_SMCR_SMS_ENCODER_MODE3`
  + `SPI_CFG1_DSIZE_8BIT`, e um aviso permanente junto a `TIM2_CR1` sobre o
  conflito latente com `tim2_pwm_init()` (EWG).
- **`Makefile`**: `mt6835.cpp` adicionado a `HAL_COMMON_SRC`.

### Verificação feita nesta etapa

```
make firmware-vgt6   → build limpo, ELF/HEX/BIN gerados
make firmware-rgt6   → build limpo
make firmware-mre    → build limpo
make host-test       → 1252 PASS, 0 FAIL (binário corrido diretamente;
                        `make host-test` sozinho truncou a saída no terminal,
                        mas o binário em /tmp/openems-build/host/mvp_bench_tests
                        confirma o número real)
make host-test-vgt6  → 24 PASS, 0 FAIL
```

Nenhum teste novo foi escrito para `mt6835.cpp`/`tim2_encoder_init()` nesta
etapa — ambos ficam atrás de guards (`MT6835_HW_PRESENT=0`, e
`tim2_encoder_init()` não é chamado de lado nenhum ainda) que os tornam
inertes no build atual; os 1252+24 PASS confirmam ausência de regressão no
que já existia, não cobertura do código novo.

### Fora de escopo, continua para depois

- Ligar `tim2_encoder_init()`/`mt6835_init()` ao boot (`main_stm32.cpp`) —
  não feito de propósito: chamar isto agora ligaria um periférico
  (`TIM2`/`GPIOA0`/`PA1`) que ainda não tem sensor real por trás, e mudaria
  o comportamento observável do firmware sem hardware para validar contra.
- Dispatcher em domínio de ângulo real (substituir o `TODO` no
  `TIM2_IRQHandler`) — exige adaptar `ecu_sched*.cpp`, explicitamente fora
  de escopo desde o início deste fork.
- Confirmar CS real (hoje PC13 placeholder) contra layout quando o MT6835
  tiver footprint.
- Verificar o valor inicial assumido do CRC-8 (0x00) contra uma leitura real
  do sensor.

## Revisão 2026-08-09 — PA1↔PB3, TLE8888 descartado, gap do TIM5

Três pedidos do utilizador, um deles abre uma pendência nova.

### 1. Canal B do MT6835 volta a PA1

A correção anterior (mover canal B para PB3 por conflito com CMP em PA1) foi
**revertida a pedido explícito**: canal B fica em PA1/AF1 (`TIM2_CH2`), como
no desenho original do gate 2. Isto significa que PA1 sai de `TIM5_CH2`/CMP —
o CMP precisa de um pino novo. Implementado em `tim2_encoder_init()`
(`hal/stm32h562/timer.cpp`) e documentado em `hal/timer.h`.

### 2. CMP move para PC6/TIM3_CH1 — captura de hardware, não EXTI

Primeira tentativa foi PB3 via EXTI (ver histórico abaixo), mas o
utilizador pediu explicitamente para procurar uma solução melhor antes de
aceitar o custo do vetor IRQ não verificado. Achada: **PC6, TIM3_CH1
(AF2)** — canal de captura de timer de verdade, com tudo verificado contra
fonte primária, sem nenhum número adivinhado.

- **Livre na VGT6**: confirmado em `out_pins.cpp` — o bloco que reclama
  `GPIOC` pinos 6–9 para IGN1-4 só compila no `#else` (RGT6); o `#elif
  EMS_BOARD_IS_VGT6` usa exclusivamente `GPIOE`. `sdmmc.cpp` também usa
  PC8, mas `sdmmc_init()` não é chamado de lado nenhum no boot — não é uma
  reclamação ativa.
- **AF confirmada**: `Table 15` do DS14258 Rev 6, linha `PC6`: AF2 =
  `TIM3_CH1` (junto de `TIM8_CH1` em AF3, não usado). PC7/PC8/PC9 dariam
  `TIM3_CH2/CH3/CH4` pela mesma tabela, caso PC6 precise mudar no futuro.
- **IRQ já nomeado, não adivinhado**: `IRQ_TIM3 = 46` já existe em
  `hal/stm32h562/regs.h` (usado por `tim3_pwm_init()`), e
  `startup_stm32h562.cpp` já lista `TIM3_IRQHandler` na posição 46 do
  vetor (`"IRQ44=TIM1_CC, 45=TIM2, 46=TIM3, 47=TIM4"`). Isto elimina por
  completo o problema que bloqueava a rota EXTI: não há vetor
  desconhecido, não há endereço-base de `EXTI`/`SYSCFG` a verificar — a
  captura usa exatamente o mesmo padrão de registradores (`CCMR1`, `CCER`,
  `DIER`, `SR`) já em uso por `tim5_ic_init()`.
- **Custo aceito**: `TIM3` é partilhado com `tim3_pwm_init()` (PWM legado,
  só usado por RGT6, sem chamador hoje — ver `auxiliaries.cpp:478`, que
  registra o histórico de um bug real quando os dois tentaram coexistir).
  Documentado como aviso permanente em `hal/timer.h`, mesmo padrão do
  aviso já existente para o conflito `TIM2` EWG/encoder.

O ISR (`TIM3_IRQHandler`, `hal/stm32h562/timer.cpp`) grava `TIM2->CNT` —
o **ângulo** do encoder no instante do flanco do CMP — num snapshot
exposto por `cmp_angle_snapshot()`/`cmp_edge_count()`. Isto é mais direto
do que o esquema atual (timestamp em `TIM5`, correlacionado com dentes do
CKP à parte): a captura já entrega o ângulo, não precisa de conversão.
Orçamento de erro: CMP é ~75 Hz no redline; mesmo a latência de ISR mais
alta já medida neste projeto (~0,4 µs) equivale a 0,022° a 9.000 RPM,
irrelevante para uma função que só precisa de desambiguar qual metade do
ciclo de 720°.

**Histórico (descartado): PB3 via EXTI.** Verificado nas **duas** tabelas
do datasheet (`Table 15` AF0–7 e `Table 16` AF8–15): PB3 não tem nenhum
canal de captura de timer em nenhuma AF (`JTDO/TIM2_CH2/-/-/I2C2_SDA/
SPI1_SCK/SPI3_SCK/UART12` seguido de `SPI6_SCK/SDMMC2_D2/CRS_SYNC/
UART7_RX/-/-/LPTIM6_ETR/EVENTOUT`) — capturar ali exigiria EXTI, e o
vetor IRQ de EXTI0-4 neste `startup_stm32h562.cpp` não estava nomeado
(só `EXTI5_9_IRQHandler` existe), nem os endereços-base de
`EXTI`/`SYSCFG` deste HAL H5 (que não usa o mapa clássico F4) estavam
verificados. O risco real de adivinhar era limitado — nada mais ocupa
essas posições do vetor hoje, então um número errado só faria o handler
nunca disparar, não colidir com outro periférico — mas não implementado
às cegas mesmo assim. PC6/TIM3 tornou essa análise de risco irrelevante:
não há nada para adivinhar.

### 3. TLE8888 descartado — SPI2 simplificado

`mt6835.cpp` deixou de fazer save/restore de `SPI2_CFG1`/`CFG2` a cada
transação (existia só para coexistir com o modo diferente do TLE8888).
MT6835 agora configura o SPI2 uma vez, em `mt6835_init()`, e o mantém fixo
(modo 3, 8 bits). Como `mt6835_init()` deixou de poder assumir que
`tle8888_init()` já correu antes (não corre mais neste fork), também passou
a configurar os próprios pinos GPIOB (PB13/14/15, AF5) e o clock do SPI2 —
antes isto vinha de graça do `tle8888_init()`. `tle8888.cpp` continua na
árvore (não removido — é limpeza fora de escopo), mas com um aviso no topo
de `mt6835.cpp`: os dois drivers não podem ser inicializados no mesmo build
enquanto ambos disputarem o SPI2 num modo diferente.

### Gap TIM5_CEN — RESOLVIDO (2026-08-09)

A decisão #1 da arquitetura base ("`TIM5` encolhe, mantém `CH2`=CMP") já não
era exata: com o CMP também de saída (PC6/TIM3), `TIM5` ficava sem nenhum
canal de captura em uso — só o papel de contador livre-corrente para os
watchdogs de dwell/injeção continua vivo. `tim5_ic_init()` era a única
função que ligava `TIM5_CR1 |= CEN`, e nesta arquitetura nova nada a
chamaria (ela também reclama PA0/PA1 para captura, que já são
`TIM2_CH1/CH2` do encoder).

**Implementado**: `tim5_freerun_init()` (`hal/timer.{h,cpp}`) — mesmo
`PSC=3` (62,5 MHz/16 ns, `kTimPrescaler`) que `tim5_ic_init()` sempre usou,
sem GPIO/CCMR/CCER/DIER/NVIC nenhum, só `CR1=CEN` num contador de 32 bits
livre-corrente. Justificação do porquê o tick tem de ser idêntico:
`ecu_sched.cpp` lê `TIM5_CNT` **diretamente** (não via `tim5_count()`) em
`ecu_sched_dwell_watchdog()` e `ecu_sched_inj_watchdog()`, e todo o
orçamento de tempo do scheduler (`ECU_SCHED_CLOCK_HZ=62 500 000`,
`ecu_sched.h:47`, com `static_assert` em `ecu_sched.cpp:49-52`) assume esse
tick — um `PSC` diferente quebraria silenciosamente `g_dwell_ticks`,
`STM32_MIN_COMPARE_LEAD_TICKS` e todos os clamps já calibrados em ticks.

Os dois mapeamentos (`tim5_ic_init()` produção-Hall vs.
`tim5_freerun_init()` + `tim2_encoder_init()` + `tim3_cmp_ic_init()`
MT6835) são **mutuamente exclusivos** — documentado no cabeçalho de
`hal/stm32h562/timer.cpp` e em `hal/timer.h`. Ainda não ligado ao boot
(mesmo padrão dos outros três: periférico pronto, não chamado até haver
sensor real).

### Verificação desta revisão

```
make firmware-vgt6   → build limpo
make firmware-rgt6   → build limpo
make firmware-mre    → build limpo
make host-test       → 1252 PASS, 0 FAIL (binário corrido diretamente)
make host-test-vgt6  → 24 PASS, 0 FAIL
```

Implementado nesta revisão: `tim3_cmp_ic_init()` + `TIM3_IRQHandler` +
`cmp_angle_snapshot()`/`cmp_edge_count()` (`hal/timer.h`,
`hal/stm32h562/timer.cpp`) — não chamado do boot ainda, mesmo padrão de
"código pronto, não ligado" que `tim2_encoder_init()` já segue.

### Pendências abertas por esta revisão

1. ~~Confirmar o vetor IRQ de EXTI3~~ — **resolvido ao trocar de mecanismo**:
   PC6/TIM3_CH1 usa `IRQ_TIM3=46`, já nomeado no vetor, sem EXTI/SYSCFG
   nenhum.
2. Ligar `tim3_cmp_ic_init()` + `tim5_freerun_init()` ao boot junto com
   `tim2_encoder_init()`/`mt6835_init()` quando a decisão de ligar tudo ao
   `main_stm32.cpp` for tomada — hoje deliberadamente fora de escopo (mesma
   razão dos outros: periférico real sem sensor real por trás).
3. ~~Resolver o gap do `TIM5_CEN`~~ — **resolvido**, ver seção "Gap
   TIM5_CEN" acima.
