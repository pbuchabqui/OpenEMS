# Errata STM32H562 (ES0565)

Referencia local revisada: `es0565-stm32h562xx563xx573xx-device-errata-stmicroelectronics.pdf`, Rev 8, January 2026.

Impactos diretos no projeto:

- Revisao operacional considerada: X. Antes de teste real, confirmar a revisao fisica por marcacao do chip e `REV_ID` em `DBGMCU_IDCODE`, e registrar essa evidencia antes de liberar ensaio com atuadores.
- PA1 tem errata de histerese na revisao A: a histerese de entrada de PA1 so e habilitada quando PA0 esta configurado como entrada. Como CMP usa `PA1/TIM5_CH2`, revisao A exige condicionamento externo robusto ou troca de pino/placa. Em revisoes Z/X/W a limitacao consta como ausente.
- Flash tem limitacao de endurance de 1 kcycle nas revisoes A/Z. NVM/calibracao/seed de sincronismo nao podem gravar com alta frequencia. Para teste real, tratar Flash como recurso de baixa taxa e preferir commit explicito/event-driven.
- A primeira operacao de erase/program apos power-on ou Standby pode congelar fetch/read de Flash por cerca de 120 us. O caminho critico de CKP/scheduler nao deve depender de escrita Flash durante motor girando. Workaround completo exige vetor/handlers criticos e rotina da primeira escrita em SRAM.
- Read-while-write em Flash aumenta latencia em revisoes A/Z. Nao executar erase/program em Bank2 durante janela critica de injecao/ignicao/CKP.
- ADC: manter amostragem regular disparada por TIM6. Nao usar fila de conversoes injetadas, modo dual interleaved, watchdog analogico misturado com canais nao guardados ou stop de conversao injetada sem aplicar os workarounds da errata.
- FDCAN: nao habilitar edge filtering (`EFBI`) e evitar mistura de dedicated Tx buffer com Tx FIFO em prioridades diferentes. Se FDCAN1/FDCAN2 forem usados juntos, manter o mesmo nivel TrustZone/privilegio nos dois. O backend atual deve permanecer simples: sem edge filtering e com disciplina de fila unica/buffer unico.
- USB: em transferencias OUT, a SRAM USB pode ainda estar sendo atualizada quando o CTR dispara. Driver USB real deve inserir atraso minimo antes de ler buffer OUT: 800 ns em Full Speed.
- TIM: conexao de USB SOF para TIM2/TIM5 ITR12 pode falhar; o projeto nao deve usar USB SOF como referencia temporal de motor. Toda a base temporal do motor (captura CKP/CMP e dispatcher de INJ/IGN) vive em TIM5 e nao depende de USB SOF.
- IWDG nao acorda o sistema de Stop mode. O projeto de ECU nao deve depender de Stop mode para recuperacao por watchdog.

Conclusao operacional: considerando silicio Rev X, a limitacao de histerese em PA1/CMP e as limitacoes de endurance/read-while-write de Flash das revisoes A/Z nao sao bloqueios esperados. Continuam relevantes em Rev X: congelamento da CPU na primeira escrita/erase de Flash, cautelas de ADC, FDCAN, USB, TIM USB SOF e IWDG em Stop mode. Placa WeAct e firmware atual seguem adequados para bancada sem atuadores energizados. Teste real de motor ainda exige confirmar fisicamente a Rev X, limitar escrita Flash durante motor girando e validar USB/FDCAN/ADC com os workarounds acima.

