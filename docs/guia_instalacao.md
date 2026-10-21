# Guia de instalação

Tudo pelo dashboard proprio (`tools/openems_dash`, aba **Install**: um guia
passo a passo com faixas validadas, lampada de ponto, teste de saidas, gravar,
exportar/importar tune e restaurar padroes de fabrica). O TunerStudio
(`tools/ts/openems.ini`, menu Install) continua funcionando como alternativa. Parametros
obrigatorios: **6** (cilindrada, vazao do injetor, AFR estequiometrico, curva
de dead time, offset do trigger, polaridade CKP/CMP). O resto ja vem com
valores de fabrica utilizaveis.

1. **Ligacao.** Roda 60-2 no CKP (PA0) e sensor de fase no CMP. Sensor Hall
   open-collector (repouso alto): polaridade "falling" (page0 byte 258).
2. **Motor** (Install → 1): cilindrada (cc), vazao do injetor
   (cc/min a 3 bar) e AFR estequiometrico (14.7 gasolina, 13.0 E30, 9.0 E100).
   Valor fora da faixa: so aquele campo e rejeitado, o resto e aplicado, e
   o canal `cfgReject` mostra qual campo (bit0 cilindrada, bit1 injetor,
   bit2 AFR, bit3 MAP ref, bit4 offset, bit5 EOI).
3. **Dead time do injetor** (Tuning → Injector Dead Time): copie a curva da
   folha de dados do injetor (ms vs tensao).
4. **Teste de saidas** (Install → 4, aba Outputs, motor parado): arme o teste;
   cada botao dispara um pulso no injetor ou bobina escolhidos. Confirme que
   injetor 1 e bobina 1 sao do cilindro 1, etc. "Exit" no fim (sai sozinho
   apos 5 s sem comando ou se o motor girar).
5. **Offset do trigger, grosso:** conte os dentes do primeiro dente apos a
   falha ate o PMS do cilindro 1 (compressao). Offset = (720 − dentes × 6) mod 720.
   Ex.: PMS 14 dentes depois → 720 − 84 = 636.
6. **Primeira partida:** sem pedal. O status `syncFull` deve acender e o RPM
   ficar estavel. Combustivel de partida = REQ_FUEL × multiplicador por CLT,
   igual com ou sem sensor de fase.
7. **Offset fino com lampada de ponto** (Install → 6): motor em marcha lenta,
   lampada no cabo da vela 1, ligue "Timing light mode" (centelha fixa em
   10.0°, sem correcoes nem knock). Ajuste "Trigger tooth0 offset" (1°) e
   "Fine offset" (0,1°) ate a lampada marcar 10°. Desligue o modo e grave
   (Burn). O modo nunca fica gravado: sempre desliga ao ligar a ECU.
8. **VE em marcha lenta:** com sonda wideband, ajuste a celula de VE da
   marcha lenta ate lambda ≈ alvo; depois ligue o closed loop e deixe o
   LTFT aprender o resto.
9. **Tabela de avanco:** comece conservador; o canal "Advance" mostra o
   avanco enviado as bobinas com 0,1° de resolucao (−20° a 60°). Knock
   atrasa so o cilindro que detonou.
10. **Gravar tudo** (Install → 7) antes de desligar. Exporte o tune para
    guardar uma copia; "Restaurar padroes de fabrica" volta aos valores do
    firmware (`tools/openems_dash/base_tune.json`, gerado do proprio firmware).

