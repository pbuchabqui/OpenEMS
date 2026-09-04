---
name: dual-firmware
description: Hall and Encoder firmwares share engine physics; rotation capture stays independent. Use when changing src/engine, src/drv, src/hal, fuel, ignition, knock, MAP, STFT, limp, NVM, scheduler, CKP, encoder, or any firmware optimization.
---

# Dual firmware (Hall / Encoder)

Before any firmware edit, read **README.md § Dois firmwares (Hall / Encoder)**
(R1–R5 + tabela). That section is the rule. Do not restate it here.

## Trees

- Hall: `/home/pedro/PROJETOS/OpenEMS`
- Encoder: `/home/pedro/PROJETOS/openems-mt6835-encoder`

## Procedure

1. Classify every file you will touch using the README table (`capture` /
   `shared` / `adapter`). New files join the table in the same change.
2. `shared` — edit **both** worktrees in this session; `make host-test` on
   both; sibling commits (same title).
3. `adapter` — change the shared API first, then wire Hall and Encoder.
4. `capture` — one tree only; commit body says `hall-only` or `encoder-only`.
5. If dual-land looks impossible (README R4 closed list) or classification
   is unclear: **write no code**. Tell the user what would change, why it
   may not fit the other firmware, and what would remain owed. Wait.

"More work" is not an exception. Landing on one side to "port later" is
forbidden.
