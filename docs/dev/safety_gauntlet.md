# Safety gauntlet vs FOME-fw

Live audit of `feat/mt6835-encoder` against [FOME-fw](https://github.com/FOME-tech/fome-fw)
(`/tmp/fome-fw` @ `e558f4f`). Goal: firmware is safe to start real physical engine testing.

Updated: 2026-08-20 — critics picked **ours** on every piece (limp+bench needed a second round).

## Status

| Piece | Round | Builder | Critic pick | Biggest remaining gap (critic) |
|---|---|---|---|---|
| 1. Trigger / encoder decode | 1 | done | **ours** | No FOME-style noisy-edge corpus; CMP stale can keep sequential up to 6 revs; ±4.4° cam window. |
| 2. Injector scheduling | 1 | done | **ours** | (closed in round 2: TIM5 INJ_OFF at 1.2×, not poll-only) |
| 3. Ignition scheduling | 1 | done | **ours** | — |
| 4. Sync / resync recovery | 1 | done | **ours** | — |
| 5. Watchdogs | 1 | done | **ours** | FOME has no MCU IWDG; ours is 100 ms. |
| 6. Fault gating | 3 | done | **ours** | Closed: `oil_fault` now cuts fuel+spark at any RPM>0 once cranking ends. |
| 7. Bench vs running-engine interlocks | 2 | done | **ours** | FOME console `fuelbench`/`sparkbench` still ungated (their hole). |
| 8. Cranking / stall edge cases | 1 | done | **ours** | — |

Exit rule: critic must pick **ours**. All eight pieces now have that pick.

## Measurements (host tests **1879 PASS / 0 FAIL**; firmware `-Werror` builds)

| Metric | OpenEMS | FOME |
|---|---|---|
| Angular quantisation | **0.02197°/count** (TIM2 16384) | 6.000°/tooth (60-2) |
| 1 µs ISR lag @ 600 / 2000 / 6000 RPM | 0.00360° / 0.01200° / 0.03600° | same physics + 6° tooth hold |
| 3 µs dispatch margin @ 6000 RPM | 0.108° | GPT late window **8 µs** ≈ 0.29° |
| FOME 5% RPM-hold over one 6° tooth | n/a (compare-match) | **0.300°** |
| Inj-open timeout | **1.2× PW TIM5 event** + 2 ms poll | **none** (stall 0.1–2.4 s) |
| Coil overdwell | **1.5× TIM5 SPARK** + 1.4× poll | 1.5× only if fire still angle-queued |
| MCU IWDG | **100 ms** (`/32`, RLR=99) | **not used** |

## What changed for first-fire safety

- IWDG runtime is actually 100 ms (was ~0.8 s because PR stayed `/256`).
- Encoder inj watchdog is 1.2× PW and **schedules** TIM5 INJ_OFF (not 36 ms / poll only).
- Coil HIGH always queues TIM5 SPARK at 1.5× dwell.
- Watchdogs stay on during bench PW lock.
- Stall / `health_ok=false` / reverse ω → phase_invalidate + pins LOW + queues empty.
- 3 consecutive CMP rejects `phase_invalidate` (wrong cam half cannot keep sequential).
- TIM2 dispatch will not raise INJ/IGN under inhibit or reverse ω; OFF/SPARK always run.
- `limp_gating` is the single writer: fatal, rev+hyst, boost fuel-only, oil 1.5 bar after 5 s + 500 ms running, lambda 2 s with lift-only restore, ETB jam → 1500 rpm, flood, engine phase.
- Bench: `'P'`/prime/FIRE only if stopped + output_test; abort on RPM **or** TIM2 motion.

## Critic log

- Trigger+sync: **A**. Sequential needs calibrated CMP + 2 edges; stall/health force pins LOW. FOME 60-2 can fire wasted on an arbitrary 360° half.
- Inj/ign/watchdogs: **A**. Position compare + 1.2×/1.5× time backups + 100 ms IWDG. FOME: no inj timer, no IWDG, queued events survive limp.
- Limp+bench round 1: **B** (defaults were 0 so oil/lambda never tripped).
- Limp+bench round 2: **A** after oil/lambda/ETB jam defaults and scheduled INJ_OFF.
