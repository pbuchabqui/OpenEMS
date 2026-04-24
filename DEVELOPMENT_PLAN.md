# OpenEMS Development Plan (Reorganization + Execution)

**Date**: 2026-04-24  
**Context**: Repository has strong technical progress, but planning artifacts are fragmented between root and `openems-v3/` docs. This plan creates a single execution model.

---

## 1) Current Pain Points

1. Multiple roadmap documents with overlapping status, making it hard to know "source of truth".
2. Legacy reference trees (`src/`, `test/`, `openems-stm32h5/`) coexist with active code and increase cognitive load.
3. Existing phase plan (3/4/5) is feature-driven, but lacks explicit ownership, risk gates, and weekly checkpoints.
4. No clear Definition of Done (DoD) checklist across firmware, tests, docs, and release tasks.

---

## 2) Planning Principles

- **Single Source of Truth**: this file owns execution cadence; detailed technical specs remain in module docs.
- **Weekly Milestones**: every week must end with measurable outputs (code + tests + docs).
- **Risk-First Delivery**: prioritize work that de-risks hardware bring-up early (USB + CAN validation).
- **Gate-Based Progression**: no phase transition without passing objective quality gates.

---

## 3) Workstreams and Ownership

| Workstream | Scope | Primary Deliverables |
|---|---|---|
| WS1 Firmware Core | `openems-v3/src` | USB CDC production implementation, CAN FD filtering, integration fixes |
| WS2 Verification | `openems-v3/test` + host test flow | New/updated unit tests, regression pack, hardware validation checklist |
| WS3 Tooling & Release | `scripts/`, `Makefile`, versioning | Flash/verify scripts, release target, reproducible build flow |
| WS4 Documentation | root + `openems-v3/*.md` | Updated README, deployment/build guides, release notes |

> Recommended owner model: one DRI per workstream and one release captain per milestone.

---

## 4) 4-Week Execution Plan

## Week 1 — Planning Cleanup + USB Foundation

### Goals
- Lock planning baseline and remove ambiguity about active targets.
- Complete USB CDC low-level path skeleton with testable seams.

### Tasks
- Mark this file as execution source of truth.
- Normalize status references in README and roadmap docs.
- Implement `usb_cdc.cpp` core init/poll/send/receive flow (no advanced optimization yet).
- Add host-side USB CDC mocks and smoke tests.

### Exit Criteria
- `make host-test` passes.
- Firmware builds with USB symbols enabled.
- Team can state current status from one plan doc in <2 minutes.

## Week 2 — USB Dual I/O Completion

### Goals
- Reach stable TunerStudio communication over UART + USB.

### Tasks
- Dual-transport arbitration in `app/tuner_studio.cpp`.
- USB reconnect/disconnect handling.
- Frame-level robustness tests (fragmentation, timing jitter, fallback to UART).

### Exit Criteria
- TunerStudio basic command round-trip over USB and UART.
- No regressions in existing 17 suites.

## Week 3 — CAN FD Filtering + Routing

### Goals
- Implement deterministic CAN ingestion under load.

### Tasks
- Add configurable filter table and routing priorities.
- Implement extended frame handling (29-bit IDs, FD flags).
- Add host tests for filter-match and routing order.

### Exit Criteria
- CAN filter/routing tests pass.
- Hardware test receives expected WBO2 + diagnostics frames with correct routing class.

## Week 4 — Hardening + Release Candidate

### Goals
- Produce `3.0.0-rc1` candidate with complete operator documentation.

### Tasks
- Add `flash_stm32h562.sh` and verification flow.
- Create `docs/BUILD_INSTRUCTIONS.md` and `docs/DEPLOYMENT.md`.
- Run stability checklist and generate release notes.

### Exit Criteria
- Reproducible firmware build + flash validated.
- Release notes and docs published.

---

## 5) Phase Gates (Quality)

### Gate A: Integration-Ready (end Week 2)
- Host tests green.
- USB dual I/O stable for TunerStudio core commands.
- No new compiler warnings on touched modules.

### Gate B: Feature-Complete (end Week 3)
- CAN FD filtering operational with test evidence.
- Transport stack (UART/USB/CAN path interactions) validated.

### Gate C: Release-Ready (end Week 4)
- Build, flash, and rollback procedure documented and tested.
- Known issues list and mitigation notes included in release artifacts.

---

## 6) Definition of Done (DoD)

A milestone item is complete only when all criteria are met:

1. Code merged with clear module-level commit message.
2. Host tests pass and include new/updated coverage for modified behavior.
3. Hardware validation step recorded (or explicitly tagged as blocked with reason).
4. Documentation updated in same milestone window.
5. Risk log updated if any residual technical debt remains.

---

## 7) Risk Register (Initial)

| Risk | Probability | Impact | Mitigation |
|---|---|---|---|
| USB CDC timing/ring-buffer edge cases | Medium | High | Add stress tests with packet bursts + disconnect/reconnect cycles |
| CAN FD bit timing mismatch in hardware | Medium | High | Validate with known-good analyzer profile and controlled bus load |
| Plan drift across multiple docs | High | Medium | Weekly doc sync pass owned by release captain |
| Hidden dependency on legacy folders | Medium | Medium | Explicitly map active vs reference paths in README |

---

## 8) Weekly Rituals (Lightweight)

- **Monday (30 min)**: milestone planning + risk review.
- **Wednesday (15 min)**: checkpoint on gate blockers.
- **Friday (30 min)**: demo, test evidence review, plan update.

Output each Friday:
- completed items,
- blocked items,
- next-week gate target.

---

## 9) Immediate Next Actions (next 48 hours)

1. Align root README to explicitly reference this plan as the execution tracker.
2. Break Week 1 tasks into issue tickets (one ticket per deliverable).
3. Start USB CDC foundation implementation with host-mock-first strategy.
4. Record first risk review notes.

---

## 10) Success Metrics

- **Planning clarity**: team can identify current milestone + next gate without checking multiple files.
- **Delivery predictability**: at least 80% of weekly planned tasks completed.
- **Quality stability**: no prolonged red test periods (>1 working day).
- **Release confidence**: RC build/flash workflow executed end-to-end at least twice.

