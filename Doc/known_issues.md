# TDC-GPX Controller — Known Issues / Intentional Trade-offs

This document records design decisions that multiple review rounds have
surfaced as "findings" but which are intentionally **not** code-fixed.
Documenting them here prevents future reviews from treating them as new
bugs.

Last updated: Round 10 (2026-04-19).

---

## Category A — Policy decisions (clamp-vs-reject on invalid runtime config)

The FSM receives a number of runtime config fields from software. When
the config is out of spec, individual modules could either:
- **Clamp** to a safe default and continue (current behavior), or
- **Hard-reject** (fail-closed: stop processing, raise sticky).

We chose clamp-to-default for the following because the SW contract
already validates these and a hard-reject would freeze the system on a
single typo in the configuration register:

| Module | Field | Out-of-spec value | Clamp | Review refs |
|---|---|---|---|---|
| `face_assembler` | `i_stops_per_chip` | `< 2` | 1 | Round 9 #13, Round 10 #8 |
| `face_assembler` | `i_max_hits_cfg` | `"000"` | 7 (via `s_max_hits_eff_u`) | Round 9 #8, Round 10 #9 |
| `header_inserter` | `cols_per_face` | `< 1` | 1 | Round 9 #11 |
| `header_inserter` | `s_max_hits_cfg_r` | `"000"` | 7 in both `cell_size` and header low byte | Round 10 #7 |
| `cell_builder` | `i_max_hits_cfg` | `"000"` | 7 (via `s_max_hits_eff_u`) | Round 9 #8 |

**Canonical 000 → 7 mapping** is now enforced consistently across all
three consumers after Round 9 #8 and Round 10 #7 / #9.

The `face_seq` start-validation layer still rejects on
`active_chip_mask=0`, `stops_per_chip<2`, `cols_per_face<1` — those are
"cannot proceed at all" cases. `max_hits_cfg=0` is survivable via the
canonical mapping and does not trigger a start reject.

---

## Category B — Intentional asymmetries / architectural choices

### 1. `cell_builder.ST_C_QUARANTINE` stays until drain_done or abort (Round 9 #9, re-raised Round 10 #10)
After a DROP-timeout enters QUARANTINE we do NOT auto-escape to IDLE
anymore. The tradeoff:
- **Exit early (original Round 6 A2 policy):** if a drain_done marker
  was lost, the FSM recovers after 65K cycles — but any late stale
  beats arriving afterward hit `tready='0'` in IDLE and stall upstream.
- **Stay forever (current):** late stale beats keep being absorbed
  (`tready='1'`); SW must issue `i_abort` to recover. This matches the
  "never stall upstream" policy that the rest of the pipeline uses.

Liveness concern is acknowledged — SW should monitor `o_shot_dropped`
and treat a stuck QUARANTINE as a call-for-abort condition.

### 2. Raw-axis FIFO carries control + data beats on one path (Round 9 #7, re-raised Round 10 #2)
`chip_ctrl` packs `drain_done` / `ififo1_done` control beats into the
same 6-deep FIFO as raw data. Depth=6 was chosen (Round 9 #7) to cover
the worst-case burst without separation. Full separation (two physical
paths) would add bookkeeping at every downstream consumer; the depth
bump provides equivalent "no drop under realistic backpressure"
guarantee without that churn.

### 3. Quasi-static config bundles use 2-FF sync + ASYNC_REG (Round 7 A-1, Round 9 #5, re-raised Round 10 #12)
`cfg` and `cfg_image` cross AXI→TDC via the 2-FF sync path rather than
`xpm_cdc_handshake`. Justification:
- Per-bit metastability IS fully guarded by the 2-FF pipeline with
  ASYNC_REG attribute.
- Per-bundle atomicity relies on the SW stability contract: SW writes
  config values, waits for the 2-FF sync window to settle (≥ 4 cycles),
  then issues the command pulse via `xpm_cdc_pulse`.
- The command pulse CDC latency itself (~4 cycles) closes the race
  window for realistic SW patterns.
- A full handshake would add ~4k flops for `cfg_image` alone.

Only the reg-access bundle (`cmd_reg_addr + cmd_reg_wdata`) uses
handshake (Round 9 #5) because it changes per-op and is the most
bit-skew exposed case.

### 4. Per-slope abort asymmetry (#22 Sprint 3 design intent)
- `shot_start_gated` / `face_start_gated` gate on **rise-only abort**
  (`s_pipeline_abort_rise`).
- `frame_done_both` waits on **both** slopes' completion.
- `frame_abort_count` (Round 9 #14) counts **either** slope's abort.

This matches the #22 Sprint 3 rise-primary / fall-optional
architecture: fall-side can abort independently without killing the
rise VDMA stream. Reviewers reading only one side can find this
asymmetric-looking; the gating in each direction is deliberate.

---

## Category C — Observability gaps (Round 11: RESOLVED)

Previously deferred pending CSR layout. Round 11 finalized STAT7 at
0x5C and wired all four signals end-to-end:

| Source | Signal | Resolution |
|---|---|---|
| `cmd_arb.o_reg_timeout_mask` | → `s_status.reg_timeout_mask` → STAT7[3:0] |
| `chip_run.o_timeout_cause` | → latched `run_timeout_cause_last` → STAT7[10:8] |
| `header_inserter.s_face_start_collapsed_cnt_r` | → new `o_face_start_collapsed_count` port → STAT7[23:16] rise + [31:24] fall |
| `cell_builder.s_stop_id_error_r` | → new `o_stop_id_error` port → sticky mask → STAT7[7:4] |

See `register_map.md` for the full STAT7 bit layout. Category C is
closed as of Round 11 follow-up.

---

## Category D — Interface deprecation held for backward compat

### `face_assembler.o_face_abort`
Permanently `'0'` since Round 4's c-simplified overrun policy. Grounded
at the top-level instantiation (Round 6 B5). Removing the port breaks
existing TB / third-party instantiations, so the port stays with a
DEPRECATED note in the entity declaration and an `o_face_abort => open`
pattern at the production instantiation.

Review recommendation ("remove the port entirely") is valid but not
prioritized — no functional risk.

---

## How to update this doc

When a review cycle finds a new item that is decided to be trade-off
rather than a bug, add it here (with the round reference) so future
reviews know it was considered.

When a deferred item graduates to "actually fix it", remove the entry
after the fix lands.
