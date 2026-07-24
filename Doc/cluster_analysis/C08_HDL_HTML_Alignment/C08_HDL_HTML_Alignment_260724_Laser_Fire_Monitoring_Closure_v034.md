# C08 Laser Fire Monitoring Closure v034

Date: 2026-07-24

## 1. Decision

The primary operational fault counter is `FIRE_TIMEOUT_COUNT`.

It increments exactly once when a physical laser command has been accepted and
issued, but the corresponding qualified `fire_done` is not received before the
executor watchdog expires. It is not inferred from `FIRE_CMD_COUNT -
FIRE_DONE_COUNT`; that difference can temporarily be one while a valid shot is
still in flight.

## 2. Event Contract

| Counter | Increment event | Lifetime |
|---|---|---|
| `FIRE_CMD_COUNT` | Executor accepts a non-simulation request and issues the physical command | Current mirror revolution |
| `FIRE_DONE_COUNT` | Executor accepts synchronized T0 for the active physical command | Current mirror revolution |
| `FIRE_TIMEOUT_COUNT` | Executor terminates an active physical command by watchdog timeout | Continuous from reset |

Simulation shots, scheduler rejects, invalid-configuration rejects, and a late
T0 arriving after timeout are excluded. Each accepted physical command has one
terminal outcome: qualified completion or timeout.

## 3. CSR Contract

| Address | Field | Meaning |
|---|---|---|
| `STAT0` `0x20`, bit 8 | `FIRE_TIMEOUT_OVERFLOW` | Sticky evidence that the timeout count wrapped; IRQ source |
| `STAT0` `0x20`, bit 9 | `FRAME_COUNT_OVERFLOW` | Sticky evidence that a per-revolution count wrapped; IRQ source |
| `STAT0` `0x20`, bit 10 | `FRAME_RESET_PENDING` | A face-0 reset is deferred until the active shot terminates |
| `STAT6` `0x38`, bits 15:0 | `FIRE_CMD_COUNT` | Physical commands in the current revolution |
| `STAT6` `0x38`, bits 31:16 | `FIRE_DONE_COUNT` | Qualified completions in the current revolution |
| `STAT7` `0x3C`, bits 31:0 | `FIRE_TIMEOUT_COUNT` | Missing-`fire_done` events since reset |

`FIRE_TIMEOUT_COUNT` intentionally wraps modulo `2^32`; it does not saturate
and it does not stop laser operation. On `0xFFFF_FFFF -> 0`, the sticky
`FIRE_TIMEOUT_OVERFLOW` bit remains set and raises IRQ, so the operator can see
that one or more wraps occurred even though the numeric count restarted. A
hardware/software reset clears the count and sticky, so software must log them
before reset.

## 4. Revolution Accounting

The command and completion support counters reset at the next `face_idx = 0`
start. If that boundary overlaps an active shot, reset is deferred until the
executor becomes idle. This keeps a command and its completion or timeout in
the same revolution interval.

At idle, `FIRE_CMD_COUNT - FIRE_DONE_COUNT` is a useful local cross-check. It
is not the long-term fault metric; `FIRE_TIMEOUT_COUNT` is authoritative.

## 5. Implementation

- `laser_ctrl_event_counters.vhd` owns the three counters, deferred revolution
  reset, natural wrap, and sticky overflow.
- `laser_ctrl_top.vhd` derives events only from executor-accepted physical
  commands and terminal results.
- `laser_ctrl_csr.vhd` transfers `STAT0..4`, `STAT6`, and `STAT7` as one
  224-bit coherent AXI-domain snapshot.
- Timeout and frame overflow are independent IRQ causes. Neither cause blocks
  command scheduling or firing.
- IP package revision 21 includes the new counter module in synthesis and
  simulation file groups.

## 6. Verification

| Level | Check | Result |
|---|---|---|
| Counter unit | Command/done/timeout classification | PASS |
| Counter unit | Deferred face-0 reset | PASS |
| Counter unit | Reduced-width timeout wrap and sticky overflow | PASS |
| Counter unit | Reset clears count and sticky | PASS |
| Full laser TB | CSR `STAT7` equals an independent physical-timeout monitor | PASS, `1211 == 1211` |
| Full laser TB | Normal physical command and completion counts are non-zero and equal | PASS |
| Laser regression | 10 testbenches | PASS; full integration `44/0` |
| IP Packager | DRC and source inclusion, revision 21 | PASS |
| Packaged OOC | `xc7z020clg484-2`, AXIS 150 MHz | PASS |
| System integration | AXIS/TDC 200/200 MHz | PASS |
| System integration | AXIS/TDC 150/200 MHz | PASS |

The full test initially compared a live independent monitor against a CSR
snapshot while new shots were still being admitted. Two new timeouts occurred
between sequential reads, producing `1213` versus `1215`. The corrected test
first stops admission and drains the active shot, then performs an exact
comparison. The final value was `1211` on both sides. This was a test snapshot
race, not a counter loss.

Relevant evidence:

- `C:/tmp/laser_ctrl_regression_fire_counters_v3`
- `C:/tmp/laser_ctrl_package_fire_monitor_v2.log`
- `C:/tmp/laser_ctrl_ip_ooc_fire_monitor.log`
- `sim_results/vivado_xsim/sessions/260724103300_laser_fire_monitor_200_200_system_integration_smoke`
- `sim_results/vivado_xsim/sessions/260724103700_laser_fire_monitor_150_200_system_integration_smoke`

## 7. Sign-off Boundary

The RTL, CSR visibility, wrap behavior, packaging, OOC synthesis, and available
integration simulation are closed for this counter contract. The current full
system integration scenario uses the Laser simulation-T0 path, so physical
timeout classification and exact CSR readback are proven by the dedicated full
Laser testbench rather than that scenario.

A parent implementation project does not yet exist. Board-level `fire_done`
electrical qualification, watchdog margin under routed timing, PS interrupt
handling, and operator polling/logging remain implementation sign-off items.
