# C08 Diagnostic CDC Complexity Optimization

| Item | Value |
|---|---|
| FPGA | `xc7z020clg484-2` |
| Vivado | 2025.2.1 |
| Functional RTL commit | `6210095` |
| Parent gate commit | `1dca2b2` |
| C06 regression | `260719_diagcollapse_c06_v002_regression` |
| C07 dedicated regression | `260719_diagcollapse_c07_c07_v002_4chip_target` |
| Parent synthesis | `260719_diagcollapse_synth_rerun_ps_fclk_parent_ref` |
| Parent implementation | `260719_diagcollapse_impl_ps_fclk_parent_ref` |
| Verdict | `REFERENCE_IMPL_PASS_WITH_BOARD_IO_OPEN` |

## 1. Objective

The first optimization checkpoint removes diagnostic state that crossed clock
domains but had no CSR or behavioral consumer. It does not add replacement
variables. Events that still affect integrity are folded into existing,
software-visible contracts or kept in the existing in-band data path.

This closes the old parent discrepancy in which 68 manual first-stage CDC pins
existed after synthesis but only 40 survived implementation. The contract is
now 40 pins at both stages:

```text
TDC pin status 24 + consumed diagnostics 12 + CSR idle 2 + command toggle 2
= 40 first-stage pins
```

The implementation commit contains 230 added and 412 removed lines, including
a new 167-line request-loss testbench. The RTL interface and status reduction
therefore remains substantial even after adding focused verification.

## 2. Observability consolidation

| Removed or reduced path | Retained contract |
|---|---|
| Duplicate `raw_overflow_mask` status field | Raw FIFO loss is OR-folded into existing per-chip `chip_error`/STAT5; existing raw-drop and drain-cap causes remain |
| Separate raw-control-drop sticky and CDC mask | Existing raw-drop/overflow evidence already proves a beat was lost |
| Drain-mismatch historical sticky | Per-shot mismatch remains on the final raw control beat as `tuser[5]` and follows the existing fault propagation path |
| Duplicate `drain_faulted_mask` latch and CDC | Same per-shot fault evidence remains in-band; no second historical mask is carried |
| Command-arbiter R+W ambiguity sticky | The discarded read is recorded by existing `reg_rejected` request-loss status |
| Per-chip register R+W ambiguity sticky | The discarded read is recorded by existing `err_reg_overflow_mask` request-loss status |
| Four-bit command-collision detail vector | Existing per-chip `cmd_collision_mask` remains software-visible |
| Force-reinit-used historical sticky | Existing `bus_fatal_mask` retains the fault history; the software-issued force-reinit command is the recovery audit action |
| Mid-shot STOPDIS debug sticky | Removed because it had no consumer; production integration must not assert the debug override during a shot |

The raw FIFO overflow behavior itself was not removed. Only the duplicate status
record field was removed, and the event was added to the existing chip-error
summary so loss cannot become silent.

## 3. Functional verification

The C06 regression passed the 32/64/128-bit output-stage cases, 150/200 MHz
dual-clock profiles, maximum-hit sweeps, bounded backpressure, status checks,
and the new request-loss test. The new test proves both consolidation rules:

1. Simultaneous command-arbiter read/write uses the defined write-wins policy
   and sets `reg_rejected`.
2. Simultaneous per-chip read/write while active preserves the pending write,
   sets `err_req_overflow`, and obeys software clear.

The C07 dedicated 4-chip regression also passed its 64-bit and 128-bit maximum
load cases, including output beat count, TLAST ownership, and Hit[16] metadata.
No failure or error marker was found in either archived session.

## 4. Synthesis effect

The post-synthesis netlist shows the expected structural reduction.

| Hierarchy | LUT before | LUT after | Delta | FF before | FF after | Delta |
|---|---:|---:|---:|---:|---:|---:|
| Parent total | 21,245 | 21,224 | -21 | 30,915 | 30,815 | -100 |
| TDC instance | 15,826 | 15,805 | -21 | 24,493 | 24,393 | -100 |
| `config_ctrl` | 7,114 | 7,097 | -17 | 14,901 | 14,801 | -100 |
| `cmd_arb` | 85 | 78 | -7 | 77 | 77 | 0 |

Post-synthesis control sets changed from 1,229 to 1,225. The improvement is
small because this checkpoint removes status storage and CDC chains; it does
not yet restructure the dominant per-chip enable/reset families.

## 5. Implementation effect

Post-route utilization is not monotonic with the synthesis reduction. Vivado's
physical remapping produced the following result from the same implementation
flow:

| Hierarchy | LUT before | LUT after | Delta | FF before | FF after | Delta |
|---|---:|---:|---:|---:|---:|---:|
| Parent total | 19,166 | 19,192 | +26 | 27,834 | 27,854 | +20 |
| TDC instance | 14,535 | 14,561 | +26 | 21,640 | 21,660 | +20 |
| `config_ctrl` | 6,373 | 6,415 | +42 | 12,640 | 12,660 | +20 |
| `cmd_arb` | 80 | 74 | -6 | 77 | 77 | 0 |

Post-route control sets still improved from 1,197 to 1,193. The `+26 LUT` and
`+20 FF` parent delta is less than 0.15%, but it means this checkpoint must be
classified as a source/CDC complexity reduction, not as proven routed-area
reduction. No route-area claim should be made from the synthesis numbers alone.

## 6. Timing, CDC, and routing closure

| Gate | Previous route | Optimized route | Result |
|---|---:|---:|---|
| Design WNS | +0.280 ns | +0.204 ns | PASS, above +0.100 ns gate |
| Design WHS | +0.036 ns | +0.035 ns | PASS |
| Design WPWS | +1.370 ns | +1.370 ns | PASS |
| FCLK0 WNS | +1.614 ns | +1.390 ns | PASS |
| FCLK1 WNS | +0.512 ns | +0.382 ns | PASS |
| FCLK2 WNS | +0.280 ns | +0.204 ns | PASS |
| Minimum bus-skew slack | +4.137 ns | +3.953 ns | PASS |
| Manual CDC, synth | 68 | 40 | Exact new contract |
| Manual CDC, route | 40 | 40 | Exact new contract |
| Fully routed nets | 40,691 | 40,724 | 0 routing errors |

All 38 parent contract checks passed. The post-route CDC report contains zero
critical findings, `CDC-6=0`, and the same 160 reviewed XPM FIFO `CDC-15`
warnings. The only manual diagnostic CDC pins are four bits each of
`cmd_collision`, `err_bus_fatal`, and `init_cfg_coalesced`.

The methodology and board-open baselines remain unchanged: `TIMING-9=1`,
`TIMING-18=292`, `TIMING-37=1`, `ULMTCS-1=1`, plus explicit `NSTD-1` and
`UCIO-1` board I/O waivers. This is reference integration closure, not board
bitstream sign-off.

## 7. HDL-to-HTML impact

This optimization does not change timing geometry, VDMA HSIZE/VSIZE, output
width, slope topology, distance packing, DDR service time, or Ethernet payload
calculations. The HTML arithmetic therefore requires no change.

The removed signals must not be introduced as simulator parameters or shown as
independent verified status. An HTML status view should represent only a
defined CSR or in-band contract. The parent profile remains 200 MHz TDC,
150 MHz AXIS, 32-bit output, `DEDICATED_2X2`, and two independent VDMA/HP paths.

## 8. Decision and next optimization

This checkpoint is accepted because it reduces RTL/state/CDC complexity,
preserves event evidence through existing contracts, passes full functional
regression, and closes parent implementation. It is not counted as a routed
area or timing improvement because both metrics moved slightly in the opposite
direction while remaining within their gates.

The next useful target is the replicated control-set structure inside
`config_ctrl`, not another status-variable cleanup. Before editing RTL, produce
a hierarchical `report_control_sets` baseline and identify one repeated
enable/reset family. Change one family at a time, then require C06+C07, parent
SYNTH, and parent IMPL to pass. For that checkpoint, compare both synthesis and
post-route utilization so physical remapping cannot hide a regression.
