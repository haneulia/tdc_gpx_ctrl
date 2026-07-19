# C08 Config Image Ownership Optimization

| Item | Value |
|---|---|
| FPGA | `xc7z020clg484-2` |
| Vivado | 2025.2.1 |
| Functional RTL | `5864438` |
| Control-set reporting | `ff82537` |
| Control-set sign-off gate | `21f0eff` |
| C06 regression | `260719_cfgowner_c06_c06_v002_regression` |
| C07 dedicated regression | `260719_cfgowner_c07_c07_v002_4chip_target` |
| Parent synthesis | `260719_cfgowner_synth_ps_fclk_parent_ref` |
| Parent implementation | `260719_cfgowner_impl_ps_fclk_parent_ref` |
| Verdict | `REFERENCE_IMPL_PASS_WITH_BOARD_IO_OPEN` |

## 1. Objective and decision

This checkpoint reduces replicated configuration state without adding a new
CSR, runtime parameter, arithmetic operation, or hardware-visible variable.
The accepted ownership rule is:

```text
CSR image -> config_ctrl -> chip_ctrl snapshot (single owner)
                                 |
                                 +-> chip_init direct read
                                 +-> chip_run direct read
```

`tdc_gpx_chip_ctrl` already snapshots the complete image before it dispatches
an init, runtime configuration write, or run operation. It also serializes its
children. The previous `tdc_gpx_chip_init` copied that stable image into both
an active image and a pending image. Those two copies did not establish an
independent transaction boundary and were therefore redundant.

The optimization is accepted. Functional regressions, parent synthesis, and
parent implementation all pass, routed area is lower, control-set count is
lower, and 200 MHz timing remains closed.

## 2. State removed and contract retained

The source RTL removed two `t_cfg_image` objects from each `u_init` instance:

| Previous state | Width per chip | New owner |
|---|---:|---|
| Active init image | 512 declared bits | `chip_ctrl/s_cfg_image_snap_r` |
| Deferred cfg-write image | 512 declared bits | Same coordinator snapshot |
| Deferred request intent | 1 bit | Retained in `u_init` |
| Busy-window coalesce sticky | 1 bit | Retained and CSR-visible |

Across four chips, 4,096 declared image bits were removed from source RTL.
The measured post-route `u_init` change from 530 to 82 FF proves that only one
448-FF image bank survived in the netlist. This matches 16 registers by the 28-bit
physical TDC bus width and indicates that Vivado had already pruned unused
upper bits and merged the mutually exclusive source storage. The measurable
reduction is therefore 448 FF per `u_init`, not 1,024 FF per instance.

The retained behavioral contract is:

1. `chip_ctrl` captures `i_cfg_image` before dispatch.
2. The snapshot remains stable while `chip_init` is active or has a deferred
   cfg-write request.
3. `chip_init` stores only the one-bit deferred intent and reads the held image
   when each register write is issued.
4. A second request while the one-entry deferred queue is occupied still sets
   `init_cfg_coalesced`; no diagnostic evidence was removed.
5. Reg14 bit 4 is still forced low, and the full-init Reg4 master-reset write
   is still generated from the held image.

A `synthesis translate_off` assertion stores a simulation-only reference image
and fails if the coordinator changes `i_cfg_image` during an active/deferred
operation. This keeps the ownership assumption executable without recreating
the removed hardware storage.

## 3. Functional verification

The new `tb_tdc_gpx_chip_init_cfg_owner` test covers the ownership boundary
directly:

| Scenario | Required result |
|---|---|
| Runtime cfg write | Register sequence uses the coordinator image |
| Full init | Configuration sequence and Reg4 master reset complete |
| Reg14 safety | Bit 4 is written as zero |
| Simultaneous start + cfg write | Start wins; cfg write is deferred and preserved |
| Repeated busy-window cfg write | Request coalesces and sticky sets |
| Stable-image contract | No assertion failure under `chip_ctrl` ownership |

The focused test passed with marker
`CHIP_INIT_COORDINATOR_CFG_OWNER PASS`. The existing `chip_ctrl` test also
passes, which proves the production coordinator keeps the new stability
contract.

The complete C06 regression passed its 32/64/128-bit output cases, 150/200 MHz
profiles, maximum-hit sweeps, bounded backpressure, status/CDC checks, and the
new focused test. The C07 dedicated four-chip regression passed both 64-bit
and 128-bit maximum-load cases, including beat/TLAST integrity and Hit[16]
metadata preservation.

## 4. Synthesis effect

The comparison baseline is
`260719_diagcollapse_impl_ps_fclk_parent_ref`. Both runs use the same part,
PS FCLK profile, output width, slope topology, and Vivado version.

| Hierarchy | LUT before | LUT after | Delta | FF before | FF after | Delta |
|---|---:|---:|---:|---:|---:|---:|
| Parent total | 21,224 | 20,700 | -524 | 30,815 | 29,026 | -1,789 |
| TDC instance | 15,805 | 15,281 | -524 | 24,393 | 22,604 | -1,789 |
| TDC core | 15,759 | 15,235 | -524 | 24,236 | 22,447 | -1,789 |
| `config_ctrl` | 7,097 | 6,571 | -526 | 14,801 | 13,012 | -1,789 |

Total post-synthesis control sets changed from 1,225 to 1,217. The four
`u_init` instances account for a gross 1,792-FF reduction; three FF are added
or retained elsewhere after netlist optimization, producing the parent net
delta of -1,789 FF.

## 5. Implementation effect

| Hierarchy | LUT before | LUT after | Delta | FF before | FF after | Delta |
|---|---:|---:|---:|---:|---:|---:|
| Parent total | 19,192 | 18,678 | -514 | 27,854 | 26,065 | -1,789 |
| TDC instance | 14,561 | 14,042 | -519 | 21,660 | 19,871 | -1,789 |
| TDC core | 14,517 | 13,997 | -520 | 21,503 | 19,714 | -1,789 |
| `config_ctrl` | 6,415 | 5,890 | -525 | 12,660 | 10,871 | -1,789 |

The repeated `u_init` hierarchy changed as follows after routing:

| Per-chip `u_init` metric | Before | After | Delta |
|---|---:|---:|---:|
| FF | 530 | 82 | -448 |
| Control sets | 7 | 5 | -2 |

| Control-set scope | Before | After | Delta |
|---|---:|---:|---:|
| Whole design, post-synthesis | 1,225 | 1,217 | -8 |
| Whole design, post-route | 1,193 | 1,184 | -9 |
| `config_ctrl`, post-route | 431 | 422 | -9 |

The parent gate now requires the generated control-set reports and enforces
the reviewed ceilings: design and `config_ctrl` must not exceed `1217/435` after
synthesis or `1184/422` after implementation. A future increase is not hidden;
it requires an explicit complexity review and baseline change.

## 6. Timing, CDC, and routing closure

| Gate | Previous route | Optimized route | Result |
|---|---:|---:|---|
| Design WNS | +0.204 ns | +0.305 ns | PASS |
| Design WHS | +0.035 ns | +0.019 ns | PASS |
| Design WPWS | +1.370 ns | +1.370 ns | PASS |
| FCLK0 WNS | +1.390 ns | +1.503 ns | PASS |
| FCLK1 WNS | +0.382 ns | +0.453 ns | PASS |
| FCLK2 WNS | +0.204 ns | +0.305 ns | PASS |
| Minimum bus-skew slack | +3.953 ns | +4.103 ns | PASS |
| Fully routed nets | 40,724 | 38,863 | 0 routing errors |

All 38 parent contract checks pass. Manual CDC remains exactly 40 first-stage
pins after synthesis and route. The CDC report remains at zero critical,
`CDC-6=0`, and 160 reviewed XPM FIFO `CDC-15` paths. Methodology and DRC rule
sets are unchanged; `NSTD-1` and `UCIO-1` remain explicit board-open waivers.

The FCLK2 worst setup path is:

```text
chip_ctrl/s_phase_r[0]
  -> chip_run/s_drain_cnt_ififo2_r[0]/CE
```

It has five logic levels, a 4.287 ns data-path delay, and +0.305 ns slack at
the 5.000 ns requirement. It is not a config-image path. The direct image read
therefore did not become the timing limiter.

## 7. HDL-to-HTML impact

This is an internal ownership optimization. It does not change any externally
visible timing geometry or simulator arithmetic:

- No change to `g_TDC_CLK_MHZ`, `g_AXIS_CLK_MHZ`, or `max_range_5ns_ticks`.
- No change to output width, beat/cell definitions, slope topology, or hit
  generation.
- No change to VDMA HSIZE/VSIZE/STRIDE, DDR service time, Ethernet repacking,
  payload size, packet count, or frame-margin calculations.
- No new CSR variable or HTML control is required.

The HTML must model external contracts, not the number of internal copies of a
configuration image. Adding a simulator parameter for this optimization would
misrepresent the HDL. The existing HTML arithmetic therefore remains valid and
requires no update for this checkpoint.

## 8. Sign-off boundary and next target

This checkpoint is fully accepted for the board-independent parent reference.
It is not final board sign-off: pin LOC/IOSTANDARD, PS7 board preset, external
TDC timing, and measured DDR/Ethernet reserve remain open until the board and
software stack are fixed.

The next structural audit should stay inside the per-chip control hierarchy.
After this change each `u_chip_ctrl` still owns 33 post-route control sets:
10 in the coordinator body, 11 in `u_run`, 5 in `u_reg`, 5 in `u_init`, and 2
in the response skid buffer. The four raw async FIFOs are XPM-owned and should
not be hand-rewritten merely to reduce a report count.

The next candidate is a read-only audit of `chip_ctrl`/`chip_run` phase and
watchdog enables, especially the current FCLK2 worst path. Any edit must first
prove that two enables have identical lifetime and reset semantics. Timing
retiming or enable merging is not justified by control-set count alone.
