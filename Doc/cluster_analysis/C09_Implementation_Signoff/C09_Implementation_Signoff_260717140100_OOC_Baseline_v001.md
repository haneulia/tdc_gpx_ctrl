# C09 TDC GPX OOC Synthesis Baseline v001

## 1. Decision

**Current sign-off: NO**

The RTL now has a reproducible, black-box-free out-of-context synthesis flow,
but the 200 MHz TDC timing target and data CDC checks do not pass. This
baseline is the comparison point for CDC hardening and complexity reduction;
it is not a release approval.

## 2. Reproducible evidence

| Item | Value |
|---|---|
| Git commit | `323e109fda88a1db8b99df2447759af87e273de6` |
| Git state | clean |
| Vivado | 2025.2.1 |
| Device | `xc7z020clg484-2` |
| Session | `2607171355_ooc_baseline_w32_a150_t200_dedicated_2x2_synth` |
| Configuration | output 32 bit, AXIS 150 MHz, TDC 200 MHz, AXI 100 MHz |
| Slope / stream | `DEDICATED_2X2` / `ASYNC` |
| Synthesis mode | `out_of_context`, timing XDC loaded before synthesis |
| Functional evidence | `260717_linepacker_synthfix_shot_bp_regression`, 7/7 PASS |

The source project omitted `tdc_gpx_sync_fifo.vhd` and
`tdc_gpx_line_packer.vhd`, and had no OOC DCP for either custom CSR IP. The
sign-off script reads those RTL files and both generated CSR implementations
explicitly. Post-synthesis black-box count is zero.

## 3. Timing baseline

| Clock | Target | WNS | TNS | Failing endpoints | Result |
|---|---:|---:|---:|---:|---|
| AXI-Lite | 100 MHz | +5.957 ns | 0 ns | 0 | PASS at synthesis |
| AXIS | 150 MHz | +1.074 ns | 0 ns | 0 | PASS at synthesis |
| TDC | 200 MHz | **-1.765 ns** | **-414.387 ns** | **690** | **FAIL** |

There are zero clockless registers, zero unconstrained internal endpoints,
zero setup/hold combinational loops, and no hold violation. External input and
output delays remain a parent-design responsibility.

The worst setup path starts at the per-chip raw asynchronous FIFO full logic
and ends in `s_raw_fifo_r(6).tdata(0)` inside the shift FIFO at
`tdc_gpx_chip_ctrl.vhd:1144`. It has eight LUT levels and a 6.762 ns estimated
data path. The methodology report contains 196 large setup violations, mostly
the same raw-FIFO shift/update cone. Placement may improve routing, but a
1.765 ns synthesis deficit is too large to waive or defer.

## 4. CDC baseline

The data-only CDC report excludes asynchronous reset pins and is the release
gate. Its summary is:

| Rule | Severity | Count | Classification |
|---|---|---:|---|
| CDC-4 | Critical | 4 | 28-bit register-read bundled data snapshot |
| CDC-10 | Critical | 14 | combinational source before synchronizer |
| CDC-6 | Warning | 23 | recognized multi-bit synchronizer; audit required |
| CDC-15 | Warning | 1,567 | clock-enable controlled/XPM handshake data; audit required |
| CDC-3 | Info | 185 | recognized one-bit synchronizer |

The 14 CDC-10 paths break down into two combinational CSR-idle reductions
(`tdc_gpx_csr_chip.vhd:751`), eight per-chip level crossings instantiated with
`SRC_INPUT_REG => 0` (`tdc_gpx_config_ctrl.vhd:1646` onward), and four bits of
the collision-vector OR reduction (`tdc_gpx_config_ctrl.vhd:960`). These can
be removed structurally by registering the source-domain levels before the
synchronizer.

The four CDC-4 paths are the per-chip 28-bit register-read buses sampled
continuously by `p_axi_rdata_snapshot` at `tdc_gpx_config_ctrl.vhd:1530`.
Comments rely on the synchronized `rvalid` pulse arriving after data settles,
but the bus and validity are not one atomic CDC primitive and have no explicit
bundled-data constraint. Replace this with a data+valid handshake or an
equivalent formally constrained snapshot protocol; do not waive it as-is.

## 5. Resource and complexity baseline

| Hierarchy | LUT | FF | LUTRAM |
|---|---:|---:|---:|
| Total | 28,521 (53.6%) | 43,536 (40.9%) | 688 |
| `u_cell_pipe` | 13,532 | 20,962 | 0 |
| `u_config_ctrl` | 10,462 | 15,315 | 112 |
| `u_output_stage` | 3,126 | 4,682 | 576 |
| `u_csr_pipeline` | 617 | 1,411 | 0 |

The design uses 1,925 control sets, above the Vivado 7.5% methodology
guideline. `tdc_gpx_cell_pipe.vhd:384` and `:415` instantiate rise and fall
builders for all four chips. In `DEDICATED_2X2`, runtime masks disable half of
them but synthesis still retains all eight builders. Four inactive builders
account for approximately 6.6k LUT and 10.3k FF, making static topology
generation the highest-value resource optimization after correctness closure.

## 6. OOC DRC scope

There are no `INBB-1` black-box violations. Remaining OOC critical warnings
are `NSTD-1` and `UCIO-1` for the 112 physical TDC bidirectional data pins.
Their I/O standard, location, board timing, and Zynq PS integration must be
closed in the parent FPGA project; they are not waived for system sign-off.
The two geometry alignment DSPs also report missing input/output pipeline
stages and should be reviewed after the 200 MHz raw FIFO path is fixed.

## 7. Closure plan

1. Remove CDC-10 structurally: register source levels/reductions and rerun CDC.
2. Replace the four register-read bundled-data crossings with atomic CDC and
   verify read data, valid, timeout, recovery, and soft-clear behavior.
3. Redesign the per-chip raw shift FIFO update so ready/full does not feed an
   eight-level whole-array shift cone; close 200 MHz timing before route.
4. Add static `DEDICATED_2X2` builder generation and tie off inactive lanes;
   preserve `SHARED_DUAL_EDGE` behavior and run both topology regressions.
5. Run synthesis corners: 32-bit 50/200, 32-bit 150/200, and 128-bit 200/200.
6. Run OOC place/route on the worst corner, then complete parent I/O/XDC and
   long-VDMA-stall policy verification.

Release sign-off requires: black boxes 0, internal unconstrained endpoints 0,
data CDC critical 0, post-route setup/hold WNS >= 0, internal critical DRC 0,
all width/topology/backpressure regressions PASS, and documented parent-level
I/O and long-stall closure.
