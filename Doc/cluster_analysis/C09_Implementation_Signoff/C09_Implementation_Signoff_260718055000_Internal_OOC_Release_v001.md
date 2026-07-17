# C09 TDC GPX Internal OOC Implementation Sign-off v001

## 1. Decision

**TDC_GPX_TOP internal OOC implementation sign-off: PASS**

**Parent FPGA, bitstream, and board sign-off: OPEN**

The internal RTL is ready to become the frozen comparison baseline for the
next complexity-reduction pass. This decision does not authorize a hardware
release. Parent clock insertion, I/O delay, pin, IOSTANDARD, PS7, and long
backpressure policy closure are outside this OOC result and remain mandatory.

The smallest routed setup margin in the representative matrix is +0.064 ns.
That is a valid OOC pass, but it is intentionally not treated as parent-level
guardband because the OOC clock ports have no final `HD.CLK_SRC` locations.

## 2. Evidence identity

| Item | Value |
|---|---|
| Evidence Git commit | `a0f9f06b214b85a0bbbdde5f2aa45452ab42416e` |
| Git state in every matrix session | clean |
| Vivado | 2025.2.1, build 6403652 |
| Device | `xc7z020clg484-2` |
| Matrix CSV | `signoff_results/matrices/260718034937_representative_matrix.csv` |
| Matrix result | 10/10 PASS |
| Routed high-rate cases | 5/5 PASS |
| C03 functional session | `260718021635_c07_v001_c03_direct_matrix` |
| C06 functional session | `260718021844_c06_v002_regression` |

The functional sessions exercise the RTL committed by `6091763`. From
`6091763` through the evidence commit, only the three OOC sign-off scripts
changed; no VHDL source changed.

## 3. Functional closure

| Regression | Coverage | Result |
|---|---|---|
| C03 direct matrix | 19 simulation logs; 32/64/128 bit; max_hits 1/3/5/7; dual buffer; input FIFO timeout; drop/quarantine; stops snapshot | 19/19 PASS |
| C06 integration | 13 simulation logs; chip control; face sequence; both slope topologies; range ticks; atomic register response; status; 32/64/128 bit top integration; AXIS 150/TDC 200; bounded backpressure | 13/13 PASS |

The `PH_RESP_DRAIN fatal` text in the chip-control log is an injected
quarantine/recovery scenario. Its checks finish with PASS and it is not an
unexpected regression failure.

The cell-buffer INIT state adds one AXIS clock to each newly allocated shot
buffer: 5 ns at 200 MHz and 20 ns at 50 MHz. Upstream skid/backpressure
preserves the first event. The direct and integration regressions verify this
contract across all supported output widths.

## 4. Frozen clock and CSR contract

1. `g_OUTPUT_WIDTH` supports 32, 64, and 128 bits. The default remains 32 bits.
2. `g_AXIS_CLK_MHZ` and `g_TDC_CLK_MHZ` support 50, 100, 125, 150, and 200 MHz.
3. Signal-processing closure requires `g_AXIS_CLK_MHZ <= g_TDC_CLK_MHZ`.
4. End-to-end processing margin is limited by the slower AXIS domain. A local
   watchdog still counts clocks in its own domain.
5. `max_range_5ns_ticks` is the single CSR representation of physical
   round-trip range. Software always supplies 200 MHz reference ticks, or
   fixed 5 ns units, independent of the implemented clocks.
6. Each consuming domain converts that value once with the elaboration-time
   `fn_range_5ns_ticks_to_clks` ratio. No runtime multiplier/divider and no
   second per-domain CSR variable are required.
7. `ASYNC` is the normal mode for independent AXIS and TDC clocks. `SYNC` is
   valid only when both ports are driven by the same clock source and phase
   relationship, not merely when both generics contain the same MHz value.

## 5. Representative matrix

`post_synth` rows use synthesis timing as a hard gate. All AXIS 200 MHz rows
use routed timing as the hard gate; a negative synthesis estimate is advisory
for those implementation runs.

| Width | AXIS/TDC MHz | Topology | Clock mode | Gate | AXIS WNS | TDC WNS | LUT | FF | Builders |
|---:|---:|---|---|---|---:|---:|---:|---:|---:|
| 32 | 50/50 | DEDICATED_2X2 | ASYNC | post_synth | +15.118 | +15.555 | 18,294 | 32,859 | 4 |
| 64 | 100/100 | SHARED_DUAL_EDGE | ASYNC | post_synth | +5.088 | +5.157 | 26,442 | 45,987 | 8 |
| 128 | 125/125 | DEDICATED_2X2 | ASYNC | post_synth | +2.710 | +3.487 | 23,914 | 40,260 | 4 |
| 32 | 150/150 | SHARED_DUAL_EDGE | ASYNC | post_synth | +1.300 | +1.899 | 24,784 | 43,303 | 8 |
| 32 | 200/200 | DEDICATED_2X2 | ASYNC | post_route | +0.154 | +0.087 | 16,871 | 30,046 | 4 |
| 64 | 200/200 | DEDICATED_2X2 | ASYNC | post_route | +0.064 | +0.219 | 18,781 | 32,617 | 4 |
| 128 | 200/200 | DEDICATED_2X2 | ASYNC | post_route | +0.093 | +0.216 | 22,637 | 37,465 | 4 |
| 128 | 200/200 | SHARED_DUAL_EDGE | ASYNC | post_route | +0.083 | +0.204 | 29,033 | 48,228 | 8 |
| 128 | 50/200 | SHARED_DUAL_EDGE | ASYNC | post_synth | +14.633 | +0.560 | 30,052 | 50,959 | 8 |
| 128 | 200/200 | DEDICATED_2X2 | SYNC | post_route | +0.101 | +0.237 | 22,092 | 36,556 | 4 |

Every row has:

- `CDC-4 = 0` and `CDC-10 = 0`
- `no_clock = 0`
- `unconstrained_internal_endpoints = 0`
- the expected 4 or 8 elaborated cell builders

## 6. Routed timing closure

| 200 MHz case | Overall WNS | TNS | Setup fail | WHS | Hold fail | WPWS | Pulse fail |
|---|---:|---:|---:|---:|---:|---:|---:|
| 32-bit dedicated ASYNC | +0.087 | 0 | 0 | +0.058 | 0 | +1.370 | 0 |
| 64-bit dedicated ASYNC | +0.064 | 0 | 0 | +0.071 | 0 | +1.370 | 0 |
| 128-bit dedicated ASYNC | +0.093 | 0 | 0 | +0.070 | 0 | +1.370 | 0 |
| 128-bit shared ASYNC | +0.083 | 0 | 0 | +0.071 | 0 | +1.370 | 0 |
| 128-bit dedicated SYNC | +0.101 | 0 | 0 | +0.061 | 0 | +1.370 | 0 |

All five designs are fully routed with zero routing-error nets. The 128-bit
dedicated and shared runs reproduced the earlier clean first-route results
exactly: +0.093 ns and +0.083 ns. Neither needed the conditional post-route
physical-optimization fallback.

## 7. CDC, methodology, and DRC scope

The worst shared ASYNC data-only CDC report contains recognized `CDC-3`,
`CDC-6`, and `CDC-15` synchronizer/handshake structures. The formerly open
critical classes remain absent:

| CDC rule | Result |
|---|---:|
| CDC-4 bundled multi-bit unknown CDC | 0 |
| CDC-10 combinational source before synchronizer | 0 |

The OOC methodology report still contains:

- `TIMING-18`: external input/output delays are owned by the parent design.
- `ULMTCS-2`: control-set count remains high and is the next internal
  complexity target.

The OOC DRC critical warnings are only `NSTD-1` and `UCIO-1` for the 112 TDC
bidirectional pins. `ZPS7-1` and `HD.CLK_SRC` are also parent-project concerns.
They must not be severity-demoted to obtain a bitstream. The Tcl Store catalog
warning is an installation-state issue; the normal open-project OOC flow and
all generated reports completed successfully.

## 8. Complexity baseline and next optimization

At 128-bit/200 MHz ASYNC:

| Item | DEDICATED_2X2 | SHARED_DUAL_EDGE | Delta |
|---|---:|---:|---:|
| Total LUT | 22,637 | 29,033 | +6,396 shared |
| FF | 37,465 | 48,228 | +10,763 shared |
| Cell-pipe LUT | 6,410 | 12,781 | +6,371 shared |
| Cell-pipe FF | 10,946 | 21,690 | +10,744 shared |
| Control sets | 1,382 | 1,950 | +568 shared |

`DEDICATED_2X2` therefore remains the recommended default. Shared mode should
be enabled only when all four chips must detect both slopes.

The next optimization should not add a CSR or runtime arithmetic. The highest
value experiment is a lazy-clear cell store in `tdc_gpx_cell_builder`:

1. Stop clearing every bit of `s_cell_buf_r` on reset and shot allocation.
2. Clear only eight validity bits for the selected ping-pong buffer.
3. Initialize one addressed cell from `c_CELL_INIT` on its first hit.
4. Return `c_CELL_INIT` for an invalid cell during output.
5. Restructure the single-address write so Vivado can infer distributed RAM
   or another compact memory instead of thousands of resettable FFs.

This targets FF and control-set reduction while preserving the current two
buffer, hit-II=1, shot overlap, max_hits snapshot, and output ordering
contracts. It must be developed after this sign-off commit so the current
baseline remains immutable.

Do not merge rise/fall output chains merely to reduce area. They are separate
VDMA lanes and sharing them can reduce throughput or add head-of-line blocking.
Only reconsider that structure with an explicit bandwidth/latency contract.

Acceptance for the lazy-clear experiment is:

- C03 direct matrix 19/19 PASS
- C06 integration 13/13 PASS
- exact 32/64/128-bit output ordering and backpressure behavior
- no new CDC-4/CDC-10 or unconstrained internal endpoint
- positive post-route setup/hold at 128-bit/200 MHz in both topologies
- measured FF/control-set reduction, with no new CSR field

## 9. Parent release checklist

Before hardware release, the parent FPGA project must still:

1. Bind actual BUFG/MMCM clock sources and close clock insertion/skew.
2. Add input/output delays for every external synchronous interface.
3. Assign LOC and IOSTANDARD for all 112 TDC data pins.
4. Include the required PS7 configuration and board constraints.
5. Re-run full top-level implementation and require setup/hold/pulse PASS.
6. Confirm that SYNC mode uses a genuinely common clock source, or use ASYNC.
7. Define the system response to VDMA/Ethernet stalls longer than the bounded
   regression window; the shot-boundary FIFO corruption itself is fixed.

## 10. Final verdict

The internal RTL, clock conversion contract, CDC structure, supported
clock/width matrix, and 200 MHz routed implementation are coherent enough to
freeze as the optimization baseline. Complexity reduction may proceed from
this point with the regression and routed timing gates above. Full product
sign-off remains conditional on parent clock and board closure.
