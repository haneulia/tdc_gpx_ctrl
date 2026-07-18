# C09 Banked Payload RAM Optimization Closure v001

## 1. Decision

**Seven-bank payload RAM optimization: ACCEPTED for the internal RTL baseline.**

**128-bit/200 MHz internal OOC implementation: PASS in both slope topologies.**

**Parent FPGA, bitstream, and board sign-off: OPEN.**

The accepted implementation replaces the per-cell payload FF geometry with
seven addressed resetless distributed-RAM banks per active cell builder. It
preserves the CSR map, AXI stream format, hit II=1, ping-pong overlap, and all
runtime geometry calculations.

The first banked prototype passed functional regression and reduced resources,
but failed the dedicated 200 MHz route because `max_hits` remained in the RAM
write-enable cone. A follow-up change separates the physical payload write
limit from the visible configured hit limit. Dedicated and shared routes now
have positive setup and hold slack.

The shared route closes with only +0.017 ns setup slack. This is an internal
hard-gate pass, not parent-level guardband. Every later cell-pipe or config-path
change must rerun the shared and dedicated 128-bit/200 MHz sentinel corners.

## 2. Evidence identity

| Item | Value |
|---|---|
| Accepted compact-store baseline | `064dc78` |
| Control-set attribution checkpoint | `24c03e4` |
| Initial banked payload RTL | `376f8e4` |
| Timing-closure RTL | `e6ffa04` |
| Overflow-mask test checkpoint | `8823bfa` |
| Implementation Git state | clean at `e6ffa04` |
| Vivado | 2025.2.1, build 6403652 |
| Device | `xc7z020clg484-2` |
| Final C03 session | `260718_overflow_shadow_c03_c07_v001_c03_direct_matrix` |
| C06 session | `260718_shadow_write_c06_c06_v002_regression` |
| Dedicated implementation | `260718120900_shadow_write_dedicated_w128_a200_t200_dedicated_2x2_impl` |
| Shared implementation | `260718122100_shadow_write_shared_w128_a200_t200_shared_dual_edge_impl` |

Both implementation sessions use:

- `g_OUTPUT_WIDTH = 128`
- `g_AXIS_CLK_MHZ = 200`
- `g_TDC_CLK_MHZ = 200`
- `ASYNC` clock mode
- `TIMING_EXPLORE` implementation strategy

This 200/200 MHz corner is the tight endpoint for the supported
50/100/125/150/200 MHz clock set. Lower frequencies do not impose a shorter
period on the same logic. The C06 suite also retains the mixed AXIS 150 MHz /
TDC 200 MHz functional case.

## 3. Accepted storage contract

Each active cell builder has two ping-pong buffers and eight stop addresses.
The persistent store is split into:

1. 16 narrow metadata entries containing `hit_count_actual` and
   `hit_dropped`.
2. Seven independent payload banks, one bank per hit sequence.

Each payload bank is:

- depth 16: `buffer & stop`
- width 17: the complete raw TDC hit
- one addressed synchronous write
- one asynchronous read feeding the existing output staging register
- resetless and explicitly marked `ram_style = "distributed"`

Vivado infers each bank as `16 x 17`, `RAM32M x 9`. Therefore:

```text
7 banks/builder * 16 addresses * 17 bits = 1,904 payload bits/builder
7 banks/builder * 36 LUTRAM LUTs/bank = 252 LUTRAM LUTs/builder
```

The serializer visibility invariant is unchanged:

```text
serialized slot[i] is visible iff i < hit_count_actual
```

Only count/drop metadata is cleared when a ping-pong buffer is reused.
Physically stale payload remains harmless because both hit data and Hit[16]
metadata are masked by the current count.

No new CSR, persistent runtime field, multiplier/divider, output beat, or
protocol state was added.

## 4. Initial timing failure and correction

The initial banked prototype made payload RAM write enable depend on:

```text
input handshake
AND valid stop
AND current_count < configured max_hits
AND sequence-bank decode
```

The dedicated failed implementation session was:

`260718113600_payload_ram_dedicated_w128_a200_t200_dedicated_2x2_impl`

| Result | Value |
|---|---:|
| Post-route WNS | -0.169 ns |
| Post-route TNS | -39.375 ns |
| Post-route WHS | +0.061 ns |
| Failed/unrouted nets | 0 / 0 |
| Worst source | `s_buf_max_hits_r[1][1]` |
| Worst destination | payload RAM `WE` |
| Data path / logic levels | 4.660 ns / 5 |

This proved that output read staging was not the actual hard-gate problem. The
fix keeps configured `max_hits` only in visible count/drop metadata and limits
RAM writes by the physical seven-slot capacity.

For `max_hits < 7`, the first overflow hit writes bank `max_hits`, which is
outside the active mask. Later overflow hits may overwrite that same inactive
shadow slot. For `max_hits = 7`, no physical slot remains and the overflow hit
does not write RAM. In both cases count remains capped and `hit_dropped` is set.

```text
visible count update: current_count < configured max_hits
payload write:        current_count < physical limit 7
```

This removes `max_hits` from RAM `WE` without adding a pipeline stage, variable,
or externally visible latency. Extra switching occurs only on an already
overflowed shot and is bounded to one inactive 17-bit bank word per excess hit.

## 5. Functional verification

| Regression | Coverage | Result |
|---|---|---:|
| C03 direct matrix | 32/64/128 bit, max_hits 1/3/5/7, timeout, dual buffer, drop/quarantine, stops snapshot, stale-buffer reuse | 20/20 PASS |
| C03 overflow extension | max_hits 3 and 7, eight injected hits, exact payload/mask/drop checks | 2/2 PASS |
| **C03 total** | all archived direct cases | **22/22 PASS** |
| C06 integration | chip control, face sequence, slope modes, range ticks, CDC/status, widths, mixed clocks, bounded backpressure | **13/13 PASS** |

Across the final C03 archive:

- failure logs: 0
- compile-time slope-role mismatch logs: 0
- overflow pulse count equals `8 - max_hits`
- only the first `max_hits` payload values are serialized
- all inactive payload slots and Hit[16] bits remain zero
- metadata count remains capped and `hit_dropped = 1`

The C06 integration run used the same `e6ffa04` RTL. Commit `8823bfa` changes
only the C03 TB and its regression script, so no C06 or implementation netlist
changed after the recorded runs.

## 6. Post-synthesis result

### Dedicated topology

| Stage | Top LUT | Top FF | Cell LUT | Cell FF | Control sets |
|---|---:|---:|---:|---:|---:|
| Compact store | 23,821 | 39,303 | 6,336 | 9,983 | 1,382 |
| Initial banked RAM | 21,698 | 31,734 | 4,139 | 2,414 | 961 |
| Final shadow-write split | 21,437 | 31,737 | 3,952 | 2,417 | 961 |

### Shared topology

| Stage | Top LUT | Top FF | Cell LUT | Cell FF | Control sets |
|---|---:|---:|---:|---:|---:|
| Compact store | 30,042 | 49,076 | 12,483 | 19,756 | 1,950 |
| Initial banked RAM | 25,554 | 33,940 | 7,995 | 4,620 | 1,109 |
| Final shadow-write split | 25,310 | 33,949 | 7,751 | 4,629 | 1,109 |

The timing correction does not add structural cost. Relative to the initial
banked prototype it removes another 261 dedicated top LUTs and 244 shared top
LUTs, while adding only 3 and 9 FFs respectively. Control-set counts remain at
the improved banked-RAM values.

Relative to the compact store, final control-set reduction is:

| Topology | Compact | Final | Delta |
|---|---:|---:|---:|
| Dedicated | 1,382 | 961 | -421 (-30.46%) |
| Shared | 1,950 | 1,109 | -841 (-43.13%) |

Payload inference is exact in every active builder:

- dedicated: 4 builders, 28 banks, 1,008 cell-pipe LUTRAM LUTs
- shared: 8 builders, 56 banks, 2,016 cell-pipe LUTRAM LUTs
- each builder: 7 banks of `16 x 17`, each reported as `RAM32M x 9`

## 7. Post-route resource result

The comparison baseline is the accepted compact-store route.

| Topology and scope | Compact LUT | Final LUT | Delta | Compact FF | Final FF | Delta |
|---|---:|---:|---:|---:|---:|---:|
| Dedicated top | 22,432 | 20,001 | -2,431 (-10.84%) | 36,496 | 28,927 | -7,569 (-20.74%) |
| Dedicated cell pipe | 6,313 | 3,925 | -2,388 (-37.83%) | 9,978 | 2,409 | -7,569 (-75.86%) |
| Shared top | 28,679 | 23,887 | -4,792 (-16.71%) | 46,283 | 31,147 | -15,136 (-32.70%) |
| Shared cell pipe | 12,476 | 7,744 | -4,732 (-37.93%) | 19,749 | 4,613 | -15,136 (-76.64%) |

The physical implementation retains the intended storage reduction. The FF
delta scales exactly with the number of active builders, while LUTRAM replaces
the former payload FF arrays.

## 8. Timing closure

| Topology | Version | WNS | TNS | WHS | THS | Result |
|---|---|---:|---:|---:|---:|---|
| Dedicated | Compact store | +0.037 | 0.000 | +0.071 | 0.000 | PASS |
| Dedicated | Initial banked RAM | -0.169 | -39.375 | +0.061 | 0.000 | FAIL |
| Dedicated | Final shadow-write split | +0.031 | 0.000 | +0.071 | 0.000 | PASS |
| Shared | Compact store | +0.095 | 0.000 | +0.071 | 0.000 | PASS |
| Shared | Initial banked RAM | not routed after dedicated rejection | - | - | - | not run |
| Shared | Final shadow-write split | +0.017 | 0.000 | +0.071 | 0.000 | PASS |

Post-synthesis advisory timing remains unchanged from the compact baseline:

- dedicated: WNS -0.290 ns, WHS +0.157 ns
- shared: WNS -0.367 ns, WHS +0.157 ns

Final worst AXIS paths are still payload RAM write-enable paths, but configured
`max_hits` is no longer a source:

| Topology | Source class | Destination | Levels | Data path | Slack |
|---|---|---|---:|---:|---:|
| Dedicated | registered input `tuser` decode | payload bank `WE` | 5 | 4.459 ns | +0.031 ns |
| Shared | per-cell visible count | payload bank `WE` | 5 | 4.473 ns | +0.017 ns |

The shared result is the new mandatory timing sentinel. The next optimization
must not be accepted from synthesis alone even when resource deltas are good.

## 9. CDC, constraints, routing, and DRC

| Check | Dedicated | Shared |
|---|---:|---:|
| CDC-4 | 0 | 0 |
| CDC-10 | 0 | 0 |
| `no_clock` | 0 | 0 |
| `unconstrained_internal_endpoints` | 0 | 0 |
| failed/unrouted/partial nets | 0 / 0 / 0 | 0 / 0 / 0 |
| setup/hold failing endpoints | 0 / 0 | 0 / 0 |

The remaining DRC items are outside this payload optimization:

- `NSTD-1` and `UCIO-1`: expected OOC I/O standard and pin ownership
- `HD.CLK_SRC`: parent design must provide final clock-buffer locations
- `ZPS7-1`: the OOC block intentionally excludes the parent PS7
- DSP input/output pipeline warnings belong to the existing geometry alignment
  functions and are unchanged

No severity was demoted. These OOC passes do not authorize bitstream release.

## 10. Acceptance matrix

| Requirement | Result |
|---|---|
| No CSR or output-format change | PASS |
| No new persistent runtime variable or arithmetic block | PASS |
| Hit II=1 and ping-pong overlap preserved | PASS |
| 32/64/128 bit and max_hits behavior preserved | PASS |
| Stale and overflow payload cannot escape the active mask | PASS |
| C03 22/22 and C06 13/13 | PASS |
| Distributed RAM inferred in every active builder | PASS |
| Material LUT, FF, and control-set reduction | PASS |
| CDC and internal constraint checks clean | PASS |
| Dedicated and shared 200 MHz setup/hold positive | PASS |
| Parent implementation and board constraints | OPEN |

## 11. Next optimization direction

The payload store is no longer the dominant dedicated control-set owner. The
next step should be a fresh attribution of the remaining 961/1,109 control sets
before another RTL change. The current measured fixed owners include about 487
sets in config control and 191 in output stage; these now outweigh the 159
dedicated cell-pipe sets.

Recommended order:

1. Decompose config-control control sets by CSR CDC, command arbitration, and
   per-chip register paths.
2. Select one local owner with no protocol or clock-domain change.
3. Run C03 22/22 and C06 13/13 as applicable.
4. Re-run both 128-bit/200 MHz route sentinels; shared +0.017 ns is the tighter
   acceptance point.

Do not start a count epoch/tag redesign merely to remove the remaining 16
per-builder count controls. It would add state and verification burden at the
current worst-path source for a much smaller resource return than the payload
banking change.

## 12. Final status

The banked payload RAM plus shadow-write split is accepted as the new internal
RTL baseline. It closes the targeted complexity reduction without changing the
software or streaming contracts, and it restores both 200 MHz routed corners
to positive setup and hold slack. Parent-level timing, pins, I/O standards,
clock sources, PS7 integration, and bitstream checks remain explicitly open.
