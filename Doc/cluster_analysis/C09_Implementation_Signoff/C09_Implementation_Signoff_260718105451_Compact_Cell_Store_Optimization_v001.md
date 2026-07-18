# C09 Compact Cell Store Optimization Closure v001

## 1. Decision

**Compact cell store optimization: ACCEPTED for the internal RTL baseline.**

**128-bit/200 MHz internal OOC implementation: PASS in both slope topologies.**

**Parent FPGA, bitstream, and board sign-off: OPEN.**

The accepted implementation reduces both LUT and FF use without adding a CSR,
runtime arithmetic, output beat, or protocol state. Functional regressions are
clean and both routed 200 MHz corners have positive setup and hold slack.

The dedicated route closes with only +0.037 ns setup slack. This satisfies the
existing positive-slack internal acceptance gate, but it is not usable as
parent-level guardband. Every subsequent cell-pipe change must re-run this
corner, and the parent design must still close timing with final clock sources,
I/O delays, pins, and I/O standards.

## 2. Evidence identity

| Item | Value |
|---|---|
| Frozen comparison baseline | `a0f9f06b214b85a0bbbdde5f2aa45452ab42416e` |
| Rejected experiment checkpoint | `14d9658` |
| Accepted RTL commit | `064dc78695baff7315e109df836547659a8aae37` |
| Implementation session Git state | clean at `064dc78` |
| Vivado | 2025.2.1, build 6403652 |
| Device | `xc7z020clg484-2` |
| Dedicated implementation session | `260718101200_compact_store_dedicated_w128_a200_t200_dedicated_2x2_impl` |
| Shared implementation session | `260718103100_compact_store_shared_w128_a200_t200_shared_dual_edge_impl` |
| C03 functional session | `260718_compact_store_c03_c07_v001_c03_direct_matrix` |
| C06 functional session | `260718_compact_store_c06_c06_v002_regression` |

Both implementation sessions use:

- `g_OUTPUT_WIDTH = 128`
- `g_AXIS_CLK_MHZ = 200`
- `g_TDC_CLK_MHZ = 200`
- `ASYNC` clock mode
- `TIMING_EXPLORE` implementation strategy

## 3. Accepted storage contract

The external cell-slice format and ping-pong behavior are unchanged. The
optimization changes only what is retained physically inside each cell builder.

The internal `t_cell_store` now stores only:

1. `hit_slot`
2. `hit_msb_vec`
3. `hit_count_actual`
4. `hit_dropped`

The following output fields are derived during serialization:

- `hit_valid` is the contiguous active mask below `hit_count_actual`.
- `slope_vec` is that active mask for a rising builder and zero for a falling
  builder.
- `error_fill` remains zero in the normal cell builder. Synthetic blank/error
  fill remains owned by the face assembler.

Only `hit_count_actual` and `hit_dropped` are cleared when a ping-pong buffer is
allocated to a new shot. Payload slots and MSB bits may retain stale physical
values, but every hit-data and metadata output is masked at or above the current
count. An accepted hit overwrites its addressed sequence slot before that slot
can become valid.

This gives the following invariant:

```text
serialized slot[i] is visible iff i < hit_count_actual
```

The builder role is also compile-time static:

- rise instance: `g_SLOPE_VALUE = '1'`
- fall instance: `g_SLOPE_VALUE = '0'`

This removes redundant per-hit slope storage. A simulation-only assertion
reports any event routed to a builder whose compile-time slope role does not
match the input sideband.

No clock-domain contract, CSR map, `max_range_5ns_ticks` conversion, VDMA
geometry, output ordering, or backpressure behavior changed.

## 4. Rejected valid-mask experiment

Commit `14d9658` first tried a per-cell valid mask plus a whole-record first-hit
merge. It passed functional regression but failed the physical optimization
objective.

Dedicated 128-bit/200 MHz post-synthesis comparison:

| Scope | Baseline LUT | Experiment LUT | Delta | Baseline FF | Experiment FF | Delta |
|---|---:|---:|---:|---:|---:|---:|
| Top | 24,106 | 27,475 | +3,369 (+14.0%) | 40,260 | 40,270 | +10 |
| Cell pipe | 6,547 | 9,916 | +3,369 (+51.5%) | 10,940 | 10,950 | +10 |

The whole-record conditional initialization created a large mux network on the
cell write path and did not remove meaningful FF state. The route was stopped
after the directly comparable synthesis result proved the approach unsuitable.
The stale-buffer regression added with this experiment was retained because it
is required to prove the accepted count-derived masking contract.

Rejected probe:

`260718092100_lazy_clear_dedicated_w128_a200_t200_dedicated_2x2_impl`

## 5. Functional verification

| Regression | Coverage | Result |
|---|---|---:|
| C03 direct matrix | 32/64/128 bit, max_hits 1/3/5/7, dual buffer, timeout, drop/quarantine, stops snapshot, reused-buffer stale guard | 20/20 PASS |
| C06 integration | chip control, face sequence, both slope modes, range ticks, register response CDC, status, 32/64/128 bit top integration, AXIS 150/TDC 200, bounded backpressure | 13/13 PASS |

Across the archived simulation logs:

- unexpected `FAIL` markers: 0
- compile-time slope-role mismatch reports: 0
- the new reused-buffer scenario seeds stale payload, reuses the buffer with a
  sparse next shot, and confirms that no stale slot or metadata bit escapes

## 6. Post-synthesis resource result

### DEDICATED_2X2

| Scope | Baseline LUT | New LUT | Delta | Baseline FF | New FF | Delta |
|---|---:|---:|---:|---:|---:|---:|
| Top | 24,106 | 23,821 | -285 (-1.18%) | 40,260 | 39,303 | -957 (-2.38%) |
| Cell pipe | 6,547 | 6,336 | -211 (-3.22%) | 10,940 | 9,983 | -957 (-8.75%) |

### SHARED_DUAL_EDGE

| Scope | Baseline LUT | New LUT | Delta | Baseline FF | New FF | Delta |
|---|---:|---:|---:|---:|---:|---:|
| Top | 30,530 | 30,042 | -488 (-1.60%) | 51,002 | 49,076 | -1,926 (-3.78%) |
| Cell pipe | 12,971 | 12,483 | -488 (-3.76%) | 21,682 | 19,756 | -1,926 (-8.88%) |

Control-set counts remain unchanged:

| Topology | Baseline | New | Delta |
|---|---:|---:|---:|
| DEDICATED_2X2 | 1,382 | 1,382 | 0 |
| SHARED_DUAL_EDGE | 1,950 | 1,950 | 0 |

The optimization therefore closes its metadata-storage objective, especially
the cell-pipe FF reduction, but it does not close the separate high-control-set
complexity item.

## 7. Post-route resource result

| Topology and scope | Baseline LUT | New LUT | Delta | Baseline FF | New FF | Delta |
|---|---:|---:|---:|---:|---:|---:|
| Dedicated top | 22,637 | 22,432 | -205 (-0.91%) | 37,465 | 36,496 | -969 (-2.59%) |
| Dedicated cell pipe | 6,410 | 6,313 | -97 (-1.51%) | 10,946 | 9,978 | -968 (-8.84%) |
| Shared top | 29,033 | 28,679 | -354 (-1.22%) | 48,228 | 46,283 | -1,945 (-4.03%) |
| Shared cell pipe | 12,781 | 12,476 | -305 (-2.39%) | 21,690 | 19,749 | -1,941 (-8.95%) |

Physical optimization replication does not erase the storage benefit. Both
topologies retain about 8.9% fewer cell-pipe FFs after route.

## 8. Timing closure

| Topology | Stage | Baseline WNS | New WNS | New TNS | Baseline WHS | New WHS | Result |
|---|---|---:|---:|---:|---:|---:|---|
| Dedicated | post-synth advisory | -0.290 | -0.290 | -1.159 | +0.157 | +0.157 | unchanged advisory estimate |
| Dedicated | post-route hard gate | +0.093 | +0.037 | 0.000 | +0.070 | +0.071 | PASS |
| Shared | post-synth advisory | -0.367 | -0.367 | -4.092 | +0.157 | +0.157 | unchanged advisory estimate |
| Shared | post-route hard gate | +0.083 | +0.095 | 0.000 | +0.071 | +0.071 | PASS |

All setup and hold failing-endpoint counts are zero after route.

The worst paths remain the same architectural class as the baseline: AXIS-domain
cell-write control to `hit_slot` clock-enable pins.

- Dedicated: 4 logic levels, 4.721 ns data path, 79.2% route delay. The baseline
  was also 4 levels and 79.0% route delay. The 56 ps WNS reduction is a physical
  placement/routing result, not an added logic-depth regression.
- Shared: 3 logic levels, 4.663 ns data path, 82.8% route delay. The baseline
  was also 3 levels; setup slack improves by 12 ps.

The dedicated +0.037 ns result is accepted only as an internal OOC pass. It is
the mandatory sentinel corner for the next optimization.

## 9. CDC, constraints, routing, and DRC

Both accepted implementation sessions report:

| Check | Dedicated | Shared |
|---|---:|---:|
| CDC-4 | 0 | 0 |
| CDC-10 | 0 | 0 |
| `no_clock` | 0 | 0 |
| `unconstrained_internal_endpoints` | 0 | 0 |
| nets with routing errors | 0 | 0 |

The post-route DRC reports only the expected OOC boundary critical warnings:

- `NSTD-1`: parent project owns I/O standards
- `UCIO-1`: parent project owns pin locations

The OOC `HD.CLK_SRC` warnings also remain a parent-clock concern. These warnings
are not severity-demoted and this result does not authorize bitstream release.

## 10. Acceptance matrix

| Requirement | Result |
|---|---|
| No new CSR or runtime multiplier/divider | PASS |
| Existing 32/64/128 output contract | PASS |
| Two-buffer overlap and backpressure behavior | PASS |
| Stale payload cannot escape a reused buffer | PASS |
| Dedicated and shared functional regressions | PASS |
| Cell-pipe LUT and FF reduction | PASS |
| No new critical CDC class or unconstrained internal endpoint | PASS |
| Positive 128-bit/200 MHz setup and hold in both topologies | PASS |
| Control-set reduction | NOT ACHIEVED; separate next item |

## 11. Next optimization direction

The next pass should target control-set and cell-write fanout, not add another
validity field or repeat the whole-record merge.

Recommended sequence:

1. Attribute the 1,382/1,950 control sets by hierarchy, clock, reset, and enable
   source before changing RTL.
2. Prototype an explicitly addressed, resetless payload store with narrow
   count/drop state. Preserve one addressed write and avoid a whole-record mux.
3. Measure whether distributed-RAM inference and cell-write enable fanout improve
   together. Reject the prototype at post-synthesis if LUT or logic depth grows.
4. Keep `g_SLOPE_VALUE` compile-time static and keep the two VDMA slope output
   chains separate.
5. Re-run C03 20/20, C06 13/13, then the dedicated + shared 128-bit/200 MHz
   routed corners. Dedicated must retain positive setup and hold slack.

The compact store is now the frozen comparison point for that work. The first
valid-mask implementation is recorded as a rejected experiment and must not be
restored without a different synthesis architecture.

## 12. Final verdict

The accepted compact cell store makes the HDL more direct: only irreducible hit
payload and count/drop state are stored, while valid and slope metadata are
derived from fixed contracts. The implementation reduces cell-pipe FF use by
about 8.9% and LUT use in both topologies, preserves functional behavior, and
passes both 200 MHz OOC routes.

Internal optimization sign-off is complete at commit `064dc78`. Full product
sign-off remains conditional on parent implementation and board constraints.
