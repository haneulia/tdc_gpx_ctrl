# C08 PS-FCLK Parent Reference Sign-off

| Item | Value |
|---|---|
| FPGA | `xc7z020clg484-2` |
| Vivado | 2025.2.1 |
| Production RTL commit | `ebd225b` |
| Functional regression | `260719033000_c07_v002_4chip_target` |
| Parent implementation | `260719042200_ps_fclk_parent_ref` |
| Optional reference | `C:/Sky/LiDAR/VIRTUAL_TDC_TEST_V003` |
| Verdict | `REFERENCE_IMPL_PASS_WITH_BOARD_IO_OPEN` |

## 1. Sign-off scope

This checkpoint signs off a reproducible, board-independent PS-FCLK parent
integration for `tdc_gpx_top`. The optional legacy project was used only to
confirm the Zynq-7000 PS FCLK and HP-port integration pattern. No legacy TDC
logic was copied.

The result is sufficient to begin controlled complexity optimization from a
known-good integration baseline. It is not a board bitstream or product
throughput sign-off because the board PS preset, I/O constraints, real event
sources, software VDMA setup, and measured DDR/Ethernet reserve are not yet
available.

## 2. Fixed parent contract

| Contract | Fixed value |
|---|---|
| PS FCLK0 | 100 MHz, AXI-Lite control |
| PS FCLK1 | 150 MHz, event processing, AXIS, VDMA S2MM and HP clocks |
| PS FCLK2 | 200 MHz, TDC bus/control |
| `g_OUTPUT_WIDTH` | 32 bits |
| `g_SLOPE_CHIP_MODE` | `DEDICATED_2X2` |
| `g_STREAM_CLK_MODE` | `ASYNC` |
| Rising storage path | 32-bit AXIS -> rise VDMA -> 64-bit HP0 |
| Falling storage path | 32-bit AXIS -> fall VDMA -> 64-bit HP1 |

The two VDMA writers remain independent. This preserves slope ownership and
prevents one lane from creating HP-port arbitration coupling in the other.
Widening the TDC AXIS output is not an optimization target: 32 bits is the
selected parent interface contract and wider packing must only be reconsidered
from measured bandwidth requirements.

## 3. Production RTL closure

Before parent implementation, timeout causes and geometry tuples were changed
to atomic CDC transfers. Commit `ebd225b` includes the reusable
`tdc_gpx_atomic_snapshot_cdc` and its asynchronous, retrigger, and reset test.
The complete C06+C07 regression then passed for the 32/64/128-bit output-stage
cases, 150/200 MHz clock profiles, bounded backpressure, dedicated lane masks,
and W64/W128 maximum-load scenarios.

The parent uses the same helper to transfer this 48-bit tuple from FCLK1 to
FCLK0 as one generation:

```text
{ VSIZE[15:0], fall HSIZE[15:0], rise HSIZE[15:0] }
```

Software must treat geometry changes as a stopped-state transaction:

1. Stop acquisition and both VDMAs.
2. Wait until TDC configuration and CDC activity are idle.
3. Read both AXI GPIO channels.
4. Program both VDMA HSIZE, VSIZE, stride, and frame-buffer addresses.
5. Start the next acquisition.

The two AXI GPIO channel reads are separate bus transactions. Runtime geometry
changes are therefore outside the supported contract even though each hardware
snapshot is internally coherent.

## 4. Implementation evidence

| Gate | Post-synthesis | Post-route |
|---|---:|---:|
| Design WNS | +0.165 ns | +0.280 ns |
| Design WHS | +0.019 ns | +0.036 ns |
| Design WPWS | +1.370 ns | +1.370 ns |
| FCLK0 WNS | +5.411 ns | +1.614 ns |
| FCLK1 WNS | +0.773 ns | +0.512 ns |
| FCLK2 WNS | +0.165 ns | +0.280 ns |
| Internal unconstrained endpoints | 0 | 0 |
| Minimum bus-skew slack | +4.387 ns | +4.137 ns |
| Manual first-stage CDC pins | 68 | 40 |
| Critical CDC findings | 0 | 0 |

Post-route contains 40,691 fully routed nets, zero unrouted or failed nets,
zero routing errors, and a non-empty `post_route.dcp`. The sign-off gate also
requires a minimum +0.100 ns setup slack, exact rule baselines, XPM-owned
inter-clock max-delay constraints, and zero `TIMING-24` findings.

## 5. Manual CDC interpretation

The parent XDC does not declare global asynchronous clock groups. XPM
instances retain their own max-delay and bus-skew constraints, while four
named manual groups false-path only their first synchronizer stage. The second
stage remains timed.

| Manual CDC group | Synthesis | Route | Interpretation |
|---|---:|---:|---|
| TDC pin status | 24 | 24 | Six 4-chip pin-status vectors |
| Diagnostic status | 40 | 12 | Only consumed diagnostics survive implementation |
| CSR idle | 2 | 2 | Cross-domain idle synchronizers |
| Command toggle | 2 | 2 | Soft-reset and force-reinit toggles |
| Total | 68 | 40 | All matched cells have `ASYNC_REG=TRUE` |

The 12 routed diagnostic pins are the four-chip masks for
`cmd_collision`, `err_bus_fatal`, and `init_cfg_coalesced`. The other 28 pins
are optimized away because these seven status fields have no downstream
consumer in the current parent:

```text
cmd_collision_vec
drain_faulted_mask
err_drain_mismatch
err_force_reinit
err_raw_ctrl_drop
err_rw_ambiguous_reg
err_stopdis_mid_shot
```

This is not a data-path integrity failure, but it is an observability and
complexity debt. The stage-specific `68 -> 40` baseline is checked exactly so
an accidental constraint or ownership change cannot silently pass.

## 6. Reviewed warnings and boundary

| Finding | Classification |
|---|---|
| `CDC-15=160` | Reviewed XPM FIFO paths; no non-XPM path allowed |
| `TIMING-9=1` | One unknown-CDC methodology item; detailed CDC has 0 critical |
| `TIMING-18=292` | Missing board I/O delays; board-open item |
| `TIMING-37=1` | Bus-skew endpoint with fanout; measured slack is positive |
| `ULMTCS-1=1` | 1,197 control sets; optimization candidate |
| `NSTD-1`, `UCIO-1` | Explicit board I/O waivers; severity is not reduced |
| `REQP-181=2` | Xilinx VDMA BRAM `WRITE_FIRST` advisory, one per lane |
| `RTSTAT-10=1` | 219 no-load nets, mainly unused IP and FIFO output bits |

`NSTD-1` and `UCIO-1` prevent this result from being called board sign-off.
The gate exposes them as explicit waivers instead of hiding or downgrading
them.

## 7. Resource baseline

| Hierarchy | LUT | FF | LUTRAM | BRAM36 | BRAM18 |
|---|---:|---:|---:|---:|---:|
| Parent total | 19,166 | 27,834 | 1,984 | 2 | 2 |
| TDC parent/core | 14,535 | 21,640 | 1,768 | 0 | 0 |
| Geometry atomic CDC | 46 | 157 | 0 | 0 | 0 |
| `config_ctrl` | 6,373 | 12,640 | 208 | 0 | 0 |
| `cell_pipe` | 4,217 | 2,154 | 1,008 | 0 | 0 |
| `output_stage` | 2,790 | 4,502 | 552 | 0 | 0 |
| Rise VDMA | 1,384 | 2,045 | 8 | 1 | 1 |
| Fall VDMA | 1,385 | 2,045 | 8 | 1 | 1 |

`config_ctrl` is the first measured RTL optimization target. The geometry CDC
cost is small and buys an explicit transaction contract, so removing it would
trade correctness for negligible resource recovery.

## 8. HDL-to-HTML status

The following simulator inputs may now be fixed to the implemented parent
profile: `g_TDC_CLK_MHZ=200`, `g_AXIS_CLK_MHZ=150`,
`g_OUTPUT_WIDTH=32`, `DEDICATED_2X2`, and two independent VDMA/HP paths.
HSIZE and VSIZE must continue to be derived from the same RTL geometry
functions and readback contract.

DDR effective bandwidth/latency and Ethernet effective payload throughput are
not implementation-timing results. Until measured with the final PS preset,
DDR software, UDP/TCP stack, and 1440-byte payload policy, the HTML may expose
them as adjustable assumptions but must not turn the corresponding reserve
checks into unconditional PASS. Routed WNS proves clock timing, not DDR or
Ethernet service time.

## 9. Complexity optimization plan

1. Resolve the seven unconsumed status masks. Remove each field end-to-end if
   it has no diagnostic value, or map it to an existing CSR status word if
   software must observe it. Do not add new variables merely to preserve dead
   structure.
2. Reduce the 1,197 control sets, starting in `config_ctrl` and
   `output_stage`. Prefer shared enables and synchronous state clearing where
   the reset contract permits it; preserve CDC synchronizer reset behavior.
3. Audit `config_ctrl` for four-chip replicated configuration images,
   per-bit reset logic, and status registers. It owns 44% of TDC-core LUTs and
   59% of TDC-core FFs, so local simplification has the highest return.
4. Re-measure synthesis, route timing, CDC, and functional regression before
   considering `cell_pipe` or `output_stage` structural changes.
5. Measure DDR and Ethernet reserve on the real parent before changing the
   two-VDMA topology or output width.

Each optimization checkpoint must keep the complete RTL regression green and
pass parent `SYNTH`; changes that affect timing, CDC, or hierarchy ownership
must also pass parent `IMPL`. Commit each independently so resource and timing
deltas remain attributable.

## 10. Final decision

Reference integration sign-off is complete and complexity optimization may
start from this baseline. Full board/product sign-off remains open until the
PS7 DDR/MIO preset, TDC LOC/IOSTANDARD/input-output delays, real laser/echo
integration, software VDMA programming, and measured DDR/Ethernet reserve are
closed.
