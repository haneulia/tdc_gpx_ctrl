# C07 CSR Geometry / Status / Shot-Boundary Hardening v002

| Item | Result |
|---|---|
| Scope | Pipeline CSR geometry, STAT6/7 map, masked-slope lifecycle, FIFO reset guard |
| RTL | `tdc_gpx_pkg`, `tdc_gpx_cfg_pkg`, `tdc_gpx_csr_pipeline`, `tdc_gpx_cell_pipe`, `tdc_gpx_face_assembler`, `tdc_gpx_output_stage`, `tdc_gpx_top` |
| Verification | Vivado 2025.2.1 xsim |
| HTML handoff | C08-S14 `CSR_Geometry_Status_Contract_Simulator_v014.html` |

## 1. Review conclusion

The overall P0/P1/P2 direction was correct, but three contracts needed to be
made explicit before the result could be treated as closed:

1. Pipeline CSR geometry must describe the packed VDMA serialization, not the
   legacy power-of-two internal cell allocation.
2. A diagnostic sticky must remain readable after cmd_stop/abort and have an
   explicit software epoch-clear operation.
3. A shot-boundary FIFO reset may occur only when interface outstanding data is
   exactly zero. The tracker must model the full AXIS elastic capacity, not RAM
   entries alone.

## 2. CSR geometry decision

`c_CELL_SIZE_BYTES=32` remains an internal legacy allocation constant. The
published CSR reports the canonical packed maximum:

```text
STAT3 CELL_SIZE = (ceil(7 hits / 2 per 32-bit word) + 1 metadata word) * 4
                = 20 bytes/cell

STAT4 MAX_HSIZE = 48-byte prefix + align16(32 full-mask cells * 20 bytes)
                = 688 bytes/line
```

These values are independent of `g_OUTPUT_WIDTH`. Increasing 32 to 64 or 128
bits reduces AXIS beat count; it does not add per-cell payload or DDR time.
Runtime topology still controls the actual line contract through
`o_vdma_hsize_bytes_rise/fall`. A 16-cell dedicated lane at seven hits is
`48 + align16(16 * 20) = 368 bytes`.

## 3. CSR address and STAT map hardening

The generated CSR IP places its native STAT registers at `0x20..0x3C`, while
the published wrapper contract uses `0x40..0x5C`. Translation is now scoped:

| External access | Internal action |
|---|---|
| Read `0x40..0x5C` | Translate to native STAT0..7 |
| Read `0x20..0x3C` | Hidden; return zero |
| Read `0x60..0x7F` | Reserved; return zero |
| Write `0x00..0x1C` | Forward to CTL0..7 |
| Other write | Unmapped; no CSR/IRQ side effect |

`tdc_gpx_cfg_pkg` now names every implemented STAT6/7 field. The packing RTL
uses those names instead of duplicated numeric slices. Simulation assertions
reject X/U in STAT5/6/7 before CDC, closing the prior exact-zero false-PASS
path.

## 4. Masked-slope behavior and lifecycle

For a disabled `(chip, slope)` lane:

- `shot_start` and `drain_done` do not wake its cell builder.
- A hit is consumed without backpressuring on stale disabled-lane state.
- The hit sets the per-chip/slope diagnostic sticky.
- The OR-reduced indication is published at `STAT7[15]`.
- cmd_stop/global abort clears in-flight demux state but preserves evidence.
- `CTL2[1] err_soft_clear` or hard reset starts a clean diagnostic epoch.
- Clear wins if a clear and a masked hit occur in the same cycle.

This keeps the runtime datapath small: the masks already required by topology
also drive ready/load gating; no additional per-run counters or arithmetic are
introduced.

## 5. FIFO reset guard invariant

Both `face_assembler.u_fifo_out` and the two output-stage face FIFOs use the
same rule:

```text
outstanding = accepted input handshakes - emitted output handshakes
shot reset allowed iff outstanding == 0 and source_tvalid == 0
abort reset allowed unconditionally
```

The XPM FIFO has `FIFO_DEPTH=16`, but `xpm_fifo_axis` instantiates its base FIFO
in FWFT `READ_MODE=1`. Two output elastic stages can hold accepted beats in
addition to the 16 RAM entries, so the observable maximum is 18 outstanding
beats. A stress probe reached 18 and then backpressured normally while all 88
beats and both TLASTs were preserved. The permanent assertion therefore
rejects a 19th write-only handshake, not the valid 17th/18th handshakes.

This distinction is important: the tracker is a reset-safety proof counter,
not a claim about RAM occupancy.

## 6. Verification matrix

| Check | Expected |
|---|---|
| CSR reads | `0x40=0x00010000`, `0x48=32`, `0x4C=20`, `0x50=688` |
| Reserved reads | `0x20=0`, `0x60=0` |
| STAT source knownness | No X/U assertion |
| Masked hit | `STAT7=0x00008000` |
| Abort after masked hit | Sticky retained |
| `CTL2[1]` pulse | `STAT7=0` |
| Dedicated disabled lane | No slices, no drain-induced sticky |
| Shot-boundary stall | Beat/TLAST/SOF/frame_done preserved |
| Width sweep | 32/64/128-bit packed geometry and output preserved |
| Clock split | AXIS 150 MHz / TDC 200 MHz preserved |

Final archived sessions are listed in section 8.

## 7. Remaining engineering policy

No functional P0/P1 item remains in this scope. The following are operational
or future observability choices:

1. Keep `max_scan_clks` nonzero in production even though the 16-bit hard cap
   prevents an infinite wait.
2. Decide a system-level policy for a VDMA stall longer than the current
   watchdog window: drop/abort the whole face, retry, or escalate to software.
3. Add per-chip masked-slope detail only if field diagnostics require more than
   the current `STAT7[15]` OR bit; do not grow the runtime datapath preemptively.
4. Re-run synthesis/timing/resource reports after the clock/topology parameters
   are frozen. The current closure is behavioral xsim verification.

## 8. Evidence

- `260717_contract_fix_precheck`: CSR exact reads, lane-mask lifecycle,
  32/64/128, split-clock and bounded-backpressure suite passed.
- `260717_contract_hardening_p0_shot_stall_top_int`: integrated 64/32-bit
  shot-boundary stall suite passed.
- `260717_fifo_depth_probe_shot_bp_regression`: confirmed XPM maximum of 18
  interface-outstanding beats and passed all output-stage scenarios.
- `260717_contract_hardening_output_final_shot_bp_regression`: permanent
  18-beat assertion, no-backpressure/stall, 32/64/128-bit and max_hits 0..7
  output-stage release suite passed (35 archived artifacts).
- `260717_contract_hardening_release_p0_shot_stall_top_int`: final integrated
  release suite passed (50 archived artifacts), including exact CSR geometry,
  reserved aliases, lane-mask lifecycle, AXIS 150/TDC 200, bounded
  backpressure, and 64/32-bit shot-boundary stalls.
