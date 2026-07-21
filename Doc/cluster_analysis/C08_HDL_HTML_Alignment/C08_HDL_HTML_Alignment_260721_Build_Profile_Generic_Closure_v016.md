# C08 Build Profile Generic Closure

| Item | Value |
|---|---|
| FPGA target | `xc7z020clg484-2` |
| Vivado | 2025.2.1 |
| Default cleanup checkpoint | `47d79be` |
| C06 regression | `260721151240_c06_v002_regression` |
| C08 project regression | `vivado_c08_build_profile_postinit_260721.log` |
| Verdict | `FUNCTIONAL_PROFILE_PASS / RESOURCE_DELTA_NOT_YET_CLAIMED` |

## 1. Classification decision

The design now separates a fixed external ABI from the implementation selected
before synthesis.

| Layer | Source of truth | Meaning |
|---|---|---|
| Fixed ABI/format ceiling | `tdc_gpx_pkg.vhd` | Four external chip slots, eight Stop IDs, and seven Hit slots remain available in types, ports, CSR fields, and the canonical data format. |
| Build profile | `tdc_gpx_top` generics | Selects which chip slots exist and the maximum Stop/Hit geometry software may use in this bitstream. |
| Runtime configuration | CSR | Selects an active subset and current Stop/Hit values inside the build profile. |

The top-level build profile is:

```vhdl
g_PRESENT_CHIP_MASK
g_MAX_STOPS_PER_CHIP
g_MAX_HITS_PER_STOP
```

There is deliberately no separate `g_N_CHIPS`. A count cannot describe sparse
physical slot placement and could disagree with a mask. The implemented chip
count is derived once as `popcount(g_PRESENT_CHIP_MASK)`.

## 2. Duplicate-meaning cleanup

`c_N_CHIPS`, `c_MAX_STOPS_PER_CHIP`, and `c_MAX_HITS_PER_STOP` remain package
constants because they define fixed type and format capacities. They are no
longer reported as if they were necessarily the selected implementation.

The all-chip default is now named once as `c_ALL_CHIPS_MASK`. Package defaults
and child generic defaults use that symbol instead of independently spelling
`1111` or `(others => '1')` for the same contract.

The following package values remain useful derived constants, not competing
configuration sources:

| Derived constant | Derivation |
|---|---|
| `c_MAX_ROWS_PER_FACE` | `c_N_CHIPS * c_MAX_STOPS_PER_CHIP` |
| `c_CANONICAL_CELL_BYTES_MAX` | canonical bytes at the fixed Hit ceiling |
| `c_VDMA_LINE_BYTES_MAX` | full fixed-capacity line geometry |

The overloaded `fn_effective_max_hits(cfg, build_max)` is the single rule for
the build-aware zero alias and upper clamp. Declaration initial values in
`face_seq` and `header_inserter` now also use their generics, so reset behavior
and declared defaults express the same profile.

## 3. End-to-end enforcement

| Boundary | Enforced behavior |
|---|---|
| Pipeline CSR | Reports build chip count, maximum Stops, maximum Hits, rows, Cell bytes, and maximum HSIZE. |
| Active chip request | `request AND present`; an empty intersection falls back to the lowest-index present slot. |
| Stop request | Clamped to `2..g_MAX_STOPS_PER_CHIP`. |
| Hit request | Zero aliases to `g_MAX_HITS_PER_STOP`; larger requests clamp to it. |
| Chip control | Absent slots remain reset and do not contribute busy state. |
| Cell pipe | Builders for absent chip/slope combinations are excluded by static generate conditions. |
| Face sequencer | Rejects masks outside the present set and geometry outside the build profile. |
| VDMA geometry | Uses the effective slope mask, effective Stops, and build-aware effective Hits. |
| Header word 5 | Reports effective Hits, canonical Cell bytes, Hit width, build chip count, and build maximum Stops. |

For the tested sparse profile `present=0101`, maximum Stops `4`, maximum Hits
`3`, and AXIS width `64`:

| Observable | Expected value |
|---|---:|
| `HW_CONFIG` | `0x00810342` |
| `MAX_ROWS` | 8 |
| `CELL_SIZE` | 12 B |
| `MAX_HSIZE` | 144 B |
| Shared-edge two-chip runtime HSIZE | 144 B/lane |
| Empty-intersection fallback HSIZE | 96 B/lane for chip 0 only |

## 4. Verification result

The final C06 regression completed in 264.3 seconds and archived 95 artifacts.
Its `simulate` directory contains 26 logs, zero failure/fatal/assertion-error
markers, and eight integrated top output PASS markers.

The two new non-default build tests both passed:

| Scenario | Effective result | AXIS result |
|---|---|---|
| SW requests `1111`, 8 Stops, 7 Hits | present `0101`, 4 Stops, 3 Hits | Rise/Fall 18 beats, one TLAST each |
| SW requests absent `1010`, 8 Stops, zero-alias Hits | fallback chip 0, 4 Stops, 3 Hits | Rise/Fall 12 beats, one TLAST each |

Both tests also check exact compile-time CSR geometry, first-line header fields,
inactive-chip bus inactivity, output beat counts, TLAST counts, and VDMA HSIZE.

The C08 Vivado project regression rebuilt from a clean simulation state and
passed all four profiles:

- Dedicated 2x2, AXIS 32 bit
- Dedicated 2x2, AXIS 64 bit
- Dedicated 2x2, AXIS 128 bit
- Shared dual edge, AXIS 64 bit

The C08 log contains four runs, four output PASS markers, and zero runtime
failure markers. Board-part availability warnings are Vivado installation
warnings and are unrelated to the simulated design.

## 5. Review findings closed during verification

The stronger header checks exposed two testbench interpretation errors, not
RTL data corruption:

1. Header word 3 carries the active mask for that slope lane. In dedicated
   mode it is `0011` for Rise and `1100` for Fall, not the global union mask.
2. Actual Face metadata is present only in the first VDMA line. Later lines
   retain the reserved 48-byte prefix but contain zeros there.

The C08 project script also now calls `reset_simulation` after compile-order
updates. This prevents stale dependent VDB data from masking package or entity
interface changes.

## 6. HDL-to-HTML boundary

The current C08 HTML correctly models the default fixed capacities and runtime
CSR values, but it does not yet represent the three build-profile generics as a
separate layer. Therefore it remains correct for the default `1111/8/7` build,
but it cannot yet prove a non-default bitstream such as `0101/4/3`.

The next HTML revision should add a Build Profile section and retain the
existing runtime controls below it. Its calculations must use:

```text
effective_chip_mask = requested_mask AND present_mask
effective_stops     = clamp(requested_stops, 2, build_max_stops)
effective_max_hits  = build_max_hits when requested_hits = 0
                      else min(requested_hits, build_max_hits)
```

It must also display the resulting `HW_CONFIG`, `MAX_ROWS`, `CELL_SIZE`, and
`MAX_HSIZE` values so the HTML can be compared directly with CSR reads.

## 7. Sign-off and optimization boundary

The build-profile behavior is functionally signed off at RTL and C08 project
simulation level. This does not yet prove a resource reduction for lower Stop
or Hit maxima. Fixed ABI records and some internal arrays remain sized to the
package ceilings, so synthesis may retain part of that storage.

The next rational optimization step is an A/B synthesis on the same
`xc7z020clg484-2` constraints:

| Build A | Build B |
|---|---|
| `1111 / 8 Stops / 7 Hits` | `0101 / 4 Stops / 3 Hits` |

Compare LUT, FF, BRAM, control sets, and WNS. Only if Stop/Hit reductions fail
to produce a useful netlist delta should the fixed internal arrays be replaced
with generic-sized local storage. The external four-slot ABI and canonical
packet format should remain stable during that optimization.
