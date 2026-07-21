# TDC GPX PS-FCLK Parent Reference

This directory creates a reproducible parent integration project for
`tdc_gpx_top`. It is a new reference design; it does not import the legacy TDC
logic from `VIRTUAL_TDC_TEST_V003`.

## Fixed integration profile

| Contract | Value |
|---|---|
| FPGA | `xc7z020clg484-2` |
| PS FCLK0 | 100 MHz, AXI-Lite control |
| PS FCLK1 | 150 MHz, event processing, AXIS, VDMA S2MM and HP clocks |
| PS FCLK2 | 200 MHz, TDC bus/control clock |
| TDC output width | 32 bits |
| Present chips | `1111` (chip 0 through chip 3) |
| Rising-capable mask | `0011` (chip 0 and chip 1 while falling is enabled) |
| Falling-capable mask | `1100` (chip 2 and chip 3) |
| Build limits | 8 stops/chip, 7 hits/stop |
| DDR writers | rise VDMA to HP0, fall VDMA to HP1 |

The parent uses two independent VDMA S2MM channels. Each accepts one 32-bit
TDC output stream and writes through a 64-bit PS HP port. Runtime HSIZE and
VSIZE are exposed to software through a read-only AXI GPIO snapshot.

The wrapper exposes every public `tdc_gpx_top` build generic, including HW
version, clock/time policy, OEN mode, and stop/fire stream widths, and passes
them through without a second interpretation. The reference project script
locks those generics to the profile above; another parent may override them at
elaboration while retaining the same single-source hierarchy.

Before Vivado starts, `verify_parent_generic_parity.ps1` compares the generic
declarations of `tdc_gpx_top` and `tdc_gpx_parent_core` with the wrapper's
generic map. A missing, renamed, extra, or indirectly mapped public generic
fails the run. The generated block-design contract then exact-checks all 25
configured values, so declaration parity and the selected integration profile
are independently verified.

Slope ownership is an explicit compile-time contract. `g_RISE_CHIP_MASK` and
`g_FALL_CHIP_MASK` may overlap when one chip must collect both edges. With the
default masks, CSR `CTL21[19]=1` selects the 2-rising/2-falling assignment;
`CTL21[19]=0` makes all four present chips rising and idles the falling lane.
For a permanently rising-only product, set `g_RISE_CHIP_MASK=1111` and
`g_FALL_CHIP_MASK=0000`; the falling builders and output chain are then removed
by synthesis, while the fixed fall AXIS/geometry ports remain tied idle/zero.

## Address map

| Base | Function |
|---|---|
| `0x4000_0000` | TDC chip/control CSR |
| `0x4000_0800` | TDC pipeline CSR |
| `0x4120_0000` | packed rise/fall HSIZE and VSIZE snapshot |
| `0x4300_0000` | falling-lane VDMA registers |
| `0x4301_0000` | rising-lane VDMA registers |

## Usage

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File parent_ref/scripts/run_parent_ref.ps1 -Mode VALIDATE

powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File parent_ref/scripts/run_parent_ref.ps1 -Mode SYNTH

powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File parent_ref/scripts/run_parent_ref.ps1 -Mode IMPL
```

Every run recreates the project in `.tmp_parent/<stamp>`. The runner parses the
generated block-design JSON and exact-checks the part, actual FCLK values,
interface widths/domains, lane-to-VDMA-to-HP paths, and address map. `SYNTH` and
`IMPL` also run `verify_parent_signoff.ps1`; a run fails if timing, CDC,
bus-skew, methodology, DRC, or routing departs from the reviewed baseline.

The sign-off gate checks all three FCLK domains independently, requires zero
internal unconstrained endpoints, and verifies that inter-domain paths retain
their XPM-owned max-delay constraints. It rejects `TIMING-24`, unsafe clock
interactions, critical CDC findings, negative bus-skew slack, new methodology
or DRC rule IDs, incomplete routing, and a missing route checkpoint. It also
requires the generated hierarchical control-set reports and bounds design and
`config_ctrl` control sets to `1217/435` after synthesis and `1184/422` after
implementation. Crossing one of those ceilings requires an explicit
complexity review and baseline update.

The parent XDC does not use global asynchronous clock groups. Its only manual
CDC exceptions target named first-stage synchronizer pins in four groups; the
second synchronizer stage remains timed. The gate requires exactly 40 pins
after both synthesis and implementation (`24/12/2/2`), all with
`ASYNC_REG=TRUE`. The diagnostic group contains only the consumed
`cmd_collision`, `err_bus_fatal`, and `init_cfg_coalesced` masks. There is no
stage-specific allowance for downstream-unobserved diagnostic logic; any
change to the fixed 40-pin baseline is a sign-off failure.

## Current reference evidence

| Item | Result |
|---|---|
| Session | `260721_final_timing_generic_ps_fclk_parent_ref` |
| Slope contract | present `1111`, rise `0011`, fall `1100` |
| Build limits | 8 stops/chip, 7 hits/stop |
| Generic parity | top 25, parent 25, same-name map 25 |
| Block design | generated and validated for `xc7z020clg484-2` |
| Contract checks | 58 exact checks passed |
| Verdict | `PARENT_REF_VALIDATE_PASS` |

The current generic-mask RTL has not yet inherited the older implementation
numbers below. They remain a historical physical baseline only and must not be
used as current-route sign-off evidence.

### Previous implementation baseline

| Item | Result |
|---|---|
| Session | `260719_cfgowner_impl_ps_fclk_parent_ref` |
| Production RTL | `5864438` |
| Control-set reporting | `ff82537` |
| Sign-off gate | `21f0eff` |
| Route utilization | 18,678 LUT, 26,065 FF |
| Route control sets | 1,184 design, 422 `config_ctrl` |
| Route timing | WNS `+0.305 ns`, WHS `+0.019 ns`, WPWS `+1.370 ns` |
| FCLK0/FCLK1/FCLK2 WNS | `+1.503 / +0.453 / +0.305 ns` |
| Bus skew | 38 constraints, minimum slack `+4.103 ns` |
| Manual CDC | 40 exact pins after synthesis and route |
| CDC | 0 critical, `CDC-6=0`; 160 reviewed XPM FIFO `CDC-15` paths |
| Routing | 38,863 fully routed nets, 0 routing errors |
| Verdict | `REFERENCE_IMPL_PASS_WITH_BOARD_IO_OPEN` |

The 48-bit rise-HSIZE/fall-HSIZE/VSIZE tuple crosses from FCLK1 to FCLK0 as one
atomic snapshot. Software must still treat geometry reconfiguration as a
stopped-state operation: stop acquisition and both VDMAs, wait for the TDC
configuration/CDC idle indication, read both AXI GPIO channels, program VDMA
HSIZE/VSIZE/STRIDE and frame buffers, then start the next run. AXI GPIO channel
reads are separate bus transactions, so changing geometry while running is not
a supported contract.

## Sign-off boundary

This is an integration and timing reference, not a board bitstream sign-off.
`REFERENCE_SYNTH_PASS_WITH_BOARD_IO_OPEN` and
`REFERENCE_IMPL_PASS_WITH_BOARD_IO_OPEN` are the strongest valid results from
this board-independent shell. `NSTD-1` and `UCIO-1` remain explicit waivers;
the gate does not lower their severity or hide them.
The following remain intentionally open until the target board is defined:

- PS7 board preset, DDR and MIO electrical configuration.
- TDC physical pin LOC, IOSTANDARD and board timing constraints.
- Real laser/echo producer integration; those inputs are idle-tied here.
- Software VDMA frame-buffer addresses, stride, HSIZE and VSIZE programming.
- Measured DDR and Ethernet throughput/reserve under the final software stack.

Do not treat implementation timing or DRC from this shell as board sign-off
until those items are closed.
