# LiDAR Multi-IP Integration Baseline

This directory turns `tb_tdc_gpx_full_int.vhd` into the executable contract
between the sibling IP modules and `tdc_gpx_top`.

## Source ownership

The production source of each sibling IP remains in its canonical `HDL`
directory. The integration runner references those files; it does not copy
generated Vivado project sources.

| Function | Canonical source |
|---|---|
| Motor decoder and virtual encoder | `C:/Projects/my_sp/lib/IP/motor_decoder/HDL` |
| Laser controller | `C:/Projects/my_sp/lib/IP/laser_ctrl/HDL` |
| Motor/Laser integration wrapper | `C:/Projects/my_sp/lib/IP/motor_laser_ctrl/HDL` |
| Echo receiver | `C:/Projects/my_sp/lib/IP/echo_receiver/HDL` |
| TDC-GPX controller | this repository |

The sibling directories are not Git repositories yet. Every run therefore
records a SHA-256 hash for every compiled source file. Before changing a
sibling IP, establish its own Git baseline; hashes are reproducibility evidence,
not a replacement for source control.

## Encoder modes

`motor_laser_ctrl_top` contains `motor_decoder_top`, which already contains
`enc_top` for simulation. The integration TB supports two mutually exclusive
modes:

- `internal`: use the encoder inside the wrapped Motor Decoder; physical A/B/Z
  inputs are tied low.
- `external`: instantiate a TB-only encoder and select the wrapped Motor
  Decoder's physical A/B/Z path.

A product parent does not need a second virtual encoder instance.

Both modes use one clock-derived encoder profile. For the current 100 us
revolution at 200 MHz, the fractional scheduler uses 13/14 clocks per
quadrature state. Reusing the sibling TB's old 100 MHz or 150 MHz constants
would change the physical RPM and shot PRF. The external scenario observes
220 us because the first revolution establishes the Z-index reference; the
following revolution is the first complete measurement interval.

## Clock profiles

Motor, Laser, Echo, and the GPX processing path use `axis_clock_mhz`. The GPX
physical-bus model and chip controller use the independent `tdc_clock_mhz`.
Both values must be one of 50, 100, 125, 150, or 200 MHz and AXIS must not be
faster than TDC. A scenario that omits `tdc_clock_mhz` remains a same-clock run
for schema-v1 compatibility.

The maintained integration gate is AXIS 150 MHz / TDC 200 MHz. It is run once
with the internal encoder and once with the external A/B/Z path. Same-clock
profiles remain available for diagnosis, but they are not part of the routine
two-scenario regression.

The shared CSR range remains in 5 ns ticks. `rtl_contract.json` reports the two
derived values separately as `max_range_axis_clks` and
`max_range_tdc_clks`; there is no ambiguous `max_range_local_clks` field in
schema version 2.

## Result contract

Each run archives:

- the scenario JSON;
- the TDC repository commit;
- SHA-256 hashes of all canonical and generated dependency files;
- compile, elaborate, and simulation logs;
- `rtl_result.json`, parsed from the TB's `RTL_RESULT` marker, including the
  full source manifest and worktree-dirty flag;
- `rtl_contract.json`, the compact scenario/metric contract consumed by the
  C08 HTML simulator.

`STAT5`, `STAT6`, and `STAT7` are always stored as eight-digit uppercase hex
strings. This preserves leading zeroes and gives HTML one stable data type.
VDMA rise/fall HSIZE and shared VSIZE are sampled from the RTL output ports,
not recalculated by the runner.

The contract also carries the topology in two layers. The elaborated build
profile is reported as `present_chip_mask`, `rise_capability_mask`, and
`fall_capability_mask`; the CTL21/active-mask result is reported separately as
`runtime_rise_chip_mask` and `runtime_fall_chip_mask`. `chip_slope_mask` is only
the slope bit emitted by the behavioral GPX model. It is stimulus data and must
not be used as a substitute for the DUT's build-time topology.

The TB writes Chip CSR `CTL21` before `CFG_WRITE` and `START`. Its
distance-derived `max_hits` value therefore controls the actual Cell payload;
leaving this field at zero would select the safe build-maximum alias of seven
hits and invalidate any width/throughput comparison. The current smoke keeps
`max_scan_5ns_ticks=0` because a nonzero Face-assembler timeout must include
separately justified GPX drain and AXIS service margins. Copying the range
window alone into that field is not a safe timeout policy.

Each accepted VDMA line is checked immediately against the exported HSIZE.
The fixed observation interval is then extended only as needed to reach the
next complete `columns_per_face` boundary before Motor-to-Laser requests are
gated. This keeps the normal smoke free of an intentionally incomplete Face;
mid-Face cancellation and partial-header behavior belong in a dedicated abort
scenario.

Pipeline CSR `RANGE_COLS[31:16]` is the sole owner of `cols_per_face`. The
current Laser result stream carries `step_idx + 1` in bits 15:0 and the
remaining step count in bits 31:16; it is not a geometry stream and is not
connected to the GPX pipeline configuration. This single-owner rule prevents
live Laser progress from changing VDMA VSIZE during a face.

## External GPX I-Mode ownership

The system smoke keeps the production boundaries explicit:

1. `echo_receiver_top` creates physical STOP waveforms. In simulation its CSR
   delay table may create deterministic STOPs; fixed patterns do not belong to
   `tdc_gpx_top`.
2. `tdc_gpx_external_chip_model` is a simulation-only external-chip model. It
   measures START-to-STOP time, fills IFIFO1/2, and drives the GPX status pins
   and 28-bit data bus.
3. `tdc_gpx_top` reads and decodes the external bus. It has no generic that
   calculates or replaces a GPX Hit value.

The implemented SINGLE_SHOT I-Mode word is:

| Bits | Field | Contract |
|---|---|---|
| `[27:26]` | `ChaCode` | channel 0..3 inside the selected IFIFO |
| `[25:18]` | `StartNum` | zero in the current SINGLE_SHOT profile |
| `[17]` | `Slope` | rising/falling edge direction |
| `[16:0]` | `Hit` | 17-bit external GPX time/distance code |

The scoreboard observes START and Echo STOP independently, checks every raw
bus word, and reconstructs the final Hit from the canonical VDMA cell:
`Hit[15:0]` in the hit slot plus `Hit[16]` in metadata bit 0. The result
contract publishes `i_mode_hit_refs`, rise/fall check counts, raw bus check
count, and the last expected/rise/fall Hit values.

The Stage I1 pass marker is `SYSTEM_INTEGRATION_SMOKE_PASS`. It requires
activity through Motor, Laser, Echo, GPX acquisition, and both VDMA slope
streams. It also requires exact Laser start/stop/result-TLAST accounting,
rejects a Motor-to-Laser `face_valid` loss, and exact-compares the healthy
pipeline status contract:

- `STAT5 = 0x00000001`: the GPX run remains enabled; no fault bits are set.
- `STAT6[31:28] = active_chip_mask`: every active chip completed a drain.
- `STAT7 = 0x00000000`: no extended fault or masked-slope drop.

The Laser `stop_tdc` pulse marks the end of the current measurement window.
It is not the next-shot deadline or a GPX session-stop command. A sequence
error is raised only when that pulse arrives before synchronized GPX IrFlag;
arrival after IrFlag while the chip drains or runs the ALU is normal.

The 150/200 MHz scenarios prove the AXIS/TDC functional CDC path in simulation.
The parent reference structurally validates the AXI 100 MHz clock, but this TB
does not yet dynamically exercise an independent AXI clock. Board pin timing,
long-stall behavior, and post-route closure remain separate sign-off gates.

## C08 HTML comparison

Open
`Doc/cluster_analysis/C08_HDL_HTML_Alignment/C08_HDL_HTML_Alignment_260727_External_GPX_I_Mode_Integration_Simulator_v022.html`
and select either `rtl_contract.json` or the full `rtl_result.json` in the
**RTL contract comparison** panel. With **Apply scenario** enabled, the HTML
loads the clock pair, output width, range/scan CSR values, max hits, build and
runtime slope masks, stops, Face geometry, and target distance before comparing
the observed RTL metrics.

PASS requires schema version 2, an RTL PASS marker, explicit build/runtime mask
evidence, exact HSIZE/VSIZE and clock conversions, complete-line beat
accounting, balanced Laser transactions, every active chip's raw 28-bit I-Mode
word, exact 17-bit Hit preservation, expected `STAT5/6/7`, and zero fault
counters. A legacy or ambiguous contract remains readable but is intentionally
shown as CHECK; inferred stimulus masks are not sign-off evidence.

## Run

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/scripts/run_smoke.ps1 `
  -Scenario system_integration/scenarios/smoke_internal_axis150_tdc200_v001.json
```

Run the physical A/B/Z input path at the same 150/200 MHz clock pair:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/scripts/run_smoke.ps1 `
  -Scenario system_integration/scenarios/smoke_external_axis150_tdc200_v001.json
```

`-EncoderSource` remains available for short experiments, but archived
`scenario.json` always contains the effective configuration after overrides.
