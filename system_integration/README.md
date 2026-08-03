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

The sibling IPs are maintained as independent Git repositories. Every run also
records a SHA-256 hash for every compiled source file because an integration
result spans several repository commits and may include an intentionally dirty
working tree.

## Unified CSR migration

The staged migration from duplicated local CSR banks to one 32 CTL / 32 STAT /
4 IRQ bank is defined in `UNIFIED_CSR_MIGRATION_PLAN.md`. The executable address
allocation is `rtl/lidar_unified_csr_pkg.vhd`; run its static contract gate with:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/scripts/run_unified_csr_contract.ps1
```

The migration does not place real-time Motor, Laser, Echo, or GPX event signals
behind software registers. It changes configuration ownership and software
observability only.

The Stage 6 source-level owner is
`rtl/lidar_unified_csr_top.vhd`; its register, startup, status and interrupt
contract is documented in `UNIFIED_CSR_TOP_GUIDE.md`. Run its focused AXI and
aggregation gate with:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/scripts/run_unified_csr_top.ps1
```

The IPI-distributable owner is generated separately at
`C:/Projects/my_sp/lib/IP/lidar_unified_csr/ip_repo`. Its thin
`rtl/lidar_unified_csr_ip_top.vhd` wrapper gives Motor, Laser, Echo, and TDC
one custom Master interface each while retaining exactly one internal CSR32
owner. Regenerate and check it with:

```powershell
vivado.bat -mode batch -source package_lidar_unified_csr_ip.tcl
vivado.bat -mode batch `
  -source system_integration/scripts/check_lidar_unified_csr_ip_package.tcl
vivado.bat -mode batch `
  -source system_integration/scripts/run_lidar_unified_csr_packaged_ip_ooc.tcl
```

The package checker validates two IPI layouts: the four standalone child IPs,
and the product parent layout using one `motor_laser_ctrl` plus Echo Receiver
and TDC-GPX. In the parent layout the combined Motor/Laser IP is configured
with `g_ENABLE_LOCAL_CSR=false`, so its two local AXI banks disappear and the
two named unified CSR Slave interfaces connect directly to the central owner.

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
Both values must be one of 50, 100, 125, 150, or 200 MHz. The maintained
`ASYNC` stream path permits either frequency ordering, including AXIS 150 MHz /
TDC 100 MHz. `SYNC` is legal only when both frequencies are equal and the
physical clocks are synchronous. A scenario that omits `tdc_clock_mhz` remains
a same-clock run for schema-v1 compatibility.

The routine integration gate remains AXIS 150 MHz / TDC 200 MHz. The slower-TDC
closure adds AXIS 150 MHz / TDC 100 MHz external-encoder profiles for board
bring-up. Same-clock profiles remain available for diagnosis.

The shared CSR range remains in 5 ns ticks. `rtl_contract.json` reports the two
derived values separately as `max_range_axis_clks` and
`max_range_tdc_clks`; there is no ambiguous `max_range_local_clks` field in
schema version 2. Scenario schema version 3 additionally records
`bus_clk_div`, `bus_ticks`, and the resulting `bus_word_period_ns` so a PASS is
bound to a reproducible physical-bus profile.

Mirror and Return timing are independent scenario inputs. The revolution
period and optical shot interval derive the real Motor-to-Laser cadence;
`returns_per_stop` selects the physical LVDS pulses actually generated on all
32 GPX STOP pins, while `max_hits_cfg` remains the independent Cell capacity.
The profile is valid only when all of the following hold:

- `returns_per_stop <= max_hits_cfg <= 7`;
- the GPX range window plus `tdc_drain_margin_time_ns` covers capture and FIFO
  drain without `STAT5[11:8]`;
- the mirror-derived shot interval completes without Laser schedule overrun;
- the final rise/fall VDMA lines preserve every expected 17-bit Hit.

The 200 MHz GPX dedicated 2-rise + 2-fall reference profile uses a 6,000 ns
drain margin. This value was closed with `bus_clk_div=2`, `bus_ticks=5`, eight
STOP channels and seven Returns per STOP on each independently drained chip.
It is not a topology-independent constant. When one GPX chip timestamps both
STOP edges, the same R7 shot serializes 112 I-Mode words on one physical bus.
The verified one-chip dual-edge point uses a 12,000 ns drain margin and a
0.26-degree operating resolution at 1,200 RPM. Its 150 MHz AXIS measurements
are 2,709 clocks per planned shot, 2,584 clocks to the latest output, and 125
clocks of remaining point budget. A 0.24-degree point reaches the next shot
before the previous IFIFO/Cell tail is complete. A slower runtime GPX bus
setting requires a new margin sweep; the range value alone does not cover FIFO
readout time.

The verified AXIS 150 MHz / TDC 100 MHz operating points are:

| BUS profile | Word period | RPM / optical interval | Drain margin | Measured point margin |
|---|---:|---:|---:|---:|
| div1 / ticks5 | 50 ns | 1200 RPM / 0.24 deg | 12 us | 202 AXIS clocks (1.347 us) |
| div2 / ticks5 | 100 ns | 1200 RPM / 0.30 deg | 18 us | 322 AXIS clocks (2.147 us) |

Both profiles exercise all 16 APD channels, four GPX chips in dedicated
2-rise + 2-fall mode, and seven Returns per STOP. The 1200 RPM / 0.20 deg
profile is intentionally not accepted at TDC 100 MHz: the next shot arrives
before the previous Return-7 result completes. Clock-domain safety therefore
does not replace the scan-rate budget check.

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
`runtime_rise_chip_mask` and `runtime_fall_chip_mask`. Schema-v3 scenarios use
independent `rise_chip_mask` and `fall_chip_mask` fields. Their bits may overlap,
which means one physical GPX chip captures both STOP edges. With
`runtime_falling_enable=false`, every active chip is routed to rising and the
runtime fall mask is zero. The legacy `chip_slope_mask` field is accepted only
for schema-v1/v2 compatibility and is converted to complementary masks; it
cannot describe an overlapping dual-edge chip.

The TB writes Chip CSR `CTL21` before `CFG_WRITE` and `START`. Its explicit
`max_hits_cfg` value controls the Cell capacity independently of target
distance and actual Return count; leaving this field at zero would select the
safe build-maximum alias of seven hits and invalidate a claimed smaller
geometry. The current smoke keeps
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
count, and the last expected/rise/fall Hit values. In `physical_multi` mode,
all eight STOP pins of every active GPX chip are exercised for Return 1..7.
Dedicated 2-rise + 2-fall mode is the 32-STOP-pin representation of 16 APD
channels split across slope-specific GPX pairs. An overlapping mask instead
checks both edge directions on each enabled STOP of the same chip; the model
queues both slope words into that chip's physical IFIFOs and the scoreboard
checks them independently through the final VDMA streams.

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

The maintained clock-corner scenarios prove AXIS 50/TDC 200 MHz and AXIS
200/TDC 50 MHz ASYNC operation, plus the same-net 150/150 MHz SYNC bypass.
ASYNC synthesis retains four raw-stream CDC FIFO roots; SYNC synthesis removes
all four. Equal-frequency clocks on independent nets must still use ASYNC.
The parent reference structurally validates the AXI 100 MHz clock, but this TB
does not yet dynamically exercise an independent AXI clock. Board pin timing,
long-stall behavior, and post-route closure remain separate sign-off gates.

Every `xpm_cdc_handshake` latest-value sender follows the complete four-phase
contract: assert `src_send`, observe `src_rcv`, deassert `src_send`, then wait
for `src_rcv` to return low before replaying a value that changed in flight.
The transmitted payload comes from a hold register rather than the live bus,
and CDC idle requires both `src_send=0` and `src_rcv=0`. This rule is required
in both frequency orderings; checking only whether the source is slower than
the destination is not sufficient.

The Motor/Laser feasibility gate also treats the angular point interval and
the inactive gap between adjacent active Face windows as independent budgets.
Both must cover the worst re-arm time. A positive point margin cannot hide a
Face-boundary request that arrives while the previous shot is still busy.

## C08 HTML comparison

Open
`Doc/cluster_analysis/C08_HDL_HTML_Alignment/C08_HDL_HTML_Alignment_260729_Unified_CSR_Timing_Contract_Simulator_v025.html`
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
  -Scenario system_integration/scenarios/smoke_internal_axis150_tdc200_v002.json
```

Run the physical A/B/Z input path at the same 150/200 MHz clock pair:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/scripts/run_smoke.ps1 `
  -Scenario system_integration/scenarios/return7_external_axis150_tdc200_v001.json
```

Run the maintained Return/width boundary matrix. It checks Return 1..7 at
32-bit output and repeats the maximum seven-Return payload at 64 and 128 bits:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/scripts/run_return_feasibility_matrix.ps1
```

Add `-FullCrossProduct` only when all 21 Return/width combinations are needed.

Run the slower-TDC board profiles:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/scripts/run_smoke.ps1 `
  -Scenario system_integration/scenarios/return7_external_axis150_tdc100_bus1_v001.json

powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/scripts/run_smoke.ps1 `
  -Scenario system_integration/scenarios/return7_external_axis150_tdc100_bus2_v001.json
```

Run the extreme ASYNC clock ratios, followed by the shared-clock SYNC profile:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/scripts/run_smoke.ps1 `
  -Scenario system_integration/scenarios/return7_external_axis50_tdc200_async_v001.json

powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/scripts/run_smoke.ps1 `
  -Scenario system_integration/scenarios/return7_external_axis200_tdc50_async_v001.json

powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/scripts/run_smoke.ps1 `
  -Scenario system_integration/scenarios/return7_external_axis150_tdc150_sync_v001.json

powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/scripts/run_smoke.ps1 `
  -Scenario system_integration/scenarios/return7_external_axis150_tdc150_sync_5face_boundary_v001.json
```

`SYNC` in the testbench drives `i_tdc_clk` from the AXIS clock signal itself;
it does not model two unrelated 150 MHz oscillators.

`-EncoderSource` remains available for short experiments, but archived
`scenario.json` always contains the effective configuration after overrides.
