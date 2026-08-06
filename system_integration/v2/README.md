# TDC-GPX LiDAR Controller v2 RTL

This tree contains the clean v2 implementation. The verified v1 RTL remains
unchanged and is used as the observable-behavior reference during migration.

## Directory Contract

| Path | Purpose | Synthesis source |
|---|---|---|
| `pkg/lidar_build_pkg.vhd` | Build topology, legal generic values and stable error codes | Yes |
| `pkg/lidar_config_types_pkg.vhd` | Runtime source and derived record types | Yes |
| `pkg/lidar_event_types_pkg.vhd` | Registered Processing event records and latency metadata | Yes |
| `pkg/lidar_processing_pkg.vhd` | Integrated Processing diagnostics, monitor ABI and measured path-latency contracts | Yes |
| `pkg/lidar_csr_map_pkg.vhd` | Unified CSR address/field and pack/unpack ABI | Yes |
| `pkg/lidar_gpx_pkg.vhd` | Typed GPX bus, image, Shot and acquisition-result contracts | Yes |
| `pkg/lidar_gpx_image_pkg.vhd` | Adapter to the single board-proven v1 GPX default image | Yes |
| `pkg/lidar_gpx_data_pkg.vhd` | I-Mode fields plus typed B6 Hit, B7 Cell and B8 Frame-lane contracts | Yes |
| `pkg/lidar_config_reference_pkg.vhd` | Exact arithmetic oracle for tests and equivalence checks | No |
| `rtl/config/` | Sequential validator, derivation controller and shared arithmetic | Yes |
| `rtl/csr/` | AXI4-Lite shadow/status/IRQ bank and atomic-config boundary | Yes |
| `rtl/proc/` | Direct registered Motor/Face/Shot/Laser Processing path | Yes |
| `tb/` | Self-checking package and RTL tests | No |
| `scripts/` | Reproducible, compact regressions | No |
| `sw_reference/` | Portable C PACKED17-to-Viewer decoder and ownership API | PS software |

`lidar_config_reference_pkg` contains wide division on purpose. Production RTL
does not call it. The sequential commit calculator derives the same values over
multiple clocks and is checked against this reference model.

## Current Status

- The authoritative Stage/Checkpoint mapping is Section 5 of
  `../v2_architecture/V2_MIGRATION_VERIFICATION_PLAN.md`.
- Checkpoint A: architecture contracts complete and committed.
- Checkpoint B: configuration types and reference arithmetic verified.
- Checkpoint C: sequential commit calculator verified at 150/200 MHz.
- Checkpoint D: atomic configuration manager and Processing/TDC domain gateways
  verified for 150/200 and 200/150 MHz asynchronous profiles.
- Checkpoint E: unified 32 CTL / 32 STAT / 4 IRQ CSR bank integrated with the
  atomic configuration subsystem and route-verified for both clock profiles.
- Stage 2 and Stage 3 / Checkpoint F are closed. F0 through F5 cover the
  Processing source contract, B0 through B3, production assembly, local drain
  and the observation-only AXIS monitor.
- Stage 4 / Checkpoint G Echo frontend and Stage 5 / Checkpoint H are closed. H0 froze
  the B5 oracle, H1 wrapped the proven physical GPX bus, H2A verified the
  atomic Processing/TDC Shot/result gateways, H2B-1/2A completed the typed
  lane/coordinator, H2B-2B completed indexed image activation and H3 closed
  the 32-physical-STOP / 16-logical-APD x 7-Return B5 acquisition boundary.
  Stage 6 I0 froze the Hit/Cell/Frame oracle, I1 completed the typed B6 Hit
  decoder, I2 completed the width-independent B7 Cell collector, I2A closed
  their registered ownership/timing optimization and I3 completed canonical
  Rise/Fall B8 Frame-lane assembly, including one-Chip and four-Chip all-dual
  slope operation. I4 now closes the integrated B5..B8 path, explicit
  Face-close ownership, trailing/all-hole completion and both routine
  asynchronous clock profiles. Stage 7 J1/J2 established the serializer and
  legacy lane-formatter baseline. J3 froze the PACKED17 target ABI and HTML
  Golden model, and J4 now owns the target geometry functions, exact Cell
  Metadata map and intentional-filter versus physical-overflow semantics.
  J5A/J5B close T0/Shot Metadata and explicit Hole Lines, J6 closes the sole
  32/64/128-bit AXIS packer, and J7/J8 close the ordered Face Footer plus
  sequential, acknowledged Face-boundary VDMA profile activation. The DDR
  Golden comparison is closed by J9, and J10 closes the portable PS H-Line
  decoder, Viewer packet byte comparison and Cortex-A9 build. The physical
  parent VDMA/HP/cache path remains unmigrated; the existence of its v1 cores
  does not mark that Stage complete.

## Next Implementation Order

Checkpoint F was completed in this fixed order:

1. freeze the v1 B0..B3 trace oracle;
2. replace writable v1 latency fields with committed `SIMULATION_MODE`, then
   rerun every Stage 2 configuration regression;
3. `motor_position_core` and B0 comparison;
4. `face_tracker` and B1 comparison;
5. operation/safety state owner, then `shot_scheduler` and B2 comparison;
6. `laser_executor` and B3 comparison;
7. direct registered-path integration and read-only AXIS monitoring.

Each boundary passed before the next block was migrated. Checkpoint F ran the
Processing/TDC 150/200 and 200/150 MHz profiles, Checkpoint G preserved the
direct low-latency STOP path, and H2A through H3 now own the atomic event CDC
and typed acquisition boundaries. Stage 6 I1 owns B6 raw parsing, I2 owns B7
Return ordering and Hit-to-Cell collection, I2A proves their direct linked
boundary, I3 owns width-neutral B8 Rise/Fall Frame-lane assembly, and I4 proves
the complete B5..B8 production chain plus explicit Face-close ordering. J7/J8
now close the focused B9 geometry and Footer boundary. J9 and J10 close the
DDR image and host PS/Viewer comparison. The next implementation is J11, the
parent VDMA/HP-port, real cache ownership and board measurement gate.

Run the current package regression with:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/scripts/run_v2_config_pkg.ps1
```

The pass marker is `LIDAR_V2_CONFIG_TYPES_PASS`.

Run the commit-calculator functional and implementation regression with:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/scripts/run_v2_commit_calculator.ps1
```

The pass marker is `LIDAR_V2_COMMIT_CALCULATOR_PASS`. The detailed Checkpoint C
result is in `system_integration/v2_architecture/V2_CHECKPOINT_C_COMMIT_CALCULATOR.md`.

Run the configuration-manager regression with:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/scripts/run_v2_config_manager.ps1
```

The pass marker is `LIDAR_V2_CONFIG_MANAGER_PASS`. The regression requires
non-negative post-route WNS, zero inferred latches, 28 recognized ASYNC_REG
flops and zero Critical CDC paths. See
`system_integration/v2_architecture/V2_CHECKPOINT_D_CONFIG_MANAGER.md`.

Run the unified-CSR functional and implementation regression with:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/scripts/run_v2_unified_csr.ps1
```

The pass marker is `LIDAR_V2_UNIFIED_CSR_PASS`. It runs the CSR map/package,
AXI bank and both asynchronous clock profiles, then requires non-negative
post-route WNS, zero latches and zero Critical CDC paths. See
`system_integration/v2_architecture/V2_CHECKPOINT_E_UNIFIED_CSR.md` and
`system_integration/v2_architecture/V2_UNIFIED_CSR_REGISTER_MAP.md`.

Run the Motor-position functional and implementation regression with:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/scripts/run_v2_motor_position.ps1
```

The pass marker is `LIDAR_V2_MOTOR_POSITION_PASS`. It runs P00..P04 at 150
and 200 MHz, then requires non-negative post-route WNS, zero latches, 12
physical synchronizer ASYNC_REG flops and zero Critical CDC findings. See
`system_integration/v2_architecture/V2_CHECKPOINT_F1_MOTOR_POSITION.md`.

Run the Face-tracker functional and implementation regression with:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/scripts/run_v2_face_tracker.ps1
```

The pass marker is `LIDAR_V2_FACE_TRACKER_PASS`. It runs P10..P13 for build
Face counts 1 through 5 at 150 and 200 MHz, including runtime masks, modular
wrap, reversal, zero-gap Face changes, overlap diagnostics and II=1 input.
The worst-case 5-Face build must have non-negative post-route WNS, zero
latches and zero Critical CDC paths. See
`system_integration/v2_architecture/V2_CHECKPOINT_F2_FACE_TRACKER.md`.

Run the operation/safety functional, CDC and implementation regression with:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/scripts/run_v2_operation.ps1
```

The pass marker is `LIDAR_V2_OPERATION_PASS`. It runs reset, RUN/STOP,
ARM/DISARM, physical permit loss/re-arm, simulation exclusion, mailbox busy
rejection, destination-reset flushing and CSR-reset command-authority revocation
at 50, 150 and 200 MHz. The 150/200
MHz implementation gate requires non-negative WNS, zero latches and zero
Critical CDC findings. See
`system_integration/v2_architecture/V2_CHECKPOINT_F3A_OPERATION_SAFETY.md`.

Run the Shot-scheduler functional, integration and implementation regression
with:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/scripts/run_v2_shot_scheduler.ps1
```

The pass marker is `LIDAR_V2_SHOT_SCHEDULER_PASS`. It runs P20..P24 and the
F3a/B1/B2 P25 chain at 150 and 200 MHz, then requires non-negative post-route
WNS, zero latches and zero Critical CDC paths. See
`system_integration/v2_architecture/V2_CHECKPOINT_F3B_SHOT_SCHEDULER.md` and
`system_integration/v2_architecture/V2_PROCESSING_PIPELINE_INTEGRATION_GUIDE.md`.

Run the Laser-executor boundary regression with:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/scripts/run_v2_laser_executor.ps1
```

The pass marker is `LIDAR_V2_LASER_EXECUTOR_PASS`. It runs P30..P36 and the
F3a/B1/B2/B3 control chain at 150 and 200 MHz, then audits the direct physical
START preset path. See
`system_integration/v2_architecture/V2_CHECKPOINT_F4_LASER_EXECUTOR.md`.

Run the production Processing-subsystem regression with:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/scripts/run_v2_processing_subsystem.ps1
```

The pass marker is `LIDAR_V2_PROCESSING_SUBSYSTEM_PASS`. It runs P50..P53 for
150/200 and 200/150 MHz, checks B0..B3 identity and measured latency, stalls the
monitor without stalling control, and requires non-negative WNS, zero latches,
the expected 16 ASYNC_REG cells, exactly two raw `fire_done` endpoints and zero
Critical CDC findings. See
`system_integration/v2_architecture/V2_CHECKPOINT_F5_PROCESSING_SUBSYSTEM.md`.

Run the GPX Shot/result gateway functional, CDC and implementation regression
with:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/scripts/run_v2_gpx_event_gateway.ps1
```

The pass marker is `LIDAR_V2_GPX_EVENT_GATEWAY_PASS`. It verifies exact
95-bit Shot and 149-bit raw-result payload order under backpressure for
150/200, 200/150 and shared-clock 150 MHz profiles. See
`system_integration/v2_architecture/V2_CHECKPOINT_H2A_GPX_EVENT_GATEWAYS.md`.

Run the multi-Chip acquisition coordinator regression with:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/scripts/run_v2_gpx_acquisition_coordinator.ps1
```

The pass marker is `LIDAR_V2_GPX_ACQUISITION_COORDINATOR_PASS`. It verifies
atomic Shot broadcast, deterministic registered merge, terminal completion and
the GPX image activation link at 150/200 and 200/150 MHz. See
`system_integration/v2_architecture/V2_CHECKPOINT_H2B2A_GPX_COORDINATOR.md` and
`system_integration/v2_architecture/V2_CHECKPOINT_H2B2B_GPX_CONFIG_ACTIVATION.md`.

Run the production GPX acquisition-subsystem regression with:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/scripts/run_v2_gpx_acquisition_subsystem.ps1
```

The pass marker is `LIDAR_V2_GPX_ACQUISITION_SUBSYSTEM_PASS`. It verifies
4 Chip x 8 STOP x 7 Return with dedicated 2-rise/2-fall roles, exact
raw-28/Shot identity, timeout recovery, build-derived physical cap/purge and
one-full-Shot backpressure for 150/200 and 200/150 MHz. The 32 physical GPX
STOP lanes represent 16 logical APD channels observed on both slopes. The
implementation gate requires non-negative WNS, zero latches, zero Critical CDC
and zero blocking DRC. Echo delay remains CTL20
`CH0 + channel * STEP`; no 32-word delay table is added. See
`system_integration/v2_architecture/V2_CHECKPOINT_H3_GPX_ACQUISITION_SUBSYSTEM.md`.

Run the typed GPX Hit-decoder regression with:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/scripts/run_v2_gpx_hit_decoder.ps1
```

The pass marker is `LIDAR_V2_GPX_HIT_DECODER_PASS`. It verifies the complete
I-Mode field mapping, 4-Chip dedicated and one-Chip dual-edge topology,
identity diagnostics, registered input/output behavior and backpressure at
Processing 150 and 200 MHz. See
`system_integration/v2_architecture/V2_CHECKPOINT_I1_GPX_HIT_DECODER.md`.

Run the width-independent GPX Cell-collector regression with:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/scripts/run_v2_gpx_cell_collector.ps1
```

The pass marker is `LIDAR_V2_GPX_CELL_COLLECTOR_PASS`. It verifies runtime
visible-Hit clipping without shortening physical drain, Hit[16] preservation,
dedicated and dual-edge Cell ordering, 8th-Return no-wrap handling,
timeout/abort scrub recovery, B6-B7 direct linking and output backpressure at
Processing 150 and 200 MHz. See
`system_integration/v2_architecture/V2_CHECKPOINT_I2_GPX_CELL_COLLECTOR.md`.
The ownership and timing optimization evidence is in
`system_integration/v2_architecture/V2_CHECKPOINT_I2A_GPX_PIPELINE_OPTIMIZATION.md`.

Run the canonical GPX Frame-lane regression with:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/scripts/run_v2_gpx_frame_lane_assembler.ps1
```

The pass markers are `LIDAR_V2_GPX_FRAME_LANE_ASSEMBLER_PASS` and
`LIDAR_V2_GPX_RUNTIME_SLOPE_MASKS_PASS`. The regression verifies dedicated,
one-Chip dual-edge, Falling-OFF all-Rise, fault and independent-lane
backpressure scenarios at Processing 150 and 200 MHz. See
`system_integration/v2_architecture/V2_CHECKPOINT_I3_GPX_FRAME_LANE_ASSEMBLER.md`.

Run the integrated B5-B8 GPX regression with:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/scripts/run_v2_gpx_b5_b8_subsystem.ps1
```

The pass marker is `LIDAR_V2_GPX_B5_B8_SUBSYSTEM_PASS`. It drives dedicated
2-Rise/2-Fall and four-Chip all-dual GPX bus models, compares every lower
17-bit Hit through B5-B8, and verifies leading/trailing holes, an all-hole
Face and close backpressure at 150/200 and 200/150 MHz. OOC implementation
requires non-negative WNS, three
asynchronous FIFOs, 230 ASYNC_REG cells, zero latches, zero Critical CDC and
zero blocking DRC categories. See
`system_integration/v2_architecture/V2_CHECKPOINT_I4_GPX_B5_B8_INTEGRATION.md`.

Run the target PACKED17 ABI package and Cell-word regression with:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/scripts/run_v2_gpx_cell_word_serializer.ps1 `
  -SkipImplementation
```

The pass marker is `LIDAR_V2_GPX_CELL_WORD_SERIALIZER_PASS`. J4 verifies the
exact Cell Metadata bit map, target HSIZE/VSIZE/STRIDE calculations for
32/64/128-bit builds, visible Return counts 1 through 7, and the separation of
intentional runtime filtering from an eighth physical Return overflow. See
`system_integration/v2_architecture/V2_CHECKPOINT_J4_PACKED17_ABI_GEOMETRY.md`.

Run the target-width AXIS packer regression with:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/scripts/run_v2_gpx_axis_word_packer.ps1
```

The pass marker is `LIDAR_V2_GPX_AXIS_WORD_PACKER_PASS`. J6 proves canonical
Word order, final-Beat-only zero padding, SOF/TLAST and output stability for
150/200 MHz and 32/64/128-bit builds. See
`system_integration/v2_architecture/V2_CHECKPOINT_J6_AXIS_WORD_PACKER.md`.

Run the Face Footer and VDMA-profile regression with:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/scripts/run_v2_gpx_face_footer.ps1
```

The pass markers are `LIDAR_V2_GPX_FACE_FOOTER_PASS`,
`LIDAR_V2_GPX_VDMA_PROFILE_MANAGER_PASS`, and
`LIDAR_V2_GPX_FRAME_CLOSE_FORK_PASS`. J7/J8 verify one/two-Line Footer layout,
fixed maximum STRIDE, safe Face-boundary activation, VDMA programming
backpressure, abort preservation, and positive post-route timing for all six
150/200 MHz by 32/64/128-bit profiles. See
`system_integration/v2_architecture/V2_CHECKPOINT_J7_J8_VDMA_PROFILE_FOOTER.md`.

Run the DDR image versus HTML Golden regression with:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/scripts/run_v2_gpx_ddr_golden.ps1
```

The final marker is `LIDAR_V2_GPX_DDR_GOLDEN_PASS`. J9 executes the model in
the C08 HTML, checks the frozen Golden JSON, captures a real Shot, a Hole Shot,
and the ordered Footer through the RTL AXIS path, then compares every DDR Word
and fixed-STRIDE reserve at 150/200 MHz for 32/64/128-bit builds. See
`system_integration/v2_architecture/V2_CHECKPOINT_J9_DDR_HTML_GOLDEN.md`.

Run the J9-to-J10 DDR, PS H-Line and Ethernet packet comparison with:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/scripts/run_v2_gpx_ps_hline.ps1
```

The final marker is `LIDAR_V2_GPX_PS_HLINE_SIGNOFF_PASS`. The script creates
fresh 150/200 MHz by 32/64/128-bit DDR captures, executes the portable C PS
decoder, compares every Viewer packet byte against the HTML Golden model, and
cross-compiles the same decoder for Cortex-A9. See
`system_integration/v2_architecture/V2_CHECKPOINT_J10_PS_HLINE_ETHERNET.md` and
`system_integration/v2_architecture/V2_PS_HLINE_ETHERNET_ABI_KO.md`.
