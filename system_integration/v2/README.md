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
| `pkg/lidar_config_reference_pkg.vhd` | Exact arithmetic oracle for tests and equivalence checks | No |
| `rtl/config/` | Sequential validator, derivation controller and shared arithmetic | Yes |
| `rtl/csr/` | AXI4-Lite shadow/status/IRQ bank and atomic-config boundary | Yes |
| `rtl/proc/` | Direct registered Motor/Face/Shot/Laser Processing path | Yes |
| `tb/` | Self-checking package and RTL tests | No |
| `scripts/` | Reproducible, compact regressions | No |

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
- Stage 4 / Checkpoint G Echo frontend is closed. Stage 5 is active: H0 froze
  the B5 oracle, H1 wrapped the proven physical GPX bus and H2A verified the
  atomic Processing/TDC Shot/result gateways. H2B acquisition coordination,
  frame/formatter and the full parent datapath remain unmigrated; the existence
  of their v1 cores does not mark those Stages complete.

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
direct low-latency STOP path, and H2A now owns only the atomic event CDC
boundary. The next implementation is the typed H2B acquisition coordinator.

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
