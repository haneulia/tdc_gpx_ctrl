# TDC-GPX LiDAR Controller v2 RTL

This tree contains the clean v2 implementation. The verified v1 RTL remains
unchanged and is used as the observable-behavior reference during migration.

## Directory Contract

| Path | Purpose | Synthesis source |
|---|---|---|
| `pkg/lidar_build_pkg.vhd` | Build topology, legal generic values and stable error codes | Yes |
| `pkg/lidar_config_types_pkg.vhd` | Runtime source and derived record types | Yes |
| `pkg/lidar_event_types_pkg.vhd` | Registered Processing event records and latency metadata | Yes |
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
- Stage 2 is closed. Stage 3 / Checkpoint F is active; F0 oracle/source-mode
  closure and F1 `motor_position_core`/B0 comparison are complete, so the next
  allowed sub-step is F2 `face_tracker` and B1 comparison.
- The v2 integrated functional datapath is not implemented yet; the existence
  of a v1 core does not mark the corresponding v2 Stage complete.

## Next Implementation Order

Checkpoint F proceeds in this fixed order:

1. freeze the v1 B0..B3 trace oracle;
2. replace writable v1 latency fields with committed `SIMULATION_MODE`, then
   rerun every Stage 2 configuration regression;
3. `motor_position_core` and B0 comparison;
4. `face_tracker` and B1 comparison;
5. operation/safety state owner, then `shot_scheduler` and B2 comparison;
6. `laser_executor` and B3 comparison;
7. direct registered-path integration and read-only AXIS monitoring.

Each boundary must pass before the next block is migrated. The final
Checkpoint F gate runs Processing/TDC 150/200 and 200/150 MHz profiles.

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
