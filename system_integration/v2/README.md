# TDC-GPX LiDAR Controller v2 RTL

This tree contains the clean v2 implementation. The verified v1 RTL remains
unchanged and is used as the observable-behavior reference during migration.

## Directory Contract

| Path | Purpose | Synthesis source |
|---|---|---|
| `pkg/lidar_build_pkg.vhd` | Build topology, legal generic values and stable error codes | Yes |
| `pkg/lidar_config_types_pkg.vhd` | Runtime source and derived record types | Yes |
| `pkg/lidar_csr_map_pkg.vhd` | Unified CSR address/field and pack/unpack ABI | Yes |
| `pkg/lidar_config_reference_pkg.vhd` | Exact arithmetic oracle for tests and equivalence checks | No |
| `rtl/config/` | Sequential validator, derivation controller and shared arithmetic | Yes |
| `rtl/csr/` | AXI4-Lite shadow/status/IRQ bank and atomic-config boundary | Yes |
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
- Stage 2 is closed. The next and only active migration target is Stage 3 /
  Checkpoint F, the Processing event pipeline and B0..B3 comparison.
- The v2 integrated functional datapath is not implemented yet; the existence
  of a v1 core does not mark the corresponding v2 Stage complete.

## Next Implementation Order

Checkpoint F proceeds in this fixed order:

1. freeze the v1 B0..B3 trace oracle;
2. `motor_position_core` and B0 comparison;
3. `face_tracker` and B1 comparison;
4. `shot_scheduler` and B2 comparison;
5. `laser_executor` and B3 comparison;
6. direct registered-path integration and read-only AXIS monitoring.

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
