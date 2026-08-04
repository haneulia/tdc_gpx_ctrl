# TDC-GPX LiDAR Controller v2 RTL

This tree contains the clean v2 implementation. The verified v1 RTL remains
unchanged and is used as the observable-behavior reference during migration.

## Directory Contract

| Path | Purpose | Synthesis source |
|---|---|---|
| `pkg/lidar_build_pkg.vhd` | Build topology, legal generic values and stable error codes | Yes |
| `pkg/lidar_config_types_pkg.vhd` | Runtime source and derived record types | Yes |
| `pkg/lidar_config_reference_pkg.vhd` | Exact arithmetic oracle for tests and equivalence checks | No |
| `rtl/config/` | Sequential validator, derivation controller and shared arithmetic | Yes |
| `tb/` | Self-checking package and RTL tests | No |
| `scripts/` | Reproducible, compact regressions | No |

`lidar_config_reference_pkg` contains wide division on purpose. Production RTL
does not call it. The sequential commit calculator derives the same values over
multiple clocks and is checked against this reference model.

## Current Status

- Checkpoint A: architecture contracts complete and committed.
- Checkpoint B: configuration types and reference arithmetic verified.
- Checkpoint C: sequential commit calculator verified at 150/200 MHz.
- Checkpoint D: atomic configuration manager and Processing/TDC domain gateways
  verified for 150/200 and 200/150 MHz asynchronous profiles.
- Unified CSR bank and the v2 integrated functional datapath are not implemented
  yet.

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
