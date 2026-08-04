# TDC-GPX LiDAR Controller v2 RTL

This tree contains the clean v2 implementation. The verified v1 RTL remains
unchanged and is used as the observable-behavior reference during migration.

## Directory Contract

| Path | Purpose | Synthesis source |
|---|---|---|
| `pkg/lidar_build_pkg.vhd` | Build topology, legal generic values and stable error codes | Yes |
| `pkg/lidar_config_types_pkg.vhd` | Runtime source and derived record types | Yes |
| `pkg/lidar_config_reference_pkg.vhd` | Exact arithmetic oracle for tests and equivalence checks | No |
| `tb/` | Self-checking package and RTL tests | No |
| `scripts/` | Reproducible, compact regressions | No |

`lidar_config_reference_pkg` contains wide division on purpose. Production RTL
must not call it. The v2 commit calculator will derive the same values over
multiple clocks and will be checked against this reference model.

## Current Status

- Checkpoint A: architecture contracts complete and committed.
- Checkpoint B: configuration types and reference arithmetic verified.
- Functional cores and the v2 integrated top are not implemented yet.

Run the current package regression with:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/scripts/run_v2_config_pkg.ps1
```

The pass marker is `LIDAR_V2_CONFIG_TYPES_PASS`.
