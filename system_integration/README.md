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
| Echo receiver | `C:/Projects/my_sp/lib/IP/echo_receiver/HDL` |
| TDC-GPX controller | this repository |

The sibling directories are not Git repositories yet. Every run therefore
records a SHA-256 hash for every compiled source file. Before changing a
sibling IP, establish its own Git baseline; hashes are reproducibility evidence,
not a replacement for source control.

## Encoder modes

`motor_decoder_top` already contains `enc_top` for simulation. The integration
TB supports two mutually exclusive modes:

- `internal`: use the encoder inside `motor_decoder_top`; physical A/B/Z inputs
  are tied low.
- `external`: instantiate a TB-only encoder and select the motor decoder's
  physical A/B/Z path.

A product parent does not need a second virtual encoder instance.

Both modes use one clock-derived encoder profile. For the current 100 us
revolution at 200 MHz, the fractional scheduler uses 13/14 clocks per
quadrature state. Reusing the sibling TB's old 100 MHz or 150 MHz constants
would change the physical RPM and shot PRF. The external scenario observes
220 us because the first revolution establishes the Z-index reference; the
following revolution is the first complete measurement interval.

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

Pipeline CSR `RANGE_COLS[31:16]` is the sole owner of `cols_per_face`. The
current Laser result stream carries `step_idx + 1` in bits 15:0 and the
remaining step count in bits 31:16; it is not a geometry stream and is not
connected to the GPX pipeline configuration. This single-owner rule prevents
live Laser progress from changing VDMA VSIZE during a face.

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

This baseline uses one clock intentionally. It proves functional wiring only;
100/150/200 MHz CDC is a later gate and must not be inferred from this result.

## Run

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/scripts/run_smoke.ps1
```

Run the physical A/B/Z input path with its explicit scenario:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/scripts/run_smoke.ps1 `
  -Scenario system_integration/scenarios/smoke_external_200m_v001.json
```

`-EncoderSource` remains available for short experiments, but archived
`scenario.json` always contains the effective configuration after overrides.
