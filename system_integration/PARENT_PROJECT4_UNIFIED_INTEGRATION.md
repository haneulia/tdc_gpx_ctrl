# project_4 Unified LiDAR Candidate

## Purpose

The integration script creates `design_1_unified` beside the existing
`design_1`. The original block design remains the comparison baseline until
the candidate passes structural, synthesis, timing, and board-pin review.

## Candidate architecture

- one `motor_laser_ctrl_top`; its internal Virtual Encoder, Motor Decoder, and
  Laser Controller replace the three duplicated standalone IP instances;
- one physical-path `echo_receiver_top`;
- one `tdc_gpx_top` with 4 chips, 8 STOPs/chip, Return-7 capacity, 32-bit AXIS,
  split Rise/Fall masks `0011/1100`;
- one `lidar_unified_csr` at PS address `0x40000000`;
- AXI/CSR clock 50 MHz, processing/AXIS clock 150 MHz, TDC bus clock 200 MHz;
- PS FCLK1 remains 100 MHz and `proc_clk_wiz` generates the exact 150 MHz
  processing clock. A direct 150 MHz PS request resolves to about 142.857 MHz
  with this board preset and is therefore not used;
- direct Motor/Laser shot, face, and stop signals to Echo/TDC without an AXIS
  broadcaster, FIFO, or register slice;
- external Rise/Fall TDC result AXIS interfaces with full `TREADY` support.

The physical GPX START waveform is `o_start_tdc`. Echo Receiver consumes the
logical `o_shot_start` marker, not the physical START waveform. Echo Receiver
drives the 32 physical GPX STOP pins through `o_tdc_stop`.

## IP repository ownership

- TDC GPX IP: `C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/ip_repo`, VLNV
  `victek.co.kr:my_ip:tdc_gpx_top:1.0`;
- unified CSR IP: `C:/Projects/my_sp/lib/IP/lidar_unified_csr/ip_repo`, VLNV
  `victek.co.kr:my_ip:lidar_unified_csr:1.0`.

The unified CSR is intentionally a separate reusable IP. `tdc_gpx_top` only
exposes the `tdc_unified_csr` child interface when local CSR is disabled.

## Run

```powershell
& 'C:\AMDDesignTools\2025.2\Vivado\bin\vivado.bat' `
  -mode batch -nojournal `
  -source system_integration/scripts/integrate_project4_unified_candidate.tcl
```

The optional first Tcl argument may override the default parent directory:

```powershell
& 'C:\AMDDesignTools\2025.2\Vivado\bin\vivado.bat' `
  -mode batch -nojournal `
  -source system_integration/scripts/integrate_project4_unified_candidate.tcl `
  -tclargs 'C:\Projects\my_sp\ALINX\Logic\project_4'
```

Success markers are `PROJECT4_UNIFIED_CANDIDATE_PASS`, AXIS 150 MHz, TDC
200 MHz, and CSR base `0x40000000`.

Run the independent saved-design checker after generation:

```powershell
& 'C:\AMDDesignTools\2025.2\Vivado\bin\vivado.bat' `
  -mode batch -nolog -nojournal `
  -source system_integration/scripts/check_project4_unified_candidate.tcl `
  -tclargs 'C:\Projects\my_sp\ALINX\Logic\project_4'
```

Its success marker is `PROJECT4_UNIFIED_CANDIDATE_CHECK_PASS`.

## Deliberately open before board sign-off

- Map the new encoder, Laser FIRE, GPX START/control/data/flag pins to the
  AC7021B schematic and XDC. Existing Echo LVDS Bank 34/35 constraints remain.
- Connect `m_axis_tdc_rise` and, when enabled, `m_axis_tdc_fall` to the final
  VDMA/DDR path. They are external candidate interfaces only to keep the data
  sink decision explicit.
- Replace the temporary 81 ps/bin and Q16.16 unity calibration constants when
  board calibration ownership is finalized.
- Promote `design_1_unified` to the project top only after OOC synthesis and
  post-route timing close at 150/200 MHz.

The candidate is registered in the project but `design_1_wrapper` remains the
active top, so the validated baseline is not silently replaced.
