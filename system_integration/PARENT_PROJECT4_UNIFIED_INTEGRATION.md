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

Run synthesis in an isolated temporary project without changing the active
parent top:

```powershell
& 'C:\AMDDesignTools\2025.2\Vivado\bin\vivado.bat' `
  -mode batch -nolog -nojournal `
  -source system_integration/scripts/synth_project4_unified_candidate.tcl `
  -tclargs 'C:\Projects\my_sp\ALINX\Logic\project_4'
```

The flow checks that synthesis completes and that 150 MHz and 200 MHz clocks
exist in the synthesized design. It also applies
`constraints/project4_unified_clock_domains.xdc` and produces a detailed CDC
report. The 50 MHz CSR, 150 MHz processing/AXIS, and 200 MHz TDC domains are
declared asynchronous because their RTL boundaries use XPM handshakes or
explicit `ASYNC_REG` synchronizers. FCLK1 and its generated 150 MHz clock stay
in one synchronous group. Its success markers are
`PROJECT4_UNIFIED_OOC_SYNTH_PASS` and
`PROJECT4_UNIFIED_OOC_TEMP_CLEAN_PASS`. The temporary project is deleted only
after a successful run; a failed run remains at
`C:/tmp/project4_unified_ooc` for diagnosis.

The OOC flow copies the candidate BD before generation. Vivado may migrate or
save the disposable copy, but it cannot rewrite the parent candidate. The
worst synthesized timing path is printed between
`PROJECT4_UNIFIED_OOC_WORST_PATH_BEGIN/END` for timing triage.
CDC rule/count rows are printed between
`PROJECT4_UNIFIED_OOC_CDC_SUMMARY_BEGIN/END`; timing closure is accepted only
after this CDC summary is reviewed for unsafe or unknown crossings.

The parent project stores this XDC in the dedicated
`design_1_unified_constrs` constraint set. The active baseline `synth_1` run
continues to use `constrs_1`, so the candidate's generated-clock constraint
cannot change baseline timing. When the candidate is promoted, its synthesis
and implementation runs must select `design_1_unified_constrs` and include the
final board-pin XDC files.

The current candidate permits `CDC-15` only when every such row belongs to the
four TDC raw-data XPM asynchronous FIFOs. Any other non-Info CDC rule, any
CDC-15 outside `xpm_fifo_base_inst`, or a negative synthesized WNS fails the
OOC flow and preserves the temporary project for diagnosis.

## Verified result (2026-07-29)

The current packaged IP and saved parent candidate passed the following
closure sequence with AXIS at 150 MHz and TDC processing at 200 MHz:

- `tb_tdc_gpx_chip_ctrl`: all 21 test groups passed, including output-cap
  truncation, full IFIFO depth, burst reads, purge, timeout, and recovery;
- `tb_tdc_gpx_top_int`: Rise/Fall each produced 24 beats and two `TLAST`
  markers with no physical GPX words left unread;
- TDC IP package source-sync, IP-XACT/XGUI, local/unified CSR visibility, and
  32/64/128-bit width validation passed;
- saved `design_1_unified` structure, clocks, masks, pin widths, direct
  shot/stop paths, unified CSR address, and candidate-only constraint set
  passed the independent checker;
- isolated synthesis completed with overall WNS `+0.096 ns`. The reported
  worst path is a 150 MHz Motor/Laser path, so the complete analyzed design,
  including the 200 MHz TDC paths, meets synthesized setup timing;
- CDC summary: `CDC-3 Info = 140`, `CDC-9 Info = 1`, and
  `CDC-15 Warning = 160`. Every CDC-15 detail row belongs to an
  `xpm_fifo_base_inst`; no other non-Info CDC rule was accepted.

The former 200 MHz TDC critical path calculated `cap - drained_count` and then
compared the result while planning a GPX burst. `tdc_gpx_chip_run` now stores
the equivalent per-IFIFO remaining cap budget and selects
`min(fill, cap_remaining)`. This removes the subtract/compare chain without
adding a register stage or changing the cap, purge, or output ordering
contract.

This is RTL/package/structural/OOC synthesis closure, not final board
sign-off. External board inputs have no OOC input-delay model, and the final
Rise/Fall AXIS sink, board pin constraints, calibrated distance constants,
and post-route timing still belong to the promotion checklist below.

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
