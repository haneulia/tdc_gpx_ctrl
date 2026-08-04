# v2 Frozen Baseline

## 1. Baseline Identity

| Item | Value |
|---|---|
| Baseline date | 2026-08-04 |
| TDC repository baseline | commit `6c95b23` |
| FPGA | `xc7z020clg484-2` |
| Routine processing/TDC clocks | 150 MHz / 200 MHz |
| Stream clock mode | asynchronous unless the clocks are the same physical clock |
| Output widths | 32, 64 and 128 bits |
| Maximum topology | 4 GPX chips, 8 STOPs/chip, 7 Returns/STOP |
| GPX mode | I-Mode single measurement |

The baseline commit is the golden source reference. Local uncommitted files
present when this document was created are not part of the frozen baseline:

- `.gitignore`;
- `tdc_gpx_chip_ctrl.vhd`;
- `NA/`;
- `tmp_parent_bd_audit.tcl`.

These files must not be staged by a v2 architecture commit.

## 2. Production RTL Size

Testbenches, generated IP copies and simulation archives are excluded.

| Block | VHDL files | Lines | Entities | Processes |
|---|---:|---:|---:|---:|
| `my_axil_csr32` | 7 | 1,642 | 5 | 12 |
| `motor_decoder` | 19 | 5,598 | 16 | 40 |
| `laser_ctrl` | 17 | 4,812 | 14 | 27 |
| `motor_laser_ctrl` wrapper | 1 | 494 | 1 | 0 |
| `echo_receiver` | 9 | 2,600 | 7 | 15 |
| `tdc_gpx_ctrl` | 32 | 20,523 | 29 | 114 |
| Unified CSR RTL | 3 | 1,061 | 2 | 0 |
| Integrated top | 1 | 811 | 1 | 0 |
| **Total** | **89** | **37,541** | **75** | **208** |

Additional structural indicators:

- `tdc_gpx_lidar_ctrl_top` has 53 generic declarations and 84 port declarations;
- `motor_laser_ctrl_top` has 32 generic declarations and 125 port declarations;
- `tdc_gpx_top` has 23 generic declarations and 115 port declarations;
- the four processing repositories contain 65 explicit XPM CDC/FIFO instances;
- Motor, Laser, Echo and TDC each retain local and unified CSR behavior;
- identical 535-line `px_utility_pkg.vhd` copies are manually present in
  multiple source repositories.

The line count is not itself a defect. The primary cost is repeated ownership:
configuration values are represented as generics, CSR words, staging values,
CDC snapshots, active values and readbacks, with separate state machines in
each IP.

## 3. Canonical Source Fingerprints

These hashes identify the source snapshots used for the architecture review.
They do not replace Git history.

| Source | SHA-256 |
|---|---|
| `tdc_gpx_lidar_ctrl_top.vhd` | `F8E8656E3F74217413ED24BE35058C28ACDDC05D17C9A2CB2811C73EF1F4D3E2` |
| `motor_laser_ctrl_top.vhd` | `4754EF743B722C1FBB0529329BF3C7B32FF8E9A9D14F3C125899F41B71C65AF3` |
| `motor_decoder_top.vhd` | `D17B8757748FCB58040444E318EC44E053F6E26206B94BB590B482FF5588CC60` |
| `laser_ctrl_top.vhd` | `669EF21EBA00EC848CBC8BC9A5C011A4FF00FE12D9E1282941A22ADB8D3AC477` |
| `echo_receiver_top.vhd` | `5013C4C9479EA8F566C566213150EB18C6AD3BE0E16885A5A96E71E390AA6A47` |
| `tdc_gpx_top.vhd` | `3F8CFE1726345F5DA51011952F4463DB235AB2B79750697ACC46A8609429EE09` |
| `lidar_unified_csr_top.vhd` | `B2BDCC808C2C8AB3B34D124598BB6962AA213CB0660038213FFCCB5BD3B616DA` |
| `lidar_unified_csr_pkg.vhd` | `4A5754EB538E37E41288404F16A24C17DE3CB5C5A7EFAF38FDA245AE8E1DF52D` |

## 4. Golden Functional Contract

The following behavior is preserved unless a later v2 decision record changes
it explicitly.

1. Motor input accepts physical A/B/Z or a build-enabled virtual source.
2. Face activity is based on decoded motor state and supports one to five
   polygon faces.
3. Physical `fire_pulse` is suppressed in simulation mode.
4. Physical `start_tdc` is generated only from synchronized `fire_done` after
   a physical fire command.
5. Simulation `start_tdc` is generated only by the simulation execution path.
6. Echo Receiver converts LVDS channels to physical GPX STOP signals without
   passing through CSR, AXIS buffering or the GPX processing pipeline.
7. GPX external data is 28 bits:

   - `[27:26]`: `ChaCode`;
   - `[25:18]`: `StartNum`;
   - `[17]`: slope;
   - `[16:0]`: 17-bit Hit.

8. The final cell preserves Hit `[15:0]` in hit storage and Hit `[16]` in
   metadata.
9. Output width changes beat packing only. It must not change canonical line
   bytes, Cell meaning, HSIZE or VSIZE.
10. Rise and Fall output streams preserve their own SOF/EOL and VDMA geometry.

## 5. Golden Comparison Points

v2 must be compared with v1 at these boundaries, in this order:

| Boundary | Comparison |
|---|---|
| B0 Encoder | decoded position, direction and Z/revolution event |
| B1 Face | active flag, face index, lower/upper traversal behavior |
| B2 Shot scheduler | accepted shot position, face and shot index |
| B3 Laser | `fire_pulse`, timeout, `start_tdc`, `stop_tdc` and measured latencies |
| B4 Echo | every physical STOP pulse per channel and edge |
| B5 GPX bus | every accepted 28-bit I-Mode word plus chip/IFIFO ordering |
| B6 Hit event | 17-bit Hit, chip, STOP, Return, slope and shot identity |
| B7 Cell | canonical bytes including Hit MSB metadata |
| B8 Frame | line order, header, SOF/EOL, HSIZE and VSIZE |
| B9 AXIS | byte-exact accepted output under bounded backpressure |

## 6. Preserved Evidence

The preserved v1 baseline regression remains AXIS 150 MHz / TDC 200 MHz.
Current v2 routine and release profiles are controlled by Section 3 of
`V2_MIGRATION_VERIFICATION_PLAN.md`; release CDC gates retain the previously
verified 4:1, 1:4 and synchronous 150/150 profiles.
The following coverage remains mandatory:

- 32/64/128-bit output widths;
- one-chip dual-edge, three-chip 2-rise/1-fall and four-chip 2-rise/2-fall;
- falling-disabled all-rise mode;
- all 16 APD channels and Return counts 1 through 7;
- CW/increasing and CCW/decreasing traversal;
- shot-boundary and bounded downstream backpressure;
- target range, RPM and angular-resolution feasibility boundaries.

## 7. Baseline Limits

The v1 RTL/package sign-off does not close these product-level items:

- final PCB I/O voltage and skew;
- GPX read-capture margin on the physical board;
- measured START/STOP and status-pin waveforms;
- sustained VDMA/DDR/Ethernet backpressure;
- product laser safety and fail-safe policy.

v2 must not claim board sign-off from RTL equivalence alone.
