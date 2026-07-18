# C08 PS-FCLK Parent Reference Checkpoint

| Item | Value |
|---|---|
| FPGA | `xc7z020clg484-2` |
| Vivado | 2025.2.1 |
| Parent source | `parent_ref/scripts/create_parent_ref.tcl` |
| Parent wrapper | `parent_ref/rtl/tdc_gpx_parent_core.vhd` |
| Validation session | `260718200000_ps_fclk_parent_ref` |
| Result | `validate_bd_design` PASS |

## 1. Architecture decision

The parent project is newly generated. The project under
`C:/Sky/LiDAR/VIRTUAL_TDC_TEST_V003` was used only to confirm the PS7 FCLK,
reset and AXI integration pattern. No legacy TDC implementation was copied.

The fixed first integration profile is:

1. FCLK0 = 100 MHz for both TDC AXI-Lite CSR interfaces and VDMA control.
2. FCLK1 = 150 MHz for TDC event processing, both 32-bit AXIS outputs, VDMA
   S2MM data clocks and PS HP0/HP1 clocks.
3. FCLK2 = 200 MHz for the TDC bus/control clock.
4. `g_OUTPUT_WIDTH=32`, `g_SLOPE_CHIP_MODE=DEDICATED_2X2` and
   `g_STREAM_CLK_MODE=ASYNC`.
5. Rising and falling lanes use independent VDMA writers and independent PS
   HP ports so one lane cannot create memory-port arbitration coupling in the
   other lane.

## 2. Validated contract

The generated `.bd` was parsed as JSON after Vivado validation. Exact checks
confirmed:

| Check | Observed |
|---|---|
| Actual PS clocks | 100.000000 / 150.000000 / 200.000000 MHz |
| TDC output interfaces | 4 bytes per beat, 150 MHz, FCLK1 domain |
| Control clock association | `s_axi:s_axi_pipe`, 100 MHz, FCLK0 domain |
| TDC clock | 200 MHz, FCLK2 domain |
| Rising path | `m_axis -> vdma_rise -> HP0` |
| Falling path | `m_axis_fall -> vdma_fall -> HP1` |
| VDMA memory width | 64 bits per channel |
| VDMA frame stores | 3 per channel |

Validated addresses are `0x4000_0000` (chip CSR), `0x4000_0800` (pipeline
CSR), `0x4120_0000` (geometry), `0x4300_0000` (fall VDMA) and `0x4301_0000`
(rise VDMA). Both VDMA data address spaces map 512 MiB from address zero into
their corresponding PS HP DDR aperture.

## 3. Warning classification

No project-internal critical warning remains in the final validation. The
host Vivado installation reports a corrupted per-user XilinxTclStore catalog;
the runner explicitly supplies the installation Tcl package paths and Vivado
falls back to the installation catalog. This is an environment maintenance
item, not a parent-design failure.

Non-blocking project warnings remain for deprecated `xlconstant`/`xlconcat`
utility IP and direct GPIO input-pin wiring. These can be migrated to 2025.2
inline HDL after synthesis closure; they do not change the validated data or
clock contract.

## 4. Open sign-off items

This checkpoint proves parent connectivity, clock-domain intent and address
mapping. It is not board sign-off. The board PS7 preset/DDR/MIO configuration,
TDC LOC/IOSTANDARD constraints, real laser/echo producers and software VDMA
programming remain open. Synthesis is the next gate; implementation timing is
meaningful only as a reference until the board XDC and PS preset are supplied.
