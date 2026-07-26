# C08-S23 Output Width Generic Closure

Date: 2026-07-27

## Decision

`tdc_gpx_top.g_OUTPUT_WIDTH` is the single build-time contract for both VDMA
AXI4-Stream masters. No duplicate width generic and no runtime CSR control are
added.

The legal values are 32, 64, and 128 bits. The default remains 32 bits.
Changing this generic changes top-level port widths and packing geometry at
elaboration, so Vivado output products and the Block Design must be regenerated.

## Propagation Contract

| Boundary | Width derived from `g_OUTPUT_WIDTH` |
|---|---:|
| Rise/Fall `TDATA` | `g_OUTPUT_WIDTH` bits |
| Rise/Fall `TKEEP`, `TSTRB` | `g_OUTPUT_WIDTH / 8` bits |
| Cell pipe and output stage | 32, 64, or 128 bits |
| Face assembler, line packer, header inserter | same build-time width |
| CSR `HW_CONFIG` | reports the selected build width |

The IP-XACT user parameter is a three-value list. The XGUI also performs an
explicit legality check, and the RTL assertion remains the final elaboration
guard. A value such as 96 is rejected rather than silently treated as a valid
intermediate value.

## End-to-End Verification

The maintained integration matrix uses the external encoder path, external GPX
I-Mode model, AXIS 150 MHz, and TDC 200 MHz.

| Output width | HSIZE/line | Rise beats | Fall beats | Raw 28-bit checks | Rise/Fall 17-bit checks | Result |
|---:|---:|---:|---:|---:|---:|---|
| 32 bit | 96 B | 288 | 288 | 48 | 12 / 12 | PASS |
| 64 bit | 96 B | 144 | 144 | 48 | 12 / 12 | PASS |
| 128 bit | 96 B | 72 | 72 | 48 | 12 / 12 | PASS |

All three cases reconstructed the same final Hit value:

```text
expected = 31272
rise     = 31272
fall     = 31272
```

`STAT7=0x00000000` in every case. HSIZE remains 96 bytes while the accepted
beat count halves as width doubles. This confirms that a wider AXIS transfers
the same line in fewer clocks and does not increase DDR payload size.

Integration summary:

`sim_results/vivado_xsim/matrices/260727032600_output_width_matrix.csv`

## Packaged-IP Verification

The regenerated User Repository VLNV
`victek.co.kr:my_ip:tdc_gpx_top:1.0` was instantiated three times with
`CONFIG.g_OUTPUT_WIDTH=32/64/128` and synthesized out of context for
`xc7z020clg484-2`.

Each run completed at 100 percent with no black boxes. Netlist port checks
confirmed:

| Generic | Rise/Fall TDATA | Rise/Fall TKEEP/TSTRB |
|---:|---:|---:|
| 32 | 32 bits | 4 bits |
| 64 | 64 bits | 8 bits |
| 128 | 128 bits | 16 bits |

The package static checker also confirms the exact XGUI choice list, default
32-bit value, and all six output-port width dependencies.

## Operational Rule

Choose the width before synthesis. A downstream VDMA or AXI4-Stream Data Width
Converter must use the same interface width. Do not attempt to change this
contract through CSR while a bitstream is running.

The C08 HTML simulator may continue to sweep 32/64/128 as build scenarios. It
must keep the compact Ethernet payload independent of this internal AXIS width;
only AXIS serialization clocks change for an equal 96-byte line.
