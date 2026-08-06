# V2 Checkpoint J9: DDR Image versus HTML Golden

## Purpose

J9 proves that the target PACKED17 AXIS stream becomes the same DDR memory
image described by the executable C08 HTML oracle. It compares every allocated
32-bit Word, including the address gaps created by fixed STRIDE.

## Independent reference chain

```text
C08 HTML buildJ9Golden()
          |
          +--> checked Golden JSON
          |          |
          |          +--> HTML/JSON exact-object comparison
          |
RTL profile manager -> Footer builder -> AXIS width packer
                                             |
                                      VDMA write model
                                             |
                                      DDR capture HEX
                                             |
                                  address + Word comparison
```

The checked JSON is not regenerated during a normal regression. The verifier
first executes the model embedded in the HTML and requires exact equality with
the checked file. RTL output is then compared against that frozen file.

## Sign-off scenario

The focused vector deliberately uses the smallest legal Cell geometry because
it makes width-dependent alignment and Footer splitting visible:

- one active Rise Chip;
- one STOP;
- one visible Return;
- two planned geometric Shots;
- Shot 0 is valid PACKED17 data;
- Shot 1 is an explicit Hole with preserved CCW/simulation/last context;
- one ordered Face Footer with Face Frame ID `0x11223344` and Active Config
  version `0x1234`;
- DDR is initialized with `0xA5A5A5A5` before capture.

The VDMA write model writes only HSIZE Words at `line * STRIDE`. Therefore every
reserved Word must remain `0xA5A5A5A5`. This distinguishes valid Line padding,
which is transferred as zero, from the untransferred STRIDE reserve.

## Geometry and comparison result

| AXIS width | HSIZE | STRIDE | VSIZE | Compared | Preserved reserve |
|---:|---:|---:|---:|---:|---:|
| 32 bits | 24 B | 36 B | 4 Lines | 36 Words / 144 B | 12 Words |
| 64 bits | 24 B | 40 B | 4 Lines | 40 Words / 160 B | 16 Words |
| 128 bits | 32 B | 48 B | 3 Lines | 36 Words / 144 B | 12 Words |

The complete matrix passed at Processing 150 and 200 MHz. Captures at the two
clock frequencies have identical SHA-256 values for a given width, proving
that the memory ABI is clock-rate independent.

Archived session:

- `260807_j9_full_v2_gpx_ddr_golden`

Pass markers:

- `LIDAR_V2_PACKED17_HTML_GOLDEN_PASS`
- `LIDAR_V2_GPX_DDR_CAPTURE_PASS`
- `LIDAR_V2_GPX_DDR_GOLDEN_PASS`

## Artifacts

- HTML oracle:
  `C08_HDL_HTML_Alignment_260806_PACKED17_VDMA_ABI_Simulator_v026.html`
- Checked vector:
  `system_integration/v2/golden/packed17_j9_ddr_golden.json`
- HTML/vector verifier:
  `system_integration/v2/scripts/verify_packed17_html_golden.mjs`
- RTL capture test:
  `system_integration/v2/tb/tb_lidar_gpx_ddr_golden.vhd`
- Regression:
  `system_integration/v2/scripts/run_v2_gpx_ddr_golden.ps1`

## Gate decision

J9 is complete at the deterministic RTL memory-image boundary. This is valid
Sign-off evidence for Word order, width packing, HSIZE/VSIZE, Footer splitting,
fixed STRIDE addressing, SOF/TLAST, and reserved-address preservation.

It is not yet evidence for the physical AXI VDMA core, HP-port arbitration, or
CPU cache ownership. J10 now decodes these exact captures into byte-checked
H-Line/Ethernet packets with the portable C reference implementation. K0/K1
must first assemble and align the full v2 top; Stage 9 L0 then repeats the
ownership sequence with the parent VDMA and on the board.
