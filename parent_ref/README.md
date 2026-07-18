# TDC GPX PS-FCLK Parent Reference

This directory creates a reproducible parent integration project for
`tdc_gpx_top`. It is a new reference design; it does not import the legacy TDC
logic from `VIRTUAL_TDC_TEST_V003`.

## Fixed integration profile

| Contract | Value |
|---|---|
| FPGA | `xc7z020clg484-2` |
| PS FCLK0 | 100 MHz, AXI-Lite control |
| PS FCLK1 | 150 MHz, event processing, AXIS, VDMA S2MM and HP clocks |
| PS FCLK2 | 200 MHz, TDC bus/control clock |
| TDC output width | 32 bits |
| Slope topology | `DEDICATED_2X2` |
| DDR writers | rise VDMA to HP0, fall VDMA to HP1 |

The parent uses two independent VDMA S2MM channels. Each accepts one 32-bit
TDC output stream and writes through a 64-bit PS HP port. Runtime HSIZE and
VSIZE are exposed to software through a read-only AXI GPIO snapshot.

## Address map

| Base | Function |
|---|---|
| `0x4000_0000` | TDC chip/control CSR |
| `0x4000_0800` | TDC pipeline CSR |
| `0x4120_0000` | packed rise/fall HSIZE and VSIZE snapshot |
| `0x4300_0000` | falling-lane VDMA registers |
| `0x4301_0000` | rising-lane VDMA registers |

## Usage

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File parent_ref/scripts/run_parent_ref.ps1 -Mode VALIDATE

powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File parent_ref/scripts/run_parent_ref.ps1 -Mode SYNTH

powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File parent_ref/scripts/run_parent_ref.ps1 -Mode IMPL
```

Every run recreates the project in `.tmp_parent/<stamp>`. The runner parses the
generated block-design JSON and exact-checks the part, actual FCLK values,
interface widths/domains, lane-to-VDMA-to-HP paths, and address map.

## Sign-off boundary

This is an integration and timing reference, not a board bitstream sign-off.
The following remain intentionally open until the target board is defined:

- PS7 board preset, DDR and MIO electrical configuration.
- TDC physical pin LOC, IOSTANDARD and board timing constraints.
- Real laser/echo producer integration; those inputs are idle-tied here.
- Software VDMA frame-buffer addresses, stride, HSIZE and VSIZE programming.

Do not treat implementation timing or DRC from this shell as board sign-off
until those four items are closed.
