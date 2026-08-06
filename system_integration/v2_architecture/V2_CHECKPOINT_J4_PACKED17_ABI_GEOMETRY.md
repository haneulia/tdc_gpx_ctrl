# V2 Checkpoint J4 - PACKED17 ABI Geometry

## 1. Purpose

J4 turns the J3 HTML/contract oracle into shared synthesizable package rules.
It does not yet replace the J2 lane formatter. The legacy 48-byte prefix API is
kept temporarily so the existing formatter can remain a regression baseline
until J5/J6 migrate the stream.

## 2. Return ownership correction

The physical TDC-GPX IFIFO still drains to the EF-controlled completion point.
The runtime visible-Return setting only controls how many Hits are retained for
PACKED17 serialization.

- A physical Return above the runtime selection but within Return 1..7 is an
  intentional filter. It does not set `hit_dropped` or a fault.
- The eighth or later physical Return sets `return_overflow` and the Cell fault
  summary.
- `hit_dropped` remains reserved for unintended internal loss.

`return_overflow` is preserved through the B7 Cell record and the B8 packed
LUTRAM before becoming Cell Metadata bit 26.

## 3. Cell Metadata ABI

| Bits | Meaning |
|---:|---|
| 6:0 | Return 0..6 Hit bit 16 |
| 13:7 | Return 0..6 valid |
| 16:14 | Stored Hit count |
| 17 | Slope, 1 Rise / 0 Fall |
| 19:18 | Logical Chip index |
| 22:20 | STOP index |
| 23 | Blank Cell |
| 24 | Error-filled Cell |
| 25 | Unintended internal Hit drop |
| 26 | Eighth-or-later physical Return overflow |
| 27 | Cell/line fault summary |
| 30:28 | Timeout cause |
| 31 | Metadata valid marker |

The testbench computes this word independently from literal bit positions; it
does not call the production packing function for the expected value.

## 4. Target geometry functions

```text
BEAT_BYTES   = OUTPUT_WIDTH / 8
CELL_BYTES   = (ceil(VISIBLE_RETURNS / 2) + 1) * 4
RAW_HSIZE    = 16 + CELL_SLOTS * CELL_BYTES
HSIZE        = align_up(RAW_HSIZE, BEAT_BYTES)
FOOTER_LINES = ceil(32 / HSIZE)
VSIZE        = PLANNED_SHOTS + FOOTER_LINES
STRIDE_MAX   = align_up(16 + MAX_SLOTS * MAX_CELL_BYTES, BEAT_BYTES)
```

`STRIDE_MAX` and frame allocation remain fixed for a synthesized build.
Active HSIZE/VSIZE may change only at a Face boundary in J8.

## 5. Verified examples

| Profile | Expected | Result |
|---|---:|---|
| 16 Cells, Return 7, 32/64/128 bit | HSIZE 336 B | PASS |
| 16 Cells, Return 7, 32/64/128 bit | 84/42/21 Beats | PASS |
| 1 Cell, Return 1, 32/64 bit | HSIZE 24 B, Footer 2 Lines | PASS |
| 1 Cell, Return 1, 128 bit | HSIZE 32 B, Footer 1 Line | PASS |
| 32 Cells, Return 7 | STRIDE 656 B | PASS |
| 1,800 Shots, active HSIZE 336 B | VSIZE 1,801 | PASS |
| 1,800 maximum Shots, 32/64 bit | MAX_VSIZE 1,802 | PASS |
| 1,800 maximum Shots, 128 bit | MAX_VSIZE 1,801 | PASS |
| 32-bit maximum allocation | 1,182,112 B | PASS |

## 6. Regression evidence

| Regression | Clocks | Result |
|---|---|---|
| B7 Cell collector plus B6-B7 link | 150/200 MHz | PASS |
| B8 Frame-lane assembler and runtime slope masks | 150/200 MHz | PASS |
| B5-B8 dedicated and all-dual integration | 150/200 and 200/150 MHz | PASS |
| PACKED17 Cell-word serializer and target geometry | 150/200 MHz | PASS |
| Legacy J2 formatter, widths 32/64/128 | 150/200 MHz | PASS |

Implementation gates:

| Design | Clock profile | WNS | ASYNC FIFO | ASYNC_REG | Latch | Critical CDC | Blocking DRC |
|---|---|---:|---:|---:|---:|---:|---:|
| Cell-word serializer | 150 MHz | +3.206 ns | N/A | N/A | 0 | N/A | 0 |
| Cell-word serializer | 200 MHz | +1.752 ns | N/A | N/A | 0 | N/A | 0 |
| B5-B8 integration | PROC 150 / TDC 200 MHz | +0.309 ns | 3 | 230 | 0 | 0 | 0 |
| B5-B8 integration | PROC 200 / TDC 150 MHz | +0.420 ns | 3 | 230 | 0 | 0 | 0 |

Sessions:

- `260806_j4_contract_r2_v2_gpx_cell_collector`
- `260806_j4_contract_v2_gpx_cell_word_serializer`
- `260806_j4_contract_v2_gpx_frame_lane_assembler`
- `260806_j4_contract_v2_gpx_vdma_lane_formatter`
- `260806_j4_contract_v2_gpx_b5_b8_subsystem`
- `260806_j4_contract_impl_r2_v2_gpx_cell_word_serializer`
- `260806_j4_contract_impl_v2_gpx_b5_b8_subsystem`

## 7. Next gate

J5 must capture the TDC measurement start reference time (T0, Time Zero) and
the remaining Shot identity into four canonical 32-bit Shot Metadata words.
It must preserve one scheduled Shot per line, including explicit hole lines,
before J6 changes the physical AXIS packing.
