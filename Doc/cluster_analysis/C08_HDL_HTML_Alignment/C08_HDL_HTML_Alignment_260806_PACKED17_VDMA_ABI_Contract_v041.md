# C08 PACKED17 VDMA ABI Contract v041

## 1. Purpose

This contract defines the target data layout from the external TDC-GPX I-Mode
reader through AXI4-Stream, VDMA, DDR, PS Ethernet repacking, and the Viewer.
It replaces the legacy repeated 48-byte line prefix with per-Shot metadata and
one ordered Face-completion footer.

This document is the source of truth for the C08 HTML oracle, RTL package
functions, DDR golden vectors, and the PS reference decoder. The current legacy
RTL remains valid until the migration checkpoints explicitly replace it.

## 2. Fixed and runtime ownership

| Owner | Field | Change boundary |
|---|---|---|
| Build Generic | AXIS width: 32, 64, or 128 bits | Synthesis |
| Build Generic | Present/Rise/Fall-capable chip masks | Synthesis |
| Build Generic | Maximum STOP and Return capacities | Synthesis |
| Active Config | Active chip mask | Face boundary |
| Active Config | Falling enable | Face boundary |
| Active Config | STOP count per chip | Face boundary |
| Active Config | Visible Return count, 1 to 7 | Face boundary |
| Active Config | Face window and Shot interval | Face boundary |

Runtime lane masks are:

```text
falling_enable = 0:
    rise_mask = active_chip_mask
    fall_mask = 0

falling_enable = 1:
    rise_mask = active_chip_mask AND build_rise_mask
    fall_mask = active_chip_mask AND build_fall_mask
```

Overlapping build masks permit the same chip to contribute to both independent
Slope lanes. Cell order in each lane is logical chip index ascending, then STOP
index ascending.

## 3. External Return policy

The physical TDC-GPX IFIFO always drains to the EF-controlled completion
boundary. Runtime Return count only selects how many Returns are serialized.

- Returns above the selected count but within the physical capacity of seven
  are intentional filtering, not a fault.
- An eighth physical Return is `return_overflow`.
- `hit_dropped` is reserved for unintended internal loss.
- PACKED17 is the only DDR Hit ABI. PACKED18-20 and U32-per-Hit are excluded.

## 4. Canonical Cell

PACKED17 is a sequence of canonical 32-bit words independent of AXIS width.

```text
Hit Word k: [31:16] Return (2k+1) low 16 bits
            [15:0]  Return (2k)   low 16 bits

Final Word: Cell Metadata, including each Return Hit[16] and Valid bit.
```

```text
CELL_WORDS = ceil(visible_returns / 2) + 1
CELL_BYTES = CELL_WORDS * 4
```

| Visible Returns | Cell words | Cell bytes |
|---:|---:|---:|
| 1-2 | 2 | 8 |
| 3-4 | 3 | 12 |
| 5-6 | 4 | 16 |
| 7 | 5 | 20 |

The Cell Metadata word is:

| Bits | Meaning |
|---:|---|
| `[6:0]` | Return 0..6 Hit[16] |
| `[13:7]` | Return 0..6 Valid |
| `[16:14]` | Stored Hit count, 0..7 |
| `[17]` | Slope, 1 Rise / 0 Fall |
| `[19:18]` | Logical chip index |
| `[22:20]` | STOP index |
| `[23]` | Blank Cell |
| `[24]` | Error-filled Cell |
| `[25]` | Unintended internal Hit drop |
| `[26]` | Physical Return overflow, eighth or later Return |
| `[27]` | Cell fault summary |
| `[30:28]` | Timeout cause |
| `[31]` | Metadata valid marker |

## 5. Shot Line

Every scheduled geometric Shot occupies one VDMA line, including a missed Shot.
A missed Shot is emitted as an invalid/hole line so later columns never shift.

The Hole Line encoding is exact and width-independent:

| Field | Hole value |
|---|---|
| W0/W1 | Zero; no fabricated T0 |
| W2 `[15:0]` | Missing geometric Shot index |
| W2 `[31:16]` | `0xFFFF`, meaning encoder position was not measured |
| W3 valid/hole | `0/1` |
| W3 direction/source/last | Preserved from Face geometry |
| Cell area | All zero, including Cell Metadata valid bit |
| Line size | Identical to a real Line in the same active Face profile |

A Hole does not become a fault merely because it is explicit. A separate
geometry, timeout or abort cause sets the corresponding fault bit. Leading and
interior counts are expanded before the next real Shot; trailing and all-Hole
counts are expanded before the Face Footer.

```text
Shot Line = 16-byte Shot Metadata
          + lane_cell_slots * CELL_BYTES
          + final AXIS-beat padding only
```

The 16-byte Shot Metadata is four canonical words:

| Word | Meaning |
|---:|---|
| W0 | TDC measurement start reference time (T0) tick `[31:0]` |
| W1 | T0 tick `[63:32]` |
| W2 | Shot index `[15:0]`, encoder position `[31:16]`; a Hole uses position `0xFFFF` |
| W3 | Exact Shot status and context flags defined below |

W3 is frozen as follows. All unlisted bits are reserved and zero.

| Bit | Meaning |
|---:|---|
| `[0]` | Data valid; zero for a Hole Line |
| `[1]` | Hole Line |
| `[2]` | Direction is CCW |
| `[3]` | Simulation source |
| `[4]` | Shot timeout |
| `[5]` | Shot aborted |
| `[6]` | Line fault summary |
| `[7]` | TDC measurement start reference time (T0) timestamp valid |
| `[8]` | External time synchronization valid |
| `[9]` | Last planned Shot in Face |
| `[10]` | Position-source latency valid |
| `[18:11]` | Position-source latency in Processing clocks |

The timestamp is a 64-bit monotonic Processing-domain tick associated with the
actual TDC measurement start reference event (T0). Physical `start_tdc` keeps
its asynchronous low-latency path; the timestamp is sampled on the first
Processing-clock edge that observes that event. It is therefore ordered and
monotonic, but quantized to one Processing-clock period. Simulation T0 is
captured directly on its registered Processing-clock event. Until an external
PPS/PTP discipline source is implemented, external-time-sync-valid is zero.
The Active/Build Config must publish the tick frequency so PS can convert local
ticks to time. Hole Lines contain zero in W0/W1, invalid T0, and no fabricated
encoder position.

## 6. Face Footer

There is no PL Face Header at the beginning of the VDMA frame. A 32-byte Face
Footer is emitted after every planned Shot line and is valid only after Face
completion.

| Word / bits | Exact meaning |
|---:|---|
| W0 `[31:0]` | `0x47504631` (`GPF1`): GPX Face Footer, ABI version 1 |
| W1 `[31:0]` | Face Frame ID. It increments once for every accepted Face entry, wraps naturally at 32 bits, and resets only with the Processing reset |
| W2 `[2:0]` | Face index, 0 to 4 |
| W2 `[3]` | Slope lane, 1 Rise / 0 Fall |
| W2 `[4]` | Direction, 1 CCW / 0 CW |
| W2 `[5]` | Source, 1 simulation / 0 physical |
| W2 `[7:6]` | AXIS width code: 0=32, 1=64, 2=128 bits; 3 is reserved |
| W2 `[31:8]` | Reserved, zero |
| W3 `[15:0]` | Active Config version used by every Shot in this Face |
| W3 `[31:16]` | Reserved, zero |
| W4 `[15:0]` | Planned geometric Shot count |
| W4 `[21:16]` | Cell slots in each Shot Line |
| W4 `[24:22]` | Visible Return count, 1 to 7 |
| W4 `[31:25]` | Reserved, zero |
| W5 `[15:0]` | Active HSIZE in bytes |
| W5 `[31:16]` | Active VSIZE in lines, including Footer Line(s) |
| W6 `[15:0]` | Actually completed Shot/Hole Line count |
| W6 `[16]` | Face fault summary from the Face-close owner |
| W6 `[17]` | Completed count differs from planned count |
| W6 `[18]` | At least one Shot Line fault |
| W6 `[19]` | At least one explicit Hole Line |
| W6 `[20]` | At least one Shot timeout |
| W6 `[21]` | At least one aborted Shot |
| W6 `[22]` | Every planned Shot was a Hole |
| W6 `[31:23]` | Reserved, zero |
| W7 `[31:0]` | `0x434F4D54` (`COMT`): ordered Face completion commit |

PS discards a buffer with a missing/invalid commit marker. W7 is the final
logical Footer Word at Footer byte offset 28; zero HSIZE padding may follow it
before the final Footer Line ends. Static geometry and calibration are not
repeated in every Face; PS resolves them by Active Config version. PS creates
the Viewer Face Header from that config and this Footer.

## 7. VDMA geometry

For each independent Rise/Fall lane:

```text
BEAT_BYTES       = output_width / 8
CELL_SLOTS       = popcount(runtime_lane_mask) * stops_per_chip
RAW_HSIZE        = 16 + CELL_SLOTS * CELL_BYTES
HSIZE            = align_up(RAW_HSIZE, BEAT_BYTES)
FOOTER_LINES     = ceil(32 / HSIZE)
VSIZE            = planned_shot_count + FOOTER_LINES
STRIDE_MAX       = align_up(max_legal_RAW_HSIZE, BEAT_BYTES)
FRAME_ALLOCATION = STRIDE_MAX * MAX_VSIZE
LINE_ADDRESS     = frame_base + line_index * STRIDE_MAX
```

HSIZE and VSIZE are reprogrammed atomically at a Face boundary. STRIDE, frame
base addresses, and buffer allocations remain fixed at their synthesis maximum.
The gap between active HSIZE and STRIDE reserves address space but consumes no
VDMA payload bandwidth.

The formatter does not force HSIZE to 32 bytes merely to fit the Footer. The
normal two-or-more-Cell case uses one Footer line. A one-Cell, low-Return 32/64
bit profile may use two Footer lines; 128-bit alignment makes it one line.

## 8. Width transport

The canonical word order never changes:

| AXIS width | Canonical words per Beat |
|---:|---:|
| 32 | 1 |
| 64 | 2 |
| 128 | 4 |

Cells pack continuously across Beat boundaries. There is no per-Cell padding.
Only the final Beat of a Shot/Footer line may contain padding. VDMA Stream and
Memory Map widths match. Zynq-7000 uses an external SmartConnect to convert a
128-bit VDMA memory path to the 64-bit HP port when that build is selected.

Canonical Words occupy increasing AXIS lanes: the first Word is
`TDATA[31:0]`, the second is `TDATA[63:32]`, and so on. At 32-bit width no
transport padding is possible. At 64-bit width the final Beat may contain one
zero Word; at 128-bit width it may contain one to three zero Words. Because
HSIZE is already aligned to the selected Beat size, `TKEEP` and `TSTRB` are all
ones. The zero bytes are part of the VDMA Line allocation, not omitted AXIS
bytes. `TUSER[0]` marks the first Beat of geometric Shot index zero and `TLAST`
marks the final Beat of every Shot, Hole, or Footer Line.

## 9. Atomic reconfiguration

```text
CSR request -> validate pending profile -> finish current Face
-> wait for TDC/AXIS/VDMA idle -> program HSIZE/VSIZE
-> increment Active Config version -> start next Face
```

The finest supported angular interval and widest Face window define MAX_VSIZE.
Any request exceeding the allocated maximum is rejected before commit.

## 10. Sign-off evidence

1. HTML self-tests cover widths 32/64/128, Returns 1-7, minimum and maximum
   Cell slots, Rise-only, dedicated Rise/Fall, and overlapping dual-edge masks.
2. HTML exports deterministic canonical Word and DDR byte Golden Vectors.
3. XSIM captures accepted AXIS bytes into a DDR-memory image and compares every
   Word against the same Golden Vector schema.
4. A host PS reference decoder reads that image using HSIZE, VSIZE, and fixed
   STRIDE, then emits deterministic H-Line packets for byte comparison.
5. Board Sign-off repeats the decode after DMA cache ownership synchronization.
   FreeRTOS uses explicit invalidate/ownership barriers; PetaLinux uses the DMA
   API rather than direct cached-buffer access.

## 11. Migration checkpoints

| Checkpoint | Closure |
|---|---|
| J3 | HTML oracle and Golden Vector schema |
| J4 | ABI package geometry and Cell Metadata map |
| J5 | Shot Metadata builder and width-independent word stream |
| J6 | 32/64/128 width packer with final-Beat handling |
| J7 | Face Footer and fixed maximum STRIDE |
| J8 | Face-boundary VDMA reconfiguration handshake |
| J9 | DDR image versus Golden Vector comparison |
| J10 | PS H-Line reference decoder and Ethernet packet comparison |
| J11 | Parent Vivado, implementation timing, and board cache/DMA Sign-off |
