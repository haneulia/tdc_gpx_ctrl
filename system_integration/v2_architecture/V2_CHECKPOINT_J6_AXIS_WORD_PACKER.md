# V2 Checkpoint J6: 32/64/128-bit AXIS Word Packer

## Purpose

J6 is the only block that knows the synthesis-time AXIS data width. Every
upstream ABI block remains a canonical 32-bit Word stream, so Cell, Shot and
Footer definitions do not fork into three RTL implementations.

## Packing rule

```text
canonical W0 -> TDATA[31:0]
canonical W1 -> TDATA[63:32]
canonical W2 -> TDATA[95:64]
canonical W3 -> TDATA[127:96]
```

Only lanes present in the selected 32/64/128-bit width exist. A partial final
Beat is zero-filled in its unused upper lanes. No padding is inserted between
Shot Metadata and Cells or between Cells.

`TKEEP` and `TSTRB` are all ones because VDMA HSIZE is aligned to the selected
Beat size. The final zeros are allocated HSIZE bytes, not invalid AXIS bytes.
`TUSER[0]` is asserted only on the first Beat of Shot index zero; `TLAST` is
asserted on the final Beat of every Line.

## Sequential structure

The block uses a small registered assembly register and a registered AXIS
holding register. Input ready is removed whenever the output Beat is stalled.
There is no combinational ready path from the upstream producer through an
unregistered output payload.

## Functional verification

Six combinations passed:

- Processing 150 MHz with 32, 64 and 128-bit output;
- Processing 200 MHz with 32, 64 and 128-bit output.

The stimulus uses a 9-Word Line. This proves:

- no padding at 32-bit;
- one zero Word (4 B) only in the last 64-bit Beat;
- three zero Words (12 B) only in the last 128-bit Beat;
- two consecutive Lines preserve order and SOF/TLAST;
- AXIS payload and sideband remain stable under backpressure;
- abort clears both a partial assembly and a held output Beat.

## Implementation evidence

| Clock | Width | WNS | Latches | Blocking DRC |
|---:|---:|---:|---:|---:|
| 150 MHz | 32 | `+4.270 ns` | 0 | 0 |
| 150 MHz | 64 | `+3.166 ns` | 0 | 0 |
| 150 MHz | 128 | `+2.933 ns` | 0 | 0 |
| 200 MHz | 32 | `+2.702 ns` | 0 | 0 |
| 200 MHz | 64 | `+1.596 ns` | 0 | 0 |
| 200 MHz | 128 | `+1.755 ns` | 0 | 0 |

Archived sessions:

- `260806_j6_packer_r1_v2_gpx_axis_word_packer`
- `260806_j6_packer_impl_150_v2_gpx_axis_word_packer`
- `260806_j6_packer_impl_200_v2_gpx_axis_word_packer`

## Remaining work

J7 must construct the 32-byte Face Footer as canonical Words before this
packer, split it into one or more HSIZE-sized Lines, and preserve the final
commit Word. J8 then owns Face-boundary HSIZE/VSIZE activation.
