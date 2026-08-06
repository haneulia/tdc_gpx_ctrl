# V2 Checkpoint J5B: Explicit Hole Lines

## Purpose

Every planned geometric Shot must occupy one fixed VDMA line. Without this
rule, one missing Shot shifts every later column and PS can no longer map DDR
row number directly to horizontal Shot index.

J5B consumes the gap ownership already established by B8:

- `gap_before` on the next real line represents leading or interior holes;
- `trailing_gap` on Face close represents final missing columns;
- `all_hole` states that no real Shot exists from which geometry can be
  inferred.

## Encoding

Each Hole has exactly the active Line word count:

```text
4 Shot Metadata words + active Cell slots * active Cell words
```

The encoding is:

- T0 W0/W1: zero and invalid;
- W2 position: `0xFFFF`, Shot index: the missing geometric index;
- W3: data-valid zero, Hole one, Face direction/source/last preserved;
- Cell words: all zero, including Cell Metadata valid;
- `gap_before`: cleared because every gap has become an explicit Line.

An ordinary Hole is not automatically a fault. If Face-close geometry is
already faulted, that cause is preserved separately.

## Module boundary

`lidar_gpx_hole_line_expander` is one slope-lane block:

```text
Shot-Line builder real words ------+
                                    +--> explicit real/Hole word stream
B8 Face close + active geometry ---+--> ordered Face close
```

The ordered Face-close output is not released until the final trailing Hole
word is accepted. AXIS width, final-Beat padding and Footer creation remain
J6/J7 ownership.

## Verification

Functional tests at both 150 and 200 MHz cover:

1. two leading holes followed by a real Line;
2. trailing holes followed by ordered Face close;
3. a Face consisting entirely of holes;
4. zero-gap Face close with no invented Line;
5. output stability under backpressure;
6. abort while a Hole Line is pending;
7. unchanged real-Shot Metadata behavior after the shared package update.

Implementation evidence:

| Clock | WNS | Latches | Blocking DRC |
|---:|---:|---:|---:|
| 150 MHz | `+1.896 ns` | 0 | 0 |
| 200 MHz | `+0.574 ns` | 0 | 0 |

Archived sessions:

- `260806_j5b_hole_r3_v2_gpx_hole_line_expander`
- `260806_j5b_hole_impl_v2_gpx_hole_line_expander`
- `260806_j5b_meta_reg_v2_gpx_shot_line_builder`

## Remaining boundary work

1. J6 combines canonical 32-bit words into 32/64/128-bit AXIS beats.
2. Only the final beat may contain zero padding; no Cell-internal padding is
   permitted.
3. J7 creates the Face Footer and fans one ordered B8 Face close to the active
   Rise/Fall lane formatters.
4. J8 applies active lane/Return geometry changes only at a Face boundary.
