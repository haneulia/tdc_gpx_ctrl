# V2 Checkpoint J7/J8: VDMA Profile and Face Footer

## Purpose

J7/J8 closes the target Face-frame geometry boundary without placing runtime
division, modulo, or multiplication in the AXIS streaming path.

- J7 passes every Shot/Hole Line and appends one ordered 32-byte Face Footer.
- J8 calculates a Pending lane profile over multiple clocks.
- Active HSIZE/VSIZE changes only after a safe Face boundary and a VDMA
  programming acknowledgement.
- STRIDE remains fixed at the synthesis-time maximum for the lane.

## Ownership and data flow

```text
Runtime lane mask / Returns / planned Shots
                    |
                    v
      sequential VDMA profile manager
      count Chips -> accumulate Cell words -> align
                    |
                    v
              Pending profile
                    |
         Face boundary + datapath idle
                    |
                    v
       VDMA HSIZE/VSIZE/STRIDE request
                    |
             configuration ACK
                    |
                    v
               Active profile
             /                \
            v                  v
 Shot/Hole geometry check   Footer formatter
                                  |
                                  v
                8 canonical Footer Words + zero line padding
                                  |
                                  v
                    32/64/128-bit AXIS packer
```

The Active profile is one registered record shared by the VDMA programmer and
Footer formatter. A rejected or aborted Pending request never changes Active.

## Sequential profile calculation

The manager performs these operations in order:

1. validate the lane mask, visible Return count, and nonzero planned Shots;
2. inspect one possible Chip bit per clock;
3. accumulate one Cell slot per clock;
4. align the Word count with fixed low-bit rules for 32/64/128-bit builds;
5. derive one or two Footer Lines and VSIZE;
6. wait for `activate_valid` and `datapath_idle`;
7. hold the VDMA request stable until `vdma_cfg_ready`;
8. replace Active atomically.

The data path therefore contains registered counters, comparisons, and fixed
bit alignment only. The original dynamic geometry expression in the Footer
stream caused 39 logic levels and `-11.360 ns` at 200 MHz/128-bit. The linked
J7/J8 path now has positive timing margin in every required profile.

## Footer contract

The exact W0..W7 bit map is defined in
`C08_HDL_HTML_Alignment_260806_PACKED17_VDMA_ABI_Contract_v041.md` and generated
by `fn_gpx_vdma_footer_word`.

- W0 is `GPF1` and W7 is `COMT`.
- W1 is the monotonic 32-bit Face Frame ID allocated at accepted Face entry.
- W2 identifies Face, Slope, direction, source, and synthesis-time AXIS width.
- W3 identifies the immutable Active Config used by the Face.
- W4/W5 freeze planned geometry and active VDMA geometry.
- W6 records completed Line count and Face summary diagnostics.
- W7 is at logical Footer byte offset 28. HSIZE padding after W7 is zero.

The final accepted Footer Beat asserts Frame done. A Shot/Hole Line can never
assert Frame done.

## Functional verification

The J7/J8 regression passed at Processing 150 and 200 MHz for 32, 64, and
128-bit AXIS widths. It covers:

- minimum geometry: one Chip, one STOP, one Return;
- maximum Rise geometry and seven Returns;
- dedicated Fall lane, invalid Fall mask, and disabled Fall lane;
- invalid Return/Shot requests;
- safe-boundary waiting and VDMA programming backpressure;
- Active-profile immutability before acknowledgement;
- abort cancellation while preserving the previous Active profile;
- one-Line and two-Line Footer layouts;
- AXIS backpressure, Footer padding, TLAST, SOF, and Frame done;
- Face-close fork delivery to independent Rise/Fall consumers.

Focused sessions:

- `260806_j78_functional_all_v2_gpx_face_footer`
- `260806_j78_stop_range_v2_config_pkg`
- `260806_j78_packer_recheck_v2_gpx_axis_word_packer`
- `260806_j78_b5_b8_recheck_v2_gpx_b5_b8_subsystem`

## Implementation evidence

Target device: `xc7z020clg484-2`.

| Processing clock | AXIS width | Post-route WNS | Latches | Blocking DRC |
|---:|---:|---:|---:|---:|
| 150 MHz | 32 | `+2.152 ns` | 0 | 0 |
| 150 MHz | 64 | `+1.132 ns` | 0 | 0 |
| 150 MHz | 128 | `+1.079 ns` | 0 | 0 |
| 200 MHz | 32 | `+0.620 ns` | 0 | 0 |
| 200 MHz | 64 | `+0.489 ns` | 0 | 0 |
| 200 MHz | 128 | `+0.553 ns` | 0 | 0 |

The implementation wrapper contains the profile manager and Footer formatter,
so these figures include the registered Pending-to-Active path rather than a
Footer-only constant geometry shortcut.

Vivado reports the expected out-of-context `HD.CLK_SRC` warning because the
clock-buffer placement belongs to the parent design. The local Tcl Store
catalog warning is an installation-state issue; the regression explicitly
loads the installation Tcl Store and finishes without RTL errors.

## Gate decision

J7 and J8 are complete at focused RTL/OOC scope. Stage 7 is not yet signed off:

1. J9 must capture accepted AXIS bytes into a STRIDE-aware DDR image and compare
   every byte/Word with the checked-in HTML Golden Vector.
2. J10 must decode that same image into deterministic H-Line/Ethernet packets.
3. J11 must repeat timing and DMA/cache ownership checks in the parent design
   and on the board.
