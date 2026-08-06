# V2 Checkpoint J5A: T0 and Shot Metadata

## Scope

J5A closes the real-Shot half of the target PACKED17 line contract:

1. associate every accepted Shot with a 64-bit TDC measurement start
   reference time (T0);
2. preserve T0 and the complete Shot context across Processing-to-TDC CDC;
3. prepend four canonical 32-bit Shot Metadata words to every serialized Cell
   line;
4. keep this boundary independent of the final 32/64/128-bit AXIS width.

Hole expansion is intentionally not claimed by J5A. Leading, interior,
trailing and all-Hole Lines remain J5B work.

## Data flow

```text
physical fire_done
  -> asynchronous low-latency start_tdc
  -> first Processing clock observing T0 captures local 64-bit ticks
  -> shot_start_event_t
  -> atomic Processing/TDC event gateway
  -> Hit -> Cell -> Frame Cell
  -> 32-bit Cell word serializer
  -> Shot-Line builder
       W0 T0[31:0]
       W1 T0[63:32]
       W2 position[31:16] + Shot index[15:0]
       W3 exact status/context bits
       remaining words: continuous Cell words
```

Simulation T0 is already synchronous and captures the local counter on the
registered event. Physical T0 keeps the safety-critical asynchronous START
path unchanged; its metadata timestamp is quantized to the first Processing
clock that observes START. No unsafe 64-bit asynchronous latch is introduced.

## Exact W3 map

| Bit | Meaning |
|---:|---|
| 0 | Data valid |
| 1 | Hole |
| 2 | CCW direction |
| 3 | Simulation source |
| 4 | Timeout |
| 5 | Aborted |
| 6 | Line faulted |
| 7 | T0 timestamp valid |
| 8 | External time synchronization valid |
| 9 | Last Shot in Face |
| 10 | Source latency valid |
| 18:11 | Source latency in Processing clocks |
| 31:19 | Reserved, zero |

External time synchronization remains invalid until a PPS/PTP discipline
source is implemented. The current T0 tick unit is one Processing clock and
must be published to PS by the completed configuration/status contract.

## Verification evidence

| Evidence | Result |
|---|---|
| Shot-Line functional, 150/200 MHz | PASS; metadata, Cell order, backpressure hold and abort |
| Laser/T0 functional, 150/200 MHz | PASS; physical and simulation timestamp validity |
| Laser implementation, 150 MHz | WNS `+1.010 ns`, latch 0, Critical CDC 0 |
| Laser implementation, 200 MHz | WNS `+0.428 ns`, latch 0, Critical CDC 0 |
| Event CDC functional | PASS at async 150->200, async 200->150 and sync 150 |
| Event CDC implementation 150->200 | WNS `+0.531 ns`, latch 0 |
| Event CDC implementation 200->150 | WNS `+0.701 ns`, latch 0 |
| Event path sync 150 | WNS `+4.054 ns`, latch 0, no async FIFO |
| Existing B5-B8 chain | PASS at 200/150 MHz including four-Chip dual-edge |
| Frame-lane regression | PASS at 150/200 MHz for dedicated, fall-off, dual-edge, four-Chip dual-edge and fault cases |

Archived sessions:

- `260806_j5_line_r3_v2_gpx_shot_line_builder`
- `260806_j5_line_impl_r2_v2_gpx_shot_line_builder`
- `260806_j5_t0_r2_v2_laser_executor`
- `260806_j5_t0_impl_r2_v2_laser_executor`
- `260806_j5_t0_cdc_v2_gpx_event_gateway`
- `260806_j5_t0_cdc_impl_v2_gpx_event_gateway`
- `260806_j5_t0_chain_v2_gpx_b5_b8_subsystem`
- `260806_j5_t0_lane_v2_gpx_frame_lane_assembler`

## Open work

1. J5B expands every missing geometric Shot into an equal-size Hole Line.
2. J6 packs the canonical words into 32/64/128-bit beats and pads only the
   final beat.
3. J7 emits the 32-byte Face Footer.
4. J8 applies geometry changes only at a Face boundary.
5. J9/J10 compare the DDR image and PS/Ethernet output against the HTML Golden
   Vector. Stage 9 L0 supplies the final board DMA/cache evidence after K0/K1.
