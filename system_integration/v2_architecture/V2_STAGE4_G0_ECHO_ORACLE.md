# Stage 4 G0 Echo B4 Oracle

## 1. Scope

This document freezes the v1 Echo behavior that Checkpoint G must preserve.
It covers only the LVDS-to-GPX STOP boundary, optional synthetic Echo source,
compact delay profile and observation diagnostics. GPX bus reads, I-Mode word
decoding, Hit ordering and VDMA formatting remain Stage 5 and later work.

## 2. Reviewed v1 Sources

| Source | SHA-256 |
|---|---|
| `echo_receiver_stop_frontend.vhd` | `0A68BED7E6FDEDA81B0C6437EDB2958F94F858DE83ED34EDD3505D708E9B4850` |
| `echo_receiver_core.vhd` | `E68CE57AC3D3FB363E259415C92CD32DAB872F6AC6359779EA0C6EBCF37092B6` |
| `echo_receiver_top.vhd` | `5013C4C9479EA8F566C566213150EB18C6AD3BE0E16885A5A96E71E390AA6A47` |
| `echo_receiver_pkg.vhd` | `0DDD4565E6BD6CBA87225D528E18F0ED85FBBF84FA7FCC748F423E1B6535A6E3` |

The canonical v1 source is
`C:/Projects/my_sp/lib/IP/echo_receiver/HDL`. Generated IP-repository copies
are not oracle sources.

## 3. Frozen B4 Physical Contract

```text
LVDS P/N[channel]
    -> IBUFDS(LVDS_25, DIFF_TERM=TRUE, IBUF_LOW_PWR=FALSE)
    -> GPX STOP[channel]
```

- The production path has no Processing-clock register, window gate, CSR
  gate, hit limiter, FIFO or AXIS dependency.
- Both waveform transitions are preserved. GPX Reg0 and the build slope masks
  decide which edge is measured; Echo does not assign Rise/Fall ownership.
- The path latency is I/O and routing propagation, not an integer Processing
  clock count. Board delay calibration must include the implemented path.
- A 1.5 ns legal physical pulse must reach the STOP output even when the
  clock-sampled diagnostic observer misses it.
- `shot_start`, `shot_result`, profile activity and monitor readiness never
  gate the physical waveform.

## 4. Channel Order

The only channel mapping is:

```text
flat_channel = chip_index * stops_per_chip + stop_index
```

The implementation is parameterized by the build topology. Checkpoint G
tests both the required 16-channel APD profile and the maximum 4 chip x 8 STOP
= 32 STOP-pin profile. A dedicated Rise/Fall board may route one APD waveform
to more than one GPX STOP pin; that duplication belongs to the board wrapper,
not to Echo channel arithmetic.

## 5. Return Contract

Return 1 through Return 7 means one through seven physical pulses on the same
STOP channel during one accepted Shot. The frontend passes every pulse and
does not infer GPX IFIFO occupancy. Diagnostic counts are pre-GPX observations
only and may not terminate or bound acquisition.

## 6. Optional Simulation Contract

- `enable_echo_receiver=false` removes the IBUFDS frontend, diagnostic
  observer and simulation source. STOP and diagnostics are then inactive;
  `profile_ready` and `idle` stay asserted so the absent block cannot stall a
  parent safe-point. This is a structural build option, not a runtime CSR gate.
- `enable_echo_simulation=true` is legal only when
  `enable_echo_receiver=true`. A production build contains neither the delay
  expansion logic nor the synthetic source mux.
- CTL20 stores only two 16-bit values in fixed 5 ns ticks:

  ```text
  delay_ticks(channel) = CH0_DELAY + channel * CHANNEL_STEP
  ```

- The compact profile supports the complete 1..32-channel topology without a
  32-entry CSR table. The maximum expanded value is kept at 32 bits so
  `CH0 + 31 * STEP` cannot overflow a 16-bit intermediate.
- On each accepted active configuration version, one channel is expanded and
  converted per Processing clock. A 32-channel profile is therefore ready no
  later than 32 Processing clocks after expansion starts.
- A zero delay disables the synthetic pulse for that channel.
- The selected source is latched at the accepted `shot_start_event_t`
  boundary and remains stable for that Shot.
- The synthetic source produces one one-clock Rise event per enabled channel.
  Multi-Return verification uses the physical input because the frozen v1
  profile has one delay, not seven Return delays, per channel.

## 7. Compact Profile and Commit Contract

Echo consumes the normal atomic runtime configuration transaction:

1. write CTL20 `[15:0]` with `CH0_DELAY`;
2. write CTL20 `[31:16]` with `CHANNEL_STEP`;
3. issue one W1S `COMMIT` command;
4. wait for the normal configuration transaction completion;
5. the Echo profile reports ready only after its version matches the active
   configuration version.

There is no Echo-specific INDEX/DATA portal, write toggle, epoch or APPLY FSM.
The physical STOP path does not consume CTL20. A synthetic Shot requested
before the expanded profile is ready is rejected safely and recorded by the
`profile_not_ready` diagnostic instead of using a mixed or stale profile.

## 8. G0 Exit Criteria

- B4 edge, channel, latency and Return meanings are explicit.
- 16-channel and maximum 32-channel topology tests are required.
- The physical-path exception to the sequential-logic preference is bounded
  to the IBUFDS-to-STOP connection.
- F5 Shot identity is observation context only.
- Stage 5 GPX ownership has not been pulled into Checkpoint G.
