# v2 Clock, Event and Data Contract

## 1. Clock Domains

| Domain | Clock/reset | Owned behavior |
|---|---|---|
| CSR | `s_axi_csr_aclk`, `s_axi_csr_aresetn` | AXI transactions, shadow config, commands, software status |
| Processing | `proc_aclk`, `proc_aresetn` | Encoder, Face tracker, shot scheduler, Laser executor, Echo diagnostics, AXIS output |
| TDC | `i_tdc_clk`, TDC-domain reset | GPX bus timing, acquisition, raw result production |
| External async | Encoder, `fire_done`, LVDS Echo, GPX status pins | Physical inputs requiring I/O or CDC treatment |

Equal configured frequencies do not prove synchronous clocks. `SYNC` mode is
legal only when Processing and TDC receive the same physical clock source and
the timing constraints declare that relation.

## 2. CDC Classes

| Transfer | Required mechanism |
|---|---|
| Build constants | No CDC hardware; elaborate locally |
| Multiword configuration | Coherent request/ack mailbox with held source payload |
| One-shot command | Command mailbox with internal sequence ID and completion |
| Activation | Prepared-version handshake followed by activation event |
| Low-rate status snapshot | Coherent destination request/ack snapshot |
| Sticky event | Set in the owning domain, synchronize a level or snapshot status |
| Counter | Snapshot the complete counter; do not synchronize bits independently |
| GPX result stream | Async FIFO in ASYNC mode; direct registered path in legal SYNC mode |
| External level/pulse | I/O primitive plus explicit synchronizer or pulse-capture circuit |

No functional core instantiates a CSR CDC block. The only configuration CDC
instances are `proc_config_gateway` and `tdc_config_gateway`.

## 3. Reset Contract

- Every domain has an asynchronously asserted, synchronously released reset.
- Reset crossing is not implemented by sampling another domain's reset level.
- Software reset is a command event, not a replacement for physical reset.
- A software reset clears operational state and selected stickies but does not
  silently change build topology or active calibration.
- The configuration manager reports reset completion only after all required
  domain acknowledgements arrive.

## 4. Real-Time Processing Events

### 4.1 `position_event_t`

Produced by `motor_position_core`, consumed by `face_tracker` and
`shot_scheduler` in the Processing domain.

| Field | Meaning |
|---|---|
| `valid` | One-cycle registered event for a decoded position update |
| `position` | Modular decoded state in the active `states_per_rev` range |
| `direction` | Actual decoded traversal direction |
| `source_sim` | Physical/virtual source identity |
| `source_latency_clks` | Measured or configured input-to-event latency metadata |
| `z_event` | Qualified revolution/index event |

There is no AXIS backpressure on this path. A monitoring AXIS stream may tap
the event after the scheduler and may drop monitor samples without affecting
control.

### 4.2 `face_event_t`

Produced by `face_tracker`.

| Field | Meaning |
|---|---|
| `inside` | Current position is inside one active Face window |
| `enter` / `exit` | One-cycle boundary events |
| `face_index` | Selected active Face |
| `overlap` | More than one Face window matches |
| `position` / `direction` | Aligned position context |

Overlapping windows are a configuration error unless an explicit future
policy defines priority. v2 validation rejects overlap by default.

### 4.3 `shot_request_t`

Produced by `shot_scheduler`, consumed by `laser_executor`.

| Field | Meaning |
|---|---|
| `valid` | Registered request at an eligible angular position |
| `face_index` | Face that owns the shot |
| `position` | Exact decoded state used for the decision |
| `shot_index` | Monotonic index within the Face |
| `last_in_face` | Last scheduled point in this Face |
| `source_sim` | Simulation/physical execution selection |
| `position_latency_clks` | Carried timing metadata |

The executor returns `accept` in the same Processing-domain contract. The
scheduler issues a request only when angular spacing and minimum shot-time
conditions are satisfied. Executor busy cannot move a physical shot to a later
unqualified angle; it produces a blocked-shot diagnostic instead.

### 4.4 Physical and Simulation Laser Contract

| Mode | `fire_pulse` | start source | `start_tdc` |
|---|---|---|---|
| Physical | Generated from an accepted physical request | synchronized `fire_done` | generated only after the matching physical `fire_done` |
| Simulation | Forced inactive | simulation executor | generated only from the simulation event |

The two start sources are mutually exclusive and asserted as an invariant.
`fire_done` arriving without an armed physical command is ignored for START and
recorded as a diagnostic.

`start_tdc`, `shot_start`, Face context and the TDC acquisition command must be
derived from one accepted-shot record so they cannot refer to different shots.

## 5. Echo STOP Exception

The physical LVDS-to-GPX STOP path is latency critical and is deliberately not
routed through AXIS, CSR, a general-purpose FIFO or the TDC data pipeline.

```text
LVDS pin -> differential input primitive -> shortest qualified path
         -> GPX STOP output
```

Any register on this path must have a measured, fixed latency and must be part
of the distance calibration. The initial v2 physical path has no runtime delay
table. A future IODELAY-based calibration is a separate build feature and may
not reuse the simulation delay table. A long combinational cone is prohibited,
but a short I/O path is allowed when required to preserve physical timing.
Simulation STOP generation is a separate build-time generate block and is
removed from production builds.

## 6. GPX Data Types

### 6.1 External I-Mode word

```text
27          26 25                         18 17 16               0
+--------------+-----------------------------+--+------------------+
|   ChaCode    |          StartNum           |Sl|      Hit         |
+--------------+-----------------------------+--+------------------+
```

| Field | Width | Contract |
|---|---:|---|
| `ChaCode` | 2 | channel inside the selected IFIFO |
| `StartNum` | 8 | start association; zero in the current single-shot profile |
| `Slope` | 1 | rising/falling edge identity |
| `Hit` | 17 | external GPX time/distance code |

### 6.2 Internal records

v2 uses typed records rather than unrelated `TDATA/TUSER` slices inside the
core.

```text
gpx_word_t  = chip + ififo + raw_28 + drain/end metadata
hit_event_t = shot + chip + stop + return + slope + hit_17
cell_t      = shot + chip + stop + slope + valid count + Hit[0..N-1]
line_t      = face + slope lane + canonical bytes + SOF/EOL metadata
```

The records are flattened only at the IP-XACT boundary or a proven FIFO width
boundary.

## 7. Backpressure Rules

| Path | Backpressure allowed? | Rule |
|---|---|---|
| Encoder -> scheduler | No | Control event; monitor drops must not affect it |
| Scheduler -> executor | Bounded accept contract | Busy blocks and diagnoses; it cannot shift the shot angle |
| Laser -> Echo shot context | No arbitrary stall | One-entry registered context, proven empty before next accepted shot |
| Echo -> physical GPX STOP | No | Physical timing path |
| GPX bus -> async result FIFO | Yes, bounded by acquisition policy | FIFO-full becomes a fault before data corruption |
| Hit -> Cell -> Frame | Yes | Ready/valid or local FIFO with stable payload |
| Frame -> VDMA AXIS | Yes | Full AXI stability and Face abort policy required |
| Diagnostic AXIS | Yes or drop | Must never throttle a physical control path |

## 8. Latency Accounting

Every deterministic Processing-domain stage publishes an elaboration-time or
measured latency contribution. Status reports both the configured expectation
and the observed latency.

| Metric | Start | End |
|---|---|---|
| Encoder-to-position | synchronized A/B transition | `position_event.valid` |
| Position-to-fire | `position_event.valid` | physical `fire_pulse` rising edge |
| Fire-to-done | physical `fire_pulse` | synchronized matching `fire_done` |
| Fire-to-TDC | physical `fire_pulse` | `start_tdc` rising edge |
| START-to-first-word | `start_tdc` | first accepted 28-bit GPX word |
| START-to-last-output | `start_tdc` | final accepted AXIS byte of the shot |

No hard-coded measured latency is used to repair functional timing. Measured
values are diagnostics and calibration inputs; the control path remains
correct if pipeline registers are deliberately changed and the contract is
updated.
