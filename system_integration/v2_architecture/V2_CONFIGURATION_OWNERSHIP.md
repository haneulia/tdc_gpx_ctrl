# v2 Configuration Ownership

## 1. Configuration Classes

Every setting belongs to exactly one of these classes.

| Class | Meaning | Update mechanism |
|---|---|---|
| Build | Changes hardware topology or timing implementation | Generic before synthesis |
| Runtime source | Operator-owned value that may change while stopped or running | CSR shadow plus commit |
| Command | One operation, not persistent configuration | W1S write event |
| Derived | Deterministic result calculated from Build/Runtime sources | Commit calculator, read-only |
| Status | Live observation, sticky cause or counter | Read-only; explicit clear command where required |

A value must never be both Runtime source and Derived. A status readback may
show an active source value, but it does not become a second owner.

## 2. Single-Owner Matrix

| Physical concept | Class | v2 owner | Consumers | v2 rule |
|---|---|---|---|---|
| Processing clock frequency | Build | top build profile | all processing cores, time converter | One generic, legal set 50/100/125/150/200 MHz |
| GPX clock frequency | Build | top build profile | GPX bus/acquisition | One generic, legal set 50/100/125/150/200 MHz |
| Stream sync/async mode | Build | top build profile | GPX CDC gateway | SYNC requires the same physical clock, not only equal numeric MHz |
| GPX chip count | Build | GPX topology profile | Echo, GPX bus, lane generator | 1..4; top vectors scale with this value |
| Present-chip mask | Derived build value | build helper | Echo, GPX bus, validation | Lower `NUM_CHIPS` bits are one; no separate generic |
| STOPs per chip | Build | GPX topology profile | Echo and GPX acquisition | 2..8; no separate Echo copy |
| Maximum Returns/STOP | Build | GPX topology profile | Cell storage and formatter | 1..7 capacity only |
| Rise/Fall capabilities | Build | GPX topology profile | Echo wiring and lane generator | Masks may overlap for per-chip dual-edge operation |
| AXIS output width | Build | formatter profile | Cell serializer and output ports | 32/64/128 only; canonical bytes unchanged |
| Polygon face count | Build | mirror profile | Face tracker, scheduler, XGUI | 1..5 and not runtime writable |
| Echo frontend inclusion | Build | top build profile | top generate | Disabled frontend is not instantiated |
| Echo simulation inclusion | Build | verification profile | Echo frontend | Production build removes simulation logic |
| Encoder CPR | Runtime source | Motor config | decoder, geometry calculator | One active value for physical and virtual modes |
| Physical decode multiplier | Runtime source | Motor config | decoder, geometry calculator | x1/x2/x4; virtual source remains x4 internally |
| Direction/polarity | Runtime source | Motor config | decoder, face traversal, scheduler | Applied direction and observed decoded direction remain distinct status |
| Physical/simulation source mode | Runtime source | Motor config | source selector, scheduler, laser executor | Atomic mode switch; physical and simulation fire/start paths are mutually exclusive |
| Encoder/path latency | Status | Processing subsystem | event metadata, software and HTML | F1/F5 measured read-only values with validity; never insert padding and are not writable configuration |
| Processing monitor retention/drop | Status | Processing AXIS monitor | software diagnostics only | One retained beat; newer stalled samples drop/count and never own control or safe-point state |
| Run/stop and arm/disarm | Command | Processing operation manager | scheduler and laser executor | W1S operation events; not persistent geometry and not inferred from config validity |
| External laser permit | Live safety input | top safety boundary | operation manager and final fire gate | Reset/unconnected/unknown means inhibit; software cannot override it |
| Virtual speed profile | Runtime source | Motor config | virtual encoder | `ticks_lo` plus fractional high-count; `ticks_hi=ticks_lo+1` derived |
| Face centers 0..4 | Runtime source | Mirror config | Face tracker | Direct shadow words; inactive faces ignored |
| Common Face half-width | Runtime source | Mirror config | Face tracker | One value shared by all active faces |
| Face lower/upper bounds | Derived | commit calculator | Face tracker and XGUI readback | Modular center +/- common half-width |
| Fire pulse width | Runtime source | Laser config | laser executor | Fixed 5 ns ticks converted once at gateway |
| Fire-done timeout | Runtime source | Laser config | laser executor and fault manager | Fixed 5 ns ticks; validated against the shot timing policy |
| Requested target round-trip window | Runtime source | `CTL12.TARGET_RANGE` | Commit calculator | One 5 ns source; the only software-owned range time |
| Effective target round-trip window | Derived | Commit calculator | Laser executor and TDC acquisition | Requested value rounded up to the 25 ns GPX MTimer grid |
| GPX Reg7.MTimer | Derived | Commit calculator | GPX active register image | `ceil(TARGET_RANGE/5)` at the fixed 40 MHz GPX reference; direct staging value is overridden |
| GPX capture-window adjustment | Runtime calibration | TDC calibration | TDC acquisition | Optional signed board offset; never a second target range |
| Requested optical shot interval | Runtime source | Scan config | commit calculator | Fixed-point angle or equivalent software ABI value |
| Shot interval in states | Derived | commit calculator | shot scheduler | Rounded up, never rounded down |
| Columns per Face | Derived | commit calculator | frame builder and VDMA geometry | Derived from active Face span and shot interval |
| Shot geometric column index | Event identity | shot scheduler | laser executor, frame builder and VDMA formatter | Advances at every due lattice point; busy skips leave holes and never compact later columns |
| START/STOP pulse widths | Runtime source | Laser/TDC timing config | laser executor | Fixed 5 ns ticks; one owner |
| Echo channel delay profile | Runtime source | CTL20 compact profile | Echo simulation path only | `delay[n] = CH0 + n * STEP`; one atomic commit, no indexed portal |
| GPX bus divider/ticks | Runtime source | TDC bus profile | GPX bus engine | Commit accepts divider 1..63 and ticks 1..7; applied only while quiescent |
| GPX register image | Runtime source | GPX image table | GPX initialization engine | One standardized indexed portal |
| Runtime active-chip mask | Runtime source | TDC scan config | acquisition and lane generator | Must be a subset of the build-present mask |
| Runtime maximum Hits | Runtime source | TDC scan config | Cell builder and formatter | 1..build maximum; zero alias is removed in v2 |
| Maximum scan timeout | Runtime source | TDC scan config | acquisition watchdog | Explicitly includes range, drain and service budgets |
| Distance calibration | Runtime source | calibration config | distance formatter | Coherent scale/offset snapshot |
| Cell bytes and VDMA geometry | Derived | commit calculator | frame builder, status and software | Never independently writable |

### 2.1 Fixed Processing implementation timing

The following values are implementation contracts, not CSR settings and not
build generics:

| Value | Fixed value | Meaning |
|---|---:|---|
| B0-to-executor accept | 5 Processing clocks | Registered `position_event` through B1/B2 and the B3 request-ingress register to matching accept |
| Physical sample-to-fire | 9 Processing clocks | First stable-pin synchronizer sample to physical fire; excludes pre-sample asynchronous phase |
| Virtual source-to-accept | 7 Processing clocks | Internal virtual-source A/B/Z transition through the common input and B3 ingress pipelines; excludes configured simulation START delay |
| Fire-done observation budget | 3 Processing clocks | Two synchronizer stages plus the consuming FSM edge; used to resolve a T0 captured at timeout/abort boundary |
| Re-arm margin | 2 Processing clocks | Required quiet interval after every generated pulse is inactive |

The path values are exposed by the F5 Processing subsystem and the final two
values by B3 so software/HTML can report the actual implementation. They must
change only with RTL and verification evidence; an operator cannot use them to
repair an invalid shot-rate configuration.

### 2.2 Operation and safety transition contract

`lidar_operation_manager` is the only owner of persistent RUN and ARM state.
The CSR bank emits W1S events, and an acknowledged one-entry mailbox transfers
them to the Processing clock without converting them into writable levels.

- reset and loss of `ACTIVE_VALID` produce STOPPED/DISARMED;
- RUN is accepted only while the Processing active configuration is released;
- ARM additionally requires RUN; physical mode also requires a synchronized
  external permit, while simulation mode does not;
- STOP clears RUN and ARM together, so RUN cannot revive a latent ARM;
- DISARM clears only ARM;
- raw external-permit LOW/unknown closes the physical fire gate immediately;
  an observed physical-mode permit loss clears ARM and requires a new ARM;
- external permit can never be overridden by software;
- atomic PREPARE temporarily closes operation enable but preserves RUN/ARM
  memory while the pipeline drains, then RELEASE reopens the same state;
- a command presented while the CDC mailbox is busy is rejected and diagnosed,
  never overwritten or reordered.
- CSR reset revokes command authority in the Processing domain and forces
  STOPPED/DISARMED regardless of the request-toggle parity.

The scheduler consumes `scheduler_enable`; the physical executor/final pin gate
consumes `physical_fire_enable`. Neither block may reconstruct permission from
`ACTIVE_VALID`, source mode or Face membership.

### 2.3 Optical candidate-point priority

The requested optical angle defines the only legal shot lattice. At every due
point, the scheduler samples both the laser-executor ready state and the GPX
acquisition ready state. If either is low, it advances the geometric column,
emits no late replacement shot and records `schedule_overrun`. This keeps the
point angle truthful: insufficient timing budget appears as a Hole/error rather
than as a shot at the wrong angle.

The live test covers the actual fire-done response, effective target
round-trip window, re-arm interval, GPX Lane drain/merge and accumulated output
backpressure. These quantities are not all predictable from configuration
alone, especially with a physical motor, so COMMIT validates representable
values while the candidate-point check closes the Runtime timing contract.

## 3. Derived Equations

The commit calculator uses integer/fixed-point arithmetic and runs only during
configuration commit. It is not in the encoder, shot or TDC data path.

```text
decoded_states_per_rev = CPR * physical_decode_multiplier

mechanical_angle_per_state = 360 deg / decoded_states_per_rev
optical_angle_per_state    = 2 * mechanical_angle_per_state

shot_interval_states = ceil(
    requested_optical_shot_interval / optical_angle_per_state)

face_lower = modulo(face_center - common_half_width, decoded_states_per_rev)
face_upper = modulo(face_center + common_half_width, decoded_states_per_rev)

face_active_positions = 2 * common_half_width + 1
face_angular_intervals = 2 * common_half_width

columns_per_face = ceil(
    face_angular_intervals / shot_interval_states)
```

`face_lower` and `face_upper` are both inclusive for Face detection and XGUI
readback, giving `2*half+1` observable positions. The shot lattice represents
the `2*half` angular intervals between those boundaries and deliberately uses
`[entry, exit)` traversal. The exit boundary is a Face position and boundary
event, but is not a second endpoint shot. This is an explicit scheduler rule,
not a hidden `upper-1` change to the stored geometry.

CW/CCW does not rewrite the stored geometric bounds. Direction selects whether
the tracker traverses `lower -> upper` or `upper -> lower` and changes boundary
crossing tests accordingly. Direction therefore exchanges Entry and Exit but
does not change the number of angular intervals or columns.

The range relation is:

```text
tdc_capture_window = target_range_window + signed_capture_adjustment

minimum_shot_time = fire_done_allowance
                  + target_range_window
                  + deterministic_rearm_margin
```

`target_range_window` is the only operator-owned flight-time value. Any TDC
board offset is a named calibration and must be visible in active readback.
The deterministic re-arm margin is an implementation-owned, read-only number;
it is not another runtime timing knob.

The proven GPX acquisition watchdog is 16 bit. Therefore the derived capture
window must satisfy `ceil(capture_window_5ns * TDC_CLK_MHZ / 200) <= 65535`.
The commit validator rejects a larger value as `CFG_RUNTIME_CAPTURE_WINDOW`;
the TDC lane never truncates an accepted 32-bit source value.

Commit validation includes:

- requested optical interval is not smaller than one optical state;
- `fire_done_timeout <= target_range_window` for the current operating policy;
- Face windows are valid and do not overlap;
- derived columns and VDMA geometry fit their build capacities;
- active masks and runtime Hit capacity fit the build topology.

The Processing pipeline evaluates the angular-time margin from measured
revolution timing in physical mode and the configured virtual timing in
simulation mode. This is a live feasibility/status decision rather than a
configuration-validity decision: a geometrically valid commit remains valid,
but firing is inhibited and diagnosed while the current speed cannot satisfy
the fire-done, range-window and deterministic re-arm budget. At every due grid
point, a busy executor blocks the shot, never delays it to an off-grid angle,
and records an overrun. The due point still consumes its geometric column
index, so downstream logic can distinguish "no return" from "no shot was
issued" without shifting all later points toward column zero.

## 4. Application Safe Points

Runtime CSR writes may occur at any time because they only change shadow
storage. Activation follows one conservative policy so a Frame never contains
two configurations.

| Change group | Earliest activation point | Additional action |
|---|---|---|
| Live run/stop command | Command-specific | Does not modify configuration |
| CPR, decode, direction, Z policy | Stopped plus next qualified Z/re-lock | Position and Face state restart coherently |
| Virtual speed | Revolution boundary | Fractional scheduler restarts at phase zero |
| Face centers/common width | Face boundary with no shot in flight | New geometry begins with a complete Face |
| Shot interval, range, Laser timing | Face boundary with GPX acquisition idle | New Shot issuance is gated during activation |
| Runtime chip mask/Hit capacity | Face boundary after TDC and output drain | Frame geometry changes in the same transaction |
| GPX bus timing/image | TDC quiescent, normally stopped | Reinitialization is explicit when image changes |
| Distance calibration | Face boundary after output drain | One Face uses one calibration version |

For the first v2 implementation, one commit containing several groups uses the
strictest safe point in the table. The manager closes the new-shot gate, drains
the current acquisition/output work, activates every prepared domain, then
reopens operation. Finer-grained live updates may be added only with separate
proof; they are not part of the initial design.

The F5 `processing_pipeline_idle` owner includes only B2 unresolved ownership
and B3 activity. A pending or stalled monitor beat is deliberately excluded.
The central manager must add TDC acquisition and output-drain idle after Stage
H/J; no monitor or diagnostic stream may become a safe-point owner.

## 5. Commit Transaction

v1 reports whether independently operating adapters eventually accepted the
same software epoch. It does not prevent one adapter from activating while a
different adapter rejects. v2 replaces that post-fact comparison with one
central transaction.

```text
Software writes shadow values
        |
        v
COMMIT W1S event
        |
        v
central validation + sequential derived calculations
        |
        +-- invalid: keep every active domain unchanged, report one reason
        |
        v
close new-shot gate and reach the required safe point
        |
        v
PREPARE(version, proc snapshot, TDC snapshot)
        |
        +-- proc ACK prepared
        +-- TDC ACK prepared
        |
        v
ACTIVATE(version) at a defined safe boundary
        |
        v
ACTIVE_VERSION update and DONE event
```

Rules:

- software never writes a sequence/epoch token;
- a new command while BUSY is rejected with a stable error code;
- an invalid commit changes no active value;
- a timeout before all PREPARE acknowledgements aborts without activation;
- activation is simultaneous from the system contract perspective;
- operation remains inhibited if required domains report different versions.

## 6. CSR Shape

The external address geometry remains 32 CTL, 32 STAT and four interrupt words
for software and IP packaging stability. Not every word needs to be populated.

The v2 map follows these rules:

- Face centers use five direct words plus one common-width word. Spending a few
  words is preferred over keeping Face index, write epoch, read index and
  shadow-selection state machines.
- Echo uses one direct CTL20 word because its 32 channel values follow the
  deterministic `CH0 + channel * STEP` profile. Only the 16-entry GPX image
  remains indexed.
- GPX indexed writes are driven by an AXI write event from the central CSR bank.
  Software does not toggle a bit or invent a new epoch.
- system commands are W1S events with `BUSY`, `DONE`, `ERROR` and
  `ERROR_CODE` status.
- important active and derived values have direct readback.
- reserved words read zero and ignore writes.

The register count is not minimized at the expense of transaction logic. A
direct register that removes an index/epoch FSM is considered a simplification.

## 7. Prohibited Duplicates

v2 must fail static review if it contains any of these patterns:

- separate Laser roundtrip and TDC maximum-range operator settings;
- `G_PRESENT_CHIP_MASK` alongside pin-scaling `G_NUM_CHIPS`;
- separately writable `COLS_PER_FACE`, HSIZE or VSIZE;
- `g_TOTAL_STATES` alongside CPR and fixed virtual x4 decode;
- per-Face half-width values when the optical contract uses one common width;
- local and unified CSR generate branches inside a functional core;
- software-visible reset/config epochs or command toggles;
- header-only runtime controls with no implemented behavior.
