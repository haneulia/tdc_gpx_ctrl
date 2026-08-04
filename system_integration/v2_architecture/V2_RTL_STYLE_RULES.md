# v2 RTL Structure and Style Rules

## 1. Module Boundary Rules

Create an entity only when at least one of these is true:

1. it owns an independent state machine;
2. it is a clock/reset or protocol boundary;
3. it is a reusable storage/flow-control primitive;
4. it has a separately verifiable functional contract;
5. it maps directly to a physical I/O responsibility.

Do not create an entity only to rename or forward a large port list. In
particular, v2 must not reproduce a wrapper that carries disabled local AXI
ports and duplicate unified CSR words.

Target limits are review triggers rather than synthesis rules:

| Item | Target | Required action when exceeded |
|---|---:|---|
| Functional entity | 400 lines | Review state ownership and split by contract |
| Top-level wrapper | 250 lines | Move behavior to an owned core or gateway |
| Clocked process | 120 lines | Split independent state or extract pure helpers |
| Main FSMs/entity | 1 | Document why multiple FSMs share one owner |
| Clock domains/entity | 1 | Use a named gateway for the crossing |

## 2. Top-Level Rule

`tdc_gpx_lidar_ctrl_v2_top` contains only:

- external port declarations and IP-XACT attributes;
- build-time legality assertions;
- entity instantiations and record wiring;
- build-time generate statements;
- direct assignments required to flatten records/arrays at the package edge.

It contains no runtime FSM, no CSR field decoding, no arithmetic and no CDC
implementation.

## 3. Sequential and Combinational Logic

Sequential logic is the default for state, event alignment and long arithmetic.
Combinational logic is allowed for short, explicit transforms.

| Allowed combinational use | Prohibited combinational use |
|---|---|
| equality/range checks on registered inputs | variable-length search in a shot-critical path |
| small mux from an enumerated state | chained arithmetic across module boundaries |
| pure pack/unpack functions | division or modulo in the real-time path |
| next-state logic with bounded depth | large priority encoders without a timing budget |
| low-latency Echo physical path | CDC based on combinational pulses |

All division, ceiling and geometry derivation occurs in the commit-time
sequential calculator. One small arithmetic engine may be reused over several
cycles because configuration latency is not shot latency.

## 4. Registered Event Rule

- A real-time event is generated from registered state.
- Event payload and `valid` are registered together.
- A pulse has one named producer.
- A pulse is never reconstructed by comparing independently synchronized data.
- A required event either has a bounded accept response or an explicit
  overrun/fault result.
- Monitoring and debug logic may observe an event but may not gate it.

## 5. Ready/Valid and FIFO Rule

Use ready/valid only where a receiver is allowed to delay data. Each boundary
states its maximum buffering and failure policy.

- Use a one-entry elastic register for a single pipeline boundary.
- Add a FIFO only when a quantified rate or CDC requirement needs more depth.
- Do not reset a FIFO or payload store at a shot boundary.
- Reset pointers and validity, not every payload bit, unless the technology
  primitive requires data reset.
- Full/overflow must become a fault before silent loss.
- A disabled lane is removed by generate or receives no events; it does not run
  blank transactions that are later discarded.

## 6. Array and Lane Rule

Chip, STOP, Return and slope dimensions use arrays and generate loops.

Preferred:

```vhdl
type gpx_lane_array_t is array (natural range <>) of gpx_lane_t;
signal rise_lane : gpx_lane_array_t(0 to G_NUM_CHIPS - 1);
```

Avoid separate `_0`, `_1`, `_2`, `_3` data ports inside the core. Flattening is
allowed only at an external package boundary that cannot represent the record.
Rise/Fall use one parameterized lane implementation. A slope is metadata and a
capability mask, not a reason to copy an entire pipeline by hand.

## 7. Reset and Fanout Rule

- Large data arrays and BRAM payloads are not synchronously cleared.
- Reset validity, state, pointers and sticky controls only.
- Wide active configuration records may be registered once per consuming
  module to control fanout, but all copies carry the same active version.
- High-fanout enables and masks are registered near consumers.
- Reset release and configuration activation are separate operations.

## 8. Naming Rule

New v2 RTL uses one consistent scheme:

| Object | Rule | Example |
|---|---|---|
| Generic | `G_` prefix | `G_NUM_CHIPS` |
| Constant | `C_` prefix | `C_MAX_RETURNS` |
| Type | `_t` suffix | `hit_event_t` |
| Registered signal | `_r` suffix | `active_cfg_r` |
| Combinational signal | `_c` suffix | `face_match_c` |
| Next-state value | `_n` suffix | `state_n` |
| Input/output port | `i_` / `o_` | `i_cfg`, `o_event` |
| Domain-specific signal | domain in name when needed | `cfg_proc_r`, `status_tdc` |

Do not encode historical review rounds, temporary fixes or line numbers in
signal names. Comments explain the invariant or hardware reason, not the edit
history.

## 9. Package Rule

Use separate packages for separate ownership:

- `lidar_build_pkg`: immutable capacities and common scalar types;
- `lidar_config_types_pkg`: source/derived/active configuration records;
- `lidar_event_types_pkg`: real-time event records;
- `gpx_data_types_pkg`: GPX word, Hit, Cell and lane records;
- `lidar_csr_map_pkg`: software ABI addresses and bit positions only.

Generic-dependent values are calculated locally or by pure functions. A
package does not pretend that an entity generic is a global variable.

There is one canonical source for each common package. IP packaging may copy a
snapshot into `ip_repo`, but generated copies are never edited manually.

## 10. Assertions and Diagnostics

Every entity asserts its local contract close to the boundary:

- generic legality at elaboration;
- impossible state combinations;
- event mutual exclusion;
- ready/valid stability;
- counter/index bounds;
- configuration version match;
- FIFO overflow/underflow;
- physical/simulation isolation.

Assertions use stable IDs such as `V2-CFG-001` or `V2-DATA-004`. A test passes
only when it checks functional results and expected diagnostics; absence of a
failure marker is not a PASS.

## 11. Review Checklist

Before a new entity is accepted:

- its owner and clock domain are stated in the header;
- its input/output record contract is documented;
- its latency and buffering are bounded;
- it does not decode raw CSR words;
- it does not synchronize unrelated fields independently;
- it has focused tests for nominal, boundary, reset and fault behavior;
- synthesis shows no latch, unintended RAM reset or unconstrained CDC;
- the change preserves the next v1 golden comparison boundary.
