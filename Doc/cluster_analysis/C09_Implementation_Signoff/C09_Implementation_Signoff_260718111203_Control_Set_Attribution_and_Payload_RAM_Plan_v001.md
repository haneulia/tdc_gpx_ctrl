# C09 Control-Set Attribution and Payload RAM Plan v001

## 1. Purpose

This audit identifies the source of the high internal control-set count before
the next complexity-reduction change. The comparison point is the accepted
compact-store RTL at commit `064dc78`, recorded by C09 commit `b735f99`.

The audit uses the clean 128-bit/200 MHz ASYNC post-synthesis reports from:

- `260718101200_compact_store_dedicated_w128_a200_t200_dedicated_2x2_impl`
- `260718103100_compact_store_shared_w128_a200_t200_shared_dual_edge_impl`

## 2. Top-level attribution

| Owner | DEDICATED_2X2 | SHARED_DUAL_EDGE | Shared delta |
|---|---:|---:|---:|
| Cell pipe | 580 | 1,148 | +568 |
| Config control | 487 | 487 | 0 |
| Output stage | 191 | 191 | 0 |
| CSR pipeline | 79 | 79 | 0 |
| Face sequence | 3 | 3 | 0 |
| Top or other | 42 | 42 | 0 |
| **Total** | **1,382** | **1,950** | **+568** |

The non-cell-pipe count is topology-independent. Shared mode elaborates eight
cell builders instead of four, and the four extra builders add exactly 568
control sets:

```text
568 / 4 extra builders = 142 control sets per builder
```

## 3. Cell-pipe attribution

| Family | Dedicated control sets | Shared control sets | Per builder |
|---|---:|---:|---:|
| Persistent hit payload | 448 | 896 | 112 |
| Per-cell hit count | 64 | 128 | 16 |
| Builder control | 55 | 107 | about 13 |
| Input skid and pipe control | 13 | 17 | topology overhead |

The payload count is an exact geometry identity:

```text
2 ping-pong buffers * 8 stops * 7 hit sequence slots = 112 CE sets/builder
```

Each persistent 17-bit hit payload slot is currently implemented as resetless
FFs with its own write enable. The payload therefore accounts for:

- 32.4% of all dedicated control sets
- 45.9% of all shared control sets
- 77.2% of dedicated cell-pipe control sets
- 78.0% of shared cell-pipe control sets

The previous compact-store change removed redundant metadata FFs but could not
reduce this count because the payload write-enable geometry was unchanged.

## 4. Root cause

The current persistent store is a nested record array. A static 16-way cell
dispatch protects timing by localizing the selected cell count, but synthesis
implements each `(buffer, stop, sequence)` payload as a distinct FF control
set. The storage has no reset, yet it is not inferred as LUTRAM.

The rejected whole-record valid-mask experiment is not a solution. It added a
large first-hit merge mux and increased cell-pipe LUT use by 51.5% while leaving
the essential payload geometry intact.

## 5. Selected prototype

Split persistent state into two structures:

1. Keep the narrow per-cell `hit_count_actual` and `hit_dropped` state.
2. Move the 17-bit hit payload into seven explicit resetless banks.

Each payload bank has:

- depth: 16 addresses (`buffer & stop`)
- width: 17 bits (raw hit)
- one synchronous addressed write port
- one asynchronous read address used by the output staging register

The seven banks preserve parallel readout of all hit slots for one cell while
giving synthesis a direct distributed-RAM template. The existing count selects
which bank receives the next hit. No validity bitmap, epoch field, CSR, runtime
multiplier/divider, or output-format change is introduced.

Expected structural effect per builder:

- remove up to 112 payload FF control sets
- replace 1,904 persistent payload FFs with LUTRAM
- retain 16 count control sets and the existing serializer staging register
- preserve hit II=1 and ping-pong collect/output overlap

## 6. Safety invariants

The prototype must preserve:

1. A slot is serialized only when `slot_index < hit_count_actual`.
2. Count/drop state is cleared before a reused buffer accepts a new shot.
3. Every accepted hit overwrites its addressed payload bank before count makes
   that sequence visible.
4. Collection and output access different ping-pong buffer addresses whenever
   both are active.
5. The output staging cycle remains in place, so asynchronous RAM read delay is
   not added directly to the AXI output combinational path.
6. Rise/fall builders and their VDMA output chains remain independent.

## 7. Acceptance gates

The prototype is accepted only if all gates pass:

| Gate | Required result |
|---|---|
| C03 direct matrix | 20/20 PASS, including stale-buffer reuse |
| C06 integration | 13/13 PASS |
| Slope-role assertion | no mismatch reports |
| Payload implementation | LUTRAM inferred or equivalent control-set reduction |
| Dedicated cell-pipe control sets | materially below 580 |
| Shared cell-pipe control sets | materially below 1,148 |
| LUT/FF | no whole-record-mux style LUT regression; FF must fall |
| CDC-4 / CDC-10 | 0 / 0 |
| Internal constraints | no_clock=0, unconstrained_internal_endpoints=0 |
| 128-bit/200 MHz route | positive setup and hold in both topologies |

Early rejection is allowed after clean functional regression and a comparable
post-synthesis report if any of these occurs:

- payload remains FF-based with no meaningful control-set reduction
- top or cell-pipe LUT rises materially
- post-synthesis logic depth or fanout worsens on the cell-write path
- synthesis creates a multi-read mux instead of seven independent banks

## 8. Decision

The next RTL experiment will use the seven-bank addressed resetless payload
store. The narrow count/drop state remains unchanged in the first prototype so
the experiment isolates payload-memory inference from metadata and epoch
changes.

This audit is a measurement checkpoint, not implementation sign-off. The
accepted compact-store commit remains the rollback comparison point until the
prototype passes every gate above.
