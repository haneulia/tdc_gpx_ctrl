# v2 Migration and Verification Plan

## 1. Migration Policy

v2 is developed beside v1. A stage replaces one observable boundary at a time
and must pass comparison before the next boundary moves. The proven GPX bus
engine and complete data pipeline are never rewritten in the same stage.

The proposed implementation root is:

```text
system_integration/v2/
    pkg/
    rtl/
        config/
        csr/
        proc/
        echo/
        tdc/
        data/
        top/
    tb/
    scripts/
```

The final packaged IP uses a new entity and VLNV revision. The v1 packaged IP
remains available until v2 board sign-off.

## 2. Stage Gates

### Stage 0: Freeze and Oracle

Deliverables:

- baseline source/hash manifest;
- maintained scenario list;
- B0..B9 golden trace schema;
- explicit board-sign-off exclusions.

Gate:

- current 150/200 regression and package checks remain reproducible;
- no v1 source is changed by Stage 0/1 commits.

### Stage 1: Types and Ownership

Deliverables:

- `lidar_build_pkg` for fixed capacities and common scalar types;
- `lidar_config_types_pkg` for source, derived, active and status records;
- configuration validator specification;
- clock/event/data contracts in this directory.

Gate:

- static review finds one owner for every configuration concept;
- no Derived field appears in the writable source record;
- no real-time event depends on an AXIS monitor ready signal.

### Stage 2: Unified CSR and Configuration Manager

Deliverables:

- one AXI4-Lite CSR bank with write-event output;
- shadow configuration store;
- commit-time sequential validator/calculator;
- proc/TDC PREPARE and ACTIVATE gateways;
- system command/status/IRQ block.

Focused tests:

- byte strobes and independent AXI AW/W arrival;
- COMMIT/RESET/CLEAR W1S behavior;
- valid commit and active readback;
- invalid commit changes no active domain;
- destination timeout and retry policy;
- command while BUSY;
- sequence wrap without software-visible epochs;
- mixed change groups select the strictest safe activation point;
- the new-shot gate drains current acquisition/output work before activation;
- reset during PREPARE and ACTIVATE.

Gate:

- no mixed active version can be observed;
- no child functional core contains AXI or local/unified CSR selection.

### Stage 3: Processing Event Pipeline

Implementation order:

1. `motor_position_core`;
2. `face_tracker`;
3. operation/safety state owner;
4. `shot_scheduler`;
5. `laser_executor`;
6. read-only monitoring AXIS tap.

Comparison:

- B0 through B3 exact event sequence;
- CW/CCW, x1/x2/x4 physical decode and virtual x4;
- one through five Faces, modular wrap and overlap rejection;
- inclusive geometric lower/upper boundaries plus explicit `[entry, exit)`
  shot-lattice behavior;
- physical/simulation mutual exclusion;
- no late-angle fire after executor busy;
- cycle-accurate latency accounting with no artificial v1 AXIS-delay padding.

Gate:

- the control path has no AXIS backpressure dependency;
- all physical fire/start safety assertions pass.

### Stage 4: Echo Frontend

Deliverables:

- production LVDS-to-STOP frontend;
- optional simulation generator in a separate generate block;
- delay calibration table using the standard indexed portal;
- diagnostic snapshot/counters outside the physical path.

Comparison:

- B4 pulse count, edge, channel and latency;
- all 16 APD channels;
- Return 1 through 7;
- production synthesis contains no simulation pulse generator.

Gate:

- STOP timing is independent of CSR and monitor-stream readiness.

### Stage 5: GPX Bus and Acquisition

First wrap the proven v1 bus engine behind typed request/result records. Do not
rewrite its physical timing FSM in the first pass.

Comparison:

- B5 byte/word exactness;
- chip, IFIFO and register-read ordering;
- 50/100/125/150/200 MHz legal profiles;
- runtime bus timing applies only while quiescent;
- EF/LF/IrFlag/ErrFlag/OEN diagnostics remain observable.

Gate:

- zero request loss, duplication and stale response;
- ASYNC FIFO and SYNC bypass modes pass CDC assertions.

### Stage 6: Hit, Cell and Frame Pipeline

Replace boundaries in this order:

1. 28-bit word to `hit_event_t`;
2. Hit event to `cell_t`;
3. Cell lanes to frame builder;
4. parameterized Rise/Fall lane generation.

Comparison:

- B6 through B8 byte exactness;
- Hit bit 16 metadata preservation;
- active/present/rise/fall mask sweeps;
- one-chip dual-edge and dedicated 2-rise/2-fall;
- maximum seven Returns without padding-dependent geometry.

Gate:

- canonical line bytes are independent of output width;
- no inactive lane builder runs or contaminates diagnostics.

### Stage 7: AXIS/VDMA Formatter

Comparison:

- B9 at 32/64/128 bits;
- exact SOF, EOL, TKEEP, TLAST, HSIZE and VSIZE;
- shot-boundary stall and bounded continuous backpressure;
- Face abort policy and recovery.

Gate:

- wider AXIS width never increases canonical DDR bytes;
- no accepted byte is lost, duplicated or reordered.

### Stage 8: Integrated RTL and HTML Alignment

The RTL result contract is the source of observed timing and bytes. HTML uses
the same documented equations and compares itself with exported RTL results;
it does not invent a parallel set of constants.

Sweep dimensions:

- RPM, optical shot interval and target range;
- output width;
- chip/slope topology;
- Return count and runtime Hit capacity;
- processing/TDC clock relation.

Gate:

- every HTML PASS/CHECK state is traceable to a named RTL metric;
- no required margin uses a header-only or independently writable duplicate.

### Stage 9: Implementation and Board

Target `xc7z020clg484-2` with PS FCLK.

Required evidence:

- synthesis with zero black boxes and zero inferred latches;
- implementation timing and pulse-width checks;
- CDC report with every crossing classified;
- utilization comparison against v1;
- pin/I/O voltage and generated-clock constraints;
- laser-disabled board capture;
- controlled physical laser/GPX measurement;
- VDMA/DDR/Ethernet sustained-backpressure test.

Gate:

- v2 becomes the release IP only after board evidence closes the exclusions in
  `V2_BASELINE.md`.

## 3. Regression Policy

| Processing / TDC profile | Use |
|---|---|
| 150 / 200 MHz | routine migration regression |
| 200 / 150 MHz | routine reverse-clock-relation regression |
| 150 / 150 MHz, same physical clock | synchronous release gate |
| 200 / 50 MHz | 4:1 CDC release gate |
| 50 / 200 MHz | 1:4 CDC release gate |
| 150 / 100 MHz | slower-board operating-point budget gate |

Every completed functional checkpoint runs the two routine profiles, 150/200
and 200/150 MHz. Synchronous, extreme-ratio and slower-board profiles run only
when that checkpoint introduces or changes a CDC boundary, and again at the
integrated release gate. This keeps ordinary regressions bounded without
weakening clock-relation coverage.

## 4. Assertions Required from the First RTL Stage

- active configuration versions match before operation enable;
- physical and simulation fire/start sources are mutually exclusive;
- `start_tdc` cannot occur without the matching physical `fire_done` in
  physical mode;
- a scheduler-busy condition cannot produce a late-angle fire;
- runtime chip masks are subsets of build masks;
- rising-capable chip count is not smaller than falling-capable chip count;
- every accepted ready/valid payload remains stable while stalled;
- output bytes, HSIZE and VSIZE share one derived geometry record;
- FIFO overflow, timeout and abort are diagnosed before silent data loss.

## 5. Stage-to-Checkpoint Traceability

`Stage` and `Checkpoint` are related but are not synonyms:

- a **Stage** is a functional migration gate defined in Section 2;
- a **Checkpoint** is an immutable, reviewable implementation/evidence
  milestone recorded in Git;
- one Stage may require several Checkpoints when splitting it reduces risk;
- a completed Checkpoint is never renamed or renumbered to make a later table
  look sequential.

The original table assumed one Checkpoint for all of Stage 2. Actual
implementation deliberately split that work into calculator, atomic manager
and unified CSR boundaries. The authoritative mapping is therefore:

| Stage | Checkpoint | Status | Commit/evidence | Scope |
|---:|:---:|---|---|---|
| 0-1 | A | Complete | `3d58005` | Baseline, architecture and ownership contracts |
| 1 | B | Complete | `cbcbc45` | Common configuration types, reference arithmetic and package tests |
| 2 | C | Complete | `1b8b015`, `8789e7b` | Sequential validator/deriver and all runtime timebase conversions |
| 2 | D | Complete | `67e0800` | Atomic configuration manager and Processing/TDC gateways |
| 2 | E | Complete | `6cd1adf` | Unified 32 CTL / 32 STAT / 4 IRQ CSR boundary |
| 3 | F | In progress | F0/F1/F2/F3a/F3b complete; B3/F5 pending | Processing event pipeline |
| 4 | G | Pending | B4 evidence pending | Echo frontend |
| 5 | H | Pending | B5 evidence pending | Proven GPX bus/acquisition wrapper |
| 6 | I | Pending | B6..B8 evidence pending | Hit, Cell and Frame pipeline |
| 7 | J | Pending | B9 evidence pending | AXIS/VDMA formatter |
| 8 | K | Pending | Integrated RTL/HTML evidence pending | Full RTL integration and HTML alignment |
| 9 | L | Pending | Parent/board evidence pending | Implementation, board sign-off and release tag |

**Current migration state:** Stage 2 is closed at Checkpoint E. Stage 3 /
Checkpoint F is in progress; F0a/F0b, F1/B0, F2/B1, F3a operation/safety and
F3b/B2 are complete. The only valid next sub-step is **F4 laser_executor**.
Stage 4 or
later work must not be treated as migrated merely because its v1
implementation exists.

Commit only after the focused sub-step passes. Intermediate commits inside a
Checkpoint are allowed, but the Checkpoint result document and status row move
to Complete only after its complete Stage gate passes.

Never stage unrelated user changes, generated Vivado work directories, wave
databases or transient logs. Each archived sign-off session contains only the
scenario, source manifest, compact results, final logs and required waveforms.

## 6. Immediate Next RTL Work Package: Stage 3 / Checkpoint F

Checkpoint F moves only the Processing-domain event path. Its internal order
is fixed so that each new block has one already-verified input boundary:

| Step | Work | Required evidence before continuing |
|---:|---|---|
| F0a | Freeze v1 B0..B3 trace fields, stimulus and latency counting edges | Trace schema, source hashes and expected vectors reviewed |
| F0b | Close Processing source-mode ownership and CSR ABI | `SIMULATION_MODE` is active configuration, measured latency is read-only; all Stage 2 regressions pass |
| F1 | Implement `motor_position_core` | B0 exact comparison for CW/CCW and physical x1/x2/x4 plus virtual x4 |
| F2 | Implement `face_tracker` | B1 exact comparison for one to five Faces, wrap and boundary direction |
| F3a | Implement operation/safety state owner | RUN/STOP/ARM/DISARM and external permit semantics are self-checked; reset is fail-safe |
| F3b | Implement `shot_scheduler` | B2 exact geometric request sequence, angular quantization and busy suppression |
| F4 | Implement `laser_executor` | B3 exact physical/simulation exclusion, fire/start/stop timing and timeout behavior |
| F5 | Integrate the direct registered event path and read-only AXIS monitor tap | B0..B3 end-to-end comparison, assertions and both routine clock profiles |

Current sub-step status:

| Step | Status | Evidence |
|---:|---|---|
| F0a/F0b | Complete | `V2_STAGE3_F0_PROCESSING_ORACLE.md` |
| F1 | Complete | `V2_CHECKPOINT_F1_MOTOR_POSITION.md`; session `260804170100_v2_motor_position` |
| F2 | Complete | `V2_CHECKPOINT_F2_FACE_TRACKER.md`; session `260804184000_v2_face_tracker` |
| F3a | Complete | `V2_CHECKPOINT_F3A_OPERATION_SAFETY.md`; sessions `260804203000_v2_operation`, `260804204000_v2_unified_csr` |
| F3b | Complete | `V2_CHECKPOINT_F3B_SHOT_SCHEDULER.md`; session `260804211500_v2_shot_scheduler` |
| F4 | Next | B3 physical/simulation lifecycle evidence not yet generated |
| F5 | Blocked by order | F4 boundary has not passed |

Rules for this package:

1. `lidar_csr_config_subsystem` remains the only AXI4-Lite owner.
2. Each core consumes the active Processing-domain configuration record; it
   does not reinterpret CSR words.
3. Motor-to-Laser control events use direct registered records. AXIS is a
   monitor copy and cannot apply backpressure to `fire_pulse` or `start_tdc`.
4. The next step starts only after the preceding boundary comparison passes.
5. Checkpoint F closes only after 150/200 and 200/150 MHz regressions pass with
   zero inferred latches and no new unclassified CDC path.
6. F1 was gated by the F0b package/calculator/manager/CSR regressions, and F3b
   was gated by F3a ownership of every operation and laser-permit state. Those
   gates are now closed; the invariant remains that a scheduler must never
   infer safety permission from configuration validity alone.
