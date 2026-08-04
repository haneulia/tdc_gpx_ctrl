# v2 Migration and Verification Plan

## 1. Migration Policy

v2 is developed beside v1. A stage replaces one observable boundary at a time
and must pass comparison before the next boundary moves. The proven GPX bus
engine and complete data pipeline are never rewritten in the same stage.

The proposed implementation root is:

```text
system_integration/v2/
    pkg/
    control/
    proc/
    echo/
    tdc/
    top/
    tb/
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
3. `shot_scheduler`;
4. `laser_executor`;
5. read-only monitoring AXIS tap.

Comparison:

- B0 through B3 exact event sequence;
- CW/CCW, x1/x2/x4 physical decode and virtual x4;
- one through five Faces, modular wrap and overlap rejection;
- inclusive geometric lower/upper boundaries plus explicit `[entry, exit)`
  shot-lattice behavior;
- physical/simulation mutual exclusion;
- no late-angle fire after executor busy;
- exact latency metrics.

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

| Frequency/profile | Use |
|---|---|
| Processing 150 / TDC 200 MHz | routine integrated regression |
| Processing 150 / TDC 150 MHz, same physical clock | synchronous release gate |
| Processing 200 / TDC 50 MHz | 4:1 CDC release gate |
| Processing 50 / TDC 200 MHz | 1:4 CDC release gate |
| Processing 150 / TDC 100 MHz | slower-board operating-point budget gate |

Routine work uses the requested 150/200 profile. Extreme ratios run at CDC
release checkpoints, not on every small edit.

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

## 5. Git Checkpoints

Commit only after the stage's focused gate passes.

| Checkpoint | Commit scope |
|---|---|
| A | Architecture and ownership contracts only |
| B | Common types plus package unit tests |
| C | CSR/config manager plus atomic-commit tests |
| D | Processing event pipeline plus B0..B3 comparison |
| E | Echo frontend plus B4 comparison |
| F | GPX bus wrapper plus B5 comparison |
| G | Hit/Cell/Frame pipeline plus B6..B8 comparison |
| H | Formatter plus B9 comparison |
| I | Integrated synthesis and HTML alignment |
| J | Board sign-off and release tag |

Never stage unrelated user changes, generated Vivado work directories, wave
databases or transient logs. Each archived sign-off session contains only the
scenario, source manifest, compact results, final logs and required waveforms.

## 6. Immediate Next RTL Work Package

After Checkpoint A, the first implementation package is deliberately small:

1. create `lidar_config_types_pkg.vhd`;
2. define Build, Runtime source, Derived and Active records;
3. implement pure validation/helper functions that require no clock;
4. add package-level tests for legal/illegal profiles and all derived geometry;
5. do not instantiate Motor, Laser, Echo or TDC cores yet.

This gives later modules one stable vocabulary before any behavior is moved.
