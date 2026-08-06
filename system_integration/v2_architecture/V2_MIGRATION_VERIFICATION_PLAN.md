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
- compact simulation delay profile using CTL20 `CH0_DELAY + channel * STEP`;
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
| 3 | F | Complete | `V2_CHECKPOINT_F5_PROCESSING_SUBSYSTEM.md`; session `260804_f5_busy_opt_v2_processing_subsystem` | Processing event pipeline |
| 4 | G | Complete | `V2_CHECKPOINT_G_ECHO_FRONTEND.md`; sessions `260805_stage4_g_echo_final_v2_echo`, `260805_stage4_g_ch0_step_csr_v2_unified_csr` | Echo frontend |
| 5 | H | Complete | `V2_CHECKPOINT_H3_GPX_ACQUISITION_SUBSYSTEM.md`; sessions `260805_h3_capacity_fix_sim_v2_gpx_acquisition_subsystem`, `260805_h3_capacity_fix_impl_v2_gpx_acquisition_subsystem` | Proven GPX bus/acquisition wrapper |
| 6 | I | Complete | `V2_CHECKPOINT_I4_GPX_B5_B8_INTEGRATION.md`; sessions `260806_i4_final_order_v2_gpx_b5_b8_subsystem`, `260806_i4_all_dual_sim_v2_gpx_b5_b8_subsystem` | Hit, Cell and Frame pipeline |
| 7 | J | Complete | J0-J10; `V2_CHECKPOINT_J10_PS_HLINE_ETHERNET.md` | AXIS/VDMA/PS formatter chain |
| 8 | K | In progress | K0 plan; `V2_STAGE8_K0_INTEGRATED_TOP_PLAN_KO.md` | Full RTL integration and HTML alignment |
| 9 | L | Pending | Parent/board evidence pending | Implementation, board sign-off and release tag |

**Current migration state:** Stage 2 is closed at Checkpoint E, Stage 3 is
closed at Checkpoint F and Stage 4 is closed at Checkpoint G. F0a/F0b through
F5 cover B0..B3 and G covers the B4 physical/synthetic Echo boundary. Stage 5
is closed at Checkpoint H: H0 froze the oracle, H1 proved the typed physical-bus wrapper,
H2A proved the atomic command/result CDC boundary, H2B-0 closed the capture-
counter width, H2B-1 proved one typed acquisition lane, H2B-2A proved the
multi-Chip coordinator and H2B-2B closed the indexed GPX image portal plus
physical all-Chip programming ACK. H3 then closed the B5 end-to-end boundary
for 32 physical GPX STOP lanes representing 16 logical APD channels, normal
full-capacity drain, timeout, cap/purge and backpressure at
150/200 and 200/150 MHz. Stage 6 / Checkpoint I is now complete: I0 froze
the B6..B8 oracle, I1 completed B6, I2 completed width-independent B7 Cell
collection, I2A closed B6/B7 Return ownership plus sequential timing
optimization, I3 completed canonical Rise/Fall B8 Frame-lane assembly, and I4
closed the production B5..B8 chain plus explicit Face-close ownership for
trailing and all-hole completion. Stage 7 / Checkpoint J is in progress. J0
through J2 provide the original B9 oracle, serializer and lane-formatter
baseline. J3 froze the revised PACKED17 Shot-Line/Face-Footer ABI and HTML
Golden schema. J4 implemented the target geometry functions, exact Cell
Metadata map, and Return-filter/physical-overflow ownership. J5A/J5B close the
T0/Shot Metadata stream and explicit geometric Hole Lines. J6 closes
target-width AXIS packing. J7/J8 now close the 32-byte Face Footer, fixed
maximum STRIDE, sequential profile calculation and acknowledged Face-boundary
activation. J9 closes the STRIDE-aware DDR image comparison against the
executable HTML Golden model. J10 now closes the portable C PS H-Line decoder,
Viewer Ethernet byte comparison, ownership guards and Cortex-A9 build. This
closes Checkpoint J. The only valid next step is **Stage 8 K0 synthesisable v2
top assembly**, followed by K1 full RTL/HTML alignment. Parent VDMA/HP/cache is
Stage 9 L0 and must not begin against the current v1 parent IP.
Checkpoint K or a later stage must not be treated as migrated merely because
its v1 implementation exists or because an intermediate J sub-step passed.

Commit only after the focused sub-step passes. Intermediate commits inside a
Checkpoint are allowed, but the Checkpoint result document and status row move
to Complete only after its complete Stage gate passes.

Never stage unrelated user changes, generated Vivado work directories, wave
databases or transient logs. Each archived sign-off session contains only the
scenario, source manifest, compact results, final logs and required waveforms.

## 6. Completed RTL Work Package: Stage 3 / Checkpoint F

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

Final sub-step status:

| Step | Status | Evidence |
|---:|---|---|
| F0a/F0b | Complete | `V2_STAGE3_F0_PROCESSING_ORACLE.md` |
| F1 | Complete | `V2_CHECKPOINT_F1_MOTOR_POSITION.md`; session `260804170100_v2_motor_position` |
| F2 | Complete | `V2_CHECKPOINT_F2_FACE_TRACKER.md`; session `260804184000_v2_face_tracker` |
| F3a | Complete | `V2_CHECKPOINT_F3A_OPERATION_SAFETY.md`; sessions `260804203000_v2_operation`, `260804204000_v2_unified_csr` |
| F3b | Complete | `V2_CHECKPOINT_F3B_SHOT_SCHEDULER.md`; session `260804211500_v2_shot_scheduler` |
| F4 | Complete | `V2_CHECKPOINT_F4_LASER_EXECUTOR.md`; follow-up session `260804_f4_busy_opt_v2_laser_executor` |
| F5 | Complete | `V2_CHECKPOINT_F5_PROCESSING_SUBSYSTEM.md`; session `260804_f5_busy_opt_v2_processing_subsystem` |

Rules for this package:

1. `lidar_csr_config_subsystem` remains the only AXI4-Lite owner.
2. Each core consumes the active Processing-domain configuration record; it
   does not reinterpret CSR words.
3. Motor-to-Laser control events use direct registered records. AXIS is a
   monitor copy and cannot apply backpressure to `fire_pulse` or `start_tdc`.
4. Each step started only after the preceding boundary comparison passed.
5. Checkpoint F closed only after 150/200 and 200/150 MHz regressions passed with
   zero inferred latches and no new unclassified CDC path.
6. F1 was gated by the F0b package/calculator/manager/CSR regressions, and F3b
   was gated by F3a ownership of every operation and laser-permit state. Those
   gates are now closed; the invariant remains that a scheduler must never
   infer safety permission from configuration validity alone.

## 7. Completed RTL Work Package: Stage 4 / Checkpoint G

Checkpoint G moves only B4 Echo frontend. It must not pull the GPX bus engine,
Hit decoder or formatter forward from later Stages.

Required order:

1. freeze the v1 LVDS-to-STOP edge, channel and latency oracle;
2. define the physical direct path and a separately generated simulation path;
3. implement all 16 APD channels without CSR or AXIS backpressure in the STOP
   path;
4. attach F5 shot identity as observation/context without delaying STOP;
5. verify Return 1 through 7, channel ordering and production removal of the
   simulation generator;
6. run the two routine 150/200 and 200/150 MHz profiles before closing G.

All six steps are complete. The compact CTL20 profile replaces the originally
planned 32-entry indexed table, so one atomic commit supplies all 32 channels
without an Echo-specific command FSM. Checkpoint G consumes F5
`shot_start_event_t`, but does not claim that the event has crossed into the
GPX acquisition domain.

## 8. Completed RTL Work Package: Stage 5 / Checkpoint H

Checkpoint H wraps the proven v1 GPX bus engine behind typed command/result
records. The first pass preserves its physical bus FSM and verifies B5 before
any algorithmic cleanup. Echo diagnostics remain observation-only and must not
be reinterpreted as IFIFO occupancy or GPX completion.

Required order and final status:

1. **H0 complete:** freeze v1 bus/acquisition sources, pin semantics, IFIFO
   order and the 28-bit B5 word contract;
2. **H1 complete:** add `gpx_bus_request_t`, `gpx_bus_response_t` and
   `gpx_pin_status_t`, then prove cycle/pin equivalence against a direct v1 PHY;
3. **H2A complete:** add named Processing-to-TDC command and
   TDC-to-Processing result gateways, with direct registered SYNC and
   handshake/FIFO ASYNC paths;
4. **H2B-0 complete:** reject a derived capture window above the proven
   16-bit GPX watchdog range instead of silently truncating it;
5. **H2B-1 complete:** preserve `tdc_gpx_chip_run` IFIFO1/IFIFO2 drain ordering
   behind a typed single-Chip acquisition lane;
6. **H2B-2A complete:** broadcast one accepted Shot to every active lane,
   merge typed results through registered lane ingress slots and complete only
   after all active terminal events cross the output handshake;
7. **H2B-2B complete:** provide the CTL21/22 indexed GPX image portal and
   defer TDC-domain ACTIVATE ACK until every physically present Chip completes
   programming. CTL20 remains the compact Echo `CH0 + channel * STEP` profile;
8. **H3 complete:** compare B5 `chip + IFIFO + raw_28 + shot` identity under
   status changes, timeout, cap and output backpressure;
9. **H3 complete:** routine 150/200 and 200/150 MHz integration profiles passed
   with non-negative WNS, zero latches, zero Critical CDC and zero blocking DRC.

H3 capacity correction distinguishes physical STOP lanes from logical APD
channels. The default dedicated 2-Rise/2-Fall topology has 4 Chips x 8 STOP =
32 physical lanes and 16 logical APD channels. Each IFIFO owns four STOP lanes,
so its cap is 28 words on a single-slope Chip and 56 words on a dual-edge Chip.
The result FIFO depth is derived from those build masks; no runtime CSR or new
generic was added.

H1 evidence is recorded in `V2_CHECKPOINT_H1_GPX_BUS_ENGINE.md` and H2A
evidence is recorded in `V2_CHECKPOINT_H2A_GPX_EVENT_GATEWAYS.md`. The central
CSR keeps a 6-bit `BUS_TICKS` field for ABI stability, but commit validation now
accepts only 1..7 because the proven physical PHY owns a 3-bit input. Values
above 7 are rejected before PREPARE and cannot be silently truncated.
The H2B acquisition decomposition and the 16-bit capture-window evidence are
recorded in `V2_STAGE5_H2B_ACQUISITION_PLAN.md`.
The single-Chip lane implementation and evidence are recorded in
`V2_CHECKPOINT_H2B1_GPX_ACQUISITION_LANE.md`.
The multi-Chip coordinator implementation and evidence are recorded in
`V2_CHECKPOINT_H2B2A_GPX_COORDINATOR.md`.
The indexed image transaction, deferred physical activation and evidence are
recorded in `V2_CHECKPOINT_H2B2B_GPX_CONFIG_ACTIVATION.md`.
The production acquisition assembly, topology-sized result FIFO, cap/purge
behavior and final B5 evidence are recorded in
`V2_CHECKPOINT_H3_GPX_ACQUISITION_SUBSYSTEM.md`.

## 9. Completed RTL Work Package: Stage 6 / Checkpoint I

Checkpoint I converts the verified B5 raw stream into canonical Hit, Cell and
Frame records. The order remains fixed:

1. **I0 complete:** freeze the v1 B6..B8 source hashes, bit meanings and
   intentional correction list;
2. **I1/B6 complete:** decode all I-Mode fields into `gpx_hit_event_t`, validate
   build topology, and prove dedicated plus dual-edge field preservation;
3. **I2/B7 complete:** own Return order by Chip/STOP/slope, collect typed Hits
   into width-independent Cell records, preserve Hit bit 16 and apply runtime
   visible-Hit capacity;
4. **I2A complete:** remove duplicate Return state from B6, pipeline B6/B7
   ready/context/RAM paths and prove the directly linked boundary at 150/200 MHz;
5. **I3/B8 complete:** assemble slope lanes and Frame ordering without pulling
   AXIS width or VDMA padding into canonical geometry; prove dedicated,
   one-Chip dual-edge, Falling-OFF and independent lane backpressure at
   150/200 MHz;
6. **I4 complete:** own the explicit Face-close event, integrate B5 through
   B8, prove dedicated and four-Chip all-dual exact payloads, then pass the
   150/200 and 200/150 MHz profiles.

I2A removes the v1-style intermediate Return metadata from B6. B7 is the single
owner of the Chip + STOP + slope Cell count, so a one-Chip dual-edge build has
independent Rise/Fall Return 0..6 sequences. The eighth Return reaches B7 and
is consumed and diagnosed without a wrapped output event.

I0 is documented in `V2_STAGE6_I0_HIT_CELL_FRAME_ORACLE.md`; I1 structure,
tests, resource use and timing are documented in
`V2_CHECKPOINT_I1_GPX_HIT_DECODER.md`. I2 Cell storage, ordering, throughput,
Echo non-regression and route evidence are documented in
`V2_CHECKPOINT_I2_GPX_CELL_COLLECTOR.md`. The ownership cleanup, sequential
pipeline and linked timing evidence are documented in
`V2_CHECKPOINT_I2A_GPX_PIPELINE_OPTIMIZATION.md`. I3 canonical line ordering,
runtime slope-mask closure, gap semantics and timing evidence are documented in
`V2_CHECKPOINT_I3_GPX_FRAME_LANE_ASSEMBLER.md`. I4 Face-close ownership,
integrated B5..B8 data flow, timing optimization and final evidence are
documented in `V2_CHECKPOINT_I4_GPX_B5_B8_INTEGRATION.md`.

## 10. In-Progress RTL Work Package: Stage 7 / Checkpoint J

Checkpoint J converts the canonical B8 Cell lanes into the VDMA byte contract.
Its current sub-step status is:

| Step | Status | Evidence |
|---:|---|---|
| J0 | Complete | `V2_STAGE7_J0_AXIS_VDMA_ORACLE.md` |
| J1 | Complete | `V2_CHECKPOINT_J1_GPX_CELL_WORD_SERIALIZER.md`; session `260806_j1_cell_word_v2_v2_gpx_cell_word_serializer` |
| J2 | Complete | `V2_CHECKPOINT_J2_GPX_VDMA_LANE_FORMATTER.md`; sessions `260806_j2_abort_ready_v2_gpx_vdma_lane_formatter`, `260806_j2_final_impl_v2_gpx_vdma_lane_formatter` |
| J3 | Complete | `C08_HDL_HTML_Alignment_260806_PACKED17_VDMA_ABI_Contract_v041.md`; HTML v026 Golden model |
| J4 | Complete | `V2_CHECKPOINT_J4_PACKED17_ABI_GEOMETRY.md`; sessions `260806_j4_contract_r2_v2_gpx_cell_collector`, `260806_j4_contract_v2_gpx_cell_word_serializer` |
| J5A | Complete | T0 capture, exact 16-byte Shot Metadata, and width-independent real-Shot line stream; `V2_CHECKPOINT_J5A_T0_SHOT_METADATA.md` |
| J5B | Complete | Explicit leading/interior/trailing/all-Hole Line expansion; `V2_CHECKPOINT_J5B_HOLE_LINES.md` |
| J6 | Complete | Target 32/64/128 packer with final-Beat-only padding; `V2_CHECKPOINT_J6_AXIS_WORD_PACKER.md` |
| J7 | Complete | 32-byte Face Footer and fixed maximum STRIDE; `V2_CHECKPOINT_J7_J8_VDMA_PROFILE_FOOTER.md` |
| J8 | Complete | Sequential profile calculation and acknowledged Face-boundary HSIZE/VSIZE activation; `V2_CHECKPOINT_J7_J8_VDMA_PROFILE_FOOTER.md` |
| J9 | Complete | XSIM DDR image versus executable HTML Golden Vector, every allocated Word; `V2_CHECKPOINT_J9_DDR_HTML_GOLDEN.md` |
| J10 | Complete | Portable C PS H-Line decoder and HTML Ethernet packet byte comparison; `V2_CHECKPOINT_J10_PS_HLINE_ETHERNET.md`; sessions `260807_j10_full2_v2_gpx_ps_hline`, `260807_j10_final_v2_gpx_ps_hline` |

J2 remains useful as a registered transport and backpressure baseline, but its
repeated 48-byte prefix is not the target ABI. J3/J4 replace that geometry with
`16-byte Shot Metadata + continuous Cells + final-Beat padding`, followed by a
32-byte Face Footer. J5A closes the real-Shot Metadata/T0 path; J5B closes
explicit leading, interior, trailing and all-Hole Line expansion. J7/J8 append
the ordered Footer and activate a precomputed VDMA profile only after the safe
Face boundary and VDMA acknowledgement. J9 compares the resulting DDR image
against the executable HTML Golden model, and J10 decodes those exact captures
into byte-exact Viewer Face Header and H-Line packets with the portable C PS
reference implementation.
Checkpoint J closes at J10. Stage 8 K owns synthesisable top integration and
the complete RTL/HTML operating matrix; Stage 9 L owns parent implementation,
DMA/cache ownership and board release evidence.
