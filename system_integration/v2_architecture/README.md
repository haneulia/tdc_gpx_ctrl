# TDC-GPX LiDAR Controller v2 Architecture

> Status: Checkpoints A through G are complete. The unified CSR bank,
> sequential calculator and atomic Processing/TDC configuration gateways are
> route-verified at 150/200 and 200/150 MHz. The integrated v2 functional
> datapath is being migrated. Stage 2, Stage 3 / Checkpoint F and Stage 4 /
> Checkpoint G are closed. F0 through F5 cover B0 through B3; G covers B4,
> including 16/32-channel physical Echo, optional simulation and compact delay
> expansion. Stage 5 / Checkpoint H is closed: H0 froze the B5 oracle, H1 proved the
> typed GPX physical-bus wrapper, H2A proved the atomic Shot/result CDC
> gateways, and H2B-2A proved multi-Chip Shot coordination and registered
> result merge. H2B-2B proved the two-word indexed GPX image portal and
> physical all-chip activation ACK. H3 proved the complete 32-physical-STOP /
> 16-logical-APD, 7-Return B5 acquisition boundary under timeout, cap and
> backpressure in both routine clock profiles. Stage 6 is closed: I0 froze the
> Hit/Cell/Frame oracle, I1/I2/I2A close registered Hit/Cell ownership and I3/I4
> close Frame-lane assembly plus the complete B5..B8 path. Stage 7 is in
> progress. J3/J4 freeze PACKED17, J5/J6 close Shot/Hole serialization and
> width packing, and J7/J8 close the sequential VDMA profile plus ordered Face
> Footer. J9 DDR-versus-HTML Golden comparison is next.

## 1. Purpose

This directory defines the architecture contract for a clean v2 implementation
of the integrated LiDAR controller. The current v1 RTL remains the verified
reference and board fallback. v2 is built beside v1 and is compared against it
at observable boundaries; v1 is not refactored in place.

The redesign has four goals:

1. one owner for every physical configuration concept;
2. no CSR or CDC machinery inside functional cores;
3. deterministic real-time event paths separated from buffered data paths;
4. a configuration transaction that cannot leave domains on mixed versions.

## 2. Decision Summary

| ID | Decision |
|---|---|
| V2-D01 | Freeze v1 as the golden functional and board-recovery reference. |
| V2-D02 | Build v2 under a new top/entity/VLNV; do not change the v1 ABI in place. |
| V2-D03 | Keep one unified AXI4-Lite owner and remove local/unified CSR modes from v2 cores. |
| V2-D04 | Replace software-managed epoch/toggle commands with W1S command events and internal sequence IDs. |
| V2-D05 | Apply configuration with central validation and `PREPARE -> ACK -> ACTIVATE`. |
| V2-D06 | Use direct registered records for Motor-to-Laser real-time events; AXIS is for buffered data and monitoring. |
| V2-D07 | Perform CDC only at named domain gateways. |
| V2-D08 | Calculate geometry and timing derivatives in a commit-time sequential calculator, not in the shot data path. |
| V2-D09 | Preserve the Echo LVDS-to-GPX STOP path as an explicit low-latency exception. |
| V2-D10 | Retain proven GPX bus and data algorithms first, then replace their boundaries incrementally. |

## 3. Documents

| Document | Role |
|---|---|
| [V2_BASELINE.md](V2_BASELINE.md) | Frozen v1 reference, source metrics, hashes, preserved behavior and known limits |
| [V2_CONFIGURATION_OWNERSHIP.md](V2_CONFIGURATION_OWNERSHIP.md) | Build/runtime/derived classification and single-owner rules |
| [V2_CLOCK_EVENT_DATA_CONTRACT.md](V2_CLOCK_EVENT_DATA_CONTRACT.md) | Clock domains, CDC, real-time events and GPX data contracts |
| [V2_RTL_STYLE_RULES.md](V2_RTL_STYLE_RULES.md) | Module boundaries, sequential logic, timing and naming rules |
| [V2_MIGRATION_VERIFICATION_PLAN.md](V2_MIGRATION_VERIFICATION_PLAN.md) | Incremental implementation, regression and Git stage gates |
| [V2_CHECKPOINT_B_CONFIG_CONTRACT.md](V2_CHECKPOINT_B_CONFIG_CONTRACT.md) | Verified build/runtime/derived configuration contract and package-test result |
| [V2_CHECKPOINT_C_COMMIT_CALCULATOR.md](V2_CHECKPOINT_C_COMMIT_CALCULATOR.md) | Sequential validator/deriver implementation and 150/200 MHz timing result |
| [V2_CHECKPOINT_D_CONFIG_MANAGER.md](V2_CHECKPOINT_D_CONFIG_MANAGER.md) | Atomic commit manager, domain gateway, timeout, reset and CDC verification |
| [V2_CHECKPOINT_E_UNIFIED_CSR.md](V2_CHECKPOINT_E_UNIFIED_CSR.md) | Unified CSR, active readback, IRQ and dual-clock implementation result |
| [V2_UNIFIED_CSR_REGISTER_MAP.md](V2_UNIFIED_CSR_REGISTER_MAP.md) | Korean bit-level CTL/STAT/IRQ software ABI and defaults |
| [V2_STAGE3_F0_PROCESSING_ORACLE.md](V2_STAGE3_F0_PROCESSING_ORACLE.md) | Frozen B0..B3 event, safety and latency comparison contract for Checkpoint F |
| [V2_CHECKPOINT_F1_MOTOR_POSITION.md](V2_CHECKPOINT_F1_MOTOR_POSITION.md) | B0 motor position, virtual source, latency and 150/200 MHz route result |
| [V2_CHECKPOINT_F2_FACE_TRACKER.md](V2_CHECKPOINT_F2_FACE_TRACKER.md) | B1 Face membership, traversal, 1..5-Face and 150/200 MHz route result |
| [V2_CHECKPOINT_F3A_OPERATION_SAFETY.md](V2_CHECKPOINT_F3A_OPERATION_SAFETY.md) | RUN/STOP/ARM/DISARM, external permit and operation command CDC result |
| [V2_CHECKPOINT_F3B_SHOT_SCHEDULER.md](V2_CHECKPOINT_F3B_SHOT_SCHEDULER.md) | B2 angular lattice, busy-hole identity, chain integration and route result |
| [V2_CHECKPOINT_F4_LASER_EXECUTOR.md](V2_CHECKPOINT_F4_LASER_EXECUTOR.md) | B3 physical/simulation lifecycle, low-latency T0 bridge, safety and route result |
| [V2_CHECKPOINT_F5_PROCESSING_SUBSYSTEM.md](V2_CHECKPOINT_F5_PROCESSING_SUBSYSTEM.md) | Production B0..B3 assembly, monitor ABI, end-to-end latency, safe-point and dual-profile evidence |
| [V2_PROCESSING_PIPELINE_INTEGRATION_GUIDE.md](V2_PROCESSING_PIPELINE_INTEGRATION_GUIDE.md) | Human-review guide for module roles, record flow, assembly/disassembly and integration risks |
| [V2_STAGE4_G0_ECHO_ORACLE.md](V2_STAGE4_G0_ECHO_ORACLE.md) | Frozen B4 physical path, channel, Return and compact simulation-profile contract |
| [V2_CHECKPOINT_G_ECHO_FRONTEND.md](V2_CHECKPOINT_G_ECHO_FRONTEND.md) | Echo module roles, CH0/STEP expansion, build-option behavior and sign-off evidence |
| [V2_STAGE5_H0_GPX_ACQUISITION_ORACLE.md](V2_STAGE5_H0_GPX_ACQUISITION_ORACLE.md) | Frozen GPX bus, IFIFO ordering, status-pin and B5 raw-word oracle |
| [V2_CHECKPOINT_H1_GPX_BUS_ENGINE.md](V2_CHECKPOINT_H1_GPX_BUS_ENGINE.md) | Typed physical-bus wrapper, OEN modes, bus-timing width guard and implementation evidence |
| [V2_CHECKPOINT_H2A_GPX_EVENT_GATEWAYS.md](V2_CHECKPOINT_H2A_GPX_EVENT_GATEWAYS.md) | Atomic Shot/result payloads, SYNC/ASYNC CDC, backpressure and implementation evidence |
| [V2_STAGE5_H2B_ACQUISITION_PLAN.md](V2_STAGE5_H2B_ACQUISITION_PLAN.md) | H2B lane/coordinator decomposition, capture-width guard and Echo non-regression contract |
| [V2_CHECKPOINT_H2B1_GPX_ACQUISITION_LANE.md](V2_CHECKPOINT_H2B1_GPX_ACQUISITION_LANE.md) | Single-Chip lifecycle, image normalization, typed event ordering and 150/200 MHz evidence |
| [V2_CHECKPOINT_H2B2A_GPX_COORDINATOR.md](V2_CHECKPOINT_H2B2A_GPX_COORDINATOR.md) | Multi-Chip Shot broadcast, registered fair merge, terminal completion and 150/200 MHz evidence |
| [V2_CHECKPOINT_H2B2B_GPX_CONFIG_ACTIVATION.md](V2_CHECKPOINT_H2B2B_GPX_CONFIG_ACTIVATION.md) | Indexed GPX image portal, atomic snapshot, physical programming ACK and Echo non-regression evidence |
| [V2_CHECKPOINT_H3_GPX_ACQUISITION_SUBSYSTEM.md](V2_CHECKPOINT_H3_GPX_ACQUISITION_SUBSYSTEM.md) | Processing/TDC production assembly, 32 physical STOP / 16 logical APD x 7-Return B5 identity, cap/purge, CDC and implementation evidence |
| [V2_STAGE6_I0_HIT_CELL_FRAME_ORACLE.md](V2_STAGE6_I0_HIT_CELL_FRAME_ORACLE.md) | Frozen B6..B8 field, Return, Cell and Frame comparison contract |
| [V2_CHECKPOINT_I1_GPX_HIT_DECODER.md](V2_CHECKPOINT_I1_GPX_HIT_DECODER.md) | Registered B6 raw-to-Hit parser, topology diagnostics and route evidence |
| [V2_CHECKPOINT_I2_GPX_CELL_COLLECTOR.md](V2_CHECKPOINT_I2_GPX_CELL_COLLECTOR.md) | Width-independent B7 Return ownership, Cell storage, ordering, runtime Hit limit and route evidence |
| [V2_CHECKPOINT_I2A_GPX_PIPELINE_OPTIMIZATION.md](V2_CHECKPOINT_I2A_GPX_PIPELINE_OPTIMIZATION.md) | B6/B7 ownership cleanup, sequential metadata pipeline, linked 150/200 MHz timing and low-latency exceptions |
| [V2_CHECKPOINT_I3_GPX_FRAME_LANE_ASSEMBLER.md](V2_CHECKPOINT_I3_GPX_FRAME_LANE_ASSEMBLER.md) | Canonical Rise/Fall Frame-lane ordering, runtime topology and backpressure evidence |
| [V2_CHECKPOINT_I4_GPX_B5_B8_INTEGRATION.md](V2_CHECKPOINT_I4_GPX_B5_B8_INTEGRATION.md) | Complete B5..B8 data path, Face-close ownership, Hole completion and asynchronous-profile evidence |
| [V2_CHECKPOINT_J4_PACKED17_ABI_GEOMETRY.md](V2_CHECKPOINT_J4_PACKED17_ABI_GEOMETRY.md) | PACKED17 Cell ABI, Return policy and VDMA geometry functions |
| [V2_CHECKPOINT_J5A_T0_SHOT_METADATA.md](V2_CHECKPOINT_J5A_T0_SHOT_METADATA.md) | TDC measurement start reference time (T0) and exact Shot Metadata stream |
| [V2_CHECKPOINT_J5B_HOLE_LINES.md](V2_CHECKPOINT_J5B_HOLE_LINES.md) | Explicit leading, interior, trailing and all-Hole geometric Lines |
| [V2_CHECKPOINT_J6_AXIS_WORD_PACKER.md](V2_CHECKPOINT_J6_AXIS_WORD_PACKER.md) | Sole 32/64/128-bit AXIS packer and final-Beat padding contract |
| [V2_CHECKPOINT_J7_J8_VDMA_PROFILE_FOOTER.md](V2_CHECKPOINT_J7_J8_VDMA_PROFILE_FOOTER.md) | Sequential VDMA profile, fixed STRIDE, Face-boundary activation and ordered Footer |

## 4. Target Top-Level Shape

```text
AXI4-Lite
    |
    +-- lidar_csr_bank
    +-- lidar_config_manager
    |       +-- proc_config_gateway
    |       +-- tdc_config_gateway
    |       +-- commit_calculator
    |
    +-- lidar_fault_manager -- IRQ

Encoder -> motor_position_core -> face_tracker -> shot_scheduler
                                             -> laser_executor
                                                | fire_pulse
fire_done --------------------------------------| start_tdc / stop_tdc

LVDS Echo -> echo_stop_frontend -----------------> physical GPX STOP

GPX 28-bit bus -> gpx_bus_engine -> gpx_hit_decoder -> gpx_cell_builder
               -> gpx_frame_builder -> gpx_axis_formatter -> VDMA/DDR
```

The Stage-to-Checkpoint table in `V2_MIGRATION_VERIFICATION_PLAN.md` is the
authoritative migration status. Checkpoint letters in completed result files
must not be renumbered.

## 5. Stage-1 Exit Criteria (Closed)

Stage 1 is complete only when:

- every build, runtime, command, derived and status item has one named owner;
- every signal crossing has a source domain, destination domain and CDC class;
- every real-time event has payload, acceptance and latency semantics;
- module boundaries and RTL style satisfy `V2_RTL_STYLE_RULES.md`;
- the v1 comparison points and v2 migration order are fixed;
- no v1 functional RTL is modified as part of the architecture checkpoint.
