# TDC-GPX LiDAR Controller v2 Architecture

> Status: Checkpoints A through D are complete. The sequential configuration
> calculator and atomic Processing/TDC configuration gateways are verified at
> 150/200 and 200/150 MHz. The unified CSR bank and integrated v2 functional
> datapath are not implemented yet.

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

## 5. Stage-1 Exit Criteria

Stage 1 is complete only when:

- every build, runtime, command, derived and status item has one named owner;
- every signal crossing has a source domain, destination domain and CDC class;
- every real-time event has payload, acceptance and latency semantics;
- module boundaries and RTL style satisfy `V2_RTL_STYLE_RULES.md`;
- the v1 comparison points and v2 migration order are fixed;
- no v1 functional RTL is modified as part of the architecture checkpoint.
