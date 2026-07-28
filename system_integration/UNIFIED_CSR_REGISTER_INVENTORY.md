# Unified CSR Register Inventory

## 1. Count reconciliation

The current IP-local banks allocate 154 32-bit addresses when every configured
CTL, STAT, and IRQ slot is counted.

| Existing bank | CTL | STAT | IRQ | Allocated |
|---|---:|---:|---:|---:|
| Motor Decoder | 8 | 8 | 4 | 20 |
| Laser Controller | 8 | 8 | 4 | 20 |
| Echo Receiver | 17 | 5 | 4 | 26 |
| TDC-GPX chip CSR | 32 | 32 | 4 | 68 |
| TDC-GPX pipeline CSR | 8 | 8 | 4 | 20 |
| Total | 73 | 61 | 20 | 154 |

The target uses one `my_axil_csr32_top` geometry.

| Unified bank | Active | Reserved | Address slots |
|---|---:|---:|---:|
| CTL | 26 | 6 | 32 |
| STAT | 30 | 2 | 32 |
| IRQ registers | 4 | 0 | 4 |
| Total | 60 | 8 | 68 |

Reserved slots keep the ABI stable. They are not justification for crossing all
32 CTLs through every processing clock domain. Each adapter transfers only its
owned payload.

## 2. Per-IP disposition

### Motor Decoder

| Existing item | Target disposition |
|---|---|
| CTL0/CTL3 control fields | Repacked into `MOTOR_CFG`; global reset/apply moves to System CTL |
| CTL1 encoder ticks | `MOTOR_TICKS_LO` |
| CTL2 fractional schedule | `MOTOR_SCHED_LATENCY` with physical/virtual latency fields |
| CTL4 Z offset/width | `MOTOR_Z_PARAM` |
| CTL5/6/7 face staging | `MOTOR_FACE_INDEX` plus one 30-bit center/half geometry word |
| STAT1/2 selected center/half | Combined selected-face geometry readback |
| STAT3/4 apply/CDC acknowledgement | One config-status/epoch word |
| STAT5/6 diagnostic counters | Preserved independently |
| STAT7 reserved | Used for measured revolution period |

Target contribution: 6 CTL and 6 STAT.

### Laser Controller

| Existing item | Target disposition |
|---|---|
| CTL0 enable/reset/apply | Enable bits in System control; reset/apply use common transaction token |
| CTL1..7 timing/schedule | Seven Laser CTLs, field widths preserved |
| STAT0..4 metrics/status | Preserved |
| STAT5 config acknowledgement | Merged into System config epoch/status |
| STAT6 revolution counts | Preserved |
| STAT7 timeout count | Preserved |

Target contribution: 7 CTL and 7 STAT.

### Echo Receiver

| Existing item | Target disposition |
|---|---|
| CTL0 SIM enable/reset | SIM mode and reset move to System control |
| CTL1..16 direct delays | Indexed `ECHO_DELAY_CMD` and `ECHO_DELAY_DATA` staging window |
| STAT0 32-channel countdown mask | Removed from software ABI; selected/aggregate live state moves to `ECHO_STATUS` |
| STAT1 rise OR fall mask | Removed because it equals rise mask OR fall mask |
| STAT2/3 live shot masks | Replaced by stable last-completed-shot rising/falling masks |
| STAT4 sticky/live state | Preserved and extended with config sequence/busy |
| Delay verification | New selected-channel delay readback |

Target contribution: 2 CTL and 4 STAT. Independent 32 x 16-bit delay storage
still exists in simulation-capable builds; the CSR address count and CDC width
are what become smaller.

### TDC-GPX

| Existing item | Target disposition |
|---|---|
| Chip CTL1/3/4/21 | Four dedicated unified CTLs |
| Chip CTL5..20 16-word image | Indexed image command/data staging window |
| Pipeline CTL0..2 | Three dedicated unified CTLs |
| Chip STAT0..3 | Four per-chip read-result STATs |
| Pipeline STAT0..4 constants | System capability/geometry STATs |
| Pipeline STAT5..7 | Three runtime pipeline STATs |
| Unused chip/pipeline slots | Removed from ownership, reserved unified slots read zero |

Target contribution: 9 CTL and 7 TDC runtime STATs. Five static TDC geometry
values are counted in the six System STAT slots.

## 3. Required compatibility behavior

During migration, local CSR mode remains the reference behavior. Its adapter
must produce the same internal configuration/status contract as unified mode.
The old and new software addresses do not need to coexist inside one bank, but
the following behavior must compare equal in integration tests:

- accepted configuration values and apply boundary;
- shot and Face ownership;
- Laser/TDC timing;
- rising/falling channel and sticky diagnostics;
- raw 28-bit GPX I-Mode words and reconstructed 17-bit Hit values;
- VDMA geometry and complete-line accounting;
- interrupt causes and clear lifecycle.

An indexed table write is not complete merely because AXI returned `OKAY`.
Software must wait for the adapter acknowledgement/epoch before issuing the
profile apply command.
