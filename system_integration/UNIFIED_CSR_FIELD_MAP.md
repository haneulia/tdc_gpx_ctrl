# Unified CSR Field Contract

## 1. Contract rules

The address owner constants and the field constants in
`rtl/lidar_unified_csr_pkg.vhd` are the numeric source of truth. This document
explains how software uses them.

- Multi-word configuration is staging data until its transaction token changes.
- Reset is an 8-bit epoch change, never a level that software must pulse.
- A configuration apply is an 8-bit epoch change shared by all adapters.
- Per-element table writes use a separate index/data command token.
- Software waits for the accepted epoch or command acknowledgement before reuse.
- Real-time Motor-to-Laser and Laser-to-TDC signals never pass through CSR.

## 2. System control

### CTL0 `SYS_CTRL`

| Bits | Name | Meaning |
|---:|---|---|
| 0 | `MOTOR_SIM_EN` | Select the embedded Virtual Encoder |
| 1 | `LASER_EN` | Permit Laser Controller firing |
| 2 | `LASER_STREAM_EN` | Enable Laser result descriptors |
| 3 | `ECHO_SIM_EN` | Select the synthesized Echo simulation path |
| 15:8 | `RESET_EPOCH` | A changed value requests one reset pulse in every enabled adapter |
| others | Reserved | Write zero |

### CTL1 `SYS_CFG_APPLY`

| Bits | Name | Meaning |
|---:|---|---|
| 7:0 | `CFG_EPOCH` | A changed value commits all stable adapter staging words |
| others | Reserved | Write zero |

Software must not reuse an epoch value until every participating adapter has
reported the previous value as accepted. An adapter that is busy keeps its old
accepted epoch and sets its rejection diagnostic.

## 3. Motor Decoder

### Control words

| Unified slot | Bits | Meaning |
|---|---:|---|
| CTL2 `MOTOR_CFG` | 15:0 | Encoder CPR |
|  | 16 | Direction: 0 forward/CW, 1 reverse/CCW |
|  | 18:17 | Decode mode: 00 x1, 01 x2, 10 x4 |
|  | 19 | Z early mode |
|  | 27:20 | AXIS valid-hold clocks |
| CTL3 `MOTOR_TICKS_LO` | 31:0 | Virtual Encoder low interval clocks |
| CTL4 `MOTOR_SCHED_LATENCY` | 14:0 | Fractional schedule high-count |
|  | 20:15 | Physical encoder-to-AXIS latency metadata |
|  | 26:21 | Virtual encoder-to-AXIS latency metadata |
| CTL5 `MOTOR_Z_PARAM` | 14:0 | Z offset |
|  | 29:15 | Z width |
| CTL6 `MOTOR_FACE_INDEX` | 2:0 | Face write index |
|  | 5:3 | Applied Face read index |
|  | 15:8 | Face write epoch; change after CTL7 is stable |
| CTL7 `MOTOR_FACE_GEOMETRY` | 14:0 | Face center |
|  | 29:15 | Face half-width |

The Motor adapter accepts a Face staging write when `FACE_WRITE_EPOCH` changes.
After all required faces are staged, software changes `SYS_CFG_APPLY.CFG_EPOCH`.
The complete Motor configuration and Face table then become one downstream
commit. `MOTOR_SIM_EN` is live system mode state; it is intentionally not
duplicated in `MOTOR_CFG`.

### Status words

| Unified slot | Meaning |
|---|---|
| STAT6 `MOTOR_STATUS` | Current Face, active/simulation/busy state, faults, applied decode mode, static face count and direction |
| STAT7 `MOTOR_FACE_GEOMETRY` | Selected applied center/half and valid bit |
| STAT8 `MOTOR_CFG_STATUS` | Accepted config/Face/reset epochs, read index, busy/apply/reject/valid flags |
| STAT9 `MOTOR_QUAD_INVALID` | Modulo-2^32 invalid quadrature transition count |
| STAT10 `MOTOR_AXIS_DROP` | Modulo-2^32 Motor AXIS source-drop count |
| STAT11 `MOTOR_REV_PERIOD` | Most recent complete Z-to-Z interval in Motor clocks; zero until valid |

## 4. Laser Controller

Unified CTL8..14 preserve the existing Laser CTL1..7 words bit-for-bit. The
former CTL0 fields move to `SYS_CTRL`, and its reset toggle becomes
`RESET_EPOCH`. All seven timing/schedule words are captured coherently when
`CFG_EPOCH` changes.

Unified STAT12..18 preserve existing Laser STAT0..4, STAT6 and STAT7 in that
order. The former standalone configuration acknowledgement is merged into the
system/adapter epoch status rather than consuming a Laser status slot.

## 5. Echo Receiver

### CTL15 `ECHO_DELAY_CMD`

| Bits | Meaning |
|---:|---|
| 4:0 | Channel index, 0..31 |
| 8 | Delay write toggle |
| 9 | Complete-profile apply toggle |

### CTL16 `ECHO_DELAY_DATA`

Bits 15:0 hold one delay in fixed 5 ns ticks. Echo simulation enable and reset
come from `SYS_CTRL`. STAT19..22 hold completed-shot rise/fall masks, aggregate
status and selected-channel readback.

## 6. TDC-GPX

CTL17..25 and STAT23..29 keep their address ownership from Stage 1. Their exact
indexed image command and pipeline fields are frozen during Stage 4, after the
current chip/pipeline CSR behavior is proven equivalent. No provisional field
layout in prose may be treated as a software contract before that gate passes.
