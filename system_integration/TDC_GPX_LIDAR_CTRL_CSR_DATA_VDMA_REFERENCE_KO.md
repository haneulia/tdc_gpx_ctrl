# TDC-GPX LiDAR 통합 제어기 CSR·데이터 경로·VDMA 상세 참조서

> 영문 기술 참조서: [TDC_GPX_LIDAR_CTRL_CSR_DATA_VDMA_REFERENCE.md](TDC_GPX_LIDAR_CTRL_CSR_DATA_VDMA_REFERENCE.md)

## 1. 문서 목적과 적용 기준

이 문서는 통합 `tdc_gpx_lidar_ctrl` 제어 경로와 `tdc_gpx_top` 데이터
경로를 신호처리 엔지니어와 소프트웨어 엔지니어가 같은 의미로 해석하기
위한 기준 문서이다. 다음 세 가지 내용을 하나의 시간순 흐름으로 설명한다.

1. 통합 CSR의 32-bit 레지스터마다 어떤 비트가 어떤 의미를 갖는가?
2. 외부 TDC-GPX의 28-bit I-Mode 데이터가 어떤 경로와 비트 배치를 거쳐
   DDR에 저장되는가?
3. VDMA의 수평·수직 동기가 물리 신호와 논리 데이터 구조에서 어떻게
   구분되며, HSIZE와 VSIZE는 무엇을 뜻하는가?

RTL 기준점은 커밋 `c4d36a7`의 다음 원본이다.

- `system_integration/rtl/lidar_unified_csr_pkg.vhd`
- `system_integration/rtl/lidar_unified_csr_top.vhd`
- `tdc_gpx_unified_csr_adapter.vhd`
- `tdc_gpx_decoder_i_mode.vhd`
- `tdc_gpx_raw_event_builder.vhd`
- `tdc_gpx_cell_builder.vhd`
- `tdc_gpx_face_assembler.vhd`
- `tdc_gpx_line_packer.vhd`
- `tdc_gpx_header_inserter.vhd`
- `tdc_gpx_top.vhd`

GPX 원시 데이터 해석은 현재 RTL의 SINGLE_SHOT I-Mode와 로컬
`Doc/TDC-GPX-Datasheet.pdf` 2.4절의 데이터 구조를 기준으로 한다.
레지스터명과 필드명은 VHDL 및 Vivado에서 직접 검색할 수 있도록 영문
원형을 유지한다. 설명과 운용 판단은 한국어로 제공한다.

## 2. 표기법과 공통 접근 규칙

| 표기 | 의미 |
|---|---|
| `RW` | 소프트웨어 읽기/쓰기 |
| `RO` | 소프트웨어 읽기 전용 |
| `RW1C` | 1을 쓰면 클리어되고 0을 쓰면 변화 없음 |
| `Live` | 설정 commit 없이 즉시 적용되는 값 |
| `Epoch` | 마지막 승인값과 다른 epoch를 쓸 때 한 번만 동작 |
| `Sticky` | 지정된 clear 조건까지 오류 이력을 유지 |
| `wrap` | 최대값 이후 0으로 순환하는 카운터 |
| `Reserved` | 현재 미사용. 소프트웨어는 0을 써야 함 |

모든 주소는 Vivado Address Editor가 IP에 할당한 AXI4-Lite 기준 주소에
더하는 byte offset이다. 모든 레지스터는 32-bit이고 4-byte 정렬이다.

### 2.1 고정 주소 구조

| 영역 | Offset | 개수 | 접근 |
|---|---:|---:|---|
| `CTL0..CTL31` | `0x000..0x07C` | 32 | RW |
| `STAT0..STAT31` | `0x080..0x0FC` | 32 | RO |
| `INTR_EN` | `0x100` | 1 | RW |
| `INTR_STATUS` | `0x104` | 1 | RO |
| `INTR_FLAG` | `0x108` | 1 | RW1C |
| `INTR_MODE` | `0x10C` | 1 | RW |

CSR bank가 reset되면 모든 CTL과 interrupt 레지스터 readback은
`0x00000000`이 된다. 그러나 Motor, Laser, TDC adapter의 실제 active
설정은 안전한 generic 초기값을 유지할 수 있다. 따라서 CTL이 0으로
읽힌다는 이유만으로 실제 처리 설정도 모두 0이라고 판단하면 안 된다.
운용 전에는 필요한 staging CTL을 모두 쓴 뒤 `CFG_EPOCH`를 변경해야 한다.

### 2.2 5 ns tick 시간 규칙

이름이 `*_5NS_TICKS`인 필드는 클럭 개수가 아니라 5 ns 고정 시간 단위다.

```text
설정 시간(ns) = CSR 값 * 5 ns
로컬 클럭 수 = ceil(CSR 값 * 로컬 클럭 주파수(MHz) / 200)
```

예를 들어 288 ticks는 처리 클럭이 50/100/125/150/200 MHz 중 무엇이든
항상 1,440 ns를 의미한다. 각 adapter가 소비하는 클럭 도메인에 맞춰
클럭 수로 변환한다. 필드명에 `clocks` 또는 `states`가 명시된 값은 이
5 ns 변환 대상이 아니다.

## 3. 제어 레지스터 CTL0~CTL31

## 3.1 시스템 설정 transaction

### CTL0 `SYS_CTRL` - `0x000`, RW, reset `0x00000000`

| Bits | Field | 종류 | 의미 |
|---:|---|---|---|
| `[0]` | `MOTOR_SIM_EN` | Live | `1`: 내부 가상 엔코더, `0`: 외부 물리 A/B/Z decoder |
| `[1]` | `LASER_EN` | Live | 전역 레이저 발사 허용. 다른 안전 조건은 계속 적용됨 |
| `[2]` | `LASER_STREAM_EN` | Live | Laser 결과 AXI stream 활성 |
| `[3]` | `ECHO_SIM_EN` | Live | 합성된 경우 Echo simulation 경로 활성 |
| `[7:4]` | Reserved | - | 0으로 기록 |
| `[15:8]` | `RESET_EPOCH` | Epoch | 마지막 승인값과 다른 값으로 바꾸면 각 adapter에 reset 1회 요청 |
| `[31:16]` | Reserved | - | 0으로 기록 |

### CTL1 `SYS_CFG_APPLY` - `0x004`, RW, reset `0x00000000`

| Bits | Field | 종류 | 의미 |
|---:|---|---|---|
| `[7:0]` | `CFG_EPOCH` | Epoch | staging된 Motor, Laser, TDC 설정을 adapter별 coherent snapshot으로 commit |
| `[31:8]` | Reserved | - | 0으로 기록 |

권장 순서는 관련 CTL을 모두 기록하고 `STAT31.ANY_BUSY=0`을 확인한 뒤
`CFG_EPOCH`를 증가시키고 `STAT31.ALL_CFG_ACCEPTED=1`을 기다리는 것이다.
Echo delay profile은 CTL15/16의 별도 toggle/acknowledge 방식을 사용한다.

## 3.2 Motor Decoder와 Virtual Encoder

### CTL2 `MOTOR_CFG` - `0x008`, RW

| Bits | Field | 단위/인코딩 | 의미 |
|---:|---|---|---|
| `[15:0]` | `CPR` | count/rev | Encoder pulse/revolution. 0은 허용하지 않으며 build 상한 이하여야 함 |
| `[16]` | `ENC_DIR` | `0/1` | `0`: Forward/CW 극성, `1`: Reverse/CCW 극성 보정 |
| `[18:17]` | `DEC_MODE` | `00/01/10` | Quadrature x1/x2/x4. `11`은 잘못된 설정 |
| `[19]` | `Z_EARLY` | Boolean | Z/index early 동작 방식 선택 |
| `[27:20]` | `AXIS_VALID_HOLD` | Motor clocks | Motor AXIS 위치 event를 valid로 유지할 클럭 수 |
| `[31:28]` | Reserved | - | 0으로 기록 |

### CTL3 `MOTOR_TICKS_LO` - `0x00C`, RW

| Bits | Field | 단위 | 의미 |
|---:|---|---|---|
| `[31:0]` | `TICKS_LO` | Motor clocks/state | 가상 엔코더의 짧은 state 간격. 0은 허용하지 않음 |

가상 엔코더 scheduler는 `TICKS_LO`와 `TICKS_LO+1`을 분배하므로 runtime
`TICKS_HI` 레지스터는 필요하지 않다.

### CTL4 `MOTOR_SCHED_LATENCY` - `0x010`, RW

| Bits | Field | 단위 | 의미 |
|---:|---|---|---|
| `[14:0]` | `HI_COUNT` | states/rev | 한 회전 중 `TICKS_LO+1`을 사용할 state 수. `CPR*4`보다 작아야 함 |
| `[20:15]` | `PHYS_AXIS_LATENCY` | Motor clocks | 물리 encoder 입력부터 Motor AXIS까지 측정된 지연 metadata |
| `[26:21]` | `VIRT_AXIS_LATENCY` | Motor clocks | 가상 encoder부터 Motor AXIS까지 측정된 지연 metadata |
| `[31:27]` | Reserved | - | 0으로 기록 |

### CTL5 `MOTOR_Z_PARAM` - `0x014`, RW

| Bits | Field | 단위 | 의미 |
|---:|---|---|---|
| `[14:0]` | `Z_OFFSET` | decoded states | Index pulse 위치 offset |
| `[29:15]` | `Z_WIDTH` | decoded states | Index pulse 폭 |
| `[31:30]` | Reserved | - | 0으로 기록 |

### CTL6 `MOTOR_FACE_INDEX` - `0x018`, RW

| Bits | Field | 종류 | 의미 |
|---:|---|---|---|
| `[2:0]` | `FACE_WRITE_INDEX` | index | CTL7 데이터를 기록할 Face 0~4 선택 |
| `[5:3]` | `FACE_READ_INDEX` | index | STAT7에서 확인할 Face 0~4 선택 |
| `[7:6]` | Reserved | - | 0으로 기록 |
| `[15:8]` | `FACE_WRITE_EPOCH` | Epoch | CTL7이 안정된 뒤 변경하여 Face 설정 1회 commit |
| `[31:16]` | Reserved | - | 0으로 기록 |

### CTL7 `MOTOR_FACE_GEOMETRY` - `0x01C`, RW

| Bits | Field | 단위 | 의미 |
|---:|---|---|---|
| `[14:0]` | `FACE_CENTER` | decoded states | 현재 decode 배율 기준 Face 중심 위치 |
| `[29:15]` | `FACE_HALF_WIDTH` | decoded states | 중심을 기준으로 한 공통 활성 반폭 |
| `[30]` | Reserved | - | 과거 상수명은 `FACE_VALID`지만 현재 adapter가 소비하지 않으므로 0 기록 |
| `[31]` | Reserved | - | 0으로 기록 |

CPR이 0/상한 초과, DEC_MODE가 `11`, `TICKS_LO=0`, 또는
`HI_COUNT >= CPR*4`이면 Motor 설정이 거부된다. Runtime에 decode 배율을
바꾸면 모든 활성 Face의 center/half-width도 새로운 decoded-state 단위로
다시 계산하여 기록해야 한다.

## 3.3 Laser Controller

### CTL8 `LASER_FIRE_CFG` - `0x020`, RW

| Bits | Field | 단위 | 의미 |
|---:|---|---|---|
| `[15:0]` | `FIRE_WIDTH` | 5 ns ticks | 실제 `fire_pulse` 폭. 0이면 발사 차단 오류 |
| `[31:16]` | `FIRE_DONE_TIMEOUT` | 5 ns ticks | 동기화된 `fire_done` 최대 대기시간. 0이 아니고 `TARGET_ROUNDTRIP` 이하여야 함 |

16-bit 최대값 65,535는 327,675 ns, 즉 327.675 us다.

### CTL9 `LASER_ROUNDTRIP` - `0x024`, RW

| Bits | Field | 단위 | 의미 |
|---:|---|---|---|
| `[31:0]` | `TARGET_ROUNDTRIP` | 5 ns ticks | 발사 후 목표 왕복거리 측정 대기창. 0은 허용하지 않음 |

### CTL10 `LASER_TDC_WIDTH` - `0x028`, RW

| Bits | Field | 단위 | 의미 |
|---:|---|---|---|
| `[15:0]` | `START_TDC_WIDTH` | 5 ns ticks | `start_tdc` pulse 폭. 0이면 유효한 shot을 만들 수 없음 |
| `[31:16]` | `STOP_TDC_WIDTH` | 5 ns ticks | `stop_tdc` pulse 폭. 0이면 stop pulse 비활성 |

### CTL11 `LASER_SIM_DELAY` - `0x02C`, RW

| Bits | Field | 단위 | 의미 |
|---:|---|---|---|
| `[31:0]` | `SIM_T0_DELAY` | 5 ns ticks | simulation shot의 Fire-to-TDC-start 지연. 물리 shot에서는 무시 |

### CTL12 `LASER_SCHED0` - `0x030`, RW

| Bits | Field | 단위 | 의미 |
|---:|---|---|---|
| `[15:0]` | `SHOT_INTERVAL_STATES` | decoded states | 다음 발사를 허용하는 최소 각도 간격. 0은 허용하지 않음 |
| `[20:16]` | `FACE_ENABLE_MASK` | bit/Face | bit n이 1이면 Face n에서 발사 허용 |
| `[31:21]` | Reserved | - | 0으로 기록 |

### CTL13 `LASER_SCHED1` - `0x034`, RW

| Bits | Field | 단위 | 의미 |
|---:|---|---|---|
| `[15:0]` | `START_SKIP_STEPS` | shot steps | Face 진입 후 건너뛸 발사 grid 수 |
| `[31:16]` | `ACTIVE_WINDOW_STEPS` | shot steps | 활성 grid 수. 0이면 Face 끝까지 별도 제한 없음 |

### CTL14 `LASER_SCHED2` - `0x038`, RW

| Bits | Field | 단위 | 의미 |
|---:|---|---|---|
| `[15:0]` | `MAX_SHOTS_PER_FACE` | shots | Face당 최대 shot 수. 0이면 별도 개수 제한 없음 |
| `[31:16]` | `REARM_GUARD` | 5 ns ticks | roundtrip 종료 후 다시 발사 가능 상태가 되기 전 추가 보호시간 |

Laser commit은 다음 조건을 모두 만족해야 한다.

- `FIRE_WIDTH != 0`
- `FIRE_DONE_TIMEOUT != 0`
- `TARGET_ROUNDTRIP != 0`
- `FIRE_DONE_TIMEOUT <= TARGET_ROUNDTRIP`
- `SHOT_INTERVAL_STATES != 0`

## 3.4 Echo Receiver indexed delay profile

### CTL15 `ECHO_DELAY_CMD` - `0x03C`, RW

| Bits | Field | 종류 | 의미 |
|---:|---|---|---|
| `[4:0]` | `CHANNEL_INDEX` | 0~31 | 설정/조회할 APD Echo channel 선택 |
| `[7:5]` | Reserved | - | 0으로 기록 |
| `[8]` | `DELAY_WRITE_TOGGLE` | toggle | CTL16이 안정된 후 반전하여 선택 channel의 staging delay 기록 |
| `[9]` | `PROFILE_APPLY_TOGGLE` | toggle | 전체 staging profile을 Echo window가 idle일 때 active로 적용 |
| `[31:10]` | Reserved | - | 0으로 기록 |

### CTL16 `ECHO_DELAY_DATA` - `0x040`, RW

| Bits | Field | 단위 | 의미 |
|---:|---|---|---|
| `[15:0]` | `CHANNEL_DELAY` | 5 ns ticks | 선택 channel의 simulation delay |
| `[31:16]` | Reserved | - | 0으로 기록 |

Echo Receiver를 합성에서 비활성화하면 CTL15/16 주소는 ABI 호환을 위해
남지만 처리 기능은 없고, Echo capability가 0이 되며 STAT19~22도 0이다.

## 3.5 TDC-GPX bus, 설정 image, pipeline, command

### CTL17 `TDC_BUS_TIMING` - `0x044`, RW

| Bits | Field | 인코딩 | 의미 |
|---:|---|---|---|
| `[5:0]` | `BUS_CLK_DIV` | 1~63 | GPX bus tick divider. capture 안전 최소값보다 작으면 RTL이 상향 clamp |
| `[8:6]` | `BUS_TICKS` | 통상 3~7 | bus phase 길이. divider별 안전 최소값보다 작으면 상향 clamp |
| `[9]` | Reserved | - | 0으로 기록 |
| `[13:10]` | `REG_ADDR` | 0~15 | 직접 접근할 GPX register 주소 |
| `[15:14]` | `REG_CHIP_ID` | 0~3 | `REG_CHIP_MASK=0`일 때 단일 대상 chip |
| `[19:16]` | `REG_CHIP_MASK` | bit/chip | 0이 아니면 복수 chip 선택. 0이면 `REG_CHIP_ID`를 one-hot 변환 |
| `[31:20]` | Reserved | - | local CSR의 과거 read/write trigger는 통합 모드에서 사용하지 않음 |

### CTL18 `TDC_START_OFFSET` - `0x048`, RW

| Bits | Field | 의미 |
|---:|---|---|
| `[17:0]` | `START_OFF1` | GPX 설정 image Reg5에 적용되고 Face header에도 기록되는 Start offset |
| `[31:18]` | Reserved | 0으로 기록 |

### CTL19 `TDC_CFG_REG7` - `0x04C`, RW

| Bits | Field | 의미 |
|---:|---|---|
| `[31:0]` | `CFG_REG7` | GPX Reg7 staging/override word. 실제 28-bit bus에는 `[27:0]`만 전달 |

### CTL20 `TDC_IMAGE_CMD` - `0x050`, RW

| Bits | Field | 종류 | 의미 |
|---:|---|---|---|
| `[4:0]` | `IMAGE_INDEX` | 0~15 | GPX 설정 image register 선택 |
| `[7:5]` | Reserved | - | 0으로 기록 |
| `[15:8]` | `IMAGE_WRITE_EPOCH` | Epoch | CTL21이 안정된 후 변경하여 image word 1개 staging |
| `[31:16]` | Reserved | - | 0으로 기록 |

### CTL21 `TDC_IMAGE_DATA` - `0x054`, RW

| Bits | Field | 의미 |
|---:|---|---|
| `[31:0]` | `IMAGE_DATA` | 선택된 GPX image word. `[31:28]`은 CSR readback에는 보존되지만 28-bit 물리 bus에는 출력되지 않음 |

초기화 시 GPX Reg14 bit 4는 강제로 0으로 기록된다. 지원하지 않는 GPX
16-bit mode는 CSN 관련 별도 workaround가 필요하기 때문이다. 각 GPX
설정 image bit의 물리 의미는 GPX datasheet를 따른다. CTL20/21은 그 값을
가공하지 않는 raw indexed window다.

### CTL22 `TDC_SCAN_CFG` - `0x058`, RW

| Bits | Field | 단위 | 의미 |
|---:|---|---|---|
| `[15:0]` | `MAX_SCAN` | 5 ns ticks | Face/line scan watchdog. 0은 programmable deadline만 비활성화하며 hard safety cap은 남음 |
| `[18:16]` | `MAX_HITS` | 0~7 | Stop당 최대 Return 수. 0은 합성 최대값, build 최대 초과값은 clamp |
| `[19]` | `FALLING_ENABLE` | Boolean | 구성된 falling slope lane 활성 |
| `[31:20]` | Reserved | - | 0으로 기록 |

### CTL23 `TDC_PIPELINE_MAIN` - `0x05C`, RW

| Bits | Field | 의미 |
|---:|---|---|
| `[3:0]` | `ACTIVE_CHIP_MASK` | 요청한 논리 chip mask. 합성된 present mask와 AND. 0이면 첫 present chip 선택 |
| `[4]` | `PACKET_SCOPE` | Header packet-scope metadata |
| `[6:5]` | `HIT_STORE_MODE` | `00` raw, `01` corrected, `10` distance, `11` reserved. 현재 header/contract metadata |
| `[9:7]` | `DIST_SCALE` | Header에 전달하는 거리 scale metadata |
| `[10]` | `DRAIN_MODE` | GPX FIFO drain 정책 선택 |
| `[11]` | `PIPELINE_EN` | Pipeline enable 제어/metadata |
| `[14:12]` | Reserved | Face 수는 Motor sideband가 소유 |
| `[18:15]` | `STOPS_PER_CHIP` | 요청값 2~build 최대. 범위를 벗어나면 clamp |
| `[22:19]` | `DRAIN_CAP` | drain word 상한 정책. 0이면 이 cap으로 제한하지 않음 |
| `[27:23]` | `STOPDIS_OVERRIDE` | GPX Stop-disable override |
| `[31:28]` | Reserved | 통합 command는 CTL25가 소유 |

### CTL24 `TDC_RANGE_COLS` - `0x060`, RW

| Bits | Field | 단위 | 의미 |
|---:|---|---|---|
| `[15:0]` | `MAX_RANGE` | 5 ns ticks | 목표 왕복거리 capture 한계. TDC와 AXIS 도메인 클럭 수로 각각 변환 |
| `[31:16]` | `COLS_PER_FACE` | shots/Face | VDMA 한 Frame의 line 수. 0은 1로 보정 |

### CTL25 `TDC_AUX_CMD` - `0x064`, RW

| Bits | Field | 인코딩 | 의미 |
|---:|---|---|---|
| `[2:0]` | `OPCODE` | 0~7 | `0` none, `1` start, `2` stop, `3` force reinit, `4` error clear, `5` register read, `6` register write, `7` invalid |
| `[7:3]` | Reserved | - | 0으로 기록 |
| `[15:8]` | `CMD_EPOCH` | Epoch | 값을 바꾸면 serialized command를 정확히 한 번 요청 |
| `[31:16]` | Reserved | - | 0으로 기록 |

### CTL26~CTL31 예약 영역

| Register | Offset | 기록값 |
|---|---:|---:|
| CTL26 | `0x068` | `0x00000000` |
| CTL27 | `0x06C` | `0x00000000` |
| CTL28 | `0x070` | `0x00000000` |
| CTL29 | `0x074` | `0x00000000` |
| CTL30 | `0x078` | `0x00000000` |
| CTL31 | `0x07C` | `0x00000000` |

기존 주소를 이동하지 않고 기능을 추가하기 위한 예약 공간이므로 현재는
항상 0으로 유지한다.

## 4. 상태 레지스터 STAT0~STAT31

모든 STAT는 RO다. 다른 클럭 도메인에서 넘어오는 status는 coherent
snapshot과 동기화 단계를 거치므로 reset, commit, 빠른 event 직후에는
CSR clock 기준 관측 지연이 있을 수 있다. `Sticky`는 과거 오류 이력이고
`Live`는 현재 상태다.

## 4.1 시스템 식별과 용량

### STAT0 `SYS_VERSION` - `0x080`

| Bits | Field | 현재 값/의미 |
|---:|---|---|
| `[31:24]` | Signature | `0x4C`, ASCII `L` |
| `[23:16]` | ABI major | `1` |
| `[15:8]` | ABI minor | `0` |
| `[7:0]` | RTL revision | `0` |

### STAT1 `SYS_CAPABILITY` - `0x084`

| Bits | Field | 의미 |
|---:|---|---|
| `[0]` | `MOTOR_PRESENT` | Motor Decoder 포함 |
| `[1]` | `LASER_PRESENT` | Laser Controller 포함 |
| `[2]` | `ECHO_PRESENT` | Echo Receiver 포함. no-Echo build에서는 0 |
| `[3]` | `TDC_PRESENT` | TDC-GPX controller 포함 |
| `[4]` | `ECHO_INDEXED` | Echo indexed delay window 지원 |
| `[5]` | `TDC_INDEXED` | GPX indexed image window 지원 |
| `[6]` | `CFG_EPOCH` | 공유 config epoch 지원 |
| `[7]` | `RESET_EPOCH` | 공유 reset epoch 지원 |
| `[12:8]` | `ACTIVE_CTL_COUNT` | Echo 포함 26, 미포함 24 |
| `[18:13]` | `ACTIVE_STAT_COUNT` | Echo 포함 32, 미포함 28 |
| `[24:19]` | `IRQ_SOURCE_COUNT` | 32 |
| `[31:25]` | Reserved | 0 |

### STAT2 `SYS_CONFIG` - `0x088`

| Bits | Field | 의미 |
|---:|---|---|
| `[7:0]` | `CFG_REQUESTED` | CTL1이 요청한 config epoch |
| `[15:8]` | `RESET_REQUESTED` | CTL0이 요청한 reset epoch |
| `[23:16]` | `LASER_CFG_ACCEPTED` | Laser가 마지막으로 승인한 config epoch |
| `[31:24]` | `TDC_CFG_ACCEPTED` | TDC가 마지막으로 승인한 config epoch |

### STAT3~STAT5 합성 최대 용량

| Index / 주소 | Register | Bits | 의미 |
|---:|---|---:|---|
| STAT3 / `0x08C` | `TDC_MAX_ROWS` | `[31:0]` | 논리 Cell slot 최대 32개, `4 chips * 8 Stops` |
| STAT4 / `0x090` | `TDC_CELL_SIZE` | `[31:0]` | Return 7개일 때 canonical Cell 최대 20 bytes |
| STAT5 / `0x094` | `TDC_MAX_HSIZE` | `[31:0]` | 4-chip full-mask 최대 line 688 bytes |

STAT3~5는 **현재 Face geometry가 아니라 ABI 최대 용량 상수**다. 실제 VDMA
설정에는 `o_vdma_hsize_bytes_rise`, `o_vdma_hsize_bytes_fall`,
`o_vdma_vsize_lines`를 사용해야 한다.

## 4.2 Motor 상태

### STAT6 `MOTOR_STATUS` - `0x098`

| Bits | Field | 종류 | 의미 |
|---:|---|---|---|
| `[2:0]` | `CURRENT_FACE` | Live | 현재 Face index |
| `[3]` | `ACTIVE` | Live | 현재 위치가 활성 Face window 내부 |
| `[4]` | `SIM_RUNNING` | Live | 가상 엔코더 동작 중 |
| `[5]` | `CFG_BUSY` | Live | Motor 설정 transaction 처리 중 |
| `[6]` | `Z_EARLY_OVERRUN` | Sticky | early-index overrun |
| `[7]` | `Z_COLLISION` | Sticky | index trigger 충돌 |
| `[8]` | `POSITION_OVERFLOW` | Sticky | 위치 연산 overflow |
| `[9]` | `QUAD_INVALID` | Sticky | 잘못된 quadrature 전이 검출 |
| `[10]` | `AXIS_DROP` | Sticky | Motor AXIS source event 손실 |
| `[12:11]` | `ACTIVE_DEC_MODE` | Live | 현재 적용된 x1/x2/x4 mode |
| `[15:13]` | `N_FACES` | Static | 합성 시 결정된 다면미러 Face 수 |
| `[16]` | `APPLIED_DIR` | Live | 적용된 극성/방향 설정 |
| `[17]` | `DECODED_DIR` | Live | quadrature decoder가 실제 보고한 이동 방향 |
| `[31:18]` | Reserved | - | 0 |

### STAT7 `MOTOR_FACE_GEOMETRY` - `0x09C`

| Bits | Field | 의미 |
|---:|---|---|
| `[14:0]` | `APPLIED_FACE_CENTER` | 선택 Face의 적용 center, decoded states |
| `[29:15]` | `APPLIED_FACE_HALF_WIDTH` | 선택 Face의 적용 active half-width |
| `[30]` | `GEOMETRY_VALID` | 해당 geometry가 정상 적용됨 |
| `[31]` | Reserved | 0 |

### STAT8 `MOTOR_CFG_STATUS` - `0x0A0`

| Bits | Field | 의미 |
|---:|---|---|
| `[7:0]` | `CFG_EPOCH_ACCEPTED` | 마지막 승인 공유 config epoch |
| `[15:8]` | `FACE_EPOCH_ACCEPTED` | 마지막 승인 Face-write epoch |
| `[18:16]` | `FACE_READ_INDEX` | STAT7이 표시하는 Face |
| `[19]` | `GEOMETRY_VALID` | 적용 geometry 유효 |
| `[20]` | `BUSY` | 설정 처리 중 |
| `[21]` | `APPLY_TRACK` | apply transaction 추적 중 |
| `[22]` | `REJECT` | 잘못된 설정 거부 이력 |
| `[23]` | `CFG_VALID` | 현재 active Motor 설정 유효 |
| `[31:24]` | `RESET_EPOCH_ACCEPTED` | 마지막 승인 reset epoch |

| Index / 주소 | Register | 의미 |
|---:|---|---|
| STAT9 / `0x0A4` | `MOTOR_QUAD_INVALID` | 32-bit invalid quadrature transition wrap counter |
| STAT10 / `0x0A8` | `MOTOR_AXIS_DROP` | 32-bit Motor AXIS drop wrap counter |
| STAT11 / `0x0AC` | `MOTOR_REV_PERIOD` | 마지막 Z-to-Z 회전주기, Motor clocks. 유효 측정 전에는 0 |

## 4.3 Laser 상태와 지연 측정

### STAT12 `LASER_STATUS` - `0x0B0`

| Bits | Field | 종류 | 의미 |
|---:|---|---|---|
| `[0]` | `LASER_ACTIVE` | Live | Laser enable과 Face session이 모두 활성. 물리 pulse high와 동일 의미는 아님 |
| `[1]` | `FIRE_BUSY` | Live | Fire executor 동작 중 |
| `[2]` | `START_TDC_BUSY` | Live | `start_tdc` pulse 출력 중 |
| `[3]` | `STOP_TDC_BUSY` | Live | `stop_tdc` pulse 출력 중 |
| `[4]` | `FIRE_WIDTH_ZERO` | Sticky/차단 | fire width가 0이었음 |
| `[5]` | `ROUNDTRIP_ZERO` | Sticky/차단 | target roundtrip이 0이었음 |
| `[6]` | `SIM_BLIND` | Sticky/진단 | simulation offset <= fire width |
| `[7]` | `SIM_EXCEED` | Sticky/진단 | simulation offset >= roundtrip |
| `[8]` | `TIMEOUT_COUNT_OVERFLOW` | Sticky/IRQ | 누적 timeout counter wrap |
| `[9]` | `FRAME_COUNT_OVERFLOW` | Sticky/IRQ | 회전별 command/done counter overflow |
| `[10]` | `FRAME_RESET_PENDING` | Live | 다음 Face-0 경계에서 count reset 대기 |
| `[11]` | `SCHEDULE_OVERRUN` | Sticky | 재무장 전에 다음 각도 grid 도달 |
| `[12]` | `FIRE_DONE_TIMEOUT_ZERO` | Sticky/차단 | fire_done timeout이 0이었음 |
| `[31:13]` | Reserved | - | 0 |

| Index / 주소 | Register | 단위 | 유효조건 | 의미 |
|---:|---|---|---|---|
| STAT13 / `0x0B4` | `LASER_ENCODER_TO_FIRE` | AXIS clocks | STAT16[6] | Motor 위치 event에서 실제 `fire_pulse` 상승까지 지연 |
| STAT14 / `0x0B8` | `LASER_FIRE_DONE` | AXIS clocks | STAT16[4] | `fire_pulse` 상승에서 동기화된 `fire_done`까지 지연 |
| STAT15 / `0x0BC` | `LASER_FIRE_TO_TDC` | AXIS clocks | STAT16[5] | `fire_pulse` 상승에서 `start_tdc` 상승까지 지연 |

### STAT16 `LASER_METRIC_FLAGS` - `0x0C0`

| Bits | Field | 의미 |
|---:|---|---|
| `[0]` | `SNAPSHOT_VALID` | 지연 측정 snapshot 유효 |
| `[1]` | `PHYSICAL_SHOT` | 물리 fire 경로에서 얻은 snapshot |
| `[2]` | `TIMEOUT_SHOT` | fire_done watchdog timeout shot |
| `[3]` | `SIMULATED_SHOT` | simulation 경로 shot |
| `[4]` | `FIRE_DONE_VALID` | STAT14 유효 |
| `[5]` | `START_TDC_VALID` | STAT15 유효 |
| `[6]` | `ENCODER_TO_FIRE_VALID` | STAT13 유효 |
| `[15:7]` | Reserved | 0 |
| `[31:16]` | `METRIC_SEQUENCE` | coherent snapshot wrap sequence |

일관된 측정값을 읽으려면 STAT16 sequence를 먼저 읽고 STAT13~15를 읽은
뒤 STAT16을 다시 읽는다. 두 sequence가 같고 `SNAPSHOT_VALID=1`일 때만
세 측정값을 같은 shot의 결과로 사용한다.

| Index / 주소 | Register | Bits | 의미 |
|---:|---|---:|---|
| STAT17 / `0x0C4` | `LASER_FRAME_COUNT` | `[15:0]` | 현재 한 회전의 실제 FIRE_CMD 수 |
|  |  | `[31:16]` | 현재 한 회전의 정상 FIRE_DONE 수 |
| STAT18 / `0x0C8` | `LASER_TIMEOUT_COUNT` | `[31:0]` | fire_done 미수신/watchdog timeout 누적 wrap counter |

회전별 command/done count는 다음 Face-0 경계에서 다시 시작한다.
Timeout count는 운용 중 계속 누적되고 wrap되며, wrap 발생은 STAT12[8]에
남아 장시간 레이저 모듈 이상을 운영자가 확인할 수 있다.

## 4.4 Echo Receiver 상태

### STAT19 `ECHO_RISE_MASK` - `0x0CC`

`[31:0]`은 마지막으로 완료된 shot의 rising 검출 mask다.

```text
channel_index = chip_id * stops_per_chip + stop_id
```

해당 channel bit가 1이면 observation window 안에서 rising edge가 있었다.

### STAT20 `ECHO_FALL_MASK` - `0x0D0`

`[31:0]`은 같은 shot의 falling 검출 mask다.

### STAT21 `ECHO_STATUS` - `0x0D4`

| Bits | Field | 종류 | 의미 |
|---:|---|---|---|
| `[0]` | `DESCRIPTOR_PROTOCOL` | Sticky | descriptor protocol 오류 |
| `[1]` | `DESCRIPTOR_OVERWRITE` | Sticky | pending descriptor overwrite |
| `[2]` | `DESCRIPTOR_MISSING` | Sticky | 필요한 descriptor 미수신 |
| `[3]` | `WINDOW_SEQUENCE` | Sticky | shot/window 순서 오류 |
| `[4]` | `COUNT_SATURATED` | Sticky | 내부 event count 포화 |
| `[5]` | `DESCRIPTOR_PENDING` | Live | 처리 대기 descriptor 존재 |
| `[6]` | `WINDOW_ACTIVE` | Live | Echo observation window 활성 |
| `[7]` | `SIM_MODE_ACTIVE` | Live | Echo simulation mode 활성 |
| `[8]` | `ANY_SIM_ACTIVE` | Live | 하나 이상의 channel simulation 활성 |
| `[9]` | `DELAY_WRITE_ACK` | toggle | CTL15[8] delay write 승인 |
| `[10]` | `PROFILE_APPLY_ACK` | toggle | CTL15[9] profile apply 승인 |
| `[11]` | `PROFILE_APPLY_PENDING` | Live | window가 끝날 때까지 apply 대기 |
| `[12]` | `COMMAND_REJECT` | Sticky | 잘못된 index 또는 겹친 profile command 거부 |
| `[13]` | `LAST_SHOT_VALID` | History | STAT19/20에 완료된 shot 정보가 있음 |
| `[15:14]` | Reserved | - | 0 |
| `[23:16]` | `SHOT_SEQUENCE` | wrap | 완료 shot sequence |
| `[31:24]` | `PROFILE_SEQUENCE` | wrap | 적용 profile sequence |

### STAT22 `ECHO_DELAY_READBACK` - `0x0D8`

| Bits | Field | 의미 |
|---:|---|---|
| `[15:0]` | `SELECTED_DELAY` | 선택 channel의 active delay, 5 ns ticks |
| `[20:16]` | `SELECTED_INDEX` | CTL15가 선택한 channel |
| `[21]` | `SELECTED_VALID` | 선택 index가 합성된 channel 수 안에 있음 |
| `[22]` | `SELECTED_SIM_ACTIVE` | 선택 channel simulation 활성 |
| `[23]` | `SELECTED_RISE` | 마지막 shot에서 선택 channel rising 검출 |
| `[24]` | `SELECTED_FALL` | 마지막 shot에서 선택 channel falling 검출 |
| `[31:25]` | Reserved | 0 |

## 4.5 TDC-GPX 직접 읽기와 pipeline 상태

### STAT23~STAT26 `TDC_CHIPn_RESULT` - `0x0DC..0x0E8`

| Register | 주소 | 논리 chip |
|---|---:|---:|
| STAT23 `TDC_CHIP0_RESULT` | `0x0DC` | 0 |
| STAT24 `TDC_CHIP1_RESULT` | `0x0E0` | 1 |
| STAT25 `TDC_CHIP2_RESULT` | `0x0E4` | 2 |
| STAT26 `TDC_CHIP3_RESULT` | `0x0E8` | 3 |

각 word의 비트 구조는 같다.

| Bits | Field | 의미 |
|---:|---|---|
| `[27:0]` | `REG_RDATA` | 해당 chip의 마지막 GPX 직접 register read 데이터 |
| `[31:28]` | `REG_ADDR_DONE` | 데이터와 연결된 GPX register 주소 |

### STAT27 `TDC_PIPELINE_STATUS` - `0x0EC`

| Bits | Field | 종류 | 의미 |
|---:|---|---|---|
| `[0]` | `BUSY` | Live | TDC control/pipeline 동작 중 |
| `[1]` | `PIPELINE_OVERRUN` | Event/status | pipeline overrun 관측 |
| `[2]` | `FATAL_RECOVERY` | Sticky | fatal recovery 진입 이력 |
| `[3]` | Reserved | - | 0 |
| `[7:4]` | `CHIP_ERROR_MASK` | Sticky/집계 | chip 내부 오류 또는 GPX Reg12 fault |
| `[11:8]` | `DRAIN_TIMEOUT_MASK` | Sticky | chip별 IFIFO drain timeout |
| `[15:12]` | `SEQUENCE_ERROR_MASK` | Sticky | chip별 acquisition sequence/protocol 오류 |
| `[23:16]` | `CMD_EPOCH_ACCEPTED` | Ack | 마지막 승인 CTL25 command epoch |
| `[31:24]` | `IMAGE_EPOCH_ACCEPTED` | Ack | 마지막 승인 CTL20 image-write epoch |

### STAT28 `TDC_STATUS_EXT` - `0x0F0`

| Bits | Field | 의미 |
|---:|---|---|
| `[0]` | `ERR_READ_TIMEOUT` | 직접 register read timeout |
| `[1]` | `REG_REQUEST_REJECTED` | register 요청 거부 |
| `[2]` | `REG_ZERO_MASK` | register 요청 대상 chip mask가 0 |
| `[3]` | `RISE_SHOT_FLUSH_DROP` | rising lane shot flush에서 데이터 손실 |
| `[4]` | `FALL_SHOT_FLUSH_DROP` | falling lane shot flush에서 데이터 손실 |
| `[5]` | `RISE_HEADER_DRAIN_TIMEOUT` | rising output drain watchdog timeout |
| `[6]` | `FALL_HEADER_DRAIN_TIMEOUT` | falling output drain watchdog timeout |
| `[7]` | `FRAME_WAIT_ESCAPE` | frame wait/recovery escape 발생 |
| `[11:8]` | `RISE_OVERRUN_COUNT_LO` | rising overrun wrap counter 하위 nibble |
| `[15:12]` | `SHOT_FLUSH_DROP_MASK` | chip별 flush/drop 이력 |
| `[19:16]` | `FALL_OVERRUN_COUNT_LO` | falling overrun wrap counter 하위 nibble |
| `[23:20]` | `CMD_COLLISION_MASK` | chip별 command collision |
| `[27:24]` | `REG_REQUEST_OVERFLOW_MASK` | chip별 register request queue overflow |
| `[31:28]` | `RUN_DRAIN_COMPLETE_MASK` | chip별 run drain 완료 |

### STAT29 `TDC_STATUS_EXT2` - `0x0F4`

| Bits | Field | 의미 |
|---:|---|---|
| `[3:0]` | `REG_TIMEOUT_MASK` | chip별 직접 register timeout |
| `[7:4]` | `STOP_ID_ERROR_MASK` | 활성 범위를 벗어난 decoded Stop ID |
| `[10:8]` | `LAST_RUN_TIMEOUT_CAUSE` | `000` none, `001` raw busy, `010` EF1 response, `011` EF2 response, `100` burst response, `101` flush response, `110` overrun flush, `111` capture-stop fallback |
| `[14:11]` | `QUARANTINE_ESCAPE_MASK` | cell-builder hard quarantine escape 이력 |
| `[15]` | `MASKED_SLOPE_DROP_ANY` | 해당 chip에서 비활성 slope로 Hit 입력 |
| `[19:16]` | `RISE_FACE_START_COLLAPSED_LO` | rising collapsed Face-start count 하위 nibble |
| `[23:20]` | `GPX_REG12_FAULT_MASK` | 직접 읽은 GPX Reg12 `[10:0]`이 0이 아니었음 |
| `[27:24]` | `FALL_FACE_START_COLLAPSED_LO` | falling collapsed Face-start count 하위 nibble |
| `[31:28]` | `INIT_CFG_COALESCED_MASK` | 초기화 중 config 요청 coalesced 이력 |

CTL25 opcode 4 `ERROR_CLEAR`는 GPX Reg12 fault와 운용 timeout/sequence 등
soft-clear 진단 epoch를 지운다. `QUARANTINE_ESCAPE_MASK`처럼 강한 escalation
증거는 reset-only로 유지된다. 따라서 ERROR_CLEAR 후 남아 있는 reset-only
bit를 무조건 새로운 오류로 해석하면 안 된다. 완전히 새로운 이력 기준이
필요하면 CTL0 `RESET_EPOCH`를 변경한다.

### STAT30 `TDC_IMAGE_SELECTED_DATA` - `0x0F8`

`[31:0]`은 CTL20 `IMAGE_INDEX`가 선택한 GPX 설정 image word다.

### STAT31 `SYS_ADAPTER_STATE` - `0x0FC`

| Bits | Field | 의미 |
|---:|---|---|
| `[0]` | `MOTOR_CFG_MATCH` | Motor 승인 epoch가 CTL1과 같고 config 유효 |
| `[1]` | `LASER_CFG_MATCH` | Laser 승인 epoch가 CTL1과 같고 config 유효 |
| `[2]` | `TDC_CFG_MATCH` | TDC 승인 epoch가 CTL1과 같고 config 유효 |
| `[3]` | `MOTOR_RESET_MATCH` | Motor 승인 reset epoch가 CTL0과 같음 |
| `[4]` | `LASER_RESET_MATCH` | Laser 승인 reset epoch가 CTL0과 같음 |
| `[5]` | `ECHO_RESET_MATCH` | Echo 승인 reset epoch가 CTL0과 같음 |
| `[6]` | `TDC_RESET_MATCH` | TDC 승인 reset epoch가 CTL0과 같음 |
| `[7]` | `MOTOR_BUSY` | Motor adapter busy |
| `[8]` | `LASER_BUSY` | Laser adapter busy |
| `[9]` | `ECHO_BUSY` | Echo profile apply pending |
| `[10]` | `TDC_BUSY` | TDC config 또는 command busy |
| `[11]` | `MOTOR_REJECT` | Motor config reject |
| `[12]` | `LASER_REJECT` | Laser config reject |
| `[13]` | `ECHO_REJECT` | Echo profile command reject |
| `[14]` | `TDC_REJECT` | TDC config/image/command reject |
| `[15]` | `MOTOR_VALID` | active Motor config valid |
| `[16]` | `LASER_VALID` | active Laser config valid |
| `[17]` | `TDC_VALID` | active TDC config valid |
| `[18]` | `ALL_CFG_ACCEPTED` | Motor/Laser/TDC config match 모두 1 |
| `[19]` | `ALL_RESET_ACCEPTED` | 네 adapter reset match 모두 1 |
| `[20]` | `ANY_BUSY` | 하나 이상의 adapter busy |
| `[21]` | `ANY_REJECT` | 하나 이상의 adapter reject |
| `[31:22]` | Reserved | 0 |

## 5. Interrupt 레지스터와 source bit

## 5.1 Interrupt register 동작

| Offset | Register | 의미 |
|---:|---|---|
| `0x100` | `INTR_EN` | bit n=1인 source만 `o_irq`에 기여 |
| `0x104` | `INTR_STATUS` | 동기화된 현재 source level. 관측 전용 |
| `0x108` | `INTR_FLAG` | Manual mode pending. 1을 써서 clear. 같은 시점의 새 event가 clear보다 우선 |
| `0x10C` | `INTR_MODE` | bit n=0: manual latched level, bit n=1: CSR clock 1-cycle auto pulse |

권장 초기화는 `INTR_EN=0`, mode 설정, 기존 `INTR_FLAG` W1C, source idle
확인, 필요한 enable 적용 순서다. Manual ISR에서는 `INTR_FLAG`를 읽고 상세
STAT 원인을 처리한 뒤 읽은 bit를 W1C하고 다시 읽어야 한다.

## 5.2 Interrupt source map

| Bit | 소유 IP | Event |
|---:|---|---|
| 0..3 | System | Reserved, 항상 0 |
| 4 | Motor | 활성 Face 영역 진입 |
| 5 | Motor | 활성 Face 영역 이탈 |
| 6 | Motor | 한 회전/Z-index event |
| 7 | Motor | Motor diagnostic transition |
| 8 | Laser | 발사 차단 config/error transition |
| 9 | Laser | Timeout count overflow transition |
| 10 | Laser | Frame count overflow transition |
| 11..15 | Laser | Reserved, 항상 0 |
| 16 | Echo | Echo diagnostic transition |
| 17 | Echo | Echo indexed command reject transition |
| 18..20 | Echo | Reserved, 항상 0 |
| 21 | TDC | GPX 직접 register command 완료 |
| 22 | TDC | Pipeline overrun 또는 fatal recovery transition |
| 23 | TDC | Chip error 또는 GPX Reg12 fault transition |
| 24 | TDC | Drain/register/header timeout transition |
| 25 | TDC | Sequence/Stop-ID/command collision/masked slope/request 오류 transition |
| 26 | TDC | TDC config/image reject transition |
| 27 | TDC | Serialized command reject transition |
| 28..31 | Reserved | 항상 0 |

## 6. 외부 TDC-GPX 데이터의 전체 이동 경로

## 6.1 IP별 데이터 소유권

```mermaid
flowchart LR
    E["Echo Receiver\n물리 STOP 파형"] --> G["외부 TDC-GPX\nSTART-to-STOP 측정"]
    G -->|"28-bit I-Mode word\nio_tdc_d[27:0]"| B["bus_phy / chip_ctrl\ni_tdc_clk"]
    B -->|"32b TDATA + 8b TUSER"| C["stream CDC\nSYNC 또는 ASYNC"]
    C --> D["I-Mode decoder\ni_axis_aclk"]
    D --> R["raw event builder\nchip/Stop/Return tag"]
    R --> P["rise/fall cell builder\n17-bit Hit 저장"]
    P --> A["face assembler\nchip 다음 Stop 순서"]
    A --> L["line packer\ncanonical 32-bit word"]
    L --> H["48-byte line prefix\nSOF/EOL 생성"]
    H --> V["AXI VDMA S2MM\nDDR Frame buffer"]
```

Echo Receiver는 GPX 데이터 word를 계산하지 않는다. LVDS Echo를 single-ended
STOP 파형으로 만들어 외부 GPX chip에 전달한다. 외부 GPX가 START-to-STOP을
측정하여 28-bit I-Mode word를 만든다. `tdc_gpx_top`은 그 값을 bus에서 읽어
tag를 추가하고 순서를 정렬하고 오류를 표시하고 DDR 형식으로 packing한다.

## 6.2 외부 GPX I-Mode 28-bit word

GPX Reg8은 IFIFO1, Reg9는 IFIFO2 데이터다. `io_tdc_d[27:0]`의 의미는
다음과 같다.

| Bits | GPX Field | 현재 SINGLE_SHOT 의미 |
|---:|---|---|
| `[27:26]` | `ChaCode` | 선택 IFIFO 내부 channel 0~3 |
| `[25:18]` | `StartNum` | Start index. 현재 SINGLE_SHOT에서는 0/미사용 기대 |
| `[17]` | `Slope` | RTL 기준 `1` rising, `0` falling |
| `[16:0]` | `Hit` | GPX bin 단위 17-bit START-to-STOP 시간값 |

Stop ID는 IFIFO와 ChaCode를 연결하여 복원한다.

```text
stop_id[2:0] = ififo_id & ChaCode[1:0]
IFIFO1: Stop 0~3
IFIFO2: Stop 4~7
```

## 6.3 단계별 TDATA/TUSER 계약

| 단계 | TDATA | TUSER/제어 | Clock |
|---|---|---|---|
| `bus_phy -> chip_ctrl` | `[27:0]` read data, `[31:28]=0` | `[0]` read/write, `[4:1]` register 주소, `[7:5]=0` | TDC |
| `chip_ctrl raw stream` | `[27:0]` I-Mode word, `[31:28]=0` | `[0]` IFIFO ID, `[5]` final control beat fault, `[7]` drain_done | TDC 후 CDC |
| `decoder_i_mode` | `[16:0]` Hit, `[31:17]=0` | `[0]` slope, `[2:1]` ChaCode, `[5:3]` Stop ID, `[6]` IFIFO, `[7]` drain_done | AXIS |
| `raw_event_builder` | `[16:0]` Hit | `[0]` slope, `[2:1]` chip, `[5:3]` Stop, `[6]` IFIFO, `[7]` drain_done, `[10:8]` Return index, `[15:11]` shot sequence 하위 5-bit | AXIS |
| `cell_builder` | runtime Hit beat와 metadata word | slope에 따라 rise/fall builder 분리 | AXIS |
| `face_assembler` | 정해진 순서의 완전한 Cell | 내부 chip slice 마지막에 TLAST | AXIS |
| `line_packer/header` | canonical line + 48-byte prefix | `TUSER[0]` SOF, `TLAST` EOL | AXIS |

`drain_done`은 payload가 0인 제어 beat다. TUSER[7]=1로 표시하기 때문에
data beat에서 Stop ID로 쓰이는 중첩 bit와 구별된다. IFIFO1 완료는 shot을
끝내지 않고 IFIFO2 처리를 계속한다. 마지막 IFIFO2 완료가 shot acquisition
종료를 뜻한다.

## 6.4 Cell 정의와 17-bit Hit 보존

**Cell**은 한 `(slope lane, chip, Stop, shot)` 조합에 속한 모든 Return을
모은 고정 형식 단위다. Cell은 VDMA line이 아니다. VDMA line 하나는 한
shot의 모든 Cell을 담는다.

유효 최대 Return 수를 `H=1..7`이라고 하면:

```text
hit_words  = ceil(H / 2)
cell_words = hit_words + metadata 1 word
cell_bytes = 4 * cell_words
```

| H | Hit words | Metadata words | Cell bytes |
|---:|---:|---:|---:|
| 1~2 | 1 | 1 | 8 |
| 3~4 | 2 | 1 | 12 |
| 5~6 | 3 | 1 | 16 |
| 7 | 4 | 1 | 20 |

각 32-bit Hit word에는 두 Return의 하위 16-bit가 들어간다.

| Bits | 의미 |
|---:|---|
| `[15:0]` | Return `2*w`의 `Hit[15:0]` |
| `[31:16]` | Return `2*w+1`의 `Hit[15:0]`. 해당 Return이 없으면 0 |

마지막 32-bit metadata word는 다음과 같다.

| Bits | Field | 의미 |
|---:|---|---|
| `[31:25]` | `HIT_VALID[6:0]` | 각 Return slot 유효 mask |
| `[24:18]` | `SLOPE_VEC[6:0]` | rising Cell이면 유효 slot이 1, falling Cell이면 0 |
| `[17:16]` | Reserved | 0 |
| `[15:12]` | `HIT_COUNT_ACTUAL` | 실제 저장 Return 수 0~7 |
| `[11]` | `HIT_DROPPED` | 설정/저장 용량보다 많은 Hit가 들어옴 |
| `[10]` | `ERROR_FILL` | chip/Cell timeout 때문에 blank Cell로 채움 |
| `[9:8]` | `CHIP_ID` | 논리 GPX chip 0~3 |
| `[7]` | Reserved | 0 |
| `[6:0]` | `HIT_MSB[6:0]` | 각 Return의 `Hit[16]` |

DDR에서 Return n의 17-bit Hit는 다음과 같이 복원한다.

```text
Hit17[n] = metadata.HIT_MSB[n] & hit_slot[n][15:0]
metadata.HIT_VALID[n] = 1일 때만 유효
```

현재 RTL은 GPX의 17번째 Hit bit를 잃지 않는다. Header W3 bit 30이 이
metadata MSB 형식을 사용한다는 것을 표시한다.

## 6.5 Cell과 channel 순서

한 slope lane과 한 shot에서 Cell은 논리 chip ID 오름차순, 각 chip 안에서
Stop 0부터 `STOPS_PER_CHIP-1` 순서로 출력된다. 비활성 chip은 건너뛴다.
Rising과 falling은 서로 독립된 AXI stream과 VDMA Frame이며 한 line에
섞이지 않는다.

Chip 또는 Cell timeout이 발생하면 assembler가 같은 크기의 blank Cell을
만들고 `ERROR_FILL=1`을 기록한다. 누락 데이터를 제거하여 뒤 데이터를
당기는 대신 byte 위치를 보존하므로 이후 모든 Cell과 DDR Frame 정렬이
유지된다.

64/128-bit 출력에서도 canonical 32-bit Cell word 구조는 바뀌지 않는다.
`line_packer`가 이 word들을 인접 Cell 경계까지 연속 packing한다. 따라서
Cell마다 64/128-bit padding이 생기지 않는다. 전체 payload 끝에서 16-byte
line 정렬을 위해서만 0, 4, 8, 12 byte padding이 추가될 수 있다.

## 7. VDMA line의 48-byte prefix

모든 VDMA line 앞에는 48 bytes가 예약된다. Line 0의 12개 32-bit word에는
Face metadata가 들어가고, 이후 line의 prefix 48 bytes는 0이다. DDR byte
순서는 Zynq AXI의 little-endian 규칙을 따른다.

| Word / byte | Bits | 의미 |
|---|---:|---|
| W0 / `0x00` | `[31:0]` | Magic `0x47434454`. 메모리 byte로 `TDCG` |
| W1 / `0x04` | `[31:0]` | VDMA Frame ID |
| W2 / `0x08` | `[31:0]` | 예약 scan ID, 현재 0 |
| W3 / `0x0C` | `[7:0]` | Face ID |
|  | `[11:8]` | 이 slope lane의 active chip mask |
|  | `[14:12]` | 다면미러 Face 수 |
|  | `[18:15]` | Stops/chip |
|  | `[22:19]` | Drain cap |
|  | `[23]` | Pipeline enable |
|  | `[25:24]` | Hit store mode |
|  | `[28:26]` | Distance scale metadata |
|  | `[29]` | Drain mode |
|  | `[30]` | Cell metadata가 Hit[16]을 보존함 |
|  | `[31]` | 현재 Face에서 falling lane enable |
| W4 / `0x10` | `[15:0]` | 이 slope line의 Cell slot 수. VSIZE가 아님 |
|  | `[31:16]` | Face당 columns/lines |
| W5 / `0x14` | `[7:0]` | 유효 최대 Return 수 |
|  | `[15:8]` | Canonical Cell bytes |
|  | `[23:16]` | Hit slot 폭, 현재 16 |
|  | `[27:24]` | 합성된 present chip 수 |
|  | `[31:28]` | 합성된 최대 Stops/chip |
| W6 / `0x18` | `[15:0]` | Face 시작 시 global shot sequence |
|  | `[31:16]` | GPX bin resolution, ps |
| W7 / `0x1C` | `[17:0]` | StartOff1 |
|  | `[21:18]` | Cell format ID |
|  | `[25:22]` | Chip error mask |
|  | `[31:26]` | Reserved |
| W8 / `0x20` | `[31:0]` | `k_dist_fixed`. Q-format은 외부 calibration 계약 |
| W9 / `0x24` | `[31:0]` | Timestamp/cycle counter 하위 32-bit |
| W10 / `0x28` | `[31:0]` | Timestamp/cycle counter 상위 32-bit |
| W11 / `0x2C` | `[31:0]` | 집계 오류 조건이 active였던 AXIS clock 수 |

RTL 신호명이 `timestamp_ns`이지만 실제 구현은 `i_axis_aclk`마다 1씩
증가하는 **AXIS clock cycle counter**다. ns 값이 아니다.

```text
시간(초) = timestamp_counter / AXIS_clock_Hz
```

## 8. VDMA Hsync/Vsync와 HSIZE/VSIZE

## 8.1 별도 HSYNC/VSYNC pin은 없음

`tdc_gpx_top`은 AXI4-Stream Video 방식의 경계를 사용한다.

| 영상 개념 | AXI4-Stream 신호 | 이 설계의 의미 |
|---|---|---|
| Frame/수직 시작 | `TUSER[0]=1` | 한 Face·한 slope lane의 line 0 첫 전송 beat |
| 수평 line 종료 | `TLAST=1` | 모든 shot/column line의 마지막 전송 beat |
| 데이터 전송 | `TVALID && TREADY` | 32/64/128-bit payload 1 beat 전달 |
| 유효 byte | `TKEEP/TSTRB` | 모든 beat가 full-width이므로 전송 중 모두 1 |

따라서 일반 영상의 Vsync 역할은 SOF인 `TUSER[0]`, Hsync 경계 역할은
EOL인 `TLAST`가 담당한다. 별도 pulse 폭이나 극성을 설정하는 물리
HSYNC/VSYNC port는 없다.

## 8.2 이 설계에서 수평과 수직의 물리 의미

| 차원 | RTL/VDMA 의미 | LiDAR 스캔 의미 |
|---|---|---|
| Horizontal, HSIZE | 한 shot의 모든 활성 chip/Stop Cell, prefix와 line pad를 포함한 byte 수 | 한 레이저 지점에서 모든 APD channel을 수집한 결과 |
| Vertical, VSIZE | 한 Face 안의 shot line 수 | 다면미러 한 면이 활성인 동안 시간순으로 얻은 수평 scan point 수 |

VDMA의 H/V 명칭은 2차원 메모리 저장 구조에서 나온 것이다. APD channel이
실제 화면의 수평 pixel이라는 뜻은 아니다. Cell slot은 line 안의 channel
sample이고, VDMA line은 shot 하나이며, VDMA Frame은 다면미러 Face 하나다.

## 8.3 Rise/Fall별 authoritative geometry 산식

각 slope lane에 대해 독립적으로 계산한다.

```text
lane_chips    = popcount(active_lane_chip_mask)
cell_slots    = lane_chips * stops_per_chip
cell_bytes    = 4 * (ceil(effective_max_hits / 2) + 1)
payload_bytes = cell_slots * cell_bytes
HSIZE         = 48 + align_up(payload_bytes, 16)
VSIZE         = cols_per_face
frame_bytes   = HSIZE * VSIZE
beats_per_line = HSIZE / (g_OUTPUT_WIDTH / 8)
```

활성 chip이 없는 slope lane의 HSIZE는 0이다. Header만 있는 48-byte lane을
만들지 않는다. Rise와 Fall은 같은 VSIZE를 사용하지만 active chip 수가
다르면 HSIZE는 서로 다를 수 있다.

`g_OUTPUT_WIDTH=32/64/128`은 같은 line을 몇 beat로 보내는지만 바꾼다.
HSIZE byte, Cell 내용, DDR Frame byte 크기는 바뀌지 않는다.

## 8.4 Return 7개, Stops/chip 8개 예제

| Slope당 topology | Cell slots | Payload | HSIZE | 32-bit beats | 64-bit beats | 128-bit beats |
|---|---:|---:|---:|---:|---:|---:|
| 2 chips | 16 | 320 B | 368 B | 92 | 46 | 23 |
| 4 chips | 32 | 640 B | 688 B | 172 | 86 | 43 |

Dedicated 2-rise + 2-fall 구성은 일반적으로 두 lane 모두 HSIZE 368 B다.
4개 chip 모두 rising이고 falling이 비활성이면 rise HSIZE는 688 B이고
fall HSIZE는 0 B다.

## 8.5 VDMA S2MM 설정 순서

1. TDC topology, Stops/chip, max Returns, columns를 staging하고 commit한다.
2. Face geometry pipeline이 안정된 뒤 `o_vdma_hsize_bytes_rise/fall`과
   `o_vdma_vsize_lines`를 읽는다.
3. Geometry를 바꾸기 전 해당 VDMA S2MM channel을 정지하거나 park한다.
4. 각 lane의 `HSIZE`에 해당 lane의 `o_vdma_hsize_bytes_*`를 설정한다.
5. `VSIZE`에는 `o_vdma_vsize_lines`를 설정한다.
6. `STRIDE >= HSIZE`로 설정한다. 연속 Frame이면 `STRIDE=HSIZE`를 쓴다.
7. Slope별 frame store는 최소 `STRIDE * VSIZE` byte를 확보한다.
8. 새 Face stream을 허용하기 전에 VDMA를 시작하고 SOF/EOL과 status를
   확인한다.

Face 중간에는 active topology나 columns를 바꾸지 않는다. RTL의 Face
snapshot은 현재 stream geometry를 고정하지만, 일부 Frame을 받은 상태에서
VDMA 설정을 다른 geometry로 바꾸면 메모리 경계가 깨진다.

## 8.6 Backpressure와 Frame 무결성

AXI beat는 `TVALID && TREADY`일 때만 전달된다. `TREADY=0`인 동안 source는
TDATA, TUSER, TKEEP, TLAST를 그대로 유지해야 한다. 현재 output path에는
elastic buffer와 shot-boundary FIFO reset guard가 있다. 그래도 watchdog에
의한 강제 recovery가 발생하면 STAT에 truncation/timeout 이력이 남으므로
소프트웨어는 해당 Face를 폐기해야 한다. 부분 Frame을 정상 geometry로
해석하면 안 된다.

## 9. 권장 소프트웨어 초기화와 운용 순서

1. STAT0 signature `0x4C`와 지원 ABI major를 확인한다.
2. STAT1 capability로 Echo 포함 여부와 active register 수를 확인한다.
3. `INTR_EN=0`으로 mask하고 기존 manual `INTR_FLAG`를 W1C한다.
4. Motor, Laser, TDC staging CTL과 Face/image indexed 데이터를 모두 쓴다.
5. CTL1 `CFG_EPOCH`를 증가시키고 STAT31[18]=1, STAT31[21]=0을 확인한다.
6. Echo delay profile은 CTL15/16 toggle을 사용하고 STAT21 ack를 확인한다.
7. STAT3~5가 아니라 runtime `o_vdma_*` 출력으로 VDMA를 설정한다.
8. 필요한 interrupt source만 enable하고 새 CTL25 command epoch로 TDC
   START를 요청한다.
9. Frame마다 SOF, 모든 EOL, 48-byte prefix, Cell metadata와 17-bit Return
   복원을 확인한다.
10. 오류 시 clear 전에 STAT12, STAT21, STAT27~29, STAT31, INTR_FLAG를 먼저
    기록한다.
11. 운용 오류만 새 epoch로 구분하려면 `ERROR_CLEAR`, 완전한 이력 reset이
    필요하면 `RESET_EPOCH`를 사용한다.

## 10. 해석할 때 반드시 구분할 항목

- `k_dist_fixed`와 `DIST_SCALE`은 전달되는 metadata다. 실제 fixed-point
  Q-format은 calibration 및 소프트웨어 계약이 별도로 소유해야 한다.
- `START_OFF1`은 GPX 설정 image에 반영되고 header에도 기록된다. FPGA Cell
  경로에는 같은 값을 다시 빼는 subtractor가 없으므로 이중 보정하면 안 된다.
- GPX 설정 image의 bit별 칩 기능은 GPX datasheet가 정의한다. CTL20/21은
  raw indexed 접근 창이다.
- STAT3~5는 최대 용량 상수이고 현재 slope geometry가 아니다.
- 현재 Face의 VDMA 기준값은 `o_vdma_hsize_bytes_rise/fall` 및
  `o_vdma_vsize_lines`다.
- `timestamp_ns`라는 RTL 이름과 달리 header timestamp는 AXIS clock count다.
- GPX `Hit[16:0]` 중 bit 16은 metadata에 보존되므로 DDR parser가 반드시
  16-bit slot과 metadata MSB를 결합해야 한다.

## 11. 주요 RTL 교차 참조

| 확인 목적 | 기준 RTL |
|---|---|
| 통합 주소·bit 상수 | `system_integration/rtl/lidar_unified_csr_pkg.vhd` |
| CTL/STAT/IRQ 실제 배치 | `system_integration/rtl/lidar_unified_csr_top.vhd` |
| TDC 통합 config/command/status adapter | `tdc_gpx_unified_csr_adapter.vhd` |
| GPX 28-bit I-Mode bit 분해 | `tdc_gpx_decoder_i_mode.vhd` |
| Chip/Stop/Return tag 추가 | `tdc_gpx_raw_event_builder.vhd` |
| 17-bit Hit Cell 저장과 metadata | `tdc_gpx_cell_builder.vhd` |
| Chip/Stop 정렬과 blank fill | `tdc_gpx_face_assembler.vhd` |
| 32/64/128-bit canonical packing | `tdc_gpx_line_packer.vhd` |
| 48-byte prefix와 SOF/EOL | `tdc_gpx_header_inserter.vhd` |
| Rise/Fall HSIZE와 VSIZE 계산 | `tdc_gpx_top.vhd` |
