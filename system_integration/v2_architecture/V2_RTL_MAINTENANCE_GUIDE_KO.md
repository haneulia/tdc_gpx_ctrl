# TDC-GPX LiDAR v2 RTL 유지보수 가이드

## 1. 목적

이 문서는 v2 통합 RTL을 수정할 때 변수 이름만 보고도 다음 네 가지를 판단할
수 있게 하는 코드 읽기 지도다.

1. 값의 물리적 의미와 단위;
2. 합성 전 Generic인지, Runtime source인지, 승인 시 계산되는 derived 값인지;
3. 값을 소유하는 clock domain과 갱신 시점;
4. 다른 모듈로 전달될 때의 ready/valid, sticky, atomic COMMIT 계약.

레지스터의 정확한 bit 위치는 `V2_UNIFIED_CSR_REGISTER_MAP.md`, DDR ABI는
`V2_PS_HLINE_ETHERNET_ABI_KO.md`, clock/event 경계는
`V2_CLOCK_EVENT_DATA_CONTRACT.md`를 함께 본다.

## 2. 이름과 단위 규칙

| 표기 | 정확한 의미 |
|---|---|
| `G_*` | 합성 전에 고정되는 Generic 또는 그 record인 `G_BUILD_CONFIG` |
| `C_*` | 설계 계약 상수, bit 위치, 배열 상한 또는 reset idle 값 |
| `i_*`, `o_*` | entity 입력과 출력 |
| `*_r` | 해당 process의 clock edge에서만 바뀌는 register |
| `*_c` | register에서 계산되는 조합값; clock domain을 넘기지 않음 |
| `*_meta_r`, `*_sync_r` | 비동기 입력의 1단/2단 동기화 register |
| `*_pulse` | 이름에 적힌 domain에서 정확히 1 clock인 사건 |
| `*_sticky` | 원인을 보존하며 reset 또는 CLEAR_STATUS 전까지 유지 |
| `*_count` | 사건 누적 횟수; 포화/순환 정책은 선언부 주석과 CSR 문서를 확인 |
| `*_5ns_ticks` | 200 MHz clock 수가 아니라 주파수 독립적인 5 ns 시간 단위 |
| `*_clks` | 이름에 적힌 clock domain의 실제 cycle 수 |
| `*_udeg` | micro-degree, 1 degree = 1,000,000 udeg |
| `*_bytes`, `*_words`, `*_beats`, `*_lines` | 서로 다른 geometry 단위이므로 혼용 금지 |

문서와 주석에서는 처음 등장할 때 물리 의미를 먼저 쓰고 RTL 이름을 괄호에
쓴다. 예: `요청 목표 왕복시간(CTL12.TARGET_RANGE)`. 의미가 불분명한
`물리 시간`, `레이저 시간`, `상태값` 같은 축약 명사는 사용하지 않는다.

### 2.1 약어와 도메인 용어

| 용어 | 이 설계에서의 정확한 의미 |
|---|---|
| CSR | PS가 AXI4-Lite로 접근하는 제어·상태 Register 묶음(Control and Status Register) |
| CTL / STAT | 각각 제어 주소 영역과 상태 주소 영역. CTL23/24는 주소만 CTL 영역이며 write 불가 진단 결과 포털 |
| IRQ | 원인 source를 보존하고 PS에 알리는 Interrupt Request. 이 설계는 level-high sticky 기반 |
| TDC | 시간차를 디지털 값으로 변환하는 외부 TDC-GPX와 그 획득 경로(Time-to-Digital Converter) |
| GPX Register | 외부 TDC-GPX Chip 내부의 4-bit 주소/28-bit 값 Register 0..15 |
| MTimer | GPX Reg7[27:15]의 측정 Timer. 40 MHz 기준 25 ns 단위이며 목표 왕복시간에서 파생 |
| IFIFO | GPX I-Mode 측정 결과 FIFO. EF 신호가 빌 때까지 물리 결과를 drain |
| EF / LF | 각각 IFIFO Empty Flag와 Locked Flag인 외부 GPX 입력 pin |
| OEN | GPX 28-bit data bus 출력 허용을 제어하는 Output Enable 출력 pin |
| CDC | 서로 다른 clock domain 사이의 사건·payload 전달(Clock Domain Crossing) |
| AXIS | ready/valid 기반 AXI4-Stream 데이터 인터페이스 |
| VDMA | AXI4-Stream H-Line을 DDR Frame buffer에 쓰는 Video DMA |
| HSIZE / VSIZE / STRIDE | 각각 한 H-Line byte 수, Frame의 Line 수, 다음 DDR Line 시작까지 byte 간격 |
| 측정 기준시각(T0) | 해당 Shot의 레이저 실행 기준시각. Face 기준시각과 delta로 DDR에 기록 |
| Raw28 / Hit17 | 외부 GPX 28-bit 원본 Word와 그 하위 17-bit 거리 측정값 |
| Cell | 동일한 Chip·STOP·Slope에 속하는 Return 묶음과 그 Metadata |
| Shot H-Line | 한 레이저 발사 위치에서 한 slope의 모든 Cell과 Shot Metadata를 담은 DDR 행 |
| Face Footer | Face 완료 count, geometry, fault, commit을 담아 Frame 끝에 쓰는 완료 Record |
| 결측 Shot 열(Hole) | 요청 각도 시한에 측정하지 못했지만 H-Line 번호 보존을 위해 명시적으로 남기는 빈 Shot 행 |
| COMMIT | CTL shadow 전체를 검증·파생한 뒤 Processing/TDC에 같은 version으로 원자 적용하는 명령 |
| ARM / DISARM | 물리 레이저 발사 허용 / 금지. DISARM 중에도 위치 추적과 CSR 접근은 유지 |

## 3. 설정 소유권

| 종류 | 예 | 변경 시점 | 적용 방법 |
|---|---|---|---|
| Build Generic | Chip 수, STOP 수, 최대 Return, Face 수, clock, AXIS 폭 | 합성 전 | XGUI/Generic, runtime 변경 불가 |
| Runtime source | CPR, Face center, 목표 왕복시간, Shot 광학각 | PS 운용 중 | CTL shadow write 후 COMMIT |
| Derived | total states, Shot interval states, Reg7.MTimer, HSIZE/VSIZE | COMMIT validation/계산 | software가 직접 쓰지 않음 |
| Live status | RUN/ARM, ready/busy, GPX pin | 운용 중 | STAT 또는 CTL23/24 진단 portal |
| Sticky diagnostic | timeout, drop, protocol fault | 원인 발생 시 | CLEAR_STATUS 후 IRQ_FLAG W1C |

`CTL12.TARGET_RANGE`는 요청 목표 왕복시간의 유일한 software source다.
외부 GPX 기준 Timer(`Reg7.MTimer[27:15]`)는 40 MHz 기준 25 ns/tick이며
다음과 같이 자동 파생된다.

```text
Reg7.MTimer = ceil(TARGET_RANGE_5NS / 5)
실효 목표 왕복시간 = Reg7.MTimer * 25 ns
```

Reg7.MTimer는 13 bit이므로 0..8191이다. 최대 실효 시간은 204.775 us이며
범위를 넘는 COMMIT은 오류 코드 `0x33`으로 거부한다.

근거는 [`TDC-GPX-Datasheet.pdf`](../../Doc/TDC-GPX-Datasheet.pdf)의
Register 7 표(인쇄면 19쪽)와 I-Mode MTimer 설명(인쇄면 26쪽)이다. 전자는
`MTimer[27:15]`, `0..8191`, `Tref` 단위를 정의하고, 후자는 40 MHz 기준
`25 ns..204.7 us` 운용 범위를 설명한다. 문서의 `204.7 us`는 반올림 표기이며
RTL은 정수식 `8191 * 25 ns = 204.775 us`를 사용한다.

## 4. 패키지별 소유권

| 패키지 | 소유하는 내용 | 수정 시 함께 확인할 항목 |
|---|---|---|
| `lidar_build_pkg` | 최대 용량, 합성 profile, clock/폭/topology 합법성, 오류 enum | XGUI Generic, build validation TB |
| `lidar_config_types_pkg` | Runtime source/derived/active record와 reset shadow | CSR pack/unpack, config TB |
| `lidar_config_reference_pkg` | 정확한 산술 oracle | production에서 호출 금지, sequential calculator 등가성 |
| `lidar_csr_map_pkg` | CTL/STAT/IRQ 주소, bit layout, ABI version, encoding 검사 | 문서, CSR TB, software header |
| `lidar_event_types_pkg` | 위치/Face/Shot/T0/operation 의미 event | pack payload와 pipeline 경계 |
| `lidar_processing_pkg` | monitor AXIS와 Processing 진단/실측 latency | 진단 page `0x10..0x19` |
| `lidar_echo_pkg` | Echo 채널, 지연 profile, snapshot/진단 | 32채널 전개, Processing clock 변환 |
| `lidar_gpx_pkg` | 물리 GPX 4-bit 주소/28-bit data, lane 상태, read service | bus engine, lane/coordinator |
| `lidar_gpx_image_pkg` | 검증된 v1 GPX reset image의 단일 adapter | `tdc_gpx_cfg_pkg` 원본 |
| `lidar_gpx_event_pkg` | Processing↔TDC Shot/STOP/Raw event 직렬화 | pack/unpack 역함수, CDC TB |
| `lidar_gpx_data_pkg` | Raw28, Hit17, Cell, Face-slot 의미와 fault record | B6/B7/B8 회귀 |
| `lidar_gpx_vdma_pkg` | Cell/Shot/Footer Word ABI와 HSIZE/VSIZE/STRIDE | 32/64/128-bit, DDR/HTML Golden |
| `lidar_status_pkg` | CTL23/24 page, 물리 GPX read 결과, runtime IRQ 분류 | CSR 문서, K0-8 회귀 |

## 5. 공개 함수 읽기

### 5.1 Build와 설정

| 함수 | 반환 의미 |
|---|---|
| `fn_is_legal_clock_mhz` | 지원 주파수 50/100/125/150/200 MHz 여부 |
| `fn_is_legal_output_width` | AXIS 폭 32/64/128 bit 여부 |
| `fn_is_binary` | mask에 `0/1` 이외의 `U/X/Z` 등이 없는지 검사 |
| `fn_popcount` | `std_logic_vector` 안의 `1` 개수 계산 |
| `fn_present_chip_mask` | 합성된 Chip 수로 만든 물리 present mask |
| `fn_active_face_mask` | 합성된 Face 수로 만든 Face enable 상한 mask |
| `fn_decode_multiplier` | x1/x2/x4 enum을 1/2/4로 변환 |
| `fn_validate_build_config` | Generic 조합의 최초 오류 |
| `fn_cfg_error_code` | 오류 enum을 PS ABI 8-bit 코드로 변환 |
| `fn_default_runtime_config` | Build profile에 맞는 reset shadow; 아직 active 아님 |
| `fn_total_states` | CPR × physical decode multiplier |
| `fn_face_lower`, `fn_face_upper` | Face window의 inclusive decoded-state 경계 |
| `fn_shot_interval_states` | 요청 광학각을 만족하는 최소 state 수, ceil 적용 |
| `fn_columns_per_face` | inclusive Face 폭과 Shot 간격에서 예상 Shot 열 수 |
| `fn_ticks_to_clocks` | 5 ns 시간을 선택 domain clocks로 올림 변환 |
| `fn_gpx_mtimer_ref_ticks` | 5 ns 목표시간을 25 ns GPX MTimer tick으로 올림 변환 |
| `fn_gpx_effective_target_range_5ns` | MTimer 양자화 뒤 실제 적용할 5 ns 단위 시간 |
| `fn_validate_runtime_config` | Runtime source와 Build 조합의 합법성 oracle |
| `fn_derive_runtime_config` | 승인될 모든 derived 값을 한 record로 계산하는 oracle |

`lidar_config_reference_pkg` 내부 전용 함수인 `fn_circular_distance`는 순환
좌표계에서 두 state 사이의 최단 거리를 계산하고, `fn_capture_window_ext`는
실효 목표 왕복시간과 signed capture 보정을 넓은 signed 값으로 합친다. 두
함수는 package body 내부 검증에만 사용하며 production module의 API가 아니다.

`lidar_config_reference_pkg`는 검증용 넓은 나눗셈을 포함하므로 production
datapath에서 호출하지 않는다. 합성 RTL은 `lidar_config_validator_seq`와
`lidar_config_deriver_seq`의 순차 산술로 같은 결과를 만든다.

### 5.2 Echo와 event

| 함수 | 반환 의미 |
|---|---|
| `fn_echo_channel_count` | `num_chips * stops_per_chip` 활성 물리 채널 수 |
| `fn_echo_channel_index` | `(Chip, STOP)`을 0..31 선형 APD 채널로 변환 |
| `fn_echo_ticks_to_proc_clocks` | Echo 5 ns 지연을 Processing clocks로 올림 변환 |
| `fn_pack_processing_monitor_data` | 위치, 실측 지연, active version과 사건 bit를 64-bit monitor TDATA로 조립 |
| `fn_pack_processing_monitor_user` | Face, source, direction, overlap 등 monitor TUSER를 조립 |
| `fn_operation_command_code/from_code` | operation enum과 CDC 3-bit code 상호 변환 |
| `fn_pack/unpack_operation_state` | read-only operation 상태의 CDC payload 변환 |
| `fn_pack/unpack_shot_request` | Processing Shot 요청의 CDC 변환 |
| `fn_pack/unpack_shot_start` | TDC가 승인한 Shot 시작 context의 CDC 변환 |
| `fn_pack/unpack_shot_context` | Raw GPX event가 들고 갈 Shot 식별자 변환 |
| `fn_pack/unpack_raw_event` | 28-bit GPX data/terminal event의 CDC 변환 |
| `fn_gpx_slope_to_bit/from_bit` | Rise/Fall enum과 ABI slope bit 상호 변환 |

pack/unpack record에 field를 추가할 때는 payload width와 양방향 함수를 같은
변경에서 갱신하고 150/200 및 200/150 MHz 회귀를 모두 실행한다.

### 5.3 GPX acquisition

| 함수 | 반환 의미 |
|---|---|
| `fn_gpx_drain_cap_quads` | Chip slope 능력에 따른 물리 IFIFO 최대 drain 단위 |
| `fn_gpx_events_per_shot_capacity` | 한 Shot에서 result FIFO가 수용할 Raw/terminal event 상한 |
| `fn_gpx_result_fifo_depth` | 위 상한을 수용하는 power-of-two FIFO depth |

`lidar_gpx_pkg` body의 `fn_chip_slope_capacity`, `fn_div_ceil`,
`fn_power_of_two_ceil`은 위 세 용량 함수를 구성하는 내부 정수 산술 helper다.
이 값은 Runtime Return 표시 수가 아니라 합성된 물리 Chip/slope/IFIFO 최악
용량을 계산한다.

이 용량은 합성 topology와 TDC-GPX 물리 IFIFO 경계로 정한다. Runtime Return
표시 수를 줄여도 물리 IFIFO drain 자체를 조기 종료하지 않는다.

### 5.4 VDMA와 DDR ABI

| 함수군 | 의미 |
|---|---|
| `fn_gpx_vdma_effective_max_hits` | Runtime 값 0을 Build 최대 Return으로 해석 |
| `fn_gpx_vdma_cell_word_count/bytes` | 한 Cell의 Metadata+PACKED17 Hit Word 수/byte 수 |
| `fn_gpx_vdma_beat_bytes` | 합성 AXIS 폭을 byte 수로 변환 |
| `fn_gpx_vdma_output_width_code` | Footer/상태에 넣는 32/64/128 폭 code |
| `fn_gpx_vdma_shot_raw_hsize_bytes` | 정렬 전 Shot Line byte 수 |
| `fn_gpx_vdma_shot_hsize_bytes` | AXIS beat 경계로 올린 실제 HSIZE |
| `fn_gpx_vdma_shot_line_beats` | 한 H-Line의 AXIS beat 수 |
| `fn_gpx_vdma_footer_lines` | 32-byte Footer를 HSIZE에 나누어 넣는 Line 수 |
| `fn_gpx_vdma_vsize_lines` | Shot Line 수 + Footer Line 수 |
| `fn_gpx_vdma_stride_bytes` | Runtime 최대 HSIZE를 수용하는 고정 행 간격 |
| `fn_gpx_vdma_max_vsize_lines` | 최대 Shot/최소 HSIZE에서도 필요한 Frame 행 수 |
| `fn_gpx_vdma_frame_allocation_bytes` | 고정 STRIDE × 최대 VSIZE Frame 할당량 |
| `fn_gpx_vdma_cell_word` | Cell Metadata와 PACKED17 Hit Word 생성 |
| `fn_gpx_vdma_shot_metadata_word` | H-Line의 Shot 위치/시간/상태 Word 생성 |
| `fn_gpx_vdma_footer_word` | Face 완료 count/fault/geometry/commit Word 생성 |
| `fn_gpx_vdma_align` | 임의 byte 수를 지정 정렬 단위로 올림 |
| `fn_gpx_vdma_align16` | 과거 128-bit Golden 비교를 위한 16-byte 호환 helper |
| `fn_gpx_vdma_payload_blocks` | Cell payload가 차지하는 16-byte block 수 |
| `fn_gpx_vdma_hsize_bytes/line_beats` | 기존 호출부 호환용 HSIZE/Beat wrapper |
| `fn_gpx_vdma_make_word_event` | 32-bit 의미 Word와 Line/Frame 표지를 event record로 조립 |

`fn_active_hit_mask`는 Runtime Return 수만큼 낮은 bit를 `1`로 만드는 package
body 내부 helper다. Cell 사이 padding을 만들지 않으며 Metadata의 유효 Hit
표시에만 사용한다.

`fn_gpx_vdma_align16`은 legacy 128-bit block 비교용 보조 함수다. 실제 HSIZE는
`fn_gpx_vdma_align(value, fn_gpx_vdma_beat_bytes(output_width))`를 사용하므로
32/64/128-bit 합성 폭마다 각각 4/8/16-byte 경계가 적용된다.

### 5.5 CSR와 상태/진단

| 함수 | 반환 의미 |
|---|---|
| `fn_ctl/stat/irq_byte_offset` | Word index를 AXI4-Lite byte 주소로 변환 |
| `fn_decode_mode_bits` | x1/x2/x4 enum을 CSR 2-bit encoding으로 변환 |
| `fn_direction_bit` | CW/CCW enum을 CSR 1-bit encoding으로 변환 |
| `fn_pack/unpack_runtime_config` | Runtime source record와 CTL1..20 Word 상호 변환 |
| `fn_ctl_word_encoding_valid` | reserved bit, enum, 범위를 포함한 CTL 쓰기 encoding 검사 |
| `fn_diag_is_processing` | CTL23 index가 Processing 진단 page인지 판정 |
| `fn_diag_is_tdc` | CTL23 index가 TDC 진단 또는 물리 GPX read page인지 판정 |
| `fn_diag_is_gpx_register_read` | index 형식이 `11CCAAAA`인지 판정 |
| `fn_diag_gpx_register_chip` | `11CCAAAA`에서 Chip 번호 `CC` 추출 |
| `fn_diag_gpx_register_address` | `11CCAAAA`에서 Register 주소 `AAAA` 추출 |
| `fn_pack_gpx_register_read_word` | `{요청 주소[3:0], 실제 GPX 값[27:0]}` 조립 |
| `fn_pack_diag_response` | 32-bit 결과와 ERROR bit를 33-bit mailbox 응답으로 조립 |

### 5.6 초기값 어댑터

`lidar_gpx_image_pkg.fn_default_image`는 v1의 보드 검증 Register 0..15 배열을
동일 주소 순서의 v2 `gpx_register_image_t` 상수로 한 번 변환하는 package body
내부 함수다. Runtime 회로가 아니며 기본값 literal의 소유권을 복제하지 않는다.

## 6. 외부 GPX 데이터 흐름

```text
TDC-GPX pin D[27:0]
  -> lidar_gpx_bus_engine
  -> lidar_gpx_acquisition_lane
  -> lidar_gpx_acquisition_coordinator
  -> Processing/TDC CDC result gateway
  -> lidar_gpx_hit_decoder       : Raw28 -> Hit17 + Chip/STOP/Slope
  -> lidar_gpx_cell_collector    : Return 1..N + Cell Metadata
  -> lidar_gpx_frame_lane_assembler
  -> lidar_gpx_shot_line_builder : Shot Metadata + Cell Words
  -> lidar_gpx_hole_line_expander: 결측 Shot 열을 같은 HSIZE의 빈 Line으로 보충
  -> lidar_gpx_face_footer_builder
  -> lidar_gpx_axis_word_packer  : 32-bit 의미 Word를 32/64/128-bit AXIS로 packing
  -> AXI VDMA -> DDR -> PS H-Line decoder -> Ethernet Viewer
```

I-Mode Raw28 bit 의미는 다음과 같다.

```text
[27:26] channel code
[25:18] START number
[17]    slope, 0=Rise, 1=Fall
[16:0]  distance Hit value
```

Cell은 하나의 `(Chip, STOP, Slope)`에 속한 최대 7 Return과 Cell Metadata다.
Shot H-Line은 해당 slope에서 활성인 모든 Cell slot과 Shot Metadata를 포함한다.
Face Footer는 Face 전체의 완료 count, geometry, fault와 commit marker를 담는다.

## 7. 결측 Shot 열(Hole)

결측 Shot 열(Hole)은 요청 각도 격자에는 존재하지만 그 정확한 후보점에서
레이저 실행기 또는 GPX 획득기가 준비되지 않아 측정하지 못한 column이다.

- 늦은 각도에서 재발사하지 않는다.
- `schedule_overrun` sticky/count를 남긴다.
- DDR에는 정상 Line과 동일 HSIZE의 빈 Line과 Hole metadata를 쓴다.
- 따라서 뒤의 정상 Shot index와 Viewer H-Line 번호가 이동하지 않는다.

`hole`은 빈 padding byte나 아직 사용하지 않는 reserved bit를 뜻하지 않는다.

## 8. CTL21/22와 실제 GPX read의 차이

| 경로 | 의미 | 물리 Chip bus read 여부 |
|---|---|---|
| CTL21/22 staging view | 다음 COMMIT에 쓸 GPX 설정 후보 | 아니오 |
| CTL21/22 active view | 마지막 성공 COMMIT에서 쓰려고 적용한 effective image | 아니오 |
| CTL23/24 `11CCAAAA` | 외부 Chip의 현재 Register 실제 readback | 예 |

목표 왕복시간만 바꾸면 CTL12만 수정한다. CTL21/22는 MTimer 이외의 GPX bit를
수정할 때만 필요하다. 실제 Chip 상태 확인 절차는 다음과 같다.

1. DISARM 후 STAT3의 ARMED/SCHEDULER/PHYSICAL_FIRE/SIMULATION enable이 0인지 확인;
2. CTL23에 `0x100 | 0xC0 | (Chip<<4) | Address` 기록;
3. CTL23의 BUSY=0, VALID=1, SEQUENCE 증가, ERROR=0 확인;
4. CTL24에서 `{Address[3:0], GPX_Data[27:0]}` 확인;
5. 필요한 Register를 모두 읽은 뒤 ARM.

읽기 동안 RTL은 GPX acquisition RUN만 자동 pause하고 safe를 기다린다. CSR과
Processing clock은 멈추지 않는다. COMMIT/RUN/ARM은 완료 전까지 거부한다.

## 9. IRQ 분류

| Bit | 이름 | 한 문장 정의 |
|---:|---|---|
| 0 | COMMIT_SUCCESS | 설정 transaction이 오류 없이 active가 됨 |
| 1 | COMMIT_ERROR | 시작된 transaction이 오류로 종료됨 |
| 2 | COMMIT_REJECTED | 새 transaction이 BUSY/ordering 때문에 시작되지 못함 |
| 3 | RECOVERY_REQUIRED | 동일 active version 보장이 깨져 coordinated reset 필요 |
| 4 | ACCESS_ERROR | CSR 주소/encoding/command/portal 사용 오류 |
| 5 | PROCESSING_WARNING | 위치/Face/Shot 일정/monitor/laser lifecycle 계약 오류 |
| 6 | LASER_TIMEOUT | fire_pulse 뒤 fire_done 미수신 |
| 7 | ECHO_DIAGNOSTIC | Echo window/profile/snapshot 진단 오류 |
| 8 | GPX_TRANSPORT | CDC, 물리 bus, IFIFO drain, controller, register read 오류 |
| 9 | GPX_DATA | Raw28/Hit17/Cell/Face 의미와 순서 오류 |

Source 5의 이름은 ABI 호환 때문에 WARNING이지만 `schedule_overrun`은 단순
경고가 아니라 요청 광학각 시한을 놓친 Shot 시간 계약 오류다.

`C_LIDAR_IRQ_REGISTER_COUNT=4`는 IRQ 제어 Register
`ENABLE/STATUS/FLAG/ACK`의 개수다. 실제 독립 사건 수는
`C_LIDAR_IRQ_SOURCES=10`이며 위 표의 Bit 0..9에 대응한다. 두 상수를 IRQ
개수라는 하나의 의미로 혼용하지 않는다.

일반 sticky IRQ는 `CLEAR_STATUS -> IRQ_STATUS=0 확인 -> IRQ_FLAG W1C` 순서로
지운다. `RECOVERY_REQUIRED`는 CLEAR_STATUS로 지우지 않고 coordinated reset을
수행한 뒤 W1C한다.

## 10. 내부 RTL 주석 기준

다음 항목에는 선언 또는 상태 전이 바로 위에 한글 주석을 유지한다.

- clock domain을 넘는 event와 payload 안정 조건;
- physical pin에 직접 영향을 주는 fire_pulse, start_tdc, stop_tdc, OEN;
- 5 ns tick, Processing clock, TDC clock, GPX 25 ns reference 변환;
- 자동 pause/resume, DISARM, COMMIT ordering;
- sticky를 세우는 원인과 clear 소유자;
- HSIZE/VSIZE/STRIDE 및 DDR ABI를 바꾸는 산식;
- timeout의 시작 사건, 종료 사건, 단위, 0의 의미;
- 이름만으로 의미가 불분명한 축약어의 최초 선언.

단순 loop index, ready/valid 배선, 한 줄 산술을 반복 설명하는 주석은 추가하지
않는다. 코드와 다른 오래된 주석이 가장 위험하므로 동작 변경과 같은 commit에서
관련 문서와 주석을 함께 갱신한다.

## 11. 변경 전 체크리스트

1. 이 값은 Build, Runtime source, Derived, Live, Sticky 중 무엇인가?
2. 단위가 이름에 드러나는가?
3. 어느 clock domain이 유일한 writer인가?
4. CDC payload는 ACK까지 안정적인가?
5. COMMIT 전 shadow write가 즉시 동작을 바꾸지 않는가?
6. 물리 레이저/GPX bus 변경은 DISARM과 safe 조건을 지키는가?
7. 32/64/128-bit에서 같은 의미 Word와 byte order를 유지하는가?
8. 150/200 및 200/150 MHz에서 기능 회귀를 통과하는가?
9. CSR ABI minor, register map, XGUI, software decoder 문서를 함께 갱신했는가?
10. 새 sticky의 진단 page, IRQ 분류, clear 방법이 정의됐는가?
