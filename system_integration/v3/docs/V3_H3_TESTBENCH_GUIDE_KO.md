# V3 H3 GPX Frame Assembler 테스트벤치 가이드

## 1. 문서 목적

이 문서는 H3 Cell-to-Frame 변환의 테스트 소유권과 유지보수 규칙을 정의한다.
H3는 Cell의 Hit 내용을 다시 계산하지 않고, Shot/Lane/Face 구조와 순서를 만든다.
따라서 값 비교뿐 아니라 Backpressure, 완료 시점, Abort와 gap 진단을 함께
검증해야 한다.

## 2. 권장 수행 순서

| 순서 | 실행기 | 검증 목적 | 실패 시 먼저 볼 경계 |
|---:|---|---|---|
| 1 | `run_v3_hls_frame_assembler.ps1 -Step csim` | HLS 정렬·gap·fault 알고리즘 | HLS 함수와 Bit Map |
| 2 | `run_v3_hls_frame_assembler.ps1 -Step csynth` | 합성 가능성, Loop II, LUTRAM | HLS pragma와 저장 구조 |
| 3 | `run_v3_hls_frame_assembler.ps1 -Step cosim` | 생성 RTL과 C 모델 일치 | HLS AXIS handshake와 가변 출력 |
| 4 | `run_v3_gpx_frame_assembler_diff.ps1` | V2 RTL과 V3 HLS 관측 계약 비교 | Adapter, 순서, 완료, Abort |
| 5 | `run_v3_gpx_frame_assembler_impl.ps1 -SkipHlsSynthesis` | xc7z020 150/200 MHz 구현 | WNS, latch, DRC, 배선 경로 |

H3 소스나 공통 계약 헤더를 바꾼 뒤에는 1~5를 모두 실행한다. 공통 계약 헤더를
바꿨다면 H1/H2 전체 HLS, 차등 회귀와 구현도 다시 실행한다.

## 3. HLS 독립 C 테스트

### 대상

- 구현: `hls/gpx_frame_assembler/src/gpx_frame_assembler_hls.cpp`
- 테스트: `hls/gpx_frame_assembler/tb/tb_gpx_frame_assembler_hls.cpp`
- 계약: `hls/common/include/lidar_v3_hls_contract.hpp`

### 다섯 Profile

| Profile | 핵심 목적 |
|---|---|
| `dedicated` | 2 Rise + 2 Fall 전용 Chip, column gap, Face close/all-hole |
| `one_chip_dual` | 한 Chip 양 Edge와 Reset Epoch 주소 재사용 |
| `fall_off` | Fall 출력이 전혀 없는 구성 |
| `reduced_faults` | 3 Chip/6 STOP, 누락·중복·mask 위반 오류 |
| `all_dual` | 네 Chip 모두 양 Edge 최대 슬롯 구성 |

Cell을 Chip/STOP 역순으로 넣은 뒤 출력은 canonical 오름차순이어야 한다. 누락
Cell은 삭제되지 않고 blank Cell로 같은 Slot에 남아야 한다.

전체 Profile 묶음은 H3 오류 bitmap 여덟 항목을 모두 실제 입력으로 발생시킨다.
`context_mismatch`, `unexpected_cell`, `duplicate_cell`, `duplicate_terminal`,
`missing_cell`, `geometry_error`, `column_gap`, `masked_payload_drop` 중 하나라도
검출되지 않으면 CSim 또는 차등 회귀가 실패해야 한다.

### 이 테스트가 보장하지 않는 것

C 모델은 Clock 단위 Backpressure, Adapter FIFO, 실제 `shot_done` 소비 시점과
Abort 중 생성 RTL Flush를 검증하지 않는다. 이는 차등 테스트가 맡는다.

## 4. HLS C/RTL Co-simulation

`run_v3_hls_frame_assembler.ps1 -Step cosim`은 다섯 Profile 각각에 대해 생성
Verilog를 C 테스트와 함께 실행한다. 다음을 확인한다.

- LUTRAM 동기 읽기 지연 뒤에도 Cell 순서가 같음
- Rise/Fall 가변 출력 개수와 Control 결과가 같음
- Face close/trailing/all-hole 계산이 같음
- AXI4-Stream Backpressure에서 호출이 교착되지 않음

실행기는 다섯 Profile 보고서의 Verilog `Pass`를 모두 확인한 뒤에만 최종 PASS를
출력한다.

## 5. V2 RTL/HLS 차등 테스트

### 대상

- 테스트: `tb/tb_lidar_gpx_frame_assembler_hls_diff.vhd`
- Golden: V2 `lidar_gpx_frame_lane_assembler`
- 비교 대상: `rtl/bridges/lidar_gpx_frame_lane_assembler_hls_adapter.vhd`
- 생성 RTL: `.work/v3_hls_frame_assembler_component/hls/syn/verilog/`

```text
                           +--> V2 Frame Assembler ----+
동일 Cell/Face close ------|                             +--> Lane/Control 비교
                           +--> V3 HLS + RTL Adapter ---+
```

입력은 두 DUT가 모두 ready일 때만 함께 승인한다. Rise와 Fall 출력은 Lane별로 두
DUT가 모두 valid일 때 함께 소비한다. 따라서 Pipeline latency는 허용하지만 다음
항목은 완전 일치해야 한다.

- 319-bit Cell payload와 360-bit Frame Cell record
- Cell Slot index/count와 line/column 경계
- Rise/Fall 출력 순서
- Face close, trailing gap, all-hole
- Shot 완료 context
- 오류 pulse 및 sticky

### Clock 및 Profile 행렬

다섯 Profile x 150/200 MHz, 총 10개 snapshot을 모두 실행한다.

### Backpressure 규칙

- Rise와 Fall ready는 서로 다른 주기로 토글한다.
- valid=1, ready=0인 동안 payload 전체가 고정돼야 한다.
- 한 Lane만 먼저 소비돼도 다른 Lane의 데이터가 소실되면 안 된다.
- `shot_done`은 필요한 두 Lane의 마지막 Slot이 모두 소비되기 전에는 나오면
  안 된다.

### Abort 규칙

Abort는 HLS가 한 Shot을 생성했고 외부 Lane 출력이 Backpressure로 남은 상태에서
주입한다. 단순 Idle Abort만 시험하면 보류 데이터 오염을 검출할 수 없다.

Abort 뒤에는 같은 Chip/STOP 저장 주소를 즉시 재사용해 이전 payload가 다시
나오지 않는지 확인한다.

## 6. OOC 구현 Harness

### 대상

- Harness: `tb/lidar_gpx_frame_assembler_hls_impl.vhd`
- 실행기: `scripts/run_v3_gpx_frame_assembler_impl.ps1`
- Device: `xc7z020clg484-2`

Harness는 4 Chip x 8 STOP x Rise/Fall 전체 활성의 최대 구성을 사용한다.
150 MHz와 200 MHz 각각에서 합성·배치·배선하고 다음 조건을 자동 검사한다.

- WNS 0 ns 이상
- Latch 0개
- Critical Warning/Error 등급 DRC 0건
- 실제 routed timing report 생성

H3는 넓은 상태와 AXIS 경계 때문에 HLS 스케줄 목표를 4 ns로 설정한다. 외부 제품
Clock 요구는 150/200 MHz이며, 통과 판정은 HLS 추정치가 아니라 Vivado routed
WNS로 한다.

## 7. PASS 표시

```text
LIDAR_V3_HLS_GPX_FRAME_ASSEMBLER_CSIM_PASS
LIDAR_V3_HLS_GPX_FRAME_ASSEMBLER_COSIM_PASS
LIDAR_V3_GPX_FRAME_ASSEMBLER_DIFF_PASS
LIDAR_V3_GPX_FRAME_ASSEMBLER_IMPL_PASS
```

`run_v3_hls_frame_assembler.ps1 -Step all`을 한 번에 실행한 경우에는 마지막에
`LIDAR_V3_HLS_GPX_FRAME_ASSEMBLER_ALL_PASS`가 추가로 출력된다.

중간 Profile PASS만으로 전체 성공을 선언하지 않는다. 실행기가 오류로 끝났거나
최종 표시가 없으면 실패다.

## 8. 유지보수 시 테스트 추가 기준

- 새 topology를 허용하면 CSim Profile과 150/200 MHz 차등 wrapper를 함께 추가한다.
- 새 오류 Bit를 추가하면 pulse, sticky, clear 및 같은 Clock 재발생 시험을 넣는다.
- Cell 정렬 규칙을 바꾸면 누락 Cell과 PS 채널 위치 보존 여부를 함께 검증한다.
- FIFO 깊이나 skid 구조를 바꾸면 독립 Rise/Fall Backpressure와 Abort 시험을
  반드시 다시 실행한다.
- Shot/Face 완료 의미를 바꾸면 H4/H5 소비자와 동시에 변경하고 단독 H3 PASS만
  사용하지 않는다.
- 테스트 편의를 위해 V2 Golden과 HLS 입력을 서로 다른 Clock에 승인하면 차등
  비교 자체가 무효가 된다.

## 9. 결과 보존

생성 HLS 프로젝트, Vivado snapshot, 로그와 report는 저장소 루트 `.work/` 아래에
둔다. Git에는 HLS 원본, Adapter, 테스트벤치, 재현 스크립트와 핵심 결과 문서만
추적한다. `.Xil`, `.log`, `.jou`, `.wdb`와 생성 Verilog는 H3 체크포인트에 넣지
않는다.
