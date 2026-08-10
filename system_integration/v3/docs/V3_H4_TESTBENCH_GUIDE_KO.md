# V3 H4 테스트벤치 운용 및 유지보수 가이드

## 1. 목적

H4 테스트는 단순히 HLS 함수가 실행되는지만 확인하지 않는다. H3 정렬 Cell이
V2와 동일한 Shot Metadata, PACKED17 Cell, Hole Line, Face Footer가 되고, 최종
32/64-bit AXI4-Stream Beat까지 동일함을 단계별로 증명한다.

## 2. 테스트 계층

| 계층 | 테스트 | 검증 책임 |
|---|---|---|
| 알고리즘 | CSim | Word 내용, index, Line/Frame 경계, fault 분류 |
| 생성 RTL | C/RTL CoSim | C++ 결과와 HLS 생성 RTL의 cycle-accurate 결과 일치 |
| V2 Golden | VHDL 직접 비교 | V2 전체 Lane pipeline과 최종 AXI Beat 정확히 일치 |
| 물리 구현 | OOC 배치·배선 | 150/200 MHz WNS, latch, blocking DRC, 자원 |

한 계층의 PASS가 다른 계층을 대신하지 않는다.

## 3. CSim/CoSim 테스트벤치

파일:
`hls/gpx_lane_word_formatter/tb/tb_gpx_lane_word_formatter_hls.cpp`

| 함수/Profile | 생성 이유 | 반드시 유지할 관찰점 |
|---|---|---|
| `test_return_sweep` / `return_sweep` | Return 1~7에서 PACKED17 크기가 달라짐 | Cell Word 수, Hit low16, Hit[16], valid bitmap, Metadata marker |
| `test_multi_cell_line` / `multi_cell` | 한 Shot Line에 여러 Cell이 직렬화됨 | Cell 순서, 전역 Word index, 마지막 Cell에서만 Line 종료 |
| `test_holes_and_footer_32` / `holes_footer_32` | 누락 Shot도 VDMA Line 수를 보존해야 함 | Hole Metadata, zero Cell payload, trailing Hole, Footer 8 Word |
| `test_all_hole_footer_64` / `all_hole_footer_64` | 실제 Shot이 하나도 없는 Face 복구 | 전체 Hole 수, 64-bit 정렬 HSIZE, Frame end, commit marker |
| `test_reset_epoch_and_faults` / `reset_faults` | abort 뒤 이전 Face 상태가 누출되면 안 되며 Formatter 계약 오류가 구분돼야 함 | epoch 초기화, 7개 fault exact bitmap, Bit 7 예약 0, TDC Drain timeout과 V2 Shot timeout/abort Bit 비혼용 |

Profile은 환경 변수 `V3_HLS_PROFILE`로 선택되며 실행 스크립트가 다섯 Profile을
순서대로 모두 수행한다.

## 4. V2 직접 비교 테스트벤치

파일:
`tb/tb_lidar_gpx_lane_word_formatter_hls_diff.vhd`

### 4.1 비교 대상

- V2: Cell serializer → Shot Metadata → Hole expander → Face Footer → AXIS packer
- V3: H4 HLS formatter Adapter → 동일 AXIS packer

최종 비교 신호는 `TDATA`, `TKEEP`, `TSTRB`, `TUSER(0)=SOF`, `TLAST`다.

### 4.2 Profile

| Top | 의미 |
|---|---|
| `..._min32_150` | 최소 1 Cell, 1 Return, 32-bit, 150 MHz |
| `..._max32_200` | 최대 32 Cell, 7 Return, 32-bit, 200 MHz |
| `..._mid64_150` | 16 Cell, 3 Return, Fall, 64-bit, 150 MHz |
| `..._max64_200` | 최대 32 Cell, 7 Return, 64-bit, 200 MHz |

### 4.3 시나리오

1. 첫 Face에서 Shot 0 실제, Shot 1 Hole, Shot 2 실제, Shot 3 trailing Hole을 만든다.
2. 실제 Cell의 유효 Return 수를 Runtime 직렬화 Return 슬롯 수보다 작게 만든다.
3. backpressure를 넣고 양쪽 출력이 멈춘 동안 payload가 변하지 않는지 확인한다.
4. 다음 Face는 전체 Hole로 만들고 Footer의 완료 수와 fault 요약을 확인한다.
5. Line 중간 abort를 넣고 reset epoch 이후 이전 Word가 나타나지 않는지 확인한다.

두 구현의 latency는 비교 대상이 아니다. 먼저 도착한 쪽을 정지시키고 양쪽 Beat가
모두 준비됐을 때만 비교한다. 이 동기 비교 구조를 제거하면 latency 차이를 데이터
오류로 오판한다.

## 5. 구현 타이밍 래퍼

파일:
`tb/lidar_gpx_lane_word_formatter_hls_impl.vhd`

이 래퍼는 4 Chip × 8 STOP × 7 Return, Rise/Fall 두 경로를 모두 인스턴스화한다.
외부 포트를 줄이거나 상수에 묶으면 합성기가 경로를 제거할 수 있으므로, 구현
타이밍 점검 목적을 이해하지 않고 포트를 단순화하지 않는다.

스크립트는 다음 네 조합을 각각 합성·배치·배선한다.

- 150 MHz / 32-bit
- 200 MHz / 32-bit
- 150 MHz / 64-bit
- 200 MHz / 64-bit

각 조합은 timed path 존재, WNS ≥ 0, latch 0, blocking DRC 0을 모두 만족해야 한다.

## 6. 실행 순서

```powershell
# 1. 알고리즘과 생성 RTL
./system_integration/v3/scripts/run_v3_hls_lane_word_formatter.ps1 -Step all

# 2. V2 Golden 직접 비교
./system_integration/v3/scripts/run_v3_gpx_lane_word_formatter_diff.ps1 `
    -SkipHlsSynthesis

# 3. 실제 OOC 배치·배선
./system_integration/v3/scripts/run_v3_gpx_lane_word_formatter_impl.ps1 `
    -SkipHlsSynthesis
```

주요 PASS marker:

- `LIDAR_V3_HLS_GPX_LANE_WORD_FORMATTER_CSIM_PASS`
- `LIDAR_V3_HLS_GPX_LANE_WORD_FORMATTER_COSIM_PASS`
- `LIDAR_V3_GPX_LANE_WORD_FORMATTER_DIFF_PASS`
- `LIDAR_V3_GPX_LANE_WORD_FORMATTER_IMPL_PASS`

## 7. 계약 변경 시 필수 회귀

| 변경 | 필수 회귀 |
|---|---|
| H2 Cell 필드/Return 의미 | H2 CSim·차등 + H3 CSim·차등 + H4 전체 |
| H3 Cell slot/Face-close | H3 전체 + H4 전체 |
| Shot Metadata Bit | H4 전체 + PS decoder Golden |
| PACKED17 Cell Metadata Bit | Return 1~7 + V2 직접 비교 + PS decoder Golden |
| Footer Word/marker | Hole/전체 Hole + VDMA Frame 완료 + PS decoder Golden |
| 출력 폭/정렬 | 32/64-bit 직접 비교와 OOC 네 조합 |
| abort/reset epoch | 중간 Line abort와 다음 Face stale 데이터 검사 |

## 8. 유지보수 주의사항

1. `PACKED17`은 Hit 저장 ABI이며 AXI4-Stream 폭이 아니다.
2. 합성 물리 최대 Return 수, Runtime 직렬화 Return 슬롯 수, 실제 유효 Return 수를
   하나의 이름으로 합치지 않는다.
3. Footer의 논리 크기는 8 Word/32 byte다. HSIZE에 따라 1~2 Line이 되지만 내용
   크기가 변하는 것은 아니다.
4. Hole Line도 계획된 HSIZE 전체를 방출해야 VDMA의 Shot column 위치가 보존된다.
5. Reserved bit는 0이어야 하며 Adapter assertion과 fault 테스트를 동시에 유지한다.
6. 200 MHz OOC WNS 여유가 작으므로 Parent 전체 배치·배선 결과를 반드시 별도로
   확인한다.
7. `.work`, `.Xil`, 생성 Verilog, `.log`, `.jou`, `.wdb`는 Git에 넣지 않는다.
8. Cell의 TDC-GPX Drain `timeout_cause`를 현재 입력에 없는 레이저 Shot
   timeout/abort Footer Bit로 재해석하지 않는다.

## 9. 아직 검증하지 않는 범위

- H1~H4 혼합 Top의 장시간 FIFO 점유율
- 실제 VDMA DDR 주소/STRIDE 재설정과 Frame buffer handoff
- PS cache 동기화 뒤 H-Line/Ethernet packet 결과
- 실제 TDC-GPX, LVDS Echo, 레이저, 모터가 연결된 보드 동작

이 범위는 H5/H6 및 보드 단계가 소유한다.
