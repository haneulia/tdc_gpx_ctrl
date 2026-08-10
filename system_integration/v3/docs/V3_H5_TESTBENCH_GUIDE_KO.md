# V3 H5 테스트벤치 운용 및 유지보수 가이드

## 1. 목적

H5 테스트는 H1~H4가 각각 통과했다는 사실만 재사용하지 않는다. 모든 Stage와
유지 RTL AXI packer를 실제 연결한 뒤, 같은 Raw 28-bit 입력이 V2와 동일한 최종
Rise/Fall AXI Beat가 되는지 검증한다. Stage 사이 latency, backpressure와 abort가
결합될 때만 나타나는 오류를 찾는 것이 핵심 목적이다.

## 2. 테스트 계층

| 계층 | 테스트 | 검증 책임 |
|---|---|---|
| H1 단위 | `tb_lidar_gpx_hit_decoder_hls_diff.vhd` | Raw28-to-Hit17, fault, inflight/idle 계측 |
| H2 단위 | `tb_lidar_gpx_cell_collector_hls_diff.vhd` | Return 수집, 전시 필터, overflow/timeout, abort |
| H3 단위 | `tb_lidar_gpx_frame_assembler_hls_diff.vhd` | Rise/Fall 순서, Face gap/close, 독립 stall, abort |
| H4 단위 | `tb_lidar_gpx_lane_word_formatter_hls_diff.vhd` | Metadata/PACKED17/Hole/Footer와 32/64-bit 최종 Beat |
| H5 종단 | `tb_lidar_gpx_hls_mixed_data_top_diff.vhd` | H1~H4 결합 경로의 V2 Golden 정확 비교 |
| 물리 구현 | `lidar_gpx_hls_mixed_data_top_impl.vhd` | 4 Chip 최대 구성의 150/200 MHz 배치·배선 |

각 계층의 PASS는 다른 계층을 대신하지 않는다. H5를 수정한 뒤 H5 종단 테스트만
통과해도, 수정한 Stage의 단위 fault 분류가 보존됐다고 단정할 수 없다.

## 3. H5 종단 차등 테스트

파일:
`tb/tb_lidar_gpx_hls_mixed_data_top_diff.vhd`

### 3.1 비교 경로

```text
동일 Raw 28-bit 입력
  +-> V2 H1/H2/H3 -> V2 Lane pipeline -> V2 AXI packer
  +-> V3 H1/H2/H3 -> V3 H4 formatter -> 유지 RTL AXI packer

비교: TDATA, TKEEP, TSTRB, TUSER(0), TLAST
```

V2와 V3의 내부 latency는 비교 대상이 아니다. 먼저 도착한 경로의 `TREADY`를
낮추고 양쪽 `TVALID`가 모두 1인 경우에만 같은 Cycle에 승인한다. 이 구조를
제거하면 정상적인 pipeline latency 차이를 데이터 오류로 오판한다.

### 3.2 Profile

| Top suffix | clock/폭 | slope 구성 | Runtime 직렬화 Return 슬롯 |
|---|---|---|---:|
| `dedicated_32_150` | 150 MHz / 32-bit | Rise Chip 0~1, Fall Chip 2~3 | 1 |
| `dedicated_64_200` | 200 MHz / 64-bit | Rise Chip 0~1, Fall Chip 2~3 | 3 |
| `all_dual_32_150` | 150 MHz / 32-bit | 4 Chip 모두 Rising/Falling | 7 |
| `all_dual_64_200` | 200 MHz / 64-bit | 4 Chip 모두 Rising/Falling | 7 |
| `rise_only_32_200` | 200 MHz / 32-bit | 4 Chip Rising, Falling 합성 제거 | 7 |

모든 Profile은 4 Chip, Chip당 8 STOP, 물리 Return 7개를 사용한다. Rise-only는
Fall formatter/packer의 비활성 generate 경로와 Face 완료 조건까지 검사한다. Runtime
직렬화 Return 슬롯이 1 또는 3이어도 물리 입력을 7개까지 보내 의도적인 전시
필터가 fault나 데이터 정렬 오류를 만들지 않는지 검사한다.

### 3.3 수행 순서

1. Reset을 해제하고 각 Stage의 입력 허용 창이 열릴 때까지 기다린다.
2. 완성되지 않은 Shot에 일부 Return을 넣은 뒤 abort를 발생시킨다.
3. sticky를 지우고 H1~H4, close fork와 packer 전체가 idle로 복귀하는지 확인한다.
4. 두 Shot으로 구성한 정상 Face를 입력한다.
5. Rise/Fall sink에 서로 다른 주기의 backpressure를 건다.
6. 실제 Cell, 누락 Shot Line과 Face Footer를 생성한다.
7. V2/V3의 모든 AXI Beat와 Line/Frame 완료 수를 비교한다.
8. H1 inflight counter가 0으로 돌아오고 H5 전체 idle이 1인지 확인한다.

### 3.4 반드시 유지할 assertion

- backpressure 중 `TDATA/TKEEP/TSTRB/TUSER/TLAST` 안정
- V2/V3 Beat 수와 Line 수 일치
- `TUSER(0)` SOF와 `TLAST` Line 종료 위치 일치
- Rise/Fall Face 완료 수와 통합 Frame 완료 일치
- abort 전 payload가 다음 Face에 나타나지 않음
- H1 결과 없는 입력 또는 inflight overflow 없음
- 모든 Stage fault summary와 geometry/idle 상태 일치

## 4. 구현 타이밍 Harness

파일:
`tb/lidar_gpx_hls_mixed_data_top_impl.vhd`

Harness는 합성기가 경로를 제거하지 않도록 다음 최대 구성을 유지한다.

- 4개 TDC-GPX Chip
- Chip당 8 STOP
- 물리 최대 Return 7
- 4 Chip 모두 Rising/Falling 가능
- Rise/Fall H4와 AXI packer 동시 인스턴스
- 합성 시 출력 폭 32 또는 64 bit

실행 스크립트는 150/200 MHz와 32/64-bit를 직교 조합한 네 구현을 각각 새로
합성·배치·배선한다. 각 Profile은 다음 조건을 모두 만족해야 한다.

- setup WNS >= 0
- hold WHS >= 0
- latch 0
- DRC Error/Critical Warning category 0
- routing 오류 net 0
- timed setup/hold path 존재

WNS가 양수여도 수십 ps 수준이면 Parent Sign-off 여유로 보지 않는다.

## 5. 실행 순서

```powershell
# 1. 변경한 Stage 단위 회귀
./system_integration/v3/scripts/run_v3_gpx_hit_decoder_diff.ps1 `
    -SkipHlsSynthesis
./system_integration/v3/scripts/run_v3_gpx_cell_collector_diff.ps1 `
    -SkipHlsSynthesis
./system_integration/v3/scripts/run_v3_gpx_frame_assembler_diff.ps1 `
    -SkipHlsSynthesis
./system_integration/v3/scripts/run_v3_gpx_lane_word_formatter_diff.ps1 `
    -SkipHlsSynthesis

# 2. H5 종단 V2 Golden 비교
./system_integration/v3/scripts/run_v3_hls_mixed_top_diff.ps1 `
    -SkipHlsSynthesis

# 3. xc7z020 실제 OOC 구현
./system_integration/v3/scripts/run_v3_hls_mixed_top_impl.ps1 `
    -SkipHlsSynthesis
```

주요 PASS marker:

- `LIDAR_V3_GPX_HIT_DECODER_DIFF_PASS`
- `LIDAR_V3_GPX_CELL_COLLECTOR_DIFF_PASS`
- `LIDAR_V3_GPX_FRAME_ASSEMBLER_DIFF_PASS`
- `LIDAR_V3_GPX_LANE_WORD_FORMATTER_DIFF_PASS`
- `LIDAR_V3_H5_MIXED_DATA_TOP_DIFF_PASS`
- `LIDAR_V3_H5_MIXED_DATA_TOP_IMPL_PASS`

## 6. 변경 영향과 필수 회귀

| 변경 대상 | 최소 필수 회귀 |
|---|---|
| H1 Raw/Hit 또는 inflight/idle | H1 + H2 + H5 |
| H2 Return/Cell/fault | H2 + H3 + H4 + H5 |
| H3 Rise/Fall 순서 또는 Face-close | H3 + H4 + H5 |
| H4 Metadata/PACKED17/Hole/Footer | H4 + H5 + 향후 PS decoder Golden |
| Stage 사이 skid/ready/abort | 해당 양쪽 Stage + H5 + 네 OOC 구현 |
| AXI 출력 폭/packer | H4 + H5 + 네 OOC 구현 |
| VDMA Profile/Face 안전 적용 | H5 + H6 VDMA/DDR/CSR 통합 |
| CDC 또는 상위 async FIFO | H6 전체, 4:1/1:4/1:1 clock 조합 |

## 7. 유지보수 주의사항

1. 합성 물리 최대 Return 수, Runtime 직렬화 Return 슬롯 수와 실제 유효 Return
   수를 하나의 변수명이나 설명으로 합치지 않는다.
2. H5 입력은 TDC-domain 비동기 결과 FIFO 뒤다. H5의 inflight count를 상위 FIFO
   점유율로 해석하지 않는다.
3. registered skid를 단순 latency로 보고 제거하지 않는다. 넓은 payload 배선과
   abort `ready` 역전파를 끊는 물리 pipeline 경계다.
4. abort 중 skid, HLS 내부 보류 출력과 RTL packer를 같은 epoch에서 비워야 한다.
5. Face 완료는 formatter가 Footer를 만들었다는 사실이 아니라 마지막 Footer
   Beat가 sink에 승인됐다는 사실을 뜻한다.
6. `PACKED17`은 Hit 저장 ABI이며 AXI4-Stream 폭이 아니다.
7. `.work`, 생성 HLS Verilog, `.log`, `.jou`, `.wdb`, `.dcp`는 Git에 넣지 않는다.
8. OOC 양수 WNS를 Parent 전체 timing PASS로 확대 해석하지 않는다.

## 8. 아직 검증하지 않는 범위

- TDC-GPX 물리 버스, IFIFO Drain과 상위 async FIFO 점유율
- AXI4-Lite CSR, Shadow/Active, COMMIT, IRQ
- Runtime Profile 변경의 Face 안전 경계와 VDMA 재설정
- 실제 VDMA DDR 주소, HSIZE/VSIZE/STRIDE와 DDR Golden Word
- PS cache 동기화 및 H-Line/Ethernet packet
- 물리 `fire_done` 승인부터 측정 시작 기준시점 (T0)
- Motor, Laser, Echo LVDS, Parent bitstream과 실제 보드

이 범위는 H6 Parent 통합과 보드 단계가 소유한다.
