# V3 H2 GPX Cell Collector 테스트벤치 가이드

## 1. 문서 목적

이 문서는 V3 H2 Hit-to-Cell 변환에 사용한 테스트의 역할과 유지보수 규칙을
정의한다. 단순히 `PASS` 문자열을 확인하는 데 그치지 않고, 어떤 계약이 어느
테스트에서 검증되며 어떤 항목은 다음 통합 단계에 남아 있는지를 구분한다.

H2에서 말하는 Cell은 다음 단위다.

```text
Cell = Shot 하나 x TDC-GPX Chip 하나 x STOP 하나 x slope 하나
```

## 2. 권장 수행 순서

| 순서 | 실행기 | 검증 목적 | 실패 시 먼저 볼 경계 |
|---:|---|---|---|
| 1 | `run_v3_hls_cell_collector.ps1 -Step csim` | HLS C 모델의 알고리즘 계약 | HLS 함수와 고정 Bit Map |
| 2 | `run_v3_hls_cell_collector.ps1 -Step csynth` | 합성 가능성과 HLS 추정 성능 | Loop II, 메모리 추론, 인터페이스 |
| 3 | `run_v3_hls_cell_collector.ps1 -Step cosim` | 생성 RTL이 C 모델과 동일한지 확인 | HLS 생성 RTL과 AXI4-Stream Handshake |
| 4 | `run_v3_gpx_cell_collector_diff.ps1` | V2 RTL과 V3 HLS의 Clock 단위 차등 비교 | RTL Adapter, 순서, Backpressure, Abort |
| 5 | `run_v3_gpx_cell_collector_impl.ps1 -SkipHlsSynthesis` | xc7z020의 150/200 MHz 배치·배선 | 실제 WNS, Latch, DRC, 배선 지연 |

코드 변경 뒤에는 1~5 순서를 모두 수행한다. 문서나 주석만 바뀌었을 때는
소스 변경이 없음을 확인한 뒤 기존 결과를 재사용할 수 있다.

## 3. HLS 독립 C 테스트

### 대상

- 테스트: `hls/gpx_cell_collector/tb/tb_gpx_cell_collector_hls.cpp`
- 구현: `hls/gpx_cell_collector/src/gpx_cell_collector_hls.cpp`
- H0 전체 진입점: `hls/common/include/lidar_v3_hls_contract.hpp`
- H2 Bit 소유 계약: `hls/common/include/lidar_v3_h2_cell_contract.hpp`

### 검증 항목

- 물리 Return 7개를 입력하면서 Runtime 직렬화(전시) Return 슬롯 수 1~7 Sweep
- Runtime 전시 범위를 넘는 Return이 의도적으로 필터되며 fault가 아님
- 8번째 물리 Return만 `return_overflow`로 분류
- 17-bit Hit의 최상위 Bit `Hit[16]` 보존
- START number가 0이 아닐 때 오류 분류
- IFIFO1 완료, 전체 Drain 완료, Timeout/Error-fill Cell 생성 순서
- Reset Epoch 변경 뒤 이전 Shot 소유권 제거와 같은 주소 재사용
- 다음 네 topology Profile

| Profile | Chip/STOP | Rise mask | Fall mask | 핵심 목적 |
|---|---|---|---|---|
| `dedicated` | 4 / 8 | `0011` | `1100` | Rise 2 Chip과 Fall 2 Chip 분리 |
| `one_chip_dual` | 1 / 8 | `0001` | `0001` | 한 Chip에서 Rise/Fall 양 Edge |
| `reduced` | 3 / 6 | `0011` | `0100` | 축소 topology와 오류 시나리오 |
| `all_dual` | 4 / 8 | `1111` | `1111` | 네 Chip 전체 Rise/Fall 양 Edge |

### 이 테스트가 보장하지 않는 것

HLS C 모델은 Clock, Backpressure, `ap_ctrl_hs`, RTL Adapter의 Abort Flush를
검증하지 않는다. 이 항목은 C/RTL co-simulation과 V2/HLS 차등 테스트가 맡는다.

## 4. HLS C/RTL Co-simulation

`run_v3_hls_cell_collector.ps1 -Step cosim`은 네 Profile 각각에 대해 HLS가
생성한 Verilog RTL과 C 테스트를 함께 실행한다. LUTRAM의 동기 읽기 지연,
Pipeline 스케줄 및 AXI4-Stream Handshake가 C 모델의 결과와 같은지 확인한다.

유지보수 시 네 Profile 중 하나만 통과했다고 전체 PASS로 판정하면 안 된다.
실행기는 각 Profile 보고서에서 Verilog `Pass` 행을 확인한 뒤에만 최종 PASS
표시를 출력한다.

## 5. V2 RTL/HLS 차등 테스트

### 대상

- 테스트: `tb/tb_lidar_gpx_cell_collector_hls_diff.vhd`
- 기준 구현: V2 `lidar_gpx_cell_collector`
- 비교 구현: `rtl/bridges/lidar_gpx_cell_collector_hls_adapter.vhd`
- HLS 생성 RTL: 실행기가 `.work/`에서 생성한 `gpx_cell_collector_hls`

### 비교 방법

```text
                    +--> V2 RTL Collector ----+
동일 Hit/Event -----|                          +--> Cell 전체 Record 비교
                    +--> V3 HLS + Adapter -----+
```

입력은 두 구현이 모두 ready인 Clock edge에만 동시에 승인한다. 출력은 각 구현의
valid를 독립적으로 보관한 뒤 둘 다 준비되면 비교한다. 따라서 Pipeline latency가
달라도 실패하지 않고, 다음 차이만 실패한다.

- Cell 종류, Chip, STOP, slope 및 순서
- Return 수와 Hit0~Hit6의 17-bit 값
- Shot context와 Chip별 Shot sequence
- Timeout/Error-fill/fault 정보
- Fault pulse와 sticky 상태
- Backpressure 중 payload 변경
- Abort 뒤 이전 Shot Cell 노출 또는 같은 주소의 새 Shot 오염

네 topology와 150/200 MHz를 조합한 총 8개 실행이 모두 PASS해야 한다.

### 유지보수 주의점

- 한 구현이 먼저 ready라고 입력을 제공하면 서로 다른 Event를 비교하게 된다.
- 한 구현의 출력만 먼저 소비하면 Cell 순서 비교가 무효가 된다.
- Abort 시험은 출력이 Backpressure로 보류된 상태에서 실행해야 오래된 결과
  Flush 경로를 실제로 자극한다.
- 새 fault를 추가하면 pulse와 sticky의 비교 및 clear 시나리오를 함께 추가한다.

## 6. OOC 구현 Harness

### 대상

- Harness: `tb/lidar_gpx_cell_collector_hls_impl.vhd`
- 실행기: `scripts/run_v3_gpx_cell_collector_impl.ps1`
- Device: `xc7z020clg484-2`

Harness는 네 Chip 모두 Rise/Fall 양 Edge가 가능한 최대 topology를 유지한다.
150 MHz와 200 MHz 각각을 실제 합성·배치·배선하고 다음을 검사한다.

- WNS가 0 ns 이상
- Latch 0
- 차단 등급 DRC 0
- 내부 unconstrained endpoint 0

OOC 외부 포트에는 Parent I/O delay가 없으므로, 이 결과만으로 Parent Timing 또는
보드 Sign-off를 선언하면 안 된다. 상위 FIFO 점유율, CDC, Shot 처리 여유 및 실제
I/O 제약은 H5/H6에서 닫는다.

## 7. PASS 표시와 결과 보존 규칙

실행기는 아래 최종 표시를 출력해야 성공이다.

```text
LIDAR_V3_HLS_GPX_CELL_COLLECTOR_ALL_PASS
LIDAR_V3_GPX_CELL_COLLECTOR_DIFF_PASS
LIDAR_V3_GPX_CELL_COLLECTOR_IMPL_PASS
```

생성 프로젝트, 로그, Waveform과 HLS Tool Home은 저장소 루트 `.work/` 아래에만
둔다. 소스 폴더의 `.Xil`, `solution*`, `.log`, `.jou`, `.wdb`는 Git에 넣지 않는다.
체크포인트 문서에는 재현 가능한 명령과 핵심 수치만 기록한다.

## 8. H2 이후에 남는 검증

H2 PASS로 다음 항목이 자동 보장되지는 않는다.

- H3 Rise/Fall Cell 정렬과 Shot/Face 구조화
- H4 PACKED17, Shot Line, Face Footer Word Golden 비교
- H5 32/64-bit AXI4-Stream, FIFO 최대 점유율, CDC와 전체 Shot 처리 여유
- H6 Parent 4-Chip 구현, I/O 제약, Bitstream 및 보드 검증

따라서 H2 테스트를 삭제하거나 상위 통합 테스트로 대체하지 않는다. H2는
Hit-to-Cell 경계의 빠른 회귀 기준으로 계속 유지한다.
