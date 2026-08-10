# TDC-GPX LiDAR Controller V3 HLS Migration

V3는 V2의 검증된 운용 계약을 유지하면서, 측정 이후의 데이터 처리 경로만
Vitis HLS 2025.2로 단계적으로 교체한다. V2 RTL은 비교 기준이며 직접 수정하지
않는다.

## 구현 경계

RTL에 유지하는 항목:

- AXI4-Lite CSR, Shadow/Active, COMMIT, IRQ
- Motor decoder, Face tracker, Shot scheduler, Laser executor
- 물리 `fire_done`에서 측정 시작 기준시점 (T0)을 만드는 저지연 경로
- Echo LVDS-to-STOP 경로와 TDC-GPX bus PHY/IFIFO Drain
- 모든 CDC, XPM FIFO, Reset 동기화
- VDMA profile 안전 적용과 최종 32/64-bit AXI4-Stream packer

HLS로 단계 전환하는 항목:

- GPX I-Mode Raw28 해석과 Hit17 생성
- Return 1~7 Cell 수집과 Runtime 전시 Return 필터
- Rise/Fall Cell 정렬, PACKED17, Shot/Hole/Footer 32-bit Word 생성

## 디렉터리

| 경로 | 역할 |
|---|---|
| `docs/` | 마이그레이션 계약과 체크포인트 결과 |
| `hls/common/include/` | HLS/RTL 경계의 고정 Bit Map |
| `hls/gpx_hit_decoder/` | 첫 변환 대상 Raw28-to-Hit17 컴포넌트 |
| `hls/gpx_cell_collector/` | Hit17-to-Cell 수집·필터·오류 분류 컴포넌트 |
| `rtl/bridges/` | HLS와 V2 record 경계를 연결하는 Adapter |
| `tb/` | V2/HLS 차등 및 구현 Timing Harness |
| `scripts/` | Vitis HLS 재현 실행기 |

생성 프로젝트와 로그는 저장소 루트의 `.work/` 아래에만 만들며 Git에 넣지
않는다. HLS 소스, 실행 Tcl, 테스트와 요약 결과만 형상관리한다.

## 고정 도구와 대상

- Vitis HLS 2025.2
- Vivado 2025.2 계열
- Device: `xc7z020clg484-2`
- HLS 처리 목표: 200 MHz
- 제품 AXI4-Stream 폭: 합성 시 32 또는 64 bit
- 데이터 ABI: PACKED17

## 첫 실행

```powershell
./system_integration/v3/scripts/run_v3_hls_hit_decoder.ps1 -Step all
```

`all`은 C simulation, C synthesis, RTL co-simulation 순서로 실행한다.

## 현재 체크포인트

| 단계 | 상태 | 판정 |
|---|---|---|
| H0 실행 환경 | 완료 | Vitis HLS/Vivado 2025.2.1 재현 확인 |
| H1 Raw28-to-Hit17 | 완료 | CSim, C/RTL co-sim, V2 차동 회귀, 150/200 MHz 배치·배선 PASS |
| H2 Hit-to-Cell | 완료 | CSim, C/RTL co-sim, V2 차등, Abort, 150/200 MHz 배치·배선 PASS |
| H3 Cell-to-Frame | 다음 단계 | Rise/Fall Cell 정렬과 Shot/Face 구조화 |

H1의 계약, 시험 행렬과 수치는
[`docs/V3_H1_GPX_HIT_DECODER_SIGNOFF_KO.md`](docs/V3_H1_GPX_HIT_DECODER_SIGNOFF_KO.md)에
기록한다. H2의 계약과 검증 결과는
[`docs/V3_H2_GPX_CELL_COLLECTOR_SIGNOFF_KO.md`](docs/V3_H2_GPX_CELL_COLLECTOR_SIGNOFF_KO.md)에
기록하고, 테스트별 목적·수행 순서·유지보수 규칙은
[`docs/V3_H2_TESTBENCH_GUIDE_KO.md`](docs/V3_H2_TESTBENCH_GUIDE_KO.md)에
분리해 둔다. 각 단계 PASS는 해당 경계만 닫았다는 뜻이며 V3 통합 IP 또는
Parent 프로젝트 전체 Sign-off를 뜻하지 않는다.

## H1 재현 명령

```powershell
# CSim + CSynth + 네 가지 topology C/RTL co-sim
./system_integration/v3/scripts/run_v3_hls_hit_decoder.ps1 -Step all

# V2 RTL과 V3 HLS 혼합 시뮬레이션: 네 topology × 150/200 MHz
./system_integration/v3/scripts/run_v3_gpx_hit_decoder_diff.ps1

# xc7z020clg484-2 실제 OOC 합성·배치·배선: 150/200 MHz
./system_integration/v3/scripts/run_v3_gpx_hit_decoder_impl.ps1 -SkipHlsSynthesis
```

## H2 재현 명령

```powershell
# CSim + CSynth + 네 가지 topology C/RTL co-sim
./system_integration/v3/scripts/run_v3_hls_cell_collector.ps1 -Step all

# V2 RTL과 V3 HLS 혼합 시뮬레이션: 네 topology x 150/200 MHz
./system_integration/v3/scripts/run_v3_gpx_cell_collector_diff.ps1

# xc7z020clg484-2 실제 OOC 합성·배치·배선: 150/200 MHz
./system_integration/v3/scripts/run_v3_gpx_cell_collector_impl.ps1 -SkipHlsSynthesis
```
