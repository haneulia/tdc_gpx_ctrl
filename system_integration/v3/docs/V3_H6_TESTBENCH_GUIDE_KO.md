# V3 H6 테스트벤치 운용 및 유지보수 가이드

## 1. 목적

H6 시험은 HLS Stage만 연결한 시험이 아니다. 외부 TDC-GPX 병렬 버스 모델에서
시작해 IFIFO 전체 Drain, CDC, H1~H4와 최종 AXI 출력까지 V2와 같은지 검증한다.
물리 버스 cycle latency는 달라도 되지만 DDR로 향하는 승인 Beat 내용과 순서는
완전히 같아야 한다.

## 2. 테스트 파일과 책임

| 파일 | 생성 이유 | 검증 책임 |
|---|---|---|
| `tb_tdc_gpx_stale_ready.vhd` | 등록된 `ready`가 한 Cycle 늦게 내려갈 때 경계 손실 방지 | skid 2-slot, sync FIFO 수용량, 순서·중복·손실 |
| `tb_lidar_gpx_hls_parent_data_subsystem_diff.vhd` | 실제 GPX bus/CDC/HLS/AXI 종단 비교 | 5 clock/profile, V2/V3 캡처 일치, Face ACK, fault |
| `lidar_gpx_hls_parent_data_subsystem_impl.vhd` | 최대 구성 경로가 합성 최적화로 사라지는 것을 방지 | 4 Chip 양 Edge, 7 Return, 두 clock, 모든 주요 I/O 보존 |

## 3. Parent 차분 시험의 물리 모델

1. 4개 Chip 각각의 IFIFO1/IFIFO2에 28-bit I-Mode Word를 적재한다.
2. `EF1/EF2`가 empty가 될 때까지 V2 유지 RTL이 모두 읽는다.
3. 각 Word의 하위 17 bit는 거리 Hit이며 Channel, slope와 IFIFO 출처는 상위
   I-Mode field로 표현한다.
4. Runtime 직렬화 Return 슬롯이 1이어도 물리 7 Return을 모두 Drain한다.
5. Rise/Fall AXI sink는 서로 다른 주기로 `TREADY`를 낮춘다.
6. 첫 Face는 계획 4 Shot 중 하나만 존재하고, 다음 Face는 Shot이 전혀 없다.
   H3/H4가 Hole Line과 Face Footer를 만들어 VDMA Line geometry를 보존한다.

## 4. 5개 회귀 Profile

| Profile | 검증 의도 |
|---|---|
| `p50_t200_w32_dedicated_r1` | Processing:TDC 1:4 극단 CDC, 최소 전시 Return |
| `p200_t50_w64_all_dual_r7` | Processing:TDC 4:1 극단 CDC, 최대 양 Edge/Return |
| `p150_t150_w32_all_dual_r7` | 같은 물리 clock net을 쓰는 Sync mode |
| `p150_t200_w32_dedicated_r1` | 제품 150/200 MHz, 32-bit 최소 전시 Profile |
| `p200_t150_w64_all_dual_r7` | 역 제품 200/150 MHz, 64-bit 최대 부하 Profile |

스크립트는 각 Profile의 V2와 V3를 독립 실행한다. 내부 latency를 맞추지 않고
최종 Rise/Fall 캡처 파일을 exact compare한다.

## 5. 주요 assertion

- 4개 Chip 모두 초기화되고 Run 상태에 들어감
- 한 Shot이 4개 Chip에 동일하게 broadcast됨
- EF 전체 Drain 뒤 Shot identity와 측정 시작 기준시점 (T0)이 보존됨
- backpressure 중 `TDATA/TKEEP/TSTRB/TUSER/TLAST`가 변하지 않음
- 마지막 Footer Beat 승인 전 Face 종료 승인이 발생하지 않음
- 첫 Face의 부분 Hole과 다음 Face의 전체 Hole Line 수가 정확함
- Rise/Fall Beat, Line, SOF와 Frame 완료 수가 정확함
- Shot/STOP CDC drop, context mismatch, H1/H2/H4 fault가 0임
- 의도적으로 만든 Shot gap만 H3 `column_gap` sticky에 기록됨

## 6. 실행 순서

```powershell
# 1. 공용 skid/sync FIFO의 stale-ready 단위 시험
powershell.exe -NoProfile -ExecutionPolicy Bypass -File `
  ./system_integration/v3/scripts/run_v3_shared_stream_boundary_diff.ps1

# 2. 공용 경계를 사용하는 H2/H3 회귀
powershell.exe -NoProfile -ExecutionPolicy Bypass -File `
  ./system_integration/v3/scripts/run_v3_gpx_cell_collector_diff.ps1 `
  -SkipHlsSynthesis
powershell.exe -NoProfile -ExecutionPolicy Bypass -File `
  ./system_integration/v3/scripts/run_v3_gpx_frame_assembler_diff.ps1 `
  -SkipHlsSynthesis

# 3. H6 Parent 데이터 종단 V2 Golden 비교
powershell.exe -NoProfile -ExecutionPolicy Bypass -File `
  ./system_integration/v3/scripts/run_v3_h6_parent_data_diff.ps1 `
  -SkipHlsSynthesis

# 4. 제품 두 교차 clock 조합의 OOC 배치·배선
powershell.exe -NoProfile -ExecutionPolicy Bypass -File `
  ./system_integration/v3/scripts/run_v3_h6_parent_data_impl.ps1 `
  -SkipHlsSynthesis -ImplementationStrategy timing_explore
```

PASS marker:

- `LIDAR_V3_SHARED_STREAM_BOUNDARY_DIFF_PASS`
- `LIDAR_V3_GPX_CELL_COLLECTOR_DIFF_PASS`
- `LIDAR_V3_GPX_FRAME_ASSEMBLER_DIFF_PASS`
- `LIDAR_V3_H6_PARENT_DATA_DIFF_PASS`
- `LIDAR_V3_H6_PARENT_DATA_IMPL_PASS`

## 7. 변경 영향과 필수 회귀

| 변경 대상 | 최소 필수 회귀 |
|---|---|
| skid 또는 sync FIFO | 공용 경계 + H2 + H3 + H6 차분/구현 |
| GPX bus PHY/Drain/EF 정책 | H6 5 Profile 전체 |
| Shot/STOP/Result CDC | H6 1:4, 4:1, 1:1 + CDC report |
| H1~H4 payload 또는 abort/idle | 해당 Stage + H5 + H6 |
| Face-close 또는 Footer 완료 | H3 + H4 + H5 + H6 stalled Face |
| 출력 폭/packer | H4 + H5 + H6 32/64-bit |
| 구현 directive/flatten | 두 제품 clock 조합의 새 post-route 결과 |

## 8. 유지보수 주의사항

1. 물리 최대 Return 수와 Runtime 직렬화(전시) Return 슬롯 수를 같은 변수로
   취급하지 않는다.
2. H6의 `o_shot_done`은 Cell 정렬 완료이며 Face 출력 완료가 아니다. Face 종료
   승인은 마지막 Footer AXI handshake를 기다린다.
3. XPM CDC warning을 숫자만으로 일괄 무시하지 않는다. 현재 12개 CDC-6과 1개
   CDC-15가 모두 동일한 XPM 내부 구조인지 instance 경로를 확인한다.
4. `timing_explore` 없이 200 MHz WNS가 양수라는 보장은 없다.
5. `.work` 결과는 재생성 가능한 증거이며 Git에 넣지 않는다. 문서에는 검증 Stamp와
   판정 수치만 남긴다.

## 9. 아직 소유하지 않는 시험

- CSR Shadow/Active와 COMMIT 오류 주입
- Runtime VDMA HSIZE/VSIZE/STRIDE 변경과 Frame buffer handoff
- 실제 DDR dump와 Golden Word 비교
- PS DMA cache 동기화와 Ethernet packet 비교
- Parent 전체 bitstream, IOB, TDC-GPX/레이저/모터/설정된 Echo LVDS 보드 시험
- 물리 IFIFO Drain 도중 Processing abort와 TDC-GPX soft reset을 함께 발생시키는
  통합 command CDC 복구 시험
