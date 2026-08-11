# V3 H6 테스트벤치 운용 및 유지보수 가이드

## 1. 목적

H6 시험은 HLS Stage만 연결한 시험이 아니다. 외부 TDC-GPX 병렬 버스 모델에서
시작해 IFIFO 전체 Drain, CDC, H1~H4와 최종 AXI 출력까지 V2와 같은지 검증한다.
물리 버스 cycle latency는 달라도 되지만 DDR로 향하는 승인 Beat 내용과 순서는
완전히 같아야 한다.

H6-A는 물리 GPX Drain부터 AXI까지의 V2/V3 데이터 차분을 소유한다. H6-B1은
그 경계를 V2 CSR/Shadow/Active/COMMIT/IRQ와 결합한 실제 V3 통합 Top을 소유한다.
H6-B2는 Runtime VDMA ACK, DDR Word와 PS H-Line/Ethernet 종단을 소유한다.

## 2. 테스트 파일과 책임

| 파일 | 생성 이유 | 검증 책임 |
|---|---|---|
| `tb_tdc_gpx_stale_ready.vhd` | 등록된 `ready`가 한 Cycle 늦게 내려갈 때 경계 손실 방지 | skid 2-slot, sync FIFO 수용량, 순서·중복·손실 |
| `tb_lidar_gpx_hls_parent_data_subsystem_diff.vhd` | 실제 GPX bus/CDC/HLS/AXI 종단 비교 | 5 clock/profile, V2/V3 캡처 일치, Face ACK, fault |
| `lidar_gpx_hls_parent_data_subsystem_impl.vhd` | 최대 구성 경로가 합성 최적화로 사라지는 것을 방지 | 4 Chip 양 Edge, 7 Return, 두 clock, 모든 주요 I/O 보존 |
| `tb_tdc_gpx_lidar_ctrl_v3_h6b.vhd` | V2 제어 계층과 V3 HLS 데이터 경로를 실제 통합 Top에서 결합 | 4 Chip Reg7 Shadow/Active/Physical, COMMIT 실패 복구, IFIFO Drain, 7→3→7 Return과 Lane별 VDMA ACK 원자성, AXI/Footer stall, 0x27 CSR routing |
| `tb_lidar_v3_h6b2_ddr_golden.vhd` | V3 H4 HLS와 유지 RTL AXIS packer의 실제 DDR 주소 영상을 검사 | 32/64-bit HSIZE/VSIZE/STRIDE, Shot/Hole/Footer, SOF/TLAST, 예약 STRIDE Word 보존, HTML/V2 Golden exact compare |
| `run_v3_h6b2_ps_hline.ps1` | V3 DDR 캡처가 Zynq PS와 Viewer까지 같은 의미를 갖는지 검사 | Cortex-A9 컴파일, DMA/cache 소유권 오용 거부, Face Header/H-Line Ethernet payload byte compare |
| `tb_lidar_v3_processing_status_formatter_fault.vhd` | H4 신규 fault가 기존 V2 IRQ/진단 ABI에서 빠지는 것을 방지 | 0x1A 요약, 0x27 bitmap, GPX_DATA 단독 IRQ, CLEAR_STATUS exact compare |
| `run_v3_h6b_integrated_top_impl.ps1` | HLS 데이터 경로와 CSR/처리 계층을 함께 놓았을 때 timing 확인 | 4 Chip 최대 구성, 150/200 MHz 32-bit와 200/150 MHz 64-bit post-route |

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

H6-B1 통합 Top은 사용자가 정한 두 제품 조합만 실행한다.

| Profile | 통합 기능 시험 |
|---|---|
| `proc150_tdc200_w32` | 4 Chip 모두 Rising, 7 Return, 32-bit, Footer 13 clock stall |
| `proc200_tdc150_w64` | 4 Chip 모두 Rising, 7 Return, 64-bit, Footer 13 clock stall |

2 Rise/2 Fall 및 4 Chip 모두 Rising/Falling 가능한 topology는 H6-A 5-profile
차분 시험이 소유한다. H6-B1 OOC 구현은 기본 2 Rise/2 Fall capability로 양쪽
물리 경로가 제거되지 않는지 확인한다.

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
- Reg7 staging 값과 자동 파생 Active/물리 값이 분리되어 보임
- MTimer 상한 초과 COMMIT은 오류 `0x33`으로 거절되고 이전 Active/물리 값 유지
- Runtime Return 변경 중 VDMA ACK 전 Active Version/Active Return/AXI 출력 불변
- Rise/Fall Pending이 모두 끝난 뒤에만 COMMIT 완료
- 32/64-bit DDR의 모든 할당 Word와 예약 STRIDE 영역이 Golden과 일치
- DMA/cache 소유권 위반, Footer 손상, 불법 출력 폭을 PS 참조 코드가 거부
- Face Header 1440 B와 H-Line 38 B가 Ethernet Golden과 바이트 단위로 일치
- 통합 CSR indexed portal에서 0x1A formatter 요약과 0x27 bitmap이 정상 0으로 읽힘
- formatter fault 주입 시 0x1A/0x27 exact 값과 GPX_DATA IRQ만 설정됨

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

# 5. H6-B1 4 Chip 통합 Top 기능 회귀
powershell.exe -NoProfile -ExecutionPolicy Bypass -File `
  ./system_integration/v3/scripts/run_v3_h6b_integrated_top_diff.ps1 `
  -SkipHlsSynthesis

# 6. H6-B1 H4 formatter 진단/IRQ fault injection
powershell.exe -NoProfile -ExecutionPolicy Bypass -File `
  ./system_integration/v3/scripts/run_v3_h6b_formatter_status.ps1

# 7. H6-B1 통합 Top OOC 배치·배선
powershell.exe -NoProfile -ExecutionPolicy Bypass -File `
  ./system_integration/v3/scripts/run_v3_h6b_integrated_top_impl.ps1 `
  -SkipHlsSynthesis -ImplementationStrategy timing_explore

# 8. H6-B2 V3 HLS/AXIS의 32/64-bit DDR Word Golden
powershell.exe -NoProfile -ExecutionPolicy Bypass -File `
  ./system_integration/v3/scripts/run_v3_h6b2_ddr_golden.ps1 `
  -SkipHlsSynthesis

# 9. H6-B2 DDR부터 PS cache/H-Line/Ethernet까지 연속 검증
powershell.exe -NoProfile -ExecutionPolicy Bypass -File `
  ./system_integration/v3/scripts/run_v3_h6b2_ps_hline.ps1 `
  -SkipHlsSynthesis
```

PASS marker:

- `LIDAR_V3_SHARED_STREAM_BOUNDARY_DIFF_PASS`
- `LIDAR_V3_GPX_CELL_COLLECTOR_DIFF_PASS`
- `LIDAR_V3_GPX_FRAME_ASSEMBLER_DIFF_PASS`
- `LIDAR_V3_H6_PARENT_DATA_DIFF_PASS`
- `LIDAR_V3_H6_PARENT_DATA_IMPL_PASS`
- `LIDAR_V3_H6B_INTEGRATED_TOP_DIFF_PASS`
- `LIDAR_V3_H6B2_VDMA_PROFILE_ATOMIC_PASS`
- `LIDAR_V3_H6B_FORMATTER_STATUS_IRQ_PASS`
- `LIDAR_V3_H6B_INTEGRATED_TOP_IMPL_PASS`
- `LIDAR_V3_H6B2_DDR_GOLDEN_PASS`
- `LIDAR_V3_H6B2_PS_HLINE_ETHERNET_PASS`

## 7. 변경 영향과 필수 회귀

| 변경 대상 | 최소 필수 회귀 |
|---|---|
| skid 또는 sync FIFO | 공용 경계 + H2 + H3 + H6 차분/구현 |
| GPX bus PHY/Drain/EF 정책 | H6 5 Profile 전체 |
| Shot/STOP/Result CDC | H6 1:4, 4:1, 1:1 + CDC report |
| H1~H4 payload 또는 abort/idle | 해당 Stage + H5 + H6 |
| Face-close 또는 Footer 완료 | H3 + H4 + H5 + H6 stalled Face |
| 출력 폭/packer | H4 + H5 + H6 32/64-bit |
| CSR/Shadow/Active/COMMIT 또는 Reg7 파생 | V2 관련 단위 시험 + H6-B1 통합 Top 두 Profile |
| Runtime Return 또는 VDMA profile/ACK | H6-B2 원자성 + DDR + PS |
| HSIZE/VSIZE/STRIDE 함수 | H6-B2 DDR + PS |
| PACKED17/Shot Metadata/Face Footer ABI | H0~H4 + H6-B2 DDR/PS + HTML Golden |
| PS cache 소유권 API | H6-B2 호스트/Cortex-A9 + 실제 보드 cache 시험 |
| H1~H4 fault bit/IRQ 분류 | 해당 Stage fault 시험 + formatter 상태 시험 + H6-B1 정상 CSR routing |
| Processing/AXIS idle 정의 | H5 + H6-A stalled Face + H6-B1 COMMIT/Formatter stall |
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
6. H6-B1의 assertion 문자열에 남은 `V2-K05/K06/K12/K13/K14`는 Golden 시험
   계보 표시다. V3 PASS 판정은 `LIDAR_V3_H6B_*` marker로 한다.
7. 현재 200/150 MHz 64-bit 통합 OOC의 Processing WNS는 `+0.060 ns`다. 양수지만
   작으므로 Parent 배치 결과 없이 최종 timing Sign-off로 판정하지 않는다.

재현 증거:

- H6-B1 기능: `.work/v3_h6b_integrated_top_diff/260811_h6b_integrated_precommit2`
- formatter 상태: `.work/v3_h6b_formatter_status/260811_h6b_formatter_status_precommit`
- 통합 구현: `.work/v3_h6b_integrated_top_impl/260811_h6b_impl_first`,
  `.work/v3_h6b_integrated_top_impl/260811_h6b_impl_second`
- Runtime VDMA 원자성: `.work/v3_h6b_integrated_top_diff/260811_h6b2_runtime_vdma_final`
- DDR Golden: `.work/v3_h6b2_ddr_golden/260811_h6b2_end_to_end_final_ddr`
- PS H-Line/Ethernet: `.work/v3_h6b2_ps_hline/260811_h6b2_end_to_end_final`

## 9. 아직 소유하지 않는 시험

- 실제 AXI VDMA Register 쓰기와 물리 Frame buffer handoff
- 실제 DDR DMA dump와 Golden Word 비교
- FreeRTOS/PetaLinux cache invalidate/flush API 실호출과 물리 Ethernet 전송
- Parent 전체 bitstream, IOB, TDC-GPX/레이저/모터/설정된 Echo LVDS 보드 시험
- 물리 IFIFO Drain 도중 Processing abort와 TDC-GPX soft reset을 함께 발생시키는
  통합 command CDC 복구 시험
