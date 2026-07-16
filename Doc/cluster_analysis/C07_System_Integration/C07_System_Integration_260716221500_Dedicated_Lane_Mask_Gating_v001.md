# C07 Dedicated Lane-Mask Gating (P1: wrong-slope builder churn 제거)

| 항목 | 내용 |
|---|---|
| 문서 종류 | P1 결함 수정 + 검증 결과 |
| 문서 버전 | v001 |
| 생성 시간 | 2026-07-16 22:15:00 KST |
| 선행 문서 | `C07_System_Integration_260716213500_Shot_Boundary_FIFO_Flush_Guard_v001.md` §6 부수 관찰 P1 |
| 절대 기준 문서 | `Doc/TDC-GPX-Datasheet.pdf` |
| session archive | `sim_results/vivado_xsim/sessions/260716221500_lane_mask_p1_fix/` |
| top 통합 archive | `sim_results/vivado_xsim/sessions/260716223000_c07_v002_4chip_target/` |

## 1. 결론

`DEDICATED_2X2` 구성에서 slope demux 의 drain_done broadcast 와 무게이트
shot_start 때문에 **반대-slope cell_builder 8개 중 4개가 매 shot 마다
소비자 없는 blank slice 를 생성**하던 구조를 lane mask 게이팅으로 제거했다.

```text
수정 전 메커니즘 (Scenario L 로 확인): mask 1111 에서 8개 lane 전부
  shot 당 slice 방출 -- dedicated 구성이면 4개는 blank churn
수정 후 (Scenario D): wrong-slope lane 0 beats, 정상 lane shot 당 1 slice,
  3 shots 연속 shot_dropped 없음, sticky 청정
레거시 회귀: tb_tdc_gpx_cell_pipe / tb_tdc_gpx_cell_pipe_c03_fix PASS
```

효과:
- face_assembler `shot_flush_drop` sticky / mask 가 dedicated 정상 동작에서
  더 이상 매 shot 오염되지 않음 → 본래의 "shot 경계 정렬 상실" 진단 기능 회복
- 소비자 없는 slice 생성/flush 순환 (전력·토글·버퍼 점유) 제거

## 2. 결함 메커니즘 (수정 전)

1. `tdc_gpx_cell_pipe` 의 slope demux 는 drain_done 제어 beat(tuser[7])를
   rise/fall 양쪽 builder 에 broadcast 하고, `i_shot_start_per_chip(i)` 를
   양쪽 builder 에 무게이트로 연결했다.
2. dedicated 구성(chip0/1=rise, chip2/3=fall)에서 chip2/3 의 rise builder 와
   chip0/1 의 fall builder 는 hit 이 없어도 shot_start + drain_done 을 받아
   stops_per_chip 개의 blank cell slice 를 방출했다.
3. 이 slice 는 face_assembler 의 lane-mask 로 skip 되는 chip 의 입력 FIFO 에
   쌓였다가 다음 shot_start flush 로 폐기 → `shot_flush_drop` sticky 와
   per-chip mask 가 **정상 동작인데도 매 shot 세트**되어 진단 가치 상실.
   (데이터 오염은 없음 -- per-chip FIFO 격리 + mask skip)

## 3. 수정 내용

| 파일 | 변경 |
|---|---|
| `tdc_gpx_cell_pipe.vhd` | `i_rise_chip_mask` / `i_fall_chip_mask` 입력 추가 (default all-ones = 레거시 호환). shot_start 를 (chip, slope) 로 AND 게이트. demux 로드: drain_done 은 enabled slope 에만 전달(스킵은 무해), masked slope 로 향한 **hit** beat 는 소비-드롭 + `o_masked_slope_drop_rise/fall` sticky (물리 에지 misconfig 가시성). reset/global abort 에서 sticky clear |
| `tdc_gpx_top.vhd` | `s_face_rise_mask` / `s_face_fall_mask` (C08 slope topology 산출과 동일 소스) 를 cell_pipe 에 배선. sticky 는 CSR 슬롯 미할당으로 현재 open (STAT map 개정 시 status_agg 연결 예정) |

설계 성질:
- `SHARED_DUAL_EDGE` / 레거시 인스턴스(mask 미배선): all-ones default 로
  기존 동작과 동일 분기 → 무회귀
- masked slope 의 holding register 는 로드되지 않으므로 demux ready 식은
  변경 불필요 (항상 free 취급)
- drain_done 의 "both ready" 요구는 enabled slope 에만 실효

## 4. 검증

### 4.1 신규 TB: `tb_tdc_gpx_cell_pipe_lane_mask.vhd`

32-bit, 8 stops, max_hits=1 (slice = 16 beats), 4 chips.

| Scenario | 내용 | 결과 |
|---|---|---|
| L (legacy 1111) | 8개 lane 전부 slice/shot 방출 확인 = 수정 전 churn 메커니즘 증거 + default 호환성 (게이팅 코드가 all-ones 에서 구코드와 동일 분기) | PASS |
| D (dedicated 0011/1100, 3 shots) | 정상 lane: shot 당 1 tlast·16 beats ×3. wrong-slope lane: **0 beats**. shot_dropped 무발생, sticky 청정 | PASS |
| M (masked-slope hit) | chip0 fall hit → 소비(무정지) + `masked_drop_fall(0)` sticky, chip0 rise slice 정상 완료, fall lane 0 beats | PASS |

log: session archive `260716221500_lane_mask_p1_fix/logs/simulate/xsim_lane_mask.log`

### 4.2 레거시 회귀

| TB | 결과 |
|---|---|
| `tb_tdc_gpx_cell_pipe` (기본 스모크) | PASS |
| `tb_tdc_gpx_cell_pipe_c03_fix` (Hit[16]/skid/per-slope abort) | PASS |

### 4.3 Top 통합 회귀

`scripts/run_c07_v002_4chip_target.ps1 -Stamp 260716223000` → **PASS**

- c06 v002 선행 suite (face_seq / status_agg / top_int 32·64·128-bit,
  bounded output backpressure 포함) 전부 PASS
- dedicated 4-chip target 64/128-bit:
  "output streams emitted beats/tlast as expected - PASS",
  "Hit[16] final metadata preservation - PASS"
- 이 회귀는 top 의 cell_pipe lane-mask 배선과 이번 세션의 P0
  (shot-boundary flush guard) 수정을 top 레벨에서 함께 통과시킨 결과다.

주의: 첫 시도(stamp 260716222000)는 `run_c06_v002_regression.ps1` 의
컴파일 목록이 C08 에서 추가된 `tdc_gpx_line_packer.vhd` 를 누락해
output_stage 분석 단계에서 실패했다 (P1 과 무관한 스크립트 유지보수
누락). 목록에 line_packer 를 추가해 해결 -- 신규 RTL 파일이 생기면
스크립트 컴파일 목록도 함께 갱신해야 한다.

## 5. 남은 항목

1. `o_masked_slope_drop_rise/fall` 의 CSR 노출: STAT map 빈 슬롯 확보 시
   status_agg 경유로 연결 (현재 top 에서 open, TB 관측만 가능)
2. face_assembler `shot_flush_drop` 는 의미 회복: 이후 이 sticky 가 서면
   진짜 boundary drift (P0 계열) 신호로 해석 가능
3. C08 HTML 시뮬레이터의 dedicated 모델에 wrong-slope idle 반영 여부 확인

## 6. 실행 기록

| 실행 | 명령 | 결과 |
|---|---|---|
| lane_mask TB | `xelab/xsim tb_lane_mask_snap` | 3/3 PASS |
| cell_pipe 레거시 | `cp_legacy_snap`, `cp_c03fix_snap` | PASS |
| top 통합 (1차) | `run_c07_v002_4chip_target.ps1 -Stamp 260716222000` | FAIL (스크립트 컴파일 목록의 line_packer 누락, §4.3 주의 참조) |
| top 통합 (2차) | `run_c07_v002_4chip_target.ps1 -Stamp 260716223000` | **PASS** (c06 suite + dedicated 64/128-bit) |
