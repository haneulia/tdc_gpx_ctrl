# C07 잔여 항목 Closure + CSR STAT 경로 잠재 결함 2건 수정

> 2026-07-17 후속: sticky clear 수명, scoped CSR remap, packed STAT3/4
> geometry는 `C07_System_Integration_260717_CSR_Geometry_Status_Hardening_v002.md`
> 기준으로 대체된다.

| 항목 | 내용 |
|---|---|
| 문서 종류 | 잔여 작업 closure + 잠재 결함 수정 결과 |
| 문서 버전 | v001 |
| 생성 시간 | 2026-07-17 01:30:00 KST |
| 선행 문서 | `C07_System_Integration_260716213500_Shot_Boundary_FIFO_Flush_Guard_v001.md`, `C07_System_Integration_260716221500_Dedicated_Lane_Mask_Gating_v001.md` |
| 절대 기준 문서 | `Doc/TDC-GPX-Datasheet.pdf` |
| session archives | `260716231500_p0_shot_stall_top_int`, `260717013000_csr_stat_live_read_fix`, `260717015000_lane_mask_sticky_recheck` (+중간 반복: `260716234500_masked_slope_stat7`, `260717001000_masked_slope_stat7_r2`, `260717003000_status_record_fix_stat7`) |

## 1. 결론

세 개 잔여 항목을 닫는 과정에서 **CSR STAT 경로의 잠재 결함 2건**이 드러나
함께 수정했다. 이 2건은 chain 무결성 리뷰 CHAIN-P0-03이 경고한
"false positive PASS marker"의 실증 사례다: **pipeline CSR STAT5/6/7
읽기는 블록 생성 이래 한 번도 실제 값을 반환한 적이 없었고**, 모든
exact-zero 비교는 공허하게 통과해 왔다.

```text
항목 1 (CHAIN-P0-01 통합 closure): top_int 샷 경계 stall 64/32-bit PASS
항목 2 (P2 geometry 정리): top이 face_seq legacy geometry 소비 중단
항목 3 (masked_slope_drop CSR 노출): STAT7[15] end-to-end PASS
잠재 결함 LB-1 (record 'U' source), LB-2 (STAT 주소맵 불일치): 수정 + 검증
```

## 2. 항목 1 — 샷 경계 stall 통합 closure

`tb_tdc_gpx_top_int`에 `G_BP_SHOT_STALL_CLKS` 모드 추가: shot 1의 첫 출력
beat 시도부터 **다음 shot_start 이후 N clk까지** 양 lane tready를 hold.
`mon_shot_stall_overlap`이 stall 중 shot_start 관측을 보증(전제 마커).

- 실행: `scripts/run_p0_shot_stall_top_int.ps1 -Stamp 260716231500`
- 결과: 64/32-bit `shot-boundary stall preserved beats/tlast - PASS`
- P0 flush guard(260716213500 문서)의 통합 레벨 증거 확보.

## 3. 항목 2 — legacy geometry 정리 (P2)

- `tdc_gpx_top`: face_seq의 `o_rows_per_face`/`o_hsize_bytes`를 open 처리,
  수신 신호 삭제. VDMA 계약은 per-lane `o_vdma_hsize_bytes_rise/fall` +
  `o_vdma_vsize_lines`가 유일한 소스.
- `tdc_gpx_face_seq`: 두 출력이 "full-mask 참조값, TB 검증용"임을 주석으로
  계약화 (tb_mask_sweep이 rows_per_face를 적극 검증하므로 포트는 유지).

## 4. 항목 3 — masked_slope_drop CSR 노출

pipeline CSR IP는 8 CTL + 8 STAT 고정(추가 STAT는 IP 재생성 필요)이므로
STAT7의 reserved bit 15에 OR-축약 지표를 배치했다.

| 변경 | 내용 |
|---|---|
| `tdc_gpx_pkg.vhd` | `t_tdc_status.masked_slope_drop_any` 추가 |
| `tdc_gpx_top.vhd` | cell_pipe sticky mask 수신 + OR-축약 배선 |
| `tdc_gpx_csr_pipeline.vhd` | `STAT7[15] = masked_slope_drop_any` |
| `tdc_gpx_cell_pipe.vhd` | sticky는 abort/cmd_stop에서 생존하고 `CTL2[1] err_soft_clear` 또는 hard reset에서 clear |
| `tb_tdc_gpx_top_int.vhd` | `G_EXPECT_MASKED_SLOPE_DROP` + 종단 STAT7 exact-compare(0x00008000) |
| `tb_tdc_gpx_top_int_masked_slope_stat.vhd` | wrong-slope 구성 wrapper: chip model 전 chip rising × DUT DEDICATED_2X2 → chip2/3 rise hit가 masked drop |

검증: wrapper에서 face 완결성(beats/tlast) 유지 + STAT7 = 0x00008000
exact PASS (`masked-slope drop surfaced in STAT7[15] - PASS`).

## 5. 잠재 결함 LB-1 — t_tdc_status record out 포트의 'U' source

**증상**: wrapper의 STAT7 읽기가 0. settled 프로브로
`s_status.masked_slope_drop_any='U'`, `quarantine_escape_mask="UUUU"`,
`s_stat7_src=UUUUUUUU` 확인 — **top이 구동하는 모든 status 필드가 'U'**.

**원인**: `tdc_gpx_status_agg`가 `o_status : out t_tdc_status` 레코드
포트에서 4개 필드만 할당. VHDL에서 out 포트는 actual의 **모든 요소에
대한 source**이므로, 내부에서 미할당된 요소는 초기값 'U'를 기여한다.
top의 드라이버('0'/'1')와 해소되면 'U'가 되고, Verilog XPM CDC가 이를
0으로 뭉개 CSR에서는 항상 0으로 보였다.

**수정**: status_agg가 소유 필드 4개(busy, pipeline_overrun,
rise/fall_overrun)를 **스칼라 출력**으로 내보내고 top이 record를 단독
조립 — **필드당 source 정확히 1개**. 전 RTL의 record out 포트 감사 결과
분할-소유 안티패턴은 t_tdc_status가 유일 (o_cfg 등은 전체-레코드 단일
할당).

## 6. 잠재 결함 LB-2 — pipeline CSR STAT 주소맵 불일치

**증상**: LB-1 수정 후에도 `s_stat7_out=0x8000`인데 AXI `rdata=0`.

**원인**: IP가 `num_ctl_regs=8, num_stat_regs=8`로 생성되어 STAT0..7이
**word 8..15 (byte 0x20..0x3C)** 에 위치. 공표된 레지스터 맵(주석/TB/SW)은
**0x40..0x5C** — IP 디코드 범위 밖이라 모든 pipeline STAT 읽기가 0을
반환했다. (chip CSR은 32/32 생성이라 STAT0=0x80 계약과 일치 — 정상.)

**수정**: 공표된 0x40 계약을 유지하고 `tdc_gpx_csr_pipeline` 내부에서
`0x40..0x5F` 읽기만 IP 네이티브 윈도우로 리매핑한다. 네이티브
`0x20..0x3F` alias는 숨기고 `0x60..0x7F`는 reserved로 유지하며, 쓰기는
`0x00..0x1C` CTL 범위만 통과시킨다.

## 7. 파생 수정 — TB 기대값의 정직화

STAT 읽기가 살아나자 첫 실측이 실제 상태를 드러냈다:
`after-run1`에서 **STAT5[0] busy='1'** — stop_tdc만 보내고 CMD_STOP이
없으므로 face_seq가 ST_WAIT_SHOT(armed)에 있는 **올바른 동작**. 기존
"0 기대"는 vacuous였다. `read_pipeline_status`를 STAT5/STAT7 기대값
파라미터로 변경: after-run1/after-recovery-run은 busy=1 기대,
post-soft-reset/post-force-reinit은 0 기대.

## 8. 검증 매트릭스 (최종, stamp 260717013000/015000)

| 실행 | 결과 |
|---|---|
| c06 v002 suite (face_seq/status_agg/top_int 32·64·128, bp, **살아난 STAT 비교 포함**) | ALL PASS |
| masked-slope wrapper (STAT7=0x00008000 exact) | PASS |
| top_int 샷 경계 stall 64/32 (260716231500) | PASS |
| lane_mask 단위 TB (abort retain → err_soft_clear → clean next run) + cell_pipe 레거시 2종 | PASS |

## 9. 남은/후속 항목

1. **HW 이력 주의**: 보드에서 pipeline STAT(0x40~0x5C)를 읽어온 SW가
   있다면 지금까지 전부 0을 읽었다 — SW 측 해석 이력 점검 필요.
2. per-chip masked_slope mask의 CSR 노출은 IP 재생성(STAT>8) 시점으로
   유보 (echo_receiver IP 재생성 건과 함께 처리 권장).
3. CHAIN-P0-03 소급 audit: LB-1/LB-2가 실증한 vacuous-PASS 패턴 기준으로
   **chip CSR STAT0~3 읽기와 다른 exact-compare 지점**도 소급 점검 가치.
4. 합성 관점: LB-1의 record 분할-소유는 합성에서 multi-driver로
   보고되었을 가능성 — 다음 합성 런에서 경고 로그 확인.
