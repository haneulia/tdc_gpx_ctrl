# C07 Shot-Boundary FIFO Flush Guard (CHAIN-P0-01 부분 closure)

| 항목 | 내용 |
|---|---|
| 문서 종류 | P0 결함 수정 + 검증 결과 |
| 문서 버전 | v001 |
| 생성 시간 | 2026-07-16 21:35:00 KST |
| 관련 P0 | `cluster_analysis_260514151507_Chain_Integrity_Review_v001.md` CHAIN-P0-01 (output stream CDC 전체 재설계 closure) |
| 절대 기준 문서 | `Doc/TDC-GPX-Datasheet.pdf` |
| session archive | `sim_results/vivado_xsim/sessions/260716213500_shot_bp_p0_fix/` |

## 1. 결론

`shot_start` 마다 출력 경로 FIFO 2단(면조립기 내부 `u_fifo_out`, output_stage `u_face_rise/fall_fifo`)을 무조건 리셋하던 xsim 워크어라운드가, **VDMA backpressure 상태의 mid-face shot 경계에서 이전 line의 잔류 beat(tlast 포함)를 파괴**하는 실제 데이터 손상 경로임을 시뮬레이션으로 재현하고, occupancy-gated reset으로 수정했다.

```text
수정 전: Scenario B FAIL  (beats 60/88, tlast 1/2, frame_done 0, line0 word4부터 오염)
수정 후: Scenario A/B PASS (beats 88, tlast 2, SOF 1, frame_done 1, 데이터 무결)
기존 회귀: 64/32/128-bit 시나리오 1-2, max_hits sweep 32/64/128 전부 PASS
```

## 2. 결함 메커니즘

1. `tdc_gpx_output_stage.vhd`의 `s_fifo_rst_n_rise/fall` 과
   `tdc_gpx_face_assembler.vhd`의 `s_out_fifo_rst_n` 은 xsim 2025.x
   `xpm_fifo_axis` tready-unlock 워크어라운드로 `shot_start` 마다 리셋 펄스를
   냈다.
2. 주석의 안전 근거("shot_start 시점에 face 데이터 in-flight 없음")는 face의
   첫 shot(packet_start, `hdr_idle` 인터락)에만 성립한다. mid-face shot은
   `face_seq` 가 downstream 배수 상태를 확인하지 않는다 (파이프라인 설계 의도).
3. VDMA `tready` 가 shot 경계 부근에서 수십 clk 이상 stall 하면 이전 line의
   잔류 beat가 두 FIFO에 남는다. 리셋 펄스가 이를 소거하면:
   - `tdc_gpx_line_packer` 의 cell 내부 beat index가 어긋나 이후 word 오염
   - line의 `tlast` 소실로 `tdc_gpx_header_inserter` 의 col 카운트 미달
   - `frame_done` 미발생, face 전체 손상 (drain watchdog sticky로만 관측)

## 3. 수정 내용

원칙: xsim unlock 펄스는 유지하되, **FIFO가 "증명 가능하게 비어 있을 때만"**
shot_start 리셋을 통과시킨다. 점유는 write/read handshake 카운터로 추적한다.

| 파일 | 변경 |
|---|---|
| `tdc_gpx_output_stage.vhd` | `p_face_fifo_track` (rise/fall 점유 카운터, 0..16) 추가. `s_fifo_rst_n_* <= not (abort or (shot_start and quiet))`. `quiet = count=0 and s_axis_tvalid=0` (동일 cycle write 보호) |
| `tdc_gpx_face_assembler.vhd` | `p_out_fifo_track` 카운터 추가, `m_axis_tvalid` 내부 미러(`s_m_axis_tvalid_int`) 도입. `s_out_fifo_rst_n` 동일 게이팅. `:436` stale 주석("abort ONLY") 정정 |

동작 성질:

- 최초 shot_start 시 FIFO는 비어 있으므로 unlock 펄스는 기존과 동일하게 발생
- FIFO 비어있지 않으면 리셋 skip — 그 시점엔 tready 가 살아있음이 데이터
  존재 자체로 증명되므로 unlock 불필요
- abort 경로 리셋은 기존과 동일 (카운터도 abort에서 clear)

## 4. 검증

### 4.1 신규 TB: `tb_tdc_gpx_output_stage_shot_bp.vhd`

- 32-bit, rise mask `0001`, 6 stops, max_hits=7, cols_per_face=2
- line = 12 prefix + 32 data = 44 beats, face = 88 beats
- Scenario A: backpressure 없음 (sanity + unlock 펄스 생존 확인)
- Scenario B: `m_axis_tready='0'` 로 line 1 전체(30 beats)를 FIFO 체인에
  적재 → `face_buf_tvalid='1'` source marker 확인 → shot 2 펄스(트리거) →
  tready 해제 → face 완결성 검사 (beat/tlast/SOF/frame_done + word 단위
  데이터 비교)

### 4.2 수정 전 재현 (결함 증거)

- log: `logs/simulate/xsim_shot_bp_prefix.log` (session archive)
- Scenario A PASS / **Scenario B FAIL**:
  `beats=60 tlast=1 sof=1 frame_done=0`,
  `data mismatch line=0 word=4..33`,
  `tdc_gpx_line_packer: line ended away from a cell metadata beat`
- 손상 시작점 word=4 는 line_packer 가 이미 수취한 4 word 직후 = FIFO 소거
  지점과 정확히 일치 (destination/effect marker)

### 4.3 수정 후 (output marker)

- log: `logs/simulate/xsim_shot_bp_postfix.log`
- Scenario A: `beats=88 tlast=2 sof=1 frame_done=1` PASS
- Scenario B: `beats=88 tlast=2 sof=1 frame_done=1` PASS, mismatch 0

### 4.4 무회귀

| TB | 결과 | log |
|---|---|---|
| `tb_tdc_gpx_output_stage` (64b, 시나리오 1-2) | PASS | `xsim_outstage_regr_64.log` |
| `tb_tdc_gpx_output_stage_w32` / `_w128` | PASS | `xsim_regr_w32/128_snap.log` |
| `tb_tdc_gpx_output_stage_maxhits_w32/64/128` (sweep 0..7) | PASS | `xsim_regr_mh*_snap.log` |

재실행: `powershell -File scripts\run_shot_bp_regression.ps1`

## 5. 남은 판단 (CHAIN-P0-01 전체 closure 관점)

이 수정은 CHAIN-P0-01 의 **구체적 데이터-손상 경로 하나를 제거**한 것이다.
전체 closure 로 인정하려면 다음이 남는다.

1. full_int / top_int 통합 레벨에서 shot 경계 tready stall 주입 재검증
   (기존 `G_BP_TREADY_GAP` 확장으로 가능)
2. 장기 VDMA stall 시 system policy (C06 handoff 항목) — bounded stall 을
   넘는 경우 abort/재시작 정책은 여전히 시스템 결정 사항
3. xsim tready-lock 워크어라운드 자체의 근본 제거 여부 (xpm_fifo_axis →
   자체 `tdc_gpx_sync_fifo` 대체) 는 별도 트레이드오프 항목으로 유지

## 6. 부수 관찰 (이번 세션에서 발견, 미수정)

- DEDICATED_2X2 모드에서 slope demux 의 drain_done broadcast 로 인해
  반대-slope cell_builder 가 매 shot blank slice 를 생성, face_assembler 의
  masked chip 입력 FIFO 에서 매 shot flush 됨 → `shot_flush_drop` sticky /
  mask 상시 오염 (진단 가치 저하, 데이터 오염 없음). P1 후보.
- `face_seq` 의 legacy geometry 출력(`o_rows_per_face`, `o_hsize_bytes`)은
  top 에서 수신만 되고 미사용. 정리 대상 P2.
