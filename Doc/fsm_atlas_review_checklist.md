# TDC-GPX FSM Atlas Review — 점검 결과 (2026-04-17)

**기준 덱:** `TDC_GPX_FSM_Atlas_Review.pptx` (13슬라이드, RTL/VHDL only)
**기준 코드:** 커밋 `5a7356e` (refactor/module-architecture)
**이전 기준:** 덱은 커밋 `76f1084` 기준으로 작성됨

---

## 슬라이드별 점검 요약

### Slide 1 — 표지 / 범위 정의
- 범위: RTL/VHDL 코드만 기준, 계약 문서/데이터시트 제외
- **점검 결과:** 해당 없음 (표지)

### Slide 2 — FSM 인벤토리와 연결 구조
- 명시적 FSM: bus_phy, chip_ctrl, chip_init, chip_reg, chip_run, err_handler, cell_builder(collect/output), face_seq, face_assembler, header_inserter, skid_buffer
- FSM 없는 모듈: config_ctrl, decode_pipe, raw_event_builder, cell_pipe, output_stage, cmd_arb
- **점검 결과:** 구조 변경 없음. 5a7356e에서 cell_builder에 buffer owner FSM(`t_buf_owner`) 추가됐으나 기존 collect/output FSM 위에 얹은 형태이므로 인벤토리 자체는 동일.

### Slide 3 — Control front-end: chip_ctrl + bus_phy
- **검토 포인트:** soft_reset 시 stale response quarantine이 FSM 레벨에서 필요
- **점검 결과:** ✅ `PH_RESP_DRAIN` 상태 추가 완료 (5a7356e). soft reset 시 8-cycle drain 후 PH_INIT 진입. brsp skid buffer도 soft_reset 시 flush.

### Slide 4 — chip_init / chip_reg
- **검토 포인트:** ordering 보증이 chip_ctrl phase와 bus routing에 의존
- **점검 결과:** ✅ PH_RESP_DRAIN이 stale response를 drain하므로, init/reg FSM이 이전 phase 응답을 오인하는 경로 차단됨. 추가로 p_rsp_check가 PH_INIT/PH_REG에서도 tuser 검증 수행.

### Slide 5 — chip_run: capture → drain → ALU → overrun
- **검토 포인트:** raw backpressure가 chip_ctrl 1-deep hold까지만 확실. chip_run ST_DRAIN_CHECK 밖에서는 producer 관점 ready/valid 계약 미완성.
- **점검 결과:** ✅ `o_s_axis_tready` gating 추가 (5a7356e). PH_RUN 중 raw hold가 full이면 bus response tready를 내려서 bus_phy → chip_run까지 자연 stall. 기존 `i_raw_busy` guard도 이중 보호로 유지.

### Slide 6 — err_handler: automatic recovery FSM
- **검토 포인트:** fatal로 ST_IDLE에 떨어질 때 err_fill/chip_mask/cause 정리가 명시적이어야 함
- **점검 결과:** ✅ fatal 시 `s_err_fill_r` + `s_err_chip_mask_r` clear 추가 (5a7356e). `s_err_cause_r`는 진단용으로 보존.

### Slide 7 — cell_builder: collect/output dual FSM
- **검토 포인트:** deferred swap + pending queue 대신 buffer owner FSM 또는 3-buffer로 분리 권장
- **점검 결과:** ✅ per-buffer owner FSM (`BUF_FREE/BUF_COLLECT/BUF_SHARED`) 전면 재설계 (5a7356e).
  - `s_swap_pending_r` 제거
  - `s_output_pending_r/s_pending_buf_idx_r/s_pending_full_r/s_consume_pending_r` 제거
  - shot_start 시 BUF_FREE 버퍼를 찾아 할당, 없으면 drop
  - ififo1_done → BUF_COLLECT→BUF_SHARED (동시 collect+output)
  - output 완료 → BUF_SHARED→BUF_FREE
  - `i_abort` 포트 추가: 모든 버퍼 BUF_FREE로 즉시 복귀

### Slide 8 — face_seq: face / frame / shot sequencer
- **검토 포인트:** shot_start_gated/pipeline_abort/face_closing이 enum state 밖 derived logic이므로 함께 봐야 함
- **점검 결과:** face_seq 자체는 변경 없음. pipeline_abort가 이제 모든 stateful stage를 flush하므로 abort 후 파이프라인 정합성은 개선됨. 다만 **quiesce_ack 수집 후 다음 shot gating**은 미구현 (아래 참조).

### Slide 9 — face_assembler: strict in-order row assembly
- **점검 결과:** 구조 변경 없음. 기존 `i_abort` 포트로 abort 처리 유지. `active_chip_mask=0` guard는 76f1084에서 이미 추가.

### Slide 10 — header_inserter + skid_buffer
- **점검 결과:** 구조 변경 없음. 기존 abort-drain 메커니즘 유지.

### Slide 11 — 명시적 FSM이 없는 모듈들
- **검토 포인트:** cmd_arb와 config_ctrl는 명시적 FSM 없지만 전체 정합성에 큰 영향
- **점검 결과:**
  - cmd_arb: 16-bit reg timeout 카운터 추가됨 (7621194). chip 무응답 시 강제 all-done.
  - config_ctrl: err_active 중 write 차단, err_fill gating, brsp skid soft_reset flush 모두 반영.
  - raw_event_builder: `i_abort` 포트 추가 (5a7356e). hit counter + output register flush.
  - cell_pipe: `i_abort` 포트 추가 (5a7356e). slope demux register flush.

### Slide 12 — FSM-to-FSM Contract Hotspots

| # | Hotspot | 반영 상태 |
|---|---------|----------|
| 1 | soft reset → stale response | ✅ PH_RESP_DRAIN + brsp flush |
| 2 | raw backpressure source end | ✅ o_s_axis_tready gating |
| 3 | cell_builder ownership / queue | ✅ per-buffer owner FSM |
| 4 | fatal cleanup | ✅ err_fill + chip_mask clear |

### Slide 13 — 추가 보완 검토 결론

#### 필수 before freeze (4개)

| 항목 | 반영 | 커밋 |
|------|:----:|------|
| cell_builder owner-driven buffer FSM | ✅ | 5a7356e |
| soft reset RESP_DRAIN state / epoch | ✅ | 5a7356e |
| raw path end-to-end handshake | ✅ | 5a7356e |
| err_handler fatal cleanup | ✅ | 5a7356e |

#### 강한 권장 (3개)

| 항목 | 반영 | 비고 |
|------|:----:|------|
| pipeline_abort 뒤 quiesce_ack gating | ⚠️ | 모든 stage에 i_abort 추가 완료. quiesce_ack 수집+gating은 미구현 |
| response 검증 init/run 확대 | ✅ | p_rsp_check 전 phase 확장 (5a7356e) |
| pending queue depth > 1 / overflow policy | ✅ | owner FSM 구조상 depth=1이 정확, 주석 명시 |

#### Freeze 판단 (덱 기준)

| 덱 기준 | 현재 상태 |
|---------|----------|
| "필수 4개가 닫히면 상당수 위험이 FSM contract 차원에서 정리됨" | ✅ 4개 전부 닫힘 |
| "그 이후에는 local bug fix + regression으로 freeze 검토 가능" | ✅ 현재 이 단계 |
| "바로 freeze 권고 단계는 아님" | → 필수 4개 닫힘으로 조건 충족 |

---

## 미완료 항목

### quiesce_ack (강한 권장, 미구현)

**현재 상태:** `pipeline_abort` 시 모든 stateful stage(`raw_event_builder`, `cell_pipe`, `cell_builder`)가 `i_abort`를 받아 즉시 idle로 복귀한다. 그러나 face_seq는 abort 후 각 stage의 idle 상태를 **확인하지 않고** 다음 shot을 허용할 수 있다.

**위험도:** 낮음. abort는 1-cycle 동기 신호이므로 모든 stage가 같은 cycle에 idle로 돌아간다. face_seq의 `s_pipeline_abort`도 같은 cycle에 asserted되므로 shot_start_gated가 막힌다. 하지만 abort pulse가 끝난 다음 cycle에 즉시 새 shot이 가능한 구조라 이론적 경합 여지는 있다.

**권장 수정:** face_seq에서 abort 후 1~2 cycle quiesce guard를 추가하거나, 각 stage의 idle 상태를 수집하여 gating.

---

## 커밋 이력

| 커밋 | 내용 |
|------|------|
| `dec8c63` | 초기 8개 fix: handshake, backpressure, pending queue, write mux, slope BP, flush, generic, leaf safety |
| `7621194` | P3 7개 fix: multi-driver 제거, backpressure 전파, purge drain_done, fatal idle, cmd_arb timeout, 주석, mask=0 guard |
| `547315c` | deferred swap, bus response tuser validation |
| `76f1084` | observability: unmasked chip error, shot_seq in tuser |
| `5a7356e` | **FSM contract hardening**: buffer owner FSM, tready gating, PH_RESP_DRAIN, end-to-end abort, fatal err_fill clear, tuser 전 phase 검증, rsp_mismatch status 노출 |

---

## 결론

덱(TDC_GPX_FSM_Atlas_Review)이 제시한 **필수 4개 항목 전부 닫힘**, **강한 권장 3개 중 2개 완료, 1개(quiesce_ack) 부분 반영**.
덱 기준 "local bug fix + regression으로 freeze 검토 가능" 단계에 진입.
