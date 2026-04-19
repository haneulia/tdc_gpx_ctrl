#!/bin/bash
set -e
GH="/c/Program Files/GitHub CLI/gh.exe"
R="haneulia/tdc_gpx_ctrl"
REVIEW="Doc/260418/fsm_code_only_detailed_review_2026-04-18.md"

mk() {
  local title="$1"; local labels="$2"; local body="$3"
  "$GH" -R "$R" issue create --title "$title" --label "$labels" --body "$body"
}

# -------------------------------------------------------------------
# A. 고신뢰 문제 (10) — P1-critical
# -------------------------------------------------------------------

mk "[A-1] chip_run: 응답 '도착'과 '소비'를 구분 못 함" \
"P1-critical,fsm-bug,area-chip-run,area-chip-ctrl" \
"$(cat <<'EOF'
## 파일
- `HDL/tdc_gpx_chip_ctrl.vhd:425-439`
- `HDL/tdc_gpx_chip_run.vhd:496-600`

## 핵심
- `chip_ctrl`가 `s_bus_rsp_fire <= i_s_axis_tvalid and o_s_axis_tready`만 sub-FSM으로 전달함.
- 즉 `chip_run`은 **버스 응답이 도착했는지**가 아니라 **응답이 받아들여졌는지**만 안다.
- `PH_RUN`에서 raw hold가 가득 차면 `o_s_axis_tready <= '0'`이 됨.

## 결과
- 응답 beat가 bus/skid 쪽에 pending 상태여도 `chip_run`은 "응답 없음"으로 봄.
- `ST_DRAIN_EF1`, `ST_DRAIN_EF2`, `ST_DRAIN_BURST`, `ST_DRAIN_FLUSH` watchdog이 **가짜 timeout**을 낼 수 있음.

## 권고
- `chip_run`에 `i_bus_rsp_pending` 또는 `i_rsp_seen` 신호를 추가해 "도착"과 "소비"를 분리.

## 참조
`Doc/260418/fsm_code_only_detailed_review_2026-04-18.md` 섹션 A-1
EOF
)"

mk "[A-2] ST_DRAIN_FLUSH가 pending response를 남긴 채 종료 가능" \
"P1-critical,fsm-bug,area-chip-run,area-chip-ctrl" \
"$(cat <<'EOF'
## 파일
- `HDL/tdc_gpx_chip_run.vhd:575-600`
- `HDL/tdc_gpx_chip_ctrl.vhd:425-439`
- `HDL/tdc_gpx_chip_ctrl.vhd:582-609`

## 핵심
- `chip_run`은 `i_bus_busy='0' and i_bus_rsp_valid='0'`이면 flush 완료로 봄.
- 하지만 여기서 `i_bus_rsp_valid`는 실제 pending valid가 아니라 `s_bus_rsp_fire`.

## 결과
- bus_phy/skid에 읽기 응답이 남아 있어도, downstream backpressure로 `fire=0`이면 `chip_run`은 flush를 끝냄.
- 다음 phase에서 stale response가 튀어나올 수 있음.

## 권고
- `ST_DRAIN_FLUSH` 종료 조건에 `rsp_pending`까지 포함하거나, `chip_ctrl`가 flush 완료 pulse를 만들어 `chip_run`이 보도록.

## 참조
섹션 A-2
EOF
)"

mk "[A-3] chip_ctrl 2-depth raw hold/skid — beat drop 가능성" \
"P1-critical,fsm-bug,area-chip-ctrl" \
"$(cat <<'EOF'
## 파일
- `HDL/tdc_gpx_chip_ctrl.vhd:735-803`
- `HDL/tdc_gpx_chip_ctrl.vhd:822-833`

## 핵심
- hold+skid가 둘 다 찬 상태에서 새 beat가 오면 drop하고 `s_err_raw_overflow_r`을 올림.
- `s_raw_hold_busy`는 등록형이라 1 cycle 늦게 반영됨.

## 결과
- 최악 동시성에서 **3번째 beat**가 들어오면 raw data뿐 아니라 control beat도 유실 가능.

## 권고
- 최소 3-depth로 확장하거나, `drain_done` / `ififo1_done` 같은 control beat를 별도 채널로 분리.

## 참조
섹션 A-3
EOF
)"

mk "[A-4] drop 대상에 control beat까지 포함됨" \
"P1-critical,fsm-bug,area-chip-ctrl" \
"$(cat <<'EOF'
## 파일
- `HDL/tdc_gpx_chip_ctrl.vhd:740-748`
- `HDL/tdc_gpx_chip_ctrl.vhd:785-790`

## 핵심
- `s_run_drain_done`, `s_run_ififo1_beat`도 동일한 hold/skid 경로로 패킹됨.

## 결과
- overflow 시 raw word뿐 아니라 `drain_done` / `ififo1_done` 같은 FSM 제어 의미 beat도 사라질 수 있음.

## 권고
- control beat를 별도 소형 FIFO 또는 우선순위 경로로 분리.

## 참조
섹션 A-4
EOF
)"

mk "[A-5] chip_run overrun recovery가 같은 cycle의 bus response를 버림" \
"P1-critical,fsm-bug,area-chip-run" \
"$(cat <<'EOF'
## 파일
- `HDL/tdc_gpx_chip_run.vhd:684-721`

## 핵심
- `i_shot_start='1'`가 비-ARMED 상태에 오면 overrun recovery가 우선.
- 같은 cycle `i_bus_rsp_valid='1'`이면 `s_err_overrun_drop_r`만 세우고 beat는 소실.

## 결과
- drain accounting과 실제 전달 raw beat 수가 어긋날 수 있음.

## 권고
- response beat를 1-entry 우선 보관 후 overrun으로 전환, 또는 `fire`가 난 cycle은 overrun을 1 cycle defer.

## 참조
섹션 A-5
EOF
)"

mk "[A-6] cell_builder DROP-timeout 복귀 후 late stale beat — upstream stall" \
"P1-critical,fsm-bug,area-cell-builder" \
"$(cat <<'EOF'
## 파일
- `HDL/tdc_gpx_cell_builder.vhd:305-315`
- `HDL/tdc_gpx_cell_builder.vhd:513-534`

## 핵심
- `ST_C_DROP` timeout 시 강제로 `ST_C_IDLE`로 복귀.
- 이후 늦게 stale beat가 오면 `IDLE`에서 `tready='0'`이라 upstream backpressure를 걺.

## 결과
- incomplete recovery 후 파이프라인이 소프트리셋 전까지 묶일 수 있음.

## 권고
- `ST_C_QUARANTINE` 상태 도입 — explicit reset/drain marker 전까지 버리도록 설계.

## 참조
섹션 A-6
EOF
)"

mk "[A-7] config_ctrl multi-bit snapshot — handshake 없는 CDC 샘플링" \
"P1-critical,cdc,area-config-ctrl" \
"$(cat <<'EOF'
## 파일
- `HDL/tdc_gpx_config_ctrl.vhd:702-719`

## 핵심
- `s_cfg_merged`, `o_cfg_image`, `s_expected_ififo1/2`, `s_cmd_reg_addr_mux`, `s_cmd_reg_wdata`를 TDC clock에서 매 cycle 그냥 샘플.
- 주석도 "quasi-static" 가정에 기대고 있음.

## 결과
- SW가 값 변경과 command pulse 타이밍을 잘못 겹치면 비트 혼합 / 반쯤 바뀐 snapshot이 잡힐 수 있음.
- 특히 expected IFIFO count가 엇갈리면 drain burst sizing이 깨질 수 있음.

## 권고
- `cfg_shadow_valid/toggle` 기반 명시적 CDC handshake 또는 dual-clock register bank.

## 참조
섹션 A-7
EOF
)"

mk "[A-8] TDC clock 도메인 모듈이 AXI reset을 직접 받음" \
"P1-critical,cdc,area-config-ctrl" \
"$(cat <<'EOF'
## 파일
- `HDL/tdc_gpx_config_ctrl.vhd:742-746`
- `HDL/tdc_gpx_config_ctrl.vhd:957-967`
- `HDL/tdc_gpx_config_ctrl.vhd:781-789`

## 핵심
- `i_tdc_clk`로 구동되는 `u_bus_phy`, `u_chip_ctrl`, `u_sk_brsp`에 `i_rst_n => i_axis_aresetn`이 직접 들어감.

## 결과
- reset assert는 괜찮지만 **deassert가 i_tdc_clk 기준 비동기**라 release metastability / 부분 해제 위험.

## 권고
- `i_axis_aresetn`을 TDC domain에서 2FF sync 후 deassert하는 domain-local reset 생성.

## 참조
섹션 A-8
EOF
)"

mk "[A-9] PH_RESP_DRAIN 15-cycle hard cap — stale response 남긴 채 phase 강행" \
"P1-critical,fsm-bug,area-chip-ctrl" \
"$(cat <<'EOF'
## 파일
- `HDL/tdc_gpx_chip_ctrl.vhd:582-609`

## 핵심
- 15 cycle 내 `i_bus_busy` / `i_bus_rsp_pending`이 안 비워지면 sticky만 올리고 phase를 강행.

## 결과
- 다음 phase에 stale response가 섞일 수 있는 구조.

## 권고
- hard cap 유지하더라도 다음 phase 진입 전 bus reset/recover 절차 강제, 또는 stale response quarantine.

## 참조
섹션 A-9
EOF
)"

mk "[A-10] cmd_arb가 active reg transaction 중 새 reg request를 무시" \
"P1-critical,fsm-bug,area-cmd-arb" \
"$(cat <<'EOF'
## 파일
- `HDL/tdc_gpx_cmd_arb.vhd:246-319`

## 핵심
- 새 요청 수락은 `v_new_request='1' and s_reg_active_r='0'`일 때만.
- active 동안 들어온 새 transaction-set은 pending queue / sticky reject 없이 사라짐.

## 결과
- SW / 상위 FSM이 pulse를 겹치게 주면 요청 손실 발생.

## 권고
- arb 레벨에 1-depth pending set 또는 explicit reject/status bit 추가.

## 참조
섹션 A-10
EOF
)"

# -------------------------------------------------------------------
# B. 고위험 구조 가정 / 관찰성 결함 (8) — P2-important
# -------------------------------------------------------------------

mk "[B-1] chip_ctrl o_drain_done은 실제 drain 완료가 아니라 'control beat accepted' 시점" \
"P2-important,observability,area-chip-ctrl" \
"$(cat <<'EOF'
## 파일
- `HDL/tdc_gpx_chip_ctrl.vhd:811-820`

## 핵심
- 주석에도 명시돼 있듯, downstream backpressure가 있으면 `o_drain_done`이 늦어짐.

## 리스크
- 소비 측이 이 pulse를 "chip_run drain 종료 시점"으로 오해하면 상태 판단이 어긋남.

## 권고
- `o_run_drain_complete` (internal completion)와 `o_drain_done_accepted` (stream handshake) 분리.

## 참조
섹션 B-1
EOF
)"

mk "[B-2] chip_reg 3번째 pulse overflow sticky가 외부로 노출 안 됨" \
"P2-important,observability,area-chip-reg" \
"$(cat <<'EOF'
## 파일
- `HDL/tdc_gpx_chip_reg.vhd:16-21`
- `HDL/tdc_gpx_chip_reg.vhd:46-62`
- `HDL/tdc_gpx_chip_reg.vhd:175-190`

## 핵심
- `s_err_req_overflow_r` sticky가 내부에는 있지만 port가 없음.

## 리스크
- 실제 요청 손실이 나도 SW / 상위는 모름.

## 권고
- status port 또는 status register로 노출.

## 참조
섹션 B-2
EOF
)"

mk "[B-3] err_handler read-timeout sticky가 외부로 노출 안 됨" \
"P2-important,observability,area-err-handler" \
"$(cat <<'EOF'
## 파일
- `HDL/tdc_gpx_err_handler.vhd:16-21`
- `HDL/tdc_gpx_err_handler.vhd:76-80`
- `HDL/tdc_gpx_err_handler.vhd:268-280`

## 핵심
- `s_err_read_timeout_r`는 내부 sticky뿐, output 없음.

## 리스크
- 분류 실패가 "없었다"로 보일 수 있음.

## 권고
- `o_err_read_timeout` 또는 cause 확장.

## 참조
섹션 B-3
EOF
)"

mk "[B-4] chip_init이 busy 중 추가 cfg_write_req를 일반적으로 큐잉하지 않음" \
"P2-important,fsm-bug,area-chip-init" \
"$(cat <<'EOF'
## 파일
- `HDL/tdc_gpx_chip_init.vhd:21-22`
- `HDL/tdc_gpx_chip_init.vhd:151-163`

## 핵심
- ST_OFF에서 `start`와 동시인 `cfg_write_req`만 1-depth pending으로 저장.
- busy 중 새 cfg_write pulse는 별도 흡수 경로 없음.

## 리스크
- SW가 연속 cfg_write pulse를 주면 요청 소실.

## 권고
- pending latch를 busy 중에도 유지, 또는 level request로 변경.

## 참조
섹션 B-4
EOF
)"

mk "[B-5] face_assembler가 shot boundary 보존 위해 늦게 온 이전 shot data를 무조건 버림" \
"P2-important,fsm-bug,area-face-assembler" \
"$(cat <<'EOF'
## 파일
- `HDL/tdc_gpx_face_assembler.vhd:350-358`

## 핵심
- `s_flush <= i_shot_start or i_abort`
- 주석: "Late-arriving beats from previous shot are intentionally dropped."

## 리스크
- 시스템 타이밍이 조금만 어긋나도 데이터 completeness보다 boundary integrity가 무조건 우선 — old-shot tail이 잘림.

## 권고
- per-shot tag 비교 후 old-shot만 선별 폐기, 또는 new shot accept 시점을 line-close 이후로 제한.

## 참조
섹션 B-5
EOF
)"

mk "[B-6] face_assembler overrun 시 이후 chip을 blank-fill로 강제하는 공격적 정책" \
"P2-important,observability,area-face-assembler" \
"$(cat <<'EOF'
## 파일
- `HDL/tdc_gpx_face_assembler.vhd:692-711`

## 핵심
- mid-row shot overrun 시 `s_shot_cnt_r <= x\"FFFF\"`로 upcoming chips를 blank mode로 몰고 감.

## 리스크
- 정책상 맞을 수는 있지만, 아주 짧은 경계 교차에서도 정상 데이터 일부를 버림.

## 권고
- 정책 유지 시에도 카운터 / trace / status를 더 노출해 원인 추적성 향상.

## 참조
섹션 B-6
EOF
)"

mk "[B-7] cmd_arb zero-mask reg request — 런타임 관찰성 없음" \
"P2-important,observability,area-cmd-arb" \
"$(cat <<'EOF'
## 파일
- `HDL/tdc_gpx_cmd_arb.vhd:247-255`

## 핵심
- zero mask는 ignore되고 sim assert만 남음.

## 리스크
- 실기에서 원인 불명의 "아무 일도 안 일어남"으로 보임.

## 권고
- sticky status 추가.

## 참조
섹션 B-7
EOF
)"

mk "[B-8] chip_run expected IFIFO count 계약 — 주석 자기모순" \
"P3-maintenance,maintenance,area-chip-run" \
"$(cat <<'EOF'
## 파일
- `HDL/tdc_gpx_chip_run.vhd:24-29`
- `HDL/tdc_gpx_chip_run.vhd:84-87`

## 핵심
- 파일 상단 주석: "drain_latch 전 안정성 assertion을 제거했고 continuous update와 충돌하므로 기대하면 안 된다."
- 포트 주석: 여전히 "must be stable from i_shot_start through ST_DRAIN_LATCH."

## 리스크
- 기능보다 유지보수 위험. 잘못된 상위 가정을 다시 심을 수 있음.

## 권고
- 포트 주석을 현재 실제 설계 의도와 맞춰 정리.

## 참조
섹션 B-8
EOF
)"

# -------------------------------------------------------------------
# C. 유지보수 / 인터페이스 드리프트 (5) — P3-maintenance
# -------------------------------------------------------------------

mk "[C-1] face_assembler o_face_abort는 사실상 죽은 포트" \
"P3-maintenance,maintenance,area-face-assembler" \
"$(cat <<'EOF'
## 파일
- `HDL/tdc_gpx_face_assembler.vhd:220-221`
- `HDL/tdc_gpx_face_assembler.vhd:725-730`
- `HDL/tdc_gpx_face_assembler.vhd:743-747`

## 핵심
- `s_face_abort_r`는 reset만 되고, overrun/abort 경로에서 더 이상 assert하지 않는다고 주석에 명시.

## 리스크
- 외부가 이 포트를 아직 의미 있다고 믿으면 오동작.

## 권고
- 제거, 또는 deprecated 표기 강화 + 연결부 정리.

## 참조
섹션 C-1
EOF
)"

mk "[C-2] face_seq s_shot_overrun_r — set/clear만 되고 내부에서 안 읽힘" \
"P3-maintenance,maintenance,area-face-seq" \
"$(cat <<'EOF'
## 파일
- `HDL/tdc_gpx_face_seq.vhd:114-119`

## 리스크
- dead logic이 future edit에서 혼란 유발.

## 권고
- 실제로 쓸 계획이 없으면 삭제.

## 참조
섹션 C-2
EOF
)"

mk "[C-3] header_inserter 주석이 'non-IDLE face_start ignored'라고 하지만 실제 pending latch 있음" \
"P3-maintenance,maintenance,area-header-inserter" \
"$(cat <<'EOF'
## 파일
- `HDL/tdc_gpx_header_inserter.vhd:480-500`

## 핵심
- 코드 자체는 좋아졌지만 주석은 예전 contract를 일부 유지.

## 리스크
- 나중에 누군가 주석을 믿고 상위 pulse timing을 바꾸면 혼란.

## 권고
- 주석을 현재 동작 기준으로 다시 정리.

## 참조
섹션 C-3
EOF
)"

mk "[C-4] cell_pipe 시뮬레이션 assert — concurrent 영역에서 rising_edge 사용" \
"P3-maintenance,maintenance,area-cell-pipe" \
"$(cat <<'EOF'
## 파일
- `HDL/tdc_gpx_cell_pipe.vhd:220-227`

## 리스크
- 일부 시뮬레이터 / lint에서 비표준 또는 tool-fragile.

## 권고
- clocked process 내부 assert로 이동.

## 참조
섹션 C-4
EOF
)"

mk "[C-5] err_handler 포트명이 여전히 reg11인데 실제는 Reg12 읽기" \
"P3-maintenance,maintenance,area-err-handler" \
"$(cat <<'EOF'
## 파일
- `HDL/tdc_gpx_err_handler.vhd:51-57`

## 리스크
- backward compatibility 때문에 남겼다고 적혀 있지만, 인터페이스 이름과 실제 의미가 다름.

## 권고
- wrapper에서 alias를 두거나 다음 라운드에서 명칭 정리.

## 참조
섹션 C-5
EOF
)"

echo "--- all 23 issues created ---"
