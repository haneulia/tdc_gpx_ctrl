# TDC-GPX FSM 코드 전수 점검 보고서 (코드 기준)

기준:
- 판단 근거는 업로드된 **VHDL 코드만**이다.
- 계약 문서, 요구 문서, 데이터시트는 판단 근거에서 제외했다.
- 초점은 **FSM 상태 전이, 완료 조건, timeout/liveness, pulse 흡수, 상태 관찰성, 상호 모듈 정합성**이다.

총평:
- 이전 라운드 대비 분명히 좋아진 부분이 있다.
- 하지만 현재 업로드본을 **FSM 안정화 완료**라고 부르기에는 아직 이르다.
- 이번 전수 점검에서 **총 24개 항목**을 정리했다.
  - 치명/높음: 10
  - 중간: 8
  - 유지보수/관찰성: 6

---

## 먼저 좋아진 점

1. `tdc_gpx_err_handler.vhd`
   - `ST_WAIT_READ`에 watchdog이 추가되어, read done이 안 와도 영구 정지하지 않도록 보강됨.

2. `tdc_gpx_face_assembler.vhd`
   - `ST_FORWARD` real-chip 경로에 mid-stream stall watchdog이 추가되어, 중간 stall 시 blank-fill로 전환 가능해짐.

3. `tdc_gpx_header_inserter.vhd`
   - non-idle `face_start`를 무시하지 않고 pending latch로 흡수하도록 개선됨.

4. `tdc_gpx_chip_ctrl.vhd`
   - `PH_RESP_DRAIN` hard-cap 이후에도 secondary quarantine watchdog을 두어 영구 정지 대신 재-init로 탈출할 수 있게 됨.

5. `tdc_gpx_chip_init.vhd`
   - pending `cfg_write`에 대해 요청 시점의 `cfg_image` snapshot을 저장하도록 개선됨.

6. `tdc_gpx_config_ctrl.vhd`
   - reg command bundle은 handshake CDC로 옮겨져, 주소/데이터 atomicity가 예전보다 나아짐.

---

## A. 치명 / 높음 우선순위

### 1. `chip_run`의 `ST_DRAIN_FLUSH`는 `i_bus_rsp_pending='1'`이 지속되면 사실상 멈출 수 있음
- 파일: `tdc_gpx_chip_run.vhd`
- 라인: 667-699
- 내용:
  - `i_bus_rsp_valid='1'`이면 `s_wait_cnt_r`를 리셋한다.
  - `i_bus_rsp_valid='0'`일 때는 **오직 `i_bus_rsp_pending='0'`인 경우에만** `s_wait_cnt_r`가 증가한다.
  - 종료 조건도 `i_bus_busy='0' and i_bus_rsp_pending='0'`이다.
- 문제:
  - pending이 계속 1이면 watchdog 카운터가 전혀 전진하지 않는다.
  - EF/BURST 상태에는 pending-stuck 카운터가 있는데, `ST_DRAIN_FLUSH`에는 그 성격의 별도 watchdog이 없다.
- 영향:
  - drain flush에서 FSM이 장시간 또는 영구 정지 가능.
- 권고:
  - `ST_DRAIN_FLUSH`에도 별도 `pending_stuck` watchdog 추가.
  - 또는 `pending` 지속 시간 상한 도달 시 timeout cause를 분리해서 탈출.

### 2. `chip_ctrl` raw FIFO는 여전히 beat drop 가능성이 있고, data/control beat가 같은 경로를 공유함
- 파일: `tdc_gpx_chip_ctrl.vhd`
- 라인: 796-840
- 내용:
  - `s_run_raw_valid` beat와 `s_run_drain_done`, `s_run_ififo1_beat` control beat가 모두 같은 `v_new` 경로로 enqueue된다.
  - FIFO가 꽉 차면 `s_err_raw_overflow_r <= '1'` 후 beat를 drop한다.
- 문제:
  - 단순 raw data만이 아니라 **drain_done / ififo1_done 같은 control beat도 drop**될 수 있다.
  - control beat가 사라지면 상위 완료/정렬 로직이 깨진다.
- 영향:
  - 데이터 유실 + 상태 전이 의미 유실.
- 권고:
  - data/control 경로 분리.
  - 최소한 control beat 우선 보존 정책 추가.
  - overflow 발생 시 어떤 종류의 beat가 유실됐는지 별도 status 분리.

### 3. `cmd_arb`는 `s_all_done='1'`인 그 cycle에 들어온 새 reg request를 놓칠 수 있음
- 파일: `tdc_gpx_cmd_arb.vhd`
- 라인: 299-331, 334-420
- 내용:
  - branch 우선순위가 `if s_all_done ... elsif v_new_request ... elsif s_reg_active ...` 순서다.
  - 완료 cycle에 새 request가 1-cycle pulse로 들어오면 `s_all_done` branch가 먼저 먹고, 새 pulse를 queue에 흡수하지 않는다.
- 문제:
  - just-finished transaction의 완료 cycle과 새 요청이 겹치면 1-cycle pulse 소실 가능.
- 영향:
  - SW/상위 FSM이 넣은 reg access가 간헐적으로 누락될 수 있음.
- 권고:
  - `s_all_done` branch 내부에서 `v_new_request`를 우선 queue absorb.
  - 또는 새 요청 capture용 선행 래치 추가.

### 4. `err_handler`는 `ST_READ_REG12`에서 `i_reg_outstanding='0'`만 기다리며 watchdog이 없음
- 파일: `tdc_gpx_err_handler.vhd`
- 라인: 238-249
- 문제:
  - 이미 진행 중인 reg transaction이 영구 outstanding이면 recovery FSM이 `ST_READ_REG12`에서 못 빠져나온다.
  - `ST_WAIT_READ` timeout은 추가됐지만, 그 전에 read issuance 자체가 막히는 상태는 보호되지 않는다.
- 영향:
  - error recovery path livelock.
- 권고:
  - `ST_READ_REG12`용 timeout 추가.
  - timeout 시 원인 sticky를 세우고 recovery를 계속하거나 상위에 escalation.

### 5. `err_handler`는 `ST_WAIT_FRAME_DONE`에서 shot/frame boundary를 무한정 기다림
- 파일: `tdc_gpx_err_handler.vhd`
- 라인: 342-357
- 내용:
  - `s_frame_done_seen_r='1' and i_shot_start='1'`일 때만 IDLE 복귀.
- 문제:
  - recovery는 끝났는데 다음 `shot_start`가 오지 않으면 FSM이 이 상태에 머문다.
- 영향:
  - 에러 처리 후 quiescent 상태가 길어질 때 FSM이 장시간 봉인될 수 있음.
- 권고:
  - 재동기화 timeout 또는 software-release path 추가.

### 6. `face_assembler` shot overrun 시 inactive chip까지 error flag로 마킹할 수 있음
- 파일: `tdc_gpx_face_assembler.vhd`
- 라인: 743-752
- 내용:
  - `s_chip_error_r <= s_chip_error_r or (not s_chip_done_r);`
- 문제:
  - active mask를 고려하지 않고 “done이 아닌 모든 chip”을 error로 올린다.
  - inactive chip도 `not done`이면 error bit가 설 수 있다.
- 영향:
  - `o_chip_error_flags`가 active mask와 논리적으로 어긋날 수 있음.
- 권고:
  - `not s_chip_done_r` 대신 `(s_active_mask_r and not s_chip_done_r)`로 제한.

### 7. `header_inserter`는 invalid `max_hits_cfg="000"`에서 헤더 내부 정보가 서로 모순됨
- 파일: `tdc_gpx_header_inserter.vhd`
- 라인: 296-305
- 내용:
  - low byte에는 raw `s_max_hits_cfg_r`를 그대로 기록한다.
  - `cell_size`는 `others => fn_cell_size_rt(7)`로 계산한다.
- 문제:
  - 헤더에는 `max_hits=0`처럼 보이는데, `cell_size`는 7 기준으로 써진다.
- 영향:
  - 같은 헤더 워드 안에서 self-consistency가 깨짐.
- 권고:
  - invalid config는 상위에서 reject.
  - 또는 header도 동일하게 000→7 alias를 적용해 내부 정합성 유지.

### 8. `face_seq` start validation이 `max_hits_cfg` invalid를 걸러주지 않음
- 파일: `tdc_gpx_face_seq.vhd`
- 라인: 196-203
- 현재 검증:
  - `active_chip_mask == 0`
  - `stops_per_chip < 2`
  - `cols_per_face < 1`
- 문제:
  - `max_hits_cfg="000"`은 reject되지 않는다.
  - downstream들은 이 값을 제각각 alias/clamp/경고 처리한다.
- 영향:
  - invalid runtime config가 fail-closed 되지 않고 모듈마다 다른 방식으로 소비됨.
- 권고:
  - `face_seq`에서 시작 자체를 reject하는 것이 가장 안전.

### 9. `cell_builder`는 `i_max_hits_cfg="000"`을 sim에서는 invalid로 보지만 synth에서는 7로 처리함
- 파일: `tdc_gpx_cell_builder.vhd`
- 라인: 317-320, 434-436
- 내용:
  - `s_max_hits_eff_u <= 7 when i_max_hits_cfg = 000`
  - 동시에 sim assert는 “invalid”라고 경고.
- 문제:
  - simulation과 synthesis 의미가 다르다.
- 영향:
  - TB에서 invalid라고 본 입력이 실기에서는 조용히 7로 처리됨.
- 권고:
  - invalid reject 정책으로 통일하거나, sim/synth 모두 alias 7로 맞추되 주석/상태 레지스터 일치시킬 것.

### 10. `cell_builder`의 `ST_C_QUARANTINE`는 drain marker나 abort가 안 오면 영구 체류함
- 파일: `tdc_gpx_cell_builder.vhd`
- 라인: 567-582
- 내용:
  - 코드 주석대로 escape watchdog을 제거하고 무기한 quarantine 정책으로 변경됨.
- 문제:
  - late stale-beat stall은 막지만, drain marker가 영영 안 오면 collect FSM이 forever quarantine 된다.
- 영향:
  - shot 경계에서 기능은 보호되지만 liveness는 약해짐.
- 권고:
  - 2단계 정책 권장: 일정 시간 quarantine 후 dedicated quarantine_timeout status와 함께 soft escape.
  - 또는 상위가 quarantine stuck를 명확히 관찰할 수 있도록 port 추가.

---

## B. 중간 우선순위

### 11. `header_inserter`의 `ST_DRAIN_LAST` / `ST_ABORT_DRAIN`에는 watchdog이 없음
- 파일: `tdc_gpx_header_inserter.vhd`
- 라인: 464-478
- 문제:
  - downstream이 영구 backpressure면 frame_done 또는 abort drain이 끝나지 않는다.
- 영향:
  - 프레임 종료/abort 완료가 영구 지연 가능.
- 권고:
  - drain watchdog 또는 top-level dead-consumer detect 추가.

### 12. `config_ctrl`의 quasi-static cfg bundle은 여전히 2-FF bundle sync에 의존하며 atomicity가 보장되지 않음
- 파일: `tdc_gpx_config_ctrl.vhd`
- 라인: 850-875
- 코드 주석 자체가 말하는 점:
  - per-bit metastability는 줄였지만, per-bundle atomicity는 source stability에 의존.
- 문제:
  - `cfg` / `cfg_image`가 여러 비트 동시에 바뀌는 구조라면 mixed snapshot 가능성이 남는다.
- 영향:
  - 희박하지만 “이전 값 + 새 값”이 섞인 구성으로 한 shot이 시작될 수 있음.
- 권고:
  - quasi-static도 handshake snapshot으로 옮기거나 shadow-apply 구조 사용.

### 13. `chip_init`의 pending `cfg_write`는 1-depth라 busy-window 다중 pulse가 collapse됨
- 파일: `tdc_gpx_chip_init.vhd`
- 라인: 311-327
- 내용:
  - 첫 pending만 snapshot하고, 같은 busy window에서 뒤이은 pulse는 같은 pending으로 합쳐진다.
- 문제:
  - 여러 개의 서로 다른 cfg_write 요청이 빠르게 오면 마지막/중간 요청이 구분 없이 collapse된다.
- 영향:
  - config write intent 손실.
- 권고:
  - 최소 2-depth queue 또는 reject status + request counter 추가.

### 14. `chip_reg`는 ST_ACTIVE 중 read+write 동시 입력 시 read가 유실됨
- 파일: `tdc_gpx_chip_reg.vhd`
- 라인: 191-205
- 내용:
  - sim assert는 warning만 띄우고, 실제로는 `s_pend_rw_r <= i_start_write`라 write wins.
- 문제:
  - 실제 하드웨어에서 read intent가 silent loss 된다.
- 권고:
  - 동시 입력 reject 정책 명시.
  - 또는 read/write 각자 queue 분리.

### 15. `cmd_arb`도 queue가 1-depth뿐이라 2개 이상 겹친 reg request는 reject sticky만 남기고 손실됨
- 파일: `tdc_gpx_cmd_arb.vhd`
- 라인: 409-425
- 문제:
  - overlap burst를 정확히 흡수하지 못한다.
- 영향:
  - control plane 트래픽이 순간적으로 몰리면 일부 reg request 손실.
- 권고:
  - 소형 FIFO queue 또는 backpressure/ack 프로토콜 추가.

### 16. `cell_builder`의 out-of-range stop_id는 별도 에러 없이 generic `hit_dropped`로만 뭉개짐
- 파일: `tdc_gpx_cell_builder.vhd`
- 라인: 444-455
- 문제:
  - hit overflow와 routing/config 오류가 같은 종류의 “dropped”로만 관찰된다.
- 영향:
  - 디버깅 시 원인 분리가 어렵다.
- 권고:
  - `o_stop_id_invalid` 또는 cause-encoded drop status 추가.

### 17. `chip_ctrl`는 `chip_run`의 detailed timeout cause를 버림
- 파일: `tdc_gpx_chip_ctrl.vhd`
- 라인: 386
- 내용:
  - `o_timeout_cause => open`
- 문제:
  - run timeout이 발생해도 top에서는 cause 분해가 안 된다.
- 영향:
  - timeout 분석 시 raw_busy/ef/burst/flush 구분 불가.
- 권고:
  - cause code를 상위 status로 surface.

### 18. `config_ctrl`는 `cmd_arb`의 per-chip timeout mask도 버림
- 파일: `tdc_gpx_config_ctrl.vhd`
- 라인: 717
- 내용:
  - `o_reg_timeout_mask => open`
- 문제:
  - 어떤 chip이 timeout을 냈는지 mask를 상위에서 직접 못 본다.
- 권고:
  - top/status CSR로 mask를 surface.

---

## C. 유지보수 / 관찰성 / 인터페이스 정합성

### 19. `chip_ctrl`의 `o_err_raw_overflow`는 서로 다른 두 사건을 하나의 sticky로 합침
- 파일: `tdc_gpx_chip_ctrl.vhd`
- 라인: 912-918
- 내용:
  - raw FIFO full에 의한 beat drop과 `PH_RESP_DRAIN` quarantine cap 도달이 같은 포트로 OR됨.
- 문제:
  - “데이터 유실”과 “버스 quarantine escalation”이 관찰상 구분되지 않는다.
- 권고:
  - 원인별 status 분리.

### 20. `header_inserter`의 `s_face_start_collapsed_cnt_r`는 증가하지만 외부로 전혀 보이지 않는 dead observability 상태
- 파일: `tdc_gpx_header_inserter.vhd`
- 라인: 164, 493-499
- 문제:
  - collapse count를 올리지만 port/CSR 연결이 없어 실사용 관찰이 불가능.
- 권고:
  - status export 또는 signal 제거.

### 21. `face_assembler`의 `o_face_abort`는 사실상 dead compatibility port
- 파일: `tdc_gpx_face_assembler.vhd`
- 라인: 119-126, 797-801
- 문제:
  - 포트는 남아 있지만 내부에서 더 이상 assert하지 않는다.
- 영향:
  - 인터페이스를 읽는 사람은 기능이 살아 있다고 오해하기 쉽다.
- 권고:
  - 완전히 제거하거나 deprecated status를 top 주석/CSR에 명확히 반영.

### 22. `cell_builder`의 `s_rt_max_hits_r`는 latched state처럼 보이지만 실제로는 읽히지 않는 dead register
- 파일: `tdc_gpx_cell_builder.vhd`
- 라인: 242, 660, 702
- 문제:
  - 상태 레지스터처럼 선언/갱신되지만 동작에는 사용되지 않는다.
- 영향:
  - 유지보수 시 “runtime max_hits snapshot이 실제로 쓰인다”는 착시를 준다.
- 권고:
  - 실제 소비 경로에 연결하거나 제거.

### 23. `chip_ctrl`의 overflow 관련 주석 일부가 현재 코드와 어긋남
- 파일: `tdc_gpx_chip_ctrl.vhd`
- 라인: 829-832, 915-917 부근
- 문제 예:
  - FIFO depth는 현재 6인데 주석은 여전히 depth=3 기준 설명을 남김.
  - `o_err_raw_overflow` 설명 주석에는 이미 제거된 “chip_run overrun override dropped response” 의미가 남아 있음.
- 영향:
  - 디버깅/유지보수자가 잘못된 mental model을 가질 수 있음.
- 권고:
  - 현재 구현 기준으로 주석 정리.

### 24. `chip_ctrl`의 `o_drain_done`와 `o_run_drain_complete`는 의미가 다르며, 소비자가 잘못 고르면 상태 정렬이 틀어질 수 있음
- 파일: `tdc_gpx_chip_ctrl.vhd`
- 라인: 852-881
- 내용:
  - `o_drain_done`는 downstream raw path에서 control beat가 handshake된 시점.
  - `o_run_drain_complete`는 chip_run 내부 drain 완료 pulse.
- 문제:
  - 이름만 보면 비슷하지만 timing 의미가 다르다.
- 영향:
  - 상위 모듈/상태 집계기가 잘못 선택하면 “실제 drain 완료”와 “외부 전달 완료”를 혼동할 수 있다.
- 권고:
  - 이름을 더 명확히 하거나 주석/CSR 매핑에서 둘을 엄격히 구분.

---

## 우선 수정 순서 제안

### 1차 즉시 수정
1. `chip_run` `ST_DRAIN_FLUSH` pending-stuck watchdog 추가
2. `chip_ctrl` raw/control beat 분리 또는 control beat 우선 보호
3. `cmd_arb` all_done cycle의 new request capture
4. `err_handler` `ST_READ_REG12` watchdog 추가
5. `face_assembler` overrun 시 inactive chip error 마킹 수정
6. `face_seq`에서 `max_hits_cfg=000` reject
7. `header_inserter` / `cell_builder` invalid max_hits 처리 일관화

### 2차 보강
8. `err_handler` `ST_WAIT_FRAME_DONE` 탈출 정책 보강
9. `header_inserter` drain watchdog 추가
10. `config_ctrl` quasi-static cfg bundle snapshot atomic화
11. `chip_init` / `cmd_arb` / `chip_reg` control queue depth 정책 정리
12. drop/error cause 세분화 포트 추가

### 3차 정리
13. dead port / dead register / dead counter 제거
14. stale comment 정리
15. timeout cause / timeout mask / collapse counter surface

---

## 한 줄 결론

현재 코드는 **이전보다 확실히 안정화됐지만**, 아직도 다음 네 축이 남아 있다.

- pending/backpressure가 오래 지속될 때 탈출하지 못하는 FSM 경로
- control beat와 data beat를 동일 완충기에 태워 의미 손실이 가능한 구조
- invalid runtime config를 모듈마다 다르게 해석하는 문제
- 상태/원인 관찰성이 부족하거나 dead interface가 남은 문제

이 네 축만 정리해도 FSM 안정성은 한 단계 더 크게 올라갈 가능성이 높다.
