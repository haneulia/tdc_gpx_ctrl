# TDC-GPX FSM 코드 논리 정합성 점검 (코드만 기준)

기준:
- 분석 대상: 현재 업로드된 VHDL 소스
- 제외: 계약 문서, 요구사항 문서, 데이터시트 해석
- 관점: FSM 상태 전이, 핸드셰이크, timeout, CDC, pending/queue 의미, 상태 종료 조건 정합성

## 총평

현재 업로드본은 이전보다 분명히 안정화된 부분이 많다. 특히 다음은 이미 개선되어 있다.

- `tdc_gpx_err_handler.vhd`: `ST_WAIT_READ` watchdog 추가
- `tdc_gpx_header_inserter.vhd`: non-idle `face_start` pending latch 추가
- `tdc_gpx_cell_builder.vhd`: IFIFO2 timeout 시 synthetic `tlast` EOS 추가
- `tdc_gpx_chip_run.vhd`: drain timeout 경로에서 `range_active` clear 보강
- `tdc_gpx_cmd_arb.vhd`: reg timeout 시작 시점을 “모든 target dispatch 이후”로 이동
- `tdc_gpx_config_ctrl.vhd`: expected IFIFO count를 handshake CDC로 이관
- `tdc_gpx_config_ctrl.vhd`: TDC reset deassert를 동기화

그럼에도 불구하고, 코드만 기준으로 보면 **아직 즉시 보강이 필요한 항목이 20개** 남아 있다.

---

## A. 우선순위 가장 높은 항목

### 1. `chip_run`이 pending response가 계속 남아 있으면 EF/BURST 대기에서 영구 정지할 수 있음
- 파일: `tdc_gpx_chip_run.vhd`
- 위치: 105-115, 519-605
- 내용:
  - `ST_DRAIN_EF1/EF2/BURST`에서 `i_bus_rsp_pending='1'`이면 watchdog을 증가시키지 않고 `null` 처리한다.
  - 의도는 “정상 도착했지만 downstream backpressure 때문에 아직 fire 안 된 응답” 보호다.
  - 문제는 pending이 영구적으로 안 풀리는 경우다. 이 경우 timeout도 안 나고 상태도 안 빠져나온다.
- 연계 위험:
  - `chip_ctrl`가 `s_run_rsp_pending <= (i_bus_rsp_pending or i_s_axis_tvalid)`로 pending을 합성하기 때문에, raw path가 막히면 `chip_run`은 응답이 “계속 pending”인 것으로 본다.
- 개선:
  - `pending 유지 시간` 전용 watchdog 추가
  - 또는 `pending이 일정 시간 지속되면 raw path deadlock으로 별도 timeout cause 부여`

### 2. `face_assembler` real-chip forwarding 중에는 중간 stall watchdog이 없음
- 파일: `tdc_gpx_face_assembler.vhd`
- 위치: 557-570, 599-688
- 내용:
  - `ST_SCAN`에서는 chip data가 아예 안 올 때 timeout으로 blank 전환한다.
  - 그런데 `ST_FORWARD` real-chip 경로에서는 `s_in_tvalid(v_chip_idx)='1'`일 때만 진전이 있고, 중간에 valid가 끊겨도 별도 timeout/blank 전환이 없다.
  - 즉 “첫 beat는 왔는데 chip slice가 중간에서 멈춤” 상황은 영구 대기가 될 수 있다.
- 개선:
  - `ST_FORWARD` real-chip 경로에도 intra-chip watchdog 추가
  - 일정 시간 무진전이면 현재 chip을 error+blank remainder로 마감

### 3. `cmd_arb`는 “dispatch조차 못 하는 상황”에 timeout이 없음
- 파일: `tdc_gpx_cmd_arb.vhd`
- 위치: 238-255, 315-330, 343-356
- 내용:
  - 현재 timeout 카운터는 `s_reg_active_r='1'` 이고 `s_reg_pending_r=0` 일 때만 동작한다.
  - 즉 target chip이 계속 busy여서 dispatch pending이 안 풀리면 timeout이 시작조차 안 된다.
  - 결과적으로 특정 chip busy 고착 시 reg transaction이 영구 active 상태로 남을 수 있다.
- 개선:
  - `dispatch-wait timeout`과 `response timeout`을 분리
  - pending 상태가 너무 오래 지속되면 chip mask 기반 timeout 처리

### 4. `chip_init`의 pending cfg_write는 pulse만 보존하고 config snapshot은 보존하지 않음
- 파일: `tdc_gpx_chip_init.vhd`
- 위치: 149-169, 287-296
- 내용:
  - same-cycle `i_start` + `i_cfg_write_req` 또는 busy 중 `i_cfg_write_req`는 `s_cfg_write_pending_r`로 1-depth 흡수한다.
  - 하지만 실제 payload snapshot은 pending 시점이 아니라 나중에 `ST_OFF`에서 `s_cfg_image_snap_r <= i_cfg_image`로 다시 잡는다.
  - 즉 “요청 시점의 cfg”가 아니라 “나중의 live cfg”가 써질 수 있다.
- 개선:
  - pending latch에 `cfg_image` snapshot도 함께 저장
  - 주석도 현재 동작에 맞게 수정

### 5. `config_ctrl`의 multi-bit config CDC는 atomic하지 않음
- 파일: `tdc_gpx_config_ctrl.vhd`
- 위치: 444-451, 850-867
- 내용:
  - `cfg`, `cfg_image`, reg addr, reg wdata는 2-FF bundle sync만 사용한다.
  - 주석에도 atomicity가 SW hold timing contract에 의존한다고 적혀 있다.
  - per-bit metastability는 줄어들지만 multi-bit bundle의 mixed snapshot 가능성은 남는다.
- 개선:
  - config bundle도 `xpm_cdc_handshake` 또는 shadow+toggle 방식으로 atomic 전송

### 6. `chip_ctrl`의 `PH_RESP_DRAIN` quarantine는 무기한 정지 가능성이 남아 있음
- 파일: `tdc_gpx_chip_ctrl.vhd`
- 위치: 615-650
- 내용:
  - hard cap 15 도달 후 bus가 여전히 busy/pending이면 phase advance를 막고 quarantine에 머문다.
  - stale response leak 방지 측면은 좋아졌지만, 반대로 bus가 영구 고착되면 coordinator도 영구 정지한다.
- 개선:
  - quarantine 2차 watchdog 추가
  - 또는 fatal sticky + reset escalation

### 7. `chip_ctrl` raw FIFO는 depth=3으로 좋아졌지만 beat drop 가능성은 여전히 존재
- 파일: `tdc_gpx_chip_ctrl.vhd`
- 위치: 757-823
- 내용:
  - slot 3개가 모두 valid일 때 새 beat가 오면 drop하고 `s_err_raw_overflow_r`를 세운다.
  - 예전의 “silent corruption”은 아니지만, control beat까지 같은 경로를 타므로 의미 손실이 여전히 가능하다.
- 개선:
  - control beat와 raw data beat를 분리 버퍼링
  - 또는 FIFO depth 추가 확대
  - 최소한 overflow 발생 시 lane stop/recovery policy 명확화

---

## B. 기능은 동작하지만 구조적으로 취약한 항목

### 8. `cell_builder`의 `i_max_hits_cfg=000`은 synth에서 fail-closed가 아님
- 파일: `tdc_gpx_cell_builder.vhd`
- 위치: 422-445, 673-682
- 내용:
  - collect 경로에서는 sim assert만 있고, synth에서는 `hit_count_actual < ('0' & i_max_hits_cfg)`가 항상 false에 가까워져 hit 저장이 비정상화된다.
  - 반면 output 경로 runtime beat lookup은 `others => 7`로 처리한다.
  - 즉 저장 쪽과 출력 포맷 쪽 의미가 서로 어긋난다.
- 개선:
  - `000` 입력을 로컬에서 hard reject하거나 1로 clamp하는 대신 sticky error + no-op 명시

### 9. `cell_builder` quarantine escape 후 late stale beat가 다시 upstream stall을 유발할 수 있음
- 파일: `tdc_gpx_cell_builder.vhd`
- 위치: 312-317, 552-575
- 내용:
  - QUARANTINE은 stale beat를 흡수하지만 두 번째 65K watchdog 후 IDLE로 빠진다.
  - 이후 더 늦은 stale beat가 다시 오면 IDLE에서는 `tready=0`이라 upstream stall이 재발할 수 있다.
- 개선:
  - stale generation source가 명확히 종료될 때까지 quarantine 유지
  - 또는 epoch/shot tag로 stale beat 하드 필터링

### 10. `header_inserter`의 `face_start` pending은 1-depth라 여러 pulse가 collapse됨
- 파일: `tdc_gpx_header_inserter.vhd`
- 위치: 482-500
- 내용:
  - non-idle에서 들어온 `face_start`는 pending latch로 흡수되지만 1-depth다.
  - busy 중 여러 pulse가 와도 결국 하나만 남는다.
- 개선:
  - 큐 깊이를 늘리거나, 최소한 dropped count/sticky 추가

### 11. `header_inserter`는 `cols_per_face<1`을 로컬 reject하지 않고 1로 clamp함
- 파일: `tdc_gpx_header_inserter.vhd`
- 위치: 514-519
- 내용:
  - 상위 `face_seq`가 보통 걸러주지만, 로컬 모듈 자체는 잘못된 geometry를 fail-fast 하지 않는다.
- 개선:
  - 로컬 sticky + assert + start reject

### 12. `header_inserter`는 `max_hits_cfg=000`도 사실상 7로 취급함
- 파일: `tdc_gpx_header_inserter.vhd`
- 위치: 291-300
- 내용:
  - header의 `cell_size` 계산에서 `others => fn_cell_size_rt(7)`로 간다.
  - invalid config가 downstream format상 정상처럼 보일 수 있다.
- 개선:
  - invalid면 header에 에러 표기 또는 face start reject

### 13. `face_assembler`도 degenerate geometry를 로컬 clamp함
- 파일: `tdc_gpx_face_assembler.vhd`
- 위치: 499-513
- 내용:
  - `stops_per_chip<2`면 1 stop으로 clamp
  - `max_hits_cfg=000`이면 runtime beat를 7로 간주
  - 상위가 막는 전제에 기대고 있다.
- 개선:
  - local reject/sticky/error path 추가

### 14. `face_seq`의 `frame_abort_cnt`는 fall-only abort를 세지 않음
- 파일: `tdc_gpx_face_seq.vhd`
- 위치: 275-281, 469-476
- 내용:
  - abort counter는 `s_pipeline_abort`만 보는데, 이 legacy alias는 rise-side abort only다.
  - 반면 fall-side abort는 `s_pipeline_abort_fall`에 따로 존재한다.
  - 결과적으로 fall-only abort는 frame_abort_cnt 통계에서 누락된다.
- 개선:
  - rise/fall abort 카운터 분리 또는 both-side OR로 집계

### 15. `face_seq`의 abort gating은 rise-only alias를 써서 의미가 비대칭적임
- 파일: `tdc_gpx_face_seq.vhd`
- 위치: 469-476, 554-571
- 내용:
  - `s_pipeline_abort`는 rise-only alias인데, `s_shot_start_gated`와 `o_face_start_gated`는 이를 사용한다.
  - fall-only abort는 `frame_done_both`에는 반영되지만 gating에는 직접 반영되지 않는다.
- 개선:
  - gating 의도를 명확히 해서 `abort_any`/`abort_rise`를 분리 사용

---

## C. 인터페이스/아키텍처 정합성 측면의 문제

### 16. `chip_reg`는 ST_ACTIVE 중 read/write 동시 입력 시 write가 read를 덮어씀
- 파일: `tdc_gpx_chip_reg.vhd`
- 위치: 191-196
- 내용:
  - `s_pend_rw_r <= i_start_write`라서 same-cycle read+write면 write가 우선된다.
  - 사용자가 정말 둘 다 올릴 수 있다면 read intent는 소실된다.
- 개선:
  - simultaneous read/write 금지 assert
  - 또는 2-entry typed queue

### 17. `chip_reg`의 내부 pending auto-start는 `chip_ctrl` phase mux와 구조적으로 어긋남
- 파일: `tdc_gpx_chip_reg.vhd`, `tdc_gpx_chip_ctrl.vhd`
- 위치:
  - `chip_reg`: 133-151
  - `chip_ctrl`: 436-439, 572-581, 607-610
- 내용:
  - `chip_reg`는 ST_OFF에서 내부 pending이 있으면 스스로 active로 들어간다.
  - 하지만 bus mux는 `PH_REG`일 때만 reg request를 외부 bus로 연결한다.
  - 정상 사용에서는 대체로 맞아떨어지지만, local pending이 phase 밖에서 살아 있으면 phantom timeout 구조가 된다.
- 개선:
  - reg sub-FSM의 auto-start를 coordinator dispatch pulse로만 허용
  - 또는 PH_REG 외 auto-start 금지 assert

### 18. `stop_cfg_decode`는 running-total 계약에 매우 강하게 의존하고 synth 보호가 약함
- 파일: `tdc_gpx_stop_cfg_decode.vhd`
- 위치: 10-19, 62-84, 93-129
- 내용:
  - 입력 beat는 delta가 아니라 running total이어야 하며 monotonic해야 한다.
  - 실제 RTL은 overwrite semantics이고, 감소 검출은 sim-only assert다.
  - synth에서는 잘못된 source가 들어와도 조용히 잘못된 expected count를 만든다.
- 개선:
  - synth용 monotonic violation sticky 추가
  - protocol versioning 또는 tag 부여

### 19. `err_handler`는 `ST_WAIT_FRAME_DONE` 진입 직전에 이미 지나간 frame_done pulse를 기억하지 않음
- 파일: `tdc_gpx_err_handler.vhd`
- 위치: 310-315, 342-357
- 내용:
  - recovery stable-low가 성립한 뒤에야 `ST_WAIT_FRAME_DONE`으로 들어간다.
  - 만약 그 직전 cycle에 frame_done이 지나갔다면, 다음 frame_done까지 기다리게 된다.
  - 기능상 치명적이진 않지만 clear 타이밍이 한 frame 늦어질 수 있다.
- 개선:
  - frame_done_seen latch를 state 밖에서 edge-capture

### 20. `header_inserter`는 abort가 queued restart를 취소함
- 파일: `tdc_gpx_header_inserter.vhd`
- 위치: 488-500, 575-592
- 내용:
  - busy 중 들어온 `face_start`는 pending-latch되지만, `face_abort`가 오면 그 pending을 강제로 지운다.
  - same-cycle `face_start` + `face_abort`도 abort가 이긴다.
- 개선:
  - 정책적으로 맞다면 status에 “restart canceled by abort”를 남기고,
  - 아니라면 pending restart를 유지

---

## D. 유지보수 / 관찰성 문제

### 21. `face_assembler`의 `o_face_abort` 포트는 사실상 dead port
- 파일: `tdc_gpx_face_assembler.vhd`
- 위치: 119-126, 770-774
- 내용:
  - 포트는 남아 있지만 architecture에서 사실상 영구 0이다.
  - 외부 사용자가 의미 있다고 생각하면 오해를 만든다.
- 개선:
  - 제거하거나 이름에 `_deprecated` 반영

### 22. `chip_ctrl`의 `o_drain_done`와 `o_run_drain_complete` 의미가 달라 오용 위험이 있음
- 파일: `tdc_gpx_chip_ctrl.vhd`
- 위치: 832-861
- 내용:
  - 하나는 downstream handshake 시점, 하나는 chip_run 내부 drain 종료 시점이다.
  - 현재는 둘 다 존재해서 좋아졌지만, 소비자가 잘못 고르면 상태 정렬이 어긋날 수 있다.
- 개선:
  - 포트명 더 강하게 구분하거나 consumer 쪽 사용처 점검

### 23. `cell_builder`의 `o_shot_dropped`는 여러 원인을 한 신호로 합침
- 파일: `tdc_gpx_cell_builder.vhd`
- 위치: 406-413, 535-547, 570-573
- 내용:
  - no-free-buffer, DROP timeout, QUARANTINE escape가 모두 같은 pulse에 합쳐진다.
  - SW/상위 FSM이 원인별 대응을 하기 어렵다.
- 개선:
  - cause-bit 분리

### 24. `cmd_arb`는 simultaneous read+write request를 명시적으로 거부하지 않음
- 파일: `tdc_gpx_cmd_arb.vhd`
- 위치: 230-236
- 내용:
  - read/write가 동시에 오면 write wins다.
  - ambiguous command가 silent precedence로 처리된다.
- 개선:
  - assert + reject sticky

---

## 우선 수정 권장 순서

1. `chip_run` pending-freeze watchdog 추가
2. `face_assembler` ST_FORWARD real-chip stall watchdog 추가
3. `cmd_arb` dispatch-wait timeout 추가
4. `chip_init` pending cfg snapshot 보존
5. `config_ctrl` config bundle CDC atomic화
6. `chip_ctrl` PH_RESP_DRAIN quarantine 2차 탈출 경로 추가
7. raw FIFO에서 control/data 분리 또는 depth 확대
8. `max_hits_cfg=000`, degenerate geometry를 downstream에서도 fail-closed 처리
9. `face_seq` abort 의미를 rise/fall 기준으로 재정의
10. 관찰성 신호(cause 분리, deprecated 제거) 정리

---

## 최종 결론

현재 코드는 “예전보다 훨씬 나아졌지만, 아직 완전 안정화라고 말하긴 이른 상태”다.

핵심 미해결 축은 네 가지다.

1. **pending/backpressure 영구화 시 탈출하지 못하는 FSM 경로**
2. **mid-stream stall에 대한 watchdog 부재**
3. **config/request snapshot의 atomicity 부족**
4. **invalid runtime config를 fail-closed 하지 않는 downstream 모듈들**

이 네 축을 먼저 정리하면 전체 FSM 안정성은 크게 올라갈 가능성이 높다.
