# TDC-GPX FSM / 상태성 로직 코드 점검 보고서 (코드만 기준)

작성 기준:
- **업로드된 RTL 코드만** 근거로 분석함.
- 계약 문서 / 요구 문서 / 데이터시트 해석은 판단 근거에서 제외함.
- 분석 대상은 FSM, 상태 전이, timeout, pending latch, completion semantics, recovery path, queue depth, CDC/clear semantics 중 **코드의 논리 정합성에 직접 영향을 주는 부분**임.

---

## 총평

현재 업로드본은 이전 라운드들보다 **확실히 안정화가 많이 진행된 상태**다.
특히 다음은 실제로 좋아졌다.

- `face_seq`의 shot/packet start one-shot 정리
- `chip_run`의 pending-stuck watchdog 보강
- `cmd_arb`의 dispatch-wait watchdog 보강
- `header_inserter`의 drain watchdog 추가
- `cell_builder`의 IFIFO2-timeout synthetic `tlast` 추가
- `config_ctrl`의 TDC→AXI status CDC 보강
- `chip_ctrl`의 raw/control drop 원인 분리 노출

하지만, 코드만 놓고 보면 아직 **"FSM 안정화 완료"라고 부르기엔 이른 상태**다.
이번 기준으로 **총 24개 항목**을 정리했다.

- 높음: 9개
- 중간: 9개
- 낮음/유지보수/관찰성: 6개

핵심 미해결 축은 아래 5가지다.

1. **soft_clear 경로가 top에서 실제로 연결되지 않음**
2. **raw FIFO 완전 포화 시 control beat도 drop 가능**
3. **stuck-bus 시 PH_RESP_DRAIN이 사실상 dead-end quarantine**
4. **일부 faulted completion이 downstream에는 normal completion처럼 보임**
5. **여러 모듈이 여전히 1-depth/2-depth pending latch에 의존하여 overlap을 collapse/reject 함**

---

## 우선순위 최상위 (즉시 수정 권고)

### 1. `s_err_soft_clear`가 top에서 실제로 구동되지 않는다
- 파일/라인:
  - `tdc_gpx_top.vhd:193`
  - `tdc_gpx_top.vhd:508`
  - `tdc_gpx_top.vhd:872`
  - `tdc_gpx_top.vhd:1044`
  - `tdc_gpx_csr_pipeline.vhd:104-111`
  - `tdc_gpx_csr_pipeline.vhd:645-650`
- 내용:
  - top에 `s_err_soft_clear` 신호가 선언되어 있고 여러 모듈에 전달되지만, **top 내부 어디에서도 이 신호를 실제로 생성/할당하지 않는다**.
  - 반면 `status_agg`, `config_ctrl`, `chip_reg`, top-level 일부 sticky clear 로직은 이 신호가 올 것이라 가정하고 작성되어 있다.
  - `csr_pipeline`가 내보내는 command output은 `start/stop/soft_reset/force_reinit/cfg_write`뿐이고, `soft_clear` 출력은 없다.
- 영향:
  - 코드 주석/정책상 `SOFT-CLEAR`여야 하는 항목들이 실제 동작에서는 **hard reset 없이는 안 지워지는 상태**가 된다.
  - 상태 수명(category) 설계 의도와 실제 동작이 어긋난다.
- 수정 방향:
  - `csr_pipeline`에 명시적인 `o_err_soft_clear` pulse 추가 또는 CTL2 spare bit에 soft_clear 할당.
  - top에서 `s_err_soft_clear`를 실제로 구동하고, 현재 soft_clear consumer 전부가 같은 pulse semantics를 따르도록 통일.

### 2. `chip_ctrl` raw FIFO는 완전 포화되면 control beat도 drop될 수 있다
- 파일/라인:
  - `tdc_gpx_chip_ctrl.vhd:926-1044`
- 내용:
  - reserve 정책이 강화되어 data beat가 마지막 3 slot을 먹지 못하게 했지만,
  - **FIFO가 control beat들로 이미 가득 찬 경우**에는 control beat도 결국 drop될 수 있다.
  - 코드도 이를 인정하고 `s_err_raw_ctrl_drop_r` sticky를 세운다.
- 영향:
  - 단순 data loss가 아니라 `drain_done`, `ififo1_done` 같은 **FSM 의미 신호 자체가 사라질 수 있다**.
  - downstream completion 정렬이 깨질 수 있다.
- 수정 방향:
  - 최종적으로는 raw data와 control beat를 분리하거나,
  - 최소한 control beat 전용 FIFO/credit를 두는 것이 가장 안전하다.

### 3. `chip_ctrl`의 `PH_RESP_DRAIN`은 stuck bus에서 사실상 dead-end quarantine이다
- 파일/라인:
  - `tdc_gpx_chip_ctrl.vhd:745-809`
  - `tdc_gpx_chip_ctrl.vhd:816-835`
- 내용:
  - hard cap 이후 bus가 계속 busy/pending이면 `PH_RESP_DRAIN`에 머물며 quarantine 된다.
  - 이후 soft reset도 다시 `PH_RESP_DRAIN`로 들어가므로, **자가 회복 경로가 닫혀 있다**.
  - `force_reinit`만 bypass escape로 남겨뒀는데, 그건 SW가 외부 flush를 했다는 강한 전제에 기대는 우회로다.
- 영향:
  - stuck bus 상황에서 FSM이 스스로 회복하지 못한다.
  - 운영 관점에서는 “status는 보이지만 시스템은 계속 묶이는” 상태가 된다.
- 수정 방향:
  - 현재 정책을 유지하더라도, `bus_fatal` 후 SW가 사용할 수 있는 **명시적 recovery state machine path**를 하나 더 두는 편이 안전하다.

### 4. `chip_run`은 drain mismatch를 발견해도 `drain_done`을 정상 completion처럼 내보낸다
- 파일/라인:
  - `tdc_gpx_chip_run.vhd:527-560`
- 내용:
  - fallback completion 경로에서 expected vs actual drain count mismatch를 감지하면 `s_err_drain_mismatch_r`와 `s_drain_done_faulted_r`를 세운다.
  - 그러나 동시에 **항상** `s_drain_done_r <= '1'`도 올린다.
- 영향:
  - downstream cell_builder / face_assembler는 정상 completion처럼 진행하고,
  - supervisor-level SW만 별도 faulted bit를 보고 문제를 알아야 한다.
  - 즉 **faulted completion이 logical success와 분리되지 않는다**.
- 수정 방향:
  - 최소한 supervisor path 외에도 downstream에 faulted completion semantic을 별도로 주거나,
  - mismatch 시 synthetic blank/truncate 같은 더 명시적인 degraded path를 검토.

### 5. `stop_cfg_decode`의 orphan stop-event 검출은 “reset 후 첫 shot 전”만 잡는다
- 파일/라인:
  - `tdc_gpx_stop_cfg_decode.vhd:103-123`
- 내용:
  - `s_shot_ever_seen_r`가 첫 `shot_start` 이후 영구 1이 되므로,
  - 그 이후 **shot과 shot 사이 공백 구간에 잘못 들어온 stop event**는 orphan으로 잡히지 않는다.
- 영향:
  - upstream format drift가 “처음 한 번만 아니면” 조용히 expected count를 오염시킬 수 있다.
- 수정 방향:
  - orphan 판단 기준을 `ever seen`이 아니라 “현재 shot window 안인지”로 바꾸는 것이 더 정확하다.

### 6. `cell_builder` QUARANTINE 강제 탈출은 pending shot까지 버리고, 늦은 stale beat는 다시 stall을 만들 수 있다
- 파일/라인:
  - `tdc_gpx_cell_builder.vhd:356-367`
  - `tdc_gpx_cell_builder.vhd:648-671`
  - `tdc_gpx_cell_builder.vhd:716-718`
- 내용:
  - QUARANTINE escape 시 버퍼를 강제로 free하고 IDLE로 돌아가며 `s_shot_pending_r <= '0'`로 queued next shot도 버린다.
  - 이후 `s_post_escape_cnt_r` 동안만 stale beat를 흡수한다.
  - 이 window 뒤에 더 늦은 stale beat가 오면, IDLE의 `tready=0`와 다시 충돌할 수 있다.
- 영향:
  - availability는 살리지만, recovery가 완전하지 않다.
  - synthetic recovery 뒤에 다시 upstream stall이 재발할 수 있다.
- 수정 방향:
  - post-escape 흡수창을 시간 기반이 아니라 marker/idle-detect 기반으로 바꾸거나,
  - quarantine escape 이후 별도 “discard-until-marker” 상태를 두는 편이 더 안전하다.

### 7. `header_inserter` drain watchdog은 deadlock을 막는 대신 beat를 버린다
- 파일/라인:
  - `tdc_gpx_header_inserter.vhd:519-565`
- 내용:
  - `ST_DRAIN_LAST`, `ST_ABORT_DRAIN`에서 watchdog 만료 시 `tvalid`를 강제로 내리고 IDLE 복귀한다.
- 영향:
  - wedged sink에서 FSM은 안 멈추지만,
  - 마지막 beat / abort drain beat는 **실제로 유실**될 수 있다.
  - 즉 liveness를 위해 completeness를 희생하는 구조다.
- 수정 방향:
  - 현재 정책 유지 가능하나, 상위에서 반드시 이 timeout sticky를 frame-discard 조건으로 취급해야 안전하다.

### 8. `face_assembler`는 다음 shot를 1비트 pending으로만 보관한다
- 파일/라인:
  - `tdc_gpx_face_assembler.vhd:560-561`
  - `tdc_gpx_face_assembler.vhd:762-764`
  - `tdc_gpx_face_assembler.vhd:818-825`
- 내용:
  - overlap이 여러 번 와도 `s_shot_pending_r`는 결국 1비트라 다음 shot 하나만 남는다.
  - `o_shot_overrun_count`는 늘어나지만, **실제 queue depth는 1**이다.
- 영향:
  - shot burst overlap이 심해지면 “몇 번 겹쳤는지”는 알 수 있어도, 실제로는 하나만 예약된다.
- 수정 방향:
  - queue depth를 늘리거나, pending count 기반 정책으로 바꾸는 편이 더 명확하다.

### 9. `chip_ctrl`의 live `stopdis_override`와 `force_reinit`는 FSM 상태 가정을 깨뜨릴 수 있는 강한 escape hatch다
- 파일/라인:
  - `tdc_gpx_chip_ctrl.vhd:823-835`
  - `tdc_gpx_chip_ctrl.vhd:888-911`
- 내용:
  - `stopdis_override`는 mid-run에 바로 핀 상태를 바꾼다.
  - `force_reinit`는 bus flush를 SW가 외부에서 했다고 가정하고 `PH_RESP_DRAIN`을 우회해 `PH_INIT`로 진입한다.
- 영향:
  - debug/field recovery용으로는 유용하지만,
  - 정상 운용에서 잘못 쓰면 핀 상태와 내부 FSM 의미가 어긋난다.
- 수정 방향:
  - production build에서는 mask 하거나, 최소한 status/CSR에서 “dangerous command”로 분리 표기.

---

## 중간 우선순위 (구조 취약 / 의미 충돌 / interface drift)

### 10. `stop_cfg_decode`는 running-total overwrite semantics에 매우 강하게 의존한다
- 파일/라인:
  - `tdc_gpx_stop_cfg_decode.vhd:10-20`
  - `tdc_gpx_stop_cfg_decode.vhd:157-166`
- 내용:
  - stop event beat마다 expected count를 누적하는 게 아니라 **overwrite**한다.
  - 즉 upstream이 “현재까지의 running total”을 보낸다는 가정이 깨지면 바로 오염된다.
- 영향:
  - monotonic decrease만 안 나면, 잘못된 format drift가 조용히 흘러갈 수 있다.
- 수정 방향:
  - beat sequence number / end-of-shot marker / explicit final-count valid 같은 메타정보가 있으면 훨씬 안전하다.

### 11. `header_inserter`의 `face_start` pending queue는 2-depth이고, 그 이상은 collapse한다
- 파일/라인:
  - `tdc_gpx_header_inserter.vhd:179-185`
  - `tdc_gpx_header_inserter.vhd:602-610`
  - `tdc_gpx_header_inserter.vhd:619-625`
- 내용:
  - legal upstream에서는 충분하다는 주석이 있지만,
  - 실제로는 2-depth를 넘는 pulse burst는 `s_face_start_collapsed_cnt_r`로만 남고 queue는 잃는다.
- 영향:
  - robustness margin은 있지만 무한하지 않다.
- 수정 방향:
  - 현재는 observability가 있으므로 중간 우선순위. burst tolerance를 더 키우려면 counter/FIFO화 필요.

### 12. `chip_init`의 busy-window cfg_write는 1-depth coalescing이다
- 파일/라인:
  - `tdc_gpx_chip_init.vhd:321-340`
- 내용:
  - busy 상태에서 들어오는 cfg_write는 한 장의 snapshot만 보관하고, 2개 이상이면 `s_cfg_write_coalesced_r`만 세운다.
- 영향:
  - 중간 업데이트가 사라질 수 있다.
- 수정 방향:
  - cfg_write를 truly queued transaction으로 만들지 않을 거라면, SW contract를 더 강하게 두고 sticky를 반드시 읽게 해야 한다.

### 13. `cmd_arb`의 overlap queue는 1-depth라 2개 이상 겹치면 reject된다
- 파일/라인:
  - `tdc_gpx_cmd_arb.vhd:460-491`
- 내용:
  - reg request overlap은 1개만 큐잉되고, 그 이상은 `s_reg_rejected_r`가 선다.
- 영향:
  - caller가 burst reg access를 시도하면 transaction loss가 생긴다.
- 수정 방향:
  - 현재 구조 유지 시 SW가 반드시 done/idle handshaking을 지키게 해야 한다.

### 14. `chip_reg`도 busy 중 overlap queue가 1-depth뿐이다
- 파일/라인:
  - `tdc_gpx_chip_reg.vhd:191-229`
- 내용:
  - pending slot이 이미 차 있으면 `s_err_req_overflow_r`만 세우고 추가 요청을 잃는다.
- 영향:
  - local reg access caller도 burst/overlap에 약하다.
- 수정 방향:
  - `cmd_arb`와 동일하게 queue depth 또는 caller contract를 명확히 고정.

### 15. `cmd_arb`는 동시 read/write 입력 시 write-wins이다
- 파일/라인:
  - `tdc_gpx_cmd_arb.vhd:245-258`
- 내용:
  - sticky를 세워서 과거보다 낫지만, 여전히 read intent는 버려진다.
- 영향:
  - caller contract 위반 시 결과가 deterministic하긴 하나, semantic loss는 남아 있다.
- 수정 방향:
  - 가능하면 reject-one-shot로 바꾸거나, read/write 동시 입력을 illegal로 강제하는 것이 명확하다.

### 16. `chip_reg`도 동시 read/write 입력 시 write-wins이다
- 파일/라인:
  - `tdc_gpx_chip_reg.vhd:196-229`
- 내용:
  - cmd_arb와 같은 ambiguity가 local reg path에도 존재한다.
- 영향:
  - layer 간 정책은 일치하지만, ambiguity 자체는 여전히 남는다.
- 수정 방향:
  - cmd_arb와 chip_reg 둘 다 같은 reject policy로 통일하는 것이 깔끔하다.

### 17. `o_drain_done`와 `o_run_drain_complete`의 의미가 다르다
- 파일/라인:
  - `tdc_gpx_chip_ctrl.vhd:1056-1085`
- 내용:
  - `o_drain_done`는 raw control beat가 downstream handshake된 시점,
  - `o_run_drain_complete`는 chip_run 내부 drain 완료 rising edge다.
- 영향:
  - 소비자가 잘못된 신호를 고르면 상태 정렬이 어긋난다.
- 수정 방향:
  - 인터페이스 이름을 더 분명히 하거나, 한쪽을 내부 전용으로 숨기는 것도 방법이다.

### 18. `csr_pipeline`의 `o_cmd_start`는 포트 주석과 달리 진짜 1-cycle pulse가 아니다
- 파일/라인:
  - `tdc_gpx_csr_pipeline.vhd:104-111`
  - `tdc_gpx_csr_pipeline.vhd:625-650`
- 내용:
  - 포트 설명은 `1-clk` pulse처럼 보이지만,
  - 실제 구현은 `s_start_pending_r and s_cdc_all_idle_ff(1)`라 **accepted될 때까지 반복적으로 올라갈 수 있는 level-ish 출력**이다.
- 영향:
  - 현재 `face_seq`는 견디도록 바뀌었지만, 인터페이스 의미가 문서/포트 주석과 어긋난다.
- 수정 방향:
  - 포트 주석을 수정하거나, 진짜 one-shot emitter로 바꾸는 편이 좋다.

### 19. leaf 모듈들이 invalid runtime config를 여전히 local clamp/alias로 정규화한다
- 파일/라인:
  - `tdc_gpx_face_assembler.vhd:570-584`
  - `tdc_gpx_header_inserter.vhd:343-360`
  - `tdc_gpx_header_inserter.vhd:640-645`
  - `tdc_gpx_cell_builder.vhd:351-354`
  - `tdc_gpx_cell_builder.vhd:776-790`
- 내용:
  - 상위 `face_seq`가 geometry를 reject하긴 하지만,
  - 하위 모듈들은 여전히 `stops_per_chip<2`, `cols_per_face<1`, `max_hits_cfg=000` 등을 local clamp/alias한다.
- 영향:
  - 현재 top-level 흐름에서는 대체로 안전하지만,
  - 모듈 재사용/우회 연결 시 fail-fast 대신 silent normalization이 일어난다.
- 수정 방향:
  - 상/하위 정책을 하나로 통일하는 편이 좋다. 보통은 상위 reject + 하위 assert가 가장 깔끔하다.

---

## 낮음 / 유지보수 / 관찰성 이슈

### 20. top-level sticky clear 정책이 일관되지 않다
- 파일/라인:
  - `tdc_gpx_top.vhd:951-968`
  - `tdc_gpx_top.vhd:1015-1038`
  - `tdc_gpx_top.vhd:1041-1052`
  - `tdc_gpx_top.vhd:1062-1069`
  - `tdc_gpx_pkg.vhd:294-324`
- 내용:
  - 어떤 sticky는 hard-reset-only, 어떤 sticky는 soft_clear 대상, 어떤 것은 last-tx 성격이다.
  - 설계 철학은 pkg에 잘 적혀 있지만, 실제 top 구현은 mixed policy다.
  - 게다가 상위 1번 문제(soft_clear 미배선) 때문에 일부 intended category가 현실에서는 동작하지 않는다.
- 영향:
  - SW 입장에서 status lifetime이 읽기 어렵다.
- 수정 방향:
  - status field마다 category를 코드/CSR 문서에 강제 표기하고, clear path를 실제 배선까지 완료해야 한다.

### 21. `o_face_abort`는 사실상 dead compatibility port다
- 파일/라인:
  - `tdc_gpx_face_assembler.vhd:127-144`
  - `tdc_gpx_face_assembler.vhd:911-915`
  - `tdc_gpx_top.vhd:749-755`
- 내용:
  - face_assembler 내부에서 더 이상 assert되지 않고, top에서는 `open` 처리된다.
- 영향:
  - 큰 기능 버그는 아니지만, 인터페이스 유지보수에 혼선을 준다.
- 수정 방향:
  - 정말 보존이 필요하면 deprecated 표시를 더 강하게 하고, 아니면 다음 인터페이스 정리 때 제거.

### 22. `shot_drop_any` / `slice_timeout_any`는 sticky가 아니라 pulse성 summary다
- 파일/라인:
  - `tdc_gpx_top.vhd:1054-1057`
- 내용:
  - per-chip pulse들을 OR해서 한 cycle summary bit를 만든다.
- 영향:
  - polling SW는 놓칠 수 있다.
- 수정 방향:
  - summary도 sticky 또는 run-scoped counter로 두는 편이 관찰성 측면에서 낫다.

### 23. legacy `o_err_raw_overflow`는 여전히 두 원인을 합쳐서 보여준다
- 파일/라인:
  - `tdc_gpx_chip_ctrl.vhd:1126-1135`
- 내용:
  - distinct 원인 포트(`o_err_raw_drop`, `o_err_drain_cap`)는 이미 있는데,
  - legacy `o_err_raw_overflow`는 둘을 OR로 합친다.
- 영향:
  - 구형 소비자가 이 비트만 보면 root cause를 정확히 못 본다.
- 수정 방향:
  - 새 소비자는 distinct cause만 쓰게 하고, legacy bit는 점진적으로 축소.

### 24. `chip_init` coalesced sticky도 결국 historical 성격이 강하다
- 파일/라인:
  - `tdc_gpx_chip_init.vhd:160`
  - `tdc_gpx_chip_init.vhd:338`
  - `tdc_gpx_chip_init.vhd:354`
- 내용:
  - busy-window cfg_write coalescing을 잘 보이게 해주긴 하지만,
  - clear 정책이 soft_clear 기반으로 연결돼 있지 않아 사실상 historical에 가깝다.
- 영향:
  - 운용 중 “이번 run에서만 본 것인지, 옛날 것인지” 구분이 약하다.
- 수정 방향:
  - top soft_clear 연결 완료 후 run/category 정책을 다시 정리하는 것이 좋다.

---

## 현재 코드에서 “이미 좋아진 부분”

아래는 이번 업로드본에서 실제로 개선된 부분이다.

1. `face_seq`
   - `packet_start` / `shot_start_raw` 처리 정리가 잘 되어 있다.
   - `tdc_gpx_face_seq.vhd:469-506`

2. `chip_run`
   - EF/BURST/FLUSH pending-stuck watchdog이 들어가 있어, 예전형 무한 대기 문제는 많이 줄었다.
   - `tdc_gpx_chip_run.vhd:587-607`, `634-647`, `681-746`

3. `cmd_arb`
   - dispatch-wait watchdog과 same-cycle all_done/new-request capture가 보강되었다.
   - `tdc_gpx_cmd_arb.vhd:286-307`, `324-358`

4. `header_inserter`
   - non-idle `face_start`를 그냥 버리지 않고 pending queue로 흡수한다.
   - `tdc_gpx_header_inserter.vhd:602-625`

5. `cell_builder`
   - IFIFO2 timeout 시 synthetic `tlast`를 만들어 상위 영구대기를 막는다.
   - `tdc_gpx_cell_builder.vhd:862-883`

6. `config_ctrl`
   - TDC→AXI status 신호들을 2-FF sync로 정리했다.
   - `tdc_gpx_config_ctrl.vhd:828-881`

---

## 파일별 수정 우선순위

### 1순위
- `tdc_gpx_top.vhd`
- `tdc_gpx_csr_pipeline.vhd`
- `tdc_gpx_chip_ctrl.vhd`
- `tdc_gpx_chip_run.vhd`
- `tdc_gpx_stop_cfg_decode.vhd`

### 2순위
- `tdc_gpx_cell_builder.vhd`
- `tdc_gpx_header_inserter.vhd`
- `tdc_gpx_face_assembler.vhd`
- `tdc_gpx_cmd_arb.vhd`
- `tdc_gpx_chip_reg.vhd`
- `tdc_gpx_chip_init.vhd`

### 3순위
- `tdc_gpx_face_seq.vhd`
- `tdc_gpx_pkg.vhd`

---

## 한 줄 결론

현재 코드는 **“멈추지 않게 만드는 방어”는 많이 들어갔지만, 일부 faulted 경로가 여전히 정상 completion과 같은 형태로 흘러가고, pending/queue depth/clear wiring 쪽에서 구조적 취약점이 남아 있는 상태**다.

특히 이번 업로드본의 최우선 실제 버그는 **top의 `s_err_soft_clear` 미배선**이고,
그 다음이 **`chip_ctrl`의 control beat drop 가능성**과 **`PH_RESP_DRAIN` stuck-bus dead-end**다.
