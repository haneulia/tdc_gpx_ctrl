# TDC-GPX FSM 코드 전수 점검 보고서 (코드 논리 기준)

기준:
- **오직 현재 업로드된 RTL/VHDL 코드만** 기준으로 검토함.
- 계약 문서, 요구 문서, 데이터시트 해석은 판단 근거에서 제외함.
- 아래 항목은 `문제 확률이 높은 실제 결함`, `가용성/복구성 리스크`, `구조적 가정`, `관찰성/유지보수 부채`를 함께 정리한 것임.

## 총평

현재 업로드본은 이전 라운드에서 지적됐던 문제들이 많이 정리되어 있습니다. 특히 아래는 **고쳐진 것으로 보이는 항목**입니다.

- `face_seq`의 `packet_start/shot_start`가 내부 edge-detect + registered one-shot으로 정리됨 (`tdc_gpx_face_seq.vhd:447-507`, `598-647`)
- `config_ctrl`의 cfg/cfg_image/expected_ififo 경계가 `xpm_cdc_handshake` 기반으로 재구성됨 (`tdc_gpx_config_ctrl.vhd:939-980` 이하)
- `err_handler`에 Reg-read watchdog/entry watchdog이 있음 (`tdc_gpx_err_handler.vhd:261-331`)
- `header_inserter`의 drain 상태에 watchdog이 추가됨 (`tdc_gpx_header_inserter.vhd:520-548`)
- `chip_run`의 pending-stuck watchdog / flush pending watchdog이 보강됨 (`tdc_gpx_chip_run.vhd:618-720`)
- `cell_builder`의 IFIFO2-timeout synthetic `tlast`가 들어감 (`tdc_gpx_cell_builder.vhd:823-855`)
- `chip_init`의 pending cfg snapshot이 요청 시점 이미지로 고정됨 (`tdc_gpx_chip_init.vhd:174-183`, `194-203`, `321-340`)
- `chip_ctrl` raw FIFO가 6-depth + control-beat reserve 정책으로 강화됨 (`tdc_gpx_chip_ctrl.vhd:853-920`)

하지만 **FSM/경계조건/복구성 관점에서 아직 남은 문제는 적지 않습니다.**

---

## A. 고신뢰 문제 / 우선 수정 권고

### 1) `chip_ctrl`의 `PH_RESP_DRAIN`이 영구 quarantine에 빠질 수 있음
- 위치: `tdc_gpx_chip_ctrl.vhd:661-725`
- 현상:
  - stale response drain 중 `i_bus_busy='1'` 또는 `i_bus_rsp_pending='1'`이 오래 유지되면 `s_drain_cnt_r`가 15에서 포화된 뒤 **계속 `PH_RESP_DRAIN`에 머무름**.
  - Round 11 코드에서 의도적으로 “강제 PH_INIT 재진입”을 제거했고, `s_err_drain_cap_r`만 세운 채 **영구 quarantine**로 남음 (`694-700`).
  - 같은 파일 하단의 soft reset 처리도 결국 다시 `PH_RESP_DRAIN`으로 들어가므로 (`717-725`), **버스 stuck 상태에서는 soft reset만으로는 복구되지 않음**.
- 영향:
  - 해당 chip coordinator가 사실상 영구 busy 상태가 되어 전체 파이프라인을 막을 수 있음.
- 권고:
  - `hard quarantine` 정책은 유지하더라도, 최소한 **외부에서 회복 가능한 별도 escape path**(예: 관리자 전용 force-idle / force-reinit / bus_flush_ack)를 마련해야 함.

### 2) `chip_ctrl` raw FIFO는 여전히 beat drop 가능, control beat도 완전 보장 아님
- 위치: `tdc_gpx_chip_ctrl.vhd:853-920`
- 현상:
  - data beat는 2개 이상 free slot이 있을 때만 넣고, control beat는 1개 이상 free slot이면 넣는 구조로 개선됨.
  - 하지만 FIFO가 **이전에 들어온 control beat들까지 포함해 완전히 포화**되면 (`v_free=0`) control beat도 drop 가능.
  - 코드도 이 경우 `s_err_raw_overflow_r <= '1'`만 세움 (`904-919`).
- 영향:
  - `drain_done`, `ififo1_done` 같은 제어 경계 beat가 drop되면 상위 FSM 완료 의미가 깨질 수 있음.
- 권고:
  - data/control 경로를 분리하거나,
  - control beat 전용 별도 소형 FIFO를 두거나,
  - control beat는 절대 drop되지 않도록 강제 backpressure 구조로 분리하는 것이 안전함.

### 3) TDC-domain 상태가 AXI-domain status로 **무동기 직결**되는 경로가 남아 있음
- 위치:
  - `tdc_gpx_config_ctrl.vhd:639-657` (`o_run_timeout_cause`)
  - `tdc_gpx_config_ctrl.vhd:663-672` (`o_init_cfg_coalesced_mask`, `o_cmd_collision_mask`)
  - `tdc_gpx_top.vhd:899-905`
  - `tdc_gpx_top.vhd:370-408` + `tdc_gpx_csr_pipeline.vhd:376`, `400-408`
- 현상:
  - `run_timeout_cause_last`, `init_cfg_coalesced_mask`, `cmd_collision_mask`는 **TDC clock domain**에서 만들어짐.
  - 그런데 `config_ctrl`에서 “No CDC for now”로 그대로 내보내고, top에서 `s_status`에 직결한 뒤 AXI-domain `csr_pipeline`가 바로 읽음.
  - 특히 `run_timeout_cause_last`에 대해서는 `config_ctrl` 주석이 “status CDC가 처리한다”고 적지만 (`641-642`), 현재 top/csr_pipeline 구조에는 그 CDC가 실제로 보이지 않음.
- 영향:
  - metastability, torn read, 이벤트-원인 misalignment 발생 가능.
- 권고:
  - 최소 2-FF sync 또는 status shadow register를 AXI domain에서 다시 잡아야 함.
  - pulse-원인 쌍(`run_timeout`, `cause`)은 **같은 CDC envelope**로 묶는 편이 더 안전함.

### 4) `chip_run`은 drain count와 expected count의 정합성을 끝까지 확인하지 않음
- 위치: `tdc_gpx_chip_run.vhd:503-514`
- 현상:
  - `ST_DRAIN_CHECK`에서 두 IFIFO가 모두 done으로 판정되지 않았더라도,
  - `v_ififo1_can_read/v_ififo2_can_read`가 둘 다 false면 **Fallback completion**으로 바로 `drain_done`을 내고 종료함.
  - 이 경로에서는 `expected_ififo` 대비 실제 drain count가 정확히 맞았는지 따로 sticky/error를 세우지 않음.
- 영향:
  - upstream count drift, late/early EF 판정, expected count 오염이 있어도 조용히 완료 처리될 수 있음.
- 권고:
  - `expected != actual` mismatch를 별도 sticky/status로 남기고,
  - fallback completion은 “완료”가 아니라 “강제 종료”로 구분하는 편이 좋음.

### 5) `cmd_arb`는 concurrent read+write를 silicon에서 명시적으로 잡아주지 못함
- 위치: `tdc_gpx_cmd_arb.vhd:240-253`
- 현상:
  - `i_cmd_reg_read`와 `i_cmd_reg_write`가 동시에 오면 write wins.
  - 현재는 sim-only assert만 있고, synth/live path에는 **전용 sticky가 없음**.
- 영향:
  - 상위에서 잘못된 동시 펄스가 들어오면 read intent가 조용히 write로 바뀜.
- 권고:
  - `ambiguous_reg_cmd` sticky를 추가하거나,
  - 아예 둘 다 무시하고 reject sticky를 세우는 쪽이 안전함.

### 6) `chip_reg`도 busy 중 concurrent read+write가 write-wins로 먹힘
- 위치: `tdc_gpx_chip_reg.vhd:192-217`
- 현상:
  - `ST_ACTIVE`에서 `i_start_read`와 `i_start_write`가 동시에 오면 `s_pend_rw_r <= i_start_write` 때문에 write wins.
  - 여기 역시 sim-only assert만 있고 runtime sticky는 없음.
- 영향:
  - per-chip reg path도 동일한 ambiguity를 가짐.
- 권고:
  - `cmd_arb`와 정책을 통일해서 runtime sticky를 추가하거나 reject하도록 바꾸는 편이 좋음.

### 7) `cell_builder` QUARANTINE 강제 탈출은 가용성은 살리지만 데이터 정합성은 크게 깨뜨림
- 위치: `tdc_gpx_cell_builder.vhd:629-655`, `345-351`
- 현상:
  - `ST_C_QUARANTINE`에서 drain marker가 끝내 안 오면, 긴 watchdog 후 buffer를 전부 free로 만들고 `ST_C_IDLE`로 강제 복귀함.
  - 동시에 `s_shot_pending_r`도 버리고 `s_quarantine_escape_sticky_r`만 남김.
  - 그런데 이 모듈은 `ST_C_IDLE`에서 `o_s_axis_tready='0'`이므로 (`348-351`), 강제 탈출 후에도 늦은 stale beat가 오면 **다시 upstream stall**을 만들 수 있음.
- 영향:
  - “영구 sink”는 피하지만, 결국 늦은 stale stream이 계속 있으면 다시 막힐 수 있음.
- 권고:
  - 강제 탈출 시 상위에 quarantine-clear handshake를 요구하거나,
  - stale source를 함께 flush할 수 있는 상위 연동이 필요함.

### 8) `face_assembler`는 새 shot에서 이전 shot tail을 즉시 flush함
- 위치: `tdc_gpx_face_assembler.vhd:397-399`, `481-487`
- 현상:
  - `s_flush <= i_shot_start or i_abort`로 입력 FIFO를 바로 reset.
  - 이전 shot tail beat가 아직 input FIFO에 남아 있으면 드롭되고, 그 사실은 `s_shot_flush_drop_r/mask`로만 남김.
- 영향:
  - shot boundary alignment는 맞추지만, **late tail data completeness**는 희생됨.
- 권고:
  - 현재 정책을 유지하더라도, status만이 아니라 **복구 정책**(예: 다음 frame discard, SW-visible severity level)을 명확히 두는 편이 좋음.

### 9) `chip_ctrl`의 live `stopdis_override`는 FSM과 핀 상태를 의도적으로 분리함
- 위치: `tdc_gpx_chip_ctrl.vhd:780-800`
- 현상:
  - `stopdis_override(4)`가 켜지면 FSM snapshot과 무관하게 핀 상태가 바로 바뀜.
  - 주석도 “FSM state와 잠시 disagree할 수 있음”을 명시함.
- 영향:
  - mid-shot에 enable/disable이 바뀌면 in-flight capture/drain 의미가 흐트러질 수 있음.
- 권고:
  - debug 전용이라면 CSR 보호/잠금 비트를 두고,
  - 실운영에서는 mid-shot override를 금지하거나 auto-abort 연동이 필요함.

### 10) `header_inserter`의 `face_start` 큐는 1-depth라 pulse burst를 보존하지 못함
- 위치: `tdc_gpx_header_inserter.vhd:554-582`, `590-592`
- 현상:
  - non-IDLE에서 들어온 `face_start`는 `s_face_start_pending_r` 1비트로만 보관됨.
  - 이미 pending이 있는 상태에서 추가 pulse가 오면 collapsed counter만 증가하고 실제 start는 하나로 합쳐짐.
- 영향:
  - upstream contract가 깨질 경우 face 하나 이상이 조용히 합쳐질 수 있음.
- 권고:
  - 현재 face_seq가 1-cycle serialized pulse를 보장하더라도, 안전성을 높이려면 최소 2-depth queue가 더 낫다.

---

## B. 구조적 위험 / 검증 권고 / 관찰성 부족

### 11) `chip_init`의 deferred cfg_write는 1-depth coalesce 정책이라 중간 이미지를 잃음
- 위치: `tdc_gpx_chip_init.vhd:174-183`, `194-203`, `321-340`
- 현상:
  - busy 중 cfg_write가 여러 번 오면 첫 snapshot 하나만 유지하고 나머지는 `s_cfg_write_coalesced_r` sticky만 남김.
- 영향:
  - SW가 “요청한 순서대로 여러 cfg 이미지가 모두 적용된다”고 기대하면 틀림.
- 권고:
  - 정책 문서화 또는 FIFO화 필요.

### 12) `header_inserter`는 로컬에서 invalid geometry를 reject하지 않고 clamp함
- 위치: `tdc_gpx_header_inserter.vhd:608-611`
- 현상:
  - `cols_per_face < 1`이면 local에서 1로 clamp.
- 영향:
  - 상위 validation이 우회되면 잘못된 config가 “조용히 정상처럼” 흘러갈 수 있음.
- 권고:
  - 로컬에서도 reject sticky를 세우는 편이 더 안전함.

### 13) `face_assembler`도 일부 invalid geometry를 local clamp/self-complete로 처리함
- 위치: `tdc_gpx_face_assembler.vhd:535-539`, `552-558`
- 현상:
  - `stops_per_chip < 2`면 1 stop으로 clamp.
  - active mask가 0이면 row_done을 바로 내고 self-complete.
- 영향:
  - top-level `face_seq` validation이 깨졌을 때 하위가 조용히 fail-open 동작.
- 권고:
  - 최소한 runtime sticky를 두는 편이 좋음.

### 14) `face_assembler`의 `o_face_abort`는 사실상 dead port
- 위치: `tdc_gpx_face_assembler.vhd:830-834`
- 현상:
  - 주석상으로도 Round 4 이후 assert되지 않는 포트인데 인터페이스에 남아 있음.
- 영향:
  - 유지보수 혼선, top/output_stage 인터페이스 오해 위험.
- 권고:
  - 제거하거나, truly-live semantic으로 되살리거나 둘 중 하나로 정리 필요.

### 15) `chip_ctrl`의 `o_err_raw_overflow`는 여러 원인을 한 비트로 뭉개 버림
- 위치: `tdc_gpx_chip_ctrl.vhd:994-1000`
- 현상:
  - raw FIFO beat drop과 `PH_RESP_DRAIN` hard-cap quarantine이 모두 `o_err_raw_overflow`로 OR됨.
- 영향:
  - SW가 “데이터 drop인지”, “bus quarantine인지”를 구분할 수 없음.
- 권고:
  - 최소한 status cause bit를 분리하는 것이 좋음.

### 16) `stop_cfg_decode`는 여전히 upstream 포맷에 강하게 의존함
- 위치: `tdc_gpx_stop_cfg_decode.vhd:84-147`
- 현상:
  - 기대 drain count는 매 stop event마다 overwrite됨.
  - synth-side checker는 “running total 감소”만 잡고 (`110-118`),
    - final beat 누락,
    - shot boundary 펄스 누락,
    - 잘못된 칩/IFIFO grouping 같은 다른 드리프트는 직접 잡지 않음.
- 영향:
  - monotonic 위반이 아닌 종류의 upstream format drift는 expected count 오염으로 이어질 수 있음.
- 권고:
  - 최소한 `shot_start 이후 첫 beat / 마지막 beat / valid chip mask` 수준의 검증 sticky를 보강하면 좋음.

### 17) `config_ctrl`의 timeout cause 집계는 “마지막 cause 하나”만 남김
- 위치: `tdc_gpx_config_ctrl.vhd:643-657`
- 현상:
  - 여러 chip timeout이 같은 시점에 나도 마지막으로 돈 loop iteration의 cause만 저장됨.
- 영향:
  - per-chip/per-event cause granularity가 사라짐.
- 권고:
  - mask+cause 배열, 또는 last+mask 조합으로 올리는 편이 진단에 유리함.

### 18) `face_assembler` overrun 시 에러 flag는 “현재 chip partial”과 “아직 시작 안 한 chip”을 구분하지 않음
- 위치: `tdc_gpx_face_assembler.vhd:777-785`
- 현상:
  - `s_active_mask_r and (not s_chip_done_r)` 전체를 한 번에 error 처리함.
- 영향:
  - 실제 partially-corrupted chip과 blank-filled future chip이 같은 error 비트로 보임.
- 권고:
  - current-chip partial / future-chip blank-fill 을 별도 status로 구분하면 디버깅이 쉬워짐.

### 19) `header_inserter` abort는 synthetic `tlast`를 내지 않음
- 위치: `tdc_gpx_header_inserter.vhd:656-666`, `668-686`
- 현상:
  - abort 시 truncated line으로 끝내며, 의도적으로 valid EOL을 만들지 않음.
- 영향:
  - downstream consumer는 sideband/status 해석을 반드시 해야 하고, 단순 AXIS line parser만 보면 불완전 프레임으로 보일 수 있음.
- 권고:
  - 현 정책이 맞더라도, 이 상태를 명시하는 별도 frame-level status와 discard policy를 더 강하게 연결하는 편이 좋음.

### 20) `PH_IDLE` command collision은 generic sticky만 있고, 어떤 명령이 탈락했는지는 남지 않음
- 위치: `tdc_gpx_chip_ctrl.vhd:577-601`
- 현상:
  - start/cfg/reg가 동시에 오면 우선순위대로 하나만 살리고 `s_err_cmd_collision_r`만 남김.
- 영향:
  - root-cause 분석 시 “무슨 명령이 떨어졌는지”가 안 보임.
- 권고:
  - collision vector 또는 lost-command bitmap이 있으면 훨씬 진단이 쉬움.

### 21) `cell_builder`의 `s_rt_max_hits_r`는 현재 실사용 흔적이 없음
- 위치: `tdc_gpx_cell_builder.vhd:253`, `720`, `762`
- 현상:
  - latch는 하지만 이후 소비되는 경로가 보이지 않음.
- 영향:
  - dead state/obsolete path일 가능성이 높아 유지보수 혼선을 줌.
- 권고:
  - 제거하거나 실제 사용처를 연결해 의미를 분명히 할 필요가 있음.

### 22) `err_handler` 내부 신호명은 여전히 `reg11`인데 실제로는 Reg12 status를 읽음
- 위치: `tdc_gpx_err_handler.vhd:180`, `255-315`
- 현상:
  - 내부 배열 이름 `s_reg11_data`가 남아 있는데, 주석/로직은 Reg12 status decode를 수행함.
- 영향:
  - 기능은 맞아도 코드를 읽는 사람이 잘못 이해하기 쉽다.
- 권고:
  - 이름을 정리하는 편이 좋음.

---

## 우선순위 제안

### 1순위: 바로 수정 권고
1. `chip_ctrl` PH_RESP_DRAIN 영구 quarantine 복구 경로 추가
2. `chip_ctrl` raw/control beat 완전 분리 또는 control beat 무손실화
3. `config_ctrl/top`의 미동기 status CDC 정리 (`run_timeout_cause_last`, `init_cfg_coalesced_mask`, `cmd_collision_mask`)
4. `chip_run` drain completion에 expected-vs-actual mismatch sticky 추가
5. `cmd_arb`, `chip_reg`의 concurrent R/W ambiguity를 silicon-visible sticky로 노출

### 2순위: 구조 안정화
6. `cell_builder` QUARANTINE forced-escape 이후 stale source와의 정합성 보강
7. `header_inserter` / `chip_init` 1-depth pending/coalesce 정책을 queue 또는 stronger reject로 정리
8. `chip_ctrl` live stopdis_override 사용 정책 제한

### 3순위: 유지보수/관찰성 정리
9. dead port / dead state 정리 (`o_face_abort`, `s_rt_max_hits_r`, `s_reg11_data` naming)
10. 원인 구분 status 분리 (`raw_overflow`, command collision detail, overrun granularity)

---

## 최종 결론

현재 업로드본은 예전보다 **확실히 안정화가 진행된 버전**입니다. 다만 코드만 놓고 보면, 아직도 핵심 위험 축이 남아 있습니다.

1. **stuck bus / stale response 상황에서 복구가 닫혀 있는 경로**
2. **control beat까지 포함한 완충 포화 시 의미 손실 가능성**
3. **TDC→AXI status 경계의 미정리 CDC**
4. **동시 명령/동시 요청 ambiguity를 synth-live에서 명확히 잡지 못하는 부분**

이 4축을 먼저 정리하면, FSM 안정성은 한 단계 더 올라갈 가능성이 큽니다.
