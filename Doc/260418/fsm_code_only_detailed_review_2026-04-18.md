# TDC-GPX FSM 코드 기준 상세 점검 (계약 문서 배제)

작성 기준:
- 판단 근거는 업로드된 RTL/VHDL 코드만 사용
- 계약 문서/요구사항 문서/데이터시트는 판정 근거에서 제외
- 아래 라인 번호는 현재 업로드된 파일 기준

## 총평

이번 업로드본은 이전에 자주 문제 되던 몇몇 항목이 이미 보강되어 있습니다.
예를 들어:
- `tdc_gpx_err_handler.vhd`의 `ST_WAIT_READ` timeout 추가
- `tdc_gpx_header_inserter.vhd`의 non-idle `face_start` pending latch 추가
- `tdc_gpx_cell_builder.vhd`의 `ST_O_WAIT_IFIFO2` timeout 시 synthetic `tlast` 생성
- `tdc_gpx_chip_run.vhd`의 drain timeout 경로에서 `range_active` clear 보강
- `tdc_gpx_cmd_arb.vhd`의 reg-timeout 카운트 시작 시점 개선

하지만 **코드 논리만 기준으로 보면 FSM이 완전히 안정화됐다고 보기는 아직 어렵습니다.**
특히 `chip_run ↔ chip_ctrl ↔ bus_phy` 경계, `AXI reset → TDC domain` 경계, `AXI cfg → TDC snapshot` 경계에 아직 고위험 항목이 남아 있습니다.

아래 항목을 3단계로 분류했습니다.

- **A. 고신뢰/high-confidence 문제**: 실제 오동작 가능성이 높음
- **B. 고위험 구조 가정/관찰성 결함**: 경계조건에서 깨질 수 있거나 디버그가 어려움
- **C. 유지보수/계약 드리프트**: 당장 치명상은 아니더라도 future bug 유발 가능

총 항목 수: **23개**

---

## A. 고신뢰 문제 (10개)

### A-1. `chip_run`이 “응답 도착”이 아니라 “응답 handshake 완료”만 본다
- 파일: `tdc_gpx_chip_ctrl.vhd:425-439`, `tdc_gpx_chip_run.vhd:496-600`
- 핵심:
  - `chip_ctrl`는 `s_bus_rsp_fire <= i_s_axis_tvalid and o_s_axis_tready`만 sub-FSM으로 전달한다.
  - 즉 `chip_run`은 **버스 응답이 도착했는지**가 아니라 **응답이 받아들여졌는지**만 안다.
  - 그런데 `PH_RUN`에서 raw hold가 가득 차면 `o_s_axis_tready <= '0'`가 된다.
- 결과:
  - 응답 beat가 bus/skid 쪽에 pending 상태여도 `chip_run`은 “응답 없음”으로 본다.
  - `ST_DRAIN_EF1`, `ST_DRAIN_EF2`, `ST_DRAIN_BURST`, `ST_DRAIN_FLUSH` watchdog이 **가짜 timeout**을 낼 수 있다.
- 권고:
  - `chip_run`에 `i_bus_rsp_pending` 또는 `i_rsp_seen` 같은 별도 신호를 추가해 “도착”과 “소비”를 분리해야 한다.

### A-2. `ST_DRAIN_FLUSH`가 pending response를 남긴 채 종료할 수 있다
- 파일: `tdc_gpx_chip_run.vhd:575-600`, `tdc_gpx_chip_ctrl.vhd:425-439`, `tdc_gpx_chip_ctrl.vhd:582-609`
- 핵심:
  - `chip_run`은 `i_bus_busy='0' and i_bus_rsp_valid='0'`이면 flush 완료로 본다.
  - 하지만 여기서 `i_bus_rsp_valid`는 실제 pending valid가 아니라 `s_bus_rsp_fire`다.
- 결과:
  - bus_phy/skid에 읽기 응답이 남아 있어도, downstream backpressure 때문에 `fire=0`이면 `chip_run`은 flush를 끝낸다.
  - 다음 phase에서 stale response가 튀어나올 수 있다.
- 권고:
  - `ST_DRAIN_FLUSH` 종료 조건에 `rsp_pending`까지 포함시키거나, 아예 `chip_ctrl`가 flush 완료 pulse를 만들어 `chip_run`이 그걸 보도록 바꾸는 게 안전하다.

### A-3. `chip_ctrl`의 2-depth raw hold/skid는 여전히 beat drop 가능성이 있다
- 파일: `tdc_gpx_chip_ctrl.vhd:735-803`, `tdc_gpx_chip_ctrl.vhd:822-833`
- 핵심:
  - hold+skid가 둘 다 찬 상태에서 새 beat가 오면 drop시키고 `s_err_raw_overflow_r`를 올린다.
  - `s_raw_hold_busy`는 등록형이라 1 cycle 늦게 반영된다.
- 결과:
  - 최악 동시성에서 **3번째 beat**가 들어오면 raw data뿐 아니라 control beat도 유실될 수 있다.
- 권고:
  - 최소 3-depth로 늘리거나, `drain_done`/`ififo1_done` 같은 control beat는 별도 채널로 분리하는 게 낫다.

### A-4. drop 대상이 raw data만이 아니라 control beat까지 포함된다
- 파일: `tdc_gpx_chip_ctrl.vhd:740-748`, `tdc_gpx_chip_ctrl.vhd:785-790`
- 핵심:
  - `s_run_drain_done`와 `s_run_ififo1_beat`도 동일한 hold/skid 경로로 패킹된다.
- 결과:
  - overflow가 나면 raw word뿐 아니라 `drain_done`/`ififo1_done` 같은 FSM 제어 의미가 있는 beat도 사라질 수 있다.
- 권고:
  - control beat는 별도 소형 FIFO 또는 우선순위 경로로 분리.

### A-5. `chip_run` overrun recovery가 같은 cycle의 bus response를 의도적으로 버린다
- 파일: `tdc_gpx_chip_run.vhd:684-721`
- 핵심:
  - `i_shot_start='1'`가 비-ARMED 상태에 오면 overrun recovery가 우선한다.
  - 같은 cycle `i_bus_rsp_valid='1'`이면 `s_err_overrun_drop_r`만 세우고 beat는 소실된다.
- 결과:
  - drain accounting과 실제 전달 raw beat 수가 어긋날 수 있다.
- 권고:
  - response beat를 1-entry라도 우선 보관한 뒤 overrun으로 전환하거나, `fire`가 난 cycle은 overrun을 1 cycle defer하는 방식이 더 안전하다.

### A-6. `cell_builder`의 DROP-timeout 복귀 후 late stale beat가 오면 upstream stall로 이어질 수 있다
- 파일: `tdc_gpx_cell_builder.vhd:305-315`, `tdc_gpx_cell_builder.vhd:513-534`
- 핵심:
  - `ST_C_DROP` timeout 시 강제로 `ST_C_IDLE`로 복귀한다.
  - 주석 그대로, 그 뒤 늦게 stale beat가 오면 `IDLE`에서 `tready='0'`라 upstream backpressure를 건다.
- 결과:
  - incomplete recovery 후 파이프라인이 소프트리셋 전까지 묶일 수 있다.
- 권고:
  - `ST_C_QUARANTINE` 같은 상태를 두고 explicit reset/drain marker 전까지 버리도록 설계하는 편이 안전하다.

### A-7. `config_ctrl`의 multi-bit config snapshot은 handshake 없는 CDC 샘플링이다
- 파일: `tdc_gpx_config_ctrl.vhd:702-719`
- 핵심:
  - `s_cfg_merged`, `o_cfg_image`, `s_expected_ififo1/2`, `s_cmd_reg_addr_mux`, `s_cmd_reg_wdata`를 TDC clock에서 그냥 매 cycle 샘플한다.
  - 주석도 “quasi-static” 가정에 기대고 있다.
- 결과:
  - SW가 값 변경과 command pulse 타이밍을 잘못 겹치면 비트 혼합/반쯤 바뀐 snapshot이 잡힐 수 있다.
  - 특히 expected IFIFO count가 엇갈리면 drain burst sizing이 깨질 수 있다.
- 권고:
  - `cfg_shadow_valid/toggle` 기반 명시적 CDC handshake 또는 dual-clock register bank로 바꾸는 게 바람직하다.

### A-8. TDC clock 도메인 모듈들이 AXI reset을 직접 reset으로 받는다
- 파일: `tdc_gpx_config_ctrl.vhd:742-746`, `tdc_gpx_config_ctrl.vhd:957-967`, `tdc_gpx_config_ctrl.vhd:781-789`
- 핵심:
  - `i_tdc_clk`로 구동되는 `u_bus_phy`, `u_chip_ctrl`, `u_sk_brsp` 등에 `i_rst_n => i_axis_aresetn`이 직접 들어간다.
- 결과:
  - reset assert는 괜찮아도 **deassert가 i_tdc_clk 기준 비동기**라 release metastability/부분 해제 위험이 있다.
- 권고:
  - `i_axis_aresetn`을 TDC domain에서 2FF sync 후 deassert하는 domain-local reset을 만들어 써야 한다.

### A-9. `PH_RESP_DRAIN` 하드캡 15 cycle은 stale response를 남긴 채 다음 phase로 갈 수 있다
- 파일: `tdc_gpx_chip_ctrl.vhd:582-609`
- 핵심:
  - 15 cycle 내 `i_bus_busy`/`i_bus_rsp_pending`이 안 비워지면 sticky만 올리고 phase를 강행한다.
- 결과:
  - 다음 phase에 stale response가 섞일 수 있는 구조다.
- 권고:
  - hard cap을 유지하더라도, 다음 phase 진입 전 bus reset/recover 절차를 강제하거나, stale response quarantine를 둬야 한다.

### A-10. `cmd_arb`는 active reg transaction 중 새 reg request를 그냥 무시한다
- 파일: `tdc_gpx_cmd_arb.vhd:246-319`
- 핵심:
  - 새 요청 수락은 `v_new_request='1' and s_reg_active_r='0'`일 때만 한다.
  - active 동안 들어온 새 transaction-set은 pending queue나 sticky reject 없이 사라진다.
- 결과:
  - SW/상위 FSM이 pulse를 겹치게 주면 요청 손실이 발생한다.
- 권고:
  - arb 레벨에도 1-depth pending set 또는 explicit reject/status bit를 추가해야 한다.

---

## B. 고위험 구조 가정 / 관찰성 결함 (8개)

### B-1. `chip_ctrl`의 `o_drain_done`는 실제 drain 완료 시점이 아니라 “control beat accepted” 시점이다
- 파일: `tdc_gpx_chip_ctrl.vhd:811-820`
- 핵심:
  - 주석에도 명시돼 있듯, downstream backpressure가 있으면 `o_drain_done`이 늦어진다.
- 리스크:
  - 소비 측이 이 pulse를 “chip_run drain 종료 시점”으로 오해하면 상태 판단이 어긋난다.
- 권고:
  - `o_run_drain_complete`(internal completion)와 `o_drain_done_accepted`(stream handshake)를 분리.

### B-2. `chip_reg`는 3번째 pulse overflow를 잡지만 외부로 내보내지 않는다
- 파일: `tdc_gpx_chip_reg.vhd:16-21`, `tdc_gpx_chip_reg.vhd:46-62`, `tdc_gpx_chip_reg.vhd:175-190`
- 핵심:
  - `s_err_req_overflow_r` sticky가 내부에는 있지만 port가 없다.
- 리스크:
  - 실제 요청 손실이 나도 SW/상위는 모른다.
- 권고:
  - status port나 status register로 노출.

### B-3. `err_handler`의 read-timeout sticky도 외부로 노출되지 않는다
- 파일: `tdc_gpx_err_handler.vhd:16-21`, `tdc_gpx_err_handler.vhd:76-80`, `tdc_gpx_err_handler.vhd:268-280`
- 핵심:
  - `s_err_read_timeout_r`는 내부 sticky뿐이고 output이 없다.
- 리스크:
  - 분류 실패가 “없었다”로 보일 수 있다.
- 권고:
  - `o_err_read_timeout` 또는 cause 확장 필요.

### B-4. `chip_init`는 busy 중 추가 `cfg_write_req`를 일반적으로 큐잉하지 않는다
- 파일: `tdc_gpx_chip_init.vhd:21-22`, `tdc_gpx_chip_init.vhd:151-163`
- 핵심:
  - ST_OFF에서 `start`와 동시인 `cfg_write_req`만 1-depth pending으로 저장한다.
  - busy 중 새 cfg_write pulse는 별도 흡수 경로가 없다.
- 리스크:
  - SW가 연속 cfg_write pulse를 주면 요청 소실 가능.
- 권고:
  - pending latch를 busy 중에도 유지하거나 level request로 변경.

### B-5. `face_assembler`는 shot boundary 보존을 위해 늦게 온 이전 shot data를 무조건 버린다
- 파일: `tdc_gpx_face_assembler.vhd:350-358`
- 핵심:
  - `s_flush <= i_shot_start or i_abort`
  - 주석도 “Late-arriving beats from previous shot are intentionally dropped.”라고 명시.
- 리스크:
  - 시스템 타이밍이 조금만 어긋나도 데이터 completeness보다 boundary integrity가 무조건 우선되어 old-shot tail이 잘린다.
- 권고:
  - per-shot tag 비교 후 old-shot만 선별 폐기하거나, new shot accept 시점을 line-close 이후로 제한.

### B-6. `face_assembler` overrun 처리도 현재 chip 이후를 blank-fill 강제하는 공격적 정책이다
- 파일: `tdc_gpx_face_assembler.vhd:692-711`
- 핵심:
  - mid-row shot overrun이 오면 `s_shot_cnt_r <= x"FFFF"`로 upcoming chips를 blank mode로 몰고 간다.
- 리스크:
  - 정책상 맞을 수는 있지만, 아주 짧은 경계 교차에서도 정상 데이터 일부를 버린다.
- 권고:
  - 정책 유지 시에도 카운터/trace/status를 더 노출해 원인 추적성을 높이는 게 좋다.

### B-7. `cmd_arb`의 zero-mask reg request는 시뮬레이션 경고 외엔 런타임 관찰성이 없다
- 파일: `tdc_gpx_cmd_arb.vhd:247-255`
- 핵심:
  - zero mask는 ignore되고 sim assert만 남는다.
- 리스크:
  - 실기에서는 원인 불명의 “아무 일도 안 일어남”으로 보일 수 있다.
- 권고:
  - sticky status 추가.

### B-8. `chip_run` expected IFIFO count 계약이 주석 레벨에서 자기모순이다
- 파일: `tdc_gpx_chip_run.vhd:24-29`, `tdc_gpx_chip_run.vhd:84-87`
- 핵심:
  - 파일 상단 주석은 “drain_latch 전 안정성 assertion을 제거했고 continuous update와 충돌하므로 기대하면 안 된다”고 쓴다.
  - 포트 주석은 여전히 “must be stable from i_shot_start through ST_DRAIN_LATCH”라고 적혀 있다.
- 리스크:
  - 기능보다 유지보수 위험이지만, 잘못된 상위 가정을 다시 심을 수 있다.
- 권고:
  - 포트 주석을 현재 실제 설계 의도와 맞춰 정리.

---

## C. 유지보수 / 인터페이스 드리프트 (5개)

### C-1. `face_assembler`의 `o_face_abort` 포트는 사실상 죽은 포트다
- 파일: `tdc_gpx_face_assembler.vhd:220-221`, `tdc_gpx_face_assembler.vhd:725-730`, `tdc_gpx_face_assembler.vhd:743-747`
- 핵심:
  - `s_face_abort_r`는 reset만 되고, overrun/abort 경로에서 더 이상 assert하지 않는다고 주석에 명시돼 있다.
- 리스크:
  - 외부가 이 포트를 아직 의미 있다고 믿으면 오동작한다.
- 권고:
  - 제거하거나, deprecated 표기를 더 강하게 하고 연결부도 정리.

### C-2. `face_seq`의 `s_shot_overrun_r`는 set/clear만 되고 module 내부에서 읽히지 않는다
- 파일: `tdc_gpx_face_seq.vhd:114-119`
- 리스크:
  - dead logic이 future edit에서 혼란을 만든다.
- 권고:
  - 실제로 쓸 계획이 없으면 삭제.

### C-3. `header_inserter` 주석은 아직 “non-IDLE face_start ignored”라고 말하지만 실제 코드는 pending latch를 둔다
- 파일: `tdc_gpx_header_inserter.vhd:480-500`
- 핵심:
  - 코드 자체는 좋아졌지만 주석은 예전 contract를 일부 유지한다.
- 리스크:
  - 나중에 누군가 주석을 믿고 상위 pulse timing을 바꾸면 혼란이 생긴다.
- 권고:
  - 주석을 현재 동작 기준으로 다시 정리.

### C-4. `cell_pipe`의 시뮬레이션 assert는 concurrent 영역에서 `rising_edge`를 사용한다
- 파일: `tdc_gpx_cell_pipe.vhd:220-227`
- 리스크:
  - 일부 시뮬레이터/lint에서 비표준적이거나 tool-fragile하게 보일 수 있다.
- 권고:
  - clocked process 내부 assert로 이동.

### C-5. `err_handler` 포트명은 여전히 `reg11`인데 실제 의미는 Reg12 읽기다
- 파일: `tdc_gpx_err_handler.vhd:51-57`
- 리스크:
  - backward compatibility 때문에 남겼다고 적혀 있지만, 인터페이스 이름과 실제 의미가 다르다.
- 권고:
  - wrapper에서 alias를 두거나 다음 라운드에서 명칭 정리.

---

## 우선순위별 수정 권고

### 1순위: 반드시 먼저 수정
1. `chip_run`이 `response-arrived`와 `response-consumed`를 구분하도록 인터페이스 수정
2. `ST_DRAIN_FLUSH` 종료 조건에 pending response 반영
3. `chip_ctrl` raw/control beat 분리 또는 buffer depth 확장
4. `config_ctrl` multi-bit snapshot CDC를 handshake 방식으로 교체
5. TDC domain reset deassert 동기화
6. `cell_builder` DROP-timeout 이후 late stale beat quarantine 경로 추가

### 2순위: 빠르게 보강
7. `cmd_arb` overlapping reg request queue/reject status 추가
8. `chip_reg` overflow sticky 외부 노출
9. `err_handler` read-timeout sticky 외부 노출
10. `PH_RESP_DRAIN` hard cap 이후 stale response quarantine/flush 개선
11. `face_assembler` shot_start flush 정책을 더 보수적으로 조정하거나 trace 강화

### 3순위: 유지보수 정리
12. dead port / dead latch / stale comments 정리 (`o_face_abort`, `s_shot_overrun_r`, chip_run/header_inserter 주석)
13. `cell_pipe` assert 스타일 수정
14. `err_handler` reg11/reg12 naming 정리

---

## 최종 결론

**지금 업로드된 RTL은 이전보다 분명히 좋아졌지만, 아직 “FSM 완전 안정화”로 보기엔 이릅니다.**

가장 큰 이유는 세 가지입니다.

1. **`chip_run`이 handshake 완료만 보고, pending response를 못 보는 구조**
2. **AXI domain → TDC domain의 multi-bit snapshot / reset release가 명시적 CDC 보장을 갖지 않는 구조**
3. **raw data와 control beat가 같은 작은 완충 경로를 공유해 overflow 시 함께 손실될 수 있는 구조**

이 세 축만 먼저 정리해도 전체 안정성은 크게 올라갈 가능성이 큽니다.
