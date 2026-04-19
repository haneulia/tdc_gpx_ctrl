# TDC-GPX FSM Re-review (latest upload)

기준: 현재 업로드된 RTL/VHDL만 대조.
이 문서는 이전 FSM 리뷰/체크리스트와 최신 소스코드를 직접 비교한 결과를 정리한다.

## 요약

최신 업로드에는 다음 반영이 실제로 보인다.

- `tdc_gpx_chip_ctrl.vhd`
  - `PH_RESP_DRAIN` 상태 추가
  - `o_s_axis_tready` gating 추가
  - `p_rsp_check`가 `PH_INIT/PH_CFG_WRITE/PH_RUN/PH_REG`까지 확장
  - `o_err_rsp_mismatch` status 연결
- `tdc_gpx_config_ctrl.vhd`
  - `u_sk_brsp.i_flush => i_cmd_soft_reset`
  - err-active 중 write 차단 유지
  - err-fill upper bits preserve
- `tdc_gpx_raw_event_builder.vhd`
  - `i_abort` 포트 추가
  - `shot_seq`를 `tuser[15:11]`에 실어 보냄
- `tdc_gpx_cell_pipe.vhd`
  - `i_abort` 포트 추가
  - slope-aware tready 구현
- `tdc_gpx_cell_builder.vhd`
  - `t_buf_owner = (BUF_FREE, BUF_COLLECT, BUF_SHARED)` 추가
  - `i_abort` 포트 추가
- `tdc_gpx_top.vhd`
  - `s_status.rsp_mismatch_mask <= s_err_rsp_mismatch`

즉, 예전 리뷰에서 지적했던 항목 다수가 **이번 업로드에서는 실제로 반영**되었다.

다만 FSM 관점에서 **아직 닫히지 않은 핵심 4개 + 권고 1개**가 남아 있다.

---

## A. 예전 지적 중 현재는 해결된 것

### A1. soft reset stale-response 격리 구조가 전혀 없다는 지적
이건 **현재 코드에는 더 이상 그대로 해당하지 않는다**.

- `tdc_gpx_chip_ctrl.vhd`
  - `type t_phase is (PH_INIT, PH_IDLE, PH_RUN, PH_REG, PH_CFG_WRITE, PH_RESP_DRAIN);`
  - soft reset 시 `s_phase_r <= PH_RESP_DRAIN`
  - `PH_RESP_DRAIN`에서 응답 라우팅 차단 + `o_s_axis_tready='1'`
- `tdc_gpx_config_ctrl.vhd`
  - `u_sk_brsp.i_flush => i_cmd_soft_reset`

즉 **"아예 없다"는 이전 지적은 최신 업로드 기준으로는 틀렸다.**

### A2. raw_event_builder / cell_pipe에 abort 포트가 없다는 지적
이것도 **현재는 틀렸다**.

- `tdc_gpx_raw_event_builder.vhd` entity에 `i_abort`
- `tdc_gpx_cell_pipe.vhd` entity와 내부 demux reset에 `i_abort`
- `tdc_gpx_decode_pipe.vhd`가 `i_flush`를 두 모듈에 연결

### A3. top에서 rsp_mismatch가 버려진다는 지적
이것도 **현재는 틀렸다**.

- `tdc_gpx_top.vhd:718`
  - `s_status.rsp_mismatch_mask <= s_err_rsp_mismatch;`

---

## B. 아직 남아 있는 FSM 문제

### B1. `tdc_gpx_bus_phy.vhd` burst read는 downstream `tready`를 무시한다

이건 **현재도 남아 있는 실질 버그**다.

핵심 흐름:

1. `s_rsp_pending_r='1'`이면 bus_phy는 **무조건** `s_axis_tvalid_r <= '1'`로 응답을 내보낸다.
2. 이 응답이 소비되지 않았어도 burst 모드에서는 `ST_READ`가 계속 다음 read를 진행한다.
3. 다음 응답이 오면 `s_axis_tdata_r/s_axis_tuser_r`가 다시 덮일 수 있다.

문제 라인:

- `tdc_gpx_bus_phy.vhd:325-348`
  - 응답 보류(`s_rsp_pending_r`)가 풀리는 순간 `s_axis_tvalid_r`를 세우지만, `i_m_axis_tready='0'`이어도 beat를 hold만 할 뿐 burst FSM 자체는 멈추지 않음.
- `tdc_gpx_bus_phy.vhd:516-525`
  - burst 조건에서 `s_tick_r <= 0;`로 바로 다음 read를 계속 시작하고, 여기서 downstream ready를 전혀 보지 않음.

왜 문제냐:

- `tdc_gpx_chip_ctrl`는 `PH_RUN`에서 raw hold가 full이면 `o_s_axis_tready <= '0'`로 내려서 stall을 걸려고 한다.
- 하지만 bus_phy burst path가 이 stall을 transaction-level로 반영하지 않으면, **연속 read 응답이 1-deep AXI response register를 덮어쓸 수 있다.**

즉, 체크리스트의 **"raw path end-to-end handshake 완료"**는 최신 코드 기준으로 **부분적으로만 맞다.**

수정 권장:

- burst continuation 조건에 `i_m_axis_tready='1'` 또는 `s_axis_tvalid_r='0' and s_rsp_pending_r='0'`를 추가해서
  **response emission이 비워질 때까지 다음 burst read를 진행하지 않게** 해야 한다.

---

### B2. `tdc_gpx_chip_ctrl.vhd`의 `PH_RESP_DRAIN`은 고정 8클럭이라서 bus-tick가 느리면 stale response를 다 못 버릴 수 있다

이것도 **현재 남아 있다.**

문제 라인:

- `tdc_gpx_chip_ctrl.vhd:162`
  - `s_drain_cnt_r`는 단순 4-bit 카운터
- `tdc_gpx_chip_ctrl.vhd:499-508`
  - `PH_RESP_DRAIN`은 8클럭 후 무조건 `PH_INIT` 복귀
- `tdc_gpx_chip_ctrl.vhd:406-412`
  - bus tick은 `bus_clk_div`에 따라 여러 system clock마다 한 번만 뜰 수 있음

왜 문제냐:

- bus_phy transaction은 **system clock 기준 8클럭** 안에 끝난다는 보장이 없다.
- 특히 `bus_clk_div`가 크면 read transaction과 deferred response emission이 8클럭보다 늦게 나올 수 있다.
- 그러면 chip_ctrl가 `PH_INIT`로 복귀한 뒤 이전 read의 늦은 응답이 도착할 수 있다.

즉 체크리스트의 **"soft reset stale response 닫힘"**도 현재는 **완전하게 닫힌 상태는 아니다.**

수정 권장:

- fixed 8-cycle drain 대신
  - `bus_phy` idle 관측 신호를 추가하거나,
  - `brsp skid empty && bus busy==0 && rsp_pending==0`을 일정 기간 확인하거나,
  - request/response epoch tag를 넣어 세대가 안 맞는 응답은 버리게 해야 한다.

---

### B3. `tdc_gpx_err_handler.vhd` fatal은 sticky지만, 자동 recovery를 실제로 멈추지 못한다

이건 체크리스트의 **"fatal cleanup 완료"**보다 상황이 덜 닫혀 있다.

문제 라인:

- `tdc_gpx_err_handler.vhd:242-248`
  - fatal 시 `s_err_fatal_r <= '1'`, `s_err_fill_r <= 0`, `s_err_chip_mask_r <= 0`, `s_state_r <= ST_IDLE`
- `tdc_gpx_err_handler.vhd:165-180`
  - `ST_IDLE`에서는 `i_errflag_sync`를 다시 debounce해서 즉시 `ST_READ_REG11`로 재진입 가능
- fatal 경로에서 `s_debounce_cnt_r` / `s_retry_cnt_r`를 clear하거나, `s_err_fatal_r`로 automation을 막는 gate가 없음

왜 문제냐:

- fatal 후에도 ErrFlag가 계속 살아 있으면 다음 cycle부터 recovery FSM이 **자동으로 다시 재기동**될 수 있다.
- 주석은 "reg-access path를 SW 진단용으로 풀어준다"인데, 실제론 persistent ErrFlag 환경에서 manual diagnosis window가 다시 짧아질 수 있다.

수정 권장:

- fatal 후 별도 `ST_FATAL_HOLD`를 두고 SW clear 또는 system reset 전까지 자동 recovery를 막거나,
- 최소한 `if s_err_fatal_r='1' then ST_IDLE debounce 금지`가 필요하다.

---

### B4. `tdc_gpx_cell_builder.vhd`는 output 완료 시 **모든** `BUF_SHARED`를 `BUF_FREE`로 바꾼다

이건 현재 owner FSM에 남아 있는 가장 큰 버그다.

문제 라인:

- `tdc_gpx_cell_builder.vhd:310-316`
  - `s_output_done_r='1'`이면 loop로 `BUF_SHARED`인 버퍼를 전부 `BUF_FREE`로 전환
- `tdc_gpx_cell_builder.vhd:416-419`
  - active collect 중 다른 buffer가 FREE면 새 shot에 할당 가능
- `tdc_gpx_cell_builder.vhd:382-387`
  - 그 새 buffer가 `ififo1_done`을 받으면 `BUF_SHARED`로 승격 가능
- `tdc_gpx_cell_builder.vhd:434-446`
  - output idle일 때 `BUF_SHARED` buffer를 다시 auto-start

왜 문제냐:

- 두 버퍼 구조에서,
  - 버퍼 A: 현재 output 중 (`BUF_SHARED`)
  - 버퍼 B: 다음 shot이 이미 `ififo1_done`까지 와서 대기 중 (`BUF_SHARED`)
  인 상황이 실제로 가능하다.
- 그런데 A output 완료 pulse가 오면 loop가 **A와 B를 둘 다 `BUF_FREE`로 스케줄**한다.
- 이러면 B의 `buf_full`도 같이 clear되고, 대기 중이던 slice를 잃거나 잘못 다시 시작할 수 있다.

즉 체크리스트의 **"cell_builder owner-driven FSM 완료"**는 구조 추가는 맞지만, **정합성까지 완전히 닫힌 건 아니다.**

수정 권장:

- `s_output_done_r` 시에는 **현재 `s_rd_buf_r`에 해당하는 버퍼만** `BUF_FREE`로 내려야 한다.
- loop 전체 free는 제거해야 한다.

---

## C. 남아 있는 권고 사항

### C1. `tdc_gpx_face_seq.vhd`는 여전히 `quiesce_ack` 없이 abort 다음 shot을 허용한다

이건 체크리스트와 동일하게 **아직 미구현**이다.

- `tdc_gpx_face_seq.vhd:361-362`
  - `s_pipeline_abort <= i_face_abort or i_face_fall_abort or i_cmd_stop or i_cmd_soft_reset`
- `tdc_gpx_face_seq.vhd:435-440`
  - abort가 내려간 다음 cycle에는 `s_shot_start_gated`가 다시 열릴 수 있음
- `raw_event_builder/cell_pipe/cell_builder`에는 `i_abort`가 들어가긴 했지만,
  face_seq가 그 stage들의 idle/quiesce를 확인하진 않음

지금은 **저위험 권고** 수준으로 본다.

---

## 최종 판단

최신 업로드를 기준으로 보면:

1. 예전 리뷰에서 지적했던 여러 항목은 **실제로 반영되었다**.
2. 그래서 예전의 "아예 없다" 류 지적 일부는 **현재 업로드본에는 더 이상 맞지 않는다**.
3. 하지만 체크리스트의 **"필수 4개 전부 닫힘"**은 그대로 받아들이기 어렵다.

현재 FSM 재점검 결론은 다음과 같다.

- **정말 해결된 것**
  - `PH_RESP_DRAIN` 추가
  - brsp flush 추가
  - `i_abort` 전파 추가
  - `rsp_mismatch` status 노출
  - owner FSM 골격 추가

- **아직 남아 있는 것**
  - bus_phy burst backpressure 미완성
  - PH_RESP_DRAIN 고정 8클럭 문제
  - fatal 후 auto-recovery 재진입 가능
  - cell_builder output_done가 모든 SHARED buffer를 free함
  - quiesce_ack 미구현

따라서 이 업로드 기준 평가는:

**"이전보다 훨씬 좋아졌지만, FSM freeze 직전 단계라고 단정하기는 아직 이르다."**
