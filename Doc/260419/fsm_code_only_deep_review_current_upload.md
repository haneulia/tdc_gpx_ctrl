
# TDC-GPX 코드 기준 FSM 전수 점검 보고서 (현재 업로드본)

작성 기준:
- **오직 현재 업로드된 VHDL 코드만** 검토함
- 계약 문서 / 요구 문서 / 데이터시트 해석은 판단 근거에서 **제외**
- 관점은 **FSM 상태 전이, timeout/복구, pulse 보존, backpressure, 완료 의미 보존, sticky/status 일관성**
- 라인 번호는 현재 업로드본 기준

---

## 총평

현재 업로드본은 이전 라운드 대비 분명히 좋아졌다.
특히 아래 항목은 실제로 개선되어 있다.

- `tdc_gpx_face_seq.vhd`: `packet_start` / `face_start` / `shot_start` 쪽 one-shot 정리 강화
- `tdc_gpx_config_ctrl.vhd`: `frame_done_both` 및 다수 status CDC 2-FF 동기화 보강
- `tdc_gpx_chip_run.vhd`: pending-stuck watchdog, timeout 경로 `range_active` clear 보강
- `tdc_gpx_cmd_arb.vhd`: dispatch-wait watchdog 및 same-cycle new request 흡수 보강
- `tdc_gpx_chip_init.vhd`: deferred cfg_write snapshot 보존 보강
- `tdc_gpx_header_inserter.vhd`: non-idle `face_start` queue, drain watchdog 추가
- `tdc_gpx_cell_builder.vhd`: IFIFO2-timeout synthetic `tlast`, QUARANTINE escape sticky 추가

그럼에도 불구하고, **FSM이 완전히 안정화되었다고 보기는 아직 어렵다.**
현재 기준으로는 **“치명적 버그가 많이 남았다”기보다는, 경계조건/복구/관찰성/의미 보존이 아직 거칠게 남아 있는 상태”**에 가깝다.

이번 점검에서 정리한 항목 수:
- 높음: 9개
- 중간: 10개
- 낮음/유지보수/관찰성: 7개
- **총 26개**

---

## 가장 먼저 수정해야 할 핵심 8개

1. **`chip_ctrl` raw FIFO 포화 시 control beat도 여전히 drop 가능**  
   파일: `tdc_gpx_chip_ctrl.vhd:938-1015`  
   reserve 정책이 강화되긴 했지만, FIFO가 완전히 포화되면 `drain_done` / `ififo1_done_beat` 같은 control beat도 drop된다. 이건 raw data 손실보다 훨씬 위험하다. FSM 완료 의미 자체가 사라질 수 있다.

2. **`chip_ctrl`의 `PH_RESP_DRAIN`은 stuck bus에서 사실상 영구 quarantine**  
   파일: `tdc_gpx_chip_ctrl.vhd:730-779`, `786-809`  
   bus가 계속 active/pending이면 hard-cap 이후 `PH_RESP_DRAIN`에 머물며 saturate한다. in-band 자가 복구가 닫혀 있고, 사실상 `i_cmd_force_reinit` 또는 하드 리셋 의존이다.

3. **`chip_run` drain mismatch를 발견해도 정상 completion처럼 끝낼 수 있음**  
   파일: `tdc_gpx_chip_run.vhd:516-540`  
   fallback completion 경로에서 expected/actual drain count mismatch를 sticky로만 남기고 `drain_done`을 내보낸다. 데이터 정합성 실패가 “구조상 정상 종료”로 보일 수 있다.

4. **`stop_cfg_decode` orphan stop 감지가 “리셋 후 첫 shot 이전”만 잡음**  
   파일: `tdc_gpx_stop_cfg_decode.vhd:86-88`, `110-123`  
   `s_shot_ever_seen_r`는 첫 `shot_start` 후 영구 1이 된다. 그래서 그 이후 shot 사이 빈 구간에 잘못 들어온 stop event는 orphan으로 잡지 못한다.

5. **`header_inserter` watchdog은 wedged sink를 풀어주지만, 대가로 beat를 강제 폐기**  
   파일: `tdc_gpx_header_inserter.vhd:521-533`, `542-551`  
   `ST_DRAIN_LAST`, `ST_ABORT_DRAIN` timeout 시 `tvalid`를 강제로 내리고 IDLE로 복귀한다. 가용성은 살리지만, 이건 AXI 관점에서 구조적 truncation이다.

6. **`face_assembler`와 `cell_builder`가 truncation을 구조적으로 정상 종료 형태로 바꿔버림**  
   파일: `tdc_gpx_face_assembler.vhd:772-849`, `tdc_gpx_cell_builder.vhd:871-882`  
   blank-fill / synthetic final beat는 상위 데드락을 막는 장점이 있지만, SW가 sticky를 놓치면 손실된 shot/row가 정상 완결로 보일 수 있다.

7. **sticky / soft_clear / timeout status 의미가 모듈마다 다름**  
   파일: `tdc_gpx_chip_reg.vhd:255-262`, `tdc_gpx_err_handler.vhd:198-217`, `tdc_gpx_cmd_arb.vhd:317-322`, `399-401`  
   어떤 sticky는 soft_clear에 지워지고, 어떤 sticky는 transaction 시작 시 지워지고, 어떤 sticky는 하드리셋 전까지 유지된다. SW/상위 FSM이 상태 의미를 통일해서 해석하기 어렵다.

8. **`chip_ctrl` live `stopdis_override`는 여전히 mid-shot FSM 의미를 깨뜨릴 수 있는 escape hatch**  
   파일: `tdc_gpx_chip_ctrl.vhd:694-699`, `864-884`  
   sticky는 좋아졌지만, 핀 구동을 FSM 내부 상태와 분리하는 debug override 자체는 여전히 위험하다.

---

## 상세 항목

### 높음

### H-1. raw FIFO 포화 시 control beat drop 가능
- 파일: `tdc_gpx_chip_ctrl.vhd:938-1015`
- 내용: control beat(`tuser(7)='1'`)는 data보다 우선 보호되지만, 완전 포화에서는 결국 drop된다.
- 영향: `drain_done`, `ififo1_done_beat` 같은 FSM completion/boundary 의미가 사라질 수 있다.
- 권고: control/data 경로 분리 또는 control 전용 1~2 entry 별도 FIFO 추가.

### H-2. `PH_RESP_DRAIN` stuck 시 영구 quarantine 성격
- 파일: `tdc_gpx_chip_ctrl.vhd:730-779`
- 내용: drain cap 이후 bus가 계속 active/pending이면 `s_err_drain_cap_r`만 세우고 머문다.
- 영향: soft-reset만으로는 같은 문제를 반복할 수 있고, 자동 복구가 닫힌다.
- 권고: “영구 quarantine” 정책을 유지하더라도, 상위 SW/driver가 반드시 처리해야 한다는 명시적 fatal/severity 승격 필요.

### H-3. soft reset이 stuck bus를 근본 복구하지 못할 수 있음
- 파일: `tdc_gpx_chip_ctrl.vhd:786-794`
- 내용: soft reset도 다시 `PH_RESP_DRAIN`으로 보내는 구조라 bus가 계속 stuck이면 다시 같은 drain quarantine으로 들어간다.
- 영향: recovery FSM이 진행했는데 실질적 bus fault는 안 풀릴 수 있다.
- 권고: force-reinit보다 더 강한 bus-fault escalation 또는 상위 fatal reporting 추가.

### H-4. `chip_run` fallback drain completion이 mismatch에도 성공처럼 종료
- 파일: `tdc_gpx_chip_run.vhd:516-540`
- 내용: expected vs actual mismatch를 `s_err_drain_mismatch_r`로만 남기고 `s_drain_done_r`를 발생시킨다.
- 영향: 상위가 done만 보고 넘어가면 손실을 정상 completion으로 오인한다.
- 권고: mismatch 시 별도 fatal/abort completion code를 내거나 최소한 done과 별개로 “invalid done” bit를 추가.

### H-5. `chip_run` timeout cause 코드가 서로 다른 failure mode를 뭉개는 곳이 있음
- 파일: `tdc_gpx_chip_run.vhd:575-583`, `616-624`, `663-672`, `715-721`
- 내용: downstream deadlock성 pending-stuck도 `"001"`로 기록된다.
- 영향: raw_busy 자체 timeout과 pending-held deadlock을 SW가 구분하기 어렵다.
- 권고: pending-stuck 전용 cause code 분리.

### H-6. `stop_cfg_decode` orphan stop 검출이 shot-gap violation을 다 못 잡음
- 파일: `tdc_gpx_stop_cfg_decode.vhd:86-88`, `110-123`
- 내용: orphan sticky는 “첫 shot 이전 stop”만 잡는다. 첫 shot 이후에는 shot 간 빈 구간 stop도 통과한다.
- 영향: upstream format drift가 후반부 shot에서 발생해도 조용히 expected count가 오염될 수 있다.
- 권고: `shot_active` 개념을 별도로 두고 shot window 밖 stop event도 잡아야 한다.

### H-7. `header_inserter` timeout 복구는 구조적으로 beat 폐기를 동반
- 파일: `tdc_gpx_header_inserter.vhd:521-533`, `542-551`
- 내용: wedged sink 회피를 위해 pending beat를 drop하고 IDLE로 복귀한다.
- 영향: frame/line 정합성은 살려도 전송 완전성은 깨진다.
- 권고: sticky만으로 끝내지 말고 상위 frame invalid bit까지 함께 남겨야 한다.

### H-8. blank-fill / synthetic-EOS가 손실을 “정상 모양의 데이터”로 바꿈
- 파일: `tdc_gpx_face_assembler.vhd:772-849`, `tdc_gpx_cell_builder.vhd:871-882`
- 내용: overrun/timeout을 구조적 completion으로 바꿔 데드락을 막는다.
- 영향: status를 놓치면 SW가 부분 손실 frame을 정상 frame으로 읽을 수 있다.
- 권고: face/header 쪽에 명시적인 invalid-frame bit 또는 footer error marker 추가 검토.

### H-9. `stopdis_override`는 debug용이라도 mid-shot에서 위험
- 파일: `tdc_gpx_chip_ctrl.vhd:694-699`, `864-884`
- 내용: live override가 FSM 내부 state와 독립적으로 핀을 움직인다.
- 영향: capture window/stop gating 의미가 깨질 수 있다.
- 권고: 실운영 빌드에서는 compile-time disable 또는 arm-safe-state 조건 추가.

---

### 중간

### M-1. control/data가 같은 raw FIFO를 공유
- 파일: `tdc_gpx_chip_ctrl.vhd:938-1015`
- 내용: reserve가 있어도 control과 data의 완전 분리는 아니다.
- 영향: data burst가 크면 control latency/안전성이 구조적으로 data backlog에 종속된다.
- 권고: 최소한 control beat를 sideband FIFO로 분리.

### M-2. `o_drain_done`와 `o_run_drain_complete` 의미가 다름
- 파일: `tdc_gpx_chip_ctrl.vhd:1027-1056`
- 내용: 하나는 downstream handshake 기준, 하나는 run-engine internal 기준이다.
- 영향: 소비자가 잘못 선택하면 FSM 정렬 타이밍이 어긋난다.
- 권고: 둘 중 하나를 rename하거나, top/status에 둘의 semantic 차이를 강하게 드러내기.

### M-3. `cmd_arb` timeout sticky는 “historical sticky”가 아니라 “last transaction status”
- 파일: `tdc_gpx_cmd_arb.vhd:317-322`, `399-401`
- 내용: 정상 completion 또는 신규 transaction accept에서 timeout status가 지워진다.
- 영향: 다른 모듈 sticky와 수명이 달라 SW가 동일한 방식으로 읽기 어렵다.
- 권고: historical sticky와 last-status를 분리.

### M-4. `cmd_arb` overlapping reg request queue는 1-depth뿐
- 파일: `tdc_gpx_cmd_arb.vhd:460-491`
- 내용: active 중 들어온 다음 요청 1개만 보존하고, 그 이후는 reject sticky로 끝난다.
- 영향: burst reg access나 드라이버 실수 시 요청 손실.
- 권고: 2~4 depth 소형 FIFO 또는 “busy면 NACK/holdoff”를 명확히 규정.

### M-5. `chip_init` deferred cfg_write도 1-depth coalesce
- 파일: `tdc_gpx_chip_init.vhd:106-118`, `321-340`
- 내용: busy window에서 여러 cfg_write는 1 snapshot으로 collapse된다.
- 영향: 중간 요청 이미지가 사라진다.
- 권고: 현재 정책을 유지하더라도 top/status에서 coalesced count를 더 명확히 노출.

### M-6. `chip_reg` concurrent read+write sticky는 soft_clear 대상이 아님
- 파일: `tdc_gpx_chip_reg.vhd:212-215`, `255-262`
- 내용: `s_err_rw_ambiguous_r`는 set되지만 soft_clear로 안 지워진다. 반면 overflow sticky는 soft_clear로 지워진다.
- 영향: sticky 수명 규칙이 모듈 내부에서도 일관되지 않다.
- 권고: 두 sticky 모두 동일 clear policy로 맞추기.

### M-7. `err_handler`는 soft_clear 시 FSM까지 즉시 리셋
- 파일: `tdc_gpx_err_handler.vhd:198-217`
- 내용: soft_clear가 단순 sticky clear가 아니라 recovery FSM state까지 ST_IDLE로 돌린다.
- 영향: 다른 모듈의 “sticky clear only”와 의미가 다르다.
- 권고: `soft_clear_status`와 `soft_abort_recovery`를 분리 검토.

### M-8. `status_agg` error counter는 이벤트 수가 아니라 error-active cycle 수
- 파일: `tdc_gpx_status_agg.vhd:105-133`
- 내용: 여러 동시 에러가 있어도 1 cycle당 +1만 된다.
- 영향: SW가 “에러 발생 건수”로 읽으면 해석이 틀린다.
- 권고: counter 이름 또는 CSR 문구를 cycle semantics로 명확히.

### M-9. `cell_builder` QUARANTINE force-escape 뒤에도 late stale beat 가능성은 완전 제거되지 않음
- 파일: `tdc_gpx_cell_builder.vhd:356-368`, `628-676`
- 내용: post-escape absorb window가 추가됐지만, 그 창 이후 더 늦게 오는 stale beat는 다시 IDLE `tready=0`에 걸릴 수 있다.
- 영향: 극단적 지연 상황에서 upstream stall 재발 가능성.
- 권고: quarantine generation tag 또는 explicit flush marker 재동기화 검토.

### M-10. `cell_builder` synthetic timeout EOS는 데드락은 막지만 내용은 zero-fill
- 파일: `tdc_gpx_cell_builder.vhd:871-882`
- 내용: IFIFO2 미도착 시 zero final beat + `tlast`.
- 영향: 구조는 맞아도 실제 payload completeness는 깨진다.
- 권고: payload 안에 explicit timeout/error bit를 인코딩 가능하면 추가.

---

### 낮음 / 유지보수 / 관찰성

### L-1. `face_assembler` `o_face_abort`는 사실상 dead port
- 파일: `tdc_gpx_face_assembler.vhd:122-139`, `884-888`
- 내용: 주석으로 deprecated임을 명시했지만 인터페이스에 아직 남아 있다.
- 영향: 유지보수 혼선, top/output_stage/testbench 불필요 결합.
- 권고: 다음 정리 라운드에서 인터페이스 제거.

### L-2. `face_assembler` shot-flush 주석 일부가 현재 구현과 어긋남
- 파일: `tdc_gpx_face_assembler.vhd:410-414`
- 내용: 주석은 “per-chip mask was not added” 뉘앙스인데 실제로는 `s_shot_flush_drop_mask_r`가 존재한다.
- 영향: 코드 독해 혼선.
- 권고: stale comment 정리.

### L-3. `face_seq` abort gating은 rise-primary 비대칭 정책
- 파일: `tdc_gpx_face_seq.vhd:621-646`
- 내용: fall-only abort는 `shot_start`/`face_start` gating을 막지 않는다.
- 영향: 의도된 정책일 수 있지만 rise/fall path 해석이 대칭적이지 않다.
- 권고: 이 비대칭성을 CSR/status 문서와 주석에 더 강하게 남기기.

### L-4. `header_inserter` face_start queue는 2-depth라 burst는 여전히 collapse
- 파일: `tdc_gpx_header_inserter.vhd:168-182`, `589-598`, `606-613`
- 내용: 2개 초과 pulse는 collapsed counter로만 보인다.
- 영향: 손실이 없어지는 것이 아니라 “관찰 가능한 collapse”로 바뀐 것.
- 권고: upstream가 pulse burst를 만들 수 있다면 ack-based handshake화.

### L-5. `chip_run` `s_err_drain_mismatch_r` clear 시점이 매우 보수적
- 파일: `tdc_gpx_chip_run.vhd:269-297`, `524-535`, `900-901`
- 내용: 현재는 하드 리셋에서만 확실히 초기화된다.
- 영향: “historical sticky”로는 괜찮지만 다른 per-shot status와 수명 차이가 크다.
- 권고: historical sticky와 last-shot mismatch를 분리하면 해석이 쉬워진다.

### L-6. `stop_cfg_decode` monotonic violation은 IFIFO1/2 세부 원인 분리가 없음
- 파일: `tdc_gpx_stop_cfg_decode.vhd:124-137`
- 내용: chip 단위 mask만 남고 IFIFO1/2 어느 쪽이 감소했는지는 잃는다.
- 영향: 원인 추적성 부족.
- 권고: IFIFO1/2 분리 mask 추가.

### L-7. `cmd_arb`/`chip_init`/`chip_reg` 계열은 전반적으로 “손실 대신 sticky” 철학
- 파일: `tdc_gpx_cmd_arb.vhd:460-491`, `tdc_gpx_chip_init.vhd:321-340`, `tdc_gpx_chip_reg.vhd:221-229`
- 내용: 여러 overlap 조건에서 큐잉보다 collapse/reject/sticky를 택한다.
- 영향: 구조는 단순하지만, driver가 조금만 계약을 어기면 명령 손실이 실제로 생긴다.
- 권고: SW 계약을 강하게 둘 거면 CSR/status 노출과 에러 처리 루틴을 같이 정비해야 한다.

---

## 파일별 우선순위

### 1순위
- `tdc_gpx_chip_ctrl.vhd`
- `tdc_gpx_chip_run.vhd`
- `tdc_gpx_stop_cfg_decode.vhd`

### 2순위
- `tdc_gpx_header_inserter.vhd`
- `tdc_gpx_face_assembler.vhd`
- `tdc_gpx_cell_builder.vhd`

### 3순위
- `tdc_gpx_cmd_arb.vhd`
- `tdc_gpx_chip_reg.vhd`
- `tdc_gpx_chip_init.vhd`
- `tdc_gpx_err_handler.vhd`
- `tdc_gpx_status_agg.vhd`

---

## 수정 순서 제안

1. **완료 의미 보존**
   - `chip_ctrl` control/data 분리 또는 control 전용 보호 경로 추가
   - `chip_run` mismatch를 단순 done이 아닌 “faulted completion”으로 승격
   - `stop_cfg_decode` orphan window 검출 강화

2. **복구 경로 명확화**
   - `PH_RESP_DRAIN` stuck bus 처리 정책을 “영구 quarantine”인지 “강제 fatal reset”인지 명확화
   - `header_inserter` / `cell_builder` / `face_assembler` truncation 시 invalid-frame 성격 명시

3. **status 수명 통일**
   - historical sticky
   - last-transaction status
   - soft_clear 대상
   - hard-reset only 대상
   이 4개를 모듈 전반에 걸쳐 통일

4. **인터페이스 정리**
   - dead port / stale comment 제거
   - 비대칭 policy(rise-primary 등) 명시 강화

---

## 한 줄 결론

**현재 코드는 “돌아가게 만드는 복구”는 많이 들어갔지만, 그 복구가 종종 손실을 정상 completion 형태로 바꿔서 드러내는 구조다.**  
그래서 지금 남은 핵심 과제는 **데드락 방지**가 아니라, **손실/불일치를 얼마나 분명하게 fault로 드러내고 완료 의미를 보존하느냐** 쪽이다.
