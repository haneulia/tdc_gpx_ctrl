# TDC-GPX FSM 코드 정합성 재점검 보고서 (코드 기준만)

검토 기준은 **업로드된 RTL/VHDL 코드만**이다. 계약 문서, 요구사항 문서, 데이터시트 해석, 이전 검토 메모는 판단 근거로 쓰지 않았다. 아래 항목은 전부 **현재 업로드된 코드에서 직접 확인된 FSM/상태성 로직**만 대상으로 정리했다.

---

## 1. 총평

현재 상태는 “기본 동작은 가능하지만, 경계조건·동시 이벤트·복구 경로·버퍼 포화 조건에서 FSM이 깨질 가능성이 남아 있는 상태”로 보는 것이 맞다.

특히 위험한 축은 다음 다섯 가지다.

1. **복구 FSM 종료 조건 불일치** (`config_ctrl` ↔ `err_handler`)
2. **멀티칩 reg-access timeout 설계** (`cmd_arb`)
3. **raw/control beat 유실 가능성** (`chip_ctrl`)
4. **shot drop 시 입력 흡수 실패** (`cell_builder`)
5. **동시 이벤트 우선순위가 데이터 보존보다 recovery를 우선** (`chip_run`, `face_assembler`, `header_inserter`)

---

## 2. 우선순위 요약

### 치명 / 즉시 수정 권고

1. `config_ctrl`가 `err_handler`에 `i_frame_done => i_frame_done and i_frame_fall_done`를 연결함. rise/fall frame_done이 다른 cycle에 나오면 recovery FSM이 영원히 `ST_WAIT_FRAME_DONE`에서 못 빠져나올 수 있다.  
   - 근거: `tdc_gpx_config_ctrl.vhd:535-549`, `tdc_gpx_err_handler.vhd:291-306`

2. `err_handler`의 `ST_WAIT_READ`에는 timeout이 없다. `i_cmd_reg_done_pulse`가 안 오면 복구 FSM이 영구 정지한다.  
   - 근거: `tdc_gpx_err_handler.vhd:214-241`

3. `cmd_arb`의 global timeout은 **요청이 실제 dispatch되었는지와 무관하게** `s_reg_active_r='1'`이면 즉시 카운트된다. busy인 chip은 아직 시작도 못 했는데 timeout budget을 먼저 소모한다.  
   - 근거: `tdc_gpx_cmd_arb.vhd:202-215`, `255-297`

4. `cell_builder`가 `ST_C_IDLE`에서 free buffer가 없을 때 `s_shot_dropped_r`만 올리고 끝난다. 그런데 `o_s_axis_tready`는 `ST_C_ACTIVE`/`ST_C_DROP`에서만 1이므로, “drop”이라고 해놓고 실제로는 입력을 먹지 못해 upstream stall/hang을 유발할 수 있다.  
   - 근거: `tdc_gpx_cell_builder.vhd:283-285`, `355-375`

5. `chip_ctrl`의 hold+skid 2-depth가 모두 찬 상태에서 새 beat가 오면 그냥 drop되고 `s_err_raw_overflow_r`만 set된다. 이 beat는 raw data뿐 아니라 `drain_done`/`ififo1_done` control beat일 수도 있다.  
   - 근거: `tdc_gpx_chip_ctrl.vhd:675-759`

6. `chip_run`은 `i_shot_start`와 bus response가 같은 cycle에 오면 overrun recovery가 우선하고 response beat는 버려질 수 있다고 코드 주석으로 명시해 두었다. 이는 의도라고 적혀 있어도 실제로는 drain accounting과 데이터 보존에 매우 위험하다.  
   - 근거: `tdc_gpx_chip_run.vhd:621-653`

7. `chip_init`이 cfg_write/master-reset timeout으로 빠질 때 `s_stopdis_r`를 low로 복구하지 않고 `ST_OFF`로 간다. 실패 이후 chip이 stop disabled 상태에 남을 수 있다.  
   - 근거: `tdc_gpx_chip_init.vhd:168-190`, `203-237`

### 높음 / 빠른 수정 권고

8. `chip_run`의 deferred stop latch는 `ST_OVERRUN_FLUSH`를 포함하지 않는다. 이 상태에서 1-cycle `i_cmd_stop`이 들어오면 유실될 수 있다.  
   - 근거: `tdc_gpx_chip_run.vhd:230-242`, `594-617`

9. `chip_ctrl`의 `PH_RESP_DRAIN`은 `i_bus_busy`/`i_bus_rsp_pending`가 계속 남아 있어도 15 cycle 뒤 강제 종료한다. stale response가 실제로 남아 있을 때 다음 phase를 오염시킬 수 있다.  
   - 근거: `tdc_gpx_chip_ctrl.vhd:566-587`

10. `err_handler`는 `i_rst_n`으로만 `s_err_fatal_r`를 지운다. 소프트 리셋 경로가 모듈 입력에 없으므로 fatal 상태가 소프트 리셋으로 복구되지 않는다.  
   - 근거: `tdc_gpx_err_handler.vhd:32-64`, `255-279`

11. `stop_cfg_decode`는 `i_stop_evt_tvalid`가 여러 번 오면 expected IFIFO count를 누적하지 않고 **매 cycle overwrite**한다. stop event가 “shot당 1 beat aggregate”라는 강한 계약에 의존한다.  
   - 근거: `tdc_gpx_stop_cfg_decode.vhd:53-79`

12. `chip_ctrl`의 `s_err_raw_overflow_r`는 내부 sticky로만 남고 외부 status/port로 노출되지 않는다. SW가 실제 data/control beat 유실을 관찰할 수 없다.  
   - 근거: `tdc_gpx_chip_ctrl.vhd:275`, `703-759`, 출력부 전체

13. `chip_ctrl`의 `i_stop_tdc`는 “다른 domain에서 오는 1-clk pulse”라고 주석이 적혀 있는데, 구현은 2-FF sync뿐이다. 현재 clock ratio에서는 대개 잡히겠지만, generic CDC handshake로는 안전하지 않다.  
   - 근거: `tdc_gpx_chip_ctrl.vhd:73-81`, `611-649`

14. `header_inserter`는 non-IDLE 상태의 `face_start`를 하드웨어에서 그냥 무시한다. 시뮬 assertion만 있고 실제 보호/queue가 없다.  
   - 근거: `tdc_gpx_header_inserter.vhd:452-465`

15. `header_inserter` reset에서 `s_hdr_rom_pending_r`, `s_hdr_rom_r`를 명시 초기화하지 않는다. 선언 초기값에 기대고 있어 runtime reset과 power-up reset semantics가 다를 수 있다.  
   - 근거: `tdc_gpx_header_inserter.vhd:136`, `187`, `298-327`

16. `cell_builder` reset에서 `s_buf_seq_r`, `s_buf_age_r`를 명시 초기화하지 않는다. age-based arbitration이 runtime reset 직후 이전 값에 영향받을 수 있다.  
   - 근거: `tdc_gpx_cell_builder.vhd:165-167`, `301-313`

17. `cell_builder` reset에서 `s_rt_last_beat_r`, `s_rt_max_hits_r`도 명시 초기화하지 않는다. 선언 초기값에 의존한다.  
   - 근거: `tdc_gpx_cell_builder.vhd:213-214`, `551-563`

18. `face_assembler`는 self-overrun 시 즉시 `ST_IDLE`로 가고 `o_face_abort`를 pulse한다. 내부/외부 FIFO flush는 top의 `pipeline_abort`가 다음 cycle에 와야 정리된다. 1-cycle feedback timing에 강하게 의존한다.  
   - 근거: `tdc_gpx_face_assembler.vhd:620-640`, `tdc_gpx_output_stage.vhd:160-167`

19. `face_assembler`는 `i_active_chip_mask="0000"`이면 `ST_IDLE`로 바로 복귀하고 `row_done`를 내지 않는다. 현재는 `face_seq`가 zero-mask start를 막고 있지만, 모듈 단독으로는 self-consistent하지 않다.  
   - 근거: `tdc_gpx_face_assembler.vhd:453-459`, `tdc_gpx_face_seq.vhd:167-190`

20. `chip_reg`는 `i_start_read`와 `i_start_write`가 같은 cycle이면 read가 우선되고 write는 유실된다. queue가 없다.  
   - 근거: `tdc_gpx_chip_reg.vhd:100-128`

21. `chip_init`의 `ST_OFF`도 `i_start`와 `i_cfg_write_req`가 동시에 오면 `i_start` 우선, cfg_write는 유실된다.  
   - 근거: `tdc_gpx_chip_init.vhd:139-147`

22. `face_seq`는 slope 한쪽 assembler의 abort만 들어와도 `s_pipeline_abort`로 rise/fall 양쪽 전체를 동시에 죽인다. 설계 의도일 수는 있지만 healthy 쪽 데이터도 같이 버린다.  
   - 근거: `tdc_gpx_face_seq.vhd:385-387`

23. `bus_phy` burst continuation은 latched `s_req_burst_r`가 아니라 live `i_req_burst`를 본다. 현재는 `chip_run`이 burst stop 제어를 live로 하려고 의도했지만, bus FSM이 순수 snapshot 방식이 아니라 외부 live control에 묶인다.  
   - 근거: `tdc_gpx_bus_phy.vhd:559-562`

24. `chip_ctrl`의 StopDis 출력은 live override를 mid-run에도 즉시 반영한다. debug/emergency 의도는 맞지만, shot 중간에 내부 FSM과 독립적으로 핀 상태가 바뀔 수 있다.  
   - 근거: `tdc_gpx_chip_ctrl.vhd:657-665`

### 중간 / 구조 취약 / 개선 권고

25. `cell_builder`의 `ST_C_DROP` timeout은 final drain_done 없이도 강제로 `ST_C_IDLE`로 복귀한다. 이후 늦게 도착한 stale beat는 IDLE에서 tready=0이라 upstream path를 다시 막을 수 있다.  
   - 근거: `tdc_gpx_cell_builder.vhd:484-501`

26. `cell_builder`의 shot handling은 `ST_C_ACTIVE`에서는 DROP 상태로 진입하며 입력을 흡수하지만, `ST_C_IDLE`의 no-free-buffer 경로는 그렇지 않다. 같은 “shot drop”인데 동작 모델이 두 군데서 다르다.  
   - 근거: `tdc_gpx_cell_builder.vhd:460-482`, `355-375`

27. `chip_ctrl`의 `o_drain_done`는 실제 downstream handshake 때만 발생한다. 즉, chip_run 측 drain complete가 downstream backpressure 때문에 지연된다. 이는 data/control beat를 같은 AXIS에 실은 현재 구조의 의도이지만, 상위 모듈은 이를 “chip 내부 완료”가 아니라 “downstream accept 완료”로 해석해야 한다.  
   - 근거: `tdc_gpx_chip_ctrl.vhd:773`

28. `chip_run`의 expected IFIFO count는 `ST_DRAIN_LATCH`에서 live input을 그냥 샘플한다. `stop_cfg_decode`로부터 “finalized/valid” handshake가 없어서 shot boundary timing 계약에 의존한다.  
   - 근거: `tdc_gpx_chip_run.vhd:305-312`, `tdc_gpx_stop_cfg_decode.vhd:53-79`

29. `chip_run`의 `ST_CAPTURE`에서 stop 명령이 오면 purge mode로 drain recovery로 전환한다. 이때 이미 들어온 stop data와 pending bus 응답의 보존보다 stop 우선이다. 설계 의도일 수 있으나, “stop은 graceful finish”가 아니라 “강한 purge” semantics다.  
   - 근거: `tdc_gpx_chip_run.vhd:273-291`

30. `face_seq`의 `s_packet_start`는 combinational pulse다. 입력 조건이 많고 (`shot_start_raw`, `deferred`, `hdr_idle`, abort 조건 등) 같은 cycle 안정성에 의존한다. registered one-shot이 유지보수에 더 안전하다.  
   - 근거: `tdc_gpx_face_seq.vhd:375-383`

31. `face_seq`의 `s_shot_overrun_r`는 새 `cmd_start`가 오기 전까지 유지된다. face 단위 통계로는 유용하지만, face 간에 clear되지 않아 상태 해석이 다소 거칠다.  
   - 근거: `tdc_gpx_face_seq.vhd:160-163`, `167-190`

32. `face_seq`는 busy 상태에서 들어온 `cmd_start`를 그냥 무시한다. `cfg_rejected`와도 구분된다. SW가 pulse를 놓치기 쉽다.  
   - 근거: `tdc_gpx_face_seq.vhd:167-190`

33. `face_seq`의 shot deferral 로직은 꽤 정교하지만, `s_shot_drop_cnt_r`가 증가하는 경우에도 해당 사실이 외부 interface에 직접 노출되지 않는다. shot drop 분석성이 떨어진다.  
   - 근거: `tdc_gpx_face_seq.vhd:421-460`

34. `header_inserter`는 `face_start`와 `face_abort`가 같은 cycle에 겹치면 metadata latch 후 abort가 마지막에 이긴다. 이 경우 `s_hdr_rom_pending_r`가 남을 수 있는지 코드 독해 난도가 높다. 명시적 abort cleanup이 더 안전하다.  
   - 근거: `tdc_gpx_header_inserter.vhd:460-529`, `547-555`

35. `header_inserter` header 오류 정보는 face_start 시점 snapshot이다. 같은 face drain 중 생긴 에러는 다음 header에 반영된다. 설계 의도라고 주석에 적혀 있지만, status/header의 시간 일관성은 떨어진다.  
   - 근거: `tdc_gpx_header_inserter.vhd:469-499`

36. `face_assembler` self-overrun은 synthetic tlast를 만들지 않고 partial row를 truncate한다. 주석에도 그렇게 되어 있다. AXIS 규약 위반은 아니지만, downstream/VDMA 입장에서는 line integrity를 잃은 frame이 된다. SW discard 경로가 반드시 필요하다.  
   - 근거: `tdc_gpx_face_assembler.vhd:610-640`, `tdc_gpx_header_inserter.vhd:536-555`

37. `chip_reg`는 active 동안 들어오는 새 read/write pulse를 저장하지 않는다. register access가 pulse 기반이라서 upper layer가 exact one-cycle 타이밍을 맞춰야 한다.  
   - 근거: `tdc_gpx_chip_reg.vhd:100-150`

38. `config_ctrl`는 err_handler 활성 시 manual reg path를 강제로 err_handler에 넘긴다. recovery가 길어지거나 stuck되면 SW manual reg access도 굶을 수 있다.  
   - 근거: `tdc_gpx_config_ctrl.vhd:415-424`

39. `status_agg`의 `o_error_count`는 이름과 달리 “error event 수”가 아니라 “error-active cycle 수”다. 주석은 정확하지만 인터페이스 이름만 보면 오해하기 쉽다.  
   - 근거: `tdc_gpx_status_agg.vhd:90-116`

40. `status_agg` sticky error는 soft reset에서도 안 지워지고 hard reset에서만 clear된다. 히스토리 보존 의도는 맞지만, runtime 운용에서는 “이미 처리한 과거 에러”와 “현재 유효 에러”가 섞여 보일 수 있다.  
   - 근거: `tdc_gpx_status_agg.vhd:121-136`

---

## 3. 모듈별 상세 코멘트

### 3.1 `tdc_gpx_chip_run.vhd`

- 좋은 점
  - timeout cause를 별도 코드로 보존한다.
  - `range_active`는 현재 코드 기준으로 대부분의 정상/timeout 완료 경로에서 정리된다.
  - purge 모드와 normal drain을 분리했다.

- 핵심 우려
  - overrun recovery가 데이터 보존보다 항상 우선이다.
  - stop defer 대상 상태에 `ST_OVERRUN_FLUSH`가 빠져 있다.
  - expected IFIFO count snapshot에 finalized handshake가 없다.

### 3.2 `tdc_gpx_chip_ctrl.vhd`

- 좋은 점
  - raw hold/skid 2-depth로 1-cycle 늦은 busy backpressure를 흡수하려는 구조는 합리적이다.
  - response drain phase를 따로 둔 점도 방향은 맞다.

- 핵심 우려
  - 2-depth가 가득 차면 raw/control beat drop.
  - overflow status가 외부에 안 보인다.
  - cross-domain pulse를 2FF만으로 잡는다.
  - `PH_RESP_DRAIN` 종료 조건이 너무 공격적이다.

### 3.3 `tdc_gpx_cell_builder.vhd`

- 좋은 점
  - active 상태의 shot overlap 처리와 DROP 모드는 이전보다 훨씬 안정적이다.
  - IFIFO2 timeout 시 synthetic `tlast`를 만들어 상위 assembler deadlock을 막는다.

- 핵심 우려
  - IDLE no-free-buffer drop 경로가 입력을 실제로 먹지 못한다.
  - runtime reset completeness가 부족하다.
  - DROP timeout 뒤 stale beat 처리 모델이 애매하다.

### 3.4 `tdc_gpx_face_assembler.vhd`

- 좋은 점
  - active chip 순서를 강제하고, blank generation과 in-order forwarding을 깔끔히 분리했다.
  - row_done와 row tlast 생성 논리가 비교적 명확하다.

- 핵심 우려
  - self-overrun abort가 partial line truncation을 의도한다.
  - internal cleanup이 top feedback timing에 의존한다.
  - zero active mask는 모듈 단독으로 self-complete되지 않는다.

### 3.5 `tdc_gpx_face_seq.vhd`

- 좋은 점
  - rise/fall frame_done combiner는 OR-latch 방식으로 안정적이다.
  - face별 geometry latch와 rows/hsize 계산 구조는 정리돼 있다.

- 핵심 우려
  - `packet_start`가 combinational이다.
  - 한쪽 abort가 양쪽 전체 abort로 확대된다.
  - busy 중 cmd_start drop, shot drop 분석성 부족.

### 3.6 `tdc_gpx_header_inserter.vhd`

- 좋은 점
  - prefix/header/data/drain-last/abort-drain으로 상태를 잘 나눴다.
  - pending beat를 handshake시킨 후 abort하는 점은 AXIS 관점에서 맞다.

- 핵심 우려
  - non-IDLE `face_start` 무시.
  - reset completeness 부족.
  - abort와 start 동시 cycle의 명시적 cleanup 부재.

### 3.7 `tdc_gpx_err_handler.vhd`

- 좋은 점
  - debounce → Reg12 read → recovery → frame/shot boundary resync 흐름은 방향이 좋다.
  - fatal 상태에서 `err_active=0`로 풀어 SW diagnosis를 가능하게 한 점도 의도가 명확하다.

- 핵심 우려
  - `ST_WAIT_READ` no-timeout.
  - soft reset으로 fatal clear가 안 된다.
  - frame_done 입력이 잘못 연결되면 복구 FSM 전체가 영구 대기한다.

### 3.8 `tdc_gpx_cmd_arb.vhd`

- 좋은 점
  - multi-chip 병렬 dispatch 구조는 잘 잡혀 있다.
  - pending chip을 후속 cycle에 dispatch하는 구조도 있다.

- 핵심 우려
  - timeout이 “dispatch 후 경과시간”이 아니라 “global active 기간” 기준이다.
  - simultaneous read/write arbitration과 pulse drop 보호가 약하다.

### 3.9 `tdc_gpx_chip_init.vhd`, `tdc_gpx_chip_reg.vhd`, `tdc_gpx_bus_phy.vhd`

- `chip_init`
  - init/config-write FSM 자체는 단순하고 이해하기 쉽다.
  - 다만 timeout 실패 후 pin-safe-state 복귀가 미완성이다.

- `chip_reg`
  - 2-state FSM이라 단순하지만, pulse-based request 유실 가능성이 있다.

- `bus_phy`
  - tick_en 기준 전이, response pending 처리, burst high-time 보장 의도는 좋다.
  - 하지만 burst continuation이 live control(`i_req_burst`)에 묶여 있어 모듈 독립성이 약하다.

---

## 4. 바로 수정하면 효과 큰 패치 순서

1. `config_ctrl`: `err_handler.i_frame_done`에 **same-cycle AND**를 넣지 말고, rise/fall done을 누적한 “both observed” 신호를 전달.
2. `err_handler`: `ST_WAIT_READ` timeout 추가. timeout 시 retry 또는 fatal 진입.
3. `cmd_arb`: timeout 기준을 global active가 아니라 **per-chip dispatch 이후** 또는 **all-target-dispatched 이후**로 변경.
4. `cell_builder`: `ST_C_IDLE` no-free-buffer 시 `ST_C_DROP`로 들어가서 입력을 실제로 흡수하게 변경.
5. `chip_ctrl`: raw overflow를 외부 status로 노출하고, control beat 우선 보존 경로 추가.
6. `chip_run`: deferred stop 대상 상태에 `ST_OVERRUN_FLUSH` 포함.
7. `chip_init`: timeout exit 시 `s_stopdis_r <= '0'` 또는 명시 safe-state로 복구.
8. `header_inserter`, `cell_builder`: runtime reset 누락 레지스터 명시 초기화.
9. `chip_ctrl`: `PH_RESP_DRAIN` 하드 cap 종료를 status/error와 함께 더 보수적으로 처리.
10. `stop_cfg_decode`: stop event가 truly single aggregate beat가 아니라면 누적 로직으로 변경.

---

## 5. 결론

현재 코드는 “완전히 깨져 있다” 수준은 아니지만, **FSM이 충분히 안정화됐다**고 말하기는 어렵다. 특히 아래 세 곳은 실제 현장에서 가장 먼저 문제를 만들 가능성이 높다.

- `config_ctrl` ↔ `err_handler` 복구 완료 조건
- `cmd_arb` reg-access timeout 모델
- `chip_ctrl` / `cell_builder` 경계의 beat 보존과 shot drop 처리

즉, 지금 단계에서 가장 중요한 것은 **정상경로 최적화**보다 **복구경로/포화경로/동시이벤트 경로를 먼저 닫는 것**이다.
