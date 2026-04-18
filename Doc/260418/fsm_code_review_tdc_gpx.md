# TDC-GPX FSM 코드 리뷰 (코드만 기준)

대상: 업로드된 VHDL 소스
원칙: 계약/사양/추가 문서 근거 제외, 코드의 논리적 정합성과 상태 전이 안정성만 검토

## 치명도 기준
- **치명적**: 실제 데드락, 상태 고착, 데이터 손실 가능성이 높은 항목
- **높음**: 특정 타이밍/경합에서 오동작 가능성이 큰 항목
- **중간**: 즉시 고장은 아니지만 유지보수/경계조건에서 취약한 항목
- **낮음**: 명시성/초기화/방어코드 측면 개선 권고

## 1. 치명적 / 즉시 수정 권고

1. **chip_run timeout 경로에서 `s_range_active_r` 미해제**
   - 파일: `tdc_gpx_chip_run.vhd`
   - 위치: 425-435, 448-457, 471-480, 500-510, 530-537
   - 내용: raw_busy/EF1/EF2/burst/flush timeout에서 `ST_ALU_PULSE`로 빠질 때 `s_range_active_r <= '0'`가 빠져 있음.
   - 영향: `o_range_active`가 ARMED 이후에도 남아 있을 수 있어 상위 range counter / timeout 로직이 오염될 가능성.

2. **cell_builder IFIFO2 대기 timeout이 `tlast` 없이 slice 종료 처리**
   - 파일: `tdc_gpx_cell_builder.vhd`
   - 위치: 643-658
   - 내용: `ST_O_WAIT_IFIFO2` timeout 시 `s_output_done_r`만 올리고 `s_tlast_r`는 내보내지 않음.
   - 영향: 상위 `face_assembler`가 chip 완료를 `tlast`로만 인지하므로 영구 대기 가능.

3. **face_assembler가 real-chip 완료를 입력 `tlast`에만 의존**
   - 파일: `tdc_gpx_face_assembler.vhd`
   - 위치: 583-611
   - 내용: 정상 데이터 경로에서 chip done 판정은 `s_in_tlast(v_chip_idx)` 하나뿐임.
   - 영향: 하위가 timeout으로 `slice_done`만 주고 `tlast`를 못 주면 `ST_FORWARD`에서 빠져나오지 못할 수 있음.

4. **chip_ctrl raw hold/skid 둘 다 찼을 때 신규 beat 드롭 가능성**
   - 파일: `tdc_gpx_chip_ctrl.vhd`
   - 위치: 731-747, 763-770
   - 내용: busy는 1사이클 늦게 올라오며, hold+skid가 모두 찬 뒤 신규 beat가 오면 `else: should not happen`으로 그냥 버려짐.
   - 영향: 제어 beat(`drain_done`, `ififo1_done`) 또는 raw beat 유실 가능.

5. **chip_run overrun post-case override가 같은 cycle bus response beat를 의도적으로 버림**
   - 파일: `tdc_gpx_chip_run.vhd`
   - 위치: 621-653
   - 내용: 주석대로 overrun 우선 처리 시 해당 cycle response beat가 유실될 수 있음.
   - 영향: 설계 의도라고 적혀 있지만, 실제로는 drain 카운트/상태와 데이터 보존성이 깨질 수 있어 매우 공격적인 정책.

## 2. 높음 / 강하게 의심되는 논리 취약점

6. **chip_run이 IrFlag 진입 시 expected IFIFO 카운터를 먼저 0으로 덮음**
   - 파일: `tdc_gpx_chip_run.vhd`
   - 위치: 297-301
   - 내용: `ST_DRAIN_LATCH` 바로 전 cycle에 `s_expected_ififo1_r/2_r`를 0으로 초기화.
   - 영향: 다음 state의 안정성 assert와 논리적으로 충돌.

7. **chip_run의 expected IFIFO assert는 현재 코드 구조와 모순**
   - 파일: `tdc_gpx_chip_run.vhd`
   - 위치: 304-315
   - 연관 파일: `tdc_gpx_stop_cfg_decode.vhd` 62-76
   - 내용: `stop_cfg_decode`는 shot 중 stop event가 들어오면 expected IFIFO 값을 계속 갱신하는데, `chip_run`은 shot_start~drain_latch 사이 값 불변을 assert함.
   - 영향: 시뮬레이션 false error 유발. 현재 assertion 가정이 실제 RTL 데이터 흐름과 맞지 않음.

8. **cell_builder에서 shot_start와 drain_done이 동시 발생하면 shot_start가 사라짐**
   - 파일: `tdc_gpx_cell_builder.vhd`
   - 위치: 446-464
   - 내용: drain_done이 같은 cycle에 있으면 `shot_start` 처리 자체를 skip.
   - 영향: 다음 shot용 버퍼 전환을 놓쳐 shot drop 또는 shot mixing 가능.

9. **cell_builder DROP 상태에서 새로운 shot_start를 보존하지 않음**
   - 파일: `tdc_gpx_cell_builder.vhd`
   - 위치: 466-478
   - 내용: drop 중에는 새 shot_start를 pending으로 잡지 않음.
   - 영향: 연속 shot 환경에서 후속 shot 누락 가능.

10. **face_assembler ST_FORWARD에 real-chip watchdog 없음**
   - 파일: `tdc_gpx_face_assembler.vhd`
   - 위치: 531-615
   - 내용: blank 경로에는 간접 timeout 진입이 있지만, 이미 forward 시작한 real-chip 데이터가 끊기면 탈출 경로가 없음.
   - 영향: 하위 slice 출력이 중간에서 멈추면 face row 전체가 고착될 수 있음.

11. **header_inserter는 non-IDLE `face_start`를 무시만 함**
   - 파일: `tdc_gpx_header_inserter.vhd`
   - 위치: 453-465
   - 내용: ST_IDLE이 아니면 face_start는 silent ignore.
   - 영향: 상위가 1-cycle pulse를 너무 이르게 주면 face 시작 자체가 소실.

12. **face_seq는 busy 중 `cmd_start`를 pending 보관하지 않음**
   - 파일: `tdc_gpx_face_seq.vhd`
   - 위치: 167-197
   - 내용: pipeline busy면 assert note만 내고 start를 버림.
   - 영향: SW/상위 CSR이 pulse 방식이면 시작 명령 손실.

13. **face_seq의 `s_packet_start`가 다수 live 입력 조합의 combinational event**
   - 파일: `tdc_gpx_face_seq.vhd`
   - 위치: 375-383
   - 내용: raw shot, deferred, abort, hdr idle 등을 조합한 즉시성 신호.
   - 영향: 타이밍은 짧지만 상태 검증이 어려워지고, 추후 수정 시 glitch성 버그 유입 위험이 큼.

14. **face_seq의 abort quiesce guard가 1 cycle뿐**
   - 파일: `tdc_gpx_face_seq.vhd`
   - 위치: 158-159, 375-383, 466-472
   - 내용: abort 직후 1 cycle만 start 차단.
   - 영향: 상위 abort/idle 정렬이 늦으면 다음 start가 너무 빨리 열릴 수 있음.

15. **chip_ctrl의 raw backpressure는 1 cycle 늦고, 실제 수용력은 2-depth 고정**
   - 파일: `tdc_gpx_chip_ctrl.vhd`
   - 위치: 409-419, 763-773
   - 내용: 주석상 safe라지만 hold/skid 2개에 모든 burst/control beat를 의존.
   - 영향: worst-case 동시성에서 margin이 매우 작음.

16. **chip_ctrl PH_RUN 완료 후 항상 PH_RESP_DRAIN으로 가는 구조는 숨은 stale response 문제를 인정한 형태**
   - 파일: `tdc_gpx_chip_ctrl.vhd`
   - 위치: 532-541
   - 내용: every run completion마다 stale response drain 필요하다고 명시.
   - 영향: 상위/하위 bus transaction 종료 정의가 충분히 self-contained하지 않다는 신호.

17. **err_handler ST_WAIT_READ에 read timeout 분기 없음**
   - 파일: `tdc_gpx_err_handler.vhd`
   - 위치: 214-240
   - 내용: `i_cmd_reg_done_pulse`가 안 오면 무한 대기.
   - 영향: reg read path 이상 시 recovery FSM 고착 가능.

18. **err_handler는 done_pulse에 의존하되 일부 chip rvalid=0이어도 recovery로 진행**
   - 파일: `tdc_gpx_err_handler.vhd`
   - 위치: 215-240
   - 내용: targeted chip 중 일부만 valid여도 cause 분류 부분적으로만 수행 후 다음 단계로 이동.
   - 영향: 원인 분류 누락 / 진단 일관성 저하.

## 3. 중간 위험 / 경계조건 취약점

19. **cell_builder reset 시 `s_buf_seq_r` 미초기화**
   - 파일: `tdc_gpx_cell_builder.vhd`
   - 선언: 165
   - reset 구간: 295-305
   - 영향: age arbitration이 리셋/abort 후 deterministic하지 않음.

20. **cell_builder reset 시 `s_buf_age_r` 미초기화**
   - 파일: `tdc_gpx_cell_builder.vhd`
   - 선언: 167
   - reset 구간: 295-305
   - 영향: 둘 다 SHARED일 때 older buffer 선택이 과거 값에 의존.

21. **cell_builder output reset 시 `s_rt_last_beat_r` 미초기화**
   - 파일: `tdc_gpx_cell_builder.vhd`
   - 선언: 207
   - reset 구간: 528-540
   - 영향: 초기값 의존도가 남음.

22. **cell_builder output reset 시 `s_rt_max_hits_r` 미초기화**
   - 파일: `tdc_gpx_cell_builder.vhd`
   - 선언: 208
   - reset 구간: 528-540
   - 영향: 기능상 치명적이진 않지만 reset 후 상태 완결성이 떨어짐.

23. **cell_builder abort 시 cell buffer 데이터는 의도적으로 유지**
   - 파일: `tdc_gpx_cell_builder.vhd`
   - 위치: 315-320
   - 영향: 논리상 재사용 전에 clear되므로 대체로 괜찮지만, 잘못된 state 전이와 결합되면 stale data 노출 위험.

24. **cell_builder deferred auto-start는 `s_output_req_r` 1-cycle pulse 구조에 매우 민감**
   - 파일: `tdc_gpx_cell_builder.vhd`
   - 위치: 489-507
   - 영향: edge case에서 SHARED 버퍼 시작이 1 cycle 늦어질 수 있고 디버깅이 어려움.

25. **face_assembler ST_SCAN에서 all-done skip 경로가 `s_row_done_r` 없이 IDLE 복귀**
   - 파일: `tdc_gpx_face_assembler.vhd`
   - 위치: 477-481
   - 영향: 비정상 geometry/active mask 조합에서 row completion pulse 누락 가능.

26. **face_assembler overrun abort는 partial row를 그냥 폐기**
   - 파일: `tdc_gpx_face_assembler.vhd`
   - 위치: 620-640
   - 영향: 의도된 정책이지만, 검증 없이 쓰면 다운스트림은 truncated line을 받게 됨.

27. **header_inserter reset 시 `s_hdr_rom_pending_r` 미초기화**
   - 파일: `tdc_gpx_header_inserter.vhd`
   - 선언: 136
   - reset 구간: 298-327
   - 영향: 선언 초기값에 의존. reset 일관성 부족.

28. **header_inserter reset 시 `s_hdr_rom_r` 미초기화**
   - 파일: `tdc_gpx_header_inserter.vhd`
   - 선언: 187
   - reset 구간: 298-327
   - 영향: 다음 face_start에서 덮어쓰기 전까지 stale metadata 보존.

29. **header_inserter의 `face_abort`가 `face_start`보다 우선**
   - 파일: `tdc_gpx_header_inserter.vhd`
   - 위치: 547-555
   - 영향: 같은 cycle 동시 입력 시 새 face 시작이 조용히 취소됨.

30. **header_inserter의 마지막 beat drain 대기 조건이 비교적 느슨함**
   - 파일: `tdc_gpx_header_inserter.vhd`
   - 위치: 433-438
   - 내용: `s_out_tvalid_r='0' or i_m_axis_tready='1'`이면 frame_done.
   - 영향: handshake와 register clear 타이밍을 읽는 사람이 오해하기 쉬움. 기능상 바로 문제라고 단정은 어렵지만 명시성 부족.

31. **face_seq의 global shot / face shot counter가 output 신호 `o_shot_start_gated`에 의존**
   - 파일: `tdc_gpx_face_seq.vhd`
   - 위치: 227-256
   - 영향: 내부 제어가 외부 출력 조합망에 의존해 유지보수가 까다로움.

32. **face_seq `p_frame_done_both`는 `s_packet_start`가 다시 뜨면 rise/fall done 기록을 모두 초기화**
   - 파일: `tdc_gpx_face_seq.vhd`
   - 위치: 317-336
   - 영향: packet_start 중복/글리치가 있으면 frame completion history 손실.

33. **face_seq shot deferral 로직이 다중 조건 재할당 구조라 추론 난이도 높음**
   - 파일: `tdc_gpx_face_seq.vhd`
   - 위치: 421-460
   - 영향: 기능은 가능해 보이나, 경계조건에서 formal proof 없이는 확신이 어려움.

34. **chip_init ST_OFF→ST_POWERUP 진입 시 `s_wait_cnt_r` 재초기화 없음**
   - 파일: `tdc_gpx_chip_init.vhd`
   - 위치: 132-147
   - 영향: 현재 다른 경로가 대부분 0으로 되돌리지만, future edit에 취약.

35. **chip_init req_valid를 모든 진입 경로에서 명시적으로 정리하지 않음**
   - 파일: `tdc_gpx_chip_init.vhd`
   - 위치: 173-223 전후
   - 영향: 현재는 state 흐름상 안전하지만, 새 exit path 추가 시 stale valid 위험.

36. **chip_reg Reg14 bit4 차단 구현이 이중 신호 할당 스타일**
   - 파일: `tdc_gpx_chip_reg.vhd`
   - 위치: 117-120
   - 영향: 합성은 되더라도 읽기 난이도 높고 향후 확장 시 실수 유발 가능.

37. **chip_reg는 busy 중 새 read/write request를 큐잉하지 않음**
   - 파일: `tdc_gpx_chip_reg.vhd`
   - 위치: 100-127
   - 영향: pulse 기반 제어면 명령 손실 가능.

38. **skid_buffer reset 후 첫 cycle `s_s_ready_r='0'`**
   - 파일: `tdc_gpx_skid_buffer.vhd`
   - 위치: 76-83
   - 영향: reset 직후 source가 ready를 기다리지 않으면 첫 beat 손실 가능.

39. **skid_buffer ST_SKID에는 explicit else hold 설명이 부족**
   - 파일: `tdc_gpx_skid_buffer.vhd`
   - 위치: 101-108
   - 영향: 기능상 문제는 아니지만 state retention 의도가 코드에 드러나지 않음.

40. **err_handler fatal 이후 `o_err_active=0`이 되도록 설계되어 직관과 다를 수 있음**
   - 파일: `tdc_gpx_err_handler.vhd`
   - 위치: 321-325
   - 영향: SW가 active/fatal 의미를 분리 이해하지 못하면 오판 가능.

41. **err_handler debounce counter는 debounced chip에서 즉시 clear되지 않음**
   - 파일: `tdc_gpx_err_handler.vhd`
   - 위치: 173-185
   - 영향: recovery 중간 재유입 시 debounce 이력 해석이 모호해질 수 있음.

42. **chip_ctrl `o_busy`는 PH_RESP_DRAIN/PH_INIT까지 모두 busy로 노출**
   - 파일: `tdc_gpx_chip_ctrl.vhd`
   - 위치: 775-777
   - 영향: SW/상위 FSM이 busy 의미를 “실제 run 중”으로 해석하면 혼동 가능.

43. **chip_ctrl soft reset override가 현재 phase 작업을 바로 덮어씀**
   - 파일: `tdc_gpx_chip_ctrl.vhd`
   - 위치: 593-598 부근(soft reset override logic)
   - 영향: 동시 명령의 우선순위는 명확하지만, 어떤 작업이 중단됐는지 진단 정보는 없음.

44. **bus_phy read burst continuation이 latched burst가 아니라 live `i_req_burst`를 봄**
   - 파일: `tdc_gpx_bus_phy.vhd`
   - 위치: 552-568
   - 영향: 상위가 mid-burst에 `i_req_burst`를 일찍 내리면 burst 종료 타이밍이 live input 정합성에 좌우됨.

45. **bus_phy TURNAROUND/READ/WRITE는 `i_tick_en` 정지 시 탈출 불가**
   - 파일: `tdc_gpx_bus_phy.vhd`
   - 위치: 479 이하 전반
   - 영향: tick generator 이상 시 FSM 영구 정지. 보통 허용 가능하지만 watchdog 부재.

## 4. 비교적 안정적으로 보이는 점

46. **bus_phy는 상태 전이를 tick enable에만 묶어 글리치 가능성을 잘 줄임**
47. **chip_run은 drain path에 timeout cause 코드를 분리해 진단성이 좋음**
48. **face_assembler는 strict chip order를 유지하려는 구조가 명확함**
49. **header_inserter는 last beat를 drain한 뒤 frame_done을 내보내려는 의도가 분명함**
50. **chip_ctrl는 raw response를 2-depth로 흡수해 단순 one-shot pulse보다 훨씬 안전함**

## 5. 우선 수정 순서

1. `tdc_gpx_chip_run.vhd`
   - 모든 timeout exit에 `s_range_active_r <= '0'` 추가
   - `ST_DRAIN_LATCH` assertion/latched expected_ififo 정책 재설계
   - `s_oen_permanent_r`의 모든 진입 경로 else-clear 일관화

2. `tdc_gpx_cell_builder.vhd` + `tdc_gpx_face_assembler.vhd`
   - `IFIFO2 timeout` 시 synthetic `tlast`를 내보내거나
   - 별도 `slice_abort/slice_timeout_ack` handshake를 face_assembler까지 전파
   - shot_start pending latch 추가
   - reset에서 `s_buf_seq_r/s_buf_age_r/s_rt_*` 명시 초기화

3. `tdc_gpx_chip_ctrl.vhd`
   - raw hold overflow를 assert/error로 승격
   - control beat 별도 우선 버퍼 또는 depth 3 이상 검토

4. `tdc_gpx_face_seq.vhd` / `tdc_gpx_header_inserter.vhd`
   - pulse 입력(start/face_start) pending latch 추가
   - combinational `packet_start`를 registered pulse로 정리 검토

5. `tdc_gpx_err_handler.vhd`
   - ST_WAIT_READ timeout 추가
   - partial rvalid 시 recovery 진행 조건 재정의

## 6. 권장 검증 시나리오

- shot_start와 drain_done 동시 발생
- IFIFO1 done 후 IFIFO2 never-done timeout
- chip_run burst response 중 overrun 동시 발생
- raw_hold/skid full 상태에서 추가 control beat 유입
- busy 중 cmd_start / face_start pulse 1-cycle 입력
- err_handler reg read done_pulse 누락
- abort 직후 다음 shot이 1-cycle만 들어오는 경우
- reset 직후 첫 beat/first face_start

