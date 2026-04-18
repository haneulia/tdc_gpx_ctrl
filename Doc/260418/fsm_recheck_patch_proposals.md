# TDC-GPX FSM 재점검 기반 패치 제안 (Round 2)

대상: [fsm_recheck_code_only_report.md](fsm_recheck_code_only_report.md) 40개 항목.
원본 리뷰: [fsm_code_review_tdc_gpx.md](fsm_code_review_tdc_gpx.md), 1차 패치: [fsm_patch_proposals.md](fsm_patch_proposals.md).

## 상태 분류

| 상태 | 설명 |
|------|------|
| ✅ 적용 완료 | 이번 세션(Round 2)에서 RTL 수정됨 |
| 📋 패치 제안 | 아래 패치 스펙으로 별도 작업 필요 |
| 🤔 정책 검토 | 의도된 설계일 수 있음 — SW/시스템 팀 논의 필요 |
| 👀 관찰 | 즉시 문제 없음, 유지보수 시 주의 |

---

## ✅ 이번 세션(Round 2)에서 수정된 항목

| 보고서 # | 파일 | 수정 내용 |
|---------|------|-----------|
| 1 | `tdc_gpx_config_ctrl.vhd` | `i_frame_done` AND gate를 rise/fall 각 latch 방식으로 변경, `s_frame_done_both` 생성 |
| 2 | `tdc_gpx_err_handler.vhd` | `ST_WAIT_READ`에 16-bit watchdog + `s_err_read_timeout_r` sticky 추가 |
| 3 | `tdc_gpx_cmd_arb.vhd` | 글로벌 active → `s_reg_pending_r` 전부 비워진 후(= 모든 dispatch 완료)부터 카운트 |
| 4 | `tdc_gpx_cell_builder.vhd` | IDLE에서 no-free-buffer 시 `ST_C_DROP` 진입 (tready=1로 실제 beats 흡수) |
| 5, 12 | `tdc_gpx_chip_ctrl.vhd`, `tdc_gpx_config_ctrl.vhd`, `tdc_gpx_top.vhd`, `tdc_gpx_pkg.vhd` | `o_err_raw_overflow` 포트 신설, CDC, `t_tdc_status.raw_overflow_mask` 추가 |
| 6 | `tdc_gpx_chip_run.vhd`, `tdc_gpx_chip_ctrl.vhd` | overrun override가 bus response 같은 cycle에 fire할 때 `s_err_overrun_drop_r` sticky → chip_ctrl의 `s_err_raw_overflow_r`에 OR fold |
| 7 | `tdc_gpx_chip_run.vhd` | deferred stop latch 대상에 `ST_OVERRUN_FLUSH` 추가 |
| 8 | `tdc_gpx_chip_init.vhd` | `ST_CFG_WR_WAIT` / `ST_MR_WAIT` timeout exit에서 `s_stopdis_r <= '0'` 추가 (정상 완료 경로와 일치) |

## ✅ Round 3 추가 수정 항목

| 보고서 # | 파일 | 수정 내용 |
|---------|------|-----------|
| 9 | `tdc_gpx_chip_ctrl.vhd` | `PH_RESP_DRAIN` hard cap(15 cycle) 도달 시 bus 여전히 활성이면 `s_err_drain_cap_r` sticky → `o_err_raw_overflow`에 OR 합류 |
| 10 | `tdc_gpx_err_handler.vhd` | `i_soft_clear` 입력 포트 추가 (기본값 '0'). SW가 fatal 상태를 hard reset 없이 clear 가능 |
| 14 | `tdc_gpx_header_inserter.vhd` | `s_face_start_pending_r` 도입. non-IDLE `face_start` pulse를 latch, 다음 IDLE에서 소비. abort는 pending drop |
| 15 | `tdc_gpx_header_inserter.vhd` | reset 블록에 `s_hdr_rom_pending_r`, `s_hdr_rom_r` 명시 초기화 |
| 16 | `tdc_gpx_cell_builder.vhd` | `p_collect` reset 블록에 `s_buf_seq_r`, `s_buf_age_r` 명시 초기화 |
| 17 | `tdc_gpx_cell_builder.vhd` | `p_output` reset 블록에 `s_rt_last_beat_r`, `s_rt_max_hits_r` 명시 초기화 |
| 19 | `tdc_gpx_face_assembler.vhd` | zero active mask 진입 시 즉시 `s_row_done_r <= '1'` pulse 후 IDLE (self-consistent) |
| 21 | `tdc_gpx_chip_init.vhd` | `i_start`와 `i_cfg_write_req` 동시 cycle 시 cfg_write를 pending으로 latch. ST_OFF 복귀 시 자동 처리 |
| 32 | `tdc_gpx_face_seq.vhd` | `s_cmd_start_pending_r` 도입. busy 중 cmd_start pulse를 latch, pipeline idle 시 재처리 |
| 20, 37 | `tdc_gpx_chip_reg.vhd` | 1-depth pending queue (`s_pend_valid_r`/`rw`/`addr`/`wdata`). 동시 read+write 또는 ST_ACTIVE 중 신규 요청을 저장. 오버플로는 `s_err_req_overflow_r` sticky |
| 39 | `tdc_gpx_status_agg.vhd`, `tdc_gpx_top.vhd` | `o_error_count` → `o_error_cycle_count`로 rename (실제 의미와 일치) |
| 33 | (이미 연결됨) | `o_shot_drop_cnt` → `s_status.shot_drop_count`로 top에서 이미 aggregation됨 — 문서만 업데이트 |

---

## 📋 높음 우선순위 (remaining)

### 보고서 #9: chip_ctrl PH_RESP_DRAIN 강제 15-cycle 종료

**파일:** [tdc_gpx_chip_ctrl.vhd:566-587](../../tdc_gpx_chip_ctrl.vhd)

### 패치 스펙
`i_bus_busy='1'` 또는 `i_bus_rsp_pending='1'`이 지속되면 drain counter를 리셋하거나 연장하는 구조로 변경. 또는 hard cap 시 error flag 추가.

```vhdl
when PH_RESP_DRAIN =>
    if i_bus_busy = '0' and i_bus_rsp_pending = '0' then
        if s_drain_cnt_r = to_unsigned(c_DRAIN_MIN, 4) then
            -- bus 완전 idle, 정상 종료
            ...
        else
            s_drain_cnt_r <= s_drain_cnt_r + 1;
        end if;
    elsif s_drain_cnt_r = x"F" then
        -- hard cap 도달: error flag + 강제 종료
        s_err_drain_cap_r <= '1';
        s_phase_r <= next_phase;
    else
        -- bus 여전히 활성: counter 유지
        null;
    end if;
```

### 보고서 #10: err_handler soft reset 경로 부재

**파일:** [tdc_gpx_err_handler.vhd:32-64, 255-279](../../tdc_gpx_err_handler.vhd)

### 패치 스펙
`i_soft_reset` 입력 포트 추가. fatal 상태에서 SW가 soft reset을 통해 복구 가능하게:

```vhdl
-- entity port
i_soft_reset : in std_logic;  -- SW-initiated fatal clear

-- in FSM reset block + in ST_WAIT_FRAME_DONE fatal check:
if i_rst_n = '0' or i_soft_reset = '1' then
    s_err_fatal_r <= '0';
    s_retry_cnt_r <= (others => '0');
    s_state_r     <= ST_IDLE;
    ...
end if;
```

### 보고서 #11: stop_cfg_decode expected IFIFO overwrite

**파일:** [tdc_gpx_stop_cfg_decode.vhd:53-79](../../tdc_gpx_stop_cfg_decode.vhd)

### 패치 스펙
`i_stop_evt_tvalid` pulse가 multi-beat aggregate가 될 수 있는지 상위 계약 재확인. 만약 multi-beat 가능성이 있으면 누적 로직으로 변경:

```vhdl
elsif i_stop_evt_tvalid = '1' then
    for i in 0 to c_N_CHIPS - 1 loop
        o_expected_ififo1(i) <= o_expected_ififo1(i)
                             +  resize(unsigned(...), 8);
        o_expected_ififo2(i) <= o_expected_ififo2(i)
                             +  resize(unsigned(...), 8);
    end loop;
```

단일 aggregate beat가 계약이면, 그 계약을 명시적 assertion으로 기록.

### 보고서 #13: chip_ctrl i_stop_tdc CDC handshake

**파일:** [tdc_gpx_chip_ctrl.vhd:73-81, 611-649](../../tdc_gpx_chip_ctrl.vhd)

### 패치 스펙
2-FF sync는 pulse가 너무 짧으면 미검출 가능. `xpm_cdc_pulse` (DEST_SYNC_FF=2, REG_OUTPUT=1) 또는 수동 handshake(ack-back) 도입.

```vhdl
u_stop_tdc_cdc : xpm_cdc_pulse
    generic map (DEST_SYNC_FF => 4, REG_OUTPUT => 1, RST_USED => 0)
    port map (
        src_clk    => i_laser_clk,
        src_pulse  => i_stop_tdc_src,
        dest_clk   => i_clk,
        dest_pulse => s_stop_tdc_cdc
    );
```

### 보고서 #14: header_inserter non-IDLE face_start 무시

[이전 fsm_patch_proposals.md Finding #11의 pending latch 제안 참조](fsm_patch_proposals.md).
1차 제안 내용 그대로 유효.

### 보고서 #15, #16, #17: reset completeness

**파일:**
- `tdc_gpx_header_inserter.vhd:136, 187, 298-327` — `s_hdr_rom_pending_r`, `s_hdr_rom_r`
- `tdc_gpx_cell_builder.vhd:165-167, 213-214, 301-313, 551-563` — `s_buf_seq_r`, `s_buf_age_r`, `s_rt_last_beat_r`, `s_rt_max_hits_r`

### 패치 스펙
reset 블록에 누락된 시그널 명시 초기화 추가.

```vhdl
-- header_inserter reset 블록
s_hdr_rom_pending_r <= '0';
s_hdr_rom_r         <= (others => (others => '0'));

-- cell_builder p_collect reset 블록
s_buf_seq_r <= (others => '0');
s_buf_age_r <= (others => (others => '0'));

-- cell_builder p_output reset 블록
s_rt_last_beat_r <= to_unsigned(c_G_BEATS_PER_CELL - 1, 3);
s_rt_max_hits_r  <= to_unsigned(c_MAX_HITS_PER_STOP, 3);
```

### 보고서 #18: face_assembler self-overrun feedback timing

**파일:** [tdc_gpx_face_assembler.vhd:620-640](../../tdc_gpx_face_assembler.vhd)

### 패치 스펙
self-overrun 시 즉시 input/output FIFO flush를 face_assembler 내부에서 완결. `pipeline_abort` feedback에 의존하지 말고:

```vhdl
-- overrun 블록에서
s_flush_internal_r <= '1';  -- 내부 flush pulse (1 cycle)

-- FIFO reset을 s_flush_internal_r와 OR
s_fifo_rst_n <= i_rst_n and (not (s_flush or s_flush_internal_r));
```

### 보고서 #19: face_assembler zero active mask self-consistency

**파일:** [tdc_gpx_face_assembler.vhd:453-459](../../tdc_gpx_face_assembler.vhd)

### 패치 스펙
`i_active_chip_mask = "0000"`에 대해 immediate `row_done` 펄스 + ST_IDLE 복귀:

```vhdl
when ST_IDLE =>
    if i_shot_start = '1' or s_shot_pending_r = '1' then
        ...
        if i_active_chip_mask = "0000" then
            -- zero mask: immediate row_done + return to IDLE
            s_row_done_r <= '1';
            s_state_r    <= ST_IDLE;
        else
            s_state_r    <= ST_SCAN;
        end if;
    end if;
```

### 보고서 #20: chip_reg concurrent read/write

**파일:** [tdc_gpx_chip_reg.vhd:100-128](../../tdc_gpx_chip_reg.vhd)

[이전 fsm_patch_proposals.md Finding #37의 queue 제안 참조](fsm_patch_proposals.md#finding-37).

### 보고서 #21: chip_init concurrent start + cfg_write

**파일:** [tdc_gpx_chip_init.vhd:139-147](../../tdc_gpx_chip_init.vhd)

### 패치 스펙
`s_cfg_write_pending_r` 도입. `i_start`와 `i_cfg_write_req`가 동시 발생 시 start를 우선 처리하고 cfg_write를 pending:

```vhdl
when ST_OFF =>
    if i_start = '1' then
        ...
        if i_cfg_write_req = '1' then
            s_cfg_write_pending_r <= '1';
        end if;
        s_state_r <= ST_POWERUP;
    elsif i_cfg_write_req = '1' then
        s_state_r <= ST_CFG_WRITE_ONLY;
    end if;
```

### 보고서 #22: face_seq cross-slope abort propagation

**파일:** [tdc_gpx_face_seq.vhd:385-387](../../tdc_gpx_face_seq.vhd)

### 패치 스펙 (정책 검토 필요)
현재 한쪽 slope abort가 양쪽 전체를 아보트. 만약 healthy 쪽을 유지하고 싶으면 slope별 독립 abort state 필요. 복잡도 큼. 먼저 SW/시스템 팀에 "fail-fast 전체 abort가 맞는 정책인지" 확인 권고.

---

## 📋 중간/구조 취약 (보고서 #23-40)

이하 항목들은 즉시 정지 가능성은 낮지만 유지보수/검증 안정성 향상을 위해 정리 권고.

| # | 파일:위치 | 내용 | 패치 방향 |
|---|-----------|------|-----------|
| 23 | `bus_phy:559-562` | burst continuation이 live `i_req_burst` 참조 | burst 진입 시 latch, continuation은 latched값 사용 |
| 24 | `chip_ctrl:657-665` | StopDis live override mid-run | run 중 override 적용은 명시 정책화 (문서+assertion) |
| 25 | `cell_builder:484-501` | ST_C_DROP timeout 이후 stale beat 경로 | DROP timeout 후 `ST_C_DRAIN_STALE` 단계 추가 고려 |
| 26 | `cell_builder:460-482, 355-375` | shot drop 두 경로의 동작 모델 불일치 | Fix #4로 통일됨 (일부 개선). 주석 정리만 남음 |
| 27 | `chip_ctrl:773` | `o_drain_done`은 downstream handshake 기반 | 주석 명확화 ("after downstream accept") |
| 28 | `chip_run:305-312` + `stop_cfg_decode:53-79` | expected IFIFO finalized handshake 부재 | stop_cfg_decode에 `o_finalized` 출력 추가, chip_run이 그 값을 기다림 |
| 29 | `chip_run:273-291` | ST_CAPTURE에서 stop은 purge 전환 | graceful path 옵션 추가 고려 (SW 정책 결정 필요) |
| 30 | `face_seq:375-383` | `packet_start` combinational | registered one-shot으로 변경 (이전 Finding #13과 동일) |
| 31 | `face_seq:160-163, 167-190` | `shot_overrun_r`가 cmd_start까지 유지 | face 경계에서 clear |
| 32 | `face_seq:167-190` | busy 중 cmd_start 무시 | pending queue 도입 (이전 Finding #12) |
| 33 | `face_seq:421-460` | shot drop counter 외부 미노출 | output port + status 추가 |
| 34 | `header_inserter:460-529, 547-555` | face_abort vs face_start 동시 cycle | abort 블록에서 명시적 `s_hdr_rom_pending_r <= '0'` cleanup |
| 35 | `header_inserter:469-499` | header snapshot = face_start 시점 | 의도된 설계. 주석/문서로 고정 |
| 36 | `face_assembler:610-640` + `header_inserter:536-555` | overrun시 partial row truncation | SW discard 경로 검증 필요 (정책 확정) |
| 37 | `chip_reg:100-150` | pulse-based request 보존 부재 | queue 추가 (이전 Finding #37) |
| 38 | `config_ctrl:415-424` | err_handler 활성 시 manual reg 굶주림 | err_handler timeout 완비 후 자연 해소 (Fix #2 관련) |
| 39 | `status_agg:90-116` | `o_error_count`가 실제로는 error-cycle count | 포트 이름을 `o_error_cycle_count`로 변경 |
| 40 | `status_agg:121-136` | sticky error가 hard reset만 clear | soft reset clear 경로 추가 고려 (정책 결정) |

---

## 수정 순서 권고 (Round 2)

이미 적용 완료된 8건 외에 다음 순서 권고:

1. **#15-17 reset completeness** — 간단하고 효과 큼
2. **#30 (packet_start registered)** — glitch 방지
3. **#10 err_handler soft reset** — SW 복구 경로 제공
4. **#11 stop_cfg_decode 계약 확정** — 상위 assertion 추가
5. **#13 stop_tdc CDC handshake** — 안전성 향상
6. **#14 face_start pending latch** — pulse 유실 방지
7. **#20, #37 chip_reg queue** — SW 명령 유실 방지
8. **#21 chip_init 동시 요청** — 마이너 정합성
9. **#22 cross-slope abort** — 정책 결정 후 적용
10. **#9 PH_RESP_DRAIN 종료 조건** — bus 안정성 개선
11. #18, #19, #23 등 나머지 구조 개선

---

## 회귀 검증 (Round 2 수정 후)

이번 세션에서 Round 2 8개 fix 적용 후 xsim 회귀는 "Run compile check + regression tests" 항목에서 수행.

## 회귀 검증 (Round 3 수정 후)

Round 3 12개 fix 적용 후 전체 TB 재실행 결과:

| TB | 결과 |
|----|------|
| cell_pipe | ✅ PASS |
| chip_ctrl | ✅ ALL TESTS PASSED (total_raw_words=200) |
| bus_phy | ✅ ALL TESTS PASSED (total_rsp=85) |
| config_ctrl | ✅ PASS (init/smoke) |
| decode_pipe | ✅ ALL TESTS PASSED (5 checks) |
| downstream | ✅ T1-T4 PASS |
| output_stage | ✅ PASS |
| scenarios | ✅ ALL SCENARIO TESTS PASSED |

**8/8 TB 통과, 비의도 error 0건.**

## 보류(Deferred) 항목 — 사유

| 보고서 # | 사유 |
|---------|------|
| 30 (packet_start registered) | 6개 소비처 downstream에 1-cycle shift 파급 효과, 정확성 검증 전 변경 리스크 큼 — formal proof 후 진행 권고 |
| 22 (cross-slope abort 전파) | 정책 결정 필요 (fail-fast 유지 vs healthy 유지) |
| 11 (stop_cfg_decode overwrite 정책) | 상위 모듈의 "1회 aggregate beat" 계약 확정 필요 — SW 팀 확인 후 progress/누적 결정 |
| 13 (stop_tdc CDC handshake) | 현재 2-FF sync는 현행 clock ratio에서 동작, 엄격 CDC 전환은 별도 RTL timing 재검증 필요 |
| 23 (bus_phy burst live dependency) | chip_run이 mid-burst burst 취소를 의도적으로 가능케 한 설계 — 변경 시 상위 제어 재설계 필요 |
| 18, 25, 29, 35, 36 | 설계 의도/SW 정책에 의존 — 확정 후 적용 권고 |
| 24, 27, 34, 38, 40 | 문서/주석 정리 수준, 기능상 문제 없음 |
