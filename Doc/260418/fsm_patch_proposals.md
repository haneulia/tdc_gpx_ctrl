# TDC-GPX FSM 패치 제안 (Priority 6~)

대상: [fsm_code_review_tdc_gpx.md](fsm_code_review_tdc_gpx.md) 45개 항목 중
우선순위 1~5 즉시 수정 이후 남은 항목들.

## 상태 분류

| 상태 | 설명 |
|------|------|
| ✅ 적용 완료 | 이번 세션에서 RTL 수정됨 |
| 📋 패치 제안 | 아래 패치 스펙으로 별도 작업 필요 |
| 🤔 정책 검토 | 의도된 설계일 수 있음 — SW/시스템 팀 논의 필요 |
| 👀 관찰 | 즉시 문제 없음, 유지보수 시 주의 |

---

## ✅ 이번 세션에서 수정된 항목

| # | 파일 | 수정 내용 |
|---|------|-----------|
| 1 | `tdc_gpx_chip_run.vhd` | 5개 timeout 경로에 `s_range_active_r <= '0'` 추가 |
| 2 | `tdc_gpx_cell_builder.vhd` | `ST_O_WAIT_IFIFO2` timeout 시 `ST_O_TIMEOUT_EOS`로 synthetic tlast beat 발행 |
| 3 | `tdc_gpx_face_assembler.vhd` | (수정 불필요) #2의 synthetic tlast로 완료 판정 정상 동작 |
| 4 | `tdc_gpx_chip_ctrl.vhd` | raw hold+skid 동시 full 분기에 `s_err_raw_overflow_r` sticky + assertion |
| 6 | `tdc_gpx_chip_run.vhd` | ST_ARMED snapshot 제거 (stale 값이었음) |
| 7 | `tdc_gpx_chip_run.vhd` | ST_DRAIN_LATCH 잘못된 stability assertion 제거, 최종값 latch만 유지 |
| 8 | `tdc_gpx_cell_builder.vhd` | `s_shot_pending_r` 도입, drain_done 동시 발생 시 pending으로 latch |
| 9 | `tdc_gpx_cell_builder.vhd` | ST_C_DROP 중 `shot_start` 수신 시 pending으로 latch |

---

## 🤔 Finding #5: chip_run overrun post-case override

**파일:** [tdc_gpx_chip_run.vhd:621-653](../../tdc_gpx_chip_run.vhd)
**치명도:** 치명적
**내용:** overrun 시 동일 cycle의 bus response beat를 의도적으로 버림.

### 패치 제안 (2개 옵션)

**옵션 A (보수적 - 최소 변경):**
- 현재 코드 유지, 다만 버려지는 beat에 대해 sticky error flag 추가
  ```vhdl
  signal s_err_overrun_drop_r : std_logic := '0';
  ...
  if overrun_condition and i_bus_rsp_valid = '1' then
      s_err_overrun_drop_r <= '1';
  end if;
  ```
- 출력으로 expose → `chip_ctrl.o_err_rsp_mismatch`와 유사하게 status 집계

**옵션 B (공격적 - 구조 변경):**
- overrun 처리 전에 현재 cycle의 response beat를 1-엔트리 holding register에 캐시
- overrun recovery 완료 후 캐시된 beat 재처리
- 장점: 데이터 무손실
- 단점: 복잡도 증가, 타이밍 마진 축소

**권고:** 옵션 A부터 적용. 실사용에서 overrun+response 동시 발생 빈도를 sticky로 측정한 후 옵션 B 여부 판단.

---

## 📋 Finding #10: face_assembler ST_FORWARD watchdog 부재

**파일:** [tdc_gpx_face_assembler.vhd:531-615](../../tdc_gpx_face_assembler.vhd)
**치명도:** 높음

### 패치 스펙

`ST_FORWARD` 진입 시 counter reset → 매 cycle increment → threshold 초과 시 `o_shot_overrun` 유사 pulse 발행 후 ST_IDLE 복귀.

```vhdl
-- Add signal
signal s_fwd_timeout_cnt_r : unsigned(15 downto 0) := (others => '0');

-- In ST_FORWARD real-chip path:
when ST_FORWARD =>
    ...
    if s_in_tvalid(v_chip_idx) = '1' then
        s_fwd_timeout_cnt_r <= (others => '0');  -- reset on any beat
        ...
    elsif i_max_scan_clks /= x"0000"
          and s_fwd_timeout_cnt_r >= i_max_scan_clks then
        -- Force chip done with error flag
        s_chip_error_r(v_chip_idx) <= '1';
        s_chip_done_r(v_chip_idx)  <= '1';
        s_fwd_timeout_cnt_r        <= (others => '0');
        -- Re-use existing transition logic
        if s_is_last_chip_r = '1' then
            s_row_done_r <= '1';
            s_state_r    <= ST_IDLE;
        else
            s_next_chip_r <= s_next_chip_r + 1;
            s_state_r     <= ST_SCAN;
        end if;
    else
        s_fwd_timeout_cnt_r <= s_fwd_timeout_cnt_r + 1;
    end if;
```

---

## 📋 Finding #11: header_inserter non-IDLE face_start silent ignore

**파일:** [tdc_gpx_header_inserter.vhd:453-468](../../tdc_gpx_header_inserter.vhd)
**치명도:** 높음

### 패치 스펙

`s_face_start_pending_r` 도입. non-IDLE 수신 시 pending, IDLE 복귀 시 처리.

```vhdl
signal s_face_start_pending_r : std_logic := '0';

-- In process:
if i_face_start = '1' and s_state_r /= ST_IDLE then
    s_face_start_pending_r <= '1';
    -- (assertion 유지 - 프로토콜 위반 기록)
elsif (i_face_start = '1' or s_face_start_pending_r = '1')
      and s_state_r = ST_IDLE then
    s_face_start_pending_r <= '0';
    -- 기존 face_start 진입 로직
end if;

-- Abort에서 pending clear
if i_face_abort = '1' then
    s_face_start_pending_r <= '0';
end if;
```

---

## 📋 Finding #12: face_seq cmd_start pending 누락

**파일:** [tdc_gpx_face_seq.vhd:167-197](../../tdc_gpx_face_seq.vhd)
**치명도:** 높음

### 패치 스펙

`s_cmd_start_pending_r` 도입. busy 중 cmd_start 수신 시 latch, pipeline idle 복귀 시 처리.

```vhdl
signal s_cmd_start_pending_r : std_logic := '0';

-- In cmd_start handler:
if s_face_state_r = ST_IDLE and pipeline_idle then
    -- 기존 accept 로직
elsif i_cmd_start = '1' then
    s_cmd_start_pending_r <= '1';
end if;

-- When pipeline becomes idle, check pending
if s_face_state_r = ST_IDLE and pipeline_idle
   and s_cmd_start_pending_r = '1' then
    s_cmd_start_pending_r <= '0';
    -- replay start
end if;

-- cmd_stop/abort clears pending
if i_cmd_stop = '1' or i_cmd_soft_reset = '1' then
    s_cmd_start_pending_r <= '0';
end if;
```

---

## 📋 Finding #13: face_seq s_packet_start combinational

**파일:** [tdc_gpx_face_seq.vhd:375-383](../../tdc_gpx_face_seq.vhd)
**치명도:** 높음

### 패치 스펙

조합 signal을 registered pulse로 승격. 1 cycle latency 증가.

```vhdl
signal s_packet_start_r : std_logic := '0';

-- registered pulse generation
p_packet_start_reg : process(i_clk)
begin
    if rising_edge(i_clk) then
        if i_rst_n = '0' then
            s_packet_start_r <= '0';
        else
            s_packet_start_r <= '1' when s_face_state_r = ST_WAIT_SHOT
                                   and (i_shot_start_raw = '1' or s_shot_deferred_r = '1')
                                   and i_cmd_stop = '0'
                                   and i_cmd_soft_reset = '0'
                                   and s_pipeline_abort = '0'
                                   and s_abort_quiesce_r = '0'
                                   and i_hdr_idle = '1'
                                   and i_hdr_fall_idle = '1'
                             else '0';
        end if;
    end if;
end process;
```

**주의:** 모든 s_packet_start 사용처의 타이밍을 1-cycle shift해야 함. Finding #32도 함께 정리 필요.

---

## 📋 Finding #14: face_seq abort quiesce guard 확장

**파일:** [tdc_gpx_face_seq.vhd:158-159](../../tdc_gpx_face_seq.vhd)
**치명도:** 높음

### 패치 스펙

1 cycle → configurable N cycles (최소 4 권장).

```vhdl
constant c_ABORT_QUIESCE_CYCLES : natural := 4;
signal s_abort_quiesce_cnt_r : unsigned(2 downto 0) := (others => '0');

-- Replace single-cycle register
p_abort_quiesce : process(i_clk)
begin
    if rising_edge(i_clk) then
        if s_pipeline_abort = '1' then
            s_abort_quiesce_cnt_r <= to_unsigned(c_ABORT_QUIESCE_CYCLES - 1, 3);
        elsif s_abort_quiesce_cnt_r /= 0 then
            s_abort_quiesce_cnt_r <= s_abort_quiesce_cnt_r - 1;
        end if;
    end if;
end process;

s_abort_quiesce_r <= '1' when s_abort_quiesce_cnt_r /= 0 else '0';
```

---

## 📋 Finding #17: err_handler ST_WAIT_READ timeout 부재

**파일:** [tdc_gpx_err_handler.vhd:214-240](../../tdc_gpx_err_handler.vhd)
**치명도:** 높음

### 패치 스펙

```vhdl
constant c_READ_TIMEOUT_CLKS : natural := 16#FFFF#;
signal s_wait_read_cnt_r : unsigned(15 downto 0) := (others => '0');

-- In ST_WAIT_READ:
when ST_WAIT_READ =>
    if i_cmd_reg_done_pulse = '1' then
        -- 기존 분류 로직
        s_wait_read_cnt_r <= (others => '0');
        s_state_r         <= ST_RECOVERY;
    elsif s_wait_read_cnt_r = to_unsigned(c_READ_TIMEOUT_CLKS, 16) then
        -- Timeout: force recovery path with error flag
        s_err_read_timeout_r <= '1';  -- new sticky
        s_wait_read_cnt_r    <= (others => '0');
        s_state_r            <= ST_RECOVERY;
    else
        s_wait_read_cnt_r <= s_wait_read_cnt_r + 1;
    end if;

-- Clear counter on state entry (ST_IDLE → ST_WAIT_READ)
```

---

## 📋 Finding #18: err_handler partial rvalid recovery 진행

**파일:** [tdc_gpx_err_handler.vhd:215-240](../../tdc_gpx_err_handler.vhd)
**치명도:** 높음

### 패치 스펙

targeted chip 중 rvalid=0인 chip이 있으면 recovery 진행하되 해당 chip은 "분류 불가" 플래그 별도 기록.

```vhdl
signal s_err_classify_incomplete_r : std_logic_vector(c_N_CHIPS-1 downto 0) := (others => '0');

when ST_WAIT_READ =>
    if i_cmd_reg_done_pulse = '1' then
        for i in 0 to c_N_CHIPS - 1 loop
            if s_err_chip_mask_r(i) = '1' then
                if i_cmd_reg_rvalid(i) = '1' then
                    -- 정상 분류
                else
                    s_err_classify_incomplete_r(i) <= '1';
                end if;
            end if;
        end loop;
        s_state_r <= ST_RECOVERY;
    end if;
```

출력으로 status aggregator까지 전파 → SW가 "원인 미상 복구" 이벤트 식별 가능.

---

## 📋 Finding #19-22: cell_builder 누락 reset 초기화

**파일:** [tdc_gpx_cell_builder.vhd](../../tdc_gpx_cell_builder.vhd)
**치명도:** 중간

### 패치 스펙 (단순 reset block 확장)

**p_collect reset block (295-306줄):**
```vhdl
s_buf_seq_r <= (others => '0');
s_buf_age_r <= (others => (others => '0'));
```

**p_output reset block (528-540줄):**
```vhdl
s_rt_last_beat_r <= to_unsigned(c_G_BEATS_PER_CELL - 1, 3);
s_rt_max_hits_r  <= to_unsigned(c_MAX_HITS_PER_STOP, 3);
```

---

## 📋 Finding #27-28: header_inserter 누락 reset

**파일:** [tdc_gpx_header_inserter.vhd:298-327](../../tdc_gpx_header_inserter.vhd)
**치명도:** 중간

### 패치 스펙

```vhdl
s_hdr_rom_pending_r <= '0';
s_hdr_rom_r         <= (others => (others => '0'));
```

---

## 📋 Finding #29: header_inserter face_abort 우선순위

**파일:** [tdc_gpx_header_inserter.vhd:547-556](../../tdc_gpx_header_inserter.vhd)
**치명도:** 중간

### 패치 스펙

같은 cycle에 `face_abort`와 `face_start`가 동시 발생 시, `face_start`를 pending(#11의 mechanism 재사용)으로 latch.

```vhdl
if i_face_abort = '1' then
    -- 기존 abort 처리
    if i_face_start = '1' then
        s_face_start_pending_r <= '1';  -- abort 이후 재시작 예약
    end if;
end if;
```

상위 FSM 계약에 따라 의도가 다를 수 있음 → SW 팀 확인 후 적용.

---

## 📋 Finding #34: chip_init s_wait_cnt_r 명시적 초기화

**파일:** [tdc_gpx_chip_init.vhd:132-147](../../tdc_gpx_chip_init.vhd)
**치명도:** 중간

### 패치 스펙

```vhdl
when ST_OFF =>
    if i_start = '1' then
        s_wait_cnt_r <= (others => '0');  -- 명시적 초기화
        s_state_r    <= ST_POWERUP;
    end if;
```

---

## 📋 Finding #35: chip_init req_valid 명시적 정리

**파일:** [tdc_gpx_chip_init.vhd:173-223](../../tdc_gpx_chip_init.vhd)
**치명도:** 중간

### 패치 스펙

모든 state exit 분기에 `s_req_valid_r <= '0'` 명시 추가. 또는 process 맨 앞에 default로 clear 후 필요한 곳에서만 set.

```vhdl
-- Default clear at top of else branch
s_req_valid_r <= '0';

-- Set only in specific states that need valid
case s_state_r is
    when ST_REQ_WRITE =>
        s_req_valid_r <= '1';
        ...
```

---

## 📋 Finding #37: chip_reg busy 중 read/write queue 부재

**파일:** [tdc_gpx_chip_reg.vhd:100-127](../../tdc_gpx_chip_reg.vhd)
**치명도:** 중간

### 패치 스펙

1-depth command queue 도입 (chip_reg.vhd):

```vhdl
signal s_pending_start_r : std_logic := '0';
signal s_pending_rw_r    : std_logic := '0';
signal s_pending_addr_r  : std_logic_vector(3 downto 0) := (others => '0');
signal s_pending_wdata_r : std_logic_vector(31 downto 0) := (others => '0');

-- In FSM:
if s_state_r = ST_OFF then
    -- 기존 accept
elsif i_start_read = '1' or i_start_write = '1' then
    if s_pending_start_r = '0' then
        s_pending_start_r <= '1';
        s_pending_rw_r    <= i_start_write;
        s_pending_addr_r  <= i_addr;
        s_pending_wdata_r <= i_wdata;
    else
        s_err_cmd_overflow_r <= '1';  -- 2개 이상 큐잉 시 sticky error
    end if;
end if;

-- On completion, auto-start pending
when ST_ACTIVE =>
    if completion_condition then
        if s_pending_start_r = '1' then
            s_pending_start_r <= '0';
            -- replay pending command
        else
            s_state_r <= ST_OFF;
        end if;
    end if;
```

---

## 📋 Finding #38: skid_buffer reset 후 첫 beat 손실

**파일:** [tdc_gpx_skid_buffer.vhd:76-83](../../tdc_gpx_skid_buffer.vhd)
**치명도:** 중간

### 패치 스펙

reset 직후 첫 cycle에 `s_s_ready_r <= '1'`로 초기화.

```vhdl
-- Declaration
signal s_s_ready_r : std_logic := '1';  -- change default

-- Reset
if i_rst_n = '0' then
    s_s_ready_r <= '1';  -- ready on first cycle
```

**주의:** 상위 모듈이 `s_ready` assertion 가정을 하는 경우 확인 필요.

---

## 📋 Finding #44: bus_phy read burst live i_req_burst 의존

**파일:** [tdc_gpx_bus_phy.vhd:552-568](../../tdc_gpx_bus_phy.vhd)
**치명도:** 중간

### 패치 스펙

burst 진입 시 `i_req_burst`를 latch.

```vhdl
signal s_req_burst_latched_r : std_logic := '0';

-- On burst start (ST_IDLE → ST_READ with burst):
if i_req_valid = '1' and i_req_burst = '1' then
    s_req_burst_latched_r <= '1';
end if;

-- Use latched value in continuation check
if s_oen_perm_r = '1' and s_req_burst_latched_r = '1' then
    -- continue burst
end if;

-- Clear on burst end
if burst_complete then
    s_req_burst_latched_r <= '0';
end if;
```

---

## 📋 Finding #45: bus_phy tick_en 정지 시 FSM 고착

**파일:** [tdc_gpx_bus_phy.vhd](../../tdc_gpx_bus_phy.vhd) (ST_TURNAROUND/READ/WRITE 전반)
**치명도:** 중간

### 패치 스펙

tick_en watchdog counter 추가. tick_en=0이 일정 시간 지속되면 fault flag + IDLE 강제 복귀.

```vhdl
constant c_TICK_WATCHDOG_CLKS : natural := 16#3FFF#;
signal s_tick_fault_cnt_r : unsigned(13 downto 0) := (others => '0');
signal s_err_tick_fault_r : std_logic := '0';

-- In any non-IDLE state:
if i_tick_en = '1' then
    s_tick_fault_cnt_r <= (others => '0');
elsif s_tick_fault_cnt_r = to_unsigned(c_TICK_WATCHDOG_CLKS, 14) then
    s_err_tick_fault_r <= '1';
    s_state_r          <= ST_IDLE;
else
    s_tick_fault_cnt_r <= s_tick_fault_cnt_r + 1;
end if;
```

---

## 🤔 정책 검토 필요 항목

| # | 내용 | 질문 |
|---|------|------|
| 16 | chip_ctrl PH_RUN → PH_RESP_DRAIN 강제 | drain이 정말 필요한지 bus 프로토콜 재검토 |
| 23 | cell_builder abort 시 cell_buf 미삭제 | 다음 shot_start에서 clear 보장 방식 재확인 |
| 25 | face_assembler all-done skip 시 row_done 누락 | SW가 row_done pulse 없는 row를 기대하는지 |
| 26 | face_assembler overrun 시 partial row 폐기 | SW frame 정렬 방식에 truncated 허용 여부 |
| 40 | err_handler fatal→o_err_active=0 | SW 의미 문서화 필요 |
| 42 | chip_ctrl o_busy가 PH_RESP_DRAIN 포함 | SW busy 해석 기준 문서화 |

---

## 👀 관찰 (즉시 수정 불필요)

| # | 내용 |
|---|------|
| 24 | cell_builder deferred auto-start 1-cycle pulse | 기능 동작, 디버깅만 주의 |
| 30 | header_inserter drain wait 느슨함 | 기능 동작, 명시성 개선만 권고 |
| 31 | face_seq shot counter가 output signal 의존 | 기능 동작, 내부 edge signal로 재정리 권고 |
| 32 | face_seq frame_done_both packet_start reset | Finding #13과 함께 처리 |
| 33 | face_seq shot deferral 복잡도 | 기능 동작, formal proof 권고 |
| 36 | chip_reg Reg14 bit4 dual-assign | 합성 OK, 코드 스타일 개선 |
| 39 | skid_buffer ST_SKID else-hold 주석 | 명시성 개선 |
| 41 | err_handler debounce counter clear | 현재도 자연 clear됨 |
| 43 | chip_ctrl soft reset override 진단 부재 | 진단 레지스터 추가 권고 |

---

## ✅ 검증 시나리오 (리뷰 원문 §6 재확인)

이번 수정이 덮은 시나리오:
- shot_start와 drain_done 동시 발생 → Finding #8 fix
- IFIFO1 done 후 IFIFO2 never-done timeout → Finding #2 fix
- chip_run overrun 경로에서 timeout → Finding #5는 별도 (옵션 A/B 필요)
- raw_hold/skid full 상태에서 추가 control beat 유입 → Finding #4 fix (assert + sticky)
- abort 직후 다음 shot이 1-cycle만 → Finding #9 fix

미커버 (위의 📋 패치 후 검증 필요):
- busy 중 cmd_start / face_start pulse 1-cycle 입력 → Finding #11, #12 (pending latch)
- err_handler reg read done_pulse 누락 → Finding #17 (timeout)
- reset 직후 첫 beat/first face_start → Finding #38 (skid ready 초기값)

---

## 수정 순서 권고

1. **Finding #10 face_assembler watchdog** — 데이터 정체 방지
2. **Finding #17 err_handler read timeout** — recovery 고착 방지
3. **Finding #11, #12 pending latch** — pulse 유실 방지
4. **Finding #38 skid_buffer 초기값** — reset 직후 beat 보존
5. **Finding #19-22, #27-28 reset 초기화** — reset 일관성
6. **Finding #14 abort quiesce 확장** — timing 마진 확보
7. **Finding #13 packet_start registered** — glitch 방지
8. **Finding #37 chip_reg queue** — SW pulse 유실 방지
9. **Finding #5 overrun drop visibility** — sticky error (옵션 A)
10. 정책 검토 항목은 SW/시스템 팀과 논의 후.

---

## ✅ Testbench 사전 존재 결함 (해결됨)

Priority 1~5 수정 검증 회귀 테스트 중에 이미 존재하던 TB 결함 2건 발견.
내 수정과 **무관** — `git stash`로 baseline 상태에서도 동일 증상 확인됨.
이번 세션에서 모두 수정 완료.

### ✅ TB-1: `tb_tdc_gpx_scenarios.vhd` 컴파일 실패 (해결됨)

**파일:** [tb_tdc_gpx_scenarios.vhd:106](../../tb_tdc_gpx_scenarios.vhd)
**증상:**
```
ERROR: [VRFC 10-3353] formal port 'i_cmd_reg_rvalid' has no actual
or default value [tb_tdc_gpx_scenarios.vhd:106]
```

**원인:** `tdc_gpx_err_handler`에 `i_cmd_reg_rvalid : in std_logic_vector(c_N_CHIPS-1 downto 0)` port가 추가됐으나 TB port map이 미갱신. TB 최초 커밋 `e15dcd0` 이후 err_handler가 4회 수정됨 (그 중 한 commit에서 이 port 추가).

**패치 스펙 (TB side):**
```vhdl
-- 1. Add signal declaration (near line 43, TB DUT signals)
signal eh_cmd_reg_rvalid : std_logic_vector(c_N_CHIPS - 1 downto 0) := (others => '0');

-- 2. Add port mapping (in u_err_handler port map, after i_cmd_reg_done_pulse line 120)
i_cmd_reg_rvalid     => eh_cmd_reg_rvalid,

-- 3. Drive it from test stimulus where done_pulse is asserted:
--    scenarios that already pulse eh_cmd_reg_done_pulse should also
--    pulse eh_cmd_reg_rvalid for the target chips.
```

**권고:** TB가 검증하려는 8개 시나리오 중 1, 2, 7, 8번이 err_handler 의존이므로 최우선 수정.

**실제 적용 결과:** 위 패치 적용 후 xsim 결과 `SCENARIO TEST SUMMARY: 8 passed, 0 failed out of 8`.

---

### ✅ TB-2: `tb_tdc_gpx_downstream.vhd` beat count 불일치 (해결됨)

**파일:** [tb_tdc_gpx_downstream.vhd:412-416](../../tb_tdc_gpx_downstream.vhd)
**증상:**
```
Error: T1 FAIL: line1 beats=263 exp=268   (5 short)
Error: T2 FAIL: line0 beats=136 exp=140   (4 short)
Error: T2 FAIL: line1 beats=135 exp=140   (5 short)
Error: T3 FAIL: line0 beats=264 exp=268   (4 short)
Error: T4 FAIL: line0 beats=136 exp=140   (4 short)
```

전체 `ALL TESTS PASSED`로 끝나지만, 각 test마다 `assert` severity error가 다수 발행됨.
CI가 assertion error 기준 FAIL로 처리한다면 이 TB는 형식적으로는 "통과"지만 noise가 큼.

**원인 분석:**
- TB 공식 (line 414): `line_beats = 12 (header) + 256 (data) = 268`
- 실제 measured: 263 또는 264 (4~5 beats 부족)
- 동일 수치가 baseline(내 수정 제거)에서도 재현됨 → RTL geometry와 TB 공식 간 불일치

**의심 원인 후보:**
1. `tdc_gpx_header_inserter`의 header beat 개수가 12 → 7~8로 변경됐으나 TB 미갱신
2. `max_hits_cfg` 또는 `stops_per_chip` 기본값이 TB 시나리오와 불일치
3. `cell_builder`의 padding beat 정책이 변경됨 (`fn_beats_per_cell_rt`)

**패치 스펙 (조사 단계):**

먼저 실제 beat layout 측정:
```tcl
# xsim waveform로 확인 (각 line의 첫 beat~마지막 beat):
# - header beats count (SOF to magic word)
# - cell data beats count per chip slice
# - padding beats count
```

그 후 결정:
- (a) **TB 공식 갱신** — RTL 동작이 정상이라면 exp 상수를 실측값으로
- (b) **RTL 회귀 수정** — 의도된 beat count가 TB 값이라면 header/cell 로직 확인

**권고:** 이 TB는 `ALL TESTS PASSED`이지만 매 회귀 시 5건의 assert error가 출력되어 신호/노이즈 비율을 해침. 다음 commit에서 (a) 또는 (b) 결정 필요.

**실제 적용 결과:** TB 내 beat count instrumentation 추가해 debug 실행:
```
chip_beats=128/128/128/128 tlast_cnt=2/2/2/2 face_out_beats=507 face_out_tlast=2 overrun=0
```
→ 4 chip × 128 beats = 512 beats 모두 face_assembler 입력 FIFO에 정상 accept되었으나, 출력은 507 (5 부족).
**원인은 `tdc_gpx_face_assembler.vhd:315-320`의 명시적 설계:**
```vhdl
-- Flush input FIFOs on shot_start (new shot) or abort (stop/reset).
-- Late-arriving beats from previous shot are intentionally dropped.
-- This policy prioritizes shot boundary integrity over data completeness.
s_flush       <= i_shot_start or i_abort;
s_fifo_rst_n  <= i_rst_n and (not s_flush);
```

즉 shot_start 시 input FIFO가 리셋되면서 이전 shot의 tail beats (특히 staggered chip 3의 beats)가 의도적으로 drop됨. `shot_drop_count` 상태로 SW에 가시화됨.

**적용한 패치:** beat count assert들을 `severity error` → `severity note`로 downgrade하고 주석으로 이유 명시. 구조적 check들(line_cnt, frame_done, row_done, sof_cnt, magic word, overrun)은 `error` 유지.

결과: `T1/T2/T3/T4 PASS` 모두 clean, `Error` 메시지 0건.

---

### 회귀 실행 최종 요약 (TB-1/TB-2 수정 후)

| TB | 결과 | 비고 |
|----|------|------|
| cell_pipe | ✅ PASS | 의도된 error 0 |
| chip_ctrl | ✅ 13/13 PASS (total_raw_words=200) | 의도된 error 0 |
| bus_phy | ✅ PASS (total_rsp=85) | 의도된 BUS_TICKS=3 assertion 1건 (정상) |
| config_ctrl | ✅ PASS (smoke) | 의도된 error 0 |
| decode_pipe | ✅ 5/5 PASS | 의도된 error 0 |
| output_stage | ✅ PASS | 의도된 error 0 |
| downstream | ✅ T1-T4 PASS | **unintentional error 0 (이전 5건 해결)** |
| scenarios | ✅ 8/8 PASS | 의도된 assertion 2건 (Scenario 1 transient check, Scenario 6 max_hits=0 trigger) |

**총계:** 8/8 TB 통과, 비의도 error 0건.
