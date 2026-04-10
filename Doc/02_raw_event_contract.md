# 02. Raw Event Contract

> 참조: 00_scope_and_contract.md, deep_analysis §2.2(t5~t6), §6.2~6.3

---

## 0. 책임

TDC-GPX raw 28-bit word를 FPGA 내부 표준 포맷(raw_event record)으로 정규화:
- decode_i: 28-bit word → structured fields 분리 (순수 조합 논리)
- raw_event_builder: hit_seq_local 부여, raw_event record 완성 (registered)

**산출물**: raw_event record (stop_id, hit_seq 정확하고 SW가 시간/채널 해석 가능)

---

## 1. 비책임

- bus_phy / chip_ctrl FSM → 01번 문서
- cell 변환 (sparse → dense) → 03번 문서
- CONTINUOUS 모드의 start_num 기반 그룹핑 → 08번 문서

---

## 2. 입력

### decode_i 입력 (chip_ctrl에서)

| 신호 | 폭 | 설명 |
|------|-----|------|
| `raw_word` | 28 | IFIFO에서 읽은 raw word |
| `raw_word_valid` | 1 | raw word 유효 |
| `ififo_id` | 1 | 0=IFIFO1(Reg8), 1=IFIFO2(Reg9) |

### raw_event_builder 입력

| 신호 | 폭 | 설명 |
|------|-----|------|
| decoded fields | record | decode_i 출력 |
| `chip_id` | 2 | lane 번호 (외부에서 할당) |
| `shot_seq` | 32 | shot 순번 (shot_seq_gen에서) |
| `drain_done` | 1 | shot drain 완료 → hit_seq counter reset |

---

## 3. 출력

### decode_i 출력 (순수 조합)

| 신호 | 폭 | 설명 |
|------|-----|------|
| `raw_hit` | 17 | word[16:0], Stop-Start 시간차 (BIN 단위) |
| `slope` | 1 | word[17], 에지 방향 |
| `start_num` | 8 | word[25:18], start window 번호 |
| `cha_code_raw` | 2 | word[27:26], 채널 코드 |
| `stop_id_local` | 3 | = ififo_id × 4 + cha_code_raw |
| `decoded_valid` | 1 | = raw_word_valid (passthrough) |

### raw_event_builder 출력

| 신호 | 폭 | 설명 |
|------|-----|------|
| `raw_event` | record | t_raw_event (아래 §4 스키마) |
| `raw_event_valid` | 1 | raw_event 유효 |

---

## 4. raw_event 스키마

탑다운 구조: 식별(WHERE) → 시간(WHEN) → 측정값(WHAT) → 원본(RAW) → 메타(META)

```
t_raw_event = record
    -- ① 식별 (WHERE)
    chip_id          : unsigned(1 downto 0)   -- 0~3, lane 번호
    stop_id_local    : unsigned(2 downto 0)   -- 0~7, 칩 내 stop 채널

    -- ② 시간 컨텍스트 (WHEN)
    shot_seq         : unsigned(31 downto 0)  -- 레이저 발사 순번
    start_num        : unsigned(7 downto 0)   -- start window 번호 (SINGLE=항상 0)

    -- ③ 측정 데이터 (WHAT)
    raw_hit          : unsigned(16 downto 0)  -- 17-bit, Stop-Start (BIN 단위)
    slope            : std_logic              -- 1=rising, 0=falling

    -- ④ 원본 보존 (RAW)
    ififo_id         : std_logic              -- 0=IFIFO1, 1=IFIFO2
    cha_code_raw     : unsigned(1 downto 0)   -- 칩 출력 그대로

    -- ⑤ FPGA 생성 메타 (META)
    hit_seq_local    : unsigned(5 downto 0)   -- 같은 key 내 hit 순번
end record;
```

---

## 5. 동작 명세

### 5.1 decode_i: 28-bit word 분리 (순수 조합)

```
입력: raw_word[27:0], ififo_id

Reg8/Reg9 I-Mode 포맷:
  [27:26] = ChaCode (2-bit)
  [25:18] = Start#  (8-bit)
  [17]    = Slope    (1-bit)
  [16:0]  = Hit      (17-bit)

stop_id_local 복원:
  STOPS_PER_IFIFO = 4  (ChaCode 2-bit → 4채널/IFIFO)
  stop_id_local = ififo_id × STOPS_PER_IFIFO + cha_code_raw
```

**IFIFO별 채널 매핑:**

| ififo_id | Reg | ChaCode=00 | =01 | =10 | =11 |
|----------|-----|------------|-----|-----|-----|
| 0 | Reg8 | stop 0 | stop 1 | stop 2 | stop 3 |
| 1 | Reg9 | stop 4 | stop 5 | stop 6 | stop 7 |

### 5.2 raw_event_builder: hit_seq_local 생성

**key** = {chip_id, shot_seq, start_num, stop_id_local}

같은 key로 들어오는 raw_event에 대해:
- 첫 번째 → hit_seq_local = 0 (가장 가까운 에코)
- 두 번째 → 1
- ... (FIFO 구조가 시간순 보장)

```vhdl
-- hit_seq counter (stop_id_local별 독립)
type t_hit_cnt_array is array (0 to STOPS_PER_CHIP_MAX-1)
    of unsigned(5 downto 0);
signal hit_cnt : t_hit_cnt_array;

-- drain_done 시 전체 counter reset
if drain_done = '1' then
    hit_cnt <= (others => (others => '0'));
elsif decoded_valid = '1' then
    raw_event.hit_seq_local <= hit_cnt(to_integer(stop_id_local));
    hit_cnt(to_integer(stop_id_local)) <= hit_cnt(to_integer(stop_id_local)) + 1;
end if;
```

### 5.3 HIT_SLOT_WIDTH에 따른 truncation

| 플랫폼 | HIT_SLOT_WIDTH | 처리 |
|--------|----------------|------|
| Zynq-7000 | 16 | raw_hit[16:0] → 하위 16-bit만 cell에 저장 (MSB 절삭) |
| Zynq MPSoC | 17 | 원본 17-bit 보존 |

truncation은 raw_event_builder가 아닌 **cell_builder**에서 수행.
raw_event에는 항상 17-bit 원본을 보존.

### 5.4 SINGLE_SHOT 시간 복원식

```
실제 시간(ps) = (raw_hit - StartOff1) × BIN(ps)
거리(m) = 실제 시간(ps) × 1e-12 × c / 2
```

여기서:
- StartOff1 = Reg5[17:0] (CSR 0x10)
- BIN(ps) = (Tref × 2^RefClkDiv) / (216 × HSDiv)
- Tref = 25 ns (40 MHz reference)

---

## 6. 불변식

| 코드 | 규칙 |
|------|------|
| — | raw_event는 sparse. hit가 없는 채널에서는 아무것도 생성되지 않음 |
| — | 같은 채널 내 hit_seq_local 0,1,2... 순서 = 시간순 (FIFO 보장) |
| — | 다른 채널 간 순서는 보장하지 않음 (cell_builder가 stop_id로 라우팅) |
| — | start_num: SINGLE_SHOT에서는 항상 0. CONTINUOUS 필드는 구조만 보존 |

---

## 7. 오류 / timeout / recovery

| 상황 | 감지 | 처리 |
|------|------|------|
| stop_id_local ≥ stops_per_chip | range check | raw_event 폐기, error count 증가 |
| hit_seq_local ≥ MAX_HITS_PER_STOP | counter overflow | raw_event는 생성하되, cell_builder가 drop |

---

## 8. testbench 시나리오

### decode_i 단위 TB

미리 구성된 28-bit word → decoded field 비교:

| 시나리오 | 입력 | 기대 출력 |
|----------|------|-----------|
| 모든 cha_code 조합 | ififo_id=0/1 × cha_code=0~3 | stop_id_local 0~7 정확 |
| HIT_SLOT_WIDTH=16 | word=0x0FF_FFFF | raw_hit=0x0FFFF (하위 17-bit) |
| start_num=0 | word[25:18]=0x00 | start_num=0 |
| slope=1 | word[17]=1 | slope=1 |
| 전체 비트 경계 | word=0xFFF_FFFF | 각 필드 bit-exact |

### raw_event_builder 단위 TB

| 시나리오 | 입력 | 기대 출력 |
|----------|------|-----------|
| stop 0에 3 hits | decoded ×3 (same stop) | hit_seq=0,1,2 |
| stop 0,1 교대 | decoded 교대 | 각 stop별 hit_seq 독립 |
| drain_done | pulse | 모든 hit_cnt reset → 다음 shot hit_seq=0 |
| 8채널 × 4 hits | 32 decoded events | 각 채널 hit_seq=0~3 |

통과 기준: 모든 입력에 대해 출력 필드 bit-exact 일치

---

## 9. 완료 조건

- [ ] decode_i: 모든 cha_code × ififo_id 조합에서 stop_id_local 정확
- [ ] raw_event_builder: hit_seq_local 증가 정상
- [ ] drain_done 시 counter reset 확인
- [ ] raw_event debug stream만으로 SW가 시간/채널 해석 가능

---

## 10. 다음 단계 handoff contract

**이 Step의 출력 = 다음 Step(03)의 입력:**

```
raw_event        : t_raw_event record
raw_event_valid  : 1-bit
drain_done       : 1-bit (shot 단위 boundary)
```

03번 문서는 이 raw_event stream을 받아서 dense cell array로 변환한다.
raw_event는 sparse (발생한 hit만 존재), cell은 dense (모든 활성 stop에 존재).
