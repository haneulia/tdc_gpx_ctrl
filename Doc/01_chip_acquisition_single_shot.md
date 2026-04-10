# 01. Chip Acquisition - SINGLE_SHOT

> 참조: 00_scope_and_contract.md, deep_analysis §2, §3A, §5.2.6D

---

## 0. 책임

1-chip SINGLE_SHOT bring-up 경로 전체:
- bus_phy: 28-bit 비동기 병렬 버스 ↔ req/rsp 변환
- chip_ctrl: SINGLE_SHOT FSM (init → measure → drain → reset)
- cfg_image: TDC-GPX 레지스터 초기 write
- 물리 핀 제어: StopDis, AluTrigger, PuResN, EF/LF/IrFlag sync

**산출물**: ILA/UART로 관측 가능한 raw 28-bit word

---

## 1. 비책임

- decode_i (raw word → structured fields) → 02번 문서
- raw_event 스키마, hit_seq_local → 02번 문서
- cell 변환, face 조립 → 03~04번 문서
- CONTINUOUS 모드 → 08번 문서

---

## 2. 입력

### bus_phy 입력 (chip_ctrl → bus_phy)

| 신호 | 폭 | 설명 |
|------|-----|------|
| `bus_req_valid` | 1 | transaction 요청 |
| `bus_req_rw` | 1 | 0=READ, 1=WRITE |
| `bus_req_addr` | 4 | TDC-GPX 레지스터 주소 (0~14) |
| `bus_req_wdata` | 28 | write 데이터 |
| `oen_permanent` | 1 | 1=drain 중 OEN 상시 low |

### chip_ctrl 입력

| 신호 | 폭 | 설명 |
|------|-----|------|
| `shot_start` | 1 | laser_ctrl에서 start pulse |
| `csr` | record | t_csr_ctrl (00번 문서 §3.3 참조) |
| `bus_rsp` | record | t_bus_rsp (bus_phy에서) |
| `ef1_sync, ef2_sync` | 1 each | EF 핀 2-FF sync |
| `lf1_sync, lf2_sync` | 1 each | LF 핀 2-FF sync |
| `irflag_sync` | 1 | IrFlag 핀 2-FF sync |

---

## 3. 출력

### bus_phy 출력

| 신호 | 폭 | 설명 |
|------|-----|------|
| `bus_rsp_valid` | 1 | read 데이터 유효 (1 tick pulse) |
| `bus_rsp_rdata` | 28 | read 데이터 (IOB FF output) |
| `bus_busy` | 1 | transaction 진행 중 |

### bus_phy → 물리 핀

| 핀 | 폭 | 방향 | 설명 |
|----|-----|------|------|
| `adr_pin` | 4 | out | TDC-GPX ADR[3:0] |
| `csn_pin` | 1 | out | CSN (active low) |
| `rdn_pin` | 1 | out | RDN (active low) |
| `wrn_pin` | 1 | out | WRN (active low) |
| `oen_pin` | 1 | out | OEN (active low) |
| `d_bus_pin` | 28 | bidir | D[27:0] (IOBUF) |
| `stopdis_pin` | 4 | out | StopDis1~4 |
| `alutrigger_pin` | 1 | out | AluTrigger |
| `puresn_pin` | 1 | out | PuResN (active low) |

### chip_ctrl 출력

| 신호 | 폭 | 설명 |
|------|-----|------|
| `raw_word` | 28 | IFIFO에서 읽은 raw word |
| `raw_word_valid` | 1 | raw word 유효 |
| `ififo_id` | 1 | 0=IFIFO1(Reg8), 1=IFIFO2(Reg9) |
| `drain_done` | 1 | 현재 shot drain 완료 |
| `bus_req` | record | t_bus_req (bus_phy로) |

---

## 4. 제너릭 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `BUS_CLK_FREQ` | 200_000_000 | bus_clk 주파수 (Hz) |

CSR에서 런타임 설정:
- `BUS_CLK_DIV` (기본 1): tick enable 분주비
- `BUS_TICKS` (기본 5): ticks/transaction

---

## 5. 동작 명세

### 5.1 bus_phy: Tick-Phase 구조

```
1회 transaction = BUS_TICKS ticks

Phase A (ADR setup):   k_adr = 1 tick (고정)
Phase L (strobe low):  k_rdl = BUS_TICKS - k_adr - k_rdh (자동 산출)
Phase H (strobe high): k_rdh = 1 tick (기본)

┌──────┐ ┌──────────────────────┐ ┌──────────┐
│Phase A│ │       Phase L        │ │ Phase H  │
│k_adr │ │       k_rdl          │ │ k_rdh    │
│ ticks│ │       ticks          │ │ ticks    │
└──────┘ └──────────────────────┘ └──────────┘
│← ADR →│← RDN/WRN low ────────→│← high ──→│
  setup    strobe + data valid     turnaround
```

BUS_TICKS=5 (기본): k_adr=1, k_rdl=3, k_rdh=1

**READ**: Phase L 마지막 tick에서 IOB FF sample → bus_rsp_valid
**WRITE**: OEN 반드시 High [INV-1], Phase L 동안 WRN low

### 5.2 bus_phy: EF/LF 2-FF Sync

```vhdl
-- EF, LF, IrFlag, ErrFlag: 비동기 핀 → 2-FF synchronizer
-- HIGH active: ef1_sync='1' = IFIFO1 empty
attribute IOB : string;
attribute IOB of ef1_pin_ff : signal is "TRUE";
```

### 5.3 bus_phy: READ↔WRITE turnaround

```
READ → WRITE: OEN=1 설정 → 양쪽 Hi-Z 1 tick → WRITE 시작 [INV-5]
WRITE → READ: d_bus_oe=0 → 양쪽 Hi-Z 1 tick → OEN=0 → READ 시작 [INV-6]
```

### 5.4 chip_ctrl: SINGLE_SHOT FSM

```
RESET_POWERUP
  │ PuResN Low → High
  ▼
CFG_STOPDIS_HIGH
  │ stopdis = "1111" (모든 stop 차단)
  ▼
CFG_WRITE_SEQ
  │ Reg0~Reg7, Reg11, Reg12, Reg14 순차 write
  │ (cfg_image에서 값 읽어서 칩에 write)
  ▼
MASTER_RESET
  │ Reg4 bit22 = '1' write
  ▼
WAIT_RESET_RECOVERY
  │ 40 ns 대기
  ▼
CFG_STOPDIS_LOW
  │ stopdis = "0000" (stop 허용)
  ▼
ARMED ◄────────────────────────────────────────┐
  │ 외부 start pulse 대기                      │
  ▼                                            │
CAPTURE                                        │
  │ MTimer 카운트 중, hit 수집 중              │
  │ IrFlag rising edge 감시                    │
  ▼ (IrFlag 감지)                              │
DRAIN_IFIFO                                    │
  │ EF1 우선 round-robin:                      │
  │   EF1=0 → Reg8 read → raw_word 출력       │
  │   EF2=0 → Reg9 read → raw_word 출력       │
  │ 양쪽 empty까지 반복                        │
  ▼                                            │
ALUTRIGGER_PULSE                               │
  │ alutrigger = '1' (1 bus_tick 이상)         │
  │ alutrigger = '0'                           │
  ▼                                            │
WAIT_NEXT_SHOT                                 │
  │ reset recovery 대기 (40 ns)                │
  └────────────────────────────────────────────┘
```

### 5.5 drain 정책: EF1 우선 round-robin

```
while (ef1_sync = '0') or (ef2_sync = '0') loop
    if ef1_sync = '0' then                    -- IFIFO1 not empty
        bus_req(READ, Reg8) → raw_word, ififo_id=0
    end if;
    if ef2_sync = '0' then                    -- IFIFO2 not empty
        bus_req(READ, Reg9) → raw_word, ififo_id=1
    end if;
end loop;
```

EF1 우선 이유: stop 0~3이 VDMA line 앞쪽, stop 4~7이 뒤쪽.
cell_builder가 stop_id_local로 라우팅하므로, 같은 채널 내 시간순은 FIFO 구조가 보장.

### 5.6 cfg_image

```
CSR 0x20~0x5F: Reg0~Reg14 × 32-bit each
cfg_write_trigger(CSR 0x60 bit3) → chip_ctrl가 CFG_WRITE_SEQ 재진입

핵심 설정 (SINGLE_SHOT):
  Reg4: StartTimer=0, MasterAluTrig=1, MTimerStart=1
  Reg5: StartOff1[17:0]
  Reg7: HSDiv, RefClkDiv, MTimer
  Reg12: IrFlag mask (MTimer expire)
```

---

## 6. 불변식

| 코드 | 규칙 | 위치 |
|------|------|------|
| [INV-1] | WRITE 중 OEN = 반드시 High | bus_phy |
| [INV-2] | READ 중 FPGA D-bus = Hi-Z | bus_phy |
| [INV-3] | IDLE 중 D-bus = Hi-Z, OEN = High | bus_phy |
| [INV-4] | EF=1이면 read 금지 | chip_ctrl |
| [INV-5] | WRITE→READ: turnaround gap | bus_phy |
| [INV-6] | READ→WRITE: OEN High 선행 | bus_phy |
| [INV-7] | OEN permanent low 모드에서 WRITE 금지 | bus_phy |

---

## 7. 오류 / timeout / recovery

| 상황 | 감지 | 처리 |
|------|------|------|
| shot_overrun | ARMED 복귀 전 start pulse 도착 | CSR STATUS bit1 set, 해당 shot skip |
| ErrFlag active | 2-FF sync | CSR error_count 증가, optional IRQ |
| IrFlag timeout | CAPTURE에서 설정 시간 초과 | soft_error, ALUTRIGGER → ARMED |
| bus_phy stuck | tick_cnt overflow | watchdog reset |

---

## 8. testbench 시나리오

### 8A. bus_phy 단독 TB (sub-step 1A)

TDC-GPX behavioral model (bus timing checker) 필요:
- read 시 tV-DR 이후 데이터 drive
- 타이밍 assertion: tPW-RL ≥ 6ns, tS-AD ≥ 2ns, tPW-RH ≥ 10ns (back-to-back)

| 시나리오 | 입력 | 기대 출력 | 통과 기준 |
|----------|------|-----------|-----------|
| single write → readback | addr=0, wdata=0x1234 | rdata=0x1234 | 값 일치 |
| burst read (64 entries) | addr=8, back-to-back | 64개 rdata | 타이밍 assertion 통과 |
| write → read 전환 | write addr=0, read addr=8 | turnaround gap ≥ 1 tick | [INV-5] 위반 없음 |
| BUS_TICKS 변경 | 3/4/5/6 | 모든 경우 정상 | Phase 분배 정확 |
| BUS_CLK_DIV 변경 | 1/2/4 | tick_en 주기 정확 | div_cnt 정상 |

### 8B. bus_phy + chip_ctrl 통합 TB (sub-step 1B)

bus_phy에 TDC behavioral model 연결:

| 시나리오 | 기대 | 통과 기준 |
|----------|------|-----------|
| cfg write (Reg0~Reg7, Reg14) | 칩 레지스터에 정확한 값 | readback 일치 |
| shot_start → IrFlag → drain | IFIFO 데이터 → raw_word 출력 | entries 수 일치 |
| EF=1일 때 read 중단 | drain 정상 종료 | [INV-4] 위반 없음 |
| AluTrigger 펄스 | ≥ 10ns HIGH | 파형 확인 |
| IFIFO 0/1/8/64 entries | raw_word 출력 수 일치 | bit-exact |
| 연속 2 shots | ARMED→...→ARMED→...→ARMED | FSM 정상 순환 |

### 8C. 4-chip 병렬 (sub-step 1C)

4개 lane의 bus_phy + chip_ctrl 인스턴스:

| 시나리오 | 기대 | 통과 기준 |
|----------|------|-----------|
| 4 chip 동시 drain | 4개 raw_word stream | lane별 독립 |
| 1 chip timeout | 해당 lane만 에러 | 나머지 3 lane 정상 |

---

## 9. 완료 조건

- [ ] bus_phy 단독 TB: 모든 INV 위반 없음, 타이밍 assertion 통과
- [ ] chip_ctrl TB: SINGLE_SHOT cycle (ARMED→CAPTURE→DRAIN→ALU→ARMED) 재현
- [ ] cfg write: Reg0~Reg7 정확한 값 확인
- [ ] EF=1일 때 read 하지 않음 [INV-4]
- [ ] raw 28-bit word가 ILA/UART debug로 관측 가능
- [ ] 연속 shot 동작 확인

---

## 10. 다음 단계 handoff contract

**이 Step의 출력 = 다음 Step(02)의 입력:**

```
raw_word     : 28-bit (IFIFO에서 읽은 그대로)
raw_word_valid : 1-bit (유효 pulse)
ififo_id     : 1-bit (0=IFIFO1/Reg8, 1=IFIFO2/Reg9)
chip_id      : 2-bit (어떤 lane인가)
shot_seq     : 32-bit (shot 순번, shot_seq_gen에서)
drain_done   : 1-bit (현재 shot의 drain 완료)
```

02번 문서는 이 신호를 받아서 structured fields로 decode하고 raw_event record를 생성한다.
