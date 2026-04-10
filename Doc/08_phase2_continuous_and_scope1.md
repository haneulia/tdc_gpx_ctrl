# 08. Phase 2 - CONTINUOUS and PACKET_SCOPE=1

> 참조: 00_scope_and_contract.md, deep_analysis §3A.4~3A.8, §5.4

---

## 0. 책임

Phase 1 완료 후 확장 설계:
- CONTINUOUS 측정 모드 (데이터시트 §2.11.2)
- PACKET_SCOPE=1 (scan_frame 단위 전송)

---

## 1. Phase 1과의 차이

| 항목 | SINGLE_SHOT (Phase 1) | CONTINUOUS (Phase 2) |
|------|----------------------|----------------------|
| StartTimer | 0 | > 0 (예: 39 → 1μs 주기) |
| MasterAluTrig | 1 | 0 |
| DRAIN 트리거 | IrFlag (MTimer 만료) | EF polling (상시) |
| shot 간 reset | AluTrigger 펄스 | 없음 (연속) |
| Start# | 항상 0 | 0~255 순환 |
| Start01 | 불필요 | 필수 (Reg10 read) |
| cell cycle 경계 | AluTrigger 후 clear | start_num 기반 그룹핑 |

---

## 2. CONTINUOUS FSM

```
RESET_POWERUP → CFG → MASTER_RESET → CFG_STOPDIS_LOW
    ▼
WAIT_START01
    │ 최초 외부 start 후 (StartTimer+1)×Tref 대기
    │ Reg10 read → Start01 획득
    ▼
STREAMING ◄────────────────────────┐
    │ 상시 EF1/EF2 polling        │
    │ EF=0 → 즉시 read → decode   │
    │ IrFlag(Start# overflow) 감시 │
    └──────────────────────────────┘
    ▼ (stop 명령)
STOP_REQUESTED → stopdis="1111" → 잔여 drain → IDLE
```

---

## 3. 시간 복원 (CONTINUOUS)

```
실제 시간(ps) = (raw_hit - StartOff1 + Start01) × BIN
             + (Start# - 1) × (StartTimer + 1) × Tref

if (raw_hit - StartOff1) < 0:
    → 이전 start로 remap: Start# -= 1, time += (StartTimer+1)×Tref
```

---

## 4. cell cycle 재정의

SINGLE_SHOT: 1 shot = 1 cell cycle (AluTrigger 후 clear)
CONTINUOUS: cell cycle 기준 미정 — 후보:
1. start_num 변경 시 새 cycle
2. 외부 sync 신호 기반
3. 시간 기반 분할

→ Phase 2 설계 시 결정

---

## 5. PACKET_SCOPE=1

```
1 vdma_frame = N_FACES × (cols_per_face + 1) lines = 1 scan_frame
VSIZE = 12005 (5 × 2401)
DDR buffer = stride × 12005 ≈ 23.45 MB
triple buffer ≈ 70 MB

[INV-8] scan_frame 내 모든 face 동일 rows_per_face/hsize_actual
```

---

## 6. header 확장 필드

Phase 1에서 이미 구조 확보된 필드:
- measurement_mode (0x0D): 1=CONTINUOUS
- start01 (0x24): Reg10 측정값
- start_timer (0x28): StartTimer 값

Phase 2에서 추가 검토:
- face 경계 마커 within vdma_frame (SCOPE=1)
- Start# overflow 카운터

---

## 7. migration rule

Phase 1 → Phase 2 전환 시:
1. chip_ctrl에 CONTINUOUS FSM 분기 추가 (meas_mode=1)
2. cell_builder에 cell cycle 재정의 로직 추가
3. header_inserter에 start01/start_timer 실측값 반영
4. SW parser에 CONTINUOUS 시간 복원 분기 추가

기존 SINGLE_SHOT 경로는 변경 없음.

---

## 8. 완료 조건

- [ ] CONTINUOUS FSM 정상 동작 (상시 EF polling)
- [ ] Start# overflow → 외부 counter 확장
- [ ] PACKET_SCOPE=1 DDR layout 검증
- [ ] SW parser CONTINUOUS 분기 정상
- [ ] Phase 1 regression 통과 (기존 기능 유지)
