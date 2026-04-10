# 07. Timing Budget and Pipeline

> 참조: 00_scope_and_contract.md, deep_analysis §11

---

## 0. 책임

기능 경로가 닫힌 후 성능 검증 및 최적화:
- shot 간격 내 완료 조건 검증
- T_service 구성 요소 분석
- pipeline/double buffer 적용 조건 결정

---

## 1. 비책임

- 기능 구현 (01~06에서 완료)
- CONTINUOUS 모드 timing → 08번 문서

---

## 2. 시간 항목 정의

| 항목 | 정의 | 기본값 (DIV=1, BUS_TICKS=5) |
|------|------|-----------------------------|
| T_shot | shot 간격 = T_face / cols_per_face | 8.33 μs (2400 shots/face) |
| T_roundtrip | 2 × distance / c | 가변 (1.0~10.0 μs) |
| T_drain | IFIFO drain (entries × T_read) | 1.7 μs (64 entries × 25ns + EF overhead) |
| T_cell | cell_builder → assembler 출력 | 1.0 μs |
| T_vdma | VDMA line 전송 (beats × clk_period) | 1.28 μs (256 beats × 5ns) |
| T_alu | AluTrigger + recovery | 0.05 μs |
| **T_service** | T_drain + T_cell + T_vdma + T_alu | **4.03 μs** |

T_service 정의는 문서 전체에서 **단일 정의**:
```
T_service = T_drain + T_cell + T_vdma + T_alu
```

---

## 3. shot interval 제약식

```
T_service + T_margin ≤ T_shot - T_roundtrip
T_margin ≥ 0 필수

즉: T_roundtrip + T_service < T_shot
```

---

## 4. 순차 실행 한계 (pipeline 없음)

| 최대 거리 | T_roundtrip | T_total | 옵션 A (8.33μs) | 판정 |
|-----------|-------------|---------|-----------------|------|
| 150 m | 1.0 μs | 5.0 μs | 마진 3.33 μs | OK |
| 400 m | 2.67 μs | 6.67 μs | 마진 1.66 μs | OK |
| **650 m** | **4.33 μs** | **8.33 μs** | **마진 0** | **한계** |
| 750 m | 5.0 μs | 9.0 μs | 초과 | NG |

**옵션 A + 순차: 최대 약 650m.**

---

## 5. pipeline 적용

pipeline: cell_builder double buffer → shot N의 VDMA 전송과 shot N+1의 측정을 overlap.
단, DRAIN + AluTrigger는 overlap 불가 (칩 reset 필요).

```
T_critical = T_roundtrip + T_drain + T_alu
```

| 최대 거리 | T_critical | 옵션 A | 판정 |
|-----------|------------|--------|------|
| 750 m | 6.75 μs | 마진 1.58 μs | OK |
| **987 m** | **8.33 μs** | **마진 0** | **한계** |
| 1000 m | 8.42 μs | 초과 | NG |

**옵션 A + pipeline: 최대 약 987m.**

---

## 6. overlap 가능/불가 구간

| 구간 | overlap 가능? | 이유 |
|------|---------------|------|
| CAPTURE (측정) | 가능 | 이전 shot의 cell 출력과 병렬 |
| DRAIN | **불가** | 칩 버스를 점유 |
| AluTrigger + recovery | **불가** | 칩 reset 중 |
| cell output → VDMA | 가능 (pipeline 시) | double buffer 사용 |

---

## 7. bus tick / clk div 정책

| 모드 | BUS_CLK_DIV | 이유 |
|------|-------------|------|
| SINGLE_SHOT | 1 (권장) | burst drain → 최대 속도 유리 |
| CONTINUOUS | 1 (필수) | sustained rate ≥ 40 MHz 필요 |

BUS_TICKS=5 기준: 1 read = 5 ticks × 5ns = 25ns → 40 MHz bus rate.

---

## 8. worst-case 계산 예시

```
조건:
  옵션 A (5면, 120° FOV, 2400 shots)
  T_shot = 8.33 μs
  DIV=1, BUS_TICKS=5
  전략 C (Source Gating, MAX_HITS=8)

근거리 (150m):
  T_roundtrip = 1.0 μs
  T_total = 1.0 + 4.03 = 5.03 μs → 마진 3.30 μs ✓

순차 한계 (~650m):
  T_roundtrip = 4.33 μs → T_total ≈ 8.33 μs → 마진 0

pipeline 한계 (~987m):
  T_critical = 4.33 + 1.7 + 0.05 = 6.08 μs → 마진 2.25 μs ✓
  T_critical(987m) = 6.58 + 1.7 + 0.05 = 8.33 μs → 마진 0
```

---

## 9. 완료 조건

- [ ] T_service 실측값이 이론값과 ±10% 이내
- [ ] 순차 실행 시 650m 이하에서 shot_overrun 없음
- [ ] pipeline 실행 시 987m 이하에서 shot_overrun 없음
- [ ] CSR에서 T_shot, T_roundtrip 확인 가능
- [ ] shot_overrun 감지 시 STATUS bit 설정

---

## 10. 다음 단계

timing 검증 완료 후:
- 650m 이하: 순차 실행으로 충분
- 650~987m: pipeline 적용
- 987m 이상: 옵션 B (4면, 90° FOV) 전환 또는 분해능 축소
