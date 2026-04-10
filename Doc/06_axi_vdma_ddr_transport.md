# 06. AXI VDMA DDR Transport

> 참조: 00_scope_and_contract.md, deep_analysis §5, §10

---

## 0. 책임

header + data lines AXI-Stream을 VDMA S2MM 경유 DDR3에 기록:
- axis_data_fifo: clock domain crossing + backpressure 완충
- AXI4-Stream Video 매핑 (SOF/EOL)
- VDMA 설정값 관리

**산출물**: DDR dump (SW parser가 정상 읽기 가능)

---

## 1. 비책임

- header 생성, cell 데이터 → 05번 문서
- timing/pipeline → 07번 문서
- PACKET_SCOPE=1 DDR layout → 08번 문서

---

## 2. AXI4-Stream Video 매핑

| 비디오 개념 | AXI4-Stream 대응 | 생성 방법 |
|-------------|-------------------|-----------|
| HSync | `tlast` = 1 | 매 line 마지막 beat |
| VSync | `tuser(0)` = 1 | vdma_frame 첫 beat (header first beat) |
| H Blanking | `tvalid` = 0 구간 | shot 간 측정/drain 시간 (자연 발생) |
| V Blanking | `tvalid` = 0 구간 | face 전환 시간 (자연 발생) |
| Active Video | `tvalid` = 1 구간 | cell 데이터 전송 중 |

**tuser(0)은 vdma_frame 전체에서 딱 1 beat만 1.** 이후 모든 beat에서 0.
**tlast는 마지막 데이터와 동시에 발생** (별도 빈 beat 아님).

---

## 3. VDMA 설정값

| 레지스터 | 값 | 예시 (기본) |
|----------|-----|-------------|
| S2MM_HSIZE | hsize_actual (packet_start latch) | 32 × 32 = 1024 bytes |
| S2MM_VSIZE | cols_per_face + 1 (header 포함) | 2401 lines |
| S2MM_STRIDE | 고정, ≥ HSIZE_MAX | 2048 bytes |
| Frame buffer | triple buffer 권장 | 3 × stride × VSIZE ≈ 14 MB |

**HSIZE가 가변이어도 stride는 고정** → DDR 내 line 시작 주소 = stride × line_number.

---

## 4. DDR Layout

```
base + stride × 0:  [header line     ] hsize_actual bytes + padding
base + stride × 1:  [shot 0: row 0~N ] hsize_actual bytes + padding
base + stride × 2:  [shot 1: row 0~N ]
...
base + stride × K:  [shot K-1]

K = cols_per_face
총 lines = cols_per_face + 1 = VSIZE
vdma_frame buffer = stride × VSIZE bytes
```

padding = stride - hsize_actual bytes (VDMA가 기록하지 않음).

---

## 5. axis_data_fifo

- 역할: header_inserter ↔ VDMA 사이에서 clock domain crossing + burst buffer
- 권장: depth ≥ 16 lines = 16 × HSIZE_MAX = 16 KB
- backpressure 발생 시 tready=0 → upstream 정지

---

## 6. 불변식

| 코드 | 규칙 |
|------|------|
| — | transport는 payload를 바꾸지 않음 (SOF/EOL만 부여) |
| — | stride = 2048 (고정, CSR 변경 무관) |
| [INV-11] | hsize_actual ≤ HSIZE_MAX ≤ stride |
| [INV-12] | vdma_frame 중 HSIZE 변경 금지 |

---

## 7. 오류 / timeout / recovery

| 상황 | 감지 | 처리 |
|------|------|------|
| VDMA S2MM error | VDMA IRQ | SW에서 재시작 |
| backpressure overflow | axis_data_fifo full | upstream 정지, shot_overrun 가능 |
| frame count mismatch | VDMA frame counter vs header | SW 경고 |

---

## 8. testbench 시나리오

| 시나리오 | 기대 | 통과 기준 |
|----------|------|-----------|
| DDR dump → SW parser 읽기 | header 정상 해석, cell 정확 | bit-exact |
| tuser/tlast 타이밍 | SOF: frame 첫 beat, EOL: line 끝 | 위치 정확 |
| backpressure 발생 | tready=0 → 데이터 유실 없음 | frame contract 유지 |
| hsize_actual < stride | padding 영역 미기록 | 기존 데이터 잔류 확인 |

---

## 9. 완료 조건

- [ ] DDR에 쓴 결과를 SW parser가 그대로 읽음
- [ ] tuser/tlast만 올바르게 붙고 payload 불변
- [ ] backpressure에도 frame contract 유지
- [ ] stride 고정, hsize 가변 동작 확인

---

## 10. 다음 단계 handoff contract

이 Step의 출력은 DDR3 vdma_frame buffer이다.
SW가 DDR에서 직접 읽어 parse_header() → parse_cell() → point cloud 생성.
07번 문서(timing)와 직접적 데이터 의존성은 없으나, VDMA 전송 시간(T_vdma)이 timing budget에 포함된다.
