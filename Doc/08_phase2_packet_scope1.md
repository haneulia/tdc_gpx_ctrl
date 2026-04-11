# 08. Phase 2 - PACKET_SCOPE=1 and CELL_FORMAT=1

> 참조: 00_scope_and_contract.md, deep_analysis §5.4

---

## 0. 책임

Phase 1 완료 후 확장 설계:
- PACKET_SCOPE=1 (scan_frame 단위 전송)
- CELL_FORMAT=1 (MPSoC 64B cell)

---

## 1. PACKET_SCOPE=1

```
1 vdma_frame = N_FACES × (cols_per_face + 1) lines = 1 scan_frame
VSIZE = 12005 (5 × 2401)
DDR buffer = stride × 12005 ≈ 23.45 MB
triple buffer ≈ 70 MB

[INV-8] scan_frame 내 모든 face 동일 rows_per_face/hsize_actual
```

---

## 2. CELL_FORMAT=1 (MPSoC 64B)

Phase 2에서 설계. HIT_SLOT_DATA_WIDTH=17, TDATA_WIDTH=64 환경에서 cell을 64B로 확장.

---

## 3. header 확장

Phase 2에서 추가 검토:
- face 경계 마커 within vdma_frame (SCOPE=1)
- format_scope 필드 (header 0x3F) 활용

---

## 4. migration rule

Phase 1 → Phase 2 전환 시:
1. face_packet_assembler에 scan_frame 단위 전송 로직 추가
2. header에 face 경계 마커 삽입
3. VDMA VSIZE = N_FACES × (cols+1)
4. DDR buffer 크기 확대

기존 SINGLE_SHOT + SCOPE=0 경로는 변경 없음.

---

## 5. 완료 조건

- [ ] PACKET_SCOPE=1 DDR layout 검증
- [ ] CELL_FORMAT=1 cell 레이아웃 검증
- [ ] Phase 1 regression 통과 (기존 기능 유지)
