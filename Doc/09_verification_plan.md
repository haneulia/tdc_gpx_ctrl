# 09. Verification Plan

> 참조: 00_scope_and_contract.md, 모든 하위 문서 (01~08)

---

## 0. 목적

각 Step의 testbench를 체계적으로 관리하고,
통합 테스트 순서와 regression matrix를 정의한다.
**이 문서는 각 단계가 끝날 때마다 갱신한다.**

---

## 1. 검증 레벨

### 1.1 단위 테스트 (unit test)

각 모듈을 독립적으로 검증. stub/BFM 사용.

| 모듈 | TB 파일 | 참조 문서 |
|------|---------|-----------|
| bus_phy | tb_bus_phy.vhd | 01 §8A |
| chip_ctrl | tb_chip_ctrl.vhd | 01 §8B |
| decode_i | tb_decode_i.vhd | 02 §8 |
| raw_event_builder | tb_raw_event_builder.vhd | 02 §8 |
| cell_builder | tb_cell_builder.vhd | 03 §8 |
| face_assembler | tb_face_assembler.vhd | 04 §8 |
| header_inserter | tb_header_inserter.vhd | 05 §8 |

### 1.2 통합 테스트 (integration test)

모듈 조합으로 vertical slice 검증.

```
Phase      모듈 조합                         검증 방법
──────────────────────────────────────────────────────────
Step 0     pkg + csr + shot_seq              compile + CSR BFM
Step 1A    bus_phy                            TDC behavioral model
Step 1B    bus_phy + chip_ctrl               1-chip bring-up
Step 1C    bus_phy + chip_ctrl × 4           4-chip parallel
Step 2     decode_i + raw_event_builder      단위 (조합+counter)
Step 1~2   bus_phy + ctrl + decode           raw_event 출력 확인
Step 3     cell_builder                       단위
Step 1~3   bus_phy→ctrl→decode→cell          1-chip full path
Step 4     face_assembler                     4-lane stub
Step 5     header_inserter                    assembler stub
Step 4~5   assembler + header                통합
Step 1~5   전체 datapath (4 lanes)           full system
Step 6     + VDMA + DDR                      DDR dump / board test
```

### 1.3 시스템 테스트 (system test)

실제 TDC-GPX 칩 또는 보드에서의 검증.
ILA로 raw word, cell, header를 실시간 확인.

### 1.4 SW parser 테스트

DDR dump → parser → point cloud 변환 검증.
golden vector와 bit-exact 비교.

---

## 2. 블록별 테스트 항목

### 2.1 bus_phy

- [ ] single write → readback
- [ ] burst read 64 entries
- [ ] write→read turnaround gap [INV-5]
- [ ] read→write OEN High [INV-6]
- [ ] BUS_TICKS 3/4/5/6 조합
- [ ] BUS_CLK_DIV 1/2/4 조합
- [ ] 타이밍 assertion: tPW-RL ≥ 6ns, tS-AD ≥ 2ns

### 2.2 chip_ctrl

- [ ] cfg write (Reg0~Reg7, Reg14)
- [ ] SINGLE_SHOT cycle 완주
- [ ] EF=1 시 read 중단 [INV-4]
- [ ] AluTrigger ≥ 10ns
- [ ] IFIFO 0/1/8/64 entries
- [ ] 연속 2+ shots

### 2.3 decode_i

- [ ] cha_code 0~3 × ififo_id 0/1 → stop_id 0~7
- [ ] 비트 경계 테스트
- [ ] HIT_SLOT_DATA_WIDTH=16/17

### 2.4 cell_builder

- [ ] 모든 stop 각 1 hit
- [ ] MAX_HITS full → hit_dropped=0
- [ ] MAX_HITS+1 overflow → hit_dropped=1
- [ ] hit 없는 shot → zero cell
- [ ] stops_per_chip < MAX

### 2.5 face_assembler

- [ ] 4 chip 활성 → rows=32
- [ ] 2 chip 활성 → rows 변경
- [ ] 1 chip timeout → blank + error_fill
- [ ] 모든 chip timeout → 전체 blank

### 2.6 header_inserter

- [ ] magic byte 정확
- [ ] 모든 header offset bit-exact
- [ ] rows_per_face actual 반영
- [ ] SOF/EOL 위치
- [ ] padding zero

### 2.7 VDMA dump

- [ ] DDR → parser 정상 읽기
- [ ] stride/HSIZE 가변 동작
- [ ] backpressure 처리

---

## 3. golden input / expected output

각 TB에 golden vector 파일을 사용:

```
tb/golden/
  bus_phy_single_write.txt
  bus_phy_burst_read.txt
  decode_all_channels.txt
  cell_builder_overflow.txt
  assembler_packed_row.txt
  header_full_fields.txt
  full_system_4lane.txt
```

---

## 4. regression matrix

### 4.1 active_chip_mask 조합

| mask | 활성 chips | rows_per_face (stops=8) |
|------|-----------|------------------------|
| 0b0001 | chip 0 | 8 |
| 0b0101 | chip 0,2 | 16 |
| 0b1111 | all | 32 |

### 4.2 stops_per_chip 조합

| stops | rows (4 chips) | hsize |
|-------|----------------|-------|
| 2 | 8 | 256 |
| 4 | 16 | 512 |
| 8 | 32 | 1024 |

### 4.3 MAX_HITS 시나리오

| 시나리오 | 조건 | 검증 |
|----------|------|------|
| 0 hits | empty shot | zero cells |
| 1 hit/stop | 정상 | hit_count=1 |
| 8 hits/stop | full | hit_count=8 |
| 9+ hits/stop | overflow | hit_dropped=1 |

### 4.4 timeout / error 시나리오

| 시나리오 | 조건 | 검증 |
|----------|------|------|
| lane timeout | 1 chip 무응답 | blank slice, error_fill=1 |
| shot_overrun | T_service > T_shot | STATUS bit, skip |
| ErrFlag | TDC 에러 | error_count 증가 |

---

## 5. done 기준

Phase 1 완료 기준:
- [ ] 모든 단위 TB pass
- [ ] 1-chip full path TB pass
- [ ] 4-lane full system TB pass
- [ ] DDR dump → SW parser 정상 복원
- [ ] regression matrix 전체 pass
- [ ] timing budget 검증 (650m 이하 순차, 987m 이하 pipeline)

---

## 6. release checklist

- [ ] 모든 INV 규칙 assertion 통과
- [ ] 합성 warning 0건 (expected 제외)
- [ ] timing closure (200 MHz sys_clk 기준)
- [ ] 문서-코드 일치 확인 [원칙 1]
- [ ] CSR readback 검증
- [ ] ILA 관측 포인트 동작 확인
