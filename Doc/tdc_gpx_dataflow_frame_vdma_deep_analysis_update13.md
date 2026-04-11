# TDC-GPX 데이터 흐름 · 프레임 구조 · VDMA 연계 심층 분석

> 작성일: 2026-04-09  
> 목적: 용어 정의 → 시간순 데이터 흐름 → 프레임 구조화 → VDMA 전송 → DDR 복원까지  
> 전체 과정을 한 번에 추적 가능하도록 명료하게 정리
>
> **[2026-04-10 설계 결정] CONTINUOUS 측정 모드(§2.11.2)는 구현 범위에서 전면 배제.**
> 본 문서의 CONTINUOUS 관련 내용(§3A.4, §3A.8, start_num 필드, Start# 순환, StartTimer>0 등)은
> 참고용으로만 보존하며, 실제 구현 문서(00~09)에는 SINGLE_SHOT 모드만 반영한다.
> 이에 따라 start_num 필드는 raw_event/cell 구조에서 제거되었고, CELL_SIZE_BYTES 산출식도 변경되었다.

---

## 제1장. 용어 정의

이 문서에서 사용하는 모든 용어를 먼저 확정한다. 이후 본문에서는 이 정의만 사용한다.

### 1.1 물리 세계 용어

| 용어 | 정의 | 비고 |
|---|---|---|
| **폴리곤 미러** | 회전하는 다면 거울. 레이저를 반사시켜 수평 방향으로 스캔 | 면 수 = 보통 4~6 |
| **face** | 폴리곤 미러의 한 면. 한 face가 지나가는 동안 수평 화각 전체를 스캔 | face_id = 0, 1, 2, ... |
| **수평 샘플** | 한 face 안에서 레이저가 발사되는 각 위치. 시간 축으로 순차 발생 | = shot = column |
| **수직 채널** | 포토다이오드 배열의 한 줄. 수직 화각 방향으로 분포 | = stop channel = row |
| **에코** | 레이저 펄스가 물체에 맞고 돌아온 반사광 | 1개 발사에 여러 에코 가능 |
| **multi-hit** | 한 수평 샘플에서, 한 수직 채널이 여러 개의 에코를 수신하는 것 | 가까운 물체 + 먼 물체 동시 반사 등 |

### 1.2 시스템 용어

| 용어 | 정의 | 생성 주체 |
|---|---|---|
| **shot** | 하나의 레이저 발사~다음 발사 사이의 측정 구간. 정확히 1개의 start pulse에 대응 | laser_ctrl |
| **shot_seq** | 레이저 발사 순번. start pulse마다 1씩 증가하는 32-bit counter. "몇 번째 쐈는가" | tdc_gpx_shot_seq_gen |
| **col_in_face** | 현재 face 안에서 이 shot이 몇 번째 수평 샘플인지 | scheduler 또는 shot_seq_gen |
| **row_in_face** | Packed Row 인덱스. 0 ~ rows_per_face-1 (가변). header의 active_chip_mask, stops_per_chip으로 chip_id/stop_id 복원 | cell_builder |
| **rows_per_face** | VDMA line 내 총 pixel 수 = count_ones(active_chip_mask) × stops_per_chip. 런타임 가변 (packet_start 시 latch). 기본값 4 × 8 = 32. 최대 N_CHIPS × MAX_STOPS_PER_CHIP | CSR 종속, packet_start latch |
| **stops_per_chip** | 칩당 활성 stop 채널 수. MAX_STOPS_PER_CHIP 이하. Packed Row에서는 비활성 stop의 row가 존재하지 않음 | CSR |
| **MAX_STOPS_PER_CHIP** | 칩당 최대 stop 수. TDC-GPX I-Mode 기준 8. 합성 시 제너릭. stops_per_chip과 구별 | 합성 시 제너릭 |
| **N_CHIPS** | 병렬 TDC-GPX 칩 수. 기본 4. 합성 시 제너릭 | 합성 시 제너릭 |
| **segment** | 1 shot의 활성 수직 채널 데이터. rows_per_face pixels (가변). 비활성 row는 포함되지 않음 (Packed Row). VDMA 기준 1 line | face_packet_assembler |
| **vdma_frame** | 1 face 스캔의 전체 데이터 = VDMA의 SOF-to-SOF 단위. cols_per_face × rows_per_face cells + header. 이전 문서의 "packet"에 해당. PACKET_SCOPE=1이면 N_FACES faces = 1 vdma_frame | face_packet_assembler |
| **scan_frame** | 폴리곤 미러 1회전의 전체 데이터 = LiDAR 시스템 관점의 1 프레임. N_FACES개의 vdma_frame(SCOPE=0) 또는 1개의 vdma_frame(SCOPE=1)으로 구성 | system level |
| **vdma_frame_id** | vdma_frame의 순번. face가 바뀔 때마다 증가 (SCOPE=0) 또는 회전마다 증가 (SCOPE=1) | system level |
| **scan_frame_id** | scan_frame의 순번. 미러 1회전이 끝날 때마다 증가 | system level |

**용어 혼동 방지:**

| 문맥 | "frame"이 의미하는 것 | 본 문서의 용어 |
|---|---|---|
| AXI VDMA / AXI4-Stream Video | SOF-to-SOF 단위 | **vdma_frame** |
| LiDAR point cloud / SW | 미러 1회전 = 360° 스캔 1장 | **scan_frame** |
| VDMA 레지스터 (VSIZE, frame count) | vdma_frame 기준 | **vdma_frame** |
| DDR buffer 주소 계산 | vdma_frame 단위 | **vdma_frame** |

### 1.3 TDC-GPX 칩 용어

| 용어 | 정의 | 출처 |
|---|---|---|
| **chip_id** | 0~3. 어떤 TDC-GPX 칩인가 | FPGA 설계 |
| **lane** | chip_id와 1:1 대응되는 독립 bus 경로 | FPGA 설계 |
| **TStart** | TDC-GPX의 측정 시작 입력 핀. laser_ctrl가 생성 | 물리 핀 |
| **TStop1~8** | TDC-GPX의 측정 중지 입력 핀. echo_receiver가 생성 | 물리 핀 |
| **stop_id_local** | 칩 내부 stop 채널 번호. 0 ~ stops_per_chip-1. Packed Row에서는 활성 stop만 포함 | decode에서 복원 |
| **cha_code_raw** | Reg8/Reg9에서 읽은 2-bit 채널 코드 | 칩 출력 |
| **ififo_id** | 0 = IFIFO1(Reg8, ch1~4), 1 = IFIFO2(Reg9, ch5~8) | 칩 구조 |
| ~~**start_num**~~ | ~~이 hit가 귀속된 start window 번호. SINGLE_SHOT에서는 항상 0~~ | **[삭제됨] SINGLE_SHOT 전용 설계로 불필요** |
| **raw_hit** | Stop - Start 시간 차이. 17-bit, BIN 단위 | 칩 출력 |
| **slope** | 에지 방향. 1=rising, 0=falling | 칩 출력 |
| **Hit FIFO** | 칩 내부, 채널별 32-stage FIFO. 같은 채널의 hit를 시간순으로 보관 | 칩 내부 |
| **IFIFO** | Interface FIFO. Hit FIFO에서 collecting logic을 거쳐 외부로 나오는 FIFO | 칩 내부 |
| **EF** | Empty Flag. EF=1이면 IFIFO가 비어있음. EF=0이면 데이터 있음 | 칩 출력 핀 |
| **cfg_image** | FPGA 내부에 유지하는 "칩에 써줄 설정 값". 칩의 readback과 별개 | FPGA 설계 |
| **StopDis1~4** | Stop 입력을 쌍으로 비활성화하는 물리 핀. StopDis1→TStop1/2, StopDis2→TStop3/4, StopDis3→TStop5/6, StopDis4→TStop7/8. High=차단, Low=허용 | 물리 핀 (FPGA 출력) |
| **AluTrigger** | ALU 트리거 / Master Reset 트리거 물리 핀. Reg5의 MasterAluTrig=1일 때, HIGH 레벨 펄스 (폭 ≥ 10ns) 로 master reset 수행 | 물리 핀 (FPGA 출력) |
| **IrFlag** | TDC-GPX 인터럽트 플래그 출력. MTimer 만료 시 active | 물리 핀 (FPGA 입력) |
| **ErrFlag** | TDC-GPX 에러 플래그 출력. Reg11에서 unmask된 에러 발생 시 active | 물리 핀 (FPGA 입력) |

### 1.4 측정 모드 용어

| 용어 | 정의 | 데이터시트 참조 |
|---|---|---|
| **SINGLE_SHOT** | §2.11.1 방식. 외부 start 1회 → MTimer 윈도우 내 hit 수집 → IrFlag 대기 → IFIFO drain → AluTrigger reset → 반복. StartTimer=0, MasterAluTrig=1 | §2.11.1 Single measurement |
| ~~**CONTINUOUS**~~ | ~~§2.11.2 방식. 외부 start 1회 후 내부 retrigger로 연속 start 생성~~ | **[배제됨] 구현 범위에서 제외** |

### 1.5 FPGA 내부 데이터 용어

| 용어 | 정의 | 생성 주체 |
|---|---|---|
| **raw_event** | decode된 한 건의 TDC hit 정보. sparse, 발생한 것만 존재 | raw_event_builder |
| **hit_seq_local** | 같은 {chip_id, shot_seq, stop_id_local} 안에서의 hit 순번 | raw_event_builder |
| **cell** | vdma_frame 격자의 한 칸. 좌표 = {col_in_face, row_in_face}. 내부에 MAX_HITS_PER_STOP개의 hit slot 보유 | cell_builder |
| **cell_size** | cell 하나의 바이트 크기. VHDL: `CELL_SIZE_BYTES`. MAX_HITS_PER_STOP, HIT_SLOT_DATA_WIDTH, cell_format으로부터 산출되는 종속 상수. VDMA의 bytes_per_pixel과 동일 | 종속 상수 |
| **hit slot** | cell 안의 한 자리. 실제 hit가 채워지거나 zero-padding됨 | cell_builder |
| **pixel** | VDMA 관점에서 AXI4-Stream의 한 전송 단위. 내용 = cell의 hit slot 집합 | vdma_frame output |
| **line** | VDMA 관점에서 한 줄 = 1 segment. 내용 = 한 shot의 모든 수직 채널 pixel | vdma_frame output |

### 1.6 전송 용어

| 용어 | 정의 | 비고 |
|---|---|---|
| **SOF** | Start Of Frame. VDMA 규약에서 tuser(0)=1. 본 설계에서는 vdma_frame 시작을 의미 | vdma_frame 첫 pixel |
| **EOL** | End Of Line. VDMA 규약에서 tlast=1 | line(= segment) 마지막 pixel |
| **S2MM** | Stream to Memory-Mapped. VDMA가 AXI-Stream을 받아 DDR에 쓰는 방향 | Xilinx VDMA |
| **vdma_frame buffer** | DDR 내 연속 메모리 영역. VDMA가 vdma_frame 하나를 저장하는 공간 | DDR3 |

### 1.7 문서 용어 ↔ VHDL 식별자 매핑

문서 텍스트에서는 가독성을 위해 소문자 snake_case를 사용한다. VHDL 코드에서는 제너릭/상수는 UPPER_CASE, 신호는 소문자를 사용한다. 아래 표로 정확히 대응시킨다.

**제너릭 (합성 시 설정, 런타임 불변):**

| 문서 용어 | VHDL generic | 기본값 | 의미 |
|---|---|---|---|
| — | `N_CHIPS` | 4 | 병렬 칩 수 |
| — | `MAX_STOPS_PER_CHIP` | 8 | 칩당 최대 stop 수 |
| — | `MAX_HITS_PER_STOP` | 8 | cell 내 hit slot 수 |
| — | `HIT_SLOT_DATA_WIDTH` | 16 | hit slot 비트폭 |
| — | `TDATA_WIDTH` | 32 | AXI bus 폭 (bits) |

**종속 상수 (제너릭으로부터 자동 산출, 런타임 불변):**

| 문서 용어 | VHDL constant | 산출식 | 기본값 |
|---|---|---|---|
| MAX_ROWS_PER_FACE | `MAX_ROWS_PER_FACE` | N_CHIPS × MAX_STOPS_PER_CHIP | 32 (제너릭, stride/buffer 기준) |
| cell_size | `CELL_SIZE_BYTES` | f(MAX_HITS_PER_STOP, HIT_SLOT_DATA_WIDTH) | 32 |
| — | `MAX_VDMA_HSIZE_BYTES` | MAX_ROWS_PER_FACE × CELL_SIZE_BYTES | 1024 (stride 기준) |
| — | `TDATA_BYTES` | TDATA_WIDTH / 8 | 4 |
| — | `BITS_PER_PIXEL` | CELL_SIZE_BYTES / TDATA_BYTES | 8 |
| — | `MAX_BITS_PER_LINE` | MAX_ROWS_PER_FACE × BITS_PER_PIXEL | 256 (stride 기준) |

**CSR 레지스터 (§9A 참조):**

| 문서 용어 | VHDL CSR | 분류 | latch 신호 | 의미 |
|---|---|---|---|---|
| MAX_ROWS_PER_FACE | `MAX_ROWS_PER_FACE` (0x78) | **R/O** (Status) | — (상수) | 제너릭 최댓값 (stride/buffer 기준) |
| cell_size | `CELL_SIZE_BYTES` (0x7A) | **R/O** (Status) | — (상수) | 종속 상수 노출 |
| active_chip_mask | `ACTIVE_CHIP_MASK` (0x00) | **R/W** (Control) | `pkt_active_mask` | 활성 chip |
| stops_per_chip | `STOPS_PER_CHIP` (0x01) | **R/W** (Control) | `pkt_stops_per_chip` | 활성 stop 수 |
| cols_per_face | `COLS_PER_FACE` (0x02) | **R/W** (Control) | `pkt_cols` | face당 shot 수 |

**VHDL 신호:**

| 문서 용어 | VHDL 신호 | 의미 |
|---|---|---|
| row_in_face | `row_in_face` | pixel 물리 인덱스 |
| chip_id | `chip_id` | 칩 번호 (0~N_CHIPS-1) |
| stop_id_local | `stop_id_local` | 칩 내 stop 번호 (0~MAX_STOPS_PER_CHIP-1) |
| pixel_index | `pixel_index` | Packed Row 순번 (header 기반 동적 매핑, §1A.6) |
| shot_seq | `shot_seq` | 레이저 발사 순번 |
| col_in_face | `col_in_face` | face 내 shot 순번 |
| raw_hit | `raw_hit` | IFIFO에서 읽은 Hit 값 |
| — | `ef1_active`, `ef2_active` | EF 핀 2-FF sync. HIGH active: '1'=empty, '0'=데이터 있음 |
| — | `lf1_active`, `lf2_active` | LF 핀 2-FF sync. HIGH active: '1'=Fill 이상, '0'=Fill 미만 |

**규칙:**
- 문서에서 `cell_size`라고 쓰면 VHDL의 `CELL_SIZE_BYTES`를 의미한다
- `rows_per_face`: 런타임 가변값 (= active_chips × stops_per_chip, packet_start latch)
- `MAX_ROWS_PER_FACE`: 제너릭 상수 (= N_CHIPS × MAX_STOPS_PER_CHIP, stride/buffer 기준)
- `stops_per_chip` (CSR, 가변)과 `MAX_STOPS_PER_CHIP` (제너릭, 고정)은 **다른 변수**이다

### 1.8 설계 계획 문서 원칙

본 문서는 RTL 구현의 유일한 상위 설계 문서(single source of truth)이다. 다음 원칙을 반드시 준수한다.

**[원칙 1] 문서-코드 일치 (Document-Code Consistency)**

문서에 기술된 모든 변수명, 수식, 동작 설명은 VHDL 코드와 정확히 일치해야 한다. 코드를 변경하면 문서도 함께 변경하고, 문서를 변경하면 코드도 함께 변경한다. 둘 사이에 불일치가 발견되면 즉시 교정한다.

```
위반 예시:
  문서: "HSIZE = rows_per_face × cell_size"
  코드: hsize_reg <= active_chips * stops_per_chip * cell_size;
  → rows_per_face는 CSR 종속 가변 (Packed Row 정책, §1A.6)

준수 예시:
  문서: "hsize_actual = rows_per_face × CELL_SIZE_BYTES"
  코드: signal hsize_actual : natural := rows_per_face * CELL_SIZE_BYTES;
  → 변수명, 산출식 모두 일치
```

**[원칙 2] 용어 정의 우선 (Terminology-First)**

모든 변수는 §1장 용어 정의에 먼저 등록한 후 본문에서 사용한다. §1.6 매핑표에 문서 용어와 VHDL 식별자의 대응이 명시되어 있어야 한다. 새 변수를 도입할 때는 반드시 용어 정의를 먼저 추가한다.

```
새 변수 도입 절차:
  1. §1장 용어 정의표에 행 추가 (의미, 생성 주체)
  2. §1.6 매핑표에 문서 용어 ↔ VHDL 식별자 대응 추가
  3. 본문에서 사용
  4. VHDL 코드에서 동일 식별자로 선언
```

**[원칙 3] 독립변수/종속변수 분리 (Independent/Dependent Separation)**

제너릭에는 독립변수만 포함한다. 종속변수는 독립변수로부터 자동 산출되는 상수로 선언한다. CSR에는 런타임 가변 파라미터만 포함한다.

```
독립변수 (generic): N_CHIPS, MAX_STOPS_PER_CHIP, MAX_HITS_PER_STOP, HIT_SLOT_DATA_WIDTH, TDATA_WIDTH, CELL_FORMAT
종속상수 (constant): MAX_ROWS_PER_FACE, CELL_SIZE_BYTES, MAX_VDMA_HSIZE_BYTES, BITS_PER_PIXEL, MAX_BITS_PER_LINE
런타임 산출 (signal): rows_per_face, hsize_actual, beats_per_line (packet_start latch)
런타임 가변 (CSR):   active_chip_mask, stops_per_chip, cols_per_face, PACKET_SCOPE, ...
```

**[원칙 4] 약칭 금지 (No Undeclared Abbreviations)**

§1장에 등록되지 않은 약칭을 본문이나 코드에서 사용하지 않는다. 예를 들어 `R`(→ rows_per_face), `N`(→ cols_per_face) 같은 한 글자 약칭은 금지한다. 모든 변수는 의미가 드러나는 이름을 사용한다.

**[원칙 5] 불변식 명시 (Invariant Documentation)**

FSM 상태별 제약, 핀 제어 규칙, Packed Row 규칙 등 "위반하면 안 되는 조건"은 `[INV-N]` 태그를 붙여 문서에 명시하고, VHDL 코드에서도 동일한 태그를 주석으로 표기한다.

```
문서:  [INV-4] EF=1이면 Reg8/Reg9 read 금지
코드:  -- [INV-4] empty FIFO read 금지
       if ef1_active = '0' then ...
```

**[원칙 6] 변경 이력 추적 (Change Traceability)**

설계 결정이 변경될 때는 이전 결정과 변경 사유를 기록한다. 예: "매핑 B 제거 → 이유: transpose buffer 1.8MB 비현실적"

---

## 제1A장. 시스템 파라미터 (설계 기준값)

### 1A.1 폴리곤 미러 광학 원리

폴리곤 미러에서 반사된 빔의 스캔 각도는 미러 회전 각도의 **2배**이다 (반사 법칙).

```
물리적 화각 = 미러 면이 차지하는 기하학적 각도
광학적 화각 = 반사 빔이 실제로 쓸어가는 각도 = 물리적 화각 × 2

예: 미러가 30° 회전하면 반사 빔은 60° 이동
```

face 가장자리에서 빔 클리핑(facet transition dead zone)이 발생하므로, 사용 가능한 물리적 화각은 기하학적 face span보다 작다.

### 1A.2 스캐너 구성 옵션

| 파라미터 | **옵션 A (기본)** | 옵션 B |
|---|---|---|
| N_FACES | **5** | 4 |
| 기하학적 face span | 360°/5 = 72° | 360°/4 = 90° |
| 사용 가능 물리적 화각 | **60°** (dead zone 12°) | 45° (dead zone 45°) |
| **광학적 화각 (FOV)** | **120°** | **90°** |
| 미러 1회전 | 100 ms (10 rps) | 100 ms (10 rps) |

### 1A.3 산출값

| 파라미터 | 산출식 | **옵션 A (기본)** | 옵션 B |
|---|---|---|---|
| **cols_per_face** | 광학적 FOV / 분해능 | 120° / 0.05° = **2400** | 90° / 0.05° = **1800** |
| face 지속 시간 | 100 ms / N_FACES | **20 ms** | **25 ms** |
| shot 간격 | face 시간 / cols | 20ms/2400 = **8.33 μs** | 25ms/1800 = **13.89 μs** |
| laser 반복률 | 1 / shot 간격 | **120 kHz** | **72 kHz** |
| rows_per_face | active × stops | **32 (기본)** | **32 (기본)** |
| 수직 전체 채널 | N_FACES × 32 | **160 ch** | **128 ch** |

### 1A.4 타이밍 영향 비교 (전략 C, DIV=1 기준)

**순차 실행 (pipeline 없음):** T_total = T_roundtrip + T_service(4.0μs)

| 최대 거리 | T_roundtrip | T_total | **옵션 A** (8.33μs) | **옵션 B** (13.89μs) |
|---|---|---|---|---|
| 150 m | 1.0 μs | 5.0 μs | ✓ 마진 3.33 | ✓ 마진 8.89 |
| 400 m | 2.67 μs | 6.67 μs | ✓ 마진 1.66 | ✓ 마진 7.22 |
| **650 m** | **4.33 μs** | **8.33 μs** | **← 한계** | ✓ 마진 5.56 |
| 750 m | 5.0 μs | 9.0 μs | **✗ 초과** | ✓ 마진 4.89 |
| 1000 m | 6.67 μs | 10.67 μs | ✗ | ✓ 마진 3.22 |
| 1480 m | 9.89 μs | 13.89 μs | ✗ | ← 한계 |

**옵션 A는 650m부터 pipeline이 필수이고, 옵션 B는 1480m까지 순차 실행으로 충분하다.**

**pipeline 적용:** T_critical = T_roundtrip + T_drain(1.7μs) + T_alu(0.045μs)

| 최대 거리 | T_critical | **옵션 A** (8.33μs) | **옵션 B** (13.89μs) |
|---|---|---|---|
| 750 m | 6.75 μs | ✓ 마진 1.58 | ✓ (pipeline 불필요) |
| **987 m** | **8.33 μs** | **← 한계** | ✓ |
| 1000 m | 8.42 μs | **✗ 초과** | ✓ 마진 5.47 |

**옵션 A + pipeline 최대 거리: ~987m. 옵션 B + pipeline 최대: ~1820m (TDC 범위에 의해 ~1600m로 제한).**

**Phase 1 기본: 옵션 A (5면, 120° FOV, 2400 shots/face).** 120° 광학 화각이 시스템 요구사항. 650m 이상 시 pipeline 적용. 987m 이상은 옵션 B 전환 또는 분해능 축소 필요.

```
CSR로 N_FACES, cols_per_face를 설정 가능하게 설계하여,
옵션 A/B를 런타임 전환 가능하게 한다.
```

### 1A.5 VDMA 파라미터 (Shot-Major, 옵션 A 기준)

| 파라미터 | PACKET_SCOPE=0 (face 단위) | PACKET_SCOPE=1 (scan_frame 단위) |
|---|---|---|
| MAX_VDMA_HSIZE_BYTES | MAX_ROWS_PER_FACE × cell_size = **1024 bytes** | 동일 (stride 기준) |
| VSIZE | cols_per_face + 1 = **2401 lines** | N_FACES × (cols_per_face + 1) = **12005 lines** |
| stride | **2048 bytes** | 동일 |
| vdma_frame buffer 크기 | stride × 2401 = **~4.69 MB** | stride × 12005 = **~23.45 MB** |
| triple buffer DDR | **~14.07 MB** | **~70.34 MB** |
| SOF 주기 | 20 ms (face마다) | 100 ms (1회전마다) |

### 1A.6 Packed Row 설계 결정

**rows_per_face는 런타임 가변이다.** active_chip_mask와 stops_per_chip CSR에 의해 결정되며, packet_start 시점에 latch된다. packet_start = face_start (SCOPE=0) 또는 scan_frame_start (SCOPE=1).

```
rows_per_face = count_ones(active_chip_mask) × stops_per_chip

기본값: 4 × 8 = 32  (모든 chip 활성, 8 stops/chip)
최솟값: 1 × 2 = 2   (chip 1개, stop 2개) — header(64B) ≤ hsize_actual 보장
최댓값: N_CHIPS × MAX_STOPS_PER_CHIP = 32 (제너릭)
```

**Packed Row 정책:**

| 항목 | Packed Row (채택) |
|---|---|
| rows_per_face | active_chips × stops_per_chip **(가변)** |
| 비활성 chip | **행 자체가 없음** (대역폭 절약) |
| 비활성 stop | **행 자체가 없음** |
| HSIZE | rows_per_face × CELL_SIZE_BYTES **(가변)** |
| MAX_VDMA_HSIZE_BYTES | N_CHIPS × MAX_STOPS_PER_CHIP × CELL_SIZE_BYTES = 1024 **(고정, stride 기준)** |
| row → chip/stop 매핑 | header의 active_chip_mask, stops_per_chip으로 동적 복원 |
| DDR 낭비 | 없음 |

**Packed Row의 pixel_index 매핑:**

```
VDMA line에는 활성 row만 포함된다. 비활성 row는 존재하지 않는다.

예: active_chip_mask=0b1010 (chip1,3 활성), stops_per_chip=4

  row 0: chip 1, stop 0     ← 첫 번째 활성 chip의 stop 0
  row 1: chip 1, stop 1
  row 2: chip 1, stop 2
  row 3: chip 1, stop 3
  row 4: chip 3, stop 0     ← 두 번째 활성 chip의 stop 0
  row 5: chip 3, stop 1
  row 6: chip 3, stop 2
  row 7: chip 3, stop 3

  rows_per_face = 2 × 4 = 8 (32가 아님)
  HSIZE = 8 × 32 = 256 bytes (1024가 아님)

SW 복원:
  active_chips = [i for i in range(n_chips) if (active_chip_mask >> i) & 1]
  chip_id      = active_chips[row // stops_per_chip]
  stop_id      = row % stops_per_chip
```

**VDMA/DDR 운용:**

```
MAX_VDMA_HSIZE_BYTES   = N_CHIPS × MAX_STOPS_PER_CHIP × CELL_SIZE_BYTES = 1024 (고정)
stride      = 2048 (MAX_VDMA_HSIZE_BYTES 이상, 2KB 정렬, 고정)

S2MM_HSIZE  = rows_per_face × CELL_SIZE_BYTES  (packet_start 시 latch된 값)
            → packet_start마다 VDMA에 업데이트 (park pointer 방식 또는 scatter-gather)
S2MM_VSIZE  = cols_per_face + 1  (header 포함)
stride      = 2048 (고정, CSR 변경과 무관)

HSIZE가 가변이어도 stride는 고정이므로,
DDR 내 line 시작 주소는 항상 stride × line_number로 계산.
HSIZE < stride 구간은 padding (기록하지 않음).

[INV-8] PACKET_SCOPE=1 불변식:
  scan_frame(= 1 vdma_frame) 내의 모든 face는 동일한 rows_per_face와 hsize_actual을 사용해야 한다.
  active_chip_mask, stops_per_chip은 packet_start(= scan_frame_start) 시점에 latch되며,
  해당 scan_frame이 끝날 때까지 변경이 금지된다. (VDMA HSIZE 일관성 보장)
```

**이전 Physical Row와의 차이:**

| 항목 | Physical Row (폐기) | Packed Row (채택) |
|---|---|---|
| rows_per_face | 32 고정 | **가변** |
| HSIZE | 1024 고정 | **가변** (최대 1024) |
| zero-fill | cell_builder가 담당 | **불필요** |
| VDMA HSIZE 갱신 | 불필요 | **packet_start마다 갱신** |
| DDR 효율 | 비활성 row 낭비 | **낭비 없음** |
| SW pixel 복원 | `row // 8, row % 8` 고정 | header 기반 동적 매핑 |

### 1A.7 Phase 1 적용 범위

**본 문서의 vdma_frame path (cell → assembler → VDMA → DDR)는 SINGLE_SHOT 모드 전용이다.**

```
Phase 1 구현 범위:
  ✓ SINGLE_SHOT 측정 모드 (CONTINUOUS는 구현 범위에서 전면 배제)
  ✓ Shot-Major 매핑 (고정)
  ✓ PACKET_SCOPE = 0 (face 단위) 전용. SCOPE=1은 Phase 2
  ✓ raw_event ~ vdma_frame ~ DDR 전체 경로
  ✓ distance_conv

Phase 2 (별도 설계):
  ○ PACKET_SCOPE = 1 (scan_frame 단위 전송)
  ○ CELL_FORMAT = 1 (MPSoC 64B)
```

**[설계 결정]** CONTINUOUS 모드는 구현 범위에서 전면 배제되었다. start_num 필드는 raw_event/cell 구조에서 제거되었고, MEAS_MODE CSR도 삭제되었다.

---

## 제2장. 전체 시스템에서 데이터가 생성되는 순서

하나의 shot(= 1회 레이저 발사)을 시간축으로 추적한다. **SINGLE_SHOT 모드 기준.**

### 2.1 시간축 다이어그램

```
시간 →

t0      t1          t2              t3           t4         t5        t6        t7
│       │           │               │            │          │         │         │
▼       ▼           ▼               ▼            ▼          ▼         ▼         ▼
laser   레이저가    에코가          에코가        측정        IFIFO     cell에    AluTrigger
start   물체에     포토다이오드에    TDC-GPX     윈도우      drain     저장      펄스
pulse   도달       도달             TStop 도달   종료       시작      완료      (reset)
│                   │               │            │          │         │         │
│                   │               │            │          │         │         │
laser_ctrl          echo_receiver   TDC-GPX칩    MTimer     chip_ctrl cell_     chip_ctrl
생성                stop pulse      내부 Hit     만료       bus_phy   builder   → ARMED
                    생성            FIFO 저장    (IrFlag)   read      저장      (다음 shot)
```

### 2.2 각 시점에서 일어나는 일

#### t0: laser start pulse

laser_ctrl가 `o_tdc_start_pulse`를 1 clock pulse로 출력한다.

이 신호는 세 곳으로 간다:
1. TDC-GPX 칩의 TStart 핀 → 칩 내부에서 Start timestamp 생성
2. echo_receiver의 `i_start_tdc` → 수신 윈도우 열림
3. tdc_gpx_shot_seq_gen의 `i_start_pulse` → shot_seq 1 증가

이 시점에서 생성되는 데이터: **없음** (측정 시작 신호일 뿐)

#### t1: 레이저 → 물체

레이저 빛이 물체까지 비행한다. FPGA와 무관.

#### t2: 에코 → 포토다이오드

반사광이 포토다이오드 배열에 도달한다. echo_receiver의 LVDS 입력에 전기 신호 발생.

echo_receiver 내부에서:
- 수신 윈도우 게이팅 통과 여부 확인
- 통과하면 해당 채널의 `o_stop_pulse_r` 1 clock pulse 출력

이 시점에서 생성되는 데이터: **stop pulse** (1-bit, 채널별)

multi-hit 상황: 가까운 물체와 먼 물체가 동시에 있으면, 같은 채널에 시간차를 두고 여러 stop pulse가 발생한다. 예: 80ns, 97ns, 131ns에 각각 한 개씩.

#### t3: stop pulse → TDC-GPX TStop

echo_receiver의 stop pulse가 tdc_gpx_stop_mapper를 거쳐 TDC-GPX의 TStop1~8 핀에 도달한다.

TDC-GPX 칩 내부에서:
- TStop 에지가 감지되면 해당 채널의 **Hit FIFO**에 {Stop - Start} 값이 저장된다
- 같은 채널에 여러 stop이 오면 Hit FIFO에 순서대로 쌓인다 (FIFO이므로 시간순 보장)
- 칩 내부의 post-processing이 ChaCode, Slope를 추가한다 (Start# 필드는 SINGLE_SHOT에서 항상 0)
- post-processing 결과가 **IFIFO1 또는 IFIFO2**로 이동한다

이 이동에서 "collecting logic의 automatic bandwidth distribution"이 작용한다. 이것은 여러 채널의 Hit FIFO에서 IFIFO로 데이터를 옮길 때, 한 채널을 처리하면 인접 채널에 우선권을 주는 방식이다.

결과: **같은 채널 내부에서는 시간순 유지. 다른 채널 간에는 IFIFO 도착 순서가 물리 시간순과 다를 수 있음.**

이 시점에서 칩 내부에 생성된 데이터:

**IFIFO 한 entry = 28-bit raw word (I-Mode 기준)**

```
MSB                                                              LSB
[27] [26] [25] [24] [23] [22] [21] [20] [19] [18] [17] [16 ............. 0]
└─ChaCode─┘ └──────────── Start# ─────────────┘  Slope └──── Hit ────────┘
  (2-bit)              (8-bit)                   (1-bit)     (17-bit)
```

각 필드의 의미:

| 필드 | 비트 | 폭 | 의미 | 상세 설명 |
|---|---|---|---|---|
| **ChaCode** | [27:26] | 2-bit | 채널 코드 | 이 hit가 어떤 stop 채널에서 발생했는지를 나타내는 코드. IFIFO1(Reg8)에서 읽으면 00=TStop1, 01=TStop2, 10=TStop3, 11=TStop4. IFIFO2(Reg9)에서 읽으면 00=TStop5, 01=TStop6, 10=TStop7, 11=TStop8. 즉 ChaCode 단독으로는 8채널 중 어디인지 알 수 없고, **어떤 IFIFO에서 읽었는가**(ififo_id)와 결합해야 실제 stop_id_local을 복원할 수 있다 |
| **Start#** | [25:18] | 8-bit | (reserved) | SINGLE_SHOT 모드에서는 항상 0. 디코딩 시 무시함 |
| **Slope** | [17] | 1-bit | 에지 방향 | 이 hit를 발생시킨 stop pulse의 에지 방향. 1 = rising edge, 0 = falling edge. Reg0의 TRiseEn/TFallEn 설정에 따라 어떤 에지에 감응할지 결정됨. rising/falling 모두 활성화하면 같은 채널에서 양쪽 에지 hit가 모두 기록될 수 있음 |
| **Hit** | [16:0] | 17-bit | 시간 간격 | Stop - Start 시간 차이. BIN 단위 정수값. 물리 시간 = Hit × BIN(ps). BIN은 PLL 설정(Reg7의 HSDiv, RefClkDiv)에 의해 결정됨. 예: BIN = 82.3 ps일 때 Hit = 1000이면 82.3 ns. 내부적으로 StartOff1이 더해져 있으므로, 실제 시간 복원 시 StartOff1을 빼야 함. 측정 범위 = 2^17 × BIN ≈ 10.7 μs (BIN=82ps 기준) |

**Hit 필드 시간 복원 공식:**

```
실제 시간(ps) = (Hit - StartOff1) × BIN

여기서:
  StartOff1 = Reg5[17:0], 내부 오프셋 보상값
  Tref      = 25 ns (40 MHz reference clock 주기)
  BIN       = (Tref × 2^RefClkDiv) / (216 × HSDiv)
```

**IFIFO별 채널 매핑 표:**

| ififo_id | Reg | ChaCode=00 | ChaCode=01 | ChaCode=10 | ChaCode=11 |
|---|---|---|---|---|---|
| 0 | Reg8 | TStop1 (stop_id_local=0) | TStop2 (=1) | TStop3 (=2) | TStop4 (=3) |
| 1 | Reg9 | TStop5 (stop_id_local=4) | TStop6 (=5) | TStop7 (=6) | TStop8 (=7) |

복원 공식:
```
STOPS_PER_IFIFO = 4
  -- 의미: 하나의 IFIFO가 담당하는 stop 채널 수.
  -- 근거: ChaCode가 2-bit(0~3)이므로 IFIFO 하나당 4채널을 구분.
  --        IFIFO1 = TStop1~4 (stop_id 0~3)
  --        IFIFO2 = TStop5~8 (stop_id 4~7)
  -- 이 값은 합성 시 제너릭으로 설정된다 (TDC-GPX 칩 구조 기준 기본값).

stop_id_local = ififo_id × STOPS_PER_IFIFO + ChaCode
```

#### t4: 측정 윈도우 종료

MTimer가 만료되면 TDC-GPX가 IrFlag를 세운다. FPGA는 이를 2-FF sync 후 감지한다.

chip_ctrl FSM: CAPTURE → DRAIN_IFIFO 전이.

이 시점에서 생성되는 데이터: **없음** (상태 전이 트리거일 뿐)

#### t5: IFIFO drain (FPGA가 칩에서 데이터를 읽어오는 단계)

chip_ctrl이 bus_phy를 통해 IFIFO를 읽는다.

읽기 순서 (EF1 우선 round-robin, 데이터시트 §2.11.1 준수):

```
-- FSM invariant:
--   EF, LF 모두 HIGH active (데이터시트 §2.3)
--   ef1_empty='0' = EF pin Low = IFIFO not empty = 데이터 있음
--   (VHDL에서는 ef1_active='0' 으로 동일 의미)
--   (1) 매 루프에서 EF1과 EF2를 둘 다 확인한다.
--   (2) EF1(IFIFO1, stop 0~3)을 먼저 읽고, EF2(IFIFO2, stop 4~7)를 그 다음 읽는다.
--
-- 이유: cell 출력 순서가 stop 0 → stop 7 오름차순이므로,
--       EF1 우선 읽기로 stop 0~3 데이터가 먼저 cell_builder에 도착하게 한다.
--       cell_builder의 write 충돌 범위가 좁아지고, 데이터 흐름 방향이 일관된다.

while (ef1_empty = '0') or (ef2_empty = '0') loop

    -- ① IFIFO1 우선 (stop 0~3)
    if ef1_empty = '0' then
        read Reg8 → raw_word_1    -- IFIFO1 pop (stop 0~3)
        decode(raw_word_1, ififo_id=0) → raw_event
    end if;

    -- ② 그 다음 IFIFO2 (stop 4~7)
    if ef2_empty = '0' then
        read Reg9 → raw_word_2    -- IFIFO2 pop (stop 4~7)
        decode(raw_word_2, ififo_id=1) → raw_event
    end if;

end loop;
-- 양쪽 모두 empty → drain 완료 → AluTrigger
```

**EF1 우선의 효과:**

```
IFIFO1 = stop 0~3 (cell[0]~cell[3])  → VDMA line 앞쪽 pixel
IFIFO2 = stop 4~7 (cell[4]~cell[7])  → VDMA line 뒤쪽 pixel

EF1을 먼저 읽으면:
  - cell[0~3]이 먼저 채워짐 → chip_slice_out이 앞쪽 pixel부터 출력 준비 가능
  - cell[4~7]은 바로 뒤에 채워짐
  - 데이터 흐름 방향: stop 번호 오름차순과 일치
```

**bus transaction 관점:**

```
1 루프 iteration에서 최대 2회 bus read 발생:
  ① Reg8 read (IFIFO1): 25ns
  ② Reg9 read (IFIFO2): 25ns
  - EF는 핀 샘플링 (bus transaction 아님, ~2 clk)
  - 1 iteration 최대: ~50ns (양쪽 모두 데이터 있을 때)
  - 1 iteration 최소: ~25ns (한쪽만 데이터 있을 때)

최악: 256 entries → 128 iterations × 50ns = 6.4 μs
```

**Capped Drain (전략 B/C)에서의 EF1 우선 round-robin:**

```
drain_count = 0;
while (drain_count < N_DRAIN_CAP) and
      ((ef1_empty = '0') or (ef2_empty = '0')) loop

    -- ① IFIFO1 우선
    if ef1_empty = '0' then
        read Reg8 → decode → raw_event
        drain_count := drain_count + 1;
    end if;

    -- ② 그 다음 IFIFO2
    if (drain_count < N_DRAIN_CAP) and (ef2_empty = '0') then
        read Reg9 → decode → raw_event
        drain_count := drain_count + 1;
    end if;

end loop;
-- N_DRAIN_CAP 도달 또는 양쪽 empty → AluTrigger
```

**데이터 정렬: 명시적 정렬 불필요**

EF1 우선 round-robin으로 읽어도 IFIFO1과 IFIFO2의 hit가 iteration마다 번갈아 나온다. 그러나 cell_builder가 `stop_id_local`로 라우팅하므로, 각 cell 안에서는 시간순이 자동 보장된다.

```
읽기 순서 (뒤섞임):              cell_builder 라우팅 결과:

  Reg8 → ch1-hit0                cell[1].hit_slot[0] = ch1-hit0  ← 시간순 ✓
  Reg9 → ch5-hit0                cell[5].hit_slot[0] = ch5-hit0  ← 시간순 ✓
  Reg8 → ch2-hit0                cell[2].hit_slot[0] = ch2-hit0
  Reg9 → ch5-hit1                cell[5].hit_slot[1] = ch5-hit1  ← ch5의 두 번째
  Reg8 → ch1-hit1                cell[1].hit_slot[1] = ch1-hit1  ← ch1의 두 번째
  Reg9 → ch7-hit0                cell[7].hit_slot[0] = ch7-hit0
```

| 정렬 보장 | 근거 |
|---|---|
| 같은 채널의 hits: 시간순 ✓ | IFIFO는 FIFO 구조. 먼저 들어온 hit가 먼저 pop |
| 다른 채널 간: 정렬 불필요 | cell이 채널별로 분리되어 있으므로 순서 무관 |
| IFIFO1 vs IFIFO2 간: 정렬 불필요 | cell[0~3]과 cell[4~7]은 독립 공간 |

따라서 drain 후 별도의 정렬(sort) 단계 없이 바로 cell 출력이 가능하다.

**decode 결과 = raw_event 한 건:**

탑다운 구조: 식별(WHERE) → 시간(WHEN) → 측정값(WHAT) → 원본(RAW) → 메타(META)

```
raw_event = {

    -- ① 식별 (WHERE): 이 hit가 어디서 발생했는가
    chip_id          : 2-bit     lane 번호. 어떤 TDC-GPX 칩인가 (0~3)
    stop_id_local    : 3-bit     칩 내 stop 채널 번호 (0 ~ stops_per_chip-1)
                                  = ififo_id × STOPS_PER_IFIFO + cha_code_raw

    -- ② 시간 컨텍스트 (WHEN): 언제 발생한 hit인가
    shot_seq         : 32-bit    레이저 발사 순번. 이 hit가 속한 shot

    -- ③ 측정 데이터 (WHAT): 무엇이 측정되었는가
    raw_hit          : 17-bit    Stop - Start 시간 차이 (BIN 단위)
                                  실제 시간 = (raw_hit - StartOff1) × BIN(ps)
    slope            : 1-bit     에지 방향. 1=rising, 0=falling

    -- ④ 원본 보존 (RAW): 칩에서 읽은 원시 정보 (디버그/추적용)
    ififo_id         : 1-bit     0=IFIFO1(Reg8), 1=IFIFO2(Reg9)
    cha_code_raw     : 2-bit     칩 출력 그대로의 2-bit 채널 코드

    -- ⑤ FPGA 생성 메타 (META): FPGA가 부여한 파생 정보
    hit_seq_local    : 6-bit     같은 key 내에서의 hit 순번 (FPGA counter)
                                  key = {chip_id, shot_seq, stop_id_local}
}
```

**hit_seq_local 생성 규칙:**

key = {chip_id, shot_seq, stop_id_local}

같은 key로 들어오는 raw_event에 대해:
- 첫 번째 → hit_seq_local = 0
- 두 번째 → 1
- 세 번째 → 2
- ...

같은 채널 내에서는 Hit FIFO가 시간순을 보장하므로, hit_seq_local = 0이 가장 가까운 에코, 1이 그 다음, 2가 그 다음이다.

**중요: raw_event는 sparse하다. hit가 발생한 채널에서만 raw_event가 생성된다. hit가 없는 채널에서는 아무것도 생성되지 않는다.**

#### t6: cell에 저장

이 단계에서 sparse한 raw_event를 dense한 cell 구조로 변환한다.

cell_builder가 하는 일:

1. 현재 shot의 cell 버퍼를 0으로 초기화 (또는 lazy clear)
2. raw_event가 들어올 때마다:
   - row = stop_id_local (이 칩 내에서의 행 번호)
   - slot = hit_seq_local
   - cell[row].hit_slot[slot] = raw_hit
   - cell[row].hit_valid[slot] = 1
3. hit_seq_local >= MAX_HITS_PER_STOP이면:
   - 저장하지 않음
   - cell[row].hit_dropped = 1
4. drain 완료 후:
   - 모든 cell[0..stops_per_chip-1]을 순서대로 출력 (활성 stop만, Packed Row)
   - hit_valid가 0인 slot은 이미 0으로 초기화되어 있음 → zero-padding 완료

**cell 하나의 구조 (single source of truth):**

```
cell = {
    -- hit data (MAX_HITS_PER_STOP개)
    hit_slot[0]       : HIT_SLOT_DATA_WIDTH-bit  (첫 번째 hit의 시간/거리 값. 없으면 0)
    hit_slot[1]       : HIT_SLOT_DATA_WIDTH-bit
    ...
    hit_slot[7]       : HIT_SLOT_DATA_WIDTH-bit  (여덟 번째 hit. 없으면 0)

    -- cell-level metadata
    hit_valid         : 8-bit   (어떤 slot에 실제 hit가 있는가, bitmask)
    slope_vec         : 8-bit   (각 hit의 slope, bitmask)
    hit_count_actual  : 4-bit   (실제 hit 수, 0~8)
    hit_dropped       : 1-bit   (MAX 초과로 버려진 hit 존재 여부)
    error_fill        : 1-bit   (1 = timeout blank cell. assembler가 주입한 가짜 데이터)

HIT_SLOT_WIDTH는 플랫폼에 따라 달라진다:

| 플랫폼 | HIT_SLOT_DATA_WIDTH | raw_hit 정밀도 | 측정 범위 (BIN=82.3ps) |
|---|---|---|---|
| **Zynq-7000** | **16-bit** | 하위 16-bit (MSB 절삭) | 2^16 × 82.3ps ≈ 5.4 μs (**~800m**) |
| **Zynq MPSoC** | **17-bit** | 원본 그대로 | 2^17 × 82.3ps ≈ 10.7 μs (**~1600m**) |
}
```

MAX_HITS_PER_STOP = 8 (기본값) 기준, Zynq-7000 cell 하나의 payload 크기:

```
hit_slot:     16 × 8 = 128 bit
hit_valid:             8 bit
slope_vec:             8 bit
hit_count_actual:      4 bit
hit_dropped:           1 bit
error_fill:            1 bit     ← timeout blank cell 표지
─────────────────────────────
총 payload:          150 bit = 19 bytes → 정렬 32 bytes (256-bit)
```

**cell_size 산출 공식:**

```
cell_size는 cell_format(제너릭)에 의해 결정된다:

cell_format = 0 (Zynq-7000, Phase 1 기본):
  HIT_SLOT_BITS  = HIT_SLOT_DATA_WIDTH(16) × MAX_HITS_PER_STOP
  META_BITS      = hit_valid + slope_vec + hit_count_actual + hit_dropped + error_fill
  cell_payload   = HIT_SLOT_BITS + META_BITS
  → 150 bit → 정렬 32 bytes

cell_format = 1 (Zynq MPSoC, Phase 2):
  HIT_SLOT_PACKED = 32 × MAX_HITS_PER_STOP     ← slope/valid를 hit_slot에 inline
  META_BITS       = hit_count_actual + hit_dropped + error_fill
  cell_payload    = HIT_SLOT_PACKED + META_BITS
  → Phase 2에서 상세 설계

SW가 cell_format에 따라 parse_cell 분기:
  header 0x3F[3:0] (cell_format) → 0이면 Byte 18 bit 5 = error_fill
                             → 1이면 Byte 32 bit 5 = error_fill

cell_payload_bits = HIT_SLOT_BITS + START_NUM_BITS + META_BITS

cell_size = ceil(cell_payload_bits / 8)
          → 버스 정렬을 위해 2의 거듭제곱으로 올림 (8, 16, 32 bytes 등)
```

**MAX_HITS_PER_STOP별 cell_size 예시 (Zynq-7000, HIT_SLOT=16):**

| MAX_HITS | hit_slot | meta | payload (bit) | 정렬 후 cell_size |
|---|---|---|---|---|
| 1 | 16 | 5 | 21 | **4 bytes** |
| 2 | 32 | 6 | 38 | **8 bytes** |
| 4 | 64 | 10 | 74 | **16 bytes** |
| **8 (기본)** | **128** | **22** | **150** | **32 bytes** |

**VHDL 산출:**

```vhdl
constant CELL_SIZE_BYTES : natural :=
    align_pow2(
        (HIT_SLOT_DATA_WIDTH * MAX_HITS_PER_STOP     -- hit_slot 합계
       + MAX_HITS_PER_STOP                        -- hit_valid
       + MAX_HITS_PER_STOP                        -- slope_vec
       + 4                                         -- hit_count_actual
       + 1                                         -- hit_dropped
       + 1                                         -- error_fill
       + 7) / 8                                    -- bit→byte 올림
    );
-- 기본: (128+8+8+4+1+1)/8 = 150/8 = 18.75 → ceil=19 → align=32 bytes
```

**cell_size는 MAX_HITS_PER_STOP에 의해 결정되며, MAX_HITS_PER_STOP이 변하면 cell_size도 변한다.** 따라서 두 값은 항상 쌍으로 관리해야 하고, vdma_frame 도중 변경이 금지된다 (§8.5 Packet-Start Latching 참조).

**권장 Phase 1 기본값:** MAX_HITS_PER_STOP = 8, cell_size = 32 bytes (Zynq-7000 32-bit tdata 기준 8 beats/pixel, MPSoC 128-bit 확장 시 2 beats/pixel)

#### t7: AluTrigger 펄스 (SINGLE_SHOT 모드)

DRAIN과 cell 출력이 완료된 후, chip_ctrl이 AluTrigger 펄스를 발생시킨다.

```
chip_ctrl:
  o_tdc_alutrigger = '1'   (1 bus_tick 이상 유지)
  o_tdc_alutrigger = '0'
  → 40 ns 대기 (reset recovery)
  → ARMED 상태로 복귀
  → 다음 shot(laser start) 대기
```

이 펄스로 TDC-GPX 칩 내부가 master reset되어 Hit FIFO와 IFIFO가 비워지고, 다음 측정 cycle을 받을 준비가 된다.

#### 핵심 타이밍 제약: shot 간격 내 완료 조건

SINGLE_SHOT 모드에서, t4(IrFlag) ~ t7(ARMED 복귀)까지의 전체 과정은 **반드시 다음 laser start pulse 전에 완료**되어야 한다. 그렇지 않으면 TDC-GPX가 아직 reset 중인 상태에서 다음 start가 들어와 측정이 유실된다.

**최악의 경우 타이밍 다이어그램:**

```
shot N                                                    shot N+1
  │                                                          │
  ▼                                                          ▼
  start ──────── max_roundtrip ──────── IrFlag ── drain ── alu ── ARMED ── start
  │               (최대 비행시간)          │                          │       │
  │◄──────── T_roundtrip_max ──────────►│                          │       │
  │                                      │◄──── T_service ────────►│       │
  │                                                                 │◄─T_m─►│
  │◄──────────────────── T_shot_interval ──────────────────────────────────►│

  T_roundtrip_max : 최대 측정 거리에 의한 에코 비행시간 (= MTimer 설정값)
  T_service       : IrFlag 후 DRAIN + PAD + AluTrigger + recovery 소요 시간
  T_margin        : ARMED 복귀 ~ 다음 start 사이 여유 (≥ 0 필수)
```

**제약 조건:**

```
T_service + T_margin ≤ T_shot_interval - T_roundtrip_max

즉:
  T_service < T_shot_interval - T_roundtrip_max

여기서:
  T_shot_interval   = laser_ctrl의 연속 start pulse 간격
  T_roundtrip_max   = 2 × max_range(m) / c × 10^9 (ns)
                    = laser_ctrl의 max_roundtrip 설정값 × sys_clk_period
  T_service         = T_drain + T_cell + T_vdma + T_alu   (§11.2A.2 정의 참조)
```

**T_service 구성 요소 (SINGLE_SHOT, 전략 C, bus_clk=200MHz, BUS_TICKS=5, DIV=1):**

| 항목 | 내용 | 값 |
|---|---|---|
| T_drain | IFIFO drain (64 entries × 25ns + EF overhead) | 1.7 μs |
| T_cell | cell_builder → assembler 출력 | 1.0 μs |
| T_vdma | VDMA line 전송 (256 beats × 5ns) | 1.28 μs |
| T_alu | AluTrigger + recovery | 0.05 μs |
| **T_service 합계** | | **4.03 μs** |

T_service의 정의는 문서 전체에서 **단일 정의**이다 [원칙 1]:

```
T_service = T_drain + T_cell + T_vdma + T_alu

이 정의는 §11.2A.2에서 공식 정의되며, 여기서 참조한다.
다른 분해 방식 (T_pad, T_alutrigger, T_recovery 등)은 사용하지 않는다.
```

**구체 예시:**

```
조건:
  미러 회전 = 10 rps (600 rpm), 5면
  face당 2400 shots
  → T_shot_interval = 20 ms / 2400 = 8.33 μs

  BUS_CLK_DIV = 1, BUS_TICKS = 5 (bus_rate = 40 MHz)
  전략 C (Source Gating, MAX_HITS = 8)
  → T_service = 4.03 μs

근거리 (150m):
  T_roundtrip = 1.0 μs
  T_total = 1.0 + 4.03 = 5.03 μs
  T_margin = 8.33 - 5.03 = 3.30 μs  ✓ 충분

순차 한계 (~650m):
  T_roundtrip = 4.33 μs
  T_total = 4.33 + 4.03 = 8.36 μs ≈ 8.33 μs  ← 한계

장거리 (1500m):
  T_roundtrip = 10.0 μs
  T_total = 10.0 + 4.03 = 14.03 μs > 8.33 μs  ✗ pipeline으로도 불가
```

**설계 원칙:**

이 타이밍 제약은 시스템 설정에 따라 동적으로 변하므로, 다음 규칙을 적용한다:

1. **CSR에서 확인 가능하게 할 것**: T_shot_interval, T_roundtrip_max, BUS_CLK_DIV로부터 SW가 T_margin을 계산하여 운용 가능 여부를 판단할 수 있어야 한다

2. **위반 감지 메커니즘**: chip_ctrl이 ARMED 복귀 전에 다음 start pulse가 감지되면 `shot_overrun` 에러 플래그를 세운다

3. **Pipeline overlap 옵션**: cell_builder에 double-buffer를 두어, shot N의 cell 출력(VDMA 전송)과 shot N+1의 측정을 overlap할 수 있다. 단, DRAIN + AluTrigger 자체는 overlap 불가 (칩이 reset 되어야 다음 측정 가능)

---

## 제3장. Cell에서 Frame으로

### 3.1 Frame의 물리적 의미

```
                    수평 방향 (시간순 = face 내 순차 스캔)
                    col 0    col 1    col 2    ...    col N-1
                    shot 0  shot 1  shot 2  ...    shot N-1
               ┌────────────────────────────────────────────┐
  row 0 (ch0)  │ cell     cell     cell     ...    cell     │
  row 1 (ch1)  │ cell     cell     cell     ...    cell     │
  row 2 (ch2)  │ cell     cell     cell     ...    cell     │
  ...          │ ...                                        │
  row rows_per_face-1      │ cell     cell     cell     ...    cell     │
               └────────────────────────────────────────────┘
```

- 수평 방향 = 시간순. col 0이 face 시작, col N-1이 face 끝.
- 수직 방향 = 채널 배열순. row 0 = chip0의 stop0, row 7 = chip0의 stop7, row 8 = chip1의 stop0, ...
- 총 행 수 rows_per_face = count_ones(active_chip_mask) × stops_per_chip (가변, 기본 32)
  - 의미: 1 shot당 동시에 에코를 수신하는 수직 채널 수 (= 포토다이오드 수직 배열의 활성 검출기 수)
- 총 열 수 cols_per_face = face 내 수평 샘플 수
- 각 cell은 MAX_HITS_PER_STOP개의 hit slot을 가짐

### 3.2 4개 칩의 수직 채널 배치

```
Packed Row 배치: 활성 chip/stop만 포함, 비활성 row 없음.

예 1: 모든 chip 활성 (active_chip_mask=0b1111), stops_per_chip=8
  row  0 = chip0, stop 0
  row  1 = chip0, stop 1
  ...
  row  7 = chip0, stop 7
  row  8 = chip1, stop 0
  ...
  row 31 = chip3, stop 7
  rows_per_face = 4 × 8 = 32, HSIZE = 32 × 32 = 1024 bytes

예 2: chip 0,2만 활성 (active_chip_mask=0b0101), stops_per_chip=4
  row  0 = chip0, stop 0
  row  1 = chip0, stop 1
  row  2 = chip0, stop 2
  row  3 = chip0, stop 3
  row  4 = chip2, stop 0
  row  5 = chip2, stop 1
  row  6 = chip2, stop 2
  row  7 = chip2, stop 3
  rows_per_face = 2 × 4 = 8, HSIZE = 8 × 32 = 256 bytes

SW 복원 (header 기반):
  active_chips = [i for i in range(n_chips) if (active_chip_mask >> i) & 1]
  chip_id      = active_chips[row // stops_per_chip]
  stop_id      = row % stops_per_chip
```

총 행 수 = count_ones(active_chip_mask) × stops_per_chip (가변, packet_start 시 latch).

### 3.3 Frame 크기 계산 예시

가정 (구체 예시):
- active_chips = 4, stops_per_chip = 8
- cols_per_face = 2400 (옵션 A: 120° / 0.05°, 옵션 B: 1800)
- MAX_HITS_PER_STOP = 8
- cell_size = 32 bytes (256-bit 정렬)

```
rows_per_face = count_ones(active_chip_mask) × stops_per_chip = 4 × 8 = 32 (기본값, 모든 chip 활성)
cols_per_face = 2400 (옵션 A 기본)
cell_size = 32 bytes

packet_body = rows_per_face × cols_per_face × cell_size (기본: 32 × 2400 × 32 = 2,457,600 bytes ≈ 2.4 MB)
header_line = rows_per_face × cell_size (기본: 32 × 32 = 1024 bytes)
total_packet = 2,457,600 + 1,024 = 2,458,624 bytes ≈ 2.35 MB
```

---

## 제3A장. chip_ctrl 측정 모드: SINGLE_SHOT

### 3A.1 개요

TDC-GPX는 데이터시트 §2.11.1 "Single Measurement" 흐름을 사용한다. 본 설계는 SINGLE_SHOT 모드 전용이다.

### 3A.2 물리 핀 추가

기존 설계에 다음 물리 핀을 추가한다 (lane당):

| 핀 | 방향 | 기능 |
|---|---|---|
| `o_tdc_stopdis(3:0)` | FPGA → TDC-GPX | StopDis1~4. 각 bit가 stop 쌍을 제어. 1=차단, 0=허용 |
| `o_tdc_alutrigger` | FPGA → TDC-GPX | AluTrigger. HIGH 레벨 펄스 (≥10ns) → master reset (MasterAluTrig=1) |

### 3A.3 SINGLE_SHOT 모드 (§2.11.1)

데이터시트 §2.11.1 "Single measurement" 흐름 그대로.

**cfg_image 핵심 설정:**
- StartTimer = 0 (내부 retrigger 없음)
- MasterAluTrig = 1 (AluTrigger 핀으로 reset 가능)
- MTimerStart = 1 (Start에 의해 MTimer 시작)
- MTimer 값 설정 (측정 윈도우 결정)
- IrFlag ← MTimer 만료 (Reg12)
- StopDisStart = 1, StartDisStart = 1

**FSM 흐름:**

```
RESET_POWERUP
  │ PuResN Low → High
  ▼
CFG_STOPDIS_HIGH
  │ o_tdc_stopdis = "1111" (모든 stop 입력 차단)
  ▼
CFG_WRITE_SEQ
  │ Reg0 ~ Reg7, Reg11, Reg12, Reg14 순차 write
  ▼
MASTER_RESET
  │ Reg4 bit22 = '1' write (master reset)
  ▼
WAIT_RESET_RECOVERY
  │ 40 ns 대기
  ▼
CFG_STOPDIS_LOW
  │ o_tdc_stopdis = "0000" (stop 입력 허용)
  ▼
ARMED
  │ 외부 start pulse 대기
  │ (laser_ctrl가 start를 생성하면 칩 내부에서 측정 시작)
  ▼
CAPTURE
  │ MTimer 카운트 중, hit 수집 중
  │ IrFlag 감시
  ▼ (IrFlag rising edge 감지)
DRAIN_IFIFO
  │ round-robin: EF1/EF2 동시 확인 → Reg8/Reg9 read → decode → cell 저장
  │ 양쪽 IFIFO empty까지 반복
  ▼
PAD_CHECK
  │ zero-padding 생성 (필요 시)
  ▼
ALUTRIGGER_PULSE
  │ o_tdc_alutrigger = '1' → '0' (1 bus_tick 이상)
  │ → 칩 내부 master reset 수행
  ▼
WAIT_NEXT_SHOT
  │ reset recovery 대기 (40 ns)
  │ → ARMED로 복귀 (다음 shot 대기)
```

**핵심 특징:**
- 매 shot마다 명시적인 cycle: ARMED → CAPTURE → DRAIN → ALUTRIGGER → ARMED
- IrFlag(MTimer 만료)가 DRAIN 시작의 트리거
- AluTrigger 펄스로 칩을 reset하여 다음 shot 준비
- shot 간 경계가 명확함

### 3A.4 CSR 제어

| CSR Offset | 이름 | 설명 |
|---|---|---|
| 0x30 | `STOPDIS_OVERRIDE` | [3:0] SW 직접 StopDis 제어 (디버그용). [4] override enable |
| 0x34 | `SLICE_TIMEOUT_CLKS` | [7:0] lane timeout 설정 (sys_clk cycles, 기본 200) |

### 3A.5 chip_ctrl FSM

SINGLE_SHOT 전용 FSM. 매 shot마다 ARMED→CAPTURE→DRAIN→ALUTRIGGER→ARMED 순환.

```
                    RESET_POWERUP
                         │
                    CFG_STOPDIS_HIGH
                         │
                    CFG_WRITE_SEQ
                         │
                    MASTER_RESET
                         │
                    WAIT_RESET_RECOVERY
                         │
                    CFG_STOPDIS_LOW
                         │
                      ARMED
                         │
                      CAPTURE
                         │
                    (IrFlag)
                         │
                    DRAIN_IFIFO
                         │
                    PAD_CHECK
                         │
                    ALUTRIGGER_PULSE
                         │
                    WAIT_NEXT_SHOT
                         │
                      ARMED (반복)
```

---

## 제4장. vdma_frame 데이터가 생성되는 시간 순서

하나의 vdma_frame(= 1 face 스캔, PACKET_SCOPE=0 기준) 동안 데이터가 어떤 순서로 만들어지는지 추적한다.

### 4.1 Face 시작

```
face_start 신호 발생 (laser_ctrl의 axis_in에서 face 경계 감지)
  → vdma_frame_id 증가
  → col_in_face = 0
  → vdma_frame header 준비
```

### 4.2 Shot 0 (첫 번째 수평 샘플)

```
시간 순서:

1. laser_ctrl가 start pulse 발사
   → shot_seq 증가
   → col_in_face = 0

2. echo_receiver가 각 채널별 stop pulse 생성
   → 4개 칩의 TStop에 도달

3. 4개 칩 내부에서 hit 수집 (병렬, 칩마다 독립)

4. MTimer 만료 → 4개 lane 동시에 DRAIN 시작

5. lane 0: bus_phy로 IFIFO1/2 drain → decode → raw_event 생성 → cell_builder에 저장
   lane 1: 동시에 같은 과정
   lane 2: 동시에 같은 과정
   lane 3: 동시에 같은 과정

6. 각 lane의 drain 완료 → cell_builder가 stops_per_chip개 cell 확정

7. 활성 lane의 cell만 모아서 한 줄(line)의 rows_per_face개 pixel 형성 (Packed Row)

   pixel 순서 (line 내부, rows_per_face개, 활성 chip/stop만):
   [active_chip_0-stop0] ... [active_chip_0-stop(S-1)]
   [active_chip_1-stop0] ... [active_chip_1-stop(S-1)]
   ...
   [active_chip_(N-1)-stop0] ... [active_chip_(N-1)-stop(S-1)]
   (N = count_ones(active_chip_mask), S = stops_per_chip)
   비활성 chip/stop의 row는 존재하지 않음 (Packed Row)
```

### 4.3 Shot 1 ~ Shot N-1 (이후 수평 샘플)

위와 동일한 과정이 col_in_face를 1씩 증가시키며 반복된다.

### 4.4 Face 종료

```
col_in_face == cols_per_face - 1
→ 이 line이 vdma_frame의 마지막 line
→ 다음 face_start까지 대기
```

### 4.5 핵심 관찰

**데이터 생성 순서 = column 순서 (시간순)**

shot 0의 모든 row가 먼저 완성되고, 그 다음 shot 1의 모든 row가 완성된다. 한 shot 안에서는 4개 lane이 병렬로 drain하므로, chip0~3의 cell은 거의 동시에 완성된다.

이것은 **column-major 생성 순서**이다:
- col 0의 row 0 ~ rows_per_face-1 → col 1의 row 0 ~ rows_per_face-1 → ... → col N-1의 row 0 ~ rows_per_face-1

---

## 제5장. VDMA가 요구하는 데이터 형식

### 5.1 AXI4-Stream Video Protocol 규약

#### 5.1.1 VDMA Video Timing 구조 (참조 다이어그램)

```
        0                    Hblank Start              HSIZE
        (SAV)                (H EAV)    Hsync    Hsync
                                        Start     End
  0 ┌──────────────────────┬─────────────────────┐
    │                      │                     │  V
    │                      │    Horizontal       │  
    │    Active Video      │    Blanking         │  B
    │                      │                     │  l
    │   (유효 데이터 영역)   │   (데이터 없음)      │  a
    │                      │                     │  n
    │                      │                     │  k
Vblank├──────────────────────┤                     │
Start │                      │                     │
(VEAV)│  Vertical Blanking   │                     │
Vsync ├ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─┤                     │
Start │  (프레임 간 빈 구간)  │                     │
Vsync ├ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─┤                     │
End   │                      │                     │
VSIZE └──────────────────────┴─────────────────────┘
```

이 다이어그램은 **일반적인 비디오 프레임 타이밍**이다. 본 설계에서 이 구조가 어떻게 매핑되는지 아래에서 설명한다.

#### 5.1.2 S2MM 방향에서의 동기 신호

VDMA의 S2MM(Stream → DDR) 채널은 전통적인 HSync/VSync 신호를 사용하지 않는다. 대신 AXI4-Stream의 sideband 신호로 프레이밍을 수행한다:

```
tdata   : pixel payload (cell 데이터)
tvalid  : 유효 데이터 (1일 때만 VDMA가 기록)
tready  : VDMA가 받을 수 있음 (backpressure)
tuser(0): Start Of Frame (SOF) = VSync 역할
tlast   : End Of Line (EOL) = HSync 역할
```

| 비디오 개념 | AXI4-Stream 대응 | 생성 방법 |
|---|---|---|
| **HSync** | `tlast` = 1 | 매 line(= segment)의 **마지막 beat**에서 assert |
| **VSync** | `tuser(0)` = 1 | vdma_frame의 **첫 번째 beat**에서만 assert |
| **H Blanking** | `tvalid` = 0 구간 | shot 간 측정/drain 시간 동안 자연 발생 |
| **V Blanking** | `tvalid` = 0 구간 | face 전환 시 자연 발생 |
| **Active Video** | `tvalid` = 1 구간 | cell 데이터 전송 중 |

#### 5.1.3 HSync(tlast) 생성 방법

한 line의 시간 순서는 다음과 같다:

```
시간 →

│◄──── ① Active Video ────►│②│◄───── ③ H Blanking ──────►│
│  shot 데이터로 line 채움   │H │  다음 shot 측정/drain 대기  │
│  tvalid=1                 │S │  tvalid=0                  │
│                           │y │                            │
│                           │n │                            │
│                           │c │                            │
```

**① Active Video (tvalid=1):**
shot에서 수집된 cell 데이터를 pixel 단위로 순서대로 전송한다.
총 beats_per_line = rows_per_face × beats_per_pixel 개의 beat를 전송한다.

```
  beat 0:   cell[row=0] byte 0~3     tvalid=1, tlast=0
  beat 1:   cell[row=0] byte 4~7     tvalid=1, tlast=0
  ...
  beat 254: cell[row=31] byte 24~27  tvalid=1, tlast=0
  beat 255: cell[row=31] byte 28~31  tvalid=1, tlast=1  ← ② HSync (기본값 예시: rows=32, cell=32B, tdata=32bit)
```

**② HSync (tlast=1):**
Active Video 영역의 **모든 pixel을 다 채운 후**, 마지막 beat에서 `tlast=1`을 assert한다. 이것이 VDMA에게 "이 line이 끝났다"를 알린다. VDMA는 tlast를 받으면 DDR write 주소를 다음 stride 위치로 이동한다.

**tlast는 마지막 데이터와 동시에 발생한다** (별도의 빈 beat가 아님):
```
마지막 beat:  tdata = 유효 데이터, tvalid = 1, tlast = 1
              ↑ 이 beat에서 데이터 기록과 line 종료가 동시에 일어남
```

**③ H Blanking (tvalid=0):**
tlast 이후, 다음 shot의 cell 데이터가 준비될 때까지 tvalid=0 상태가 유지된다. 이 구간이 자연스럽게 H Blanking이 된다.

```
H Blanking 동안 일어나는 일:
  - 다음 레이저 발사 (start pulse)
  - 에코 비행 + 수신 (echo_receiver)
  - MTimer 대기 (CAPTURE)
  - IFIFO drain (bus_phy read)
  - cell_builder 데이터 정리
  - AluTrigger + recovery

이 모든 과정이 끝나면 다음 line의 tvalid=1 시작 → Active Video 재개
```

**VDMA S2MM은 tvalid=0이면 그냥 대기한다.** 따라서 H Blanking 기간의 정확한 길이를 맞출 필요 없이, 데이터가 준비되면 바로 다음 line을 전송하면 된다.

**한 line의 전체 시간 구성:**

```
T_line_total = T_active_video + T_h_blanking

T_active_video = beats_per_line × clk_period
               = 256 × 5ns = 1.28 μs  (32-bit tdata @ 200MHz)

T_h_blanking   = T_measurement + T_drain + T_cell_prep + T_alu
               = 측정/처리 시간 (shot 간격에 의존)

T_line_total   ≈ T_shot_interval = 8.33 μs (2400 shots/face 기준)
```

#### 5.1.4 VSync(tuser(0)) 생성 방법

한 vdma_frame의 시간 순서는 다음과 같다:

```
시간 →

│①│◄─ line0 ─►│blank│◄─ line1 ─►│blank│◄─ line2 ─►│...│◄─ lineN ─►│◄── V Blank ──►│
│V │  header   │     │  shot 0   │     │  shot 1   │   │  shot N-1 │  face 전환     │
│S │           │     │           │     │           │   │           │               │
```

**① VSync (tuser(0)=1):**
vdma_frame의 첫 번째 line(header line)의 첫 번째 beat에서 `tuser(0)=1`을 assert한다. VDMA는 이 신호를 받으면 vdma_frame buffer 시작 주소로 점프한다.

```
vdma_frame 시작:
  beat 0: tdata=header byte 0~3, tvalid=1, tuser(0)=1, tlast=0  ← VSync
  beat 1: tdata=header byte 4~7, tvalid=1, tuser(0)=0, tlast=0
  ...
  beat 255: tdata=header last,   tvalid=1, tuser(0)=0, tlast=1  ← header line HSync (기본값 예시)
  (H Blanking)
  beat 0: tdata=shot0 cell0,     tvalid=1, tuser(0)=0, tlast=0  ← data line 시작
  ...
```

**tuser(0)은 vdma_frame 전체에서 딱 1 beat만 1이다.** 이후 모든 beat에서 0.

**V Blanking = face 전환 구간.** 현재 face의 마지막 shot line 전송(tlast) 완료 후, 다음 face의 첫 shot이 준비될 때까지 tvalid=0인 시간. FPGA가 별도로 생성하지 않고 자연 발생한다.

#### 5.1.5 비디오 타이밍과 LiDAR 데이터의 매핑 요약

**Shot-Major 매핑:**

```
┌──────────────────────────────────────────────────────────────────┐
│ 비디오 용어        │ LiDAR 대응                │ 비고            │
├──────────────────────────────────────────────────────────────────┤
│ 1 Frame(VDMA)     │ 1 vdma_frame (PACKET_SCOPE=0) │ ~20 ms         │
│ 1 Line            │ 1 segment (1 shot)        │ rows_per_face cells         │
│ 1 Pixel           │ 1 cell (1 stop channel)   │ cell_size bytes │
│ Active Video      │ cell 데이터 전송 중       │ tvalid=1        │
│ H Blanking        │ shot 간 측정/drain 시간   │ tvalid=0        │
│ V Blanking        │ face 전환 시간            │ tvalid=0        │
│ HSync (tlast)     │ 이 shot의 채널 slice 끝   │ ≠ 수직 전체 끝  │
│ VSync (tuser(0))  │ vdma_frame 시작              │ 1 beat          │
│ HSIZE             │ rows_per_face × cell_size (가변)     │ 기본 1024 bytes  │
│ VSIZE             │ cols_per_face + 1        │ 2401 lines       │
│ 수직 전체 완성     │ N_FACES개 vdma_frame 합산    │ = 1 scan_frame       │
└──────────────────────────────────────────────────────────────────┘
```

PACKET_SCOPE 선택 상세는 §5.4 참조.

### 5.2 VDMA가 DDR에 쓰는 규칙

**용어 정의 (VDMA 관점):**

| 용어 | 정의 | 산출 |
|---|---|---|
| **1 line** | 1 segment = 1 shot에서 전체 활성 칩의 전체 활성 stop 채널을 합친 데이터 | = 1 VDMA line |
| **1 pixel** | 1개 stop 채널의 cell 데이터 (multi-hit slot 집합 포함) | = 1 cell = cell_size bytes |
| **pixels_per_line** | 한 line에 포함된 pixel 수 | = rows_per_face (가변, 기본 32) |
| **bytes_per_pixel** | pixel 하나의 바이트 크기 | = cell_size (예: 32 bytes) |
| **tdata_width** | AXI4-Stream 데이터 버스 폭 (bits) | 플랫폼에 의존 |
| **tdata_bytes** | tdata_width / 8 | 1 beat에 전송되는 바이트 수 |
| **beats_per_pixel** | 1 pixel을 전송하는 데 필요한 AXI beat 수 | = cell_size / tdata_bytes |
| **beats_per_line** | 1 line의 총 AXI beat 수 | = pixels_per_line × beats_per_pixel |

#### 5.2.1 hsize 산출

VDMA의 `S2MM_HSIZE`는 **바이트 단위**이다. tdata 폭과 무관하게 한 line의 총 바이트 수로 설정한다.

```
hsize = pixels_per_line × cell_size
      = rows_per_face × cell_size   [bytes]
```

**중요: hsize에는 multi-hit(에코) 용량이 이미 포함되어 있다.** cell_size 자체가 MAX_HITS_PER_STOP개의 hit slot을 내장하고 있기 때문이다. 따라서 에코 수를 hsize에 별도로 곱하지 않는다.

```
관계 정리:

  MAX_HITS_PER_STOP  →  cell_size 결정  →  hsize 결정

  에코 수 증가 = MAX_HITS_PER_STOP 증가 = cell_size 증가 = hsize 증가
```

| MAX_HITS_PER_STOP | cell_size | hsize (rows_per_face=32 기준) | 의미 |
|---|---|---|---|
| 1 | 4 bytes | 128 bytes | 에코 없음, 채널당 1 hit만 |
| 2 | 8 bytes | 256 bytes | 채널당 최대 2 에코 |
| 4 | 16 bytes | 512 bytes | 채널당 최대 4 에코 |
| **8 (기본)** | **32 bytes** | **1024 bytes** | **채널당 최대 8 에코** |

그러나 VDMA는 내부적으로 tdata 폭 단위로 데이터를 처리하므로, **hsize는 tdata_bytes의 정수배**여야 한다.

```
제약: hsize % tdata_bytes == 0

검증: hsize = 32 × 32 = 1024 bytes
      tdata_bytes = 4 (32-bit) → 1024 / 4 = 256 beats ✓ (기본값 예시, actual은 hsize_actual / tdata_bytes)
      tdata_bytes = 8 (64-bit) → 1024 / 8 = 128 beats ✓
      tdata_bytes = 16 (128-bit) → 1024 / 16 = 64 beats ✓
```

#### 5.2.2 AXI beat와 pixel의 관계

VDMA 관점에서 "pixel"이라는 용어는 원래 영상 프레임의 1 pixel = tdata 1 beat를 기대한다. 그러나 본 설계에서 **우리의 1 pixel(= 1 cell)은 tdata 여러 beat에 걸친다.**

```
예: cell_size = 32 bytes, tdata_width = 32-bit

  우리의 1 pixel = 8 AXI beats

  AXI-Stream 전송:
    beat 0:  cell[row0] byte 0~3
    beat 1: cell[row0] byte 4~7
    beat 2: cell[row0] byte 8~11
    beat 7:  cell[row0] byte 28~31   ← 여기서 1 pixel 완료
    beat 8:  cell[row1] byte 0~3
    ...
    beat 255: cell[row31] byte 28~31  ← tlast = 1 (EOL)
```

**tlast 위치**: line의 마지막 beat(= 마지막 pixel의 마지막 beat)에서 tlast=1.

#### 5.2.3 Zynq 플랫폼별 tdata 폭

| 플랫폼 | HP 포트 최대 폭 | 권장 tdata_width | beats_per_pixel (cell=32B) | beats_per_line (rows_per_face=32) |
|---|---|---|---|---|
| **Zynq-7000** | 64-bit | **32-bit** (안전 기본값) | 8 | 256 |
| Zynq-7000 | 64-bit | 64-bit (HP 포트 활용) | 4 | 128 |
| **Zynq MPSoC** | 128-bit | 64-bit 또는 128-bit | 4 또는 2 | 128 또는 64 |

**설계 기준: Zynq-7000, tdata_width = 32-bit.**

이 기준으로 설계하면:
- Zynq-7000에서 32-bit HP 포트로 바로 동작
- Zynq-7000에서 64-bit HP 포트로 전환 시 beats_per_pixel만 변경
- Zynq MPSoC로 확장 시 tdata_width generic만 변경

#### 5.2.4 tdata 폭을 generic으로 설계

```vhdl
entity tdc_gpx_axis_video_out is
    generic (
        -- ===== 독립 변수 (합성 시 설정) =====
        N_CHIPS            : natural := 4;     -- 병렬 TDC-GPX 칩 수
        MAX_STOPS_PER_CHIP : natural := 8;     -- 칩당 최대 stop 채널 수 (TDC-GPX I-Mode 기준)
        MAX_HITS_PER_STOP  : natural := 8;     -- cell 내 hit_slot 개수 (채널당 저장 에코 수)

        HIT_SLOT_DATA_WIDTH     : natural := 16;    -- hit_slot 하나의 비트 폭
                                                -- = IFIFO raw_hit를 저장하는 비트 수
                                                -- TDC-GPX Hit 필드 원본 = 17-bit
                                                -- Zynq-7000: 16 (MSB 절삭, 측정 ~800m)
                                                -- Zynq MPSoC: 17 (원본 보존, 측정 ~1600m)

        TDATA_WIDTH        : natural := 32;    -- AXI4-Stream 데이터 버스 폭 (bits)
                                                -- Zynq-7000:  32 (GP 포트) 또는 64 (HP 포트)
                                                -- Zynq MPSoC: 64 또는 128 (HPC 포트)

        CELL_FORMAT        : natural := 0      -- cell 바이트 레이아웃 선택
                                                -- 0 = Zynq-7000 (16-bit separate, cell_size=32B)
                                                -- 1 = MPSoC (32-bit packed, cell_size=64B) [Phase 2]
    );
    port (
        m_axis_tdata     : out std_logic_vector(TDATA_WIDTH-1 downto 0);
        m_axis_tvalid    : out std_logic;
        m_axis_tready    : in  std_logic;
        m_axis_tlast     : out std_logic;     -- EOL: line 마지막 beat
        m_axis_tuser     : out std_logic_vector(0 downto 0);  -- SOF
        ...
    );
end entity;

-- ===== 종속 변수 (제너릭으로부터 자동 산출) =====

-- cell 내부 비트 구성 (CELL_FORMAT에 따라 분기)
function calc_cell_payload_bits return natural is
begin
    if CELL_FORMAT = 0 then
        -- Format 0 (Zynq-7000): 16-bit hit_slot separate
        return HIT_SLOT_DATA_WIDTH * MAX_HITS_PER_STOP     -- hit_slot
             + MAX_HITS_PER_STOP                        -- hit_valid
             + MAX_HITS_PER_STOP                        -- slope_vec
             + 4 + 1 + 1;                              -- hit_count_actual + hit_dropped + error_fill
        -- 예: 16*8 + 8 + 8 + 6 = 150bit → 19B → align=32B
    else
        -- Format 1 (MPSoC): 32-bit packed hit_slot (raw_hit+slope+valid inline)
        return 32 * MAX_HITS_PER_STOP                  -- packed hit_slot
             + 4 + 1 + 1;                              -- hit_count_actual + hit_dropped + error_fill
        -- 예: 32*8 + 6 = 262 → +2 reserved = 264bit → 64B
    end if;
end function;

constant CELL_PAYLOAD_BITS : natural := calc_cell_payload_bits;

-- cell 바이트 크기 (2의 거듭제곱으로 올림 정렬)
function ceil_pow2(x : natural) return natural is
    variable result : natural := 1;
begin
    while result < x loop result := result * 2; end loop;
    return result;
end function;

constant CELL_SIZE_BYTES  : natural := ceil_pow2((CELL_PAYLOAD_BITS + 7) / 8);
    -- CELL_FORMAT=0: 216bit → 27bytes → 정렬 32 bytes
    -- CELL_FORMAT=1: 264bit → 33bytes → 정렬 64 bytes

-- line/frame 구조 (제너릭 종속 상수)
constant MAX_ROWS_PER_FACE : natural := N_CHIPS * MAX_STOPS_PER_CHIP;       -- 32 (stride/buffer 기준)
constant TDATA_BYTES       : natural := TDATA_WIDTH / 8;                    -- 기본 4
constant BITS_PER_PIXEL   : natural := CELL_SIZE_BYTES / TDATA_BYTES;      -- 기본 8
constant MAX_BITS_PER_LINE: natural := MAX_ROWS_PER_FACE * BITS_PER_PIXEL; -- 256 (stride 기준)
constant MAX_VDMA_HSIZE_BYTES         : natural := MAX_ROWS_PER_FACE * CELL_SIZE_BYTES; -- 1024 (stride 기준)

-- 런타임 산출 (packet_start latch)
signal rows_per_face     : natural range 2 to MAX_ROWS_PER_FACE;  -- actual
signal hsize_actual      : natural range 2*CELL_SIZE_BYTES to MAX_VDMA_HSIZE_BYTES;
signal beats_per_line    : natural;  -- rows_per_face * BITS_PER_PIXEL
```

**제너릭 의존성 그래프:**

```
독립 변수 (generic)              종속 변수 (constant)
─────────────────               ─────────────────────
N_CHIPS ──────────────┐
                      ├──→ MAX_ROWS_PER_FACE ─────────┐
MAX_STOPS_PER_CHIP ───┘                                │
                                                       ├──→ MAX_VDMA_HSIZE_BYTES (고정)
MAX_HITS_PER_STOP ────┐                                │     MAX_BITS_PER_LINE
                      ├──→ CELL_SIZE_BYTES ────────────┘
HIT_SLOT_DATA_WIDTH ───────┤        ↑                       │
CELL_FORMAT ──────────┘        │                       │
                    (2의 거듭제곱 정렬)                 │
                                                       │
TDATA_WIDTH ──────────────→ TDATA_BYTES ──→ BITS_PER_PIXEL
                                                       ↑
                                            CELL_SIZE_BYTES ─┘

  런타임 산출 (packet_start latch):
  active_chip_mask + stops_per_chip → rows_per_face → hsize_actual, beats_per_line
```

**플랫폼별 제너릭 → 종속 변수 산출 예시:**

| 독립 변수 | Zynq-7000 (기본) | Zynq MPSoC |
|---|---|---|
| N_CHIPS | 4 | 4 |
| MAX_STOPS_PER_CHIP | 8 | 8 |
| MAX_HITS_PER_STOP | 8 | 8 |
| HIT_SLOT_DATA_WIDTH | 16 | 17 |
| TDATA_WIDTH | 32 | 64 |
| **→ CELL_SIZE_BYTES** | **32** | **64** |
| **→ MAX_ROWS_PER_FACE** | **32** | **32** |
| **→ MAX_VDMA_HSIZE_BYTES** | **1024** | **2048** |
| **→ BITS_PER_PIXEL** | **8** | **8** |
| **→ BEATS_PER_LINE** | **256** | **256** |

#### 5.2.5 DDR 기록 규칙

```
packet_buffer_base_addr + (line_number × stride)

여기서:
  hsize  = pixels_per_line × cell_size                    [bytes]
         = rows_per_face × cell_size
  vsize  = cols_per_face + 1                              [lines, header 포함]
  stride = 한 line이 DDR에서 차지하는 메모리 간격            [bytes]

  hsize는 tdata_bytes의 정수배여야 함 (위 검증 참조)
```

**stride 상세 설명:**

stride는 DDR에서 한 line의 시작 주소와 다음 line의 시작 주소 사이의 바이트 간격이다.

```
DDR 메모리 레이아웃:

base + stride × 0:  [--- line 0 (hsize bytes) ---][-- padding --]
base + stride × 1:  [--- line 1 (hsize bytes) ---][-- padding --]
base + stride × 2:  [--- line 2 (hsize bytes) ---][-- padding --]
...

stride ≥ hsize (필수)
stride - hsize = padding (미사용 영역)
```

stride를 hsize보다 크게 잡는 이유:

| 이유 | 설명 |
|---|---|
| **AXI burst alignment** | AXI4 burst는 4KB 경계를 넘을 수 없다. stride를 4KB 배수로 맞추면 burst가 깔끔하게 정렬됨 |
| **캐시 라인 정렬** | Zynq PS의 L2 캐시 라인 = 32 bytes. stride가 캐시 라인 배수이면 SW 읽기 효율 향상 |
| **DMA 효율** | 일부 DMA 엔진은 2의 거듭제곱 stride에서 최적 성능 |
| **SW 편의** | power-of-2 stride는 주소 계산이 shift 연산으로 가능 |

**stride 설정 예시 (hsize = 1024 bytes 기준):**

| stride | padding | 메모리 효율 | 비고 |
|---|---|---|---|
| 1024 | 0 | 100% | 최소값. 정렬 보장 안 됨 |
| **2048** | 1024 | 50% | **2KB 정렬. 권장** |
| 4096 | 3072 | 25% | 4KB 정렬. AXI burst 최적 |

**Phase 1 권장: stride = 2048 bytes (2KB 정렬).**

```
vdma_frame 하나의 DDR 사용량 = stride × vsize
                        = 2048 × 2401 = 4,917,248 bytes ≈ 4.69 MB

triple buffer = 3 × 4.69 MB ≈ 14.07 MB
```

**VDMA 레지스터 설정:**

```
S2MM_HSIZE       = hsize  = rows_per_face × cell_size  [bytes, 가변, 기본 1024]
S2MM_FRMDLY_STRIDE = stride = 2048      [bytes, line 간 메모리 간격]
S2MM_VSIZE       = vsize  = cols_per_face + 1  [lines]
```

VDMA는 SOF를 받으면 vdma_frame buffer 시작 주소로 점프한다. 매 beat마다 tdata를 DDR에 기록한다. tlast(EOL)를 만나면 **현재 line 시작 + stride** 위치로 점프하여 다음 line을 기록한다. hsize와 stride 사이의 padding 영역에는 아무것도 쓰지 않는다.

**stride와 전송 시간**: stride가 hsize보다 크더라도 VDMA 전송 시간은 늘어나지 않는다. VDMA는 hsize 바이트만 실제로 기록하고, padding 구간은 쓰지 않고 주소만 점프한다. 전송 시간은 오직 hsize(= beats_per_line × beat 시간)에만 비례한다.

#### 5.2.6 TDC-GPX Bus 속도와 BUS_CLK_DIV 관계

**TDC-GPX bus는 비동기이다.** 칩 내부에 bus clock이 없고, WRN/RDN 스트로브의 에지만으로 동작한다. 데이터시트의 "40 MHz transfer rate"는 bus의 **최대 능력(하드웨어 한계)**이며, 이보다 빠르게 읽는 것은 불가능하다.

**BUS_CLK_DIV, BUS_TICKS_PER_TRANSACTION과 bus 속도:**

```
bus_clk:  bus FSM 동작 클록 (독립 클록 소스)
          선택 가능: 200 / 150 / 125 / 100 MHz

BUS_CLK_DIV:  정수 분주기 (CSR, 기본 1)
BUS_TICKS_PER_TRANSACTION:  1회 read/write에 소요되는 bus_tick 수 (CSR, 기본 5)
              설정 가능 범위: 3 ~ 6 (bus_clk에 따라 최소값 결정)

bus_tick      = bus_clk_period × BUS_CLK_DIV
T_transaction = BUS_TICKS_PER_TRANSACTION × bus_tick
bus_rate      = bus_clk / (BUS_TICKS_PER_TRANSACTION × BUS_CLK_DIV)

TDC-GPX 최대 transaction rate = 40 MHz
```

**기본 설정: bus_clk = 200 MHz, BUS_TICKS = 5, BUS_CLK_DIV = 1 → bus_rate = 40 MHz**

#### 5.2.6A bus FSM tick 배분

```
1회 read transaction을 BUS_TICKS tick으로 분할한다.

Phase A (ADR setup):  k_adr ticks — 주소 안정화
Phase L (RDN low):    k_rdl ticks — RDN low, 마지막 tick에서 data sample
Phase H (RDN high):   k_rdh ticks — RDN high, bus turnaround

BUS_TICKS = k_adr + k_rdl + k_rdh

데이터시트 제약:
  tS-AD  ≥  2 ns    k_adr × tick ≥ 2 ns
  tPW-RL ≥  6 ns    k_rdl × tick ≥ 6 ns
  tV-DR  ≤ 11.8 ns  k_rdl × tick ≥ 11.8 ns  (sample 시점에 data valid 보장)
  tPW-RH ≥  6 ns    k_rdh × tick ≥ 6 ns
```

**bus_clk별 최소 BUS_TICKS 결정:**

| bus_clk | tick | tV-DR 충족 (k_rdl×tick≥11.8) | tPW-RH 충족 (k_rdh×tick≥6) | 최소 TICKS | 최대 bus_rate |
|---|---|---|---|---|---|
| 200 MHz | 5 ns | k_rdl≥3 (15ns) | k_rdh≥2 (10ns) | **6** (1+3+2) | 33.3 MHz |
| 200 MHz | 5 ns | k_rdl≥3 (15ns) | k_rdh=1 (5ns **△**) | **5** (1+3+1) | **40 MHz △** |
| 150 MHz | 6.67 ns | k_rdl≥2 (13.3ns) | k_rdh≥1 (6.67ns) | **4** (1+2+1) | 37.5 MHz |
| 125 MHz | 8 ns | k_rdl≥2 (16ns) | k_rdh≥1 (8ns) | **4** (1+2+1) | 31.25 MHz |
| 100 MHz | 10 ns | k_rdl≥2 (20ns) | k_rdh≥1 (10ns) | **3** (1+1+1)? |  |

100 MHz 3-tick 확인: k_rdl=1 → 1×10=10ns < 11.8ns **✗**. k_rdl≥2 필요 → 최소 4 tick.

| bus_clk | 최소 TICKS (안전) | 최소 TICKS (△) | tick 배분 (안전) | 최대 bus_rate (안전) |
|---|---|---|---|---|
| **200 MHz** | **6** | 5 (tPW-RH △) | 1/3/2 | 33.3 MHz |
| **150 MHz** | **4** | — | 1/2/1 | **37.5 MHz** |
| **125 MHz** | **4** | — | 1/2/1 | **31.25 MHz** |
| **100 MHz** | **4** | — | 1/2/1 | **25.0 MHz** |

#### 5.2.6B bus_clk별 달성 가능한 목표 bus_rate

아래 테이블은 목표 bus_rate 중 정수 DIV로 정확히 달성 가능한 (BUS_TICKS, DIV) 조합만 표시한다. ✓=모든 타이밍 충족, △=tPW-RH 마진 부족 (SI 검증 필요).

**(A) bus_clk = 200 MHz**

| bus_rate | BUS_TICKS | DIV | 배분 (A/L/H) | T_trans | T_drain(64) | 판정 |
|---|---|---|---|---|---|---|
| **50 MHz** | **4** | **1** | **1/2/1** | **20 ns** | **1.3 μs** | **△** |
| **40 MHz** | **5** | **1** | **1/3/1** | **25 ns** | **1.6 μs** | **✓*** |
| 25 MHz | 4 | 2 | 1/2/1 | 40 ns | 2.6 μs | ✓ |
| 20 MHz | 5 | 2 | 1/3/1 | 50 ns | 3.2 μs | ✓* |
| 10 MHz | 5 | 4 | 1/3/1 | 100 ns | 6.4 μs | ✓* |
| 8 MHz | 5 | 5 | 1/3/1 | 125 ns | 8.0 μs | ✓* |
| 5 MHz | 5 | 8 | 1/3/1 | 200 ns | 12.8 μs | ✓* |
| 2 MHz | 5 | 20 | 1/3/1 | 500 ns | 32.0 μs | ✓* |
| 1 MHz | 5 | 40 | 1/3/1 | 1.0 μs | 64.0 μs | ✓* |

**판정 범례:**

```
✓   = 모든 데이터시트 타이밍 충족
△   = 단일 read에서 tPW-RH 위반 (5ns < 6ns). 아래 back-to-back 분석 참조
△△  = (사용하지 않음. back-to-back 분석에 의해 tPW-RH 해소 → △로 통합)
       두 가지 위험이 동시에 존재하며, 보드 실측 검증 필수

tPW-RH back-to-back 분석 (200MHz, k_rdh=1):
  단일 read (마지막 read): RDN high 구간 = k_rdh × tick = 1 × 5ns = 5ns < 6ns (△)
  연속 read (back-to-back): RDN high 구간 = (k_rdh + k_adr_next) × tick
                            = (1 + 1) × 5ns = 10ns ≥ 6ns (✓)
  → drain 중 연속 read에서는 다음 transaction의 Phase A(ADR setup)도 RDN=high이므로
    실효 tPW-RH = 10ns. 이는 6ns 요구를 충족한다.
  → **마지막 read (drain 완료 직전 1회)만 tPW-RH=5ns.** IDLE 전환 후 RDN=high 유지되므로
    실질적으로 무한 high → 마지막 read도 문제없음.

  결론: 200MHz/TICKS=5(40MHz)는 drain burst에서 실효적으로 ✓.
        단, 이 해석은 back-to-back 연속 read 또는 drain→IDLE 전환에만 적용.
        read-write-read 교대 패턴에서는 turnaround gap이 별도로 tPW-RH를 보장.

50 MHz (△) 위험:
  ① tV-DR 위반: data valid 최악 11.8ns인데 10ns에 sampling (-1.8ns 부족)
     → 데이터시트 주석: "11.8ns @ 87ps res. & 40pF load, depends on capacitive load"
     → PCB 부하 ≤ 20pF이면 tV-DR ≈ 8~9ns로 줄어들어 10ns sample 가능
  ② tPW-RH: back-to-back에서 실효 (1+1)×5ns=10ns ≥ 6ns ✓ (해소)

40 MHz: back-to-back 분석에 의해 실효 ✓. tV-DR=15ns > 11.8ns ✓.
```

✓* = back-to-back read에서 실효 tPW-RH=(k_rdh+k_adr)×tick=10ns≥6ns. 단일 read 기준 5ns<6ns이나 drain 운용에서는 실효 ✓.
△ = tV-DR 위반만 (tPW-RH는 back-to-back으로 해소). PCB 저부하 실측 필수.

**(B) bus_clk = 150 MHz**

| bus_rate | BUS_TICKS | DIV | 배분 (A/L/H) | T_trans | T_drain(64) | 판정 |
|---|---|---|---|---|---|---|
| **25 MHz** | **6** | **1** | **1/4/1** | **40 ns** | **2.6 μs** | **✓** |
| **10 MHz** | **5** | **3** | **1/3/1** | **100 ns** | **6.4 μs** | **✓** |
| 5 MHz | 5 | 6 | 1/3/1 | 200 ns | 12.8 μs | ✓ |
| 5 MHz | 6 | 5 | 1/4/1 | 200 ns | 12.8 μs | ✓ |
| 3 MHz | 5 | 10 | 1/3/1 | 333 ns | 21.3 μs | ✓ |
| 2 MHz | 5 | 15 | 1/3/1 | 500 ns | 32.0 μs | ✓ |
| 1 MHz | 5 | 30 | 1/3/1 | 1.0 μs | 64.0 μs | ✓ |
| 1 MHz | 6 | 25 | 1/4/1 | 1.0 μs | 64.0 μs | ✓ |

150 MHz는 BUS_TICKS 가변으로 **25 MHz 달성** (TICKS=6, DIV=1). 모든 조합 ✓.

**(C) bus_clk = 125 MHz**

| bus_rate | BUS_TICKS | DIV | 배분 (A/L/H) | T_trans | T_drain(64) | 판정 |
|---|---|---|---|---|---|---|
| **25 MHz** | **5** | **1** | **1/3/1** | **40 ns** | **2.6 μs** | **✓** |
| 5 MHz | 5 | 5 | 1/3/1 | 200 ns | 12.8 μs | ✓ |
| 1 MHz | 5 | 25 | 1/3/1 | 1.0 μs | 64.0 μs | ✓ |

125 MHz는 달성 가능한 목표가 3개로 제한적. 모든 조합 ✓.

**(D) bus_clk = 100 MHz**

| bus_rate | BUS_TICKS | DIV | 배분 (A/L/H) | T_trans | T_drain(64) | 판정 |
|---|---|---|---|---|---|---|
| **25 MHz** | **4** | **1** | **1/2/1** | **40 ns** | **2.6 μs** | **✓** |
| **20 MHz** | **5** | **1** | **1/3/1** | **50 ns** | **3.2 μs** | **✓** |
| 10 MHz | 5 | 2 | 1/3/1 | 100 ns | 6.4 μs | ✓ |
| 5 MHz | 4 | 5 | 1/2/1 | 200 ns | 12.8 μs | ✓ |
| 5 MHz | 5 | 4 | 1/3/1 | 200 ns | 12.8 μs | ✓ |
| 2 MHz | 5 | 10 | 1/3/1 | 500 ns | 32.0 μs | ✓ |
| 1 MHz | 4 | 25 | 1/2/1 | 1.0 μs | 64.0 μs | ✓ |
| 1 MHz | 5 | 20 | 1/3/1 | 1.0 μs | 64.0 μs | ✓ |

100 MHz는 BUS_TICKS=4로 **25 MHz 달성**. 모든 조합 ✓.

#### 5.2.6C bus_clk별 최적 설정 비교

| bus_clk | 최대 bus_rate | T_drain | T_service | 순차 최대 | pipeline 최대 | 판정 |
|---|---|---|---|---|---|---|
| **200 MHz TICKS=4** | **50 MHz** | **1.3 μs** | **3.63 μs** | **705m** | **987m** | **△** |
| **200 MHz TICKS=5** | **40 MHz** | **1.6 μs** | **3.93 μs** | **659m** | **987m** | **✓*** |
| 200 MHz TICKS=6 | 33.3 MHz | 1.9 μs | 4.23 μs | 615m | 942m | ✓ |
| **150 MHz TICKS=4** | **37.5 MHz** | **1.7 μs** | **4.03 μs** | **645m** | **981m** | **✓** |
| 125 MHz TICKS=4 | 31.25 MHz | 2.0 μs | 4.33 μs | 600m | 936m | ✓ |
| 100 MHz TICKS=4 | 25.0 MHz | 2.6 μs | 4.93 μs | 510m | 847m | ✓ |

**관찰:**
- **200 MHz TICKS=4 (50MHz △)**: 최고 성능. tV-DR 위험 (tPW-RH는 back-to-back으로 해소). 저부하 PCB 실측 필수.
- **200 MHz TICKS=5 (40MHz ✓*)**: back-to-back read에서 실효 tPW-RH=10ns. TDC-GPX 공칭 최대 속도. **Phase 1 기본 권장.**
- **150 MHz TICKS=4 (37.5MHz ✓)**: 모든 타이밍 안전하면서 200MHz △에 근접. **SI 안전 최적.**
- **100 MHz TICKS=4 (25MHz ✓)**: 가장 넓은 목표 주파수 달성 (25, 20, 10, 5, 2, 1 MHz).

**CSR 레지스터:**

```
BUS_CLK_DIV               : CSR R/W, 기본 1 (정수 분주기)
BUS_TICKS_PER_TRANSACTION : CSR R/W, 기본 5 (3~6, bus_clk에 따라 최소값 제한)

bus_rate = bus_clk / (BUS_TICKS_PER_TRANSACTION × BUS_CLK_DIV)
```

#### 5.2.6D bus_phy 모듈 설계

##### 5.2.6D.1 bus_phy 아키텍처

```
bus_phy 내부 구조:

  chip_ctrl (상위)          bus_phy (본 모듈)
  ─────────────────         ─────────────────────────────────
  bus_req ──────────────→  ┌─────────────────────────────┐
  (addr, wdata, r/w)       │  Upper FSM                  │
                            │  IDLE → READ → IDLE         │
  bus_rsp ←────────────── │        → WRITE → IDLE        │
  (rdata, valid)            │        → CFG_WRITE → IDLE   │
                            │                             │
                            │  ┌───────────────────────┐  │
                            │  │ Tick Counter           │  │  ← bus_clk domain
                            │  │ 0 → 1 → ... → N-1    │  │
                            │  │ (N = BUS_TICKS)       │  │
                            │  │                       │  │
                            │  │ tick_en 생성:          │  │  ← BUS_CLK_DIV에 의한 분주
                            │  │   div_cnt++           │  │
                            │  │   tick_en = (div_cnt  │  │
                            │  │     == BUS_CLK_DIV-1) │  │
                            │  └───────────────────────┘  │
                            │                             │
                            │  ┌───────────────────────┐  │
                            │  │ Pin Driver            │  │  → 물리 핀
                            │  │ ADR[3:0], CSN, RDN,   │  │
                            │  │ WRN, OEN              │  │
                            │  │ D[27:0] IOBUF (bidir) │  │
                            │  │ IOB FF (sample)       │  │
                            │  └───────────────────────┘  │
                            │                             │
                            │  ┌───────────────────────┐  │
                            │  │ 2-FF Synchronizer     │  │  ← 비동기 입력
                            │  │ EF1, EF2, LF1, LF2   │  │
                            │  │ → ef1_sync, lf1_sync  │  │
                            │  └───────────────────────┘  │
                            └─────────────────────────────┘
```

##### 5.2.6D.2 Upper FSM

```
Upper FSM 상태:

  IDLE ─── bus_req = READ ───→ READ ──→ (tick counter N-1 도달) ──→ IDLE
    │                                                                  ↑
    ├── bus_req = WRITE ──→ WRITE ─→ (tick counter N-1 도달) ──────→ │
    │                                                                  │
    └── bus_req = NONE ──→ IDLE (유지) ────────────────────────────→ ┘

Upper FSM은 transaction 종류만 결정한다.
실제 핀 제어 타이밍은 Tick Counter가 담당한다.
```

##### 5.2.6D.3 Tick Counter와 Phase 분배

```
1회 transaction = BUS_TICKS ticks

BUS_TICKS ticks를 3개 Phase로 분배:

  Phase A (ADR setup):   k_adr ticks    = 1 (고정)
  Phase L (strobe low):  k_rdl ticks    = BUS_TICKS - k_adr - k_rdh (자동 산출)
  Phase H (strobe high): k_rdh ticks    = 1 (기본) 또는 2 (200MHz 안전 모드)

┌──────┐ ┌──────────────────────┐ ┌──────────┐
│Phase A│ │       Phase L        │ │ Phase H  │
│k_adr │ │       k_rdl          │ │ k_rdh    │
│ ticks│ │       ticks          │ │ ticks    │
└──────┘ └──────────────────────┘ └──────────┘
│← ADR →│← RDN/WRN low ────────→│← high ──→│
  setup    strobe + data valid     turnaround

tick_cnt:  0        k_adr     k_adr+k_rdl-1  k_adr+k_rdl   BUS_TICKS-1
           │         │              │              │              │
           ADR set   strobe↓      sample(R)     strobe↑        완료
                                  /hold(W)
```

**BUS_TICKS별 Phase 분배:**

| BUS_TICKS | k_adr | k_rdl | k_rdh | READ sample 위치 | WRITE hold 위치 |
|---|---|---|---|---|---|
| 3 | 1 | 1 | 1 | tick 1 (strobe 1tick 후) | tick 1 |
| 4 | 1 | 2 | 1 | tick 2 (strobe 2tick 후) | tick 2 |
| **5** | **1** | **3** | **1** | **tick 3 (strobe 3tick 후)** | **tick 3** |
| 6 | 1 | 3 | 2 | tick 3 | tick 3 |

##### 5.2.6D.4 READ transaction 핀 제어

```
Tick Counter → 핀 출력 매핑 (READ):

tick_cnt 범위          │ ADR    │ CSN │ OEN │ RDN │ D-bus     │ 동작
───────────────────────┼────────┼─────┼─────┼─────┼──────────┼──────────
0 ~ k_adr-1           │ valid  │  0  │  0  │  1  │ Hi-Z     │ ADR 안정화
k_adr ~ k_adr+k_rdl-2 │ valid  │  0  │  0  │  0  │ chip→FPGA│ RDN low, data transitioning
k_adr+k_rdl-1         │ valid  │  0  │  0  │  0  │ chip→FPGA│ ★ IOB FF sample
k_adr+k_rdl ~ N-1     │ valid  │  0  │  0  │  1  │ chip→FPGA│ RDN high (turnaround)
```

**OEN 모드에 따른 동작 차이:**

```
[일반 모드] OEN = transaction 중에만 low
  IDLE:     OEN=1 (칩 D-bus Hi-Z)
  READ 시:  OEN=0 (칩 D-bus drive) → tOE-R ≤ 8.5ns 대기 후 data 유효
  WRITE 시: OEN=1 유지 [INV-1]

[OEN permanent low 모드] drain 전체 구간에서 OEN=0 유지
  장점: 연속 READ 시 OEN toggle 오버헤드 제거
       tH-DR "무한 연장" (데이터시트: OEN=0 + stable address)
  제약: WRITE 불가 [INV-7]
       drain 시작 전 OEN=0 설정, drain 완료 후 OEN=1 복귀
```

##### 5.2.6D.5 WRITE transaction 핀 제어

```
tick_cnt 범위          │ ADR    │ CSN │ OEN │ WRN │ D-bus     │ 동작
───────────────────────┼────────┼─────┼─────┼─────┼──────────┼──────────
0 ~ k_adr-1           │ valid  │  0  │  1  │  1  │ FPGA→chip│ ADR + DATA 안정화
k_adr ~ k_adr+k_rdl-2 │ valid  │  0  │  1  │  0  │ FPGA→chip│ WRN low (칩이 latch)
k_adr+k_rdl-1         │ valid  │  0  │  1  │  0  │ FPGA→chip│ WRN low hold
k_adr+k_rdl ~ N-1     │ valid  │  0  │  1  │  1  │ FPGA→chip│ WRN high, data hold

OEN은 WRITE 전체 구간에서 반드시 '1' [INV-1]
D-bus는 FPGA가 drive (d_bus_oe = '1')
```

##### 5.2.6D.6 READ↔WRITE 전환 (bus turnaround)

```
[READ → WRITE 전환] — 칩 drive 중 → FPGA drive로 전환

  ... READ 완료 ...
  tick N-1:  RDN↑, OEN 아직 low (칩이 D-bus drive 중)
  ┌────────────────────────────────────────────────┐
  │ turnaround gap (최소 1 tick)                    │
  │  OEN = 1 설정 (칩 D-bus → Hi-Z, tOE-F ≤ 9ns)  │
  │  d_bus_oe = 0 유지 (FPGA도 Hi-Z)               │
  │  → 양쪽 모두 Hi-Z인 상태에서 1 tick 대기        │
  └────────────────────────────────────────────────┘
  WRITE tick 0: d_bus_oe = 1 (FPGA drive 시작)
                OEN = 1 유지 [INV-1]

[WRITE → READ 전환] — FPGA drive 중 → 칩 drive로 전환

  ... WRITE 완료 ...
  tick N-1:  WRN↑, FPGA가 D-bus drive 중
  ┌────────────────────────────────────────────────┐
  │ turnaround gap (최소 1 tick)                    │
  │  d_bus_oe = 0 설정 (FPGA D-bus → Hi-Z)        │
  │  OEN = 1 유지 (칩도 Hi-Z)                      │
  │  → 양쪽 모두 Hi-Z인 상태에서 1 tick 대기        │
  └────────────────────────────────────────────────┘
  READ tick 0: OEN = 0 설정 (칩 drive 시작, tOE-R ≤ 8.5ns)
               d_bus_oe = 0 유지 (FPGA Hi-Z) [INV-2]
```

##### 5.2.6D.7 Tick Enable 생성 (BUS_CLK_DIV)

```vhdl
-- bus_clk domain에서 tick enable 생성
signal div_cnt : unsigned(7 downto 0) := (others => '0');
signal tick_en : std_logic;

process(bus_clk)
begin
    if rising_edge(bus_clk) then
        if div_cnt = unsigned(BUS_CLK_DIV) - 1 then
            div_cnt <= (others => '0');
            tick_en <= '1';
        else
            div_cnt <= div_cnt + 1;
            tick_en <= '0';
        end if;
    end if;
end process;

-- tick_en이 '1'일 때만 tick_cnt 증가 및 핀 상태 전이
-- BUS_CLK_DIV=1이면 매 bus_clk cycle마다 tick_en='1' (분주 없음)
-- BUS_CLK_DIV=2이면 2 cycle마다 1회 tick_en
```

##### 5.2.6D.8 Parameterized READ FSM

```vhdl
-- BUS_TICKS와 Phase 경계 산출
constant K_ADR : natural := 1;
constant K_RDH : natural := 1;  -- 200MHz 안전모드에서는 2
constant K_RDL : natural := BUS_TICKS - K_ADR - K_RDH;

signal tick_cnt    : unsigned(2 downto 0);  -- 0 ~ BUS_TICKS-1
signal sample_en   : std_logic;
signal rsp_valid   : std_logic;

process(bus_clk)
begin
    if rising_edge(bus_clk) then
        if tick_en = '1' then
            sample_en <= '0';
            rsp_valid <= '0';

            case upper_fsm is
                when READ_ACTIVE =>
                    -- Phase A: ADR setup
                    if tick_cnt < K_ADR then
                        adr_pin <= req_addr;
                        csn_pin <= '0';
                        oen_pin <= '0';
                        rdn_pin <= '1';         -- 아직 high

                    -- Phase L: RDN low
                    elsif tick_cnt < K_ADR + K_RDL then
                        rdn_pin <= '0';         -- strobe active
                        -- Phase L 마지막 tick에서 sample
                        if tick_cnt = K_ADR + K_RDL - 1 then
                            sample_en <= '1';   -- IOB FF latch
                        end if;

                    -- Phase H: RDN high
                    else
                        rdn_pin <= '1';         -- strobe release
                        -- 마지막 tick에서 transaction 완료
                        if tick_cnt = BUS_TICKS - 1 then
                            rsp_valid <= '1';   -- rdata valid
                            tick_cnt <= (others => '0');
                            if next_req then
                                upper_fsm <= READ_ACTIVE;  -- 연속 read
                            else
                                upper_fsm <= IDLE;
                            end if;
                        end if;
                    end if;

                    tick_cnt <= tick_cnt + 1;

                when others => ...
            end case;
        end if;
    end if;
end process;

-- IOB FF (물리 핀에서 직접 샘플)
attribute IOB : string;
attribute IOB of d_bus_sample : signal is "TRUE";
process(bus_clk)
begin
    if rising_edge(bus_clk) then
        if sample_en = '1' then
            d_bus_sample <= d_bus_in;  -- IOB FF에서 캡처
        end if;
    end if;
end process;
```

##### 5.2.6D.9 bus_phy 신호 요약

| 신호 | 방향 | 폭 | 설명 |
|---|---|---|---|
| **chip_ctrl → bus_phy** | | | |
| bus_req_valid | in | 1 | transaction 요청 |
| bus_req_rw | in | 1 | 0=READ, 1=WRITE |
| bus_req_addr | in | 4 | TDC-GPX 레지스터 주소 (0~14) |
| bus_req_wdata | in | 28 | write 데이터 |
| oen_permanent | in | 1 | 1=drain 중 OEN 상시 low |
| **bus_phy → chip_ctrl** | | | |
| bus_rsp_valid | out | 1 | read 데이터 유효 (1 tick pulse) |
| bus_rsp_rdata | out | 28 | read 데이터 (IOB FF output) |
| bus_busy | out | 1 | transaction 진행 중 |
| **bus_phy → 물리 핀** | | | |
| adr_pin | out | 4 | TDC-GPX ADR[3:0] |
| csn_pin | out | 1 | TDC-GPX CSN (active low) |
| rdn_pin | out | 1 | TDC-GPX RDN (active low) |
| wrn_pin | out | 1 | TDC-GPX WRN (active low) |
| oen_pin | out | 1 | TDC-GPX OEN (active low) |
| d_bus_pin | bidir | 28 | TDC-GPX D[27:0] (IOBUF) |
| **물리 핀 → bus_phy** | | | |
| ef1_pin, ef2_pin | in | 1 each | Empty Flag (→ 2-FF sync) |
| lf1_pin, lf2_pin | in | 1 each | Load-level Flag (→ 2-FF sync) |
| **추가 제어 핀** | | | |
| stopdis_pin[1:4] | out | 4 | Stop Disable (channel enable/disable) |
| alutrigger_pin | out | 1 | AluTrigger (post-processing start) |
| puresn_pin | out | 1 | Power-up Reset (active low) |

#### 5.2.6.1 SINGLE_SHOT 모드: 40 MHz 동작 (BUS_CLK_DIV = 1)

SINGLE_SHOT에서는 MTimer 윈도우 종료 후 IFIFO를 **burst drain**한다. hit가 더 이상 들어오지 않으므로 지속 속도(sustained rate) 걱정이 없다. 가능한 한 **빠르게 drain**하는 것이 shot 간격 내 완료에 유리하다.

```
BUS_CLK_DIV = 1 (40 MHz, 칩 최대):
  T_drain = 64 × 25ns = 1.6 μs (전략 C Source Gating, 최대 64 entries)
  T_service ≈ 1.7 + 1.0 + 1.28 + 0.05 = 4.03 μs
```

| 최대 거리 | T_roundtrip | T_service (DIV=1) | T_합계 | T_가용 (8.33μs) | 판정 |
|---|---|---|---|---|---|
| 150 m | 1.0 μs | 4.0 μs | 5.0 μs | 3.33 μs | ✓ 충분 |
| 400 m | 2.67 μs | 4.0 μs | 6.67 μs | 1.66 μs | ✓ |
| 650 m | 4.33 μs | 4.0 μs | 8.33 μs | 0 μs | ← 한계 |
| 750 m | 5.0 μs | 4.0 μs | 9.0 μs | -0.67 μs | ✗ (pipeline 필요) |

**SINGLE_SHOT에서 40 MHz가 가능한 이유:**
- burst drain이므로 지속 속도 문제 없음
- 칩의 40 MHz 한계 = 25 ns/transaction이고, BUS_CLK_DIV=1에서 5 bus_ticks × 5ns = 25ns로 정확히 일치
- 데이터시트 최소 타이밍: tPW-RL ≥ 6ns → 2 bus_ticks × 5ns = 10ns > 6ns ✓, tS-AD ≥ 2ns → 1 bus_tick × 5ns = 5ns > 2ns ✓, tV-DR ≤ 11.8ns → sample at 3rd tick = 15ns > 11.8ns ✓


#### 5.2.6.2 IFIFO overflow 관점

**SINGLE_SHOT**: MTimer 윈도우 동안 hit가 모두 수집된 후 drain한다. 전략 C(Source Gating) 적용 시 IFIFO1에 최대 32, IFIFO2에 최대 32, 양쪽 합산 최대 64 entries. 측정 중에는 읽지 않으므로 overflow 문제 없음.

#### 5.2.7 throughput 비교

| tdata_width | beats/line | @200MHz clk | line 전송 시간 | 비고 |
|---|---|---|---|---|
| 32-bit | 256 | 1280 ns | 1.28 μs | Zynq-7000 기본 |
| 64-bit | 128 | 640 ns | 0.64 μs | Zynq-7000 HP 64-bit |
| 128-bit | 64 | 320 ns | 0.32 μs | Zynq MPSoC |

Phase 1 (Zynq-7000, 32-bit): 한 line 전송에 1.28 μs. shot 간격 8.33 μs 내 충분.

### 5.3 수직 채널과 면(face)의 관계 (매핑 전 필수 이해)

폴리곤 미러가 N_FACES개의 면을 가질 때, 수직 해상도 구조는 다음과 같다:

```
5면 폴리곤 미러 예시:

                    수평 화각 (face 내 shot 방향)
                    ←──────────────────────────→
  face 0  수직 ↕  [shot0][shot1]...[shot2399]     ← 32 ch (rows_per_face rows)
  face 1  수직 ↕  [shot0][shot1]...[shot2399]     ← 32 ch
  face 2  수직 ↕  [shot0][shot1]...[shot2399]     ← 32 ch
  face 3  수직 ↕  [shot0][shot1]...[shot2399]     ← 32 ch
  face 4  수직 ↕  [shot0][shot1]...[shot2399]     ← 32 ch
                                                   ─────
                                          수직 전체 = 160 ch

  1 shot = rows_per_face채널 = 1개 face의 수직 slice (수직 전체의 1/5)
  수직 전체 = N_FACES × rows_per_face = 5 × 32 = 160 채널
  수직 전체 완성 = 미러 1회전(= 1 scan_frame = 5 vdma_frames)
```

**따라서 1 shot의 rows_per_face개 cell은 "수직 채널의 끝"이 아니라 "해당 face의 수직 slice 1개"이다.**

### 5.4 VDMA 전송 범위 선택 (PACKET_SCOPE)


#### 5.4.1 Shot-Major 매핑 (고정)

```
비유: 세로로 긴 영상. 가로 = 채널 수(32), 세로 = shot 수(200+)

  HSIZE 방향 (pixels_per_line):
    = 1 shot의 수직 채널 slice
    = rows_per_face개 cell (가변, 기본 32)
    = rows_per_face × cell_size bytes

  VSIZE 방향 (lines):
    = 수평 shot 수 + header
    = cols_per_face + 1

  tlast 의미:
    = "이 shot의 수직 slice 전송 완료"
    (≠ 수직 채널 전체 끝. 수직 전체는 N_FACES개 vdma_frame을 합쳐야 완성)

  tuser(0) 의미:
    = "이 face(vdma_frame) 시작"
```

**DDR 레이아웃:**

```
base + stride × 0:  [header line     ] (rows_per_face × cell_size bytes)
base + stride × 1:  [shot 0: ch0~ch31] (rows_per_face × cell_size bytes)
base + stride × 2:  [shot 1: ch0~ch31]
...
base + stride × N:  [shot N-1: ch0~ch31]
```

**VDMA 설정:**

| 레지스터 | 값 | 예시 (rows_per_face=32, cols=200, cell=32B) |
|---|---|---|
| S2MM_HSIZE | rows_per_face × cell_size (가변) | 기본 32 × 32 = 1024 bytes (packet_start latch) |
| S2MM_VSIZE | cols_per_face + 1 | 2401 lines |
| S2MM_STRIDE | ≥ HSIZE (정렬) | 2048 bytes |

**AXI-Stream 타이밍:**

```
시간 →

[header]──tlast──[blank]──[shot0]──tlast──[blank]──[shot1]──tlast──...──[shotN]──tlast
↑SOF                       ↑                       ↑                     ↑
tuser(0)=1                 Active Video            Active Video          마지막 line
                            (rows_per_face cells)               (rows_per_face cells)
```

**장점:**
- shot 완료 즉시 전송 가능 (transpose buffer 불필요)
- 데이터 생성 순서와 전송 순서 일치
- cell_builder의 double-buffer만으로 pipeline 가능
- FPGA 리소스 최소

**단점:**
- 비디오 직관(가로=수평, 세로=수직)과 축이 뒤집힘
- DDR에서 SW가 "수평 i번째, 수직 j번째"를 읽으려면 `base + stride × (i+1) + j × cell_size`로 접근 (행/열 반전)
- 수직 전체를 보려면 N_FACES개 vdma_frame을 SW가 합성해야 함

#### 5.4.2 CSR 선택

| Offset | 이름 | 설명 |
|---|---|---|
| CSR 0x0C | `PACKET_SCOPE` | [0] 0 = face 단위 전송 (기본), 1 = scan_frame 단위 전송 |

**Phase 1 권장: PACKET_SCOPE = 0 (face 단위).**

#### 5.4.3 PACKET_SCOPE: face 단위 vs scan_frame 단위 전송

VDMA가 1회 SOF~다음 SOF 사이에 전송하는 범위를 선택한다.

**PACKET_SCOPE = 0: face 단위 전송 (기본)**

```
VDMA 관점 1 "frame" = 1 face = 1 vdma_frame (PACKET_SCOPE=0 기준)

  SOF → [header] [shot0] [shot1] ... [shot2399] → 다음 SOF
         └────────── VSIZE = 2401 lines ──────────┘

  1회전(100ms) 동안 VDMA "frame"이 5번 발생
  SW가 5개 vdma_frame을 모아서 1 LiDAR frame으로 합성
```

| 항목 | 값 |
|---|---|
| VSIZE | cols_per_face + 1 = **2401 lines** |
| SOF 주기 | face 지속 시간 = **20 ms** |
| vdma_frame buffer 크기 | stride × 2401 = **~4.69 MB** |
| triple buffer DDR | 3 × 4.69 = **~14.07 MB** |
| SW 합성 필요 | 5개 vdma_frame → 1 scan_frame |

**PACKET_SCOPE = 1: scan_frame 단위 전송 [Phase 2]**

```
[Phase 2 설계. Phase 1에서는 PACKET_SCOPE=0만 지원한다.]
[SCOPE=1의 DDR layout, header 배치, SW parser는 Phase 2에서 별도 설계한다.]

VDMA 관점 1 "frame" = N_FACES faces = 1 scan_frame

  SOF → [hdr0][face0 shots][hdr1][face1 shots]...[hdr(N-1)][face(N-1) shots] → 다음 SOF
         └──────────── VSIZE = N_FACES × (cols_per_face + 1) lines ────────────┘

  1회전(100ms) 동안 VDMA "frame"이 1번 발생
  DDR에 저장된 buffer 자체가 완성된 1 scan_frame
```

| 항목 | 값 |
|---|---|
| VSIZE | N_FACES × (cols_per_face + 1) = 5 × 2401 = **12005 lines** |
| SOF 주기 | 1회전 = **100 ms** |
| vdma_frame buffer 크기 | stride × 12005 = **~23.45 MB** |
| triple buffer DDR | 3 × 23.45 = **~70.34 MB** |
| SW 합성 필요 | 불필요 (buffer = 완성 scan_frame) |

**비교:**

| 항목 | face 단위 (SCOPE=0, Phase 1) | scan_frame 단위 (SCOPE=1, Phase 2) |
|---|---|---|
| VSIZE | 2401 | **12005** |
| DDR 사용량 (triple) | ~14.07 MB | **~70.34 MB** |
| SOF 주기 | 20 ms (빠름) | 100 ms |
| SW scan_frame 합성 | 필요 (5 vdma_frame 조립) | **불필요** |
| face 간 동기 오류 위험 | SW가 관리 | HW가 보장 |
| VDMA timeout 위험 | 낮음 (짧은 전송) | 높음 (긴 전송 중 에러 시 전체 유실) |
| 개별 face 재전송 | 가능 | 불가 (scan_frame 단위) |

**Phase 1 권장: PACKET_SCOPE = 0 (face 단위).** DDR 절약, 개별 face 관리 가능. scan_frame 합성은 SW에서 5개 vdma_frame을 순서대로 이어붙이면 됨.

#### 5.4.4 수직 전체 해상도와 scan_frame 관계

```
수직 전체 = N_FACES × rows_per_face = 5 × 32 = 160 채널
수평 전체 = cols_per_face = 2400 shots

1 scan_frame = 12000 shots × 32 ch = 384,000 cells
            (header 제외)

  PACKET_SCOPE=0: 5 vdma_frames (각 2400 shots × 32 ch) → SW 합성
  PACKET_SCOPE=1: 1 VDMA frame (12005 lines) → DDR에 완성 상태
```

**face별 수직 채널 배치:**

```
face 0: row 0~31     (수직 화각의 1/5)
face 1: row 32~63    (수직 화각의 2/5)
face 2: row 64~95    (수직 화각의 3/5)
face 3: row 96~127   (수직 화각의 4/5)
face 4: row 128~159  (수직 화각의 5/5)

scan_frame = 160 ch × 2400 shots = 완성된 3D point cloud 1장
```

SW가 scan_frame을 복원할 때:
- **PACKET_SCOPE=0**: 5개 vdma_frame을 face 순서대로 수직 방향으로 concat
- **PACKET_SCOPE=1**: buffer에서 직접 읽기 (face별 header로 경계 식별)

---

## 제6장. 구체적 데이터 흐름 (블록 단위)

### 6.1 전체 경로

```
[TDC-GPX 칩]
    │ (물리 핀: D[27:0], ADR[3:0], WRN, RDN, CSN, OEN,
    │  PuResN, StopDis1~4, AluTrigger, TStart, TStop1~8,
    │  EF1, EF2, IrFlag, ErrFlag, RefClk, Phase)
    ▼
[bus_phy]          28-bit async bus timing, IOBUF, IOB FF
    │ (rsp_rdata 28-bit)
    │ + StopDis/AluTrigger/PuResN 제어 출력
    ▼
[chip_ctrl]        DRAIN FSM이 EF 확인 후 read 요청
    │ (raw word 28-bit + ififo_id)
    ▼
[decode_i]         raw word → stop_id_local, slope, raw_hit 분리
    │ (decoded fields)
    ▼
[raw_event_builder]   hit_seq_local 부여, raw_event record 생성
    │ (raw_event record)
    │
    ├──→ [m_axis_raw_event]     (optional: debug/log 경로, sparse)
    │
    ▼
[cell_builder]     raw_event → cell 격자에 저장, MAX_HITS 제한, zero-padding
    │ (stops_per_chip개 cell = chip당 1 slice)
    ▼
[chip_slice_out]   cell[0]~cell[stops_per_chip-1]을 stop 번호 오름차순으로 AXI-Stream 출력
    │ (m_axis_chip_slice: stop0 → stop1 → ... → stop7)
    ▼
[face_packet_assembler]   활성 chip slice를 chip 번호 오름차순으로 concat (Packed Row)
    │ (chip0 slice → chip1 slice → chip2 slice → chip3 slice = 1 VDMA line)
    ▼
[packet_header_inserter]  packet 첫 line에 header 삽입
    │ (header line + data lines)
    ▼
[axis_data_fifo]   VDMA backpressure 완충
    │
    ▼
[m_axis_vdma_video]   SOF/EOL 부여된 최종 AXI4-Stream Video
    │
    ▼
[AXI VDMA S2MM]    Xilinx IP. stream을 DDR3에 기록
    │
    ▼
[DDR3 vdma_frame buffer]   완성된 vdma_frame 저장
```

### 6.2 각 블록의 입출력 데이터 형태

| 블록 | 입력 | 출력 | 데이터 특성 |
|---|---|---|---|
| bus_phy | req (addr, wdata) | rsp (rdata 28-bit) | 단일 transaction |
| chip_ctrl | EF, IrFlag | bus req sequence | FSM 제어 |
| decode_i | raw word 28-bit, ififo_id | decoded fields | 조합 변환 |
| raw_event_builder | decoded fields, shot_seq | raw_event record | sparse, hit만 |
| cell_builder | raw_event stream | stops_per_chip cells | sparse → dense 변환 |
| chip_slice_out | stops_per_chip cells | AXI-Stream stops_per_chip pixels | stop 번호 오름차순 출력 |
| face_packet_assembler | 활성 chip slice | AXI-Stream rows_per_face pixels (가변) | 활성 chip 오름차순 concat |
| packet_header_inserter | rows_per_face-pixel lines | header + data lines | metadata 삽입 |
| m_axis_vdma_video | lines with SOF/EOL | VDMA input | 최종 출력 |

---

## 제7장. Cell Builder 동작 상세

이 블록이 sparse → dense 변환의 핵심이므로 상세히 기술한다.

### 7.1 내부 구조

```
cell_builder 내부:

  cell_buffer[0..stops_per_chip-1]  : stops_per_chip개 cell (활성 stop만)
                                       stop_id_local 0 ~ stops_per_chip-1에 대응
  hit_counter[0..stops_per_chip-1]  : stops_per_chip개 counter
  shot_active          : 현재 shot의 cell 수집 중
  drain_done            : chip_ctrl로부터 drain 완료 신호

  ★ Packed Row 정책:
    cell_buffer 크기는 stops_per_chip (CSR, packet_start 시 latch).
    비활성 stop의 cell은 존재하지 않으며, zero-fill도 불필요.
    → stops_per_chip=6이면 cell_buffer는 6개, 인덱스 0~5.
    → 비활성 chip은 assembler가 건너뜀 (cell_builder 자체가 동작하지 않음).
```

### 7.2 동작 시퀀스

```
Phase A: 초기화 (stops_per_chip개 cell만)
  shot 시작 시:
    for i in 0..stops_per_chip-1:              -- 활성 stop만
      cell_buffer[i].hit_slot[0..MAX-1] = 0
      cell_buffer[i].hit_valid = 0
      cell_buffer[i].hit_count_actual = 0
      cell_buffer[i].hit_dropped = 0
      cell_buffer[i].error_fill = 0
      hit_counter[i] = 0
    shot_active = 1

Phase B: 수집
  raw_event가 들어올 때마다:
    row = raw_event.stop_id_local              -- 0 ~ stops_per_chip-1
    seq = raw_event.hit_seq_local

    if hit_counter[row] < MAX_HITS_PER_STOP:
      cell_buffer[row].hit_slot[seq] = raw_event.raw_hit
      cell_buffer[row].hit_valid[seq] = 1
      cell_buffer[row].slope_vec[seq] = raw_event.slope
      cell_buffer[row].hit_count_actual += 1
    else:
      cell_buffer[row].hit_dropped = 1

    hit_counter[row] += 1

Phase C: 출력 (stops_per_chip개 cell만)
  drain_done 수신 후:
    for row in 0..stops_per_chip-1:            -- 활성 stop만 출력
      cell_buffer[row]를 assembler에 전달
    shot_active = 0
```

### 7.3 Packed Row에서 zero-fill이 불필요한 이유

Packed Row에서는 비활성 stop/chip의 row가 VDMA line에 존재하지 않는다. cell_builder는 활성 stop(stops_per_chip개)만 버퍼링하고 출력한다. assembler는 활성 chip의 cell_builder 출력만 수집하여 VDMA line을 구성한다.

Phase A에서 초기화하는 것은 "활성 stop 중 hit가 없는 경우"를 위한 것이지, 비활성 stop을 위한 zero-fill이 아니다.

### 7.4 구체 예시

MAX_HITS_PER_STOP = 8 (기본값) 인 경우.

chip2의 한 shot에서:
- stop1 채널에 80ns, 97ns, 131ns 세 개의 에코
- stop6 채널에 103ns 한 개의 에코
- 나머지 채널 에코 없음

결과:

```
cell[0] (stop0): slot=[0,0,0,0,0,0,0,0], valid=[0,0,0,0,0,0,0,0], count=0, dropped=0
cell[1] (stop1): slot=[80ns,97ns,131ns,0,0,0,0,0], valid=[1,1,1,0,0,0,0,0], count=3, dropped=0
cell[2] (stop2): slot=[0,0,0,0,0,0,0,0], valid=[0,0,0,0,0,0,0,0], count=0, dropped=0
cell[3] (stop3): slot=[0,0,0,0,0,0,0,0], valid=[0,0,0,0,0,0,0,0], count=0, dropped=0
cell[4] (stop4): slot=[0,0,0,0,0,0,0,0], valid=[0,0,0,0,0,0,0,0], count=0, dropped=0
cell[5] (stop5): slot=[0,0,0,0,0,0,0,0], valid=[0,0,0,0,0,0,0,0], count=0, dropped=0
cell[6] (stop6): slot=[103ns,0,0,0,0,0,0,0], valid=[1,0,0,0,0,0,0,0], count=1, dropped=0
cell[7] (stop7): slot=[0,0,0,0,0,0,0,0], valid=[0,0,0,0,0,0,0,0], count=0, dropped=0
```

이 stops_per_chip개 cell이 순서대로 출력되면 chip2의 한 shot line이 완성된다.

---

## 제8장. Face Frame Assembler 동작 상세

### 8.1 역할

4개 chip의 chip_slice(각 stops_per_chip pixel)를 한 줄(rows_per_face pixel)로 합친다.

### 8.2 동기화 및 Timeout 정책

4개 lane은 독립적으로 drain하므로, slice 완성 시점이 미세하게 다를 수 있다. 정상 동작 시 assembler는 4개 lane 모두 현재 shot의 slice를 완성할 때까지 기다린 후 concat한다.

```
정상 동기화:

  slice_ready[0..3] : 각 lane의 현재 shot slice 완료 플래그

  대기: slice_ready = "1111" 될 때까지
  출력: chip0 slice → chip1 slice → chip2 slice → chip3 slice 순서로 연결
  완료: slice_ready clear, 다음 shot 대기
```

그러나 실제 시스템에서는 어느 한 chip이 bus 오류, EF 고착, 물리 접촉 불량 등으로 slice를 제시간에 완성하지 못할 수 있다. 이 경우 단순 대기 방식은 vdma_frame stream 전체를 멈추게 만든다.

**따라서 assembler에는 다음 정책이 필요하다:**

#### 8.2.1 Per-Lane Timeout

```
파라미터:
  SLICE_TIMEOUT_CLKS : CSR 0x34 (R/W, [7:0], 기본 200). 단위: sys_clk cycles

동작:
  shot 시작 시 lane별 timeout counter 개시
  slice_ready[n]이 올라오면 해당 lane counter 정지
  timeout 도달 전에 slice_ready가 오지 않으면 → 해당 lane을 TIMEOUT 상태로 판정
```

#### 8.2.2 Error-Filled Blank Slice 주입

timeout된 lane에 대해, assembler가 **blank slice**를 자체 생성하여 해당 lane 위치에 삽입한다.

```
blank_slice 내용 (stops_per_chip개 cell, 활성 stop만, Packed Row 기준):
  모든 cell의 hit_slot[0..MAX-1] = 0
  hit_valid = 0x00
  hit_count_actual = 0
  hit_dropped = 0
  slope_vec = 0x00
  error_fill = 1              ← cell_format별 위치 (§13.4 참조: format 0→Byte 18 bit 5, format 1→Byte 32 bit 5)

error_fill = 1의 의미:
  "이 cell은 assembler가 주입한 가짜 데이터이다.
   실제 TDC 측정이 수행되지 않았으므로 모든 hit_slot은 무의미하다."

SW 식별 방법:
  cell.error_fill == 1 → skip (point cloud에 포함하지 않음)
  lane_error_mask로는 frame 단위만 알 수 있지만,
  error_fill로는 shot/row 단위로 정확히 어떤 cell이 무효인지 식별 가능
```

이렇게 하면:
- VDMA line 크기가 항상 일정 (rows_per_face pixels 고정)
- vdma_frame stream이 lane 장애로 멈추지 않음
- 후단 SW가 error_fill 비트로 무효 데이터를 식별 가능 (위치는 cell_format에 의존, §13.4 참조)

#### 8.2.3 Lane Error 기록

```
lane_error_flags[3:0]   : 현재 shot에서 timeout된 lane의 bitmask
lane_error_count[3:0]   : 현재 vdma_frame 내 각 lane의 누적 timeout 횟수

이 정보는:
  (1) vdma_frame header의 lane_error_mask 필드에 기록
  (2) CSR STATUS 레지스터에 반영
  (3) 필요 시 IRQ 발생
```

#### 8.2.4 Assembler 동작 정리

```
매 shot마다:

  1. timeout counter 시작
  2. 대기: slice_ready = active_chip_mask OR timeout 발생
  3. 준비된 lane: 실제 slice 사용
     timeout된 lane: blank slice(error_fill=1) 주입
  4. concat 출력 (lane 순서 유지)
  5. lane_error_flags 갱신
  6. 다음 shot 대기
```

### 8.3 출력

한 VDMA line = rows_per_face pixels × cell_size bytes (가변, 기본 32 × 32 = 1024 bytes)

**정렬 규칙 (VDMA line 내 pixel 순서):**

```
chip 내: stop 0 → stop 1 → ... → stop 7  (stop 번호 오름차순)
chip 간: chip 0 → chip 1 → chip 2 → chip 3  (lane 번호 오름차순)

pixel index = packed sequential (활성 chip/stop만, §1A.6 Packed Row 참조)

  pixel  0: chip0-stop0
  pixel  1: chip0-stop1
  ...
  pixel  7: chip0-stop7
  pixel  8: chip1-stop0
  pixel  9: chip1-stop1
  ...
  pixel 31: chip3-stop7
```

이 순서는 **chip_slice_out**과 **face_packet_assembler** 두 단계에서 보장된다:

1. **chip_slice_out**: cell_builder의 cell[0]~cell[stops_per_chip-1]을 **stop 번호 오름차순**으로 출력
2. **face_packet_assembler**: 4개 chip_slice를 **chip(lane) 번호 오름차순**으로 concat

```
AXI-Stream 출력 (beat 단위, 32-bit tdata 기준):
  beat 0~7:   chip0-stop0 cell (8 beats × 4 bytes = 32 bytes)
  beat 8~15:  chip0-stop1 cell
  ...
  beat 56~63: chip0-stop7 cell
  beat 64~71: chip1-stop0 cell
  ...
  beat 248~255: chip3-stop7 cell  ← tlast = 1 (EOL)
```

**active_chip_mask에 의해 비활성 chip이 있으면:**
비활성 chip의 slice는 건너뛴다 (Packed Row). assembler는 활성 chip의 cell_builder 출력만 수집하여 VDMA line을 구성한다.

### 8.4 SOF 처리

vdma_frame의 첫 번째 data line (shot 0)의 첫 번째 pixel (beat 0)에서 tuser(0) = 1.

단, header line이 있으면 header line의 첫 pixel에서 SOF를 세운다.

### 8.5 Packet-Start Latching 규칙 (중요)

VDMA는 vdma_frame 전체에 걸쳐 `HSIZE`(= 한 line의 바이트 수)가 일관되어야 한다. Packed Row에서 HSIZE는 런타임 가변이므로, **packet_start 시점에 latch하여 해당 vdma_frame이 끝날 때까지 고정**한다.

```
hsize_actual = rows_per_face × CELL_SIZE_BYTES
rows_per_face = count_ones(active_chip_mask_latched) × stops_per_chip_latched

vdma_frame 중간에 CSR을 변경해도 hsize_actual은 다음 packet_start까지 유지된다.
stride = 2048 (MAX_VDMA_HSIZE_BYTES 이상, 항상 고정)
```

**다음 파라미터는 vdma_frame 시작(packet_start) 시점에 latch한다:**

| 파라미터 | latch 시점 | 고정 기간 | 이유 |
|---|---|---|---|
| `active_chip_mask` | packet_start | vdma_frame 전체 | rows_per_face 결정. header에 기록 |
| `stops_per_chip` | packet_start | vdma_frame 전체 | rows_per_face 결정. header에 기록 |
| `rows_per_face` | packet_start (산출) | vdma_frame 전체 | = active_chips × stops_per_chip. hsize_actual 결정 |
| `hsize_actual` | packet_start (산출) | vdma_frame 전체 | = rows_per_face × CELL_SIZE_BYTES. VDMA S2MM_HSIZE |
| `cols_per_face` | packet_start | vdma_frame 전체 | VDMA VSIZE 결정 |
| `cell_size` | — (제너릭 상수) | 항상 고정 | = f(MAX_HITS_PER_STOP, HIT_SLOT_DATA_WIDTH, CELL_FORMAT) |
| `max_hits_per_stop` | — (제너릭 상수) | 항상 고정 | cell 내부 구조 결정 |

**제너릭 상수 vs CSR 구분:**

```
제너릭 상수 (합성 시 결정, CSR 0x70~0x7F에서 R/O Status로 노출):
  N_CHIPS, MAX_STOPS_PER_CHIP, MAX_HITS_PER_STOP, HIT_SLOT_DATA_WIDTH,
  TDATA_WIDTH, CELL_FORMAT

종속 상수 (제너릭으로부터 자동 산출):
  MAX_ROWS_PER_FACE = N_CHIPS × MAX_STOPS_PER_CHIP     (stride/buffer 기준)
  CELL_SIZE_BYTES   = f(MAX_HITS_PER_STOP, HIT_SLOT_DATA_WIDTH, CELL_FORMAT)
  MAX_VDMA_HSIZE_BYTES         = MAX_ROWS_PER_FACE × CELL_SIZE_BYTES

런타임 산출 (packet_start latch):
  rows_per_face  = count_ones(active_chip_mask) × stops_per_chip  (가변)
  hsize_actual   = rows_per_face × CELL_SIZE_BYTES                (가변)
  beats_per_line = hsize_actual / (TDATA_WIDTH / 8)               (가변)

CSR Control (R/W, 런타임 변경 가능, packet_start 시 latch):
  active_chip_mask — 어떤 chip이 활성인가 → rows_per_face에 영향
  stops_per_chip   — 칩당 몇 stop이 활성인가 → rows_per_face에 영향
  cols_per_face    — face당 shot 수 (= VSIZE-1)
```

**Packed Row 불변식:**

```
[INV-9] hsize_actual = rows_per_face × CELL_SIZE_BYTES
[INV-10] rows_per_face ≥ 2 (header 64B ≤ hsize_actual 보장)
         CSR 강제 규칙:
           active_chip_mask = 0x00 → auto-clamp to 0x01 (최소 chip 1개)
           stops_per_chip < 2     → auto-clamp to 2    (최소 stop 2개)
           stops_per_chip > MAX   → auto-clamp to MAX
         HW가 packet_start latch 시점에 auto-clamp을 적용한다.
[INV-11] hsize_actual ≤ MAX_VDMA_HSIZE_BYTES ≤ stride
[INV-12] vdma_frame 진행 중 rows_per_face, hsize_actual 변경 금지
```

**구현:**

```vhdl
generic (
    N_CHIPS            : natural := 4;
    MAX_STOPS_PER_CHIP : natural := 8;
    MAX_HITS_PER_STOP  : natural := 8;
    HIT_SLOT_DATA_WIDTH     : natural := 16;
    TDATA_WIDTH        : natural := 32;
    CELL_FORMAT        : natural := 0     -- 0=Zynq-7000(32B), 1=MPSoC(64B)
);

-- 제너릭 종속 상수 (런타임 불변)
constant MAX_ROWS_PER_FACE : natural := N_CHIPS * MAX_STOPS_PER_CHIP;  -- 32
constant CELL_SIZE_BYTES   : natural := ...;  -- f(MAX_HITS, HIT_SLOT_DATA_WIDTH, CELL_FORMAT)
constant MAX_VDMA_HSIZE_BYTES         : natural := MAX_ROWS_PER_FACE * CELL_SIZE_BYTES;  -- 1024

-- 런타임 산출 (packet_start latch)
signal rows_per_face_reg : natural range 2 to MAX_ROWS_PER_FACE;
signal hsize_actual_reg  : natural range 2*CELL_SIZE_BYTES to MAX_VDMA_HSIZE_BYTES;

process(clk)
    variable v_rows : natural range 2 to MAX_ROWS_PER_FACE;
begin
    if rising_edge(clk) then
        if packet_start = '1' then
            -- CSR 가변 파라미터 latch
            pkt_active_mask    <= csr_active_chip_mask;
            pkt_stops_per_chip <= csr_stops_per_chip;
            pkt_cols           <= csr_cols_per_face;

            -- rows_per_face, hsize_actual 산출 (variable로 동일 cycle 반영)
            v_rows := count_ones(csr_active_chip_mask)
                    * to_integer(csr_stops_per_chip);
            rows_per_face_reg <= v_rows;
            hsize_actual_reg  <= v_rows * CELL_SIZE_BYTES;
            -- ★ signal 대입은 다음 delta에 반영되므로, rows_per_face_reg를
            --   직접 쓰면 이전 packet의 값을 참조하는 버그가 됨.
            --   variable v_rows를 사용하여 동일 clock cycle에서 올바른 값 산출.
        end if;
    end if;
end process;
```

**CSR 변경 시점**: SW가 CSR을 변경해도 현재 진행 중인 vdma_frame에는 영향 없음. 변경된 값은 다음 packet_start에서 latch되어 다음 vdma_frame부터 적용됨.

**header에 latch된 값 기록**: header line에는 latch된 시점의 값이 그대로 기록되므로, DDR에서 읽을 때 header만 보면 해당 vdma_frame의 구조를 정확히 해석할 수 있다.

---

## 제9장. Frame Header

### 9.1 필요성

DDR에 저장된 vdma_frame buffer에는 AXI-Stream control 신호(SOF, EOL)가 남지 않는다. tdata만 남는다. 따라서 vdma_frame을 나중에 SW가 읽을 때 해석할 수 있으려면, metadata가 tdata 안에 포함되어야 한다.

### 9.2 Header Line 구조

header line은 vdma_frame의 첫 번째 VDMA line으로 전송된다. 크기는 data line과 동일 (rows_per_face × cell_size bytes). 내용물은 cell이 아니라 metadata이다.

```
Header line payload (hsize_actual = rows_per_face × cell_size, 기본 1024 bytes):

모든 multi-byte 필드는 little-endian. 32-bit 필드는 4-byte 정렬, 64-bit 필드는 8-byte 정렬.
**예외: magic_word(0x00~0x03)는 바이트 시그니처.** Byte 0=0x54(T), 1=0x44(D), 2=0x43(C), 3=0x47(G). LE 정수로 읽으면 0x47434454이지만, 검증은 바이트 비교 `header[0:4] == b"TDCG"`로 수행한다.

┌─────────────────────────────────────────────────────────────────────────┐
│ Offset │ 이름               │ 크기    │ 분류   │ 설명                   │
├────────┼────────────────────┼─────────┼────────┼────────────────────────┤
│        │ ▶ 식별 (Identity)  │         │        │ 이 header가 무엇인가   │
│  0x00  │ magic_word         │ 4-byte  │ 고정   │ b"TDCG" (바이트 시그니처) │
│  0x04  │ vdma_frame_id      │ 32-bit  │ 자동   │ vdma_frame 순번        │
│  0x08  │ scan_frame_id      │ 32-bit  │ 자동   │ scan_frame 순번        │
│  0x0C  │ face_id            │  8-bit  │ 자동   │ face 번호 (0~N-1)      │
│  0x0D  │ measurement_mode   │  8-bit  │ CSR    │ 0=SINGLE, 1=CONT.     │
│  0x0E  │ active_chip_mask   │  8-bit  │ CSR    │ [3:0] chip bitmask     │
│  0x0F  │ n_faces            │  8-bit  │ CSR    │ 미러 면 수              │
├────────┼────────────────────┼─────────┼────────┼────────────────────────┤
│        │ ▶ 데이터 구조       │         │        │ cell/line 해석 방법    │
│        │   (Structure)      │         │        │                        │
│  0x10  │ rows_per_face      │ 16-bit  │ 자동   │ 실제 값 = active×stops (가변) │
│  0x12  │ cols_per_face      │ 16-bit  │ CSR    │ face당 shot 수          │
│  0x14  │ max_hits_per_stop  │  8-bit  │ R/O    │ 제너릭 (기본 8)         │
│  0x15  │ cell_size_bytes    │  8-bit  │ R/O    │ 종속 상수 (기본 32)     │
│  0x16  │ hit_slot_width     │  8-bit  │ R/O    │ 제너릭 (16 or 17)      │
│  0x17  │ hit_store_mode     │  8-bit  │ CSR    │ 0=RAW, 1=CORR, 2=DIST │
├────────┼────────────────────┼─────────┼────────┼────────────────────────┤
│        │ ▶ 측정 설정         │         │        │ shot/drain 운용 조건   │
│        │   (Measurement)    │         │        │                        │
│  0x18  │ shot_seq_start     │ 32-bit  │ 자동   │ 이 frame의 첫 shot_seq │
│  0x1C  │ bin_resolution_ps  │ 16-bit  │ R/O    │ BIN (ps), 자동 산출    │
│  0x1E  │ dist_scale         │  8-bit  │ CSR    │ 0=1mm~4=2cm            │
│  0x1F  │ drain_mode         │  8-bit  │ CSR    │ 0=SrcGating, 1=Full    │
├────────┼────────────────────┼─────────┼────────┼────────────────────────┤
│        │ ▶ TDC 보정          │         │        │ 시간→거리 변환 상수    │
│        │   (Calibration)    │         │        │                        │
│  0x20  │ start_off1         │ 32-bit  │ CSR    │ Reg5[17:0] StartOff1 (18-bit) │
│  0x24  │ reserved_24        │ 32-bit  │ —      │ (reserved)              │
│  0x28  │ reserved_28        │  8-bit  │ —      │ (reserved)              │
│  0x29  │ n_drain_cap        │  8-bit  │ CSR    │ drain 읽기 제한 값      │
│  0x2A  │ stops_per_chip     │  8-bit  │ CSR    │ 칩당 활성 stop 수       │
│  0x2B  │ pipeline_en        │  8-bit  │ CSR    │ 0=순차, 1=pipeline     │
├────────┼────────────────────┼─────────┼────────┼────────────────────────┤
│        │ ▶ 거리 변환         │         │        │ DISTANCE 모드 전용     │
│        │   (Distance Conv.) │         │        │                        │
│  0x2C  │ k_dist_fixed       │ 32-bit  │ R/O    │ 거리 변환 상수 (자동)   │

├────────┼────────────────────┼─────────┼────────┼────────────────────────┤
│        │ ▶ 시각              │         │        │ 절대 시간 기준          │
│        │   (Timestamp)      │         │        │                        │
│  0x30  │ timestamp_ns       │ 64-bit  │ 자동   │ vdma_frame 시작 시각    │
│        │                    │         │        │ (8-byte 정렬)           │
├────────┼────────────────────┼─────────┼────────┼────────────────────────┤
│        │ ▶ 에러 정보         │         │        │ lane 장애 기록         │
│        │   (Error Status)   │         │        │                        │
│  0x38  │ lane_error_mask    │  8-bit  │ 자동   │ [3:0] timeout lane     │
│  0x39  │ lane_error_cnt0    │  8-bit  │ 자동   │ lane0 timeout 횟수      │
│  0x3A  │ lane_error_cnt1    │  8-bit  │ 자동   │ lane1                   │
│  0x3B  │ lane_error_cnt2    │  8-bit  │ 자동   │ lane2                   │
│  0x3C  │ lane_error_cnt3    │  8-bit  │ 자동   │ lane3                   │
├────────┼────────────────────┼─────────┼────────┼────────────────────────┤
│        │ ▶ HW 정보           │         │        │ HW capability 확인용   │
│        │   (HW Discovery)   │         │        │                        │
│  0x3D  │ n_chips            │  8-bit  │ R/O    │ 제너릭 (기본 4)         │
│  0x3E  │ stops_per_chip_max │  8-bit  │ R/O    │ 제너릭 (기본 8)         │
│  0x3F  │ format_scope       │  8-bit  │ R/O    │ [3:0] cell_format, [7:4] packet_scope │
├────────┼────────────────────┼─────────┼────────┼────────────────────────┤
│        │ ▶ 예약              │         │        │                        │
│  0x40  │ (reserved)         │ ...     │ —      │ 0x00 padding           │
│  ...   │                    │         │ —      │ (끝 = hsize_actual, 최소 64 bytes)     │
└─────────────────────────────────────────────────────────────────────────┘

사용 영역: 0x00~0x3F = 64 bytes (8개 그룹)
padding:   0x40~end = hsize_actual - 64 bytes (hsize_actual ≥ 64 보장)
```

**분류 설명:**

| 분류 | 의미 | 값 결정 주체 |
|---|---|---|
| **고정** | 모든 header에서 동일한 상수 | FPGA 하드코딩 |
| **R/O** | 제너릭/종속 상수. 합성 시 결정, 런타임 불변 | VHDL generic |
| **CSR** | SW가 런타임에 설정한 값. packet_start 시 latch된 시점의 값 | CSR Control R/W |
| **자동** | FPGA가 동작 중 자동 생성/갱신하는 값 | FPGA 내부 로직 |

**SW가 header를 읽는 순서:**
1. **식별**: magic 확인 → vdma_frame/scan_frame/face 번호 파악
2. **데이터 구조**: rows, cols, cell_size로 line/cell 파싱 방법 결정
3. **측정 설정**: hit_store_mode, dist_scale로 hit_slot 해석 방법 결정
4. **TDC 보정**: start_off1, bin_ps로 RAW→거리 변환 (RAW 모드 시)
5. **거리 변환**: k_dist_fixed로 DISTANCE 모드 검증
6. **시각**: timestamp로 point cloud에 절대 시간 부여
7. **에러**: lane_error_mask로 무효 데이터 식별
8. **HW 정보**: n_chips, stops_per_chip_max, active_chip_mask로 Packed Row 해석

**magic_word 설명:**

```
magic_word는 바이트 시그니처 (LE 정수 해석 아님):
  Byte 0 = 0x54 ('T'), Byte 1 = 0x44 ('D'), Byte 2 = 0x43 ('C'), Byte 3 = 0x47 ('G')

SW 검증: if header[0:4] != b"TDCG" then error
```


### 9.3 SW가 DDR에서 vdma_frame을 읽는 방법 (정식 파서)

```python
# ── §9.2 Header offset 상수 ──
OFF_MAGIC           = 0x00  # 32-bit
OFF_VDMA_FRAME_ID   = 0x04  # 32-bit
OFF_SCAN_FRAME_ID   = 0x08  # 32-bit
OFF_FACE_ID         = 0x0C  #  8-bit
OFF_RESERVED_0D     = 0x0D  #  8-bit (reserved)
OFF_ACTIVE_MASK     = 0x0E  #  8-bit
OFF_N_FACES         = 0x0F  #  8-bit
OFF_ROWS            = 0x10  # 16-bit
OFF_COLS            = 0x12  # 16-bit
OFF_MAX_HITS        = 0x14  #  8-bit
OFF_CELL_SIZE       = 0x15  #  8-bit
OFF_HIT_SLOT_WIDTH  = 0x16  #  8-bit
OFF_HIT_STORE_MODE  = 0x17  #  8-bit
OFF_SHOT_SEQ_START  = 0x18  # 32-bit
OFF_BIN_PS          = 0x1C  # 16-bit
OFF_DIST_SCALE      = 0x1E  #  8-bit
OFF_DRAIN_MODE      = 0x1F  #  8-bit
OFF_START_OFF1      = 0x20  # 32-bit (Reg5[17:0], 18-bit)
OFF_RESERVED_24     = 0x24  # 32-bit (reserved)
OFF_RESERVED_28     = 0x28  #  8-bit (reserved)
OFF_N_DRAIN_CAP     = 0x29  #  8-bit
OFF_STOPS_PER_CHIP  = 0x2A  #  8-bit
OFF_PIPELINE_EN     = 0x2B  #  8-bit
OFF_K_DIST          = 0x2C  # 32-bit
OFF_TIMESTAMP       = 0x30  # 64-bit
OFF_LANE_ERR_MASK   = 0x38  #  8-bit
OFF_LANE_ERR_CNT0   = 0x39  #  8-bit
OFF_LANE_ERR_CNT1   = 0x3A  #  8-bit
OFF_LANE_ERR_CNT2   = 0x3B  #  8-bit
OFF_LANE_ERR_CNT3   = 0x3C  #  8-bit
OFF_N_CHIPS         = 0x3D  #  8-bit
OFF_STOPS_MAX       = 0x3E  #  8-bit
OFF_FORMAT_SCOPE    = 0x3F  #  8-bit: [3:0] cell_format, [7:4] packet_scope

SCALE_TO_M = [0.001, 0.002, 0.005, 0.01, 0.02]  # dist_scale별 m/count
c = 299792458.0  # m/s

# ── 1. header 읽기 (vdma_frame 첫 line, PACKET_SCOPE=0 전용) ──
base   = vdma_frame_buffer_address
stride = vdma_stride
hdr    = read_memory(base, stride)    # header line = 1 VDMA line

# ▶ 식별 (Identity)
# magic 검증 (바이트 시그니처)
assert hdr[0:4] == b"TDCG", "bad magic"
vdma_frame_id   = u32(hdr, OFF_VDMA_FRAME_ID)
scan_frame_id   = u32(hdr, OFF_SCAN_FRAME_ID)
face_id         = u8(hdr, OFF_FACE_ID)
active_chip_mask = u8(hdr, OFF_ACTIVE_MASK)     # [3:0] chip bitmask
n_faces         = u8(hdr, OFF_N_FACES)

# ▶ 데이터 구조 (Structure)
rows            = u16(hdr, OFF_ROWS)            # rows_per_face (actual, packet_start latch)
cols            = u16(hdr, OFF_COLS)            # cols_per_face (CSR)
max_hits        = u8(hdr, OFF_MAX_HITS)         # max_hits_per_stop (제너릭)
cell_size       = u8(hdr, OFF_CELL_SIZE)        # cell_size_bytes (종속 상수)
hit_slot_width  = u8(hdr, OFF_HIT_SLOT_WIDTH)   # 16 or 17
hit_store_mode  = u8(hdr, OFF_HIT_STORE_MODE)   # 0=RAW, 1=CORR, 2=DIST

# ▶ 측정 설정 (Measurement)
shot_seq_start  = u32(hdr, OFF_SHOT_SEQ_START)
bin_ps          = u16(hdr, OFF_BIN_PS)          # bin_resolution_ps (자동 산출)
dist_scale      = u8(hdr, OFF_DIST_SCALE)
drain_mode      = u8(hdr, OFF_DRAIN_MODE)

# ▶ TDC 보정 (Calibration)
start_off1      = u32(hdr, OFF_START_OFF1) & 0x3FFFF  # 18-bit
n_drain_cap     = u8(hdr, OFF_N_DRAIN_CAP)
stops_per_chip  = u8(hdr, OFF_STOPS_PER_CHIP)   # 칩당 활성 stop 수
pipeline_en     = u8(hdr, OFF_PIPELINE_EN)

# ▶ 거리 변환 (Distance)
k_dist_fixed    = u32(hdr, OFF_K_DIST)

# ▶ 시각 (Timestamp)
timestamp_ns    = u64(hdr, OFF_TIMESTAMP)

# ▶ 에러 (Error)
lane_err_mask   = u8(hdr, OFF_LANE_ERR_MASK)    # [3:0] timeout lane bitmask
lane_err_cnt    = [
    u8(hdr, OFF_LANE_ERR_CNT0),                  # lane0 누적 timeout 횟수
    u8(hdr, OFF_LANE_ERR_CNT1),                  # lane1
    u8(hdr, OFF_LANE_ERR_CNT2),                  # lane2
    u8(hdr, OFF_LANE_ERR_CNT3),                  # lane3
]

# ▶ HW 정보 (Discovery)
n_chips         = u8(hdr, OFF_N_CHIPS)           # 제너릭 (기본 4)
stops_max       = u8(hdr, OFF_STOPS_MAX)         # MAX_STOPS_PER_CHIP (기본 8)
format_scope    = u8(hdr, OFF_FORMAT_SCOPE)      # 0x3F: packed byte
cell_format     = format_scope & 0x0F            #   [3:0] 0=Zynq-7000, 1=MPSoC
packet_scope    = (format_scope >> 4) & 0x0F     #   [7:4] 0=face (Phase 1 only)

# ── sanity check ──
assert cell_size > 0 and cols > 0 and rows > 0

# ── Packed Row: active chip 리스트 구성 ──
active_chips = [i for i in range(n_chips) if (active_chip_mask >> i) & 1]
assert rows == len(active_chips) * stops_per_chip, "Packed Row rows mismatch"

# ── 2. data line 순차 읽기 ──
for col in range(cols):
    line_addr = base + stride * (col + 1)    # +1: header가 line 0
    line_data = read_memory(line_addr, cell_size * rows)

    for row in range(rows):
        cell_bytes = line_data[row * cell_size : (row + 1) * cell_size]

        # Packed Row 해석 (§1A.6)
        # active_chips를 먼저 구성 (header 읽기 직후 1회)
        # active_chips = [i for i in range(n_chips) if (active_chip_mask >> i) & 1]
        chip_idx = row // stops_per_chip         # active_chips 리스트 내 인덱스
        chip_id  = active_chips[chip_idx]         # 실제 chip ID (0~3)
        stop_id  = row % stops_per_chip

        # 유효성 검사 1: lane 에러 여부 (Packed Row에서 유일하게 필요한 row-level 검사)
        if (lane_err_mask >> chip_id) & 1:
            continue                            # timeout 발생 lane → 데이터 신뢰 불가

        # cell 파싱 (§13.4 바이트 레이아웃 참조)
        cell = parse_cell(cell_bytes, cell_format, max_hits)

        # 유효성 검사 4: timeout blank cell 여부
        if cell.error_fill:
            continue                            # assembler가 주입한 blank → skip

        for hit in range(max_hits):
            if not ((cell.hit_valid >> hit) & 1):  # bitmask bit test
                continue

            # ── HIT_STORE_MODE에 따른 거리 변환 ──
            if hit_store_mode == 2:             # DISTANCE
                distance_m = cell.hit_slot[hit] * SCALE_TO_M[dist_scale]

            elif hit_store_mode == 1:           # CORRECTED
                corrected  = cell.hit_slot[hit]
                distance_m = corrected * bin_ps * 1e-12 * c / 2

            elif hit_store_mode == 0:           # RAW
                raw_hit    = cell.hit_slot[hit]
                time_ps    = (raw_hit - start_off1) * bin_ps
                distance_m = time_ps * 1e-12 * c / 2

            # ── point cloud 생성 (application layer, 본 문서 범위 외) ──
            # face_start_angle, angle_step: 미러 encoder / laser_ctrl에서 제공 (별도 정의)
            # vertical_pitch: 광학 설계에서 결정 (별도 정의)
            # 아래는 참고 예시이며, 정확한 공식은 시스템 통합 문서에서 정의한다.
            angle_h = face_start_angle + col * angle_step     # 수평각 (미정의)
            angle_v = row * vertical_pitch                    # 수직각 (미정의)
            slope   = (cell.slope_vec >> hit) & 1  # bitmask bit test

            point_cloud.append(distance_m, angle_h, angle_v, slope, hit)
```

**주의사항:**
- 모든 header 필드 접근은 §9.2의 offset 정의와 정확히 일치해야 한다 [원칙 1]
- 모든 offset 상수를 정의하고, 모든 header 필드를 읽은 후에 사용해야 한다
- `hit_store_mode`에 따라 hit_slot 해석이 완전히 달라진다: DISTANCE면 즉시 거리, RAW면 full formula
- DISTANCE 모드에서 `start_off1`을 다시 적용하면 이중 보정 오류
- `lane_err_mask`의 해당 bit가 1인 chip의 데이터는 신뢰할 수 없으므로 skip
- Packed Row: 모든 row가 활성. `lane_err_mask`로 timeout chip만 skip

---

## 제9A장. CSR 레지스터 맵 (tdc_gpx_csr)

### 9A.1 레지스터 분류

| 분류 | 접근 | 의미 | 값 결정 시점 |
|---|---|---|---|
| **Control (R/W)** | SW 읽기/쓰기 | SW가 런타임에 변경하는 운용 파라미터 | 런타임 (packet_start 시 latch) |
| **Status (R/O)** | SW 읽기만 | FPGA 제너릭/종속 상수를 SW에게 노출. HW capability discovery 용도 | 합성 시 (런타임 불변) |
| **Status-Live (R/O)** | SW 읽기만 | FPGA가 실시간 갱신하는 상태 정보 | 실시간 |

**Status(R/O) 레지스터가 필요한 이유:**
제너릭 상수(N_CHIPS, MAX_HITS_PER_STOP 등)는 합성 시 결정되며, 같은 bitstream이라도 다른 제너릭으로 빌드할 수 있다. SW는 이 레지스터를 읽어 "이 FPGA가 어떤 설정으로 빌드되었는가"를 런타임에 확인하고, header 파싱/point cloud 변환에 사용한다. write해도 값이 변경되지 않는다.

### 9A.2 레지스터 맵

```
Base: AXI-Lite base address (예: 0x4000_0000)
모든 레지스터는 32-bit 정렬 (AXI-Lite 표준). 유효 비트 외 상위 비트는 0.

┌────────┬────────────────────────┬───────┬────────┬──────────────────────────────────┐
│ Offset │ 이름                    │ 크기  │ 분류   │ 설명                              │
├────────┼────────────────────────┼───────┼────────┼──────────────────────────────────┤
│        │ === Control (R/W) ===  │       │        │ 런타임 운용 파라미터               │
│ 0x00   │ ACTIVE_CHIP_MASK       │ 32-bit│ R/W    │ [3:0] chip 활성 bitmask. ≥1 bit set 필수 [INV-10] │
│ 0x04   │ STOPS_PER_CHIP         │ 32-bit│ R/W    │ [7:0] 칩당 활성 stop 수. 2 ≤ S ≤ MAX [INV-10]    │
│ 0x08   │ COLS_PER_FACE          │ 32-bit│ R/W    │ [15:0] face당 shot 수 (기본 2400)  │
│ 0x0C   │ PACKET_SCOPE           │ 32-bit│ R/W    │ [7:0] 0=face 단위, 1=scan_frame 단위 │
│ 0x10   │ HIT_STORE_MODE         │ 32-bit│ R/W    │ [7:0] 0=RAW, 1=CORRECTED, 2=DISTANCE │
│ 0x14   │ DIST_SCALE             │ 32-bit│ R/W    │ [7:0] 0=1mm, 1=2mm, 2=5mm, 3=1cm, 4=2cm │
│ 0x18   │ DRAIN_MODE             │ 32-bit│ R/W    │ [7:0] 0=SourceGating, 1=FullDrain+LF │
│ 0x1C   │ N_DRAIN_CAP            │ 32-bit│ R/W    │ [7:0] Capped Drain 읽기 제한 (0=무제한) │
│ 0x20   │ PIPELINE_EN            │ 32-bit│ R/W    │ [7:0] 0=순차, 1=double-buffer pipeline │
│ 0x24   │ N_FACES                │ 32-bit│ R/W    │ [7:0] 폴리곤 미러 면 수 (기본 5)    │
│ 0x28   │ BUS_CLK_DIV            │ 32-bit│ R/W    │ [7:0] bus_clk 정수 분주기 (기본 1)  │
│ 0x2C   │ BUS_TICKS              │ 32-bit│ R/W    │ [7:0] ticks/transaction (3~6, 기본 5) │
│ 0x30   │ STOPDIS_OVERRIDE       │ 32-bit│ R/W    │ [3:0] SW StopDis 직접 제어 (디버그) │
│        │                        │       │        │ [4] override enable                │
│ 0x34   │ SLICE_TIMEOUT_CLKS     │ 32-bit│ R/W    │ [7:0] lane timeout (sys_clk cycles, 기본 200) │
│ 0x38   │ reserved               │ 32-bit│        │                                    │
│ 0x3C   │ reserved               │ 32-bit│        │                                    │
├────────┼────────────────────────┼───────┼────────┼──────────────────────────────────┤
│        │ === TDC 설정 (R/W) === │       │        │ cfg_image → 칩 write              │
│ 0x40   │ START_OFF1             │ 32-bit│ R/W    │ [17:0] Reg5 StartOff1 (18-bit)    │
│ 0x44   │ CFG_REG7               │ 32-bit│ R/W    │ [7:0] HSDiv (8-bit, 1~255)        │
│        │                        │       │        │ [10:8] RefClkDiv (3-bit, 0~7)     │
│        │                        │       │        │ [27:15] MTimer (13-bit, 0~8191)   │
│        │                        │       │        │ write 시 bin_ps, K_DIST 자동 갱신  │
│ 0x48   │ reserved               │ 32-bit│        │                                    │
│ 0x4C   │ reserved               │ 32-bit│        │                                    │
├────────┼────────────────────────┼───────┼────────┼──────────────────────────────────┤
│        │ === cfg_image (R/W) == │       │        │ TDC-GPX 전체 레지스터 이미지       │
│ 0x80   │ CFG_IMAGE_BASE         │       │ R/W    │ Reg0~Reg14 × 32-bit each          │
│ ~0xBC  │                        │       │        │ (16 regs × 4 bytes = 64 bytes)    │
├────────┼────────────────────────┼───────┼────────┼──────────────────────────────────┤
│        │ === Command (R/W) ===  │       │        │ 명령                              │
│ 0xC0   │ COMMAND                │ 32-bit│ R/W    │ [0] start (1=시작)                │
│        │                        │       │        │ [1] stop (1=정지)                 │
│        │                        │       │        │ [2] soft_reset                    │
│        │                        │       │        │ [3] cfg_write_trigger             │
│ 0xC4   │ reserved               │ 32-bit│        │                                    │
│ ~0xFC  │                        │       │        │                                    │
├────────┼────────────────────────┼───────┼────────┼──────────────────────────────────┤
│        │ === Status (R/O) ===   │       │        │ 제너릭 상수 노출 (HW discovery)    │
│ 0x100  │ HW_VERSION             │ 32-bit│ R/O    │ HW 버전 (major[31:16], minor[15:0])│
│ 0x104  │ HW_CONFIG              │ 32-bit│ R/O    │ [3:0] N_CHIPS                     │
│        │                        │       │        │ [7:4] MAX_STOPS_PER_CHIP          │
│        │                        │       │        │ [11:8] MAX_HITS_PER_STOP          │
│        │                        │       │        │ [16:12] HIT_SLOT_DATA_WIDTH       │
│        │                        │       │        │ [24:17] TDATA_WIDTH/8             │
│        │                        │       │        │ [27:25] CELL_FORMAT               │
│ 0x108  │ MAX_ROWS_PER_FACE      │ 32-bit│ R/O    │ [15:0] = N_CHIPS × MAX_STOPS_PER_CHIP (32) │
│ 0x10C  │ CELL_SIZE_BYTES        │ 32-bit│ R/O    │ [15:0] = f(MAX_HITS, HIT_SLOT_DATA_WIDTH, CELL_FORMAT) │
│ 0x110  │ MAX_VDMA_HSIZE_BYTES   │ 32-bit│ R/O    │ = MAX_ROWS_PER_FACE × CELL_SIZE_BYTES │
├────────┼────────────────────────┼───────┼────────┼──────────────────────────────────┤
│        │ === Status-Live (R/O)==│       │        │ 실시간 상태                        │
│ 0x140  │ STATUS                 │ 32-bit│ R/O    │ [0] busy                          │
│        │                        │       │        │ [1] pipeline_overrun              │
│        │                        │       │        │ [2] bin_mismatch                  │
│        │                        │       │        │ [7:4] lane_error_mask             │
│ 0x144  │ SHOT_SEQ_CURRENT       │ 32-bit│ R/O    │ 현재 shot_seq 값                   │
│ 0x148  │ VDMA_FRAME_COUNT       │ 32-bit│ R/O    │ 완료된 vdma_frame 수               │
│ 0x14C  │ ERROR_COUNT            │ 32-bit│ R/O    │ 누적 에러 횟수                     │
│ 0x150  │ BIN_RESOLUTION_PS      │ 32-bit│ R/O    │ [15:0] Reg7로부터 자동 산출        │
│ 0x154  │ K_DIST_FIXED           │ 32-bit│ R/O    │ 거리 변환 상수 (자동 산출)          │
└────────┴────────────────────────┴───────┴────────┴──────────────────────────────────┘

주소 영역 요약:
  0x00~0x3C  : Control (R/W) — 운용 파라미터
  0x40~0x4C  : TDC 설정 (R/W) — 칩 설정/보정
  0x80~0xBC  : cfg_image (R/W) — TDC-GPX 레지스터 이미지
  0xC0~0xFC  : Command (R/W) — start/stop/reset
  0x100~0x110: Status (R/O) — 제너릭 상수 노출
  0x140~0x154: Status-Live (R/O) — 실시간 상태 + 자동 산출 값
  총 주소 공간: 0x158 (344 bytes), addr_width = 9 bits (512-byte block)
```

### 9A.3 SW 사용 예시

```c
// 1. 런타임 설정 (Control, 0x00~0x3C)
csr_write32(0x00, 0x0F);          // ACTIVE_CHIP_MASK: 4개 chip 모두 활성
csr_write32(0x04, 8);             // STOPS_PER_CHIP: 8 stops/chip
csr_write32(0x08, 2400);          // COLS_PER_FACE: 2400 shots
csr_write32(0x0C, 0);             // PACKET_SCOPE: face 단위
csr_write32(0x10, 2);             // HIT_STORE_MODE: DISTANCE
csr_write32(0x14, 3);             // DIST_SCALE: 1cm

// 2. TDC PLL 설정 (0x40~) → bin_ps, K_DIST 자동 갱신
csr_write32(0x44, (ref_clk_div << 8) | hs_div);  // CFG_REG7
// → FPGA가 BIN_RESOLUTION_PS(0x150)와 K_DIST_FIXED(0x154)를 자동 산출

// 3. HW capability discovery (Status R/O, 0x100~ — 제너릭 상수, 합성 시 결정)
uint32_t hw_config = csr_read32(0x104);           // HW_CONFIG
int n_chips            = (hw_config >> 0) & 0xF;
int stops_per_chip_max = (hw_config >> 4) & 0xF;
int max_hits           = (hw_config >> 8) & 0xF;
int rows_per_face_max  = csr_read32(0x108) & 0xFFFF; // MAX_ROWS_PER_FACE (R/O, 기본 32)
int cell_size_bytes    = csr_read32(0x10C) & 0xFFFF; // CELL_SIZE_BYTES (R/O)
int hsize_max          = csr_read32(0x110);        // MAX_VDMA_HSIZE_BYTES (R/O, stride 기준)
// ★ 실제 rows_per_face(actual)와 hsize_actual은 header의 OFF_ROWS에서 읽는다.
//    CSR 0x108/0x110는 합성 시 결정된 최댓값(MAX)이지 현재 actual이 아님.

// 4. 실시간 상태 확인 (Status-Live, 0x140~)
uint32_t status = csr_read32(0x140);              // STATUS
uint16_t bin_ps = csr_read32(0x150) & 0xFFFF;     // 자동 산출된 bin_ps
uint32_t k_dist = csr_read(0x94);                 // 자동 산출된 K_DIST

// 5. 시작
csr_write(0x60, 0x01);          // COMMAND: start
```

---

## 제10장. VDMA 설정

### 10.1 필요한 VDMA 레지스터 설정

```
S2MM_HSIZE   = pixels_per_line × cell_size_bytes
             = rows_per_face × cell_size (가변, 기본 32 × 32 = 1024 bytes)

S2MM_VSIZE   = cols_per_face + 1          (header 포함)
             = 2401

S2MM_FRMDLY_STRIDE = stride (≥ hsize, alignment 맞춤)
                   = 2048 bytes (2KB 정렬 권장, §5.2.5 참조)

S2MM_FRAMEDELAY = 0

Packet buffer count = 2 또는 3 (triple buffering 권장)

vdma_frame DDR 사용량 = stride × vsize = 2048 × 2401 = 4,917,248 bytes ≈ 4.69 MB
triple buffer 총량 = 3 × 4.69 MB ≈ 14.07 MB
```

### 10.2 AXI-Stream 인터페이스 설정

**설계 기준: Zynq-7000, tdata_width = 32-bit**

```
tdata_width   = 32 bits (Zynq-7000 기본)
tdata_bytes   = 4 bytes
cell_size     = 32 bytes

beats_per_pixel = cell_size / tdata_bytes = 32 / 4 = 8 beats
beats_per_line  = pixels_per_line × beats_per_pixel = 32 × 8 = 256 beats
tlast는 256번째 beat에서 1 (line의 마지막 beat)
```

`TDATA_WIDTH`는 generic으로 설계하여 플랫폼 변경 시 재합성만으로 대응한다 (§5.2.4 참조).

### 10.3 플랫폼별 tdata 폭 설정

| 플랫폼 | tdata_width | beats/pixel | beats/line (rows_per_face=32) | 비고 |
|---|---|---|---|---|
| **Zynq-7000 (기준)** | 32-bit | 8 | 256 | Phase 1 기본 |
| Zynq-7000 HP 64-bit | 64-bit | 4 | 128 | HP 포트 활용 시 |
| Zynq MPSoC | 128-bit | 2 | 64 | 향후 확장 |

**Phase 1 권장: 32-bit.** Zynq-7000 GP/HP 포트에서 바로 동작하고, 이후 generic 변경만으로 64/128-bit 확장 가능.

### 10.4 Backpressure 처리

VDMA가 DDR write를 완료하지 못하면 tready를 내리고, stream이 멈춘다. 이때 TDC-GPX 칩은 계속 hit를 수집하고 있으므로, FPGA 내부에 buffer가 필요하다.

```
완충 구조:

cell_builder 내 cell buffer (1 shot분)
  + chip_slice FIFO (수 shot분)
  + axis_data_fifo (assembler와 VDMA 사이, 수 line분)

총 버퍼 = axis_data_fifo depth × line_size
권장: axis_data_fifo depth ≥ 16 lines = 16 × 1024 = 16 KB
```

---

## 제11장. 타이밍 예산

### 11.1 한 Shot의 시간 예산

가정:
- 폴리곤 미러 회전 속도 = 10 rps (600 rpm)
- face 수 = 5
- face당 광학 화각 = 120도 (물리 60도 × 2, 반사법칙)
- cols_per_face = 2400

```
face 지속 시간 = 1 / (10 × 5) × 1000 = 20 ms
shot 간격 = 20 ms / 2400 = 8.33 μs
```

### 11.2 왕복 비행시간 참조표

빛의 왕복 시간: **6.67 ns/m** (= 2 / c = 2 / 3×10⁸)

| 편도 거리 | 왕복 비행시간 | 비고 |
|---|---|---|
| 75 m | 0.5 μs | 근거리 |
| 150 m | 1.0 μs | 일반 LiDAR |
| 400 m | 2.67 μs | 장거리 LiDAR |
| 750 m | 5.0 μs | |
| 1500 m | 10.0 μs | TDC-GPX I-Mode 측정 범위 한계 부근 |

### 11.2A 타이밍 독립변수와 종속변수

#### 11.2A.1 독립변수 → 종속변수 의존성 체인

```
타이밍에 영향을 주는 독립변수 (변경 가능):

  [제너릭]                            [CSR]                         [물리]
  N_CHIPS ──────┐                     cols_per_face ──┐             distance ──┐
  MAX_STOPS_PER_CHIP ─┤               N_FACES ────────┤                        │
  MAX_HITS_PER_STOP ──┤               BUS_CLK_DIV ──┐ │             mirror_rps │
  HIT_SLOT_DATA_WIDTH ─────┤                              │ │                       │
  TDATA_WIDTH ────────┤                              │ │                       │
                      │                              │ │                       │
                      ▼                              ▼ ▼                       ▼
               ┌──────────────┐             ┌──────────────┐         ┌──────────────┐
               │ 구조 종속변수  │             │ 시간 종속변수  │         │ 물리 종속변수  │
               ├──────────────┤             ├──────────────┤         ├──────────────┤
               │ROWS_PER_FACE_MAX│             │T_shot        │         │T_roundtrip   │
               │CELL_SIZE_BYTES│            │T_read        │         │              │
               │HSIZE_MAX     │             │              │         │              │
               │BEATS_PER_LINE│             │              │         │              │
               └──────┬───────┘             └──────┬───────┘         └──────┬───────┘
                      │                            │                        │
                      ▼                            ▼                        ▼
               ┌─────────────────────────────────────────────────────────────────┐
               │                    최종 타이밍 종속변수                           │
               ├─────────────────────────────────────────────────────────────────┤
               │ T_drain   = f(MAX_HITS_PER_STOP, BUS_CLK_DIV)                  │
               │ T_cell    = f(CELL_SIZE_BYTES, 내부 pipeline)                    │
               │ T_vdma    = f(BEATS_PER_LINE, AXI_CLK)                          │
               │ T_service = T_drain + T_cell + T_vdma + T_alu                   │
               │ T_critical = T_roundtrip + T_drain + T_alu                      │
               │ T_total   = T_roundtrip + T_service                             │
               └─────────────────────────────────────────────────────────────────┘
```

#### 11.2A.2 독립변수별 타이밍 수식

**T_shot (shot 간격):**

```
T_face     = 1 / (mirror_rps × N_FACES)
T_shot     = T_face / cols_per_face

기본값: 1/(10×5) / 2400 = 20ms / 2400 = 8.33 μs

영향을 주는 독립변수:
  mirror_rps ↑  → T_shot ↓ (빨라짐, 마진 감소)
  N_FACES ↑     → T_shot ↓
  cols_per_face ↑ → T_shot ↓
```

**T_drain (IFIFO drain 시간):**

```
전략 C (Source Gating):
  worst-case: N_drain_max = stops_per_chip × MAX_HITS_PER_STOP (칩당)
              양쪽 IFIFO 합산 최대 = stops_per_chip × MAX_HITS_PER_STOP
              (기본 8 × 8 = 64. Packed Row에서 stops_per_chip < 8이면 감소)
  T_read     = BUS_TICKS × BUS_CLK_DIV × bus_clk_period  (= 5×1×5ns = 25ns @ 200MHz 기본)
  T_drain    = N_drain_max × T_read + EF_overhead
             = 64 × 25ns + ~100ns = ~1.7 μs (worst-case, stops_per_chip=8)

  actual: stops_per_chip=4이면 N_drain = 4×8 = 32 → T_drain ≈ 0.9μs

영향을 주는 변수:
  stops_per_chip ↑ → N_drain ↑ → T_drain ↑ (CSR, packet_start latch)
  MAX_HITS_PER_STOP ↑ → N_drain ↑ → T_drain ↑ (제너릭)
  BUS_CLK_DIV ↑ → T_drain ↑ (bus 느려짐)
```

**T_vdma (VDMA line 전송 시간):**

```
beats_per_line = rows_per_face × CELL_SIZE_BYTES / TDATA_BYTES  (actual, 가변)
T_vdma         = beats_per_line / AXI_CLK

worst-case (기본값): (32 × 32 / 4) / 200MHz = 256 / 200MHz = 1.28 μs
actual 예시: rows_per_face=8 → (8 × 32 / 4) / 200MHz = 64 / 200MHz = 0.32 μs

영향을 주는 변수:
  rows_per_face ↑ → beats_per_line ↑ → T_vdma ↑ (CSR, packet_start latch)
  CELL_SIZE_BYTES ↑ → beats_per_line ↑ → T_vdma ↑ (제너릭)
  TDATA_WIDTH ↑ → TDATA_BYTES ↑ → beats_per_line ↓ → T_vdma ↓ (빨라짐)

★ 본 문서의 타이밍 예산 (T_service=4.03μs 등)은 worst-case 기본값
  (rows=32, stops=8, cell=32B, tdata=32bit) 기준이다.
  Packed Row에서 활성 chip/stop이 적으면 T_drain, T_vdma 모두 줄어들어 마진이 늘어난다.
```

**T_roundtrip (빛의 왕복 시간):**

```
T_roundtrip = 2 × distance / c = distance × 6.67 ns/m

독립변수: distance (운용 거리, 물리적으로 결정)
```

**T_service (순차 실행 총 서비스 시간):**

```
T_service = T_drain + T_cell + T_vdma + T_alu
          = 1.7 + 1.0 + 1.28 + 0.05 = 4.03 μs (기본값)
```

**T_critical (pipeline 적용 시 critical path):**

```
T_critical = T_roundtrip + T_drain + T_alu
           = T_roundtrip + 1.745 μs (기본값)
```

#### 11.2A.3 판정 공식

```
순차 실행 가능 조건:
  T_roundtrip + T_service < T_shot
  → distance < (T_shot - T_service) / 6.67ns/m

pipeline 적용 가능 조건:
  T_roundtrip + T_drain + T_alu < T_shot
  → distance < (T_shot - T_drain - T_alu) / 6.67ns/m
```

#### 11.2A.4 독립변수 변경 시 구간별 타이밍 점검표

아래 표는 독립변수를 하나씩 변경했을 때 종속 타이밍과 최대 거리가 어떻게 변하는지 보여준다.

**기준 설정 (옵션 A 기본):**

```
N_CHIPS=4, MAX_STOPS_PER_CHIP=8, MAX_HITS=8, HIT_SLOT_DATA_WIDTH=16
TDATA_WIDTH=32, BUS_CLK_DIV=1, N_FACES=5, cols_per_face=2400
mirror_rps=10

→ T_shot=8.33μs, T_drain=1.7μs, T_vdma=1.28μs, T_service=4.03μs
→ 순차 최대 ~650m, pipeline 최대 ~987m
```

**① cols_per_face 변경 (T_shot에 영향):**

| cols_per_face | T_shot | T_service | 순차 최대 | pipeline 최대 |
|---|---|---|---|---|
| 1200 | 16.67 μs | 4.03 μs | 1895m | >TDC 한계 |
| 1800 | 11.11 μs | 4.03 μs | 1062m | 1405m |
| **2400** | **8.33 μs** | **4.03 μs** | **644m** | **987m** |
| 3600 | 5.56 μs | 4.03 μs | 229m | 572m |

**② N_FACES 변경 (T_shot에 영향):**

| N_FACES | T_face | T_shot (cols=2400) | 순차 최대 | pipeline 최대 |
|---|---|---|---|---|
| **5** | 20 ms | **8.33 μs** | **644m** | **987m** |
| 4 | 25 ms | 10.42 μs | 958m | 1301m |
| 3 | 33.3 ms | 13.89 μs | 1478m | >TDC 한계 |

**③ MAX_HITS_PER_STOP 변경 (T_drain, T_vdma에 영향):**

| MAX_HITS | cell_size | T_drain | T_vdma | T_service | 순차 최대 | pipeline 최대 |
|---|---|---|---|---|---|---|
| 4 | 16 B | 0.9 μs | 0.64 μs | 2.59 μs | 860m | 1040m |
| **8** | **32 B** | **1.7 μs** | **1.28 μs** | **4.03 μs** | **644m** | **987m** |
| 16 | 64 B | 3.3 μs | 2.56 μs | 6.91 μs | 213m | 736m |

**④ BUS_CLK_DIV 변경 (T_drain에 영향):**

| BUS_CLK_DIV | T_read | T_drain (64 entries) | T_service | 순차 최대 | pipeline 최대 |
|---|---|---|---|---|---|
| **1** | **25 ns** | **1.7 μs** | **4.03 μs** | **644m** | **987m** |
| 2 | 50 ns | 3.3 μs | 5.63 μs | 405m | 748m |
| 3 | 75 ns | 4.9 μs | 7.23 μs | 165m | 508m |

**⑤ TDATA_WIDTH 변경 (T_vdma에 영향):**

| TDATA_WIDTH | BEATS_PER_LINE | T_vdma | T_service | 순차 최대 | pipeline 최대 |
|---|---|---|---|---|---|
| **32** | **256** | **1.28 μs** | **4.03 μs** | **644m** | **987m** |
| 64 | 128 | 0.64 μs | 3.39 μs | 740m | 987m |
| 128 | 64 | 0.32 μs | 3.07 μs | 789m | 987m |

**⑥ 플랫폼 전환 (Zynq-7000 → MPSoC):**

| 항목 | Zynq-7000 | Zynq MPSoC |
|---|---|---|
| HIT_SLOT_DATA_WIDTH | 16 | 17 |
| TDATA_WIDTH | 32 | 128 |
| CELL_SIZE_BYTES | 32 | 64 |
| BEATS_PER_LINE | 256 | 128 (64B/16B per beat) |
| T_vdma | 1.28 μs | 0.64 μs |
| T_service | 4.03 μs | 3.39 μs |
| 순차 최대 | 644m | 740m |
| pipeline 최대 | 987m | 987m (T_drain이 지배) |

**관찰:** pipeline 모드에서 최대 거리는 T_drain + T_alu에 의해 결정되며, T_vdma는 overlap되므로 영향 없다. 따라서 TDATA_WIDTH를 늘려도 pipeline 최대 거리는 변하지 않는다. pipeline 최대 거리를 늘리려면 **T_drain을 줄이거나 T_shot을 늘려야 한다.**

#### 11.2A.5 독립변수 감도 요약

| 독립변수 | 영향 받는 종속변수 | 순차 최대 | pipeline 최대 | 감도 |
|---|---|---|---|---|
| **cols_per_face** | T_shot | **강** | **강** | T_shot 직접 결정 |
| **N_FACES** | T_shot | **강** | **강** | T_shot 직접 결정 |
| **MAX_HITS_PER_STOP** | T_drain, T_vdma | 중 | **강** | T_drain이 pipeline 한계 결정 |
| **BUS_CLK_DIV** | T_drain | 중 | **강** | DIV=1 필수 |
| **TDATA_WIDTH** | T_vdma | 약 | 없음 | pipeline에서는 overlap |
| **distance** | T_roundtrip | **강** | **강** | 물리 법칙 |
| mirror_rps | T_shot | **강** | **강** | 기계적 제약 |
| N_CHIPS | T_vdma | 약 | 없음 | 4 고정 시 무관 |

### 11.3 DRAIN_READ_COUNT vs MAX_HITS_PER_STOP (핵심 구분)

**이 둘은 다른 개념이며, 반드시 분리해서 다뤄야 한다.**

| 개념 | 정의 | 결정 주체 | 값 범위 |
|---|---|---|---|
| **MAX_HITS_PER_STOP** | cell에 저장할 hit의 최대 수. 초과 hit는 cell에서 버림 | **제너릭 (합성 시 고정)** | 기본 8 |
| **DRAIN_READ_COUNT** | IFIFO에서 실제로 읽어야 하는 raw entry 수 | 칩 내부 hit 수에 의존 | 0~256 (최악) |

**왜 다른가:**

TDC-GPX I-Mode: 채널당 32-fold multi-hit, IFIFO 각 256-stage.

MAX_HITS_PER_STOP = 8이어도, 칩 내부에 IFIFO1(4ch×32=128) + IFIFO2(4ch×32=128) = 256개가 쌓일 수 있다. MAX_HITS는 "cell에 몇 개 남기느냐"일 뿐, 칩에서 읽어야 하는 횟수를 줄여주지 않는다. extra hit를 discard하더라도, 읽는 행위 자체는 필요하다.

**IFIFO 내부 구조와 용량:**

```
TDC-GPX I-Mode IFIFO 구성:

  IFIFO1 (Reg8): TStop1~4 (stop 0~3) → 4 stop 채널 공유
  IFIFO2 (Reg9): TStop5~8 (stop 4~7) → 4 stop 채널 공유

각 IFIFO:
  depth       = 256-stage
  stop 채널 수 = 4 (MAX_STOPS_PER_CHIP / 2)
  채널당 multi-hit = 최대 32 (TDC-GPX I-Mode 하드웨어 한계)

IFIFO당 최대 entries = 4 stops × 32 hits = 128 entries
양쪽 합산 최대        = 128 + 128 = 256 entries
```

**전략별 IFIFO 점유:**

| 전략 | stop당 hit 제한 | IFIFO당 최대 | depth(256) 대비 | 양쪽 합산 |
|---|---|---|---|---|
| 제한 없음 | 32 (칩 한계) | **128** | 50% | 256 |
| **C (Source Gating, MAX=8)** | **8** | **32** | **12.5%** | **64** |
| C (MAX=4) | 4 | 16 | 6.25% | 32 |

**LF Fill 임계값과 IFIFO 용량의 관계:**

```
Register 6 Fill[7:0]: 양쪽 IFIFO에 동일하게 적용 (독립 설정 불가)

Fill의 의미: 한쪽 IFIFO의 entries가 Fill 이상이면 해당 LF=High

Fill 설정 원칙:
  Fill ≤ IFIFO당 최대 entries → LF=High 도달 가능
  Fill >  IFIFO당 최대 entries → LF=High 도달 불가 (LF 무용)

전략별 Fill 유효 범위:
  제한 없음:    Fill ≤ 128  (IFIFO당 최대 128)
  C (MAX=8):   Fill ≤ 32   (IFIFO당 최대 32)
  C (MAX=4):   Fill ≤ 16   (IFIFO당 최대 16)

Fill 권장값:
  IFIFO당 최대 entries의 1/4 ~ 1/2
  → 전략 C (MAX=8): Fill = 8~16 (기본 16)
  → 제한 없음:      Fill = 32~64 (기본 64)
```

#### 11.3.1 Drain 전략

**전략 A: Full Drain** — EF=1까지 전부 읽기

```
T_drain_worst = 256 reads × 25ns = 6.4 μs (@ DIV=1)
MAX_HITS까지 cell에 저장하고 나머지는 discard. IFIFO 완전히 빈 후 AluTrigger.
```

**전략 A + pipeline 적용 시:**

```
T_critical_A = T_roundtrip + T_drain(6.4) + T_alu(0.045)

  150m: T_critical = 1.0 + 6.445 = 7.445 μs < 8.33 ✓
  400m: T_critical = 2.67 + 6.445 = 9.115 μs > 8.33 ✗
  280m: T_critical = 1.87 + 6.445 = 8.315 μs ≈ 8.33 ← 한계
  400m: T_critical = 2.67 + 6.445 = 9.115 μs > 8.33 ✗ pipeline으로도 불가

→ 전략 A + pipeline 최대 거리 ≈ 280m (최악 hit 수 기준, 옵션 A)
→ 실제 hit가 적으면 더 먼 거리 가능하지만, 최악 보장 불가
```

**전략 B: Capped Drain** — 읽기 횟수 제한 후 AluTrigger

```
N_DRAIN_CAP회 읽은 후 AluTrigger. Master reset이 IFIFO 잔여 entry를 클리어.
T_drain = N_DRAIN_CAP × 25ns (고정, 예측 가능).
```

**전략 B의 한계:**

N_DRAIN_CAP는 **총 읽기 횟수**이지, 채널당 읽기 제한이 아니다. IFIFO 내부에서 채널 간 entry는 collecting logic의 bandwidth distribution에 의해 섞여 있으므로:

```
예: ch0에 20 hits, ch3에 30 hits, 나머지 0 hits
    IFIFO에 50 entries 섞여 있음

    N_DRAIN_CAP=64로 읽으면 50개 모두 읽힘 ✓

    그러나: ch0에 30 hits, ch3에 30 hits, ch5에 30 hits = 90 entries
    N_DRAIN_CAP=64로 읽으면 64개만 읽히고 26개 유실
    → 어떤 채널에서 유실되는지 예측 불가 (IFIFO 순서 의존)
    → 특정 채널의 실제 hit 수 < MAX_HITS일 수 있음
```

**전략 C: Source Gating** — echo_receiver에서 Stop 생성 제한 **(권장)**

```
echo_receiver 모듈이 채널당 Stop pulse 생성을 MAX_HITS_PER_STOP으로 제한.

→ TDC-GPX에 도달하는 Stop 수 자체가 채널당 최대 8개
→ IFIFO에 쌓이는 entries = 최대 8채널 × 8 hits = 64개
→ Full Drain해도 T_drain = 64 × 25ns = 1.6 μs 보장
→ 데이터 유실 없음 + 시간 예측 가능 + IFIFO가 반드시 비워짐
```

**echo_receiver 수정 내용 [코드 참고용, 검토 전용]:**

```
본 코드는 echo_receiver 모듈의 Source Gating 수정 방향을 보여주는 참고용이다.
echo_receiver는 본 설계 범위(tdc_gpx_host) 외부 모듈이므로,
실제 구현 시 echo_receiver 담당자와 인터페이스를 협의해야 한다.

수정 요점:
  - 채널별 stop_count 카운터 추가
  - shot_start 시 초기화
  - stop_count ≥ max_hits_per_stop이면 stop pulse 억제
  - max_hits_per_stop은 제너릭 상수로 echo_receiver에도 동일 값 전달 (합성 시 고정)
```

```vhdl
-- [참고용] echo_receiver Source Gating 수정 스켈레톤
-- 실제 echo_receiver 코드와 병합 시 인터페이스 확인 필요

-- 추가 입력: max_hits_per_stop (제너릭 상수, 합성 시 고정)
--            shot_start         (laser_ctrl에서 공급)

-- 채널별 stop counter
process(clk)
begin
    if rising_edge(clk) then
        if shot_start = '1' then
            stop_count(ch) <= (others => '0');  -- shot 시작 시 초기화
        elsif echo_detected(ch) = '1' then
            if stop_count(ch) < max_hits_per_stop then
                stop_out(ch) <= '1';             -- Stop pulse 출력
                stop_count(ch) <= stop_count(ch) + 1;
            else
                stop_out(ch) <= '0';             -- 제한 도달, Stop 억제
            end if;
        end if;
    end if;
end process;
```

### 11.3.2 전략 비교 종합

| 전략 | T_drain 최악 | 예측 가능 | 데이터 완전성 | 구현 위치 |
|---|---|---|---|---|
| A (Full Drain) | **6.4 μs** | ✗ | ✓ 완전 | tdc_gpx (chip_ctrl) |
| A + pipeline | **6.4 μs** | ✗ | ✓ 완전 | tdc_gpx (chip_ctrl + double-buffer) |
| B (Capped, N=64) | **1.6 μs** | ✓ 고정 | △ 불완전 가능 | tdc_gpx (chip_ctrl) |
| **C (Source Gating)** | **1.6 μs** | **✓ 보장** | **✓ 완전** | **echo_receiver** |

| 전략 | pipeline 없이 최대 거리 | pipeline 적용 최대 거리 | 근거 |
|---|---|---|---|
| A (최악 6.4μs) | ~280m | ~280m (T_drain > T_shot) | T_service=8.73μs > 8.33μs |
| B (N=64) | ~650m | ~987m | T_service=4.03μs (전략 C와 동일 T_drain) |
| **C (Source Gating)** | **~650m** | **~987m** | **T_service=4.03μs, T_critical=1.745μs** |

※ 모든 수치는 옵션 A(T_shot=8.33μs), bus_clk=200MHz, BUS_TICKS=5, DIV=1 기준.
전략 B와 C의 T_drain이 동일(1.7μs)한 이유: 전략 C의 IFIFO 최대=64 entries = 전략 B의 N_DRAIN_CAP=64.

**전략 C가 최적인 이유:**
1. T_drain 상한이 물리적으로 보장됨 (IFIFO에 64개 이상 안 쌓임)
2. 데이터 유실 없음 (MAX_HITS까지 모든 hit가 cell에 저장됨)
3. 전략 B처럼 채널 간 불균등 분배 문제 없음
4. tdc_gpx 모듈 수정 불필요 (echo_receiver만 수정)
5. Full Drain (EF=1까지 읽기)으로 IFIFO가 반드시 비워지므로, AluTrigger 전 상태가 깨끗

**Phase 1 권장: 전략 C (Source Gating).**
#### 11.3.3 LF Block-Read: SINGLE_SHOT 모드 적용 검토

LF(Load-level Flag)는 IFIFO 내 entries가 Register 6 `Fill`[7:0] 설정값 이상이면 **High**가 되는 플래그이다 (데이터시트: "All flags are HIGH active").

**데이터시트 플래그 극성 정리 (모두 HIGH active):**

```
EF (Empty Flag):
  EF = '1' (High) : IFIFO 비어있음 → read 금지 [INV-4]
  EF = '0' (Low)  : 데이터 있음 → read 가능

LF (Load-level Flag):
  LF = '1' (High) : entries ≥ Fill → EF 없이 블록 읽기 가능
  LF = '0' (Low)  : entries < Fill → EF 확인 필요

Fill: Register 6 [7:0], 0~255, 양쪽 IFIFO 공통
주의: LF는 비동기. 해당 FIFO를 읽는 중에는 스파이크 발생 가능
     → LF=High 확인 후에는 Fill개만큼 연속 읽고 LF를 다시 확인하지 않음
```

**전략별 LF 적용 가능 여부:**

| 전략 | IFIFO 최대 (각) | Fill 설정 예 | LF=High 빈도 | 효과 |
|---|---|---|---|---|
| **C (Source Gating)** | 32 | Fill=16 | 높음 | ○ 효과 있음 |
| **C (Source Gating)** | 32 | Fill=32 | 꽉 찰 때만 | △ 한계적 |
| A (Full Drain, 최악) | 128 | Fill=64 | 초기 구간 | ○ 전반부 block |

**전략 C + Fill=16 효과 분석:**

```
Source Gating: IFIFO1 최대 32, IFIFO2 최대 32

Fill=16:
  entries ≥ 16 → LF=High → 16개 블록 읽기 (EF 확인 없음)
  → 나머지 16개는 EF 확인하며 읽기

  블록 구간: 16 × 25ns = 0.4 μs
  EF 구간:   16 × 35ns = 0.56 μs
  총 T_drain ≈ 0.96 μs (양쪽 합 ~1.0 μs + EF overhead)

  LF 미적용 (전부 EF 확인):
  총 T_drain = 32 × 35ns × 2 = 2.24 μs

  → 개선: T_drain 약 55% 단축
  → 그러나 전략 C 기본 T_drain(1.7μs)이 이미 짧으므로 절대 개선폭은 ~0.7μs
```

**전략 A + Fill=64 효과 분석 (최악 128 entries/IFIFO):**

```
Phase 1 (LF=High, entries ≥ 64):
  EF 없이 64개 연속 읽기
  64 × 25ns = 1.6 μs (IFIFO1)

Phase 2 (LF→Low, entries < 64):
  EF 확인하며 읽기
  64 × 35ns = 2.24 μs

총 T_drain ≈ 1.6 + 2.24 = 3.84 μs (IFIFO1 기준, 양쪽 합산 시 interleave)

LF 미적용: 128 × 35ns = 4.48 μs/IFIFO
개선: ~14% 단축
```

**LF-aware drain FSM (정정된 극성):**

```vhdl
-- 데이터시트: All flags are HIGH active
-- lf_active = '1' → Fill 이상 → block read 가능
-- ef_active = '1' → empty → read 금지 [INV-4]

case drain_state is
    when LF_BLOCK_READ =>
        -- LF=High: Fill 이상 entries → EF 없이 연속 읽기
        -- 읽기 중에는 LF 재확인 안 함 (스파이크 방지)
        if lf1_active = '1' and block_count < fill_value then
            read Reg8;
            block_count := block_count + 1;
        end if;
        if lf2_active = '1' and block_count < fill_value then
            read Reg9;
            block_count := block_count + 1;
        end if;
        -- Fill개 완료 또는 LF 미충족 → EF 모드 전환
        if block_count >= fill_value or
           (lf1_active = '0' and lf2_active = '0') then
            drain_state <= EF_CHECK_READ;
        end if;

    when EF_CHECK_READ =>
        -- EF=Low(='0'): 데이터 있음 → 읽기
        if ef1_active = '0' then
            read Reg8;
        end if;
        if ef2_active = '0' then
            read Reg9;
        end if;
        -- 양쪽 EF='1'(empty) → drain 완료
        if ef1_active = '1' and ef2_active = '1' then
            drain_state <= DRAIN_DONE;
        end if;
end case;
```


CSR: echo_receiver에 `MAX_STOPS_PER_CHANNEL` 레지스터 추가 (기본 8).

#### 11.3.4 LF Fast Drain 전략 설계

**목표:** LF + OEN permanent low + same-address burst를 조합하여 T_drain을 최소화한다.

**3가지 최적화 기법:**

```
① LF Block Read   : LF=High 구간에서 EF 확인 생략 → 10ns/read 절약
② OEN Permanent Low: drain 전체 구간에서 OEN=Low 유지 → bus turnaround 제거
③ Same-Address Burst: 한 IFIFO를 연속 읽을 때 ADR 고정 → setup 최소화
```

**기법별 bus_tick 분석 (sys_clk=200MHz, 1 tick=5ns, DIV=1):**

```
[일반 EF-check-then-read] (현재 방식):
  tick 0: EF 핀 샘플 (2-FF sync)
  tick 1: EF 결과 판정
  tick 2: ADR = Reg8/9, OEN='0', CSN='0'
  tick 3: RDN='0' (read strobe start)
  tick 4: RDN='0' (tPW-RL ≥ 6ns 확보)
  tick 5: data sample (IOB FF latch, tV-DR ≤ 11.8ns 충족)
  tick 6: RDN='1' (read strobe end)
  = 7 ticks = 35ns/read

[LF Block Read + OEN Permanent Low]:
  (OEN 이미 Low, ADR 이미 고정)
  tick 0: RDN='0'
  tick 1: RDN='0' (tPW-RL ≥ 6ns)
  tick 2: data sample (10ns > tV-DR min 5ns, 보드 SI 검증 필요)
  tick 3: RDN='1' (tPW-RH ≥ 6ns 시작)
  tick 4: RDN='1' (tPW-RH 충족)
  = 5 ticks = 25ns/read (EF 생략 + OEN/ADR 유지)

[보수적 LF Block Read] (SI 마진 확보):
  tick 0: RDN='0'
  tick 1: RDN='0'
  tick 2: RDN='0' (tPW-RL = 15ns)
  tick 3: data sample (15ns > tV-DR max 11.8ns ✓ 확실)
  tick 4: RDN='1'
  tick 5: RDN='1' (tPW-RH = 10ns)
  = 6 ticks = 30ns/read
```

**Fast Drain FSM 설계:**

```
drain 구간 진입 시:
  OEN <= '0';                   -- permanent low (데이터시트 권장)
  drain_phase <= LF_BURST_1;    -- IFIFO1부터 시작

┌──────────────────────────────────────────────────────────┐
│ LF_BURST_1                                               │
│   ADR = Reg8 (고정)                                       │
│   LF1=High 확인 → Fill개 연속 RDN pulse (25ns each)       │
│   → Fill개 완료 후 LF_BURST_2로 전이                       │
│   → LF1=Low이면 즉시 EF_INTERLEAVE로 전이                  │
├──────────────────────────────────────────────────────────┤
│ LF_BURST_2                                               │
│   ADR = Reg9 (고정)                                       │
│   LF2=High 확인 → Fill개 연속 RDN pulse                    │
│   → Fill개 완료 후 LF_RECHECK로 전이                       │
│   → LF2=Low이면 즉시 EF_INTERLEAVE로 전이                  │
├──────────────────────────────────────────────────────────┤
│ LF_RECHECK                                               │
│   양쪽 LF 재확인 (Fill개 읽은 후 LF가 다시 High일 수 있음)  │
│   → LF1=High → LF_BURST_1                                │
│   → LF2=High → LF_BURST_2                                │
│   → 양쪽 Low → EF_INTERLEAVE                              │
│   (주의: LF 재확인은 해당 IFIFO를 읽지 않는 시점에만 유효)   │
├──────────────────────────────────────────────────────────┤
│ EF_INTERLEAVE                                            │
│   EF1 우선 round-robin (§t5와 동일)                       │
│   EF1='0' → ADR=8, read → EF2='0' → ADR=9, read         │
│   EF1='1' AND EF2='1' → DRAIN_DONE                       │
├──────────────────────────────────────────────────────────┤
│ DRAIN_DONE                                               │
│   OEN <= '1';   -- permanent low 해제                     │
│   → AluTrigger 발행                                       │
└──────────────────────────────────────────────────────────┘
```

**타이밍 비교 (전략 C, 32 entries/IFIFO):**

| 방식 | IFIFO1 | IFIFO2 | 총 T_drain | 절약 |
|---|---|---|---|---|
| 현재 (EF 항상 확인) | 32×35ns=1.12μs | 32×35ns=1.12μs | **~1.7μs** (interleave) | 기준 |
| LF Fill=16, 일반 | 16×25+16×35=0.96μs | 동일 | **~1.5μs** | 12% |
| **LF Fill=16 + OEN perm** | **16×25+16×30=0.88μs** | **동일** | **~1.3μs** | **24%** |
| LF Fill=8 + OEN perm | 4블록×(8×25)+0×35=0.8μs | 동일 | **~1.2μs** | 29% |

**Fill 최적값 결정:**

```
전략 C (32 entries/IFIFO):
  Fill이 너무 크면 → LF=High 도달 못 함 → LF 무용
  Fill이 너무 작으면 → 블록 크기가 작아 재확인 오버헤드 증가

  최적: Fill = IFIFO 최대 entries의 1/4 ~ 1/2
  전략 C: Fill = 8~16 (기본 16 권장)

전략 A (128 entries/IFIFO):
  Fill = 32~64 (기본 64 권장)
```

**VHDL 구현:**

```vhdl
-- Fast Drain Controller
-- OEN permanent low + LF burst + EF interleave

signal drain_phase    : drain_fsm_t;
signal burst_count    : unsigned(7 downto 0);
signal oen_permanent  : std_logic;

process(clk)
begin
    if rising_edge(clk) then
        case drain_phase is

            when DRAIN_INIT =>
                oen_pin        <= '0';           -- OEN permanent low 시작
                oen_permanent  <= '1';
                burst_count    <= (others => '0');
                drain_phase    <= LF_BURST_1;

            when LF_BURST_1 =>
                adr_reg <= ADR_REG8;              -- ADR 고정
                if lf1_sync = '1' and burst_count < fill_value then
                    -- LF=High: EF 없이 연속 읽기 (same-address burst)
                    rdn_pulse <= '1';             -- 5-tick read cycle
                    burst_count <= burst_count + 1;
                    -- data → decode → cell_builder
                elsif lf1_sync = '0' or burst_count >= fill_value then
                    burst_count <= (others => '0');
                    drain_phase <= LF_BURST_2;
                end if;

            when LF_BURST_2 =>
                adr_reg <= ADR_REG9;              -- ADR 전환
                if lf2_sync = '1' and burst_count < fill_value then
                    rdn_pulse <= '1';
                    burst_count <= burst_count + 1;
                elsif lf2_sync = '0' or burst_count >= fill_value then
                    burst_count <= (others => '0');
                    drain_phase <= LF_RECHECK;
                end if;

            when LF_RECHECK =>
                -- LF 재확인 (읽기 중이 아니므로 스파이크 없음)
                if lf1_sync = '1' then
                    drain_phase <= LF_BURST_1;
                elsif lf2_sync = '1' then
                    drain_phase <= LF_BURST_2;
                else
                    drain_phase <= EF_INTERLEAVE;
                end if;

            when EF_INTERLEAVE =>
                -- EF1 우선 round-robin
                if ef1_sync = '0' then
                    adr_reg <= ADR_REG8;
                    rdn_pulse <= '1';
                end if;
                if ef2_sync = '0' then
                    adr_reg <= ADR_REG9;
                    rdn_pulse <= '1';
                end if;
                if ef1_sync = '1' and ef2_sync = '1' then
                    drain_phase <= DRAIN_DONE;
                end if;

            when DRAIN_DONE =>
                oen_pin        <= '1';           -- OEN permanent low 해제
                oen_permanent  <= '0';
                -- → AluTrigger 발행

        end case;
    end if;
end process;
```

**OEN permanent low 안전 규칙:**

```
[INV-1] 보완: OEN permanent low는 drain 구간에서만 허용.
  - drain_phase = DRAIN_INIT ~ DRAIN_DONE 사이에서만 oen_permanent = '1'
  - drain 외 상태 (IDLE, WRITE, INIT)에서는 반드시 OEN='1'
  - WRITE → DRAIN 전환 시: OEN='1' → '0' turnaround gap 1 tick 필수 [INV-5]

[INV-7] OEN permanent low 중 WRITE 금지
  - oen_permanent='1'일 때 cfg_image write 요청이 들어오면 거부
  - drain 완료 후 OEN='1' 복귀한 다음 write 허용
```

**결론 (전략 정리):**

| 조합 | T_drain | 최대 거리 (pipeline) | 구현 복잡도 |
|---|---|---|---|
| 전략 C, EF only | 1.7 μs | 987m | 단순 |
| **전략 C, LF+OEN (권장)** | **1.3 μs** | **1047m** | 중간 |
| 전략 C, LF+OEN Fill=8 | 1.2 μs | 1062m | 중간 |
| 전략 A, LF+OEN Fill=64 | 3.5 μs | 717m | 복잡 |

**Phase 1 구현:** EF only (단순, 검증 우선)
**Phase 1.5 최적화:** LF+OEN 적용 시 T_drain 24% 단축 → pipeline 최대 거리 ~60m 확장

**Phase 1 구현:**
- 전략 C(Source Gating) 기본 → LF block-read 불필요 (FSM 단순화 우선)
- CSR `DRAIN_MODE`로 전략 A/C 선택 가능하게 설계
- 전략 A 선택 시 LF block-read FSM 활성화

```
CSR 추가:
  DRAIN_MODE : [1:0]
    0 = 전략 C (Source Gating + Full Drain, 기본)
    1 = 전략 A (Full Drain, LF block-read 활성)
    2 = 전략 B (Capped Drain, N_DRAIN_CAP 적용)
```

### 11.4 수정된 타이밍 예산

#### 전략 C (Source Gating): T_drain = 1.7 μs (보장)

T_service = T_drain + T_cell + T_vdma + T_alu ≈ 4.0 μs

| 최대 거리 | T_roundtrip | T_service | T_합계 | T_가용 (8.33μs) | 판정 |
|---|---|---|---|---|---|
| 150 m | 1.0 μs | 4.0 μs | 5.0 μs | 3.33 μs | ✓ |
| 400 m | 2.67 μs | 4.0 μs | 6.67 μs | 1.66 μs | ✓ (빠듯) |
| 650 m | 4.33 μs | 4.0 μs | 8.33 μs | 0 μs | ← 한계 |
| 750 m | 5.0 μs | 4.0 μs | 9.0 μs | -0.67 μs | ✗ (pipeline 필요) |

**옵션 A(5면, 120°)에서는 650m가 순차 실행 한계이다.** 750m 이상에서는 pipeline 필수.

#### 참고: 전략 A (Full Drain 최악) + pipeline

T_critical_A = T_roundtrip + 6.445 μs

| 최대 거리 | T_roundtrip | T_critical | T_가용 (8.33μs) | 판정 |
|---|---|---|---|---|
| 150 m | 1.0 μs | 7.445 μs | ✓ | pipeline으로 해결 |
| 280 m | 1.87 μs | 8.315 μs | ← 한계 | pipeline 한계 |
| 400 m | 2.67 μs | 9.115 μs | ✗ | pipeline으로도 불가 |

**전략 A는 옵션 A에서 pipeline 적용해도 280m가 최대.** 전략 C(Source Gating)가 필수.

**결론: 전략 C + pipeline 조합이 최적. 650m까지 순차 실행, pipeline 적용 시 ~987m까지 확장 가능.**

### 11.5 Pipeline 분석: 무엇이 overlap 가능하고, 무엇이 불가한가

**핵심 구분: overlap 가능 여부**

```
1 shot의 시간 순서:

  [roundtrip] → [drain] → [AluTrigger+recovery] → [cell 출력] → [VDMA 전송]
  └────── critical path (overlap 불가) ─────────┘  └── overlap 가능 ──┘

이유:
  - drain은 AluTrigger 전에 끝나야 함 (읽기 전에 reset하면 데이터 유실)
  - AluTrigger는 다음 start 전에 끝나야 함 (칩이 reset되어야 다음 측정 가능)
  - cell 출력과 VDMA 전송은 AluTrigger 이후이므로, 다음 shot의 측정과 overlap 가능
    (cell double-buffer 사용: 한쪽은 VDMA 전송, 다른 쪽은 새 데이터 수집)
```

**T_critical (overlap 불가 최소 시간):**

```
T_critical = T_roundtrip + T_drain + T_alu
           = T_roundtrip + 1.7 + 0.045
           = T_roundtrip + 1.745 μs

제약: T_critical < T_shot_interval (= 8.33 μs, 옵션 A)
→ T_roundtrip < 8.33 - 1.745 = 6.585 μs
→ 최대 거리 = 6.585 μs / 6.67 ns/m ≈ 987 m
```

**pipeline 적용 후 거리별 판정 (옵션 A, T_shot=8.33μs):**

| 최대 거리 | T_roundtrip | T_critical | 마진 | 판정 |
|---|---|---|---|---|
| 150 m | 1.0 μs | 2.745 μs | +5.585 | ✓ pipeline 불필요 |
| 400 m | 2.67 μs | 4.415 μs | +3.915 | ✓ pipeline 불필요 |
| 650 m | 4.33 μs | 6.075 μs | +2.255 | ✓ pipeline 불필요 (순차 한계 부근) |
| 750 m | 5.0 μs | 6.745 μs | +1.585 | ✓ **pipeline으로 해결** |
| **987 m** | **6.58 μs** | **8.325 μs** | **+0.005** | **← 한계** |
| 1000 m | 6.67 μs | 8.415 μs | -0.085 | ✗ pipeline으로도 초과 |
| 1500 m | 10.0 μs | 11.745 μs | -3.415 | ✗ 불가 |

**750m에서 pipeline이 필요한 이유:**

```
순차 실행: T_total = 5.0 + 4.0 = 9.0 μs > 8.33 μs ✗ (0.67μs 초과)
pipeline:  T_critical = 5.0 + 1.745 = 6.745 μs < 8.33 μs ✓ (마진 1.585μs)

→ pipeline으로 cell+VDMA(2.28μs)를 다음 shot과 overlap하여 해결
```

**1500m가 불가능한 근본 이유:**

```
T_roundtrip(10.0μs) + T_drain(1.7μs) = 11.7 μs > T_shot(8.33 μs)

roundtrip과 drain은 모두 overlap 불가:
  - roundtrip: 물리 법칙 (빛의 왕복 시간)
  - drain: AluTrigger 전에 완료 필수

따라서 VDMA를 아무리 overlap해도 해결 불가.
```

**옵션 A 거리 한계 요약:**

```
순차 실행 최대:   ~650 m  (T_roundtrip + 4.0 ≤ 8.33)
pipeline 최대:   ~987 m  (T_roundtrip + 1.745 ≤ 8.33)
그 이상:         옵션 B 전환 (T_shot=13.89μs) 또는 분해능 축소 필요
```

### 11.6 987m+ 장거리 해결 방안 (옵션 A 기준)

옵션 A(T_shot=8.33μs)에서 pipeline 최대 ~987m. 그 이상은 다른 접근이 필요하다.

| 방안 | 방법 | 효과 | 부작용 |
|---|---|---|---|
| **옵션 B 전환** | 4면, 90° FOV | T_shot=13.89μs → pipeline 최대 ~1820m | FOV 축소 (120°→90°) |
| **각도 분해능 축소** | 0.05° → 0.1° | cols=1200, T_shot=16.67μs | 수평 해상도 절반 |
| **미러 회전 감속** | 10 rps → 5 rps | T_shot=16.67μs | scan_frame rate 절반 |
| ~~CONTINUOUS 모드~~ | ~~상시 EF polling~~ | ~~shot 간 reset 불필요~~ | **구현 범위에서 제외됨** |

**현실적 권장:**
- 987m 이상: 옵션 B로 전환 (CSR에서 N_FACES=4, cols_per_face=1800 설정)
- 1600m 이상: TDC-GPX 측정 범위 한계. 분해능 축소 또는 미러 감속 병행

**Phase 1 기본 운용 범위 (옵션 A):**

```
순차 실행: ~650m 이하
pipeline:  ~987m 이하
그 이상:   옵션 B 전환 (CSR 런타임 변경)
```



### 11.7 Pipeline 설계 상세 (750m 기준)

#### 11.7.1 750m 타이밍 수치

```
T_shot_interval = 8.33 μs
T_roundtrip     = 5.0 μs
T_drain         = 1.7 μs  (DIV=1, 전략 C Source Gating)
T_alu_recovery  = 0.045 μs
T_cell_out      = 1.0 μs  (cell → assembler → chip_slice 출력)
T_vdma          = 1.28 μs (256 beats × 5ns @ 200MHz)

T_critical      = T_roundtrip + T_drain + T_alu_recovery
                = 5.0 + 1.7 + 0.045 = 6.745 μs

T_overlap       = T_cell_out + T_vdma
                = 1.0 + 1.28 = 2.28 μs (다음 shot과 overlap 가능)

pipeline 적용 후 마진 = T_shot - T_critical = 8.33 - 6.745 = 1.585 μs ✓
```

#### 11.7.2 pipeline 없는 경우 vs pipeline 적용 비교

**pipeline 없이 (순차 실행):**

```
시간(μs)  0     1     2     3     4     5     6     7     8     9  8.33
          │     │     │     │     │     │     │     │     │  ↑  │     │
shot 0:   [====== roundtrip 5.0 ======][drain 1.7][a][ cell ][ VDMA ]
                                                            1.0    1.28
          │◄─────────────── T_total = 9.025 μs ──────────────────────►│
          │                                   T_shot = 8.33 ↑│
          │                                                  ││ 초과 0.695
shot 1:   │  ← 다음 shot이 8.33μs에 시작해야 하는데, 아직 VDMA 중  │
```

**T_total = 9.025μs > T_shot = 8.33μs → 순차 실행 불가. pipeline 필수.**

**pipeline 적용 (double-buffer):**

```
시간(μs)  0     1     2     3     4     5     6     7     8.33
          │     │     │     │     │     │     │     │     │
shot 0:   [====== roundtrip 5.0 ======][drain 1.7][a]
          │◄──── T_critical = 6.745 ──────────────►│
          │                                         │
shot 1:   │                                         [====== roundtrip 5.0 ...
          │                                         │
buf A:    │                          [drain→A 저장] [cell_out A][ VDMA A ]
          │                                               ↑          ↑
          │                                          buf A에서    VDMA 전송
          │                                          cell 읽기    (overlap)
          │                                         │
          │                          마진 = 1.585 μs│◄── overlap ──►│

shot 0의 cell 출력과 VDMA 전송이 shot 1의 roundtrip 측정 중에 수행됨.
```

#### 11.7.3 Double-Buffer 동작 상세

cell_builder 내부에 **buffer A**와 **buffer B** 두 개의 cell 저장 공간을 둔다.

```
구조:

  cell_builder
   ├─ buf_A[0..stops_per_chip-1]  : cell 배열 A (활성 stop만)
   ├─ buf_B[0..stops_per_chip-1]  : cell 배열 B (활성 stop만)
   ├─ write_sel                    : 현재 drain이 쓰는 버퍼 (A 또는 B)
   ├─ read_sel                     : 현재 VDMA가 읽는 버퍼 (A 또는 B)
   └─ state                        : FSM
```

**핑퐁(ping-pong) 동작:**

```
shot 0:
  drain → buf_A에 write (write_sel = A)
  AluTrigger
  cell_out → buf_A에서 read → VDMA 전송 (read_sel = A)

shot 1:
  drain → buf_B에 write (write_sel = B)    ← shot 0의 VDMA와 동시에 가능
  AluTrigger
  cell_out → buf_B에서 read → VDMA 전송 (read_sel = B)

shot 2:
  drain → buf_A에 write (write_sel = A)    ← buf_A는 이미 VDMA 완료, 재사용 가능
  ...

규칙:
  write_sel과 read_sel은 항상 다른 버퍼를 가리킴
  shot마다 write_sel 토글: A → B → A → B → ...
```

#### 11.7.4 FSM 분리 설계

pipeline을 구현하려면 **chip_ctrl**과 **cell_output_ctrl**을 분리해야 한다.

```
chip_ctrl (critical path 담당):
  ARMED → CAPTURE → DRAIN_IFIFO → ALUTRIGGER → ARMED
  │       │          │              │
  │       │          │              └─ drain 완료 후 즉시 AluTrigger
  │       │          └─ drain 중 buf_X에 write
  │       └─ roundtrip 대기 (IrFlag)
  └─ 다음 start 대기

cell_output_ctrl (overlap path 담당):
  IDLE → WAIT_DRAIN_DONE → OUTPUT_CELLS → WAIT_VDMA_DONE → IDLE
  │                          │               │
  │                          │               └─ assembler → VDMA 전송 완료 대기
  │                          └─ buf_X에서 cell 순차 읽기 → chip_slice 출력
  └─ chip_ctrl의 "drain 완료" 신호 대기
```

**동기 신호:**

```
chip_ctrl → cell_output_ctrl:
  drain_done_pulse    : drain 완료, buf_X 준비됨
  active_buf_id       : 현재 drain이 쓴 버퍼 (A=0, B=1)

cell_output_ctrl → chip_ctrl:
  buf_free_mask[1:0]  : 각 버퍼의 VDMA 전송 완료 여부
                         chip_ctrl은 다음 drain 시 free인 버퍼에 write

에러 조건:
  cell_output_ctrl이 이전 shot의 VDMA를 아직 전송 중인데
  chip_ctrl이 같은 버퍼에 새 drain을 시작하려 함
  → pipeline_overrun 에러 플래그
```

#### 11.7.5 Pipeline Overrun 방지

pipeline overrun = chip_ctrl이 drain할 버퍼가 아직 VDMA 전송 중인 상황.

```
발생 조건:
  T_cell_out + T_vdma > T_critical (다음 shot의 critical path 시간)

750m 기준:
  T_overlap = 2.28 μs
  T_critical = 6.745 μs
  2.28 < 7.045 → ✓ overrun 발생하지 않음 (VDMA가 먼저 끝남)

위험한 경우 (T_roundtrip이 짧을 때):
  150m: T_critical = 3.045 μs, T_overlap = 2.28 μs
  3.045 > 2.28 → ✓ 아직 안전

  50m: T_critical = 0.33 + 2.045 = 2.38 μs, T_overlap = 2.28 μs
  2.38 > 2.28 → ✓ 겨우 안전 (마진 0.1μs)

  30m: T_critical = 0.2 + 2.045 = 2.245 μs, T_overlap = 2.28 μs
  2.245 < 2.28 → ✗ overrun 발생!
```

**overrun 발생 시 대응:**
- `pipeline_overrun` 에러 플래그 세움
- 해당 shot의 cell 출력을 skip하거나 이전 데이터 덮어쓰기
- CSR STATUS에 반영, 필요 시 IRQ

**근거리에서 overrun이 발생하는 역설적 상황:**
- roundtrip이 짧으면 critical path가 짧아져서 shot이 빠르게 반복됨
- 그런데 VDMA 전송 시간은 거리와 무관하게 고정 (1.28 μs)
- 따라서 극근거리에서는 VDMA가 병목이 될 수 있음
- 해결: 극근거리에서는 pipeline을 끄고 순차 실행 (T_total < T_shot이면 문제없음)

#### 11.7.6 Pipeline CSR 제어

| Offset | 이름 | 설명 |
|---|---|---|
| CSR 0x20 | `PIPELINE_EN` | [0] 0 = 순차 실행 (pipeline 없음), 1 = double-buffer pipeline 활성화 |

**동작:**
- `PIPELINE_EN = 0`: chip_ctrl이 drain → cell_out → VDMA 완료까지 모두 끝낸 후 ARMED
- `PIPELINE_EN = 1`: chip_ctrl이 drain → AluTrigger 후 즉시 ARMED, cell_out/VDMA는 병렬 진행

#### 11.7.7 750m Pipeline 타이밍 종합

```
T_shot = 8.33 μs, T_roundtrip = 5.0 μs

shot N:                                          shot N+1:
├── roundtrip(5.0) ──┤── drain(1.7) ──┤alu┤      ├── roundtrip(5.0) ──┤
│                     │                │0.05│      │                     │
│                     │  buf_A write   │    │      │  buf_B write        │
│                     │                │    ├ARMED─┤                     │
│                     │                │    │      │                     │
│                     │                │    │ overlap 구간:              │
│                     │                │    │ [cell A(1.0)][VDMA A(1.28)]│
│                     │                │    │      │                     │
├─ 0 ─── 5.0 ─── 6.7 ─ 6.75 ─┼──────┼── 8.33 ──┤
                                       │      │
                               T_critical     │
                               = 6.745        │
                                              │
                     VDMA 완료 시점 = 6.745 + 2.28 = 9.025 μs
                     다음 shot N+1의 drain 시작 = 8.33 + 5.0 = 13.33 μs
                     buf_A free 마진 = 13.33 - 9.025 = 4.305 μs → overrun 없음 ✓

마진 요약:
  T_critical → T_shot: 8.33 - 6.745 = 1.585 μs (다음 shot 시작까지)
  VDMA 완료 → 다음 drain: 13.33 - 9.025 = 4.305 μs (buf_A 재사용까지)
```

---

## 제12장. 수정된 모듈 구조

### 12.1 전체 블록도

**모듈명 규칙:** VHDL 모듈명은 기존 코드 베이스의 `packet_*` 접두사를 일부 유지한다 (예: `packet_header_inserter`). 본 문서에서는 개념명 `vdma_frame`을 사용하되, 모듈명 참조 시에는 VHDL 이름을 그대로 쓴다.

```
tdc_gpx_host_top
 │
 ├─ tdc_gpx_csr                    (AXI-Lite: cfg_image, MAX_HITS, BUS_CLK_DIV, ...)
 │
 ├─ tdc_gpx_shot_seq_gen              (start pulse → shot_seq, col_in_face)
 │
 ├─ lane[0..3]
 │   ├─ tdc_gpx_bus_phy            (IOBUF, IOB FF, 2-FF sync, bus_tick, bus FSM, StopDis/AluTrigger/PuResN 출력)
 │   ├─ tdc_gpx_chip_ctrl          (SINGLE_SHOT FSM, init/reset/capture/drain)
 │   ├─ tdc_gpx_decode_i           (raw word → decoded fields)
 │   ├─ tdc_gpx_raw_event_builder  (hit_seq_local 생성, raw_event record)
 │   ├─ tdc_gpx_cell_builder       (sparse→dense, MAX_HITS, zero-pad, double-buffer)
 │   └─ m_axis_chip_slice          (8 pixels per shot 출력)
 │
 ├─ tdc_gpx_face_packet_assembler   (N slice → rows_per_face pixel line, sync 대기)
 │
 ├─ tdc_gpx_packet_header_inserter  (header line 삽입)
 │
 ├─ axis_data_fifo                 (VDMA backpressure 완충)
 │
 └─ m_axis_vdma_video              (SOF/EOL, 최종 AXI4-Stream Video Master)
```

### 12.2 각 모듈 파일

| 파일 | 역할 |
|---|---|
| `tdc_gpx_pkg.vhd` | type, constant, record, cell record, header record |
| `tdc_gpx_host_top.vhd` | 최상위 연결 |
| `tdc_gpx_csr.vhd` | AXI-Lite CSR |
| `tdc_gpx_shot_seq_gen.vhd` | shot_seq, col_in_face 관리 |
| `tdc_gpx_bus_phy.vhd` | 물리 bus, IOBUF, bus_tick, StopDis/AluTrigger/PuResN 제어 |
| `tdc_gpx_chip_ctrl.vhd` | SINGLE_SHOT FSM |
| `tdc_gpx_decode_i.vhd` | I-Mode decode |
| `tdc_gpx_raw_event_builder.vhd` | raw_event + hit_seq_local |
| `tdc_gpx_cell_builder.vhd` | sparse→dense, double-buffer |
| `tdc_gpx_face_packet_assembler.vhd` | 4 lane concat |
| `tdc_gpx_packet_header_inserter.vhd` | header line |
| `tdc_gpx_tb.vhd` | 통합 testbench |

### 12.3 bus_phy FSM Invariant 규칙 (RTL 구현 필수)

bus_phy의 FSM 상태별로 반드시 지켜야 하는 핀 제어 규칙이다. 이 규칙을 위반하면 bus contention, garbage read, 칩 손상이 발생할 수 있다.

#### 12.3.1 상태별 핀 제어표

| FSM 상태 | D[27:0] 방향 | OEN | CSN | RDN | WRN | ADR | 비고 |
|---|---|---|---|---|---|---|---|
| **IDLE** | **input** (Hi-Z) | **1** (High) | 1 | 1 | 1 | don't care | bus 유휴. FPGA는 수신 모드 |
| **WRITE_SETUP** | **output** (drive) | **1** (High) | 0 | 1 | 1 | valid | 주소/데이터 안정화 |
| **WRITE_STROBE** | **output** (drive) | **1** (High) | 0 | 1 | **0** | valid | WRN Low → 칩이 데이터 latch |
| **WRITE_HOLD** | **output** (drive) | **1** (High) | 0 | 1 | 1 | valid | WRN 해제 후 hold time 확보 |
| **READ_SETUP** | **input** (Hi-Z) | **0** (Low) | 0 | 1 | 1 | valid | OEN Low → 칩이 D-bus drive 시작 |
| **READ_STROBE** | **input** (Hi-Z) | **0** (Low) | 0 | **0** | 1 | valid | RDN Low → 데이터 유효 |
| **READ_SAMPLE** | **input** (Hi-Z) | **0** (Low) | 0 | **0** | 1 | valid | IOB FF에서 D-bus 샘플 |
| **READ_RELEASE** | **input** (Hi-Z) | **0** (Low) | 0 | 1 | 1 | valid | RDN 해제 |

#### 12.3.2 FSM Invariant (위반 금지 규칙)

```
[INV-1] WRITE 중 OEN = 반드시 High
        OEN=Low이면 칩과 FPGA가 동시에 D-bus를 drive → bus contention → 칩/FPGA 손상 위험
        FSM이 WRITE_* 상태에 있으면 oen_reg <= '1' 강제

[INV-2] READ 중 FPGA D-bus = 반드시 input (Hi-Z)
        FPGA가 D-bus를 drive하면서 read하면 칩 출력과 충돌
        FSM이 READ_* 상태에 있으면 d_bus_oe <= '0' (IOBUF T pin = '1')

[INV-3] IDLE 중 FPGA D-bus = input, OEN = High
        유휴 상태에서 D-bus를 drive하면 칩이 write로 오해석할 수 있음
        power-on default: d_bus_oe = '0', oen_reg = '1'

[INV-4] EF=1이면 Reg8/Reg9 read 금지
        empty IFIFO를 read하면 garbage 데이터 반환
        chip_ctrl FSM에서 강제: ef_empty='1'이면 해당 IFIFO read 요청을 발행하지 않음
        이 규칙은 LF block-read 모드에서도 EF_CHECK_READ 상태에서 반드시 적용

[INV-5] WRITE→READ 전환 시 turnaround gap 필수
        FPGA가 D-bus drive를 해제(output→input)한 후, 칩이 D-bus를 drive(OEN→Low)하기까지
        최소 1 bus_tick의 gap이 필요 (bus contention 방지)
        FSM: WRITE_HOLD → IDLE(1 tick) → READ_SETUP

[INV-6] READ→WRITE 전환 시 OEN High 선행
        칩이 D-bus를 drive 중인 상태에서 FPGA가 drive하면 충돌
        FSM: READ_RELEASE → OEN='1' 설정(1 tick) → IDLE → WRITE_SETUP
```

#### 12.3.3 VHDL 구현 스켈레톤

```vhdl
-- bus_phy 핵심 출력 제어
process(clk)
begin
    if rising_edge(clk) then
        case bus_state is
            when IDLE =>
                d_bus_oe  <= '0';    -- FPGA Hi-Z [INV-3]
                oen_pin   <= '1';    -- 칩 출력 비활성 [INV-3]
                csn_pin   <= '1';
                rdn_pin   <= '1';
                wrn_pin   <= '1';

            when WRITE_SETUP | WRITE_STROBE | WRITE_HOLD =>
                d_bus_oe  <= '1';    -- FPGA가 D-bus drive
                oen_pin   <= '1';    -- 칩 출력 비활성 [INV-1]
                csn_pin   <= '0';
                wrn_pin   <= ... ;   -- STROBE에서만 '0'

            when READ_SETUP | READ_STROBE | READ_SAMPLE | READ_RELEASE =>
                d_bus_oe  <= '0';    -- FPGA Hi-Z [INV-2]
                oen_pin   <= '0';    -- 칩이 D-bus drive
                csn_pin   <= '0';
                rdn_pin   <= ... ;   -- STROBE/SAMPLE에서만 '0'

            when others =>
                d_bus_oe  <= '0';    -- 안전 기본값
                oen_pin   <= '1';
        end case;
    end if;
end process;

-- IOBUF 연결
IOBUF_gen : for i in 0 to 27 generate
    IOBUF_inst : IOBUF
        port map (
            IO => d_bus_pin(i),         -- 물리 핀 (bidirectional)
            I  => d_bus_out(i),         -- FPGA → 칩 (write data)
            O  => d_bus_in(i),          -- 칩 → FPGA (read data)
            T  => not d_bus_oe          -- '1' = Hi-Z, '0' = drive [INV-1,2,3]
        );
end generate;

-- IOB FF (read data sampling)
attribute IOB : string;
attribute IOB of d_bus_in_ff : signal is "TRUE";
process(clk)
begin
    if rising_edge(clk) then
        if bus_state = READ_SAMPLE then
            d_bus_in_ff <= d_bus_in;    -- IOB FF에서 샘플 [INV-2 보장 상태]
        end if;
    end if;
end process;
```

#### 12.3.4 EF read 금지 규칙의 FSM 적용

```vhdl
-- chip_ctrl에서 drain FSM [INV-4]
when EF_CHECK_READ =>
    -- EF1 우선 확인 (§t5 round-robin)
    if ef1_active = '0' then               -- IFIFO1 비어있지 않음
        bus_req <= read_reg8;             -- Reg8 read 허용
    end if;
    -- EF1이 비어있어도 EF2는 반드시 확인
    if ef2_active = '0' then               -- IFIFO2 비어있지 않음
        bus_req <= read_reg9;             -- Reg9 read 허용
    end if;

    -- *** EF=High(active)인 IFIFO는 절대 read하지 않음 ***
    -- ef1_active='1' AND ef2_active='1' → drain_done

when LF_BLOCK_READ =>
    -- LF=High(active) 구간: EF 확인 없이 Fill개 연속 읽기
    -- LF는 비동기이므로 블록 시작 전에만 확인, 읽기 중에는 참조하지 않음
    -- LF→Low 전이 시 (Fill 이하로 내려감) → EF_CHECK_READ로 전환 [INV-4]
    if lf1_active = '1' then
        bus_req <= read_reg8;             -- LF=High 보장: Fill개 이상 있음
    end if;
    if lf2_active = '1' then
        bus_req <= read_reg9;
    end if;
    if lf1_active = '0' and lf2_active = '0' then
        drain_state <= EF_CHECK_READ;     -- LF Low → EF 확인 모드 전환
    end if;
```

#### 12.3.5 bus_phy READ FSM: BUS_TICKS별 동작 파형

bus_phy FSM은 `BUS_TICKS_PER_TRANSACTION` 설정에 따라 READ transaction의 tick 배분을 조정한다.

**bus FSM READ 상태 머신:**

```
              ┌──────────┐
              │   IDLE   │ ← bus_req 없으면 유지
              └────┬─────┘
                   │ bus_req = READ
                   ▼
              ┌──────────┐
              │ RD_PHASE_A│ ← ADR setup (k_adr ticks)
              └────┬─────┘
                   │ k_adr 완료
                   ▼
              ┌──────────┐
              │ RD_PHASE_L│ ← RDN low, 마지막 tick에서 sample (k_rdl ticks)
              └────┬─────┘
                   │ k_rdl 완료 (IOB FF latch)
                   ▼
              ┌──────────┐
              │ RD_PHASE_H│ ← RDN high (k_rdh ticks)
              └────┬─────┘
                   │ k_rdh 완료
                   ▼
              ┌──────────┐
              │   IDLE   │ (또는 연속 read 시 RD_PHASE_A로 직접 전이)
              └──────────┘

BUS_TICKS = k_adr + k_rdl + k_rdh
```

**5-tick READ @ 200 MHz (기본 설정, bus_rate = 40 MHz):**

```
bus_clk = 200 MHz, tick = 5 ns, BUS_TICKS = 5, 배분 1/3/1

             tick 0    tick 1    tick 2    tick 3    tick 4
             │← 5ns →│← 5ns →│← 5ns →│← 5ns →│← 5ns →│
             ├────────┼────────┼────────┼────────┼────────┤
  ADR[3:0]   │ valid (Reg8 or Reg9)                      │ next ADR
             ├────────┼────────┼────────┼────────┼────────┤
  CSN        │___0____|___0____|___0____|___0____|___0____│
             ├────────┼────────┼────────┼────────┼────────┤
  OEN        │___0____|___0____|___0____|___0____|___0____│ (permanent low)
             ├────────┼────────┼────────┼────────┼────────┤
  RDN        │___1____|___0____|___0____|___0____|___1____│
             │        │← tPW-RL = 15ns (≥6ns ✓) →│        │
             ├────────┼────────┼────────┼────────┼────────┤
  DATA[27:0] │ (Hi-Z) │ ????   │  transitioning   │ valid │
             │        │        │        │← tV-DR ≤11.8ns →│
             │        │        │        │   sample here ───┤→ IOB FF
             ├────────┼────────┼────────┼────────┼────────┤
  Phase      │ ADR(A) │  RDN low (L)            │RDN hi(H)│

  tPW-RL = 3 × 5ns = 15 ns ≥ 6 ns ✓
  sample = 15 ns after RDN↓ → 15 > 11.8 ✓
  tPW-RH = 1 × 5ns = 5 ns (단일 read 기준). back-to-back에서 실효 (1+1)×5=10ns ✓*
  T_transaction = 5 × 5ns = 25 ns → bus_rate = 40 MHz
```

**4-tick READ @ 200 MHz (고속 설정, bus_rate = 50 MHz, △):

```
bus_clk = 200 MHz, tick = 5 ns, BUS_TICKS = 4, 배분 1/2/1

             tick 0    tick 1    tick 2    tick 3
             │← 5ns →│← 5ns →│← 5ns →│← 5ns →│
             ├────────┼────────┼────────┼────────┤
  ADR[3:0]   │ valid (Reg8 or Reg9)               │ next
             ├────────┼────────┼────────┼────────┤
  CSN        │___0____|___0____|___0____|___0____│
             ├────────┼────────┼────────┼────────┤
  OEN        │___0____|___0____|___0____|___0____│
             ├────────┼────────┼────────┼────────┤
  RDN        │___1____|___0____|___0____|___1____│
             │        │← tPW-RL=10ns →│          │
             ├────────┼────────┼────────┼────────┤
  DATA[27:0] │ (Hi-Z) │ ????   │ valid? │        │
             │        │        │← sample here ───┤→ IOB FF
             ├────────┼────────┼────────┼────────┤
  Phase      │ ADR(A) │ RDN low (L)   │RDN hi(H)│

  tPW-RL = 2 × 5ns = 10 ns ≥ 6 ns ✓
  sample = 10 ns after RDN↓ → 10 < 11.8 △ (tV-DR만 위험, tPW-RH는 back-to-back으로 해소)
  tPW-RH = 1 × 5ns = 5 ns (단일). back-to-back 실효 10ns ✓*. tV-DR만 △
  T_transaction = 4 × 5ns = 20 ns → bus_rate = 50 MHz
```

**4-tick READ @ 100 MHz (안전 고속, bus_rate = 25 MHz, ✓):**

```
bus_clk = 100 MHz, tick = 10 ns, BUS_TICKS = 4, 배분 1/2/1

             tick 0     tick 1     tick 2     tick 3
             │← 10ns →│← 10ns →│← 10ns →│← 10ns →│
             ├─────────┼─────────┼─────────┼─────────┤
  ADR[3:0]   │ valid                                  │
  CSN        │____0____|____0____|____0____|____0_____│
  OEN        │____0____|____0____|____0____|____0_____│
  RDN        │____1____|____0____|____0____|____1_____│
             │         │← tPW-RL = 20ns ✓→│           │
  DATA[27:0] │ (Hi-Z)  │  transitioning   │  valid   │
             │         │                   │← sample ─┤→ IOB FF
  Phase      │  ADR(A) │   RDN low (L)    │ RDN hi(H)│

  tPW-RL = 2 × 10ns = 20 ns ≥ 6 ns ✓
  sample = 20 ns after RDN↓ → 20 > 11.8 ✓
  tPW-RH = 1 × 10ns = 10 ns ≥ 6 ns ✓
  T_transaction = 4 × 10ns = 40 ns → bus_rate = 25 MHz ✓ 모든 마진 충족
```

**WRITE transaction (BUS_TICKS=5 기준):**

```
             tick 0    tick 1    tick 2    tick 3    tick 4
             ├────────┼────────┼────────┼────────┼────────┤
  ADR[3:0]   │ valid (target register)                    │
  OEN        │___1____|___1____|___1____|___1____|___1____│ [INV-1]
  CSN        │___0____|___0____|___0____|___0____|___0____│
  WRN        │___1____|___0____|___0____|___0____|___1____│
             │        │← tPW-WL = 15ns ✓→│               │
  DATA[27:0] │ valid (FPGA drives)                        │ Hi-Z
             │← tS-DW ≥ 5ns ─→│        │← tH-DW ≥ 4ns ─→│
  Phase      │ SETUP  │     STROBE      │      HOLD      │

  FPGA가 D-bus를 drive (d_bus_oe='1') [INV-2 주의: READ→WRITE 전환 시 gap 필수]
```

**연속 READ (OEN permanent low, same-address burst):**

```
drain 중 IFIFO1 연속 읽기 (ADR=Reg8 고정, OEN='0' 유지):

  read 0        read 1        read 2        read 3
  ├──5 ticks──┤──5 ticks──┤──5 ticks──┤──5 ticks──┤
  ADR  │ 8888888888│ 8888888888│ 8888888888│ 8888888888│
  OEN  │ 0000000000│ 0000000000│ 0000000000│ 0000000000│ ← permanent low
  CSN  │ 0000000000│ 0000000000│ 0000000000│ 0000000000│
  RDN  │ 1,0,0,0,1 │ 1,0,0,0,1 │ 1,0,0,0,1 │ 1,0,0,0,1 │
  DATA │    valid→S│    valid→S│    valid→S│    valid→S│ S=sample

  ADR/OEN/CSN 변경 없음 → bus_phy가 RDN pulse만 반복
  tH-DR: OEN=0 + ADR 고정이면 "무한히 연장 가능" (데이터시트)
  → RDN↑ 후에도 데이터가 유효하게 유지되어 안정적 sample
```

**IFIFO1↔IFIFO2 교대 읽기:**

```
  read IFIFO1   read IFIFO2   read IFIFO1   read IFIFO2
  ├──5 ticks──┤──5 ticks──┤──5 ticks──┤──5 ticks──┤
  ADR  │ 88888│ 99999│ 88888│ 99999│   ← ADR 교대
  OEN  │ 00000│ 00000│ 00000│ 00000│   ← permanent low 유지
  RDN  │ 1,000,1│1,000,1│1,000,1│1,000,1│

  ADR 전환은 tick 0(Phase A)에서 수행 → tS-AD = 1 tick ≥ 2ns ✓
  OEN permanent low이므로 ADR 변경 시 데이터 전환 지연 = tV-DR ≤ 11.8ns
  → Phase L에서 충분한 시간 후 sample
```

**VHDL parameterized FSM:**

```vhdl
-- BUS_TICKS 설정에 따라 tick 카운터로 phase 전환
constant K_ADR : natural := 1;
constant K_RDL : natural := BUS_TICKS - K_ADR - K_RDH;  -- 자동 산출
constant K_RDH : natural := 1;  -- bus_clk ≥ 167MHz이면 2로 변경

signal tick_cnt : unsigned(2 downto 0);  -- 0 ~ BUS_TICKS-1

process(bus_clk)
begin
    if rising_edge(bus_clk) then
        if tick_en = '1' then  -- BUS_CLK_DIV에 의한 tick enable
            tick_cnt <= tick_cnt + 1;

            -- Phase A: ADR setup
            if tick_cnt = 0 then
                adr_pin <= req_addr;
                rdn_pin <= '1';

            -- Phase L: RDN low (tick 1 ~ tick K_ADR+K_RDL-1)
            elsif tick_cnt = K_ADR then
                rdn_pin <= '0';        -- RDN strobe start

            -- Sample: Phase L 마지막 tick
            elsif tick_cnt = K_ADR + K_RDL - 1 then
                rdn_pin <= '0';        -- 아직 low
                sample_en <= '1';      -- IOB FF capture

            -- Phase H: RDN high
            elsif tick_cnt = K_ADR + K_RDL then
                rdn_pin <= '1';        -- RDN strobe end
                sample_en <= '0';

            -- Transaction 완료
            elsif tick_cnt = BUS_TICKS - 1 then
                tick_cnt <= (others => '0');
                rsp_valid <= '1';
            end if;
        end if;
    end if;
end process;
```

---

## 제13장. DDR에서 Frame 복원

### 13.1 DDR 메모리 레이아웃

```
Frame Buffer 0:
  base0 + stride × 0           : header line (1024 bytes)
  base0 + stride × 1           : shot 0 data (rows_per_face cells × cell_size bytes)
  base0 + stride × 2           : shot 1 data
  ...
  base0 + stride × cols        : shot cols-1 data

Frame Buffer 1:
  base1 + stride × 0           : header line
  ...

Frame Buffer 2:
  base2 + ...
```

Triple buffering: VDMA가 buffer 0에 쓰는 동안 SW는 buffer 2를 읽는다.

### 13.2 거리 변환: FPGA inline vs CPU 후처리

#### 13.2.1 처리량 비교

| 항목 | CPU (ARM Cortex-A9) | FPGA (inline pipeline) |
|---|---|---|
| face당 계산 수 | 2400 × 32 × 8 = 614,400 | 동일 |
| 계산당 시간 | ~10 cycles (~15ns @ 667MHz) | **1 clk (5ns, pipeline)** |
| face당 소요 시간 | **~6.9 ms** | **~2.3 ms** (cell 출력에 포함, 추가 0) |
| CPU 점유율 | face 주기(20ms)의 **35%** | **0%** (CPU 완전 해방) |
| 5 faces 합계 | **34.5 ms / 100ms** (CPU의 1/3) | 0 ms (FPGA가 처리) |
| 병렬도 | 1 (NEON SIMD: 4) | **4 lane × pipeline** |

**결론: FPGA inline 변환이 압도적으로 유리.** CPU는 point cloud 후처리(필터링, SLAM 등)에 집중할 수 있다.

#### 13.2.2 FPGA 거리 변환 pipeline

cell_builder → **distance_conv** → assembler 경로에 inline 삽입:

```
pipeline stage 1: corrected_hit = raw_hit - start_off1        (SUB, 1 clk)
pipeline stage 2: distance_mm = corrected_hit × K_DIST_MM     (DSP48 MUL, 1 clk)

K_DIST_MM = bin_ps × c / (2 × 10^9)     [mm/BIN, 고정 상수]
          = 82.3 × 10^-12 × 299792458 / (2 × 10^-9)
          ≈ 12.345 mm/BIN

예: corrected_hit = 1000 → distance = 1000 × 12.345 = 12345 mm = 12.345 m
```

**fixed-point 정밀도:**

```
K_DIST_MM ≈ 12.345 mm/BIN
fixed-point 표현: K = 12345 (× 10^-3 스케일, 즉 μm 단위)

distance_um = corrected_hit × 12345     (16-bit × 14-bit = 30-bit 결과)
→ 32-bit unsigned로 저장
→ 최대: 65535 × 12345 = 808,914,575 μm ≈ 809 m ✓ (16-bit hit 범위 일치)
```

**FPGA 리소스 소비:**

```
lane당: SUB 1개 + DSP48 1개 = 최소 리소스
4 lanes: SUB 4개 + DSP48 4개
Zynq-7000 (Z-7020 기준): DSP48 220개 중 4개 사용 = 1.8%
```

#### 13.2.3 bin_ps와 TDC PLL 설정의 Pair 관리

`bin_ps`는 TDC-GPX의 PLL 설정(Reg7의 HSDiv, RefClkDiv)으로부터 결정된다. 이 둘이 불일치하면 거리 계산이 체계적으로 틀어진다.

**BIN 산출 공식:**

```
BIN(ps) = (Tref × 2^RefClkDiv) / (216 × HSDiv)

여기서:
  Tref       = 25,000 ps (40 MHz reference clock)
  RefClkDiv  = cfg_image Reg7[10:8]  (3-bit, 0~7)
  HSDiv      = cfg_image Reg7[7:0]   (8-bit, 0~255)
```

**문제 상황: 불일치**

```
예: SW가 Reg7을 변경하여 HSDiv=2, RefClkDiv=0으로 설정
    → 실제 BIN = 25000 / (216 × 2) = 57.87 ps
    그런데 bin_ps CSR을 갱신하지 않으면
    → K_DIST가 이전 BIN(82.3ps) 기준으로 남아있음
    → 모든 거리가 82.3/57.87 = 1.42배로 잘못 계산
```

**해결: CSR Pair Group으로 관리**

`bin_ps`를 독립 CSR로 두지 않고, cfg_image의 PLL 관련 필드와 묶어서 FPGA가 자동 산출하거나, SW가 반드시 함께 설정하도록 강제한다.

**방식 A: FPGA 자동 산출 (권장)**

```
SW가 cfg_image의 Reg7을 쓸 때, FPGA가 자동으로:

  1. HSDiv = cfg_image_reg7(7 downto 0)
  2. RefClkDiv = cfg_image_reg7(10 downto 8)
  3. bin_ps = (25000 × 2^RefClkDiv) / (216 × HSDiv)    -- 정수 나눗셈
  4. K_DIST = bin_ps × K_SCALE                           -- DSP48 곱셈

이 값을 내부 레지스터에 latch하고, distance_conv에서 사용

구현: LUT 방식 (HSDiv × RefClkDiv 조합이 제한적이므로 ROM으로 구현 가능)
```

**LUT 구현 (HSDiv × RefClkDiv → bin_ps):**

```vhdl
-- HSDiv: Reg7[7:0] (8-bit, 1~255), RefClkDiv: Reg7[10:8] (3-bit, 0~7)
-- 실제 사용되는 조합은 소수이므로 ROM으로 충분

type bin_lut_t is array(0 to 255, 0 to 7) of unsigned(15 downto 0);
constant BIN_LUT : bin_lut_t := (
    -- HSDiv=1: BIN = 25000 * 2^RCD / 216
    1 => (0 => to_unsigned(11574, 16),   -- 25000/216 = 115.74 ps (×100)
          1 => to_unsigned(23148, 16),   -- 50000/216
          others => (others => '0')),
    2 => (0 => to_unsigned(5787, 16),    -- 25000/432 = 57.87 ps (×100)
          others => (others => '0')),
    -- 일반적으로 사용되는 설정만 채움, 나머지 0
    others => (others => (others => '0'))
);

-- Reg7 write 시 자동 갱신
process(clk)
begin
    if rising_edge(clk) then
        if cfg_reg7_write = '1' then
            hs_div      <= unsigned(cfg_reg7(7 downto 0));    -- Reg7[7:0]
            ref_clk_div <= unsigned(cfg_reg7(10 downto 8));   -- Reg7[10:8]
            bin_ps_x100 <= BIN_LUT(to_integer(hs_div), to_integer(ref_clk_div));
            k_dist      <= bin_ps_x100 * K_SCALE;  -- DSP48
        end if;
    end if;
end process;
```

**방식 B: SW 강제 쌍 쓰기 (대안)**

```
CSR Pair Group 정의:

  PLL_BIN_PAIR (atomic write):
    Offset +0: pll_config  (HSDiv[7:0], RefClkDiv[10:8], MTimer[27:15])
    Offset +2: bin_ps      (16-bit, ps 단위)
    Offset +4: k_dist_fixed (32-bit, fixed-point 거리 상수)

  규칙: pll_config를 쓸 때 bin_ps와 k_dist_fixed도 반드시 함께 써야 함
  검증: FPGA가 pll_config로부터 bin_ps를 재계산하여, SW가 쓴 값과 비교
        불일치 시 → BIN_MISMATCH 에러 플래그 + IRQ
```

```
CSR 추가:
  BIN_MISMATCH : STATUS 레지스터의 1-bit 플래그
                 FPGA 산출 bin_ps ≠ SW 기입 bin_ps 시 set
```

**Phase 1 권장: 방식 A (FPGA 자동 산출).** SW가 cfg_image Reg7만 설정하면 bin_ps → K_DIST가 자동으로 갱신된다. SW 실수로 인한 불일치 가능성 제거.

**연관 CSR Pair 요약:**

```
[cfg_image Reg7] ──자동 산출──→ [bin_ps] ──자동 산출──→ [K_DIST]
                                  ↓
                            [start_off1] ──함께 사용──→ [distance_conv]
                                  ↑
                            [cfg_image Reg5]

Reg5 (StartOff1)를 변경하면 → start_off1 자동 갱신
Reg7 (PLL)을 변경하면      → bin_ps, K_DIST 자동 갱신
```

**header에 기록되는 값:** bin_ps와 K_DIST는 vdma_frame header에도 기록되므로, SW가 DDR에서 vdma_frame을 읽을 때 FPGA가 어떤 상수로 변환했는지 정확히 알 수 있다.

#### 13.2.4 Cell 저장 모드 선택 (CSR)

FPGA가 cell에 무엇을 저장할지 CSR로 선택한다:

| HIT_STORE_MODE | hit_slot에 저장되는 값 | SW 추가 변환 | 용도 |
|---|---|---|---|
| 0 (RAW) | raw_hit 원본 (BIN 단위) | 필요 (full formula) | 디버그, 캘리브레이션 |
| 1 (CORRECTED) | raw_hit - start_off1 (BIN 단위) | 간단 (× K만) | 일반 운용 |
| **2 (DISTANCE)** | **거리 (DIST_SCALE 단위)** | **불필요** | **실시간 point cloud** |

```
CSR:
  HIT_STORE_MODE : [1:0]  0=RAW, 1=CORRECTED, 2=DISTANCE
```

#### 13.2.5 거리 정밀도 선택: DIST_SCALE CSR

HIT_STORE_MODE = 2 (DISTANCE)일 때, hit_slot에 저장되는 거리 값의 **단위(정밀도)**를 선택한다. 운용 거리에 따라 정밀도와 범위를 트레이드오프한다.

| DIST_SCALE | 단위 | 16-bit 최대 거리 | 양자화 오차 | 용도 |
|---|---|---|---|---|
| **0** | **1 mm** | **65.5 m** | **±0.5 mm** | 극근거리 고정밀 |
| 1 | 2 mm | 131 m | ±1 mm | 근거리 고정밀 |
| 2 | 5 mm | 327 m | ±2.5 mm | 중거리 |
| 3 | 1 cm | 655 m | ±5 mm | 중장거리 |
| **4** | **2 cm** | **1310 m** | **±1 cm** | 장거리 최대 범위 |

```
CSR:
  DIST_SCALE : [2:0]  0=1mm, 1=2mm, 2=5mm, 3=1cm, 4=2cm

header에 dist_scale 필드 추가 → SW가 단위를 자동 판별
```

**FPGA 구현:**

```
distance_conv pipeline:

  stage 1: corrected_hit = raw_hit - start_off1
  stage 2: distance_raw_um = corrected_hit × K_DIST_UM    (DSP48, μm 단위)
  stage 3: distance_scaled = distance_raw_um >> SCALE_SHIFT (barrel shifter)

SCALE_SHIFT 매핑:
  DIST_SCALE=0 (1mm):  shift = 0   (μm → mm: /1000, 실제는 K 조정)
  DIST_SCALE=1 (2mm):  shift = 1
  DIST_SCALE=2 (5mm):  shift ≈ 2.32 → LUT로 /5000
  DIST_SCALE=3 (1cm):  shift = 적절한 K_DIST로 직접 산출
  DIST_SCALE=4 (2cm):  shift = 적절한 K_DIST로 직접 산출

실제 구현: DIST_SCALE별로 K_DIST 상수를 미리 계산하여 LUT에 저장.
  K_DIST[scale] = bin_ps × c / (2 × scale_unit_um)

  예 (bin_ps = 82.3 ps):
    DIST_SCALE=0: K = 82.3e-12 × 3e8 / (2 × 1e-3) = 12345 (μm/BIN → 1mm 단위)
    DIST_SCALE=1: K = 82.3e-12 × 3e8 / (2 × 2e-3) = 6172  (→ 2mm 단위)
    DIST_SCALE=2: K = 82.3e-12 × 3e8 / (2 × 5e-3) = 2469  (→ 5mm 단위)
    DIST_SCALE=3: K = 82.3e-12 × 3e8 / (2 × 1e-2) = 1234  (→ 1cm 단위)
    DIST_SCALE=4: K = 82.3e-12 × 3e8 / (2 × 2e-2) = 617   (→ 2cm 단위)

  distance_scaled = corrected_hit × K[scale]    (단일 DSP48 곱셈)
  → 결과의 하위 16-bit를 hit_slot에 저장
```

**운용 예시:**

```
근거리 스캐닝 (실내, 50m 이내):
  DIST_SCALE = 0 (1mm), 최대 65.5m, 정밀도 ±0.5mm

일반 운용 (도로, 200m):
  DIST_SCALE = 2 (5mm), 최대 327m, 정밀도 ±2.5mm

장거리 운용 (항공, 1000m+):
  DIST_SCALE = 4 (2cm), 최대 1310m, 정밀도 ±1cm
```

**Phase 1 기본값:** DIST_SCALE = 3 (1cm, 최대 655m). 대부분의 LiDAR 응용에 적합.

#### 13.2.6 HIT_STORE_MODE에 따른 cell 내 hit_slot 해석

**Zynq-7000 (16-bit hit_slot):**

| MODE | DIST_SCALE | hit_slot 내용 | 범위 | 오차 |
|---|---|---|---|---|
| RAW | — | raw_hit[15:0] | 0~65535 BIN | — |
| CORRECTED | — | (raw_hit - start_off1)[15:0] | 0~65535 BIN | — |
| DISTANCE | 0 (1mm) | distance_mm | 0~65535 mm = 65.5 m | ±0.5 mm |
| DISTANCE | 1 (2mm) | distance_2mm | 0~131070 mm = 131 m | ±1 mm |
| DISTANCE | 2 (5mm) | distance_5mm | 0~327675 mm = 327 m | ±2.5 mm |
| DISTANCE | 3 (1cm) | distance_cm | 0~655350 mm = 655 m | ±5 mm |
| DISTANCE | 4 (2cm) | distance_2cm | 0~1310700 mm = 1310 m | ±1 cm |

**Zynq MPSoC (32-bit packed hit_slot):**

| MODE | hit_slot[16:0] 내용 | hit_slot[31:17] 내용 |
|---|---|---|
| RAW | raw_hit[16:0] (17-bit) | slope, valid, reserved |
| CORRECTED | corrected_hit[16:0] | slope, valid, reserved |
| DISTANCE | distance_mm[16:0] | distance_mm[31:17] |

32-bit에서는 DIST_SCALE = 0 (1mm)에서도 2^32 mm = 4295 km까지 표현 가능. DIST_SCALE 선택이 불필요하며 항상 1mm 정밀도 사용 가능.

### 13.3 SW가 vdma_frame을 복원하는 절차

**정식 파서 코드는 §9.3 참조.** 여기서는 §9.3과의 관계 및 HIT_STORE_MODE별 SW 부담만 기술한다.

```
복원 절차 요약:
1. VDMA vdma_frame done interrupt 수신
2. vdma_frame buffer 주소 획득
3. header 읽기 → §9.2 offset으로 모든 필드 추출
4. hit_store_mode 확인 → 아래 분기
5. data line 순회 → Packed Row 해석 + 유효성 검사 → point cloud 생성
```

**HIT_STORE_MODE별 SW 부담:**

| MODE | hit_slot 해석 | SW 연산 | CPU cycles/hit | face당 총 시간 (@667MHz) |
|---|---|---|---|---|
| 0 (RAW) | raw_hit 원본 → full formula | subtract + multiply + 조건 분기 | ~15 | ~10.4 ms |
| 1 (CORRECTED) | raw_hit - start_off1 → × K | multiply 1회 | ~5 | ~3.5 ms |
| **2 (DISTANCE)** | **거리 값 → × scale** | **읽기 + scale 1회** | **~2** | **~1.4 ms** |

**주의사항:**
- `hit_store_mode`는 header(0x17)에 기록 → SW가 자동 판별
- RAW 모드는 디버그/캘리브레이션 전용
- DISTANCE 모드에서 `start_off1`을 다시 적용하면 이중 보정 오류
- `lane_err_mask`(0x38)가 set된 chip의 데이터는 신뢰 불가

### 13.4 Cell 바이트 레이아웃

#### Zynq-7000 (HIT_SLOT_DATA_WIDTH=16, cell_size=32 bytes)

```
Cell 바이트 레이아웃 (32 bytes = 256 bits, MAX_HITS=8, 16-bit hit):

Byte 0~1   : hit_slot[0]  (16-bit, raw_hit 하위 16-bit)
Byte 2~3   : hit_slot[1]
Byte 4~5   : hit_slot[2]
Byte 6~7   : hit_slot[3]
Byte 8~9   : hit_slot[4]
Byte 10~11 : hit_slot[5]
Byte 12~13 : hit_slot[6]
Byte 14~15 : hit_slot[7]
Byte 16    : hit_valid[7:0]
Byte 17    : slope_vec[7:0]
Byte 18    : hit_count_actual[3:0] (하위 nibble, 0~8)
             hit_dropped (bit 4)
             error_fill  (bit 5, 1=timeout blank cell)
             reserved (bit 6~7)
Byte 19~31 : reserved

측정 범위: ~800m (16-bit × 82.3ps)
```

#### Zynq MPSoC (HIT_SLOT_DATA_WIDTH=17, cell_size=64 bytes) [Phase 2, cell_format=1]

MPSoC는 64-bit 데이터 버스가 기본이므로, hit_slot을 32-bit로 확장하여 정렬한다. 추가 비트에 slope, valid를 inline으로 포함시킨다.

```
Cell 바이트 레이아웃 (64 bytes = 512 bits, MAX_HITS=8, 32-bit packed hit):

1 hit_slot = 32-bit packed:
  [16:0]  = raw_hit (17-bit 원본)
  [17]    = slope (1-bit)
  [18]    = valid (1-bit)
  [31:19] = reserved (13-bit)

Byte 0~3   : hit_slot[0]  (32-bit packed)
Byte 4~7   : hit_slot[1]
Byte 8~11  : hit_slot[2]
Byte 12~15 : hit_slot[3]
Byte 16~19 : hit_slot[4]
Byte 20~23 : hit_slot[5]
Byte 24~27 : hit_slot[6]
Byte 28~31 : hit_slot[7]
Byte 32    : hit_count_actual[3:0] (하위 nibble)
             hit_dropped (bit 4)
             error_fill  (bit 5, 1=timeout blank cell)
             reserved (bit 6~7)
Byte 33~63 : reserved (확장용)

장점:
  - 17-bit raw_hit 원본 보존 → 측정 범위 ~1600m
  - slope, valid가 hit와 같은 word에 inline → SW 접근 1회로 완료
  - 64-bit bus에서 2 hits/word 자연 정렬 (beat 1회 = 2 hits)
  - beats_per_pixel = 64 / 8 = 8 beats (32-bit tdata) 또는 4 beats (64-bit tdata)
```

#### 플랫폼별 cell_size 비교

| 항목 | Zynq-7000 | Zynq MPSoC |
|---|---|---|
| HIT_SLOT_DATA_WIDTH | 16-bit | 17-bit |
| hit_slot 저장 폭 | 16-bit (2 bytes) | 32-bit packed (4 bytes) |
| cell_size | **32 bytes** | **64 bytes** |
| 측정 범위 | ~800m | ~1600m |
| HSIZE (기본, rows=32) | 1024 bytes | 2048 bytes |
| beats/pixel (32-bit tdata) | 8 | 16 |
| beats/pixel (64-bit tdata) | 4 | 8 |
| slope/valid 위치 | cell Byte 17 (별도) | hit_slot[18:17] (inline) |

`HIT_SLOT_DATA_WIDTH`와 `CELL_SIZE_BYTES`는 generic으로 설계하여 합성 시 플랫폼에 맞게 설정한다.

17-bit raw_hit를 16-bit로 truncate할 것인가:
- I-Mode range = 2^17 × ~82 ps ≈ 10.7 μs
- 16-bit로 truncate하면 2^16 × 82 ps ≈ 5.4 μs (약 800m 왕복)
- LiDAR 최대 측정 거리가 400m 이하라면 16-bit로 충분

**Phase 1 권장: cell_format=0 (Zynq-7000, 16-bit hit_slot, cell_size=32B). Phase 2에서 cell_format=1 (MPSoC, 32-bit packed, cell_size=64B) 지원.**


#### 13.4.1 parse_cell 구현 (바이트 → 필드 매핑)

§9.3 파서에서 호출하는 `parse_cell` 함수의 참고 구현. §13.4 바이트 레이아웃과 정확히 1:1 대응해야 한다 [원칙 1].

**Zynq-7000 (cell_size=32, HIT_SLOT_DATA_WIDTH=16):**

```python
def parse_cell(cell_bytes, cell_format, max_hits):
    """§13.4 바이트 레이아웃에 따라 cell raw bytes → 구조체로 변환.
    cell_format (header 0x3F[3:0])이 기준 키."""
    cell = CellData()

    if cell_format == 0:    # Zynq-7000: 16-bit separate layout, cell_size=32B
        # ── hit_slot[0..max_hits-1]: Byte 0~15 (16-bit each) ──
        for i in range(max_hits):
            cell.hit_slot[i] = u16(cell_bytes, i * 2)

        # ── cell-level metadata: Byte 16~18 ──
        cell.hit_valid  = u8(cell_bytes, 16)           # Byte 16: bitmask [7:0]
        cell.slope_vec  = u8(cell_bytes, 17)           # Byte 17: bitmask [7:0]

        meta_byte       = u8(cell_bytes, 18)           # Byte 18: packed
        cell.hit_count_actual = meta_byte & 0x0F       #   [3:0] hit count (0~8)
        cell.hit_dropped      = (meta_byte >> 4) & 1   #   [4]   MAX 초과 여부
        cell.error_fill       = (meta_byte >> 5) & 1   #   [5]   timeout blank cell

        # Byte 19~31: reserved

    elif cell_format == 1:  # MPSoC: 32-bit packed layout, cell_size=64B [Phase 2]
        # ── 32-bit packed hit_slot[0..max_hits-1]: Byte 0~31 ──
        cell.hit_valid = 0    # bitmask 초기화
        cell.slope_vec = 0    # bitmask 초기화
        for i in range(max_hits):
            packed = u32(cell_bytes, i * 4)
            cell.hit_slot[i]  = packed & 0x1FFFF       # [16:0]  raw_hit (17-bit)
            slope_bit         = (packed >> 17) & 1      # [17]    slope
            valid_bit         = (packed >> 18) & 1      # [18]    valid
            # per-hit → bitmask 변환 (§9.3 parser가 bit test로 접근)
            cell.hit_valid   |= (valid_bit << i)
            cell.slope_vec   |= (slope_bit << i)

        # ── cell-level metadata: Byte 32 ──
        meta_byte = u8(cell_bytes, 32)
        cell.hit_count_actual = meta_byte & 0x0F
        cell.hit_dropped      = (meta_byte >> 4) & 1
        cell.error_fill       = (meta_byte >> 5) & 1

        # Byte 33~63: reserved

    else:
        raise ValueError(f"unknown cell_format: {cell_format}")

    return cell
```

**error_fill 사용 흐름 (전체 경로):**

```
FPGA (생성):
  1. assembler가 timeout lane을 감지 → chip_ctrl.timeout = 1
  2. 해당 lane의 cell_builder 출력 대신 blank cell 생성
  3. blank cell의 error_fill = 1 (cell_format별 위치: §13.4 참조)
  4. lane_error_mask (header 0x38) 해당 bit도 1로 설정
  → error_fill은 cell 단위, lane_error_mask는 frame 단위

SW (소비):
  1. header.lane_error_mask로 frame 단위 에러 chip 파악
  2. 각 cell의 error_fill bit로 shot/row 단위 blank 식별
  3. error_fill=1인 cell은 point cloud에서 제외

  if cell.error_fill:
      continue  # assembler가 주입한 blank → skip
```

**error_fill vs lane_error_mask 비교:**

| 속성 | error_fill | lane_error_mask |
|---|---|---|
| 위치 | cell Byte 18 bit 5 | header 0x38 |
| 단위 | **cell 단위** (shot × row) | **frame 단위** (전체 chip) |
| 용도 | 개별 blank cell 식별 | 에러 발생 chip 식별 |
| 관계 | lane_error_mask[chip]=1이면 해당 chip의 일부 cell에 error_fill=1 | error_fill=1인 cell이 하나라도 있으면 lane_error_mask[chip]=1 |

---

## 제14장. 핵심 설계 결정 요약

| 결정 | 내용 | 이유 |
|---|---|---|
| 한 VDMA line = 한 shot | 데이터 생성 순서와 전송 순서 일치 | transpose buffer 불필요 |
| cell에서 zero-padding | cell 초기화 시 0, hit만 덮어씀 | 별도 padding 단계 불필요 |
| header line 삽입 | DDR vdma_frame이 self-descriptive | SW 파싱 독립성 |
| cell_builder double-buffer | shot N 전송 중 shot N+1 수집 | pipeline으로 시간 예산 확보 |
| bus_tick 기반 bus 속도 제어 | 단일 클럭 도메인, CDC 불필요 | CSR로 런타임 조절 |
| IOBUF + IOB FF | Xilinx bidirectional bus 구현 | timing 안정 |
| 같은 채널 내 시간순 보장 | Hit FIFO가 FIFO이므로 자연 보장 | 정렬 불필요 |
| cross-channel 정렬은 SW | HW 복잡도 절약 | Phase 2에서 HW sort 옵션 |
| 32-bit tdata (Zynq-7000 기준) | cell 1개 = 8 beats. generic으로 64/128-bit 확장 | 플랫폼 이식성 |
| triple-buffer DDR | SW read와 HW write 충돌 방지 | 실시간 운용 |

---

## 제15장. 남은 확인 사항

| 항목 | 상태 | 비고 |
|---|---|---|
| Zynq I/O bank 전압 (3.3V 지원) | 미확인 | level shifter 필요 여부 |
| 실제 cols_per_face 값 | 미확정 | 미러 속도와 shot 간격에 의존 |
| MAX_HITS_PER_STOP 최적값 | 미확정 | 응용 요구에 의존 |
| cell_size 최종 결정 | 32B (MAX_HITS=8, 16-bit truncate 기준) | MAX_HITS 변경 시 cell_size도 변경 |
| VDMA tdata 폭 | 32-bit (Zynq-7000 기준) | generic으로 64/128-bit 확장 가능 |
| axis_data_fifo depth | ≥16 lines 권장 | backpressure 시나리오에 의존 |
| bus_phy의 최소 안전 divider | 미측정 | 실제 보드 신호 무결성에 의존 |
