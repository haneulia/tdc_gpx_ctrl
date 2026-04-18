# Sprint Design — #22 Slope-Independent Abort (Rise 우선, Fall 옵션)

## 배경

**사용자 결정 (Q&A #22):**
- Lidar TDC에서 **Rise 신호는 반드시 필요** (stop pulse 발생 유무 + 반사 횟수)
- **Fall 신호는 pulse-width 검증용, 선택적**
- Fall abort 발생 시 rise를 죽이면 **healthy한 rise 데이터를 버리는 것** → 손실
- Abort는 드물게 설계되지만, 발생 시 rise 보존이 가치 있음

## 현재 (Round 4 이후) 동작

```vhdl
s_pipeline_abort <= i_face_abort or i_face_fall_abort
                    or i_cmd_stop or i_cmd_soft_reset;
```

- Rise OR Fall abort → 전체 pipeline abort
- Rise/Fall cell_builder, face_assembler는 이미 내부적으로 독립 인스턴스
- 그러나 **상위 제어(face_seq, pipeline_abort)가 둘을 묶어서** 한쪽 장애 시 양쪽 모두 아보트

## 목표

Fall abort 단독 시 Rise pipeline은 계속 flow, Fall pipeline만 정리.
Rise abort 시에는 여전히 전체 abort (Rise가 필수).

## 영향받는 모듈 및 변경 요약

### 1. `tdc_gpx_face_seq.vhd`

- `s_pipeline_abort` 를 rise/fall 분리:
  ```vhdl
  -- Fall abort does NOT imply pipeline abort. Only rise-side/system faults do.
  s_pipeline_abort_rise <= i_face_abort or i_cmd_stop or i_cmd_soft_reset;
  s_pipeline_abort_fall <= i_face_fall_abort or i_cmd_stop or i_cmd_soft_reset;
  s_pipeline_abort       <= s_pipeline_abort_rise;  -- legacy alias for "global abort"
  ```

- `s_packet_start` 체크에 포함된 `s_pipeline_abort = '0'` 조건은 rise abort만 체크하도록:
  - Rise가 healthy하면 다음 shot packet_start 허용
  - Fall은 독립 latch (또는 그냥 drop) 처리

- `s_abort_quiesce_r` 를 slope별 `s_abort_quiesce_rise_r`, `s_abort_quiesce_fall_r` 로 분리

- `s_frame_done_both` combiner에 "fall-only abort seen" 인식 추가:
  ```vhdl
  s_fall_abort_seen_r <= i_face_fall_abort or s_fall_abort_seen_r;  -- latch
  s_frame_done_both <= '1'
    when (s_frame_rise_done_r='1' or i_frame_done='1' or s_pipeline_abort='1')
     and (s_frame_fall_done_r='1' or i_frame_fall_done='1'
          or s_pipeline_abort='1' or s_fall_abort_seen_r='1')
    else '0';
  -- reset s_fall_abort_seen_r on packet_start
  ```

- Shot deferral 로직은 rise 기준으로 유지 (fall은 보조)

### 2. `tdc_gpx_cell_pipe.vhd` — 포트 분리

현재:
```vhdl
i_abort : in std_logic;  -- 공통 abort, 8개 cell_builder 모두 reset
```

변경:
```vhdl
i_abort_rise : in std_logic;  -- rise 4개 cell_builder만
i_abort_fall : in std_logic;  -- fall 4개 cell_builder만
```

내부적으로 slope별 generate loop가 해당 abort만 연결.

### 3. `tdc_gpx_output_stage.vhd` — abort 분리

현재:
```vhdl
i_pipeline_abort : in std_logic;
i_flush          : in std_logic;  -- 공통 flush
```

변경:
```vhdl
i_pipeline_abort_rise : in std_logic;
i_pipeline_abort_fall : in std_logic;
-- flush는 세분화 가능하지만 event router 수준에서 공통 유지도 OK
```

### 4. `tdc_gpx_top.vhd` — 배선 업데이트

- `s_pipeline_abort_rise`, `s_pipeline_abort_fall` 두 signal 신설
- cell_pipe, output_stage에 slope별 abort 전달
- decode_pipe의 i_flush는 공통 유지 (이벤트 라우터 수준)

### 5. `tdc_gpx_header_inserter.vhd` — 인증 표시

Rise 와 Fall header_inserter는 이미 별도 인스턴스. 각자 독립적으로 face_abort/face_start 관리. 큰 변경 없음. 다만 header metadata에 slope 상태 표시가 정확해야 함 (현재 chip_error_mask는 공통이지만, 이제 per-slope error 표시 필요할 수 있음).

### 6. `tdc_gpx_face_assembler.vhd` — 이미 slope-독립

Rise + Fall 각각 인스턴스. `i_abort`가 slope별로 들어오면 자동으로 독립 처리됨. 변경 불필요.

## 단계별 구현 순서 (권장)

**Sprint 1: 기반 구축 (2~3 세션)**
1. face_seq 내부 rise/fall 분리 (pipeline_abort, frame_done_both, abort_quiesce)
2. 전체 TB 회귀 — 기존 동작 유지 확인 (pipeline_abort_rise = pipeline_abort_fall 로 연결해서 legacy 동작 재현)

**Sprint 2: Interconnect 분리 (2 세션)**
3. cell_pipe, output_stage 포트 분리
4. top 배선 업데이트
5. TB 회귀

**Sprint 3: 정책 활성화 (1 세션)**
6. `s_pipeline_abort_fall` 만 fall side에 연결, rise에는 연결 안 함
7. 새 test case 추가:
   - Fall overrun만 발생 → rise 계속 flow
   - Rise overrun → 전체 abort
8. SW와 header/status 포맷 합의

## 검증 전략

### 새 TB scenario

```
[Scenario A] Fall-only abort:
  1. face_start → shot_start
  2. Rise cell_builder 정상 flow
  3. Fall cell_builder IFIFO2 timeout 강제 → fall abort
  4. 기대: rise line 완성, fall line partial (또는 zero-fill)
  5. 다음 shot 정상 진행

[Scenario B] Rise abort:
  1. face_start → shot_start
  2. Rise overrun 강제
  3. 기대: 양쪽 pipeline abort

[Scenario C] cmd_stop:
  1. mid-frame cmd_stop
  2. 기대: 양쪽 pipeline abort (현 동작 유지)
```

### 회귀 리스크

- 기존 `s_pipeline_abort` 사용처 ~20곳. 모두 rise/fall 분리 또는 legacy alias로 대응.
- `s_frame_done_both` combiner 변경 시 frame 경계 인식 오류 발생 가능 → 기존 TB가 이를 검증해야 함.

## 예상 규모

| 단계 | 변경 라인 | TB 추가 |
|------|-----------|---------|
| face_seq 내부 분리 | ~100 | 기존 TB 회귀 |
| cell_pipe 포트 분리 | ~30 | port mapping |
| output_stage 포트 분리 | ~40 | port mapping |
| top 배선 | ~20 | - |
| 새 TB scenarios | - | ~150 lines |
| **총계** | **~190** | **~150** |

## 결정 필요 사항 (Sprint 전에)

1. Header metadata에 "fall abort but rise valid" 표시 방법 (cell metadata 확장 vs 새 bit)
2. VDMA frame 포맷: fall truncated 된 line을 어떻게 표시?
   - Fall line을 zero-fill (Round 4 c-simplified 방식 재사용)
   - Fall line을 skip (line_beats 변동)
3. SW 파싱 로직: per-slope validity 어떻게 확인?

## 범위 외

- Rise 자체 overrun 시 fall만 살리는 케이스 (user 결정: rise는 필수)
- Cross-slope 동기화 (현재 loose coupling 유지)
