# Task: 거리 기반 shot window 표준 전면 적용

**기준일**: 2026-04-19
**참조**: `Doc/vdma_packet_structure.html` §5, `memory/project_tdc_window_timing.md`

---

## 1. 적용 기준 (복기)

### 운용 규약
- `shot_period = 1.5 × round-trip(max_distance)` (50% PRF headroom)
- `round-trip(D) = 2 × D / c`, @200MHz: `1 m ≈ 1.335 cycles`
- `shot_period` 동안 빛의 왕복 가능 거리 = 측정거리 × 1.5

### 5 거리 사용 사례 — 타이밍 기본

| 거리 | R (cycles) | shot_period (cycles) | PRF | orphan zone @ margin=32 | shot_period 빛 왕복 |
|-----:|-----------:|---------------------:|----:|-----------------------:|---------------------:|
| 100 m  | 134  | 200  | 1.00 MHz | 34 cycles   | 150 m  |
| 250 m  | 334  | 500  | 400 kHz  | 134 cycles  | 375 m  |
| 500 m  | 667  | 1000 | 200 kHz  | 301 cycles  | 750 m  |
| 750 m  | 1000 | 1500 | 133 kHz  | 468 cycles  | 1125 m |
| 1000 m | 1334 | 2000 | 100 kHz  | 634 cycles  | 1500 m |

### 5 거리 사용 사례 — max_hits / beats per cell

100 m / 500 m / 1000 m 의 max_hits 와 beats/cell 은 HTML §5 값. 250 m / 750 m 은 interpolated.

공식 (`fn_beats_per_cell_rt`):
- 32-bit: `beats = max_hits + 1` (1 슬롯/beat)
- 64-bit: `beats = ceil(max_hits / 3) + 1` (3 슬롯/beat)
- HTML §4/§5 의 "runtime truncated" 최적화: 64-bit + max_hits ≤ 5 는 hit 을 META beat 에 패킹 → 공식값보다 작음

| 거리 | max_hits | 32-bit beats/cell | 64-bit beats/cell |
|-----:|:--------:|:-----------------:|:-----------------:|
| 100 m  | 1          | **2** (공식)        | **1** (HTML packed) |
| 250 m  | 2 (interp) | **3** (interp)     | **1** (interp, packed) |
| 500 m  | 3          | **4** (공식)        | **1** (HTML packed) |
| 750 m  | 6 (interp) | **7** (interp)     | **3** (interp) |
| 1000 m | 7          | **8** (HTML §4)    | **4** (HTML §4) |

**Consumer 모듈**: `cell_builder` · `header_inserter` · `face_assembler` · `output_stage` 가 `max_hits_cfg` (CTL21[18:16]) 기반으로 같은 beats/cell 값을 산정해야 한다 (face-level consistency).

### 타이밍 관계 (shot 내부)
```
t=0           t=R(max_range_clks)    t=W(window_close)     t=T1(next shot_start)
shot[N] ──────┤ 유효 beat 완료        │ orphan 영역         │ shot[N+1]
              │◀────── R ───────────▶│◀─── margin ────────▶│◀──── T1 - T0 ─────▶
```

---

## 2. 작업 범위

### Phase A — 완료
- ✅ [tdc_gpx_stop_cfg_decode.vhd](../../tdc_gpx_stop_cfg_decode.vhd) orphan window 를 `max_range_clks + g_WINDOW_MARGIN_CLKS(=32)` 로 전환 (audit 5번 대응)
- ✅ 메모리에 거리/cycle 환산표 + 50% PRF headroom 규약 기록

### Phase B — 후속 평가 대상 (본 작업 정의의 핵심)

코드베이스의 다른 **shot-bounded 16-bit 고정 타이머** 들을 거리 기반으로 전환할지 검토한다. 목록:

| 파일:라인 | 타이머 | 현재 cap | 성격 | 전환 필요성 |
|:---|:---|---:|:---|:---|
| `tdc_gpx_chip_run.vhd:395,565,611,652,700,757,839` | `s_wait_cnt_r` | `x"FFFF"` (65535) | drain/flush watchdog | **O** — shot 기반 |
| `tdc_gpx_cell_builder.vhd:644,690` | `s_timeout_cnt_r` | `x"FFFF"` | QUARANTINE watchdog | **O** — shot 기반 |
| `tdc_gpx_cell_builder.vhd:908` | `s_out_timeout_r` | `x"FFFF"` | WAIT_IFIFO2 timeout | **O** — shot 기반 |
| `tdc_gpx_cmd_arb.vhd:273,293` | `s_reg_timeout_cnt_r`, `s_dispatch_wait_cnt_r` | `x"FFFF"` | 버스 arb protocol | X — 버스 타이밍, shot 독립 |
| `tdc_gpx_chip_init.vhd:261,292` | `s_rsp_timeout_r` | `x"FFFF"` | 버스 응답 대기 | X — 버스 타이밍 |
| `tdc_gpx_chip_reg.vhd:241` | `s_timeout_r` | `x"FFFF"` | 버스 응답 대기 | X — 버스 타이밍 |
| `tdc_gpx_chip_run.vhd: s_pending_stuck_cnt_r` | | `x"FFFF"` | pending stuck 감지 | △ — 경계 모호, 재검토 |

**전환 대상**: 상단 3개 타이머 (chip_run drain watchdog, cell_builder QUARANTINE, cell_builder WAIT_IFIFO2)

---

## 3. 설계 원칙

### DO
- **shot-bounded 타이머**는 `max_range_clks` snapshot + margin 을 cap 으로 사용
- `max_range_clks = 0` (disable) 일 때는 고정 fallback cap 사용 (기존 `x"FFFF"` 유지)
- mid-shot SW 설정 변경 방어: shot_start 시점 snapshot
- margin generic 은 모듈별로 별도 제공 (echo_receiver 외 내부 latency 가 다를 수 있음)

### DON'T
- **버스 프로토콜 타이머** (chip_init, chip_reg, cmd_arb) 는 변경하지 않는다. 이들은 TDC 버스 응답 시간에 종속이고 shot timing 과 무관
- 50% PRF headroom 을 어긴 운용을 default 로 가정하지 않는다 (margin 축소 시 별도 재검토)

---

## 4. 각 타이머별 세부 작업

### T1. `chip_run.s_wait_cnt_r` (drain/flush watchdog)

**현재**: 고정 `x"FFFF"` 카운터 7개 지점 사용 — ST_DRAIN_* / ST_OVERRUN_FLUSH 에서 stall 감지 후 synthetic drain_done

**변경**:
- `max_range_clks + g_DRAIN_MARGIN_CLKS(default=256)` 을 cap 으로
- `max_range_clks = 0` 시 `x"FFFF"` fallback
- 7개 지점 모두 같은 snapshot 참조

**영향**: 장거리 (1000m → R=1334) 에서도 드레인이 완료되기 전에 타임아웃 나지 않음. 현재 `x"FFFF" = 65535` 는 거의 모든 거리에서 넉넉하지만, 다양한 파이프라인 상태(OVERRUN 등) 조합에서는 타이밍 여유가 들쭉날쭉. 거리 기반으로 통일하면 예측 가능.

### T2. `cell_builder.s_timeout_cnt_r` (QUARANTINE watchdog)

**현재**: `x"FFFF"` — ST_C_DROP/QUARANTINE 에서 final drain_done 기다리는 watchdog

**변경**:
- `max_range_clks + g_QUARANTINE_MARGIN_CLKS(default=512)` 으로 대체 — drain 응답까지 충분한 여유 필요
- `c_QUARANTINE_TICK_CAP` (15 ticks × 65K) 반복 구조는 유지

**영향**: 장거리 shot 에서 quarantine 초조 이탈 가능성 감소.

### T3. `cell_builder.s_out_timeout_r` (WAIT_IFIFO2)

**현재**: `x"FFFF"` — ST_O_WAIT_IFIFO2 에서 ififo2 ready 대기

**변경**:
- 동일 원칙, `max_range_clks + g_IFIFO2_MARGIN_CLKS(default=256)` 으로
- fallback 유지

---

## 5. 수락 기준 (Acceptance)

### 5.1 단위 조건
- [ ] 각 타이머의 cap 이 `max_range_clks + margin` 으로 동작 (snapshot 적용)
- [ ] `max_range_clks = 0` 시 기존 `x"FFFF"` fallback 동작
- [ ] Vivado `check_syntax` 통과
- [ ] 기존 테스트벤치 통과 (cell_builder / chip_run / mask_sweep)

### 5.2 5 거리 사용 사례 시나리오 검증

각 거리 (100/250/500/750/1000m) 에서:

- [ ] **정상 shot**: `shot_period = 1.5 × round-trip` 설정 → 전 pipeline 통과, frame 완주, 에러 sticky 전부 clean
- [ ] **Orphan beat 감지**: `shot_period 끝 직전` 에 stop_evt 한 개 삽입 → `orphan_stop_evt_sticky` 세워짐
- [ ] **Pre-first-shot beat**: reset 후 첫 shot_start 전에 stop_evt → sticky 세워짐
- [ ] **Drain-done 시점 일치**: drain 완료 시점이 `R` 근방에 집중 (cap 조기 타임아웃 없음)
- [ ] **False positive 없음**: 정상 shot 에서 어떤 timer 도 `x"FFFF"` fallback 이나 synthetic recovery 경로 타지 않음

### 5.3 Tight PRF (headroom 없음) 엣지 케이스

- [ ] `shot_period = round-trip` (headroom 0%) 설정 시:
  - window 가 다음 shot_start 로 자동 리셋 → false orphan 없음
  - drain cap 여유 축소 → synthetic recovery 진입 가능 (허용되는 열화 동작)
- [ ] 회복 동작이 `s_err_force_reinit_r` / `s_err_auto_recover_r` 등 기존 sticky 로 관찰 가능

---

## 6. 범위 외 (Out of Scope)

- 버스 프로토콜 타이머 (chip_init / chip_reg / cmd_arb 응답 대기) — shot timing 과 독립
- `face_assembler.s_shot_cnt_r` — 이미 `max_scan_clks` 기반, 별도 규약 적용 (face 레벨 타임아웃)
- `header_inserter` drain watchdog — 이미 별도 설계 (audit 7번 후속)
- 50% 이외의 headroom 정책 지원 — 필요 시 별도 generic 확장

---

## 7. 후속 작업 연관

- audit 6번 (cell_builder quarantine escape 개선) ↔ T2 와 연계 가능
- audit 7번 (header_inserter drain watchdog) 은 별도 규약 필요 시 본 문서 확장

---

## 8. 변경 파일 예상 목록

| 파일 | 변경 내용 |
|:---|:---|
| `tdc_gpx_chip_run.vhd` | generic + cap 계산 + 7개 지점 치환 |
| `tdc_gpx_cell_builder.vhd` | generic + 2개 cap 계산 + 3개 지점 치환 |
| `tdc_gpx_cell_pipe.vhd` | generic pass-through (필요 시) |
| `tdc_gpx_config_ctrl.vhd` | chip_ctrl 하위 generic pass-through (필요 시) |
| `tdc_gpx_top.vhd` | 최상위 generic 노출 (필요 시) |

---

## 9. 추정 규모

- 코드 변경: 3 개 모듈, ~80 라인
- 테스트벤치 추가: 5 거리 시나리오 × 3 검증 포인트 = 15 케이스
- 예상 작업 시간: 약 2–3 시간 (syntax check + TB 수정 포함)

---

## 10. Decision Gate

본 문서의 작업 범위 (T1/T2/T3) 착수 여부는 별도 확인 필요. 현재 audit 우선순위 중:
- **1–5번 완료**, **6–9번 대기**
- Phase B 는 audit 6번 이후 또는 병행 진행 가능
