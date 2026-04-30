# C04 Distance 기반 max_hits_cfg Timing 검증 계획 v001

- 생성 시간: `2026-05-01 03:42:46 +09:00`
- 최종 수정 시간: `2026-05-01 03:42:46 +09:00`
- 기준 문서: `Doc/TDC-GPX-Datasheet.pdf`
- 선행 결과: `Doc/cluster_analysis/C04_Output_Stage/C04_Output_Stage_260501033110_MaxHits_Sweep_v001.md`
- 목적: 거리별 허용 shot interval 안에서 C04 final stream drain이 완료되는지 확인하고, `max_hits_cfg`가 어느 단계까지 PASS되는지 기록한다.

---

## 1. 검증 의도

이번 검증은 단순 기능 PASS가 아니라 운용 timing PASS/FAIL 경계를 찾는 실험이다.

요구 흐름:

1. 거리는 `150 m`에서 시작한다.
2. 거리는 timing FAIL이 발생할 때마다 `50 m` 증가한다.
3. `max_hits_cfg`는 `1`부터 시작한다.
4. 특정 거리에서 어떤 `max_hits_cfg`가 FAIL되면, 다음 거리에서는 실패한 그 `max_hits_cfg`부터 다시 시작한다.
5. 32/64/128bit output width를 모두 점검한다.

---

## 2. Timing PASS/FAIL 기준

거리 `D`에서 허용되는 shot interval은 왕복 ToF로 계산한다.

```text
allowed_time(D) = 2 * D / c
c = 299,792,458 m/s
```

TB에서는 정수 계산을 위해 다음 근사값을 사용한다.

```text
round_trip_ps_per_m = 6671 ps/m
allowed_time_ps = distance_m * 6671
```

C04 final output drain은 downstream output/VDMA 운용 클럭 기준으로 계산한다.

```text
output_clk = 150 MHz
output_clk_period_ps = 6667 ps
drain_time_ps = line_beats * 6667
```

판정:

```text
PASS if drain_time_ps <= allowed_time_ps
FAIL otherwise
```

---

## 3. Line Beat 모델

운용 한계 판단을 위해 full line 조건을 사용한다.

| 항목 | 값 |
|---|---:|
| active chips | 4 |
| stops per chip | 8 |
| cells per line | 32 |
| header bytes | 48 |
| output widths | 32 / 64 / 128 bit |
| max_hits_cfg range | 1..7 |

공식:

```text
line_beats(width, cfg)
  = header_beats(width)
  + active_chips * stops_per_chip * beats_per_cell(cfg, width)
```

`beats_per_cell(cfg, width)`는 RTL package의 `fn_beats_per_cell_rt(max_hits, tdata_width)`와 같은 기준을 사용한다.

---

## 4. Sweep 알고리즘

각 width별로 독립적으로 아래 절차를 수행한다.

```text
distance = 150
cfg = 1

while cfg <= 7:
    result = timing_check(width, distance, cfg)
    record(result)

    if result == PASS:
        cfg = cfg + 1
    else:
        distance = distance + 50
        # cfg는 증가시키지 않고 같은 cfg를 다음 거리에서 재검증
```

이 방식은 “실패한 max_hits_cfg가 거리 증가로 PASS되는지”를 직접 기록한다.

---

## 5. 예상 관찰 포인트

| Width | 예상 |
|---|---|
| 32bit | 높은 `max_hits_cfg`에서 150m FAIL 가능성이 있음 |
| 64bit | line beat 수가 줄어 대부분 150m PASS 예상 |
| 128bit | 모든 cfg가 150m PASS 예상 |

실제 결과는 `tb_tdc_gpx_distance_maxhits_matrix.vhd` xsim 로그로 기록한다.
