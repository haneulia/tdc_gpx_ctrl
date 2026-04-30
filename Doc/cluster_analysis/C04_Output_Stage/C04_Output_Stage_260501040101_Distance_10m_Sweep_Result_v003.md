# C04 100m 시작 10m 단위 거리 스윕 결과 v003

- 생성 시간: `2026-05-01 04:01:01 +09:00`
- 최종 수정 시간: `2026-05-01 04:03:51 +09:00`
- 기준 문서: `Doc/TDC-GPX-Datasheet.pdf`
- 기준 분석: `Doc/cluster_analysis/C04_Output_Stage/C04_Output_Stage_260501035419_Distance_Time_Recheck_v002.md`
- 실험 TB: `tb_tdc_gpx_distance_maxhits_matrix_100m_10m.vhd`
- 공통 TB: `tb_tdc_gpx_distance_maxhits_matrix.vhd`
- 실험 로그: `xsim_distance_maxhits_matrix_100m_10m.log`

---

## 1. 점검 목적

사용자 요청에 따라 C04 final output drain의 거리 경계를 `100 m`부터 `10 m` 단위로 다시 점검했다.

판정 모델은 v002와 동일한 post-stop 보수 모델이다.

```text
T0 = start_tdc
T1 = stop_tdc = round_trip(distance)
T2 = next start_tdc = 1.5 * round_trip(distance)
post_stop_budget = T2 - T1 = 0.5 * round_trip(distance)

PASS if C04_drain_time <= post_stop_budget
```

이 모델은 C04 출력이 `stop_tdc` 이후에 drain된다고 보는 보수적인 운용 기준이다. 측정 중 C02/C03/C04가 이미 충분히 overlap drain한다는 별도 end-to-end 검증은 포함하지 않는다.

---

## 2. 100m 시간 기준

TB의 정수 근사값은 `6671 ps/m`이다.

```text
100 m round_trip = 100 * 6671 ps = 667,100 ps = 667.100 ns
shot_period      = 1.5 * 667,100 ps = 1,000,650 ps = 1.000650 us
post_stop_budget = 333,550 ps = 333.550 ns
```

100m에서는 post-stop 예산이 333.550 ns뿐이므로, C04 final stream이 stop 이후에만 drain된다면 32/64/128bit 모두 `cfg=1`부터 FAIL이다.

근거:

| Width | cfg | drain_ps | post_stop_budget_ps | margin_ps | 결과 | 로그 |
|---:|---:|---:|---:|---:|---|---|
| 32 | 1 | 506,692 | 333,550 | -173,142 | FAIL | `xsim_distance_maxhits_matrix_100m_10m.log:34` |
| 64 | 1 | 466,690 | 333,550 | -133,140 | FAIL | `xsim_distance_maxhits_matrix_100m_10m.log:102` |
| 128 | 1 | 446,689 | 333,550 | -113,139 | FAIL | `xsim_distance_maxhits_matrix_100m_10m.log:142` |

---

## 3. 코드/TB 변경 근거

기존 TB는 150m/50m 상수로 고정되어 있었기 때문에, 재사용 가능하도록 시작 거리와 step을 generic으로 분리했다.

| 항목 | 근거 |
|---|---|
| 공통 TB generic 추가 | `tb_tdc_gpx_distance_maxhits_matrix.vhd:34` |
| 시작 거리 generic | `tb_tdc_gpx_distance_maxhits_matrix.vhd:35` |
| 거리 step generic | `tb_tdc_gpx_distance_maxhits_matrix.vhd:36` |
| 상수를 generic에서 받도록 변경 | `tb_tdc_gpx_distance_maxhits_matrix.vhd:44`, `:45` |
| post-stop 예산 계산 | `tb_tdc_gpx_distance_maxhits_matrix.vhd:72` |
| PASS/FAIL 기준 | `tb_tdc_gpx_distance_maxhits_matrix.vhd:130`, `:136` |
| 100m/10m wrapper | `tb_tdc_gpx_distance_maxhits_matrix_100m_10m.vhd:12` |
| wrapper generic map: 100m | `tb_tdc_gpx_distance_maxhits_matrix_100m_10m.vhd:20` |
| wrapper generic map: 10m | `tb_tdc_gpx_distance_maxhits_matrix_100m_10m.vhd:21` |
| shot period 1.5x 계약 | `tdc_gpx_pkg.vhd:281` |
| runtime cell beat 계산 | `tdc_gpx_pkg.vhd:814` |

---

## 4. Width별 최초 PASS 거리

### 32bit

| cfg 범위 | line_beats | drain_ps | 최초 PASS 거리 | PASS margin | 근거 |
|---|---:|---:|---:|---:|---|
| 1..2 | 76 | 506,692 | 160 m | +26,988 ps | `xsim_distance_maxhits_matrix_100m_10m.log:46` |
| 3..4 | 108 | 720,036 | 220 m | +13,774 ps | `xsim_distance_maxhits_matrix_100m_10m.log:62` |
| 5..6 | 140 | 933,380 | 280 m | +560 ps | `xsim_distance_maxhits_matrix_100m_10m.log:80` |
| 7 | 172 | 1,146,724 | 350 m | +20,701 ps | `xsim_distance_maxhits_matrix_100m_10m.log:96` |

32bit 결론:

```text
cfg=6 보수 PASS 시작 거리 = 280 m
cfg=7 보수 PASS 시작 거리 = 350 m
```

### 64bit

| cfg 범위 | line_beats | drain_ps | 최초 PASS 거리 | PASS margin | 근거 |
|---|---:|---:|---:|---:|---|
| 1..4 | 70 | 466,690 | 140 m | +280 ps | `xsim_distance_maxhits_matrix_100m_10m.log:110`, `:116` |
| 5..7 | 102 | 680,034 | 210 m | +20,421 ps | `xsim_distance_maxhits_matrix_100m_10m.log:136` |

64bit 결론:

```text
cfg=6 보수 PASS 시작 거리 = 210 m
cfg=7 보수 PASS 시작 거리 = 210 m
```

### 128bit

| cfg 범위 | line_beats | drain_ps | 최초 PASS 거리 | PASS margin | 근거 |
|---|---:|---:|---:|---:|---|
| 1..7 | 67 | 446,689 | 140 m | +20,281 ps | `xsim_distance_maxhits_matrix_100m_10m.log:150`, `:162` |

128bit 결론:

```text
cfg=6 보수 PASS 시작 거리 = 140 m
cfg=7 보수 PASS 시작 거리 = 140 m
```

---

## 5. 10m 스윕 경계표

| Width | 100m | 110m | 120m | 130m | 140m | 150m | 160m | 170m | 180m | 190m | 200m | 210m | 220m | 230m | 240m | 250m | 260m | 270m | 280m | 290m | 300m | 310m | 320m | 330m | 340m | 350m |
|---:|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 32 cfg1 | F | F | F | F | F | F | P | P | P | P | P | P | P | P | P | P | P | P | P | P | P | P | P | P | P | P |
| 32 cfg3 | F | F | F | F | F | F | F | F | F | F | F | F | P | P | P | P | P | P | P | P | P | P | P | P | P | P |
| 32 cfg5 | F | F | F | F | F | F | F | F | F | F | F | F | F | F | F | F | F | F | P | P | P | P | P | P | P | P |
| 32 cfg7 | F | F | F | F | F | F | F | F | F | F | F | F | F | F | F | F | F | F | F | F | F | F | F | F | F | P |
| 64 cfg1 | F | F | F | F | P | P | P | P | P | P | P | P | P | P | P | P | P | P | P | P | P | P | P | P | P | P |
| 64 cfg5 | F | F | F | F | F | F | F | F | F | F | F | P | P | P | P | P | P | P | P | P | P | P | P | P | P | P |
| 128 cfg1..7 | F | F | F | F | P | P | P | P | P | P | P | P | P | P | P | P | P | P | P | P | P | P | P | P | P | P |

설명:

- `cfg1`은 같은 beat count를 공유하는 cfg 범위의 대표값이다.
- 32bit는 cfg1/2, cfg3/4, cfg5/6, cfg7이 서로 다른 beat group이다.
- 64bit는 cfg1..4, cfg5..7이 서로 다른 beat group이다.
- 128bit는 cfg1..7이 같은 beat group이다.

---

## 6. 운영 판단

100m부터 10m 단위로 보면, post-stop 보수 모델에서 `100m`는 모든 width가 불가능하다.

최소 거리 기준은 다음과 같다.

| 목표 | 32bit | 64bit | 128bit |
|---|---:|---:|---:|
| cfg=1 최소 PASS | 160 m | 140 m | 140 m |
| cfg=6 최소 PASS | 280 m | 210 m | 140 m |
| cfg=7 최소 PASS | 350 m | 210 m | 140 m |

따라서 C04 기준의 보수 운용 계약은 다음처럼 업데이트하는 것이 맞다.

```text
100m 운용에서 stop_tdc 이후 C04 final drain을 완료해야 한다면
32/64/128bit 모두 현재 full-line payload 기준으로는 FAIL이다.

cfg=6 이상을 보수적으로 쓰려면:
  32bit: 280m 이상
  64bit: 210m 이상
  128bit: 140m 이상
```

---

## 7. 검증 명령 및 결과

컴파일:

```powershell
& 'C:\AMDDesignTools\2025.2.1\Vivado\bin\xvhdl.bat' --2008 .\tdc_gpx_pkg.vhd .\tb_tdc_gpx_distance_maxhits_matrix.vhd .\tb_tdc_gpx_distance_maxhits_matrix_100m_10m.vhd
```

Elaborate/run:

```powershell
& 'C:\AMDDesignTools\2025.2.1\Vivado\bin\xelab.bat' tb_tdc_gpx_distance_maxhits_matrix_100m_10m -s tb_tdc_gpx_distance_maxhits_matrix_100m_10m -log xelab_distance_maxhits_matrix_100m_10m.log
& 'C:\AMDDesignTools\2025.2.1\Vivado\bin\xsim.bat' tb_tdc_gpx_distance_maxhits_matrix_100m_10m -runall -log xsim_distance_maxhits_matrix_100m_10m.log
```

결과:

```text
*** tb_tdc_gpx_distance_maxhits_matrix PASS ***
```

근거: `xsim_distance_maxhits_matrix_100m_10m.log:166`
