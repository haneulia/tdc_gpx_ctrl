# C04 Polygon Mirror Budget 기반 거리/max_hits_cfg 스윕 결과 v001

- 생성 시간: `2026-05-01 04:25:43 +09:00`
- 최종 수정 시간: `2026-05-01 04:28:31 +09:00`
- 기준 문서: `Doc/TDC-GPX-Datasheet.pdf`
- 사용자 운용 조건: 5각 polygon mirror, 10 Hz, 한 면 72도 중 60도 사용, point 간격 0.05도, VDMA/PS/Ethernet reserve 8 us
- 실험 TB: `tb_tdc_gpx_polygon_budget_matrix.vhd`
- 실험 로그: `xsim_polygon_budget_matrix.log`

---

## 1. 이번 검증 기준

이번 기준은 이전 v003의 `shot_period = 1.5 x round_trip` 모델이 아니다. Polygon mirror의 실제 point 간격을 `start_tdc` 간격으로 놓고, 그 안에서 후단 처리 시간을 먼저 예약한 뒤 C04가 쓸 수 있는 시간을 계산한다.

사용자 조건:

```text
회전속도 = 10 Hz
미러 면 수 = 5
한 면 기계각 = 360 / 5 = 72 deg
72 deg 회전 시간 = 100 ms / 5 = 20 ms
사용 화각 = 60 deg
60 deg 회전 시간 = 20 ms * 60 / 72 = 16.666667 ms
point 간격 = 0.05 deg
point 수 = 60 / 0.05 = 1200
start_tdc 간격 = 16.666667 ms / 1200 = 13.888889 us
```

VDMA 처리, PS 신호처리, Ethernet 전송까지 `8 us`를 예약하면 FPGA/C04 쪽에 남는 pre-ToF 예산은 다음과 같다.

```text
pre_tof_budget = 13.888889 us - 8.000000 us
               = 5.888889 us
```

거리별 판정:

```text
round_trip_ps(distance) = distance_m * 6671 ps
budget_ps(distance) = 5,888,889 ps - round_trip_ps(distance)

PASS if C04_drain_ps(width, max_hits_cfg) <= budget_ps(distance)
```

주의: `8 us`는 Datasheet 값이 아니라 사용자 운용 가정이다. GPX IC 관련 기준은 계속 `Doc/TDC-GPX-Datasheet.pdf`를 상위 기준으로 둔다.

---

## 2. TB 변경/검증 근거

| 항목 | 근거 |
|---|---|
| point interval 13.888889 us | `tb_tdc_gpx_polygon_budget_matrix.vhd:40` |
| VDMA/PS/Ethernet reserve 8 us | `tb_tdc_gpx_polygon_budget_matrix.vhd:41` |
| pre-ToF budget 계산 | `tb_tdc_gpx_polygon_budget_matrix.vhd:42`, `:43` |
| 거리별 budget 계산 | `tb_tdc_gpx_polygon_budget_matrix.vhd:60`, `:62` |
| 거리별 최대 PASS cfg 산출 | `tb_tdc_gpx_polygon_budget_matrix.vhd:102`, `:106` |
| boundary 로그 출력 | `tb_tdc_gpx_polygon_budget_matrix.vhd:130`, `:131` |
| output width 32/64/128 스윕 | `tb_tdc_gpx_polygon_budget_matrix.vhd:169`, `:170`, `:171` |
| runtime cell beat 계산 | `tdc_gpx_pkg.vhd:814` |
| `max_hits_cfg` 필드 | `tdc_gpx_pkg.vhd:293` |

---

## 3. 10m 기준 예산

10m에서는 왕복 시간이 매우 짧기 때문에 모든 width가 `max_hits_cfg=7`까지 PASS한다.

```text
10m round_trip = 66,710 ps = 66.710 ns
budget = 5,888,889 ps - 66,710 ps
       = 5,822,179 ps = 5.822179 us
```

| Width | max PASS cfg | cfg1 margin | cfg6 margin | cfg7 margin | 근거 |
|---:|---:|---:|---:|---:|---|
| 32 | 7 | 5,315,487 ps | 4,888,799 ps | 4,675,455 ps | `xsim_polygon_budget_matrix.log:34` |
| 64 | 7 | 5,355,489 ps | 5,142,145 ps | 5,142,145 ps | `xsim_polygon_budget_matrix.log:212` |
| 128 | 7 | 5,375,490 ps | 5,375,490 ps | 5,375,490 ps | `xsim_polygon_budget_matrix.log:386` |

---

## 4. max_hits_cfg 스왑 결과

거리 증가에 따라 왕복 시간이 증가하고, C04에 남는 budget은 감소한다. 따라서 각 width에서 권장 가능한 최대 `max_hits_cfg`는 아래처럼 단계적으로 낮아진다.

### 32bit

| 거리 구간 | 권장 최대 cfg | 의미 | 근거 |
|---|---:|---|---|
| 10m ~ 710m | 7 | full cfg 사용 가능 | `xsim_polygon_budget_matrix.log:36`, `:176` |
| 720m ~ 740m | 6 | cfg7은 FAIL, cfg6까지 PASS | `xsim_polygon_budget_matrix.log:180`, `:184` |
| 750m ~ 770m | 4 | cfg5/6/7은 FAIL, cfg4까지 PASS | `xsim_polygon_budget_matrix.log:188`, `:192` |
| 780m ~ 800m | 2 | cfg3 이상 FAIL, cfg2까지 PASS | `xsim_polygon_budget_matrix.log:196`, `:200` |
| 810m 이상 | 0 | cfg1도 FAIL | `xsim_polygon_budget_matrix.log:204` |

경계 상세:

| 거리 | budget_ps | max cfg | cfg1 margin | cfg6 margin | cfg7 margin | 근거 |
|---:|---:|---:|---:|---:|---:|---|
| 710 | 1,152,479 | 7 | 645,787 | 219,099 | 5,755 | `xsim_polygon_budget_matrix.log:176` |
| 720 | 1,085,769 | 6 | 579,077 | 152,389 | -60,955 | `xsim_polygon_budget_matrix.log:178` |
| 740 | 952,349 | 6 | 445,657 | 18,969 | -194,375 | `xsim_polygon_budget_matrix.log:184` |
| 750 | 885,639 | 4 | 378,947 | -47,741 | -261,085 | `xsim_polygon_budget_matrix.log:186` |
| 770 | 752,219 | 4 | 245,527 | -181,161 | -394,505 | `xsim_polygon_budget_matrix.log:192` |
| 780 | 685,509 | 2 | 178,817 | -247,871 | -461,215 | `xsim_polygon_budget_matrix.log:194` |
| 800 | 552,089 | 2 | 45,397 | -381,291 | -594,635 | `xsim_polygon_budget_matrix.log:200` |
| 810 | 485,379 | 0 | -21,313 | -448,001 | -661,345 | `xsim_polygon_budget_matrix.log:202` |

### 64bit

| 거리 구간 | 권장 최대 cfg | 의미 | 근거 |
|---|---:|---|---|
| 10m ~ 780m | 7 | full cfg 사용 가능 | `xsim_polygon_budget_matrix.log:212`, `:366` |
| 790m ~ 810m | 4 | cfg5/6/7은 FAIL, cfg4까지 PASS | `xsim_polygon_budget_matrix.log:370`, `:374` |
| 820m 이상 | 0 | cfg1도 FAIL | `xsim_polygon_budget_matrix.log:378` |

경계 상세:

| 거리 | budget_ps | max cfg | cfg1 margin | cfg6 margin | cfg7 margin | 근거 |
|---:|---:|---:|---:|---:|---:|---|
| 780 | 685,509 | 7 | 218,819 | 5,475 | 5,475 | `xsim_polygon_budget_matrix.log:366` |
| 790 | 618,799 | 4 | 152,109 | -61,235 | -61,235 | `xsim_polygon_budget_matrix.log:368` |
| 810 | 485,379 | 4 | 18,689 | -194,655 | -194,655 | `xsim_polygon_budget_matrix.log:374` |
| 820 | 418,669 | 0 | -48,021 | -261,365 | -261,365 | `xsim_polygon_budget_matrix.log:376` |

### 128bit

| 거리 구간 | 권장 최대 cfg | 의미 | 근거 |
|---|---:|---|---|
| 10m ~ 810m | 7 | cfg1~7 beat 수가 같아서 full cfg 유지 가능 | `xsim_polygon_budget_matrix.log:386`, `:546` |
| 820m 이상 | 0 | cfg1도 FAIL | `xsim_polygon_budget_matrix.log:550` |

경계 상세:

| 거리 | budget_ps | max cfg | cfg1/6/7 margin | 근거 |
|---:|---:|---:|---:|---|
| 810 | 485,379 | 7 | 38,690 | `xsim_polygon_budget_matrix.log:546` |
| 820 | 418,669 | 0 | -28,020 | `xsim_polygon_budget_matrix.log:548` |

---

## 5. Width별 운용 결론

| Width | cfg=7 유지 가능 거리 | cfg swap 구간 | 완전 FAIL 시작 |
|---:|---:|---|---:|
| 32 | 710m까지 | 720~740m cfg6, 750~770m cfg4, 780~800m cfg2 | 810m |
| 64 | 780m까지 | 790~810m cfg4 | 820m |
| 128 | 810m까지 | 없음 | 820m |

이 결과는 이전 150m/100m post-stop 분석과 정반대로 보일 수 있지만, 기준 시간이 달라졌기 때문이다. 이전 분석은 `shot_period = 1.5 x 왕복시간`이라 근거리일수록 예산이 매우 작아졌다. 이번 분석은 polygon mirror의 `start_tdc` 간격이 고정 `13.888889 us`이고, 후단 8us를 뺀 뒤에도 `5.888889 us - 왕복시간`이 남기 때문에 10m~수백m 범위에서 훨씬 여유가 크다.

---

## 6. 운용 제안

현재 가정 기준에서 권장 운용은 다음과 같다.

```text
32bit:
  710m 이하: max_hits_cfg = 7
  720~740m: max_hits_cfg = 6
  750~770m: max_hits_cfg = 4
  780~800m: max_hits_cfg = 2
  810m 이상: FAIL

64bit:
  780m 이하: max_hits_cfg = 7
  790~810m: max_hits_cfg = 4
  820m 이상: FAIL

128bit:
  810m 이하: max_hits_cfg = 7
  820m 이상: FAIL
```

다음 단계에서 더 엄밀하게 닫으려면 C04만이 아니라 C02/C03/C04 end-to-end latency, VDMA 실제 backpressure, PS/Ethernet 8us의 실측 worst-case를 반영해야 한다.

---

## 7. 검증 명령 및 결과

컴파일:

```powershell
& 'C:\AMDDesignTools\2025.2.1\Vivado\bin\xvhdl.bat' --2008 .\tdc_gpx_pkg.vhd .\tb_tdc_gpx_polygon_budget_matrix.vhd
```

Elaborate/run:

```powershell
& 'C:\AMDDesignTools\2025.2.1\Vivado\bin\xelab.bat' tb_tdc_gpx_polygon_budget_matrix -s tb_tdc_gpx_polygon_budget_matrix -log xelab_polygon_budget_matrix.log
& 'C:\AMDDesignTools\2025.2.1\Vivado\bin\xsim.bat' tb_tdc_gpx_polygon_budget_matrix -runall -log xsim_polygon_budget_matrix.log
```

결과:

```text
*** tb_tdc_gpx_polygon_budget_matrix PASS ***
```

근거: `xsim_polygon_budget_matrix.log:554`
