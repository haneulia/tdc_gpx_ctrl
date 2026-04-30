# C04 거리 기반 시간 재검토 v002

- 생성 시간: `2026-05-01 03:54:19 +09:00`
- 최종 수정 시간: `2026-05-01 03:58:00 +09:00`
- 기준 문서: `Doc/TDC-GPX-Datasheet.pdf`
- 후속/보정 대상: `Doc/cluster_analysis/C04_Output_Stage/C04_Output_Stage_260501034429_Distance_MaxHits_Result_v001.md`
- 실험 TB: `tb_tdc_gpx_distance_maxhits_matrix.vhd`
- 실험 로그: `xsim_distance_maxhits_matrix.log`

---

## 1. 사용자 지적에 대한 판단

사용자 지적이 맞다. 거리 150 m에서 빛의 왕복 시간은 약 1 us이다.

정확한 물리식은 다음과 같다.

```text
round_trip_time = 2 * distance / c
c = 299,792,458 m/s
distance = 150 m
round_trip_time = 1.000692 us
```

현재 TB는 정수 ps 계산을 위해 `6671 ps/m` 근사를 사용한다.

```text
TB round_trip_ps_per_m = 6671 ps/m
150 m round_trip = 150 * 6671 = 1,000,650 ps = 1.000650 us
```

따라서 150 m 운용에서 시간축은 다음과 같이 잡는 것이 맞다.

```text
T0 = start_tdc = 0 ps
T1 = stop_tdc  = 1,000,650 ps  (150 m 왕복 도착)
```

이전 v001 결과의 문제는 `C04 output drain time <= round_trip_time`만 비교했다는 점이다. 이 판단은 C04 출력이 측정 창과 완전히 겹쳐서 진행된다는 이상적 overlap 모델이다. 하지만 사용자가 말한 것처럼 `stop_tdc`가 왕복 도착 시점이고, C04가 최종 데이터를 `stop_tdc` 이후부터 drain한다고 보는 보수 모델에서는 v001의 "32bit 150m cfg=6 PASS" 판단은 성립하지 않는다.

---

## 2. 이번 v002에서 추가한 시간 모델

기존 코드 주석의 운용 계약은 다음과 같다.

```text
shot_period = 1.5 * round_trip(max_distance)
```

근거:

| 항목 | 근거 |
|---|---|
| `max_range_clks`는 물리 왕복 bound | `tdc_gpx_pkg.vhd:278` |
| `shot_period = 1.5 * round-trip` 운용 계약 | `tdc_gpx_pkg.vhd:281` |
| TB에서 `3/2` shot period 반영 | `tb_tdc_gpx_distance_maxhits_matrix.vhd:44` |
| TB에서 post-stop budget 계산 | `tb_tdc_gpx_distance_maxhits_matrix.vhd:68` |
| TB에서 primary judgment를 post-stop 모델로 변경 | `tb_tdc_gpx_distance_maxhits_matrix.vhd:15` |

따라서 150 m에서 보수적인 C04 drain 가능 시간은 다음과 같다.

```text
start_tdc             = 0 ps
stop_tdc              = 1,000,650 ps
next start_tdc        = 1,500,975 ps
post_stop_budget      = 500,325 ps
```

판정은 다음과 같이 변경했다.

```text
기존 참고 판정:
  ideal_overlap_pass = drain_ps <= round_trip_ps

v002 주 판정:
  post_stop_pass = drain_ps <= post_stop_budget_ps
```

즉, v002의 PASS/FAIL은 `stop_tdc 이후 next start_tdc 이전에 C04 output drain이 끝나는가`를 기준으로 한다.

---

## 3. 150 m에서 6 echo 처리 가능성

`max_hits_cfg=6`은 C04 cell payload가 6 hit까지 담기도록 설정되는 경우다. C04 출력 clock은 150 MHz, 즉 1 beat는 6667 ps로 모델링했다.

| Width | cfg | line_beats | C04 drain time | stop_tdc | next start_tdc | drain end if post-stop | margin | 판단 |
|---:|---:|---:|---:|---:|---:|---:|---:|---|
| 32 | 6 | 140 | 933,380 ps | 1,000,650 ps | 1,500,975 ps | 1,934,030 ps | -433,055 ps | FAIL |
| 64 | 6 | 102 | 680,034 ps | 1,000,650 ps | 1,500,975 ps | 1,680,684 ps | -179,709 ps | FAIL |
| 128 | 6 | 67 | 446,689 ps | 1,000,650 ps | 1,500,975 ps | 1,447,339 ps | +53,636 ps | PASS |

결론:

- 150 m, `shot_period=1.5 x round_trip`, C04 drain이 `stop_tdc 이후` 시작된다고 보면 6 echo는 128bit에서만 PASS다.
- 32bit와 64bit는 150 m에서 6 echo 처리 가능하다고 말하면 안 된다.
- 만약 next `start_tdc`가 정확히 1.000650 us, 즉 `stop_tdc`와 같은 시점에 발생한다면 post-stop budget은 0이므로 C04 drain을 stop 이후에 처리하는 모델에서는 어떤 width도 PASS가 될 수 없다.
- 32/64bit 150 m 6 echo를 주장하려면 C02/C03/C04가 측정 중에 데이터를 충분히 streaming overlap으로 배출한다는 end-to-end 파이프라인 증명이 추가로 필요하다.

---

## 4. v002 xsim 결과 요약

이번 xsim은 `post_stop_pass`를 주 판정으로 사용하며, FAIL 발생 시 같은 `max_hits_cfg`를 다음 거리에서 재시도한다.

### 32bit

| Distance | cfg | line_beats | drain_ps | stop_tdc_ps | shot_period_ps | post_stop_budget_ps | 결과 | 근거 |
|---:|---:|---:|---:|---:|---:|---:|---|---|
| 150 | 1 | 76 | 506,692 | 1,000,650 | 1,500,975 | 500,325 | FAIL | `xsim_distance_maxhits_matrix.log:34` |
| 200 | 1 | 76 | 506,692 | 1,334,200 | 2,001,300 | 667,100 | PASS | `xsim_distance_maxhits_matrix.log:36` |
| 200 | 3 | 108 | 720,036 | 1,334,200 | 2,001,300 | 667,100 | FAIL | `xsim_distance_maxhits_matrix.log:40` |
| 250 | 3 | 108 | 720,036 | 1,667,750 | 2,501,625 | 833,875 | PASS | `xsim_distance_maxhits_matrix.log:42` |
| 250 | 5 | 140 | 933,380 | 1,667,750 | 2,501,625 | 833,875 | FAIL | `xsim_distance_maxhits_matrix.log:46` |
| 300 | 5 | 140 | 933,380 | 2,001,300 | 3,001,950 | 1,000,650 | PASS | `xsim_distance_maxhits_matrix.log:48` |
| 300 | 6 | 140 | 933,380 | 2,001,300 | 3,001,950 | 1,000,650 | PASS | `xsim_distance_maxhits_matrix.log:50` |
| 300 | 7 | 172 | 1,146,724 | 2,001,300 | 3,001,950 | 1,000,650 | FAIL | `xsim_distance_maxhits_matrix.log:52` |
| 350 | 7 | 172 | 1,146,724 | 2,334,850 | 3,502,275 | 1,167,425 | PASS | `xsim_distance_maxhits_matrix.log:54` |

32bit 결론: post-stop 보수 모델에서는 `cfg=6`이 300 m부터 PASS, `cfg=7`은 350 m부터 PASS다.

### 64bit

| Distance | cfg | line_beats | drain_ps | stop_tdc_ps | shot_period_ps | post_stop_budget_ps | 결과 | 근거 |
|---:|---:|---:|---:|---:|---:|---:|---|---|
| 150 | 1 | 70 | 466,690 | 1,000,650 | 1,500,975 | 500,325 | PASS | `xsim_distance_maxhits_matrix.log:60` |
| 150 | 4 | 70 | 466,690 | 1,000,650 | 1,500,975 | 500,325 | PASS | `xsim_distance_maxhits_matrix.log:66` |
| 150 | 5 | 102 | 680,034 | 1,000,650 | 1,500,975 | 500,325 | FAIL | `xsim_distance_maxhits_matrix.log:68` |
| 200 | 5 | 102 | 680,034 | 1,334,200 | 2,001,300 | 667,100 | FAIL | `xsim_distance_maxhits_matrix.log:70` |
| 250 | 5 | 102 | 680,034 | 1,667,750 | 2,501,625 | 833,875 | PASS | `xsim_distance_maxhits_matrix.log:72` |
| 250 | 6 | 102 | 680,034 | 1,667,750 | 2,501,625 | 833,875 | PASS | `xsim_distance_maxhits_matrix.log:74` |
| 250 | 7 | 102 | 680,034 | 1,667,750 | 2,501,625 | 833,875 | PASS | `xsim_distance_maxhits_matrix.log:76` |

64bit 결론: post-stop 보수 모델에서는 `cfg=1..4`만 150 m PASS이고, `cfg=5..7`은 250 m부터 PASS다.

### 128bit

| Distance | cfg | line_beats | drain_ps | stop_tdc_ps | shot_period_ps | post_stop_budget_ps | 결과 | 근거 |
|---:|---:|---:|---:|---:|---:|---:|---|---|
| 150 | 1 | 67 | 446,689 | 1,000,650 | 1,500,975 | 500,325 | PASS | `xsim_distance_maxhits_matrix.log:82` |
| 150 | 6 | 67 | 446,689 | 1,000,650 | 1,500,975 | 500,325 | PASS | `xsim_distance_maxhits_matrix.log:92` |
| 150 | 7 | 67 | 446,689 | 1,000,650 | 1,500,975 | 500,325 | PASS | `xsim_distance_maxhits_matrix.log:94` |

128bit 결론: post-stop 보수 모델에서도 `cfg=1..7`이 모두 150 m PASS다.

---

## 5. v001 대비 보정된 결론

| Width | v001 결론 | v002 보수 결론 | 판단 |
|---:|---|---|---|
| 32 | 150 m에서 cfg=1..6 PASS | 150 m에서 cfg=1도 FAIL, cfg=6은 300 m부터 PASS | v001은 overlap 가정으로 과대평가 |
| 64 | 150 m에서 cfg=1..7 PASS | 150 m에서 cfg=1..4 PASS, cfg=5..7은 250 m부터 PASS | v001은 cfg>=5 과대평가 |
| 128 | 150 m에서 cfg=1..7 PASS | 150 m에서 cfg=1..7 PASS | 동일 |

따라서 "150 m, 1 us 왕복 시간에서 6 echo가 가능하다"는 말은 width별/모델별로 분리해야 한다.

정확한 표현:

```text
150 m / post-stop 보수 모델 / shot_period = 1.5 x round_trip:
  32bit: cfg=6 FAIL
  64bit: cfg=6 FAIL
  128bit: cfg=6 PASS
```

---

## 6. 다음 판단 기준

운용 결정은 아래 중 하나를 선택해야 한다.

| 선택 | 의미 | 필요 검증 |
|---|---|---|
| 보수 모델 채택 | C04 drain은 stop_tdc 이후에 완료되어야 함 | 현재 v002 결과를 운용 경계로 사용 |
| overlap 모델 허용 | C02/C03/C04가 측정 중에도 충분히 streaming drain 가능 | end-to-end 시간 추적 TB 필요 |
| PRF 완화 | next start_tdc를 더 늦춰 post-stop budget 증가 | 거리별 shot_period 설정 테이블 보완 |
| 128bit 채택 | 150 m에서도 cfg=6/7 보수 PASS | VDMA/메모리/다운스트림 128bit 계약 유지 |

현재 증거만 기준으로 하면 C04에서 다음 단계로 넘길 운용 계약은 다음과 같다.

```text
150 m에서 max_hits_cfg=6 이상을 보수적으로 보장하려면 output width는 128bit가 필요하다.
32/64bit는 end-to-end overlap 검증 전까지 150 m cfg=6 PASS로 선언하지 않는다.
```

---

## 7. 추적 근거

| 항목 | 근거 |
|---|---|
| `start_tdc` 입력 포트 | `tdc_gpx_top.vhd:110` |
| `stop_tdc` 입력 포트 | `tdc_gpx_top.vhd:113` |
| Stop event stream from echo_receiver | `tdc_gpx_top.vhd:115` |
| Fire-count final stream | `tdc_gpx_top.vhd:122` |
| `max_range_clks` 물리 왕복 bound | `tdc_gpx_pkg.vhd:278` |
| `shot_period=1.5 x round-trip` 계약 | `tdc_gpx_pkg.vhd:281` |
| runtime cell beat 계산 | `tdc_gpx_pkg.vhd:814` |
| v002 TB primary post-stop 모델 | `tb_tdc_gpx_distance_maxhits_matrix.vhd:15` |
| v002 TB shot period ratio | `tb_tdc_gpx_distance_maxhits_matrix.vhd:44` |
| v002 TB PASS/FAIL 기준 | `tb_tdc_gpx_distance_maxhits_matrix.vhd:126` |
| xsim 최종 PASS | `xsim_distance_maxhits_matrix.log:98` |
