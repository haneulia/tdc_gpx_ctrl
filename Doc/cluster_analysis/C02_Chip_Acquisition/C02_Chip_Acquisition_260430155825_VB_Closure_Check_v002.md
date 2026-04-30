# C02_Chip_Acquisition VB Closure Check v002

문서 버전: `v002`  
작성일: `2026-04-30`  
최종 수정 시간: `2026-04-30 15:58:25 +09:00`  
작성 목적: `C02_Chip_Acquisition_260430155825_Code_Verify_v001.md`의 추가 검증 결과를 Plan v004 기능 검증 경계 Matrix에 반영한다.

---

## 1. 결론

Plan v004의 기능 검증 경계 Matrix는 아직 전체 close가 아니다. 다만 이번 추가 검증으로 C02 내부에서 직접 닫을 수 있는 항목은 더 명확해졌다.

이번에 close 또는 부분 close가 강화된 항목:

- `VB-C02-03`: count-known/stale expected-count 경계 보강
- `VB-C02-05`: latency / throughput / pipeline / II 실측 추가
- `VB-C02-06`: bounded raw output backpressure positive 검증 추가
- `VB-C02-08`: stale expected-count fault propagation 검증 추가
- `VB-C02-10`: 증거 문서 및 xsim 로그 추가

아직 별도 검증이 필요한 항목:

- forced negative simulation과 exit-code evidence
- PH_RESP_DRAIN stuck/fatal 장기 격리
- config_ctrl/top expected-count CDC integration
- downstream 전체 AXI-stream `tuser` boundary
- timing legality illegal combination matrix

---

## 2. Boundary 상태

| Boundary ID | v001 상태 | v002 상태 | 반영 근거 |
|---|---|---|---|
| VB-C02-01 I-Mode single | 부분 검증 | 변경 없음 | Quiet/M/Continuous 배제는 top/config integration에서 추가 확인 필요 |
| VB-C02-02 Datasheet 금지 조건 | 부분 close | 강화됨 | 전체 TB에서 empty IFIFO read count = 0 유지 |
| VB-C02-03 Count-known burst | 부분 close | 강화됨 | stale expected-count 조건에서 EF 우선 stop + faulted drain_done PASS |
| VB-C02-04 Count-unknown EF-only | 부분 close | 변경 없음 | `[2b]`, `[7]` PASS 유지. EF guard timestamp 전용 검증은 보류 |
| VB-C02-05 Pipeline/II | 부분 분석 | 강화됨 | `[16]`에서 first data, internal complete, output done, II min/max 계측 |
| VB-C02-06 Response/backpressure | 미검증 | 부분 close | bounded raw `tready` backpressure PASS. stuck/fatal은 보류 |
| VB-C02-07 AXI-stream sideband contract | 부분 close | 강화됨 | raw data/control/faulted control beat 모니터 추가. downstream 전체는 보류 |
| VB-C02-08 Negative/fail propagation | 미검증 | 부분 close | stale count mismatch fault propagation PASS. forced negative exit-code는 보류 |
| VB-C02-09 Timing legality | 부분 검증 | 변경 없음 | 200 MHz 현재 조건 PASS. illegal matrix는 보류 |
| VB-C02-10 Evidence boundary | 부분 close | 강화됨 | `C02_Chip_Acquisition_260430155825_Code_Verify_v001.md`, xsim PASS 로그 추가 |

---

## 3. 새로 닫힌 증거

| 증거 | 내용 |
|---|---|
| `tb_tdc_gpx_chip_ctrl.vhd:1549-1665` | bounded raw backpressure 및 latency/II 계측 |
| `tb_tdc_gpx_chip_ctrl.vhd:1667-1710` | stale expected-count fault propagation |
| `tdc_gpx_chip_run.vhd:524-554` | completion 경로 mismatch faulted 처리 |
| `xsim.log` | `*** ALL TESTS PASSED *** (total_raw_words=256)` |

---

## 4. 계측 요약

| 항목 | 결과 |
|---|---:|
| first raw data latency | 40 clk |
| chip_run internal drain complete | 167 clk |
| output drain_done accepted | 168 clk |
| raw output hold latency | 1 clk |
| accepted-output II min | 1 clk |
| accepted-output II max | 14 clk |

주의:

- 위 II는 raw output handshaking 기준이다.
- GPX IC read timing은 C01 bus_phy와 Datasheet timing을 기준으로 별도 관리한다.

---

## 5. Lineage

| 이전 문서 | v002 반영 위치 |
|---|---|
| `C02_Chip_Acquisition_260430153700_VB_Closure_Check_v001.md` | 본 문서 전체. v001의 미검증/부분검증 분류를 추가 검증 결과로 갱신 |
| `C02_Chip_Acquisition_260430155825_Code_Verify_v001.md` | section 2~6의 RTL/TB 변경, xsim 결과, latency/II 값을 closure matrix에 반영 |
| `C02_Chip_Acquisition_260430143509_Code_Fix_Plan_v004.md` | 기능 검증 경계 Matrix의 현 상태를 최신화 |
