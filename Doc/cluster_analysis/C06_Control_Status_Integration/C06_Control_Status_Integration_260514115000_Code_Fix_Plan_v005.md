# C06 Control/Status Integration Code Fix Plan v005

| 항목 | 내용 |
|---|---|
| 문서 버전 | v005 |
| 생성 시간 | 2026-05-14 11:50:00 KST |
| 수정 시간 | 2026-05-14 11:50:00 KST |
| Cluster | C06 Control / Status Integration |
| 절대 기준 문서 | `Doc/TDC-GPX-Datasheet.pdf` |
| 선행 결과 | `Doc/cluster_analysis/C06_Control_Status_Integration/C06_Control_Status_Integration_260514115000_Code_Fix_Result_v004.md` |
| 목적 | C06 recovery 보완 완료 후 handoff 갱신, release 전 정리, 다음 cluster 진입 조건을 닫는다. |

## 1. v004 완료 기준

v004 결과로 C06의 핵심 control/status recovery 항목은 닫혔다.

| 완료 ID | 내용 | 상태 | 근거 |
|---|---|---|---|
| CL-C06-01 | `force_reinit` command가 AXI source pulse에서 TDC pulse까지 전달됨 | Closed | `xsim_c06_v004_top_int_force_260514115000.log:142`, `:146` |
| CL-C06-02 | `force_reinit` 후 정상 run 재개 | Closed | `xsim_c06_v004_top_int_force_260514115000.log:236` |
| CL-C06-03 | `soft_reset` command가 AXI source pulse에서 TDC pulse까지 전달됨 | Closed | `xsim_c06_v004_top_int_soft_260514115000.log:142`, `:146` |
| CL-C06-04 | `soft_reset` 후 `PH_RESP_DRAIN -> PH_INIT -> run` 재개 | Closed | `xsim_c06_v004_top_int_soft_260514115000.log:156`, `:244` |
| CL-C06-05 | 32/64/128 output width baseline 유지 | Closed | `xsim_c06_v002_top_int_w32_260514115000.log:143`, `xsim_c06_v002_top_int_w64_260514115000.log:143`, `xsim_c06_v002_top_int_w128_260514115000.log:143` |

## 2. v005 작업 목표

| ID | 목표 | 우선순위 | 완료 기준 |
|---|---|---:|---|
| FP5-C06-01 | C06 handoff 문서 갱신 | P1 | C07 또는 다음 cluster가 받아야 하는 recovery/status 계약을 표로 정리 |
| FP5-C06-02 | recovery regression 공식 스크립트 지정 | P1 | `scripts/run_c06_v004_recovery.ps1`을 C06 공식 회귀 명령으로 기록 |
| FP5-C06-03 | release 전 simulation-only report 정책 결정 | P2 | 유지/제거/Generic gate 중 하나로 결정 |
| FP5-C06-04 | C06 완료도 체크 갱신 | P2 | 계획 대비 진행률과 남은 항목을 communication plan lineage에 연결 |
| FP5-C06-05 | 다음 cluster 진입 판단 | P2 | Datasheet 기준, 운용 계약, 검증 결과, 남은 위험을 함께 제시 |

## 3. 다음 Cluster로 넘길 계약

| 계약 ID | 인계 계약 | 근거 |
|---|---|---|
| HC-C06-01 | recovery command는 AXI 도메인 1-cycle pulse여도 TDC 도메인에서 1-cycle pulse로 보존되어야 한다. | `tdc_gpx_config_ctrl.vhd:1172`, `:1189`, v004 force/soft logs |
| HC-C06-02 | `soft_reset`은 stale response를 `PH_RESP_DRAIN`에서 처리한 뒤 `PH_INIT`으로 재진입한다. | `tdc_gpx_chip_ctrl.vhd:1010`, `:156` log |
| HC-C06-03 | `force_reinit`은 chip sub-FSM과 face/status sequencing state를 함께 recovery boundary로 닫는다. | `tdc_gpx_chip_ctrl.vhd:490`, `tdc_gpx_top.vhd:416`, `:873`, `:932` |
| HC-C06-04 | recovery 이후 output stream의 beat/tlast 총량은 보존되어야 한다. | force log `:231`~`:236`, soft log `:239`~`:244` |
| HC-C06-05 | C06 recovery 검증 범위는 I-Mode single이다. Quiet/M-mode, continuous measurement는 범위 밖이다. | 프로젝트 운용 규칙 및 v004 결과 문서 |
| HC-C06-06 | Datasheet 기반 GPX bus timing 계약은 C01/C02 결과를 따른다. C06는 외부 GPX timing을 변경하지 않는다. | `Doc/TDC-GPX-Datasheet.pdf`, C01/C02 lineage |

## 4. Timing / Latency / Throughput / Pipeline / II 관리 규칙 적용

v005 이후 C06 또는 다음 cluster 문서에는 아래 항목을 계속 포함한다.

| 항목 | 기록 방식 |
|---|---|
| Latency | command source, CDC destination, chip_ctrl action, next run T0를 분리해서 기록 |
| Throughput | recovery 전후 expected beats/run, total beats, tlast count를 stream별로 기록 |
| Pipeline | AXI command, config_ctrl CDC, chip_ctrl phase, face/status reset, output stream 순서로 도식화 |
| II | 정상 shot-to-shot II와 recovery-run-to-run interval을 분리해서 기록 |
| Timing diagram | Markdown Mermaid와 PPT 도식 모두에 포함 |

## 5. 위험과 완화

| Risk ID | 위험 | 영향 | 완화 |
|---|---|---|---|
| R5-C06-01 | simulation-only report가 장기적으로 로그를 과하게 만들 수 있음 | 회귀 로그 가독성 저하 | release 전 `g_SIM_REPORT_EN` generic 또는 제거 여부 결정 |
| R5-C06-02 | v003 recovery script가 soft-open probe semantics를 계속 보유 | 사용자가 오래된 스크립트를 잘못 사용할 수 있음 | 공식 문서는 v004 script만 회귀 명령으로 지정 |
| R5-C06-03 | C06 recovery는 64-bit top recovery TB에서 확인됨 | width별 recovery 조합 전체는 미검증 | 필요 시 C07 진입 전 recovery width sweep 추가 |
| R5-C06-04 | recovery run-to-run latency는 TB sequence 영향을 크게 받음 | RTL 최소 latency로 오해 가능 | 문서에서 TB sequence latency와 RTL command CDC latency를 분리 표기 |

## 6. 권고 진행 순서

1. `Code_Fix_Result_v004`와 PPT를 확정한다.
2. C06 handoff 문서를 갱신한다.
3. `scripts/run_c06_v004_recovery.ps1`를 공식 C06 recovery regression으로 기록한다.
4. simulation-only report 유지 정책을 사용자와 결정한다.
5. C07 진입 전, C06 계약 수락 표를 communication plan lineage에 연결한다.

## 7. v005 실행 결과 반영

| 항목 | v005 반영 위치 | 결과 |
|---|---|---|
| FP5-C06-01 C06 handoff 문서 갱신 | `C06_Control_Status_Integration_260514132259_C07_Handoff_v001.md` | Closed |
| FP5-C06-02 공식 recovery regression 지정 | `C06_Control_Status_Integration_260514132259_Code_Fix_Result_v005.md` section 3 | Closed |
| FP5-C06-03 simulation-only report 정책 | `C06_Control_Status_Integration_260514132259_Code_Fix_Result_v005.md` section 7 | Accepted / 유지 |
| FP5-C06-04 C06 완료도 체크 갱신 | `C06_Control_Status_Integration_260514132259_Code_Fix_Result_v005.md` section 8 | Closed |
| FP5-C06-05 다음 cluster 진입 판단 | `C06_Control_Status_Integration_260514132259_C07_Handoff_v001.md` section 10 | GO_WITH_CONTRACT |

최신 공식 회귀 실행:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts\run_c06_v004_recovery.ps1 -Stamp 260514132259
```

근거 archive:

```text
sim_results/vivado_xsim/sessions/260514132259_c06_v004_recovery/
```
