# TDC-GPX Cluster 단계별 분석 소통 계획 v003

| 항목 | 내용 |
|---|---|
| 문서 버전 | v003 |
| 생성 시간 | 2026-05-14 12:20:00 KST |
| 수정 시간 | 2026-05-14 12:20:00 KST |
| 이전 문서 | `Doc/cluster_analysis/cluster_analysis_communication_plan_260511200655_v002.md` |
| 기준 문서 | `Doc/TDC-GPX-Datasheet.pdf` |
| 목적 | Cluster 분석/수정/검증 소통 방식에 Vivado/xsim 산출물 정리 및 archive 경로 추적 방식을 추가한다. |

## 1. 소통 원칙

Cluster 분석은 한 번에 전체를 닫는 방식이 아니라 GPX IC에서 읽어오는 단계부터 논리적으로 Cluster를 나누고, 각 Cluster를 사용자와 함께 검토한 뒤 다음 Cluster로 넘어간다.

각 Cluster 문서는 `Doc/cluster_analysis/<Cluster>` 폴더에 Markdown과 PPT로 기록한다.

Markdown은 상세 판단 근거, 표, 도표, line 근거를 기록한다. PPT는 핵심 구조, timing diagram, pipeline, data flow를 추상화해 공유한다.

## 2. Vivado/xsim 산출물 소통 규칙

앞으로 Vivado/xsim 검증 결과를 공유할 때는 루트에 흩어진 log 이름만 말하지 않고 session archive 경로를 함께 기록한다.

표준 기록 형식:

| 항목 | 기록 예 |
|---|---|
| 실행 script | `scripts\run_c06_v004_recovery.ps1` |
| 실행 stamp | `260514115000` |
| archive session | `sim_results/vivado_xsim/sessions/260514115000_c06_v004_recovery/` |
| compile log | `logs/compile/xvhdl_c06_v002_260514115000.log` |
| simulation log | `logs/simulate/xsim_c06_v004_top_int_soft_260514115000.log` |
| waveform | `waves/tb_c06_v004_top_int_soft_snap.wdb` |
| 이동 이력 | `manifest.csv` |

## 3. 실행 후 정리 소통 절차

Vivado/xsim 실행이 끝나면 Codex는 다음 순서로 보고한다.

1. PASS/FAIL과 주요 marker를 먼저 요약한다.
2. 실행 산출물이 archive된 session 경로를 알려준다.
3. 문서에 사용할 log line 근거를 기록한다.
4. 루트에 남은 generated artifact가 있는지 확인한다.
5. 필요한 경우 `scripts/archive_vivado_xsim_outputs.ps1`를 재실행해 정리한다.

## 4. Plan-Result-Rollover 유지

Fix Plan은 실행 가능한 계획이고, Result는 실행 결과이며, 다음 Plan은 Result에서 남은 항목을 추적한다.

```text
Fix Plan vN
  -> Code/Test/Doc 실행
  -> Result vN 작성
  -> Fix Plan vN에 반영 위치 기록
  -> Open/Partial/New 항목을 Fix Plan vN+1로 rollover
  -> Cluster Handoff 또는 다음 Cluster 진입
```

Vivado/xsim 결과가 포함된 Result 문서에는 archive session 경로를 함께 남겨 이후 사용자가 동일 로그를 찾아갈 수 있게 한다.

## 5. v002 -> v003 변경 이력

| 항목 | v003 반영 위치 | 변경 내용 |
|---|---|---|
| Vivado/xsim archive 소통 | section 2 | session 경로, log, waveform, manifest 기록 형식 추가 |
| 실행 후 정리 절차 | section 3 | PASS/FAIL 보고 후 archive 경로 공유와 루트 정리 확인 절차 추가 |
| Plan-Result-Rollover 유지 | section 4 | Result 문서의 xsim 근거를 archive session과 연결하도록 보완 |
