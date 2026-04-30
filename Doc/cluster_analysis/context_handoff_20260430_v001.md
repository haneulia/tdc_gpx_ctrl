# Context Handoff

문서 버전: `v001`  
작성일: `2026-04-30`  
최종 수정 시간: `2026-04-30 11:26:53 +09:00`  
작성 목적: 대화 문맥이 압축/요약된 상태로 이어지고 있음을 기록하고, C02 이후 분석을 같은 기준으로 계속하기 위한 최소 인계 정보를 보존한다.

---

## 1. 발생 상황

사용자 확인:

> 문맥이 꽉찬것 같은데 맞아?

판단:

- 맞다. 현재 대화는 원문 전체가 아니라 압축/요약된 문맥을 기반으로 이어지고 있다.
- 다만 프로젝트 운영 규칙에 따라 주요 판단과 산출물은 `Doc/cluster_analysis` 아래 Markdown/PPT로 남아 있어 작업 연속성은 문서에서 복원 가능하다.
- 본 handoff 문서는 이 전환 지점을 시간순 대화 기록으로 고정하기 위해 생성했다.

---

## 2. 확정된 운영 규칙

| 규칙 | 현재 적용 상태 |
|---|---|
| 절대 기준 | `Doc/TDC-GPX-Datasheet.pdf`가 최상위 기준이다. 코드 주석보다 우선한다. |
| 문서 위치 | 모든 Cluster 소통 기록은 `Doc/cluster_analysis` 아래에 둔다. |
| 버전 관리 | 기존 파일을 덮어쓰지 않고 새 version 파일을 만든다. |
| 근거 추적 | 데이터시트 page, RTL file/line, test/log 근거를 문서에 남긴다. |
| 분석 필수 항목 | latency, throughput, pipeline, II, timing diagram/block diagram을 포함한다. |
| Review 처리 | Review → Plan → 사용자 승인 → 실행/회귀 → Result 순서로 진행한다. |
| Context 부족 시 | 명확히 보고하고 handoff 문서를 생성한 뒤 다음 섹션으로 이어간다. |

운영 프로토콜 기준 문서:

- `Doc/cluster_analysis/cluster_analysis_operating_protocol_20260429_v006.md`

---

## 3. 완료된 Cluster 상태

### C01_GPX_Bus_Read

상태: closure 완료, C02 진입 승인 가능 상태로 닫힘.

주요 산출물:

- `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_20260429_v009.md`
- `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_20260429_v005.md`
- `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_Review_20260429_v005.md`
- `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v006.md`

C01에서 C02로 인계된 핵심:

| 인계 항목 | 의미 |
|---|---|
| Empty FIFO read 금지 | C02 최우선 검증 항목. `EF1/EF2 active HIGH = empty`. |
| `tS-EF max 11.8 ns` | 마지막 read 후 EF pin 반영 지연. 200 MHz 기준 raw guard 3 clocks, sync 관측은 더 늦다. |
| Burst READ II | 최선 조건 `bus_ticks * bus_clk_div * Tclk`; 실제는 backpressure 의존. |
| `o_rsp_pending` 등록화 영향 | `PH_RESP_DRAIN` exit와 non-burst/burst II 영향 확인 필요. |
| Stream clock 전략 | `i_tdc_clk`와 output stream clock 분리. ASYNC mode는 FIFO 필요. |

---

## 4. 현재 Cluster: C02_Chip_Acquisition

현재 상태: C02 분석 v001 생성 완료, 사용자 검토 대기.

산출물:

- `Doc/cluster_analysis/C02_Chip_Acquisition/C02_Chip_Acquisition_260429203421_Plan_v001.md`
- `Doc/cluster_analysis/C02_Chip_Acquisition/C02_Chip_Acquisition_260429203421_Analysis_v001.md`
- `Doc/cluster_analysis/C02_Chip_Acquisition/C02_Chip_Acquisition_260429203421_Analysis_v001.pptx`

C02 v001 핵심 판단:

1. 현재 C02 구조는 `PH_RUN` 안에서 shot을 반복하는 acquisition loop이다.
2. `PH_RESP_DRAIN`은 shot마다 수행되는 단계가 아니라 run 종료, cfg/reg 완료, soft reset, timeout 이후 stale response를 flush하는 phase이다.
3. 현재 nominal 운용은 데이터시트 관점에서 `non-quiet mode + Reg5 MasterAluTrig cleanup` 전제로 해석하는 것이 가장 합리적이다.
4. 가장 큰 위험은 `empty Interface FIFO read 금지`가 아직 검증으로 닫히지 않았다는 점이다.
5. `expected_ififo1/2`는 burst sizing과 mismatch 진단에는 쓰이지만, single EF drain의 hard read bound로는 쓰이지 않는다.
6. C01 보완 후 stream CDC는 `g_STREAM_CLK_MODE="ASYNC"` 기본값과 `xpm_fifo_async` 경로로 반영되어 있다.

---

## 5. 현재 열린 C02 질문

| ID | 질문 | 추천 방향 |
|---|---|---|
| Q-C02-01 | nominal GPX mode를 non-quiet + MasterAluTrig cleanup으로 고정할 것인가? | 우선 고정 권장. Quiet/M-mode는 별도 확장으로 분리. |
| Q-C02-02 | `expected_ififo`를 read hard bound로 승격할 것인가? | count valid가 보장되는 mode에서 옵션 또는 hard guard로 검토. |
| Q-C02-03 | TB에서 empty FIFO read를 fatal assertion으로 처리할 것인가? | Yes 권장. 데이터시트 절대 기준. |
| Q-C02-04 | EF fallback guard를 늘릴 것인가? | assertion/waveform 확인 후 결정. |
| Q-C02-05 | C02 코드 보완은 별도 plan 승인 후 진행할 것인가? | 운영 프로토콜 v006대로 진행. |

---

## 6. C02 Review Finding 요약

| Finding | 요약 | 위험도 |
|---|---|---|
| F-C02-01 | Empty FIFO read 금지가 검증으로 닫히지 않음. TB가 extra read를 허용하고 있어 데이터시트와 충돌 가능. | 높음 |
| F-C02-02 | Quiet/M-mode를 허용하면 현재 `capture -> drain -> AluTrigger` 순서가 데이터시트 흐름과 맞지 않음. | 중간~높음 |
| F-C02-03 | `expected_ififo`가 hard bound가 아니라 burst/mismatch 보조로만 사용됨. | 중간 |
| F-C02-04 | TB raw count가 data beat와 control beat를 분리하지 않음. | 중간 |
| F-C02-05 | `g_STREAM_CLK_MODE="SYNC"`는 외부 clock 계약 없이는 위험. ASYNC default는 합리적. | 낮음~중간 |

---

## 7. 다음 진행 제안

사용자 피드백을 받은 뒤 다음 중 하나로 진행한다.

1. C02 분석 v001에 대한 사용자 review는 v008 이후 `<ClusterName>_<YYMMDDHHMMSS>_<ArtifactName>_Review_vNNN.md` 형식으로 기록한다.
2. 사용자가 F-C02-01/F-C02-04 보완에 동의하면 `C02_Chip_Acquisition_260430124944_Code_Fix_Plan_v001.md`를 먼저 생성한다.
3. Plan 승인 후 TB assertion, data/control monitor 분리, 필요 RTL guard 보완을 진행한다.
4. xsim 회귀 결과를 `C02_Chip_Acquisition_260430155825_Code_Verify_v001.md`로 기록한다.

---

## 8. 작업 재개 기준

새 대화 또는 다음 세션에서 이어갈 때는 다음 순서로 복원한다.

1. `cluster_analysis_operating_protocol_20260429_v006.md` 확인
2. `C02_Chip_Acquisition_260429203421_Analysis_v001.md` section 7~9 확인
3. 사용자에게 Q-C02-01~Q-C02-03에 대한 판단을 받아 code-fix plan 작성

본 handoff 기준으로 현재 상태는 “C02 분석 v001 완료, 사용자 검토 및 보완 방향 승인 대기”이다.
