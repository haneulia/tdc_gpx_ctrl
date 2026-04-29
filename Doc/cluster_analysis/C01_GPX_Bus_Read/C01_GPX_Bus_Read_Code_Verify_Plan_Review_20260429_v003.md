# C01 GPX Bus Read Code Verify Plan v003 리뷰

| 항목 | 내용 |
|---|---|
| 문서 버전 | v003 Plan 리뷰 |
| 작성 시각 | 2026-04-29 17:33:07 +09:00 |
| 수정 시각 | 2026-04-29 17:33:07 +09:00 |
| 검토 대상 | `C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v003.md` |
| 절대 기준 | `Doc/TDC-GPX-Datasheet.pdf` |
| 검토 목적 | Plan v003가 Plan v002 리뷰(P-C01-V002-01..04)를 충분히 반영했는지, 승인 전 보완이 필요한지 판단 |

## 1. 종합 판단

Plan v003는 Plan v002 리뷰에서 지적한 핵심 모호점을 대부분 해소했다. 특히 closure 문구 단일화, `--negative`의 책임 분리, 임시 파일 수정 대신 `C01_FORCE_NEGATIVE_STAGE1` hook을 쓰는 방식, ASYNC clean elaboration option B 확정은 타당하다.

다만 승인 전 아래 3가지는 명확히 고치는 것이 좋다.

| 항목 | 판단 |
|---|---|
| P-C01-V002-01 closure 문구 단일화 | 반영됨 |
| P-C01-V002-02 negative injection 위치 | 반영됨, env hook 방식이 더 안전 |
| P-C01-V002-03 ASYNC clean option 확정 | 부분 반영, 실행 경로/검증 fallback 보완 필요 |
| P-C01-V002-04 `--negative` 의미 분리 | 반영됨 |
| 승인 가능성 | 조건부 승인 가능 |

## 2. 리뷰 발견 사항

### P-C01-V003-01. Negative transcript timestamp는 Windows-safe 형식으로 고정해야 함

| 항목 | 내용 |
|---|---|
| 심각도 | Medium |
| 위치 | `C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v003.md:40`, `:42`, `:117` |
| 현상 | transcript 파일명이 `run_c01_regression_negative_<UTC_ISO_TS>.log`로 정의되어 있다. ISO timestamp를 `2026-04-29T08:10:00Z`처럼 만들면 Windows 파일명에서 `:` 문자가 문제가 될 수 있다. |
| 영향 | negative transcript 생성이 실패하거나 파일명이 의도와 다르게 처리될 수 있다. |
| 권장 수정 | timestamp 형식을 파일명 안전 형식으로 고정한다. 예: `date -u '+%Y%m%dT%H%M%SZ'`, 결과 파일명 `run_c01_regression_negative_20260429T081002Z.log`. |

### P-C01-V003-02. ASYNC clean elaboration 명령은 실행 디렉터리와 로그 절대 경로를 명시해야 함

| 항목 | 내용 |
|---|---|
| 심각도 | Medium |
| 위치 | `C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v003.md:44`, `:124-126` |
| 현상 | xelab 단발 명령은 제시되어 있지만 `cd SIM_DIR` 또는 `-log "${LOG_DIR}/elab_tb_tdc_gpx_config_ctrl_ASYNC_clean.log"` 같은 실행 위치/로그 절대 경로가 명시되어 있지 않다. |
| 영향 | 실행 위치에 따라 clean log가 `tmp/c01_verify`가 아닌 다른 디렉터리에 생성될 수 있고, v005에서 추적해야 할 evidence path가 어긋날 수 있다. |
| 권장 수정 | Plan에 실행 wrapper를 명시한다. 예: `SIM_DIR=.../xsim`, `LOG_DIR=.../HDL/tmp/c01_verify`, `cd "${SIM_DIR}"`, `-log "${LOG_DIR}/elab_tb_tdc_gpx_config_ctrl_ASYNC_clean.log"`. arg-file을 쓴다면 `__elab_ASYNC_clean.f` 위치도 `SIM_DIR`로 고정한다. |

### P-C01-V003-03. ASYNC fallback B는 FIFO generate 증거로는 약함

| 항목 | 내용 |
|---|---|
| 심각도 | Medium |
| 위치 | `C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v003.md:48-56` |
| 현상 | Primary가 실패하면 fallback B로 ASYNC mode echo와 raw_axis handshake cross-check를 제시한다. 그러나 mode echo와 handshake는 ASYNC mode smoke PASS의 증거이지, `xpm_fifo_async` generate가 실제 elaboration되었다는 1차 증거로는 약하다. |
| 영향 | R-C01-V004-02 / R-C01-CL-03을 닫는 근거가 다시 애매해질 수 있다. |
| 권장 수정 | Closure 조건은 `Primary PASS` 또는 `명시적 ASYNC FIFO branch marker PASS`로 제한한다. Primary가 실패하면 TB/RTL에 비기능 report marker를 추가하거나, xelab/compile artifact에서 `xpm_fifo_async` 인스턴스 경로를 직접 찾는 방법으로만 fallback을 인정한다. mode echo + handshake는 보조 근거로만 분류한다. |

### P-C01-V003-04. `C01_FORCE_NEGATIVE_STAGE1=1 bash ...` 실행 shell을 명시하면 재현성이 좋아짐

| 항목 | 내용 |
|---|---|
| 심각도 | Low |
| 위치 | `C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v003.md:42`, `:115` |
| 현상 | 명령은 Bash prefix env syntax를 사용한다. PowerShell에서 그대로 실행하면 의미가 다르다. |
| 판단 | 현재 회귀 스크립트가 Bash 기반이므로 방향은 맞다. |
| 권장 수정 | “Git Bash/MSYS bash에서 실행” 또는 PowerShell용 wrapper 예시를 추가한다. 예: `bash -lc 'C01_FORCE_NEGATIVE_STAGE1=1 HDL/scripts/run_c01_regression.sh --negative'`. |

## 3. 좋은 변경점

| 항목 | 판단 |
|---|---|
| `--negative`를 transcript 분기 전용으로 제한 | 좋음. 책임이 명확해짐. |
| `C01_FORCE_NEGATIVE_STAGE1` hook 사용 | 좋음. 임시 파일 수정/원복보다 안전하고 재현 가능함. |
| summary echo 직전 hook 위치 지정 | 좋음. 기대 transcript와 실제 출력이 맞을 가능성이 높음. |
| v005 closure 문구 단일안 | 좋음. “보강 권장/완료” 충돌이 해소됨. |
| Plan v002 -> v003 forward trace | 확인됨. `Plan v002` 끝에 반영 위치 기록 존재. |

## 4. 승인 전 체크리스트

Plan v003 승인 전 다음만 보완하면 된다.

| 체크 | 필요 조치 |
|---|---|
| negative log 파일명 | `UTC_ISO_TS`를 Windows-safe timestamp로 명시 |
| ASYNC clean command | `SIM_DIR`, `LOG_DIR`, `cd`, `-log` 절대 경로를 명시 |
| ASYNC fallback | mode echo + handshake를 closure fallback에서 제외하거나 보조 근거로 낮춤 |
| shell 재현성 | negative 명령 실행 shell을 Bash로 명시 |

## 5. 조건부 승인 판단

Plan v003는 **조건부 승인 가능**하다. 위 4개 보완은 설계 방향을 바꾸는 것이 아니라 실행 재현성과 evidence 강도를 높이는 정리다. 보완 후에는 v005 evidence package를 진행해도 된다.

권장 승인 문구:

> Plan v003는 조건부 승인 가능하다. negative transcript timestamp를 Windows-safe 형식으로 고정하고, ASYNC clean elaboration의 실행 디렉터리/로그 절대 경로 및 fallback 인정 기준을 보완한 뒤 진행한다.

## 6. C02 진입 영향

이 Plan v003 역시 C01 기능 결함을 다루는 계획이 아니라 closure evidence 보강 계획이다. 따라서 C02 진입 자체는 계속 가능하다. 다만 v005 evidence pack을 먼저 완료하면 C01 closure 선언을 더 강하게 가져갈 수 있다.
