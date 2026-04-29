# C01 GPX Bus Read Code Verify Plan v005 리뷰

| 항목 | 내용 |
|---|---|
| 문서 버전 | v005 Plan 리뷰 |
| 작성 시각 | 2026-04-29 17:50:25 +09:00 |
| 수정 시각 | 2026-04-29 17:50:25 +09:00 |
| 검토 대상 | `C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v005.md` |
| 절대 기준 | `Doc/TDC-GPX-Datasheet.pdf` |
| 검토 목적 | Plan v005가 Plan v004 리뷰(P-C01-V004-01..03)를 닫았는지, 실행 승인 가능한지 판단 |

## 1. 종합 판단

Plan v005는 승인 가능하다. 직전 리뷰의 핵심 blocker였던 plain `report` 문제를 concurrent assertion + `if i = 0 generate` 구조로 바꾸었고, marker emit 횟수와 실행 cwd도 명확히 정리했다.

남은 항목은 실행 전 문서/구현 스타일을 더 단단하게 만드는 minor 권고이며, 진행을 막는 blocker는 아니다.

| 항목 | 판단 |
|---|---|
| P-C01-V004-01 plain `report` 컴파일 위험 | 닫힘. concurrent assertion 방식으로 변경 |
| P-C01-V004-02 marker emit 횟수 | 닫힘. `i = 0` 한정 |
| P-C01-V004-03 cwd 혼재 | 닫힘. IP root 기준으로 통일 |
| 승인 가능성 | 승인 가능 |

## 2. 확인된 좋은 변경점

| 항목 | 판단 |
|---|---|
| ASYNC marker 구현 | plain sequential report를 제거하고 concurrent assertion으로 정정 |
| marker scope | chip 0에서만 1회 emit되도록 `if i = 0 generate` 적용 |
| 실행 cwd | `C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl` IP root 기준으로 통일 |
| negative transcript | Windows-safe timestamp 유지 |
| closure 조건 | Primary-A 또는 Primary-B 중 하나 이상 PASS 조건으로 명확화 |
| compile 검증 | `xvhdl --incr --relax -prj __c01_all_vhdl.prj`로 marker 문법 검증 계획 포함 |

## 3. Minor 권고

### P-C01-V005-01. Nested generate에는 `begin`을 명시하는 편이 repo 스타일과 맞음

| 항목 | 내용 |
|---|---|
| 심각도 | Low |
| 위치 | `C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v005.md:3.1`, `tdc_gpx_config_ctrl.vhd:1821`, `tdc_gpx_config_ctrl.vhd:1834` |
| 판단 | VHDL-2008에서는 간단한 generate body가 가능하지만, 현재 파일의 `gen_raw_sync`/`gen_raw_async`는 모두 `generate` 뒤에 `begin`을 명시한다. nested marker도 같은 스타일로 쓰는 것이 안전하다. |
| 권장 구현 | `gen_async_marker : if i = 0 generate begin ... end generate gen_async_marker;` 형태로 작성한다. |

권장 형태:

```vhdl
-- synthesis translate_off
gen_async_marker : if i = 0 generate
begin
    assert false
        report "ASYNC raw_cdc FIFO generate active"
        severity note;
end generate gen_async_marker;
-- synthesis translate_on
```

### P-C01-V005-02. PowerShell wrapper는 `bash -lc` 내부에서 `cd`까지 포함하면 더 재현성이 좋음

| 항목 | 내용 |
|---|---|
| 심각도 | Low |
| 위치 | `C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v005.md:3`, `:7` |
| 판단 | Plan은 PowerShell에서 먼저 `cd` 후 `bash -lc '...'`를 실행하는 형태를 제시한다. 대체로 동작하겠지만, login shell 동작 차이를 피하려면 `bash -lc` 내부에도 `cd`를 포함하는 것이 더 안전하다. |
| 권장 예시 | `bash -lc 'cd /c/Projects/my_sp/lib/IP/tdc_gpx_ctrl && C01_FORCE_NEGATIVE_STAGE1=1 HDL/scripts/run_c01_regression.sh --negative'` |

### P-C01-V005-03. ASYNC marker note는 v005 결과 보고서에서 expected note로 분류해야 함

| 항목 | 내용 |
|---|---|
| 심각도 | Trivial |
| 판단 | `assert false ... severity note`는 정상적으로 note를 남기기 위한 장치다. v005 결과 보고서의 allow/expected log 항목에 “ASYNC raw_cdc FIFO generate active”를 expected note로 기록하면 후속 리뷰에서 혼선이 줄어든다. |

## 4. 승인 판단

Plan v005는 진행 승인 가능하다. 위 minor 권고는 실행 안정성과 문서 선명도를 높이는 항목이며, 계획의 핵심 방향을 바꾸지 않는다.

권장 승인 문구:

> Plan v005 승인 가능. ASYNC marker는 concurrent assertion + `if i = 0 generate` 구조로 진행하되, repo 스타일에 맞춰 nested generate에 `begin`을 명시하고 v005 결과 보고서에 marker note를 expected evidence로 기록한다.

## 5. C02 진입 영향

이 계획은 C01 기능 결함을 다루는 것이 아니라 closure evidence package를 완성하는 작업이다. C02 진입 판단은 계속 가능하며, Plan v005 실행 후에는 C01 closure 선언을 더 강하게 확정할 수 있다.
