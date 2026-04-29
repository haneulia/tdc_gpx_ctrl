# C01 GPX Bus Read Code Verify Plan v004 리뷰

| 항목 | 내용 |
|---|---|
| 문서 버전 | v004 Plan 리뷰 |
| 작성 시각 | 2026-04-29 17:40:56 +09:00 |
| 수정 시각 | 2026-04-29 17:40:56 +09:00 |
| 검토 대상 | `C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v004.md` |
| 절대 기준 | `Doc/TDC-GPX-Datasheet.pdf` |
| 검토 목적 | 수정된 Plan v004가 Plan v003 리뷰(P-C01-V003-01..04)를 닫았는지, 승인 전 수정이 필요한지 판단 |

## 1. 종합 판단

Plan v004는 Plan v003 리뷰의 핵심 요구를 대부분 반영했다. Windows-safe timestamp, ASYNC clean elaboration 절대 경로 wrapper, mode echo/handshake fallback 제외, Git Bash/PowerShell 실행 방식 명시는 모두 적절하다.

다만 **P-C01-V004-01은 승인 전 반드시 수정해야 한다.** 현재 계획의 ASYNC marker 삽입 예시는 VHDL generate 영역에 plain `report` 문을 넣는 형태로 읽히며, 이는 VHDL concurrent 영역에서 컴파일 오류가 될 수 있다.

| 항목 | 판단 |
|---|---|
| P-C01-V003-01 Windows-safe timestamp | 반영됨 |
| P-C01-V003-02 ASYNC clean 절대 경로 | 반영됨 |
| P-C01-V003-03 fallback 강도 | 방향은 반영됨, marker 구현 방식 수정 필요 |
| P-C01-V003-04 shell 명시 | 반영됨 |
| 승인 가능성 | 조건부 승인 가능, 단 P-C01-V004-01 선반영 필요 |

## 2. 리뷰 발견 사항

### P-C01-V004-01. `gen_raw_async` 안에 plain `report`를 넣는 계획은 VHDL 문법상 위험함

| 항목 | 내용 |
|---|---|
| 심각도 | High |
| 위치 | `C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v004.md:42`, `tdc_gpx_config_ctrl.vhd:1834-1915` |
| 현상 | Plan은 `gen_raw_async` generate block 안에 `synthesis translate_off / report "ASYNC raw_cdc FIFO generate active" severity note; / synthesis translate_on`을 넣는다고 설명한다. 그러나 generate body는 concurrent statement 영역이며, plain `report`는 sequential statement라 그대로 넣으면 VHDL 컴파일 오류가 될 수 있다. |
| 영향 | 계획대로 구현하면 ASYNC evidence 보강 전 단계에서 `xvhdl`/`xelab`가 실패할 수 있다. |
| 권장 수정 | marker는 concurrent로 유효한 형태로 작성한다. 예: `process ... wait; end process;`를 사용하거나 concurrent assertion을 사용한다. 또한 합성 제외 주석은 VHDL 주석 형태인 `-- synthesis translate_off` / `-- synthesis translate_on`으로 명시한다. |

권장 구현 형태:

```vhdl
-- synthesis translate_off
p_async_marker : process
begin
    if i = 0 then
        report "ASYNC raw_cdc FIFO generate active" severity note;
    end if;
    wait;
end process;
-- synthesis translate_on
```

또는 concurrent assertion을 쓰려면 다음처럼 concurrent statement로 작성해야 한다.

```vhdl
-- synthesis translate_off
gen_async_marker_once : if i = 0 generate
begin
    assert false
        report "ASYNC raw_cdc FIFO generate active"
        severity note;
end generate gen_async_marker_once;
-- synthesis translate_on
```

### P-C01-V004-02. Marker emit 횟수는 “1회”가 아니라 chip generate 구조에 따라 달라짐

| 항목 | 내용 |
|---|---|
| 심각도 | Low |
| 위치 | `C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v004.md:42`, `tdc_gpx_config_ctrl.vhd:1819-1917` |
| 현상 | `gen_raw_async`는 `gen_chip` 내부에 있으므로 marker를 단순히 넣으면 chip 수만큼 emit될 수 있다. Plan은 “1회 emit”이라고 설명한다. |
| 영향 | 검증 grep 조건은 `1+ match`라 기능상 문제는 없지만, 문서 설명과 로그 기대치가 어긋날 수 있다. |
| 권장 수정 | 1회 marker가 필요하면 `if i = 0` 조건을 넣는다. chip별 marker가 목적이면 “chip당 1회, 총 c_N_CHIPS회”로 문서화한다. |

### P-C01-V004-03. 실행 명령의 기준 디렉터리를 명확히 해야 함

| 항목 | 내용 |
|---|---|
| 심각도 | Low |
| 위치 | `C01_GPX_Bus_Read_Code_Verify_Plan_20260429_v004.md:43-44`, `:121-122`, `:147` |
| 현상 | Plan은 `bash HDL/scripts/run_c01_regression.sh`와 `bash run_c01_regression.sh` 형식을 섞어 쓴다. 현재 작업 기준 폴더는 `HDL`이므로 `HDL/scripts/...`는 IP root에서 실행할 때만 맞고, `bash run_c01_regression.sh`는 `scripts` 폴더 또는 PATH 기준이 불명확하다. |
| 영향 | 사용자나 후속 실행자가 다른 cwd에서 실행하면 동일 명령이 실패할 수 있다. |
| 권장 수정 | 실행 기준을 하나로 고정한다. 예: IP root 기준이면 `cd C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl` 후 `bash HDL/scripts/run_c01_regression.sh`; HDL 기준이면 `cd C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL` 후 `bash scripts/run_c01_regression.sh`. PowerShell wrapper도 동일 기준으로 맞춘다. |

## 3. 좋은 변경점

| 항목 | 판단 |
|---|---|
| `$(date -u '+%Y%m%dT%H%M%SZ')` | 좋음. Windows-safe 파일명 조건을 만족한다. |
| `SIM_DIR`/`LOG_DIR` 절대 경로 wrapper | 좋음. clean evidence 위치 추적성이 좋아졌다. |
| mode echo + handshake fallback 제외 | 좋음. FIFO generate evidence 기준이 더 엄격해졌다. |
| Primary-A / Primary-B 둘 중 하나 이상 PASS 조건 | 현실적이다. 단 Primary-B marker 구현은 VHDL 문법에 맞게 수정해야 한다. |
| `C01_FORCE_NEGATIVE_STAGE1` hook 유지 | 좋음. negative evidence가 재현 가능하다. |

## 4. 승인 전 체크리스트

Plan v004 승인 전 다음 항목만 정리하면 된다.

| 체크 | 필요 조치 |
|---|---|
| ASYNC marker 문법 | plain `report` 대신 process 또는 concurrent assertion 형태로 수정 |
| marker 횟수 | `i = 0` 조건으로 1회 emit을 보장하거나 chip별 emit으로 문서화 |
| 실행 cwd | IP root 기준 또는 HDL 기준 중 하나로 통일 |

## 5. 조건부 승인 판단

Plan v004는 **조건부 승인 가능**하다. 위 3개 수정 중 특히 P-C01-V004-01은 실제 컴파일 성공 여부에 직접 영향을 주므로 선반영해야 한다. 이 부분만 고치면 v005 evidence package 진행 계획으로 충분히 사용할 수 있다.

권장 승인 문구:

> Plan v004는 조건부 승인 가능하다. ASYNC marker를 VHDL concurrent 영역에서 유효한 process/assert 형태로 바꾸고, marker emit 횟수와 실행 기준 디렉터리를 명확히 한 뒤 진행한다.

## 6. C02 진입 영향

이 Plan v004도 C01 기능 결함을 다루는 것이 아니라 closure evidence 보강 계획이다. 따라서 C02 진입 자체는 계속 가능하다. 다만 v005 evidence package를 먼저 완료하면 C01 closure 선언을 더 강하게 가져갈 수 있다.
