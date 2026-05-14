# Cluster Analysis Operating Protocol v013

| 항목 | 내용 |
|---|---|
| 문서 버전 | v013 |
| 생성 시간 | 2026-05-14 12:20:00 KST |
| 수정 시간 | 2026-05-14 12:20:00 KST |
| 기준 문서 | `Doc/TDC-GPX-Datasheet.pdf` |
| 이전 문서 | `Doc/cluster_analysis/cluster_analysis_260511200655_operating_protocol_v012.md` |
| 목적 | Vivado/xsim 실행 산출물의 폴더 구조, 보관/정리 절차, 문서 근거 경로 기록 규칙을 추가한다. |

## 1. 최상위 기준

기능 해석, timing 판단, 운용 개념 보완, 코드 리뷰 판단의 최상위 기준은 항상 `Doc/TDC-GPX-Datasheet.pdf`이다.

우선순위는 다음과 같다.

| 우선순위 | 기준 | 적용 원칙 |
|---:|---|---|
| 1 | `Doc/TDC-GPX-Datasheet.pdf` | GPX IC timing, mode, register, 금지 조건의 절대 기준 |
| 2 | 현재 RTL 구현 | Datasheet 요구가 현재 코드에서 어떻게 구현되었는지 확인하는 기준 |
| 3 | Testbench / Vivado xsim log | 구현 의도와 실제 검증 범위를 확인하는 기준 |
| 4 | 기존 문서 / 주석 | 참고 자료이며 Datasheet와 충돌하면 Datasheet가 우선 |

## 2. Vivado/xsim 기준 경로

Vivado/xsim 기준 설치 경로는 아래로 고정한다.

```text
C:\AMDDesignTools\2025.2.1\Vivado
```

사용 실행 파일:

```text
C:\AMDDesignTools\2025.2.1\Vivado\bin\xvlog.bat
C:\AMDDesignTools\2025.2.1\Vivado\bin\xvhdl.bat
C:\AMDDesignTools\2025.2.1\Vivado\bin\xelab.bat
C:\AMDDesignTools\2025.2.1\Vivado\bin\xsim.bat
```

## 3. Vivado/xsim 산출물 폴더 구조

Vivado/xsim 실행 산출물은 루트 디렉터리에 계속 남기지 않는다. 실행 완료 또는 실패 후 session archive로 이동한다.

표준 구조:

```text
sim_results/
  vivado_xsim/
    README.md
    sessions/
      <YYMMDDHHMMSS>_<label>/
        logs/
          compile/
          elaborate/
          simulate/
          vivado/
          journal/
        waves/
        work/
        tmp/
        crash/
        manifest.csv
        README.md
```

폴더 의미:

| 폴더 | 보관 대상 |
|---|---|
| `logs/compile` | `xvlog*.log`, `xvhdl*.log` |
| `logs/elaborate` | `xelab*.log` |
| `logs/simulate` | `xsim*.log` |
| `logs/vivado` | `vivado*.log` |
| `logs/journal` | `*.jou` |
| `waves` | `*.wdb` |
| `work` | `*.pb`, `xsim.dir`, `.Xil`, legacy simulation work directory |
| `tmp` | script-generated temporary argument/project files |
| `crash` | `hs_err_pid*.log`, `hs_err_pid*.dmp` |

## 4. 실행 후 정리 규칙

모든 신규 Vivado/xsim 실행 스크립트는 종료 시 아래 script를 호출한다.

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts\archive_vivado_xsim_outputs.ps1 -Stamp <YYMMDDHHMMSS> -Label <run_label>
```

운영 규칙:

- 기본 정책은 삭제가 아니라 이동 보관이다.
- `xsim.dir`, `.Xil`, `tmp`, `.wdb`, `.pb`, `.jou`, log 파일은 실행 완료 후 session archive로 이동한다.
- 스크립트가 실패하더라도 `finally` block에서 archive script를 호출한다.
- nested regression 구조에서는 하위 script가 중간 산출물을 먼저 이동하지 않도록 `-NoArchiveOnExit` 옵션을 사용하고, 최상위 script가 한 번만 archive한다.
- archive script는 Git tracked file 또는 tracked file을 포함한 directory를 이동하지 않는다.
- `sim_results/vivado_xsim/sessions/`는 재생성 가능한 산출물이므로 Git에 포함하지 않는다.

## 5. 문서 근거 기록 규칙

시뮬레이션 결과가 판단 근거가 될 때는 문서에 다음을 함께 기록한다.

| 항목 | 기록 예 |
|---|---|
| 실행 명령 | `scripts\run_c06_v004_recovery.ps1 -Stamp 260514115000` |
| session archive | `sim_results/vivado_xsim/sessions/260514115000_c06_v004_recovery/` |
| log 파일 | `logs/simulate/xsim_c06_v004_top_int_soft_260514115000.log` |
| line 근거 | `:142`, `:146`, `:244`처럼 line 번호 |
| PASS/FAIL 기준 | testbench marker 또는 script assertion 조건 |

기존 문서가 루트 log 파일명만 기록한 경우에는 동일 basename을 session archive의 `manifest.csv`에서 찾아 추적한다.

## 6. Testbench 및 RTL 규칙 유지

- RTL은 합성 가능한 VHDL로 작성한다.
- testbench 전용 구문은 testbench 또는 `-- synthesis translate_off/on` 영역에만 둔다.
- AXI4-Lite CSR read/write helper는 새로 만들지 않고 `px_utility_pkg.vhd`를 사용한다.
- 조합논리는 원칙적으로 제한하고, 모듈/handshake/timing boundary는 FF/register로 닫는다.
- 예외적으로 조합논리를 쓰는 경우 2-depth 이하로 제한하고, timing 분석에 영향을 주면 finding 또는 fix plan으로 관리한다.

## 7. 문서 산출물 규칙 유지

Cluster 산출물은 `Doc/cluster_analysis/<Cluster>` 아래에 Markdown과 PPT로 기록한다.

파일명에는 `YYMMDDHHMMSS` timestamp를 포함한다.

모든 분석/결과 문서는 다음 항목을 포함한다.

| 항목 | 필수 내용 |
|---|---|
| Timing diagram/block | 사용자와 구조적으로 소통 가능한 timing diagram 또는 block diagram |
| Latency | 입력 기준부터 출력/상태 반영까지 cycle/time |
| Throughput | 정상/stall 조건의 처리량 |
| Pipeline | stage, FF/register boundary, data/control flow |
| II | transaction/shot/frame initiation interval |
| 근거 | Datasheet, RTL line, TB, xsim log archive |
| 수정 시간 | 문서 본문에 생성/수정 시간 기록 |

## 8. Git 관리 규칙

- 코드, script, 문서 정책 파일은 Git으로 관리한다.
- Vivado/xsim generated artifact는 Git에 포함하지 않는다.
- commit 전 `git status --short`, `git diff --stat`, 필요 시 `git diff --check`로 범위를 확인한다.
- unrelated dirty file 또는 사용자 작업물은 stage하지 않는다.

## 9. v012 -> v013 변경 이력

| 항목 | v013 반영 위치 | 변경 내용 |
|---|---|---|
| Vivado/xsim folder structure | section 3 | `sim_results/vivado_xsim/sessions/<stamp>_<label>` 구조 추가 |
| 실행 후 archive 규칙 | section 4 | 실행 완료/실패 후 generated artifacts 이동 보관 규칙 추가 |
| nested regression 처리 | section 4 | 하위 script는 `-NoArchiveOnExit`, 최상위 script가 archive |
| log 근거 기록 방식 | section 5 | session archive 경로와 line 근거 기록 |
| Git 정책 | section 8 | generated artifact는 Git 제외, script/policy만 관리 |
