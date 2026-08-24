# V2/V3 Vivado 합성 및 구현 설정 기록

## 1. 문서 목적

이 문서는 `tdc_gpx_lidar_ctrl` V2와 V3 Parent 프로젝트가 Vivado 기본 합성·구현
설정에서 무엇을 변경했는지 기록한다. 또한 Vivado GUI에 보이는 Run 설정과 실제
Sign-off 설정이 일치하는지 판단하는 방법을 설명한다.

- 확인 도구: Vivado 2025.2.1
- 확인일: 2026-08-24
- 대상 FPGA: `xc7z020clg484-2`
- 32-bit와 64-bit Profile은 같은 합성·구현 전략을 사용한다.
- `G_OUTPUT_WIDTH`와 연결된 AXI4-Stream/VDMA 폭만 Profile별로 달라진다.

## 2. 핵심 결론

| 구분 | V2 | V3 |
|---|---|---|
| 합성 실행 방식 | Vivado Project Run `synth_1` | Tcl 직접 실행 `synth_design` |
| 합성 전략 | `Vivado Synthesis Defaults` | 전체 Block Design 직접 합성 |
| 구현 실행 방식 | Vivado Project Run `impl_1` | Tcl 직접 `opt/place/phys_opt/route` 실행 |
| 구현 전략 | `Performance_Explore` | 단계별 비기본 directive와 조건부 재배선 |
| GUI Design Runs | 실제 실행 결과를 표시 | 기본 Run은 `Not started`로 남음 |
| 최종 결과 | `.runs/synth_1`, `.runs/impl_1` | `post_synth.dcp`, `post_route.dcp`, BIT, XSA |

V2에서는 Parent `.xpr`의 Project Summary와 Design Runs가 실제 Sign-off 실행 상태를
표시한다. V3에서는 Parent `.xpr`가 편집 가능한 설계 원본이고, 실제 Sign-off는 별도
Tcl 흐름이 수행한다. 따라서 V3 Project Summary에 보이는 기본 전략과 `Not started`
상태를 최종 Sign-off 상태로 해석하면 안 된다.

### 2.1 기본 옵션을 변경한 이유

Vivado 기본 옵션은 일반 설계에서 빠르게 시작하기 위한 기준값이지, 이 설계의
`xc7z020clg484-2`, TDC 200 MHz, GPX IOB 고정 조건에 맞춘 Sign-off 보장은 아니다.
옵션 변경은 RTL 기능을 바꾸기 위한 것이 아니라 다음 세 문제를 해결하기 위한 것이다.

1. Windows 장경로와 하위 IP 생성 실패를 피하고 같은 Source를 재현성 있게 합성한다.
2. 배선 지연 비중이 큰 TDC 200 MHz 경로의 Setup 여유를 확보한다.
3. 모든 Net을 무조건 복제하지 않고, 첫 배선 결과가 기준에 미달할 때만 물리 보정을
   수행한다.

| 설정 | 기본값 유지 여부 | 변경 이유와 실제 근거 | 대가 |
|---|---|---|---|
| V2 합성 `Vivado Synthesis Defaults` | 유지 | 기본 합성으로 Black Box와 Latch 없이 Source closure가 가능했으므로 변경 근거가 없다. | 없음 |
| V2 구현 `Performance_Explore` | 변경 | 과거 동일 RTL/XDC OOC 비교에서 기본 구현은 WNS `-0.084 ns`, TNS `-0.192 ns`, 5개 Setup endpoint 실패였고 Timing Explore는 WNS `+0.041 ns`로 닫혔다. 최신 4-chip Parent도 배선 지연 비중이 `85.363%`이고 TDC 200 MHz WNS가 `+0.240 ns`이므로 배치·배선 탐색을 유지한다. | 구현시간 증가, 결과 변동 가능성 증가 |
| V3 `general.maxThreads 4` | 변경 | 논리 최적화가 아니라 Windows 호스트의 CPU·메모리 사용량과 Vivado 안정성을 제한하는 실행 자원 정책이다. | 더 많은 Core를 가진 PC에서는 실행시간이 길 수 있음 |
| V3 `synth_checkpoint_mode None` | 변경 | 하위 OOC Run의 긴 생성 경로와 캐시 DCP 의존을 피하고, HLS 생성 RTL을 포함한 전체 Source가 현재 상태로 합성되는지 한 번에 검증한다. 실제로 Windows 260자 경로와 IP Reset/Output Product 문제가 발생한 이력이 있다. | 증분/OOC 재사용이 줄어 전체 합성시간 증가 |
| V3 `generate_target all` | 명시 실행 | Block Design 및 하위 IP 생성물이 누락된 상태에서 합성을 시작하지 않도록 준비 단계를 고정한다. 타이밍 최적화 옵션은 아니다. | 생성 단계 시간이 추가됨 |
| V3 `flatten_hierarchy rebuilt` | 변경 | 계층 경계를 넘는 논리 최적화를 허용하면서, 결과 Netlist에서는 분석 가능한 계층 이름을 다시 구성한다. | 원본 RTL과 합성 후 계층명이 완전히 같지 않을 수 있음 |
| V3 `ExploreWithRemap/Explore/AggressiveExplore` | 변경 | TDC 200 MHz의 최악 경로가 논리 단계보다 배선에 지배되므로 기본 배치·배선 한 번보다 더 넓은 물리 해를 탐색한다. 최신 32-bit 첫 배선의 TDC Reset WNS는 `-0.001 ns`였으며 조건부 보정 후 Reset `+0.367 ns`, 일반 경로 `+0.227 ns`, 전체 `+0.222 ns`가 됐다. 64-bit는 첫 배선에서 기준을 만족해 보정을 생략하고 전체 WNS `+0.123 ns`로 닫혔다. | 구현시간과 메모리 사용량 증가 |
| V3 조건부 Reset/Phase FF 복제 | 기본 기능 아님 | 첫 배선의 TDC 200 MHz Reset 경로가 `0.100 ns` 미만일 때만 적용한다. 고팬아웃 배선을 지역화하되 이미 여유가 충분한 Profile에는 적용하지 않는다. | 적용 시 FF 및 배선 자원 소폭 증가 |

여기서 중요한 구분은 다음과 같다.

- **빌드 안정성 때문에 유지해야 하는 변경**: V3 직접 합성,
  `synth_checkpoint_mode None`, `generate_target all`, 짧은 결과 경로.
- **타이밍 결과에 따라 재평가할 수 있는 변경**: 구현 Explore directive와 조건부
  Reset/Phase FF 복제.
- **현재 변경할 이유가 없는 항목**: V2 합성 전략.

따라서 “기본 옵션이면 기능이 틀린다”는 의미는 아니다. 기본 옵션도 논리 기능은
같아야 하지만, 현재 200 MHz Sign-off 기준인 Setup 여유 `0.100 ns` 이상과 Windows
빌드 재현성을 보장하지 못할 수 있다. 기본 구현 전략으로 되돌리려면 같은 Commit,
XDC, 32/64-bit Profile에서 다시 A/B 구현하고 다음 항목을 모두 만족해야 한다.

- Black Box 0, Latch 0, Blocking DRC 0
- Active Critical CDC 0
- TDC 200 MHz Reset 및 일반 경로 Setup 여유 각각 `0.100 ns` 이상
- Hold 위반 0과 GPX IOB Register 배치 계약 만족
- BIT/XSA 생성 및 반복 Build 결과 재현

## 3. V2 GUI 및 실제 설정

### 3.1 Project Summary

![V2 Project Summary의 합성 및 구현 전략](./images/V2_Project_Summary_Strategies.png)

화면에서 다음 정보를 확인할 수 있다.

- Synthesis Strategy: `Vivado Synthesis Defaults`
- Implementation Strategy: `Performance_Explore`
- `synth_1`: `synth_design Complete!`
- `impl_1`: `write_bitstream Complete!`

V2의 GUI 표시값은 실제 실행한 Project Run과 일치한다.

### 3.2 V2 합성 설정

합성 전략은 Vivado 기본값을 유지한다. 하위 Block Design IP도 기본 Out-of-Context
(OOC) Run으로 합성되고, 최상위 `synth_1`이 하위 DCP를 연결한다.

```tcl
reset_run synth_1
launch_runs synth_1 -jobs 8
wait_on_run synth_1
```

`-jobs 8`은 동시 작업 수를 정하는 실행 자원 옵션이다. 합성 논리의 최적화 전략을
변경하는 옵션은 아니다.

### 3.3 V2 구현 설정

V2는 Vivado 기본 구현 전략 대신 `Performance_Explore`를 사용한다.

```tcl
set_property strategy Performance_Explore [get_runs impl_1]
launch_runs impl_1 -to_step write_bitstream -jobs 8
```

Vivado 2025.2.1의 실제 실행 로그에서 확인한 단계별 directive는 다음과 같다.

| 구현 단계 | 실제 directive |
|---|---|
| `opt_design` | `Explore` |
| `place_design` | `Explore` |
| `phys_opt_design` | `Explore` |
| `route_design` | `Explore` |

권위 있는 설정원은 다음 파일이다.

```text
system_integration/v2/parent_l0/run_v2_l0_parent_signoff.tcl
```

## 4. V3 GUI와 실제 Sign-off 설정의 차이

### 4.1 Parent Project Summary

![V3 Project Summary의 기본 Run](./images/V3_Project_Summary_Default_Runs.png)

V3 Parent `.xpr`를 일반적으로 열면 다음과 같이 표시된다.

- Synthesis Strategy: `Vivado Synthesis Defaults`
- Implementation Strategy: `Vivado Implementation Defaults`
- `synth_1`: `Not started`
- `impl_1`: `Not started`

이 화면은 V3 Sign-off가 수행되지 않았다는 의미가 아니다. V3는 Project Run을
실행하지 않고 동일 Vivado 세션 안에서 합성·배치·배선을 직접 수행하기 때문에 Run
데이터베이스가 갱신되지 않는다.

### 4.2 V3 직접 합성 설정

![V3 Vivado 편집기에서 확인한 직접 합성 설정](./images/V3_Vivado_Editor_Direct_Synthesis_Settings.png)

Vivado GUI에서 다음 경로로 실제 설정원을 확인할 수 있다.

1. `File > Text Editor > Open File...`
2. `system_integration/v3/parent_l0/run_v3_l0_parent_signoff.tcl` 열기
3. `Ctrl+G`를 눌러 437번 줄 부근으로 이동

실제 합성 설정은 다음과 같다.

```tcl
set_param general.maxThreads 4
set_property synth_checkpoint_mode None $bd_file
generate_target all $bd_file
synth_design -top design_1_lidar_ctrl_v3_wrapper \
    -part xc7z020clg484-2 -flatten_hierarchy rebuilt
write_checkpoint -force $result_dir/post_synth.dcp
```

각 설정의 목적은 다음과 같다.

| 설정 | 분류 | 목적 |
|---|---|---|
| `general.maxThreads 4` | 실행 자원 | Vivado 내부 병렬 Thread 수 제한 |
| `synth_checkpoint_mode None` | 합성 구조 | 하위 IP OOC DCP 대신 전체 소스를 한 세션에서 합성 |
| `generate_target all` | IP 생성 | Block Design과 하위 IP 생성물 준비 |
| `flatten_hierarchy rebuilt` | 합성 구조 | 계층을 최적화한 뒤 디버깅 가능한 형태로 재구성 |
| `post_synth.dcp` | 결과 보존 | GUI Run과 독립된 합성 Sign-off Checkpoint |

V3가 전체 직접 합성을 사용하는 주된 이유는 HLS 생성 RTL의 긴 파일명으로 인한
Windows 하위 Run 경로·Queue 실패를 피하고, 캐시된 DCP가 아닌 패키지 전체 Source
Closure를 검증하기 위해서다.

### 4.3 V3 직접 구현 설정

![V3 Vivado 편집기에서 확인한 직접 구현 설정](./images/V3_Vivado_Editor_Direct_Implementation_Settings.png)

동일 파일의 529번 줄 부근에서 실제 구현 directive를 확인할 수 있다.

```tcl
opt_design      -directive ExploreWithRemap
place_design    -directive Explore
phys_opt_design -directive AggressiveExplore
route_design    -directive AggressiveExplore -tns_cleanup
```

모두 Vivado 기본 구현 설정보다 적극적인 타이밍 최적화 설정이다.

첫 배선 후 TDC 200 MHz Reset 경로 또는 일반 상태·데이터 경로의 Setup 여유가
`0.100 ns`보다 작으면 다음 조건부 물리 최적화를 추가 수행한다.

1. 배치가 끝난 `pre_route.dcp`로 복귀한다.
2. TDC Reset과 Phase 고팬아웃 Net을 물리 복제한다.
3. `AggressiveExplore -tns_cleanup`으로 다시 배선한다.
4. 최종 TDC 200 MHz Reset 경로와 일반 상태·데이터 경로가 각각 `0.100 ns` 이상인지
   다시 검사한다.

최신 2026-08-24 실행에서는 다음과 같이 적용됐다.

| Profile | 첫 배선 판정 | 조건부 Net 복제 |
|---|---|---|
| V3 32-bit | Reset 경로가 기준 미달 | Reset/Phase 복제 후 재배선 |
| V3 64-bit | 첫 배선에서 기준 만족 | 적용하지 않음 |

이는 RTL 또는 Generic 차이가 아니라 첫 배치·배선 결과에 따라 선택되는 구현 단계다.

## 5. 결과 확인 위치

### 5.1 V2

V2는 Parent 프로젝트 안에서 결과를 확인한다.

```text
32-bit: .work/v2_parent_l0/w32/project_4_lidar_v2_l0.xpr
64-bit: .work/v2_parent_l0/w64/project_4_lidar_v2_l0.xpr
```

GUI 확인 순서:

1. `Project Summary`에서 Synthesis/Implementation Strategy를 확인한다.
2. `Design Runs`에서 `synth_1`과 `impl_1` 완료 상태를 확인한다.
3. `Open Synthesized Design` 또는 `Open Implemented Design`을 실행한다.

### 5.2 V3

V3 Parent `.xpr`는 Block Design과 Source 유지보수에 사용한다. 최종 물리 결과는
Sign-off Checkpoint를 열어 확인한다.

```text
32-bit project: .work/v3_parent_l0/w32/project_4_lidar_v3_l0.xpr
32-bit result : .work/v3_parent_signoff/260824_v3_w32_impl2/post_route.dcp

64-bit project: .work/v3_parent_l0/w64/project_4_lidar_v3_l0.xpr
64-bit result : .work/v3_parent_signoff/260824_v3_w64_impl2/post_route.dcp
```

V3 GUI에서 `Design Runs`의 `Not started`만 보고 구현 누락으로 판단하지 않는다.
`post_route.dcp`, `post_route_timing_summary.rpt`, BIT와 XSA가 모두 존재하고 Sign-off
요약의 PASS 조건을 만족해야 완료로 판정한다.

## 6. 유지보수 규칙

1. V2 구현 전략을 변경할 때는 `Performance_Explore` 설정과 실제 Run 로그의
   단계별 directive를 함께 확인한다.
2. V3의 Project Settings 화면에서 전략을 변경해도 직접 Sign-off 흐름에는 반영되지
   않는다. 반드시 `run_v3_l0_parent_signoff.tcl`을 수정해야 한다.
3. V3 직접 실행을 Project Run 방식으로 변경하려면 Windows 장경로, HLS 하위 IP
   생성, 조건부 Net 복제와 재배선 절차를 모두 다시 검증해야 한다.
4. 설정 변경 후 32-bit와 64-bit를 각각 합성·구현하고, Black Box, Latch, DRC,
   Setup/Hold, TDC 200 MHz 여유와 GPX IOB 배치를 다시 검사한다.
5. GUI 화면, Tcl 설정, Sign-off 결과가 서로 다른 Commit을 가리키지 않도록 결과
   Manifest의 Branch와 Commit을 먼저 확인한다.
