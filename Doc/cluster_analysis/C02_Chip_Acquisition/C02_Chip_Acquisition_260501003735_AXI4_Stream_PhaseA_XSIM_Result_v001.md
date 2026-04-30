# C02 AXI4-Stream Phase A xsim 검증 결과 v001

- 문서 생성/수정 시간(KST): 2026-05-01 00:37:35
- Vivado 경로: `C:\AMDDesignTools\2025.2.1\Vivado`
- 대상: C02 AXI4-Stream width standardization Phase A
- 핵심 질문: 64-bit 폭 성공 가능 여부
- 결론: 64-bit 폭은 컴파일, elaboration, xsim 실행 기준으로 성공 가능하다.

## 1. 실행한 검증

| 순서 | TB / 대상 | 목적 | 결과 |
|---:|---|---|---|
| 1 | `tb_tdc_gpx_header_inserter_widths` | 32/64/128-bit header path, full `tkeep/tstrb` 확인 | PASS |
| 2 | `tb_tdc_gpx_output_stage` | 기본 64-bit output_stage, face_assembler/header 포함 | PASS |
| 3 | `tb_tdc_gpx_top_int` | 기본 64-bit top 통합 흐름 | PASS |

## 2. 주요 실행 근거

### 2.1 Width matrix TB

실행 명령 요지:

```powershell
xvhdl --2008 --work xil_defaultlib tdc_gpx_pkg.vhd tdc_gpx_header_inserter.vhd tb_tdc_gpx_header_inserter_widths.vhd
xelab --debug typical --relax -L xil_defaultlib xil_defaultlib.tb_tdc_gpx_header_inserter_widths -s tb_tdc_gpx_header_inserter_widths_sim
xsim tb_tdc_gpx_header_inserter_widths_sim -runall
```

결과 핵심:

```text
Note: *** tb_tdc_gpx_header_inserter_widths PASS ***
$finish called at time : 2057500 ps
```

판단:

- 32/64/128-bit 세 폭 모두 header prefix beat 수와 full `tkeep/tstrb` 계약을 만족했다.
- 따라서 64-bit header path 자체는 성공 가능하다.

### 2.2 Output stage 64-bit TB

처음 elaboration에서는 XPM이 요구하는 `glbl` 미포함으로 실패했다.

```text
ERROR: [VRFC 10-2989] 'glbl' is not declared
```

이는 width 문제가 아니라 XPM simulation 환경 문제다. 이후 Vivado 제공 `glbl.v`를 추가해서 재실행했다.

```powershell
xvlog --work xil_defaultlib C:\AMDDesignTools\2025.2.1\Vivado\data\verilog\src\glbl.v
xelab --debug typical --relax -L xil_defaultlib -L xpm xil_defaultlib.tb_tdc_gpx_output_stage xil_defaultlib.glbl -s tb_tdc_gpx_output_stage_64_sim
xsim tb_tdc_gpx_output_stage_64_sim -runall
```

결과 핵심:

```text
Note: *** SCENARIO 1 (rise-only smoke) PASS ***
Note: *** SCENARIO 2 (slope-independent abort) PASS ***
```

64-bit 결과 수치:

| 항목 | 결과 |
|---|---:|
| Scenario 1 output beats | 14 |
| Scenario 1 SOF | true |
| Scenario 1 EOL | true |
| Scenario 1 frame_done | `'1'` |
| Scenario 2 rise beats | 28 |
| Scenario 2 fall beats | 12 |
| Scenario 2 SOF count | 2 |

판단:

- `tdc_gpx_output_stage`의 기본 64-bit 폭은 성공한다.
- 추가한 `tkeep/tstrb` 검사도 실패하지 않았다.

### 2.3 Top integration 64-bit TB

기존 Vivado 프로젝트 `launch_top_int_sim.tcl` 실행은 incremental cache stale 문제로 실패했다.

```text
'tdc_gpx_decoder_i_mode.vdb' needs to be re-saved since 'xil_defaultlib.tdc_gpx_pkg' changed
```

이는 64-bit 폭 문제가 아니라 프로젝트의 기존 xsim incremental 산출물 문제다. 별도 clean CLI 폴더 `sim_topint64_cli`에서 전체 소스와 IP simulation source를 새로 컴파일했다.

결과 핵심:

```text
Built simulation snapshot tb_tdc_gpx_top_int_64_cli_sim
Note: tb_tdc_gpx_top_int: rising stream emitted 60 beats - PASS
```

통합 결과 수치:

| 항목 | 결과 |
|---|---:|
| Simulation end time | 43137.5 ns |
| rising stream beats | 60 |
| rising stream tlast_cnt | 2 |
| falling stream beats | 60 |
| falling stream tlast_cnt | 2 |
| expected-count bound | col0/col1 PASS |

판단:

- `tb_tdc_gpx_top_int` 기본 설정은 64-bit이며, clean CLI xsim에서 PASS했다.
- 따라서 C02 Phase A의 64-bit 폭은 top 통합 수준에서도 성공 가능하다.

## 3. 주의할 점

| 항목 | 판단 |
|---|---|
| `glbl` 필요 | XPM 포함 TB는 `glbl.v`를 함께 compile/elaborate 해야 한다. |
| 프로젝트 incremental 실패 | 기존 `.sim` incremental cache가 `tdc_gpx_pkg` 변경 이후 stale 상태다. clean run 또는 simulation reset이 필요하다. |
| 64-bit width 자체 | 문제 없음. header/output/top 통합 모두 PASS 근거 확보. |

## 4. 다음 조치 제안

Vivado 프로젝트 기반 실행을 안정화하려면 `scripts/launch_top_int_sim.tcl` 또는 별도 clean script에 다음 절차를 반영하는 것이 좋다.

1. simulation reset 또는 xsim incremental 산출물 clean
2. XPM 사용 TB에서 `glbl` 포함 보장
3. `tb_tdc_gpx_header_inserter_widths.vhd`를 sim source에 추가
4. 64-bit 기본 TB와 width matrix TB를 회귀 항목으로 분리
