# C08 Virtual Encoder Legacy Source Removal v029

## 1. 목적

Virtual Encoder 최적화 이후에도 소스 루트에 남아 있던 구형 구현을 제거해,
새 parent project와 유지보수자가 현재 구조만 보도록 source ownership을
명확히 한다.

## 2. 제거 대상

| 구형 모듈 | 현재 대체 모듈 | Canonical line |
|---|---|---:|
| `enc_fractional_scheduler.vhd` | `enc_timing_generator.vhd` | 245 |
| `enc_tick_counter.vhd` | `enc_timing_generator.vhd` | 93 |
| `enc_position_counter.vhd` | `enc_position_tracker.vhd` + `enc_index_pulse.vhd` | 413 |
| 합계 | | 751 |

Motor Decoder 저장소의 호환 복사본에서도 동일한 구형 파일 3개, 757 line을
제거했다. 두 저장소를 합하면 1,508 line의 비활성 RTL이 작업 트리에서
사라졌다. Git 이력에는 원본이 그대로 보존된다.

## 3. 참조 전수 조사

### 3.1 관리 대상 Git 소스

다음 범위에서는 구형 entity/file 참조가 0건임을 확인했다.

1. Canonical Virtual Encoder의 나머지 tracked source
2. Motor Decoder의 합성 RTL
3. TDC GPX integration VHDL
4. `run_virtual_encoder_unit.ps1`
5. `run_smoke.ps1`
6. `add_sibling_sources.tcl`

현재 통합 compile 순서는 이미 새 모듈만 명시적으로 사용한다.

### 3.2 남아 있는 과거 참조

다음 위치에는 구형 이름이 남아 있지만 관리 대상 RTL이 아니라 과거 프로젝트
또는 생성 산출물이다.

| 분류 | 예 | 처리 정책 |
|---|---|---|
| 과거 IP 프로젝트 | `virtual_encoder.xpr`, `motor_decoder.xpr` | 새 active manifest로 재생성 |
| 과거 통합 프로젝트 | `tdc_gpx_ctrl.xpr`, `laser_ctrl.xpr` | 실제 parent project로 사용 금지 |
| 패키징 복사본 | `ip_repo/src` | canonical RTL 기준으로 IP 재패키징 |
| Vivado 생성물 | `.gen`, `.sim`, `.runs`, `.cache`, `ip_user_files` | source가 아니라 재생성 대상 |
| 과거 golden project | `echo_receiver/scripts/*golden.prj` | 신규 통합 검증으로 대체 |
| Python v4 모델 | `TEST/virtual_encoder_v4.py` | 현재 RTL과 1:1 모델이 아닌 legacy 참고자료 |

특히 과거 `.xpr`은 새 `enc_position_tracker`, `enc_index_pulse`,
`enc_timing_generator`를 source list에 포함하지 않아 제거 전에도 현재
`enc_top`을 정상 구성할 수 없는 상태였다. 따라서 이를 억지로 보존하는 대신
새 parent project를 만들 때 canonical source list로 다시 생성하는 편이
더 안전하다.

## 4. 유지보수 보완

Canonical 저장소에 `README.md`를 추가해 다음 정보를 한곳에 고정했다.

1. active RTL 7개 파일의 compile 순서
2. 모듈별 단일 책임
3. 유지되는 단위 TB 목록과 runner 위치
4. 구형 모듈과 대체 모듈의 관계
5. 과거 `.xpr`/`ip_repo`가 source of truth가 아니라는 경고

Canonical active RTL은 총 924 physical line이며, 루트에서 읽어야 할 합성
모듈은 아래 7개뿐이다.

```text
enc_pkg.vhd
enc_param_apply_ctrl.vhd
enc_phase_counter.vhd
enc_position_tracker.vhd
enc_index_pulse.vhd
enc_timing_generator.vhd
enc_top.vhd
```

## 5. 제거 후 검증

| 시험 | 결과 |
|---|---|
| canonical `enc_top_tb` 18 scenarios | PASS |
| `tb_enc_param_boundary` | PASS |
| Motor 호환 `enc_top_tb` | PASS |
| Internal full integration | PASS |

최종 내부 통합 결과는 변경 전과 동일하다.

| 관찰 항목 | 결과 |
|---|---:|
| Laser start/stop | 18 / 18 |
| Laser result TLAST | 18 |
| Echo rise/stop event | 18 / 18 |
| TDC rising/falling TLAST | 18 / 18 |
| `cfg_rejected` / `pipeline_abort` | 0 / 0 |
| `face_valid` violation | 0 |

Archive:

```text
sim_results/vivado_xsim/sessions/260722_legacy_removed_full_system_integration_smoke
```

구형 파일은 active synth manifest에 포함되지 않았으므로 제거 자체는 합성
netlist를 변경하지 않는다. 직전 active RTL 200 MHz 합성 결과인
703 LUT, 620 FF, WNS +0.345 ns가 그대로 적용된다.

## 6. 판정

Virtual Encoder의 maintained HDL에서 중복 구현 제거는 완료되었다. 새 parent
project는 과거 packaged IP를 재사용하지 말고 canonical Virtual Encoder와
Motor/Laser/Echo/TDC source를 명시적으로 연결하는 방식으로 시작해야 한다.
기능 통합이 안정화된 뒤에만 각각을 다시 IP package로 고정한다.
