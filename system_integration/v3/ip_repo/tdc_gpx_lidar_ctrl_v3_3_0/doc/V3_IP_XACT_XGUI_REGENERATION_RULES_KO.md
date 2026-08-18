# V3 IP-XACT 저장 및 XGUI 재생성 강제 규칙

## 1. 목적과 적용 범위

이 문서는 Vivado IP Packager의 `Customization GUI`에서 `Layout`과 `Preview`가
비어 버리는 문제를 재발시키지 않기 위한 강제 작업 규칙이다. 다음 두 V3 통합 IP에
동일하게 적용한다.

- 생성 RTL 포함형: `victek.co.kr:my_ip:tdc_gpx_lidar_ctrl_v3:3.0`
- HLS 하위 IP 참조형: `victek.co.kr:my_ip:tdc_gpx_lidar_ctrl_v3_hls_ip:3.0`

이 규칙에서 **IP-XACT 저장**은 최종 Customization Parameter, HDL Generic,
Port/Interface 및 File Group 상태를 `component.xml`에 저장하는 것을 뜻한다.
**네이티브 XGUI**는 Vivado가 `ipx::create_xgui_files`로 생성하여 IP Packager가
시각 편집 가능한 Layout/Preview로 인식하는 Tcl 형식을 뜻한다.

## 2. 핵심 강제 규칙

> XGUI는 최종 IP-XACT 상태를 저장한 뒤, 기존 core 객체를 닫고
> `component.xml`을 새 core 객체로 다시 연 다음 생성한다.

다음 순서는 바꿀 수 없다.

1. RTL, Generic, User Parameter, 값 목록·범위, Port/Interface와 File Group을 구성한다.
2. `ipx::update_checksums`와 `ipx::check_integrity`를 실행한다.
3. `ipx::save_core`로 최종 IP-XACT를 저장한다.
4. `ipx::unload_core`와 `close_project`로 기존 packaging 객체를 완전히 닫는다.
5. 새로운 in-memory Vivado project를 만든다.
6. 저장된 `component.xml`을 `ipx::open_core`로 다시 연다.
7. `common::load_features ipservices`를 실행한다.
8. 다시 연 core에서 `ipx::create_xgui_files`를 실행한다.
9. `check_v3_xgui_source_contract.tcl`로 네이티브 XGUI 계약을 검사한다.
10. checksum을 다시 계산하고 core를 저장한 뒤 unload한다.

기준 구현은 `scripts/package_v3_ip.tcl`의 마지막 XGUI 동기화 단계다.

```tcl
ipx::update_checksums $core
ipx::check_integrity -verbose $core
ipx::save_core $core
ipx::unload_core $core
close_project

create_project -in_memory ${project_name}_xgui_sync -part xc7z020clg484-2
set core [ipx::open_core $component]
common::load_features ipservices
ipx::create_xgui_files $core
v3_check_xgui_source_contract $xgui_destination
ipx::update_checksums $core
ipx::save_core $core
ipx::unload_core $core
close_project
```

최종 IP-XACT를 저장하기 전에 XGUI를 생성하거나, 아직 열려 있는 packaging core에서
XGUI를 다시 생성하면 Vivado가 이전 Parameter 관계를 사용하거나 기본 단일 Page로
Layout을 평탄화할 수 있다.

## 3. 파일별 단일 책임

| 책임 | 관리 위치 | GUI 직접 편집 |
|---|---|---|
| Page, Group, Parameter, Static text 배치 | `ip_package/tdc_gpx_lidar_ctrl_v3_xgui.tcl` | Package IP Layout에서 가능 |
| 선택 목록, 정수 범위, Display Name | `scripts/package_v3_ip.tcl`의 IP-XACT 속성 | 실험만 가능 |
| Optional port 표시 조건 | `scripts/package_v3_ip.tcl`의 enablement dependency | 실험만 가능 |
| HDL Generic 전달 | Vivado가 생성한 `update_MODELPARAM_VALUE.*` | 수동 로직 추가 금지 |
| 여러 Generic 사이의 유효성 계약 | `fn_validate_build_config`와 관련 테스트벤치 | XGUI 중복 구현 금지 |
| 배포 package | `ip_repo/tdc_gpx_lidar_ctrl_v3*_3_0` | 직접 수정 금지 |

`component.xml`은 `package_v3_ip.tcl`로 재현되는 생성물이다. 배포 package의 XGUI를
GUI에서 수정한 경우에는 승인된 import 절차로 canonical XGUI에 반영해야 한다.

## 4. 금지 사항

다음 중 하나라도 수행하면 Layout/Preview 재현성을 보장하지 않는다.

1. 최종 `ipx::save_core` 전에 `ipx::create_xgui_files` 실행
2. 저장 후 같은 core 객체에서 곧바로 XGUI 생성
3. `update_PARAM_VALUE.*` 또는 `validate_PARAM_VALUE.*`에 수동 Tcl 로직 추가
4. `update_MODELPARAM_VALUE.*`에 변환·검증 로직 추가
5. `init_gui`에서 `foreach`, `eval`, 동적 proc 또는 외부 helper로 Layout 생성
6. `Customization Parameters`의 **Merge changes**로 File Group 재구성
7. `Customization Tcl ... older` 안내에서 원인 확인 없이 **Regenerate and overwrite** 실행
8. `ip_repo/xgui` 또는 `component.xml`만 직접 고친 뒤 canonical 원본 동기화 생략
9. 파일 수정 시각만 바꾸고 네이티브 XGUI source contract 검사를 생략
10. 생성 RTL 포함형 한쪽만 갱신하고 HLS 하위 IP 참조형 갱신 생략

## 5. 승인된 변경 절차

### 5.1 RTL 또는 IP-XACT 속성을 변경한 경우

1. 관련 단위·통합 테스트를 통과시킨다.
2. 해당 package를 사용하는 모든 Vivado 창을 닫는다.
3. 두 Variant를 각각 재패키징한다.
4. Package IP의 Layout/Preview와 실제 Customize IP 화면을 모두 확인한다.

```powershell
./system_integration/v3/scripts/open_v3_ip_packager_gui.ps1 `
  -Variant EmbeddedRtl -RefreshPackage -SkipHlsSynthesis -Recreate

./system_integration/v3/scripts/open_v3_ip_packager_gui.ps1 `
  -Variant HlsIp -RefreshPackage -SkipHlsSynthesis -Recreate
```

HLS C++ 또는 공통 Header가 변경된 경우에는 `-SkipHlsSynthesis`를 사용하지 않는다.

### 5.2 Package IP Layout에서 XGUI를 변경한 경우

1. Package IP의 `Customization GUI`에서 Layout을 수정한다.
2. `Review and Package`로 저장하고 Vivado를 닫는다.
3. 생성된 네이티브 XGUI를 canonical 원본으로 가져온다.
4. 두 Variant를 모두 재패키징한다.
5. 자동 검사와 GUI 육안 검사를 다시 수행한다.

```powershell
./system_integration/v3/scripts/import_v3_xgui_from_packager.ps1 `
  -Variant HlsIp
```

`import_v3_xgui_from_packager.ps1`가 source contract 오류를 반환하면 import를 중단하고
수동 callback 로직이나 동적 Layout 생성 코드를 제거한다.

## 6. 필수 자동 검사

릴리스 또는 Git 커밋 전에 다음 검사를 모두 통과해야 한다.

| 검사 | 필수 PASS 표식 | 검증 목적 |
|---|---|---|
| `check_v3_xgui_source_contract.tcl` | `LIDAR_V3_XGUI_NATIVE_VISUAL_CONTRACT_PASS` | 네이티브 callback과 명시적 Layout |
| `check_v3_ip_package.tcl` | `TDC_GPX_LIDAR_CTRL_V3_IP_CHECK_PASS` | 생성 RTL 포함형 IP-XACT·XGUI·문서 동기화 |
| `check_v3_hls_ip_package.tcl` | `TDC_GPX_LIDAR_CTRL_V3_HLS_IP_CHECK_PASS` | HLS 하위 IP 참조형 의존성과 XGUI 동기화 |
| 32-bit customization | 해당 customization PASS | 32-bit 출력과 Dedicated slope mask |
| 64-bit customization | 해당 customization PASS | 64-bit 출력과 4-chip dual-edge mask |

`check_v3_xgui_source_contract.tcl`은 다음 상태를 실패로 처리한다.

- 필수 Page 또는 Generic callback 누락
- hand-edited callback 로직
- 동적으로 생성되는 `init_gui` Layout
- HDL Generic으로 직접 전달되지 않는 Model Parameter

## 7. GUI 육안 검사 체크리스트

Package IP의 `Customization GUI`를 열고 다음을 확인한다.

- 왼쪽에 `Layout` 트리가 표시된다.
- 오른쪽에 IP symbol과 `Preview`가 표시된다.
- `Clock and Output`, `TDC-GPX Topology`, `Echo Frontend`,
  `Physical Timing` Page가 모두 표시된다.
- Parameter가 의도한 Group 아래에 배치된다.
- 화면에 `Cannot auto-generate customization preview` 안내가 없다.
- `Customization Tcl ... older` 안내가 없다.

이어서 실제 `Customize IP` 창에서 값 목록, 범위, 32/64-bit 선택,
1~4 TDC-GPX chip, Rising/Falling capability, Echo Receiver enablement와 Optional port
표시 조건을 확인한다. Package IP Preview만 통과한 상태는 최종 완료가 아니다.

## 8. 오류 발생 시 복구

| 증상 | 원인 후보 | 복구 방법 |
|---|---|---|
| Layout과 Preview가 모두 비어 있음 | hand-edited callback 또는 오래된 XGUI | canonical source contract 확인 후 `-RefreshPackage -Recreate` |
| 기본 Page 하나만 표시 | 저장 전 또는 같은 core 객체에서 XGUI 생성 | IP-XACT 저장·unload·reopen 순서로 재패키징 |
| `Customization Tcl ... older` | `component.xml`과 XGUI 생성 순서·시각 불일치 | 공식 실행기로 package와 편집 project를 재생성 |
| GUI 변경이 다음 패키징에서 사라짐 | `ip_repo/xgui`만 수정 | import 스크립트로 canonical XGUI 동기화 |
| `../HDL` 파일이 유입됨 | Merge changes 사용 | 저장 중단 후 package 재생성 |
| 한 Variant만 이전 GUI 표시 | 두 package 중 하나만 갱신 | EmbeddedRtl과 HlsIp 모두 재패키징 |

## 9. Git 체크포인트 규칙

커밋에는 다음 항목만 포함한다.

- canonical XGUI와 문서
- `package_v3_ip.tcl` 등 재현 스크립트
- 두 `ip_repo` package의 `component.xml`, `xgui`, `doc` 및 의도된 생성물
- 자동 검사 스크립트

`.work`, Vivado log/journal, `vivado_pid*.str`, 임시 가상 드라이브 내용은 포함하지
않는다. 커밋 전에 두 package의 XGUI가 canonical XGUI와 동일한 유지보수 내용을
가지는지 자동 검사로 확인한다.
