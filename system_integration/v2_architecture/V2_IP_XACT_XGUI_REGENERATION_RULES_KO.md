# V2 IP-XACT 저장 및 XGUI 재생성 강제 규칙

## 1. 목적

이 문서는 `victek.co.kr:my_ip:tdc_gpx_lidar_ctrl_v2:2.0`을 Vivado IP Packager로
다시 열었을 때 `Customization GUI`의 `Layout`과 `Preview`가 비거나 하나의 기본
Page로 합쳐지는 문제를 방지한다.

이 문서에서 **IP-XACT 저장**은 Generic, Customization Parameter, Port/Interface,
File Group과 문서 구성을 `component.xml`에 확정하는 과정이다. **네이티브 XGUI**는
Vivado가 `ipx::create_xgui_files`로 생성하여 Package IP 화면에서 시각 편집할 수
있는 Tcl 형식이다.

## 2. 순서를 바꿀 수 없는 규칙

> 최종 IP-XACT를 저장하고 기존 core 객체를 완전히 닫은 뒤,
> `component.xml`을 다시 열어 XGUI를 생성한다.

1. 88개 RTL, 20개 합성 Generic, 값 목록·범위, Port/Interface와 문서를 구성한다.
2. `ipx::update_checksums`와 `ipx::check_integrity`를 실행한다.
3. `ipx::save_core`로 `component.xml`을 저장한다.
4. `ipx::unload_core`와 `close_project`로 packaging 객체를 닫는다.
5. 새 in-memory project를 생성한다.
6. 저장된 `component.xml`을 `ipx::open_core`로 다시 연다.
7. `common::load_features ipservices`를 실행한다.
8. `ipx::create_xgui_files`를 실행한다.
9. `check_v2_xgui_source_contract.tcl`로 생성된 XGUI를 검사한다.
10. checksum 갱신, core 저장, unload, project close 순으로 종료한다.

기준 구현은 `system_integration/v2/scripts/package_v2_ip.tcl`이다.

## 3. 책임 분리

| 내용 | 단일 관리 위치 |
|---|---|
| 탭·그룹·Parameter·설명문 배치 | `v2/ip_package/tdc_gpx_lidar_ctrl_v2_xgui.tcl` |
| 선택 목록·범위·표시명 | `v2/scripts/package_v2_ip.tcl`의 IP-XACT 속성 |
| Echo Receiver 선택에 따른 Optional port 표시 | `package_v2_ip.tcl`의 enablement dependency |
| HDL Generic 전달 | Vivado 생성 `update_MODELPARAM_VALUE.*` |
| 여러 Generic 사이의 유효성 계약 | `fn_validate_build_config`와 관련 테스트벤치 |
| 배포본 | `v2/ip_repo/tdc_gpx_lidar_ctrl_v2_2_0` |

`ip_repo`는 생성물이므로 직접 수정하지 않는다. Package IP Layout에서 XGUI를
수정했다면 승인된 import 스크립트로 canonical XGUI에 반영한 뒤 재패키징한다.

## 4. 금지 사항

1. `ipx::save_core` 전에 XGUI 생성
2. 저장 후 같은 core 객체에서 XGUI 생성
3. `update_PARAM_VALUE.*` 또는 `validate_PARAM_VALUE.*`에 수동 Tcl 로직 추가
4. `update_MODELPARAM_VALUE.*`에 변환 로직 추가
5. `init_gui`에서 `foreach`, `eval` 또는 외부 helper로 Layout 동적 생성
6. Customization Parameters의 `Merge changes`로 File Group 재구성
7. `Regenerate and overwrite` 안내에서 원인 확인 없이 덮어쓰기
8. `ip_repo/xgui`만 수정하고 canonical XGUI 동기화 생략
9. 수정 시각만 변경하고 source contract 검사 생략

## 5. 승인된 변경 절차

RTL, Generic, IP-XACT 또는 문서를 변경한 경우 관련 Vivado 창을 닫은 뒤 다음을
실행한다.

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File `
  ./system_integration/v2/scripts/open_v2_ip_packager_gui.ps1 `
  -RefreshPackage -Recreate -ValidateOnly
```

Package IP Layout에서 XGUI를 수정한 경우에는 `Review and Package`로 저장하고
Vivado를 닫은 다음 실행한다.

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File `
  ./system_integration/v2/scripts/import_v2_xgui_from_packager.ps1
```

그 뒤 반드시 package를 다시 생성한다. import 스크립트가 네이티브 XGUI 계약 오류를
반환하면 canonical 원본을 덮어쓰지 않는다.

## 6. 필수 자동 검사

| 검사 | PASS 표식 | 확인 내용 |
|---|---|---|
| XGUI source 검사 | `LIDAR_V2_XGUI_NATIVE_VISUAL_CONTRACT_PASS` | 명시적 4개 Page, 20개 callback, 직접 Generic 전달 |
| package 검사 | `LIDAR_V2_K010_IP_PACKAGE_CHECK_PASS` | IP-XACT, 88개 RTL, 5개 문서, XGUI 동기화 |
| Packager project 검사 | `TDC_GPX_LIDAR_CTRL_V2_IP_PACKAGER_CHECK_PASS` | 편집 project와 canonical core 무결성 |
| customization 검사 | 32/64/128 profile PASS | 폭, clock mode와 Echo 선택 보존 |

## 7. GUI 육안 검사

Package IP의 `Customization GUI`에서 다음을 확인한다.

- 왼쪽에 `Layout` 트리가 표시된다.
- 오른쪽에 IP symbol과 `Preview`가 표시된다.
- `Clock and Output`, `TDC-GPX Topology`, `Echo Frontend`, `Physical Timing`의
  네 Page가 모두 표시된다.
- `Cannot auto-generate customization preview` 안내가 없다.
- `Customization Tcl ... older` 안내가 없다.

실제 `Customize IP` 창에서는 32/64/128-bit, 1~4 TDC-GPX chip,
Rising/Falling capability mask, Echo Receiver enablement와 지원 clock 목록을
확인한다. Package IP Preview만 확인한 상태는 완료가 아니다.

## 8. Git 규칙

커밋에는 canonical XGUI, 재현 스크립트, 문서와 의도된 `ip_repo` 생성물만 넣는다.
`.work`, Vivado log/journal, `vivado_pid*.str`, 단축 드라이브 내용은 넣지 않는다.
