# TDC-GPX LiDAR Controller V3 완성 브랜치 기준

## 1. 브랜치 목적

`codex/tdc-gpx-lidar-v3-complete`는 V3 HLS 혼합 구현의 보드 없는 완료 기준을
고정하는 독립 브랜치다. 이 브랜치 하나만 체크아웃해도 V2에서 유지하는 물리
제어 RTL, V3 HLS 처리 경로, 테스트벤치, 두 IP-XACT 패키지, XGUI, PS 참조 코드와
보드 없는 검증 절차를 재현할 수 있어야 한다.

V3가 `system_integration/v2`를 포함하는 것은 브랜치 의존이 아니다. V3가 유지하는
Motor decoder, Face tracker, Shot scheduler, Laser executor, TDC-GPX bus PHY,
통합 AXI4-Lite CSR와 CDC의 정본을 같은 체크아웃 안에서 재사용하기 위한 의도된
소스 의존성이다.

## 2. 포함 범위

- 4 Chip, Chip당 8 STOP, 최대 7 Return의 TDC-GPX I-Mode 수집
- 물리 `fire_done` 승인에서 측정 시작 기준시점 (T0)을 만드는 RTL 경로
- V2 RTL 유지 영역과 H1~H4 HLS 데이터 처리 영역의 통합
- PACKED17 및 합성 시 선택하는 32/64-bit AXI4-Stream 출력
- Runtime Return 수의 Face 경계 원자적 적용과 VDMA profile 승인
- 통합 AXI4-Lite CSR, Shadow/Active COMMIT, IRQ 및 진단 경로
- Cortex-A9 VDMA/DDR cache 소유권과 H-Line/Ethernet 참조 처리
- Vivado 2025.2.1 IP-XACT, Customize IP XGUI와 4 Chip Parent 검증
- 생성 RTL 포함형과 HLS 하위 IP 참조형의 두 배포 방식

128-bit AXI4-Stream 출력은 V3 제품 범위에서 의도적으로 제외한다.

## 3. 기준 이력

- V3 보드 없는 H6-B3B Sign-off와 V2 공통 반영 기준: `fc3192d`
- V3 Native IP Packager, Customize IP XGUI와 두 패키지 방식 보완 포함
- 독립 브랜치 생성 원본: `168794b`

위 번호는 변경 출처 추적용이다. 실제 V3 제품 기준은 항상 이 브랜치 HEAD다.

## 4. 정본 소유권

| 항목 | 정본 경로 |
|---|---|
| 유지 RTL 및 CSR | `system_integration/v2/rtl`, `system_integration/v2/pkg` |
| 공통 AXI4-Lite 전송/IRQ RTL | `system_integration/common/rtl/csr` |
| V3 HLS C++와 의미 기반 Header | `system_integration/v3/hls` |
| V3 RTL Adapter와 통합 Top | `system_integration/v3/rtl` |
| V3 테스트벤치 | `system_integration/v3/tb` |
| XGUI 원본 | `system_integration/v3/ip_package/tdc_gpx_lidar_ctrl_v3_xgui.tcl` |
| 생성 RTL 포함형 IP | `system_integration/v3/ip_repo/tdc_gpx_lidar_ctrl_v3_3_0` |
| HLS 하위 IP 참조형 IP | `system_integration/v3/ip_repo/tdc_gpx_lidar_ctrl_v3_hls_ip_3_0` |
| HLS/운용/Sign-off 문서 | `system_integration/v3/docs` |

생성 프로젝트, Vivado/Vitis 로그, Waveform과 임시 패키징 결과는 `.work`, `tmp`,
`signoff_results/sessions`에만 두며 제품 소스로 커밋하지 않는다.

## 5. 두 IP 패키지 재현

보드 Parent 배포 기준인 생성 RTL 포함형을 재생성하고 검사한다.

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v3/scripts/run_v3_ip_package.ps1
```

HLS 컴포넌트 4개를 하위 IP로 관리하는 유지보수형을 재생성하고 검사한다.

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v3/scripts/run_v3_hls_ip_package.ps1
```

두 패키지의 32/64-bit OOC 조합을 함께 확인할 때는 다음 절차를 따른다.

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v3/scripts/run_v3_dual_package_ooc.ps1
```

## 6. Sign-off 경계

이 브랜치는 RTL/HLS 차등, CDC, 150/200 MHz 제품 검증점, IP 패키지, 4 Chip Parent,
VDMA/DDR/PS 참조 계약까지 보드 없이 재현 가능한 범위를 완료 기준으로 보존한다.

실제 PCB의 40 MHz TDC-GPX 기준 클럭, 외부 GPX I-Mode 읽기, VDMA/DDR cache API,
Ethernet, 레이저와 선택적 Echo LVDS 계측은 H6-B4 실물 보드 증거가 필요하다.
따라서 이 문서의 "완성"은 보드 없는 V3 제품 기준이며 실물 보드 Sign-off 선언은
아니다.

## 7. 독립성 계약

- 저장소 밖 `../ip_repo`의 AXI4-Lite RTL을 합성 입력으로 사용하지 않는다.
- 과거 V1 IP가 있으면 Catalog 공존을 추가 검사하지만 없어도 V3 단독 검증이 된다.
- Vitis HLS 생성물의 긴 파일명은 짧은 임시 drive에서 처리하되, 배포 IP에는 필요한
  Verilog와 LUTRAM 데이터를 모두 포함한다.
- XGUI는 IP-XACT 저장 뒤 자동 생성 파일을 정본으로 보존하며, Git의 EOF 정규화로
  Vivado Preview freshness 판정이 바뀌지 않게 한다.

## 8. 브랜치 생성 시 재검증 결과

2026-08-21에 저장소 밖 `../ip_repo`가 없는 독립 worktree에서 다음을 확인했다.

- H1 Raw28 해석, H2 Return Cell 수집, H3 Rise/Fall 정렬, H4 PACKED17 Word 생성의
  Vitis HLS C synthesis PASS
- 생성 RTL 포함형 IP 재생성, 무결성 DRC, Native XGUI와 IP-XACT 계약 PASS
- V1 package가 없는 V3 단독 IP Catalog 등록 PASS
- 생성 RTL 포함형 32/64-bit OOC 합성 PASS
- 네 HLS 하위 IP 참조형 IP 재생성, 하위 IP 4개 및 XGUI 계약 PASS
- HLS 하위 IP 참조형 32/64-bit OOC 합성 PASS
- 네 OOC 구성 모두 `synth_design completed successfully`

대표 종료 표시는 다음과 같다.

- `LIDAR_V3_STANDALONE_CATALOG_PASS legacy_v1=absent`
- `LIDAR_V3_IP_PACKAGE_RUNNER_PASS`
- `LIDAR_V3_HLS_IP_PACKAGE_RUNNER_PASS`
- `LIDAR_V3_DUAL_PACKAGE_OOC_RUNNER_PASS selector=ALL`

HLS 참조형 package에서 삭제한 평면 VHDL 80개는 IP-XACT에 등록되지 않았던 중복
사본이다. TDC-GPX 핵심 파일 8개(`tdc_gpx_pkg`, `tdc_gpx_cfg_pkg`, Chip 제어,
Bus PHY와 skid buffer)는 `component.xml`에 `src/tdc_gpx_*.vhd`로 실제 등록되므로
그 위치에 유지한다. 나머지 합성 파일은 `src/system_integration/...`,
`src/external_csr/...`, `src/shared/...`에 한 번만 보존되며 `component.xml` File
Group과 일치한다.
