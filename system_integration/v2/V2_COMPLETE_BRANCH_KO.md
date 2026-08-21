# TDC-GPX LiDAR Controller V2 완성 브랜치 기준

## 1. 브랜치 목적

`codex/tdc-gpx-lidar-v2-complete`는 V2 RTL 구현을 V3 마이그레이션 소스와
분리하여 유지보수하기 위한 독립 브랜치다. 이 브랜치만 체크아웃해도 V2 RTL,
통합 CSR, 테스트벤치, IP-XACT 저장소, XGUI, PS 참조 코드와 보드 없는 검증 절차를
재현할 수 있어야 한다.

## 2. 포함 범위

- V2 보드 없는 최종 Sign-off 기준과 32/64/128-bit AXI4-Stream 검증
- Git 저장소 내부에 고정한 공통 AXI4-Lite 전송/IRQ RTL
- 4 Chip, 32 물리 STOP, 최대 7 Return의 TDC-GPX I-Mode 처리
- 통합 AXI4-Lite CSR, Shadow/Active COMMIT, IRQ 및 진단 경로
- PACKED17 VDMA Frame과 PS H-Line/Ethernet 참조 처리
- TDC-GPX IOB 경계와 200 MHz 타이밍 보완
- Vivado 2025.2.1 V2 IP-XACT 패키지와 Customize IP XGUI
- XGUI 생성 파일을 그대로 보존하고 `component.xml`보다 새 시각으로 유지하는
  Preview 유효성 계약

V3 HLS 구현과 `system_integration/v3` 디렉터리는 이 브랜치에 포함하지 않는다.

## 3. 기준 이력

- V2 보드 없는 Sign-off 기준: `d8f7953`
- 이후 V2 공통 경로에 반영된 PS/VDMA, 상태, TDC-GPX IOB 타이밍 보완 포함
- V2 Native IP Packager 및 XGUI Preview 복구: `8edfd75`, `99b9db8`의
  V2 전용 결과를 이 브랜치에 반영

위 커밋 번호는 원 개발 브랜치에서 변경의 출처를 추적하기 위한 기준이다.
이 브랜치의 실제 제품 기준은 항상 브랜치 HEAD다.

## 4. 정본 소유권

| 항목 | 정본 경로 |
|---|---|
| V2 RTL | `system_integration/v2/rtl`, `system_integration/v2/pkg` |
| 공통 AXI4-Lite RTL | `system_integration/common/rtl/csr` |
| 공통 TDC-GPX 물리 RTL | 저장소 루트의 `tdc_gpx_*.vhd` |
| V2 테스트벤치 | `system_integration/v2/tb` |
| XGUI 원본 | `system_integration/v2/ip_package/tdc_gpx_lidar_ctrl_v2_xgui.tcl` |
| 배포 IP | `system_integration/v2/ip_repo/tdc_gpx_lidar_ctrl_v2_2_0` |
| CSR/운용 문서 | `system_integration/v2_architecture` |

생성 프로젝트, Vivado 로그, Waveform과 임시 패키징 결과는 `.work`, `tmp`,
`signoff_results/sessions`에만 두며 제품 소스로 커밋하지 않는다.

## 5. 브랜치 확인 절차

V2 패키지, XGUI freshness, IP-XACT 계약과 대표 OOC 구성을 한 번에 확인한다.

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/scripts/run_v2_ip_package.ps1
```

필수 종료 표시는 다음과 같다.

- `LIDAR_V2_K010_IP_PACKAGE_CHECK_PASS`
- `LIDAR_V2_IP_PACKAGE_RUNNER_PASS`

전체 보드 없는 Sign-off를 입력 Hash와 함께 다시 실행할 때는 다음 명령을 사용한다.

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/scripts/run_v2_k14_signoff.ps1
```

## 6. Sign-off 경계

이 브랜치는 기존 보드 없는 RTL/CDC/Timing/IP 패키지 검증을 완결된 V2 기준으로
보존한다. 실제 PCB의 40 MHz TDC-GPX 기준 클럭, 외부 GPX/VDMA/DDR cache API,
Ethernet, 레이저 및 Echo LVDS 계측은 실물 보드 증거가 필요하므로 이 브랜치의
보드 없는 완료 선언에 포함되지 않는다.

## 7. 브랜치 생성 시 재검증 결과

2026-08-21에 외부 `../ip_repo`가 없는 분리된 Git worktree에서 다음을 확인했다.

- V2 Package 재생성 및 IP 무결성 DRC PASS
- Native XGUI와 IP-XACT XGUI 계약 PASS
- XGUI Preview freshness PASS
- V2 단독 IP Catalog 등록 PASS
- 32-bit ASYNC 150/200 MHz OOC 합성 PASS
- 128-bit ASYNC 200/150 MHz, Echo Receiver 비활성 OOC 합성 PASS
- 64-bit SYNC 150/150 MHz OOC 합성 PASS
- 세 구성 모두 블랙박스 0, 오류 0, Critical Warning 0

대표 종료 표시는 `LIDAR_V2_K010_PACKAGED_OOC_MATRIX_PASS profiles=3`과
`LIDAR_V2_K010_IP_PACKAGE_SIGNOFF_PASS`다.
