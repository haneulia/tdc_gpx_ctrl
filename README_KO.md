# TDC-GPX LiDAR Controller V2

이 브랜치는 검증된 V2 RTL을 독립적으로 유지보수하기 위한 작업공간이다.

## 먼저 읽을 문서

1. [`MAINTENANCE_WORKSPACE_KO.md`](MAINTENANCE_WORKSPACE_KO.md)
   - 보존 원본과 복사본의 관계
   - 직접 수정할 정본 경로
   - 생성물과 Git 브랜치 운용 규칙
2. [`system_integration/v2/V2_COMPLETE_BRANCH_KO.md`](system_integration/v2/V2_COMPLETE_BRANCH_KO.md)
   - V2 완료 기준, Sign-off 범위와 재현 절차
3. [`system_integration/v2/README.md`](system_integration/v2/README.md)
   - V2 RTL 구조, 데이터 흐름과 단계별 검증 현황
4. [`system_integration/v2_architecture/V2_RTL_MAINTENANCE_GUIDE_KO.md`](system_integration/v2_architecture/V2_RTL_MAINTENANCE_GUIDE_KO.md)
   - Package, RTL 함수와 신호를 읽는 순서
5. [`system_integration/v2_architecture/V2_TESTBENCH_COVERAGE_GUIDE_KO.md`](system_integration/v2_architecture/V2_TESTBENCH_COVERAGE_GUIDE_KO.md)
   - 테스트벤치별 목적, 대상 RTL과 유지보수 규칙

## 첫 점검

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File scripts/maintenance/audit_workspace.ps1 `
  -Profile V2 -RequireCopyIsolation -DeepGitCheck
```

성공 표시는 `LIDAR_V2_MAINTENANCE_WORKSPACE_AUDIT_PASS`다. 이 점검은 파일을
삭제하거나 수정하지 않는다.

## GUI 프로젝트 바로 열기

유지보수 복사본 `C:\fpga_ws\tdc_gpx_v2`에는 다음 Vivado 프로젝트를 둔다.

| 목적 | 프로젝트 |
|---|---|
| IP-XACT/IP Packager 유지보수 | `.work\v2_ip_packager\tdc_gpx_lidar_ctrl_v2_ip_packager.xpr` |
| 3-profile 통합 IP 검증 | `.work\tdc_gpx_lidar_ctrl_v2_gui\tdc_gpx_lidar_ctrl_v2_gui.xpr` |

통합 IP 검증 Project에는 32-bit 비동기 150/200 MHz, 128-bit 비동기
200/150 MHz 및 64-bit 동기 150/150 MHz Block Design과 OOC 합성 결과가 들어
있다. 상세 확인 순서는
[`system_integration/v2_architecture/V2_K010_GUI_PROJECT_GUIDE_KO.md`](system_integration/v2_architecture/V2_K010_GUI_PROJECT_GUIDE_KO.md)를 따른다.
