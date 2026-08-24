# TDC-GPX LiDAR Controller V3

이 브랜치는 검증된 V2 제어/물리 RTL과 V3 HLS 데이터 처리 경로를 함께 유지보수하기
위한 독립 작업공간이다.

## 먼저 읽을 문서

1. [`MAINTENANCE_WORKSPACE_KO.md`](MAINTENANCE_WORKSPACE_KO.md)
   - 보존 원본과 복사본의 관계
   - V2 유지 RTL, V3 HLS/Adapter와 배포 IP의 정본 소유권
   - 생성물과 Git 브랜치 운용 규칙
2. [`system_integration/v3/V3_COMPLETE_BRANCH_KO.md`](system_integration/v3/V3_COMPLETE_BRANCH_KO.md)
   - V3 완료 기준, 두 IP 패키지 방식과 Sign-off 경계
3. [`system_integration/v3/README.md`](system_integration/v3/README.md)
   - H0~H6 단계, RTL/HLS 경계와 현재 검증 현황
4. [`system_integration/v3/docs/V3_HLS_CODE_READING_GUIDE_KO.md`](system_integration/v3/docs/V3_HLS_CODE_READING_GUIDE_KO.md)
   - H1~H4 HLS 함수, 상태와 데이터 흐름
5. [`system_integration/v3/docs/V3_GUI_PROJECT_GUIDE_KO.md`](system_integration/v3/docs/V3_GUI_PROJECT_GUIDE_KO.md)
   - Vitis HLS와 Vivado 검증 프로젝트를 GUI로 여는 절차
6. [`system_integration/v3/docs/V3_IP_PACKAGER_MAINTENANCE_GUIDE_KO.md`](system_integration/v3/docs/V3_IP_PACKAGER_MAINTENANCE_GUIDE_KO.md)
   - IP-XACT, File Group과 XGUI 유지보수 절차

## 첫 점검

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File scripts/maintenance/audit_workspace.ps1 `
  -Profile V3 -RequireCopyIsolation -DeepGitCheck
```

성공 표시는 `LIDAR_V3_MAINTENANCE_WORKSPACE_AUDIT_PASS`다. 이 점검은 파일을
삭제하거나 수정하지 않는다.

## GUI 프로젝트 바로 열기

유지보수 복사본 `C:\fpga_ws\tdc_gpx_v3`에는 다음 생성 프로젝트를 둔다.

| 목적 | 프로젝트 |
|---|---|
| 생성 RTL 포함형 IP Packager | `.work\v3_ip_packager\tdc_gpx_lidar_ctrl_v3_ip_packager.xpr` |
| HLS 하위 IP 참조형 IP Packager | `.work\v3_hls_ip_packager\tdc_gpx_lidar_ctrl_v3_hls_ip_packager.xpr` |
| 32-bit 통합 Parent Block Design | `.work\v3_parent_l0\w32\project_4_lidar_v3_l0.xpr` |
| 64-bit 통합 Parent Block Design | `.work\v3_parent_l0\w64\project_4_lidar_v3_l0.xpr` |
| H1~H4 Vitis HLS Component | `.work\v3_hls_*_component\vitis-comp.json` |

전체 존재 여부와 재개방 가능 여부는 다음 명령으로 확인한다.

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v3/scripts/open_v3_gui_projects.ps1 `
  -Target All -ValidateOnly
```

상세 열기·재생성 절차와 최종 구현 checkpoint 위치는
[`system_integration/v3/docs/V3_GUI_PROJECT_GUIDE_KO.md`](system_integration/v3/docs/V3_GUI_PROJECT_GUIDE_KO.md)를 따른다.
