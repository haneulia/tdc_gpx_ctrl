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
