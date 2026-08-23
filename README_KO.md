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
