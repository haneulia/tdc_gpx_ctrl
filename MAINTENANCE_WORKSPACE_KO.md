# TDC-GPX LiDAR V2 유지보수 작업공간

## 1. 목적

이 작업공간은 기존 개발 폴더를 그대로 보존하고, 검증된 V2 기준을 독립 복사본에서
유지보수하기 위한 공간이다. 소스 이동이 아니라 Git 이력을 포함한 완전한 복사본을
사용하므로 기존 Vivado 프로젝트, 로그와 작업 중 파일은 영향을 받지 않는다.

| 구분 | 경로 또는 기준 |
|---|---|
| 보존 원본 | `C:\Projects\my_sp\lib\IP\tdc_gpx_ctrl\HDL` |
| V2 유지보수 복사본 | `C:\fpga_ws\tdc_gpx_v2` |
| 고정 완료 브랜치 | `codex/tdc-gpx-lidar-v2-complete` |
| 완료 기준 커밋 | `8c97dd251b70ee602305bfc2eb7ca51ddebc5c6e` |
| 일상 유지보수 브랜치 | `codex/tdc-gpx-lidar-v2-maintenance` |

완료 브랜치는 재현 기준이므로 직접 수정하지 않는다. 모든 변경은 유지보수 브랜치
또는 그 브랜치에서 분기한 짧은 기능 브랜치에서 수행한다.

## 2. 정본 소유권

같은 VHDL이나 문서가 `ip_repo`에도 보이더라도 다음 경로만 직접 수정한다.

| 대상 | 직접 수정하는 정본 | 비고 |
|---|---|---|
| V2 Package | `system_integration/v2/pkg` | 합성용 자료형, CSR 및 GPX 계약 |
| V2 RTL | `system_integration/v2/rtl` | 통합 제어와 데이터 처리 RTL |
| 공통 AXI4-Lite CSR | `system_integration/common/rtl/csr` | V2/V3가 함께 사용하는 전송 및 IRQ RTL |
| TDC-GPX 물리 RTL | 저장소 루트의 `tdc_gpx_*.vhd`와 `px_utility_pkg.vhd` | GPX bus PHY, Chip 제어와 공통 기반 |
| V2 테스트벤치 | `system_integration/v2/tb` | V2 통합 경계의 정본 테스트 |
| 공통/기존 GPX 테스트벤치 | 저장소 루트의 `tb_tdc_gpx_*.vhd` | 물리 GPX 기반 RTL 회귀 |
| XGUI | `system_integration/v2/ip_package/tdc_gpx_lidar_ctrl_v2_xgui.tcl` | Customize IP 화면의 정본 |
| 패키징 목록 | `system_integration/v2/ip_package/v2_ip_package_manifest.tcl` | IP에 포함되는 파일 계약 |
| 배포 IP | `system_integration/v2/ip_repo/tdc_gpx_lidar_ctrl_v2_2_0` | 패키징으로 재생성되는 결과, 직접 수정 금지 |
| PS 참조 코드 | `system_integration/v2/sw_reference` | PACKED17, VDMA, cache 및 Viewer 참조 |

`ip_repo`는 사용자가 IP Catalog에 등록하는 배포 사본이다. `ip_repo/src` 또는
`ip_repo/xgui`를 먼저 고치면 다음 패키징 때 덮어써지므로, 반드시 위 정본을 수정한
뒤 패키징 스크립트로 갱신한다.

## 3. 현재 폴더 구조를 유지하는 이유

루트의 TDC-GPX 물리 RTL과 테스트벤치를 곧바로 `core/rtl`, `core/tb`로 옮기면
Vivado File Group, IP-XACT `component.xml`, Tcl 컴파일 목록과 과거 회귀 스크립트가
동시에 바뀐다. 기능 변경 없이 경로만 바꾸어도 검증 범위가 커진다.

따라서 이번 작업공간에서는 파일을 이동하지 않고 정본 소유권을 먼저 고정한다.
향후 경로 정리는 다음 조건을 모두 만족하는 별도 변경으로만 진행한다.

1. 모든 참조 위치를 자동 목록화한다.
2. 패키징 전후 파일 Hash가 일치하는지 확인한다.
3. 대표 시뮬레이션과 V2 패키지 OOC 회귀를 통과한다.
4. IP-XACT 저장 후 XGUI를 다시 열어 Preview와 Customize IP를 확인한다.

## 4. 생성물 규칙

다음 항목은 다시 만들 수 있는 생성물이며 제품 소스가 아니다.

- `.work`, `.Xil`, `.gen`, `.srcs`, `tmp`, `xsim.dir`
- Vivado/Vitis 실행 로그, Journal, Waveform, 중간 Object
- `signoff_results/sessions`, `signoff_results/matrices`
- 실행 중 만들어지는 `2025.2.1`, `tclapp` cache

이 파일은 Git에 추가하지 않는다. 디스크 정리가 필요할 때는 현재 실행 중인 Vivado,
Vitis HLS와 xsim을 먼저 닫고, 재현 가능한 요약 보고서와 PASS marker만 보존한다.
자동 점검기는 생성물의 존재를 보고만 하며 삭제하지 않는다.

### 4.1 유지보수 복사본에 두는 GUI 프로젝트

`C:\fpga_ws\tdc_gpx_v2\.work`에는 다음 항목을 재생성 가능한 GUI 작업물로
유지한다.

| 구분 | 상대 경로 | 역할 |
|---|---|---|
| V2 IP Packager | `v2_ip_packager\tdc_gpx_lidar_ctrl_v2_ip_packager.xpr` | File Group, Parameter, Port, XGUI와 IP-XACT 확인 |
| V2 통합 IP 검증 | `tdc_gpx_lidar_ctrl_v2_gui\tdc_gpx_lidar_ctrl_v2_gui.xpr` | 32/64/128-bit 및 SYNC/ASYNC 대표 구성 확인 |

두 Project는 `.work` 생성물이므로 Git에 넣지 않는다. 정본 RTL/XGUI를 변경한 뒤
각 실행기로 재생성하며, Project 내부 생성 VHDL을 직접 제품 소스로 수정하지 않는다.

## 5. 표준 유지보수 순서

1. `codex/tdc-gpx-lidar-v2-maintenance`를 최신 상태로 맞춘다.
2. 아래 비파괴 점검기를 실행하여 기준 커밋과 정본 경로를 확인한다.
3. 기능 변경은 유지보수 브랜치에서 짧은 기능 브랜치를 새로 만든다.
4. 정본 파일과 해당 테스트벤치를 함께 수정한다.
5. 영향 범위의 집중 회귀를 먼저 통과시킨다.
6. IP에 반영되는 변경이면 V2 패키징 회귀로 `ip_repo`를 재생성한다.
7. 전체 Sign-off가 필요한 변경만 K1-4 절차를 수행한다.
8. 점검기, `git diff --check`, 테스트 결과를 확인한 뒤 커밋한다.

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File scripts/maintenance/audit_workspace.ps1 `
  -Profile V2 -RequireCopyIsolation
```

V2 완료 기준과 재현 명령은
`system_integration/v2/V2_COMPLETE_BRANCH_KO.md`를 따른다.

## 6. 원격 저장소와 원본 보호

- `origin`은 GitHub 원격 저장소다.
- `source-local`은 보존 원본을 읽어 오는 로컬 원격이다.
- 복사본의 `source-local` push URL은 `disabled://read-only`로 잠근다.
- 원본 폴더에서 발생한 새 커밋이 꼭 필요할 때만 `source-local`에서 fetch한다.
- 유지보수 결과는 검증 후 `origin`에 push하며, 보존 원본으로 직접 push하지 않는다.

이 규칙은 복사본과 원본이 우연히 같은 작업 디렉터리처럼 섞이는 것을 방지한다.
