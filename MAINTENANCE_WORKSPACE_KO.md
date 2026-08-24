# TDC-GPX LiDAR V3 유지보수 작업공간

## 1. 목적

이 작업공간은 기존 개발 폴더를 그대로 보존하고, 검증된 V3 RTL/HLS 혼합 기준을
독립 복사본에서 유지보수하기 위한 공간이다. 소스 이동이 아니라 Git 이력을 포함한
완전한 복사본을 사용하므로 기존 Vivado 프로젝트, 로그와 작업 중 파일은 영향을
받지 않는다.

| 구분 | 경로 또는 기준 |
|---|---|
| 보존 원본 | `C:\Projects\my_sp\lib\IP\tdc_gpx_ctrl\HDL` |
| V3 유지보수 복사본 | `C:\fpga_ws\tdc_gpx_v3` |
| 고정 완료 브랜치 | `codex/tdc-gpx-lidar-v3-complete` |
| 완료 기준 커밋 | `bd4041474d16c2fe856de410b346ab26ffc73885` |
| 일상 유지보수 브랜치 | `codex/tdc-gpx-lidar-v3-maintenance` |

완료 브랜치는 보드 없는 Sign-off 재현 기준이므로 직접 수정하지 않는다. 모든 변경은
유지보수 브랜치 또는 그 브랜치에서 분기한 짧은 기능 브랜치에서 수행한다.

## 2. 정본 소유권

V3는 V2 제어/물리 RTL을 같은 체크아웃 안에서 공유한다. 같은 파일이 `ip_repo`에도
보이더라도 다음 경로만 직접 수정한다.

| 대상 | 직접 수정하는 정본 | 비고 |
|---|---|---|
| 유지 제어/물리 Package | `system_integration/v2/pkg` | V3가 재사용하는 V2 계약 |
| 유지 제어/물리 RTL | `system_integration/v2/rtl` | CSR, Motor/Face/Shot/Laser, GPX 수집 |
| 공통 AXI4-Lite CSR | `system_integration/common/rtl/csr` | V2/V3 공통 전송 및 IRQ RTL |
| TDC-GPX 물리 기반 RTL | 저장소 루트의 `tdc_gpx_*.vhd`와 `px_utility_pkg.vhd` | GPX bus PHY와 Chip 제어 기반 |
| V3 HLS C++/Header | `system_integration/v3/hls` | H1 Raw28부터 H4 Word 생성까지 |
| V3 RTL Adapter/Top | `system_integration/v3/rtl` | HLS와 유지 RTL의 경계 및 통합 Top |
| V3 테스트벤치 | `system_integration/v3/tb` | HLS 차등, CDC, 통합 경계의 정본 테스트 |
| XGUI | `system_integration/v3/ip_package/tdc_gpx_lidar_ctrl_v3_xgui.tcl` | 두 V3 패키지가 공유하는 Customize IP 정본 |
| 패키징 목록 | `system_integration/v3/ip_package/v3_ip_package_manifest.tcl` | IP에 포함되는 파일 계약 |
| 생성 RTL 포함형 IP | `system_integration/v3/ip_repo/tdc_gpx_lidar_ctrl_v3_3_0` | Parent 배포 기준, 직접 수정 금지 |
| HLS 하위 IP 참조형 IP | `system_integration/v3/ip_repo/tdc_gpx_lidar_ctrl_v3_hls_ip_3_0` | HLS GUI 유지보수용, 직접 수정 금지 |
| HLS 하위 IP 4개 | `system_integration/v3/ip_repo/gpx_*_hls_3_0` | Vitis HLS export 결과 |
| PS 참조 코드 | `system_integration/v3/sw_reference` | VDMA/DDR/cache/H-Line/Ethernet 참조 |

`ip_repo`는 IP Catalog 등록용 배포 사본이다. 생성 RTL 포함형과 HLS 하위 IP
참조형 모두 정본을 수정한 뒤 각 패키징 스크립트로 재생성한다. `component.xml`,
`ip_repo/src`와 생성된 XGUI를 직접 고치는 방식은 다음 재패키징에서 사라진다.

## 3. V2와 V3의 변경 경계

- GPX bus PHY, CDC, AXI4-Lite CSR, Motor decoder, Face tracker, Shot scheduler와
  Laser executor의 수정은 V2 유지 RTL에도 영향을 주므로 V2 회귀를 함께 판단한다.
- H1~H4 데이터 처리, HLS Adapter와 V3 통합 Top만 바뀌면 V3 회귀를 수행한다.
- 공통 CSR를 바꾸면 V2/V3의 Register ABI와 IRQ를 모두 점검한다.
- V3 제품 AXI4-Stream 폭은 합성 시 32 또는 64 bit이며 128 bit는 범위 밖이다.

## 4. 현재 폴더 구조를 유지하는 이유

루트의 TDC-GPX 물리 RTL을 곧바로 새 `core/rtl`로 옮기면 V2/V3의 Vivado File
Group, IP-XACT, HLS 통합 Tcl과 회귀 컴파일 목록이 동시에 바뀐다. 기능과 관계없는
경로 변경이 보드 없는 완료 기준을 흔들 수 있다.

따라서 이번 작업공간에서는 파일을 이동하지 않고 정본 소유권을 먼저 고정한다.
향후 경로 정리는 다음 조건을 모두 만족하는 별도 변경으로만 진행한다.

1. V2/V3의 모든 Tcl, XML, VHDL 및 HLS 참조 위치를 자동 목록화한다.
2. 패키징 전후 정본 파일 Hash와 File Group을 비교한다.
3. H1~H4 C synthesis, V3 차등 회귀와 32/64-bit OOC를 통과한다.
4. 생성 RTL 포함형과 HLS 하위 IP 참조형 IP-XACT/XGUI를 다시 연다.

## 5. 생성물 규칙

다음 항목은 다시 만들 수 있는 생성물이며 제품 소스가 아니다.

- `.work`, `.Xil`, `.gen`, `.srcs`, `tmp`, `xsim.dir`
- Vivado/Vitis HLS 실행 로그, Journal, Waveform과 중간 Object
- `signoff_results/sessions`, `signoff_results/matrices`
- 실행 중 만들어지는 `2025.2.1`, `tclapp` cache

이 파일은 Git에 추가하지 않는다. 디스크 정리가 필요할 때는 Vivado, Vitis HLS와
xsim을 먼저 닫고 재현 가능한 요약 보고서, 입력 Hash와 PASS marker만 보존한다.
자동 점검기는 생성물의 존재를 보고만 하며 삭제하지 않는다.

### 5.1 유지보수 복사본에 두는 GUI 프로젝트

`C:\fpga_ws\tdc_gpx_v3\.work`에는 다음 항목을 재생성 가능한 GUI 작업물로
유지한다.

| 구분 | 상대 경로 | 역할 |
|---|---|---|
| Embedded RTL IP Packager | `v3_ip_packager\tdc_gpx_lidar_ctrl_v3_ip_packager.xpr` | 생성 RTL 포함형 IP-XACT/XGUI 확인 |
| HLS-IP IP Packager | `v3_hls_ip_packager\tdc_gpx_lidar_ctrl_v3_hls_ip_packager.xpr` | 네 HLS 하위 IP와 부모 IP-XACT 확인 |
| 32-bit Parent | `v3_parent_l0\w32\project_4_lidar_v3_l0.xpr` | PS, 두 VDMA, 32-bit Stream 연결 확인 |
| 64-bit Parent | `v3_parent_l0\w64\project_4_lidar_v3_l0.xpr` | PS, 두 VDMA, 64-bit Stream 연결 확인 |
| HLS H1~H4 | `v3_hls_*_component\vitis-comp.json` | CSim, CSynth, CoSim과 HLS 코드 유지보수 |
| 물리 Sign-off 증거 | `v3_parent_signoff\260813_200mhz_adaptive_reset_w*_impl1` | post-route DCP, Bitstream, XSA와 보고서 |

IP Packager는 정본 `system_integration/v3/ip_repo`를 직접 편집하지 않는다.
실행기가 `.work/v3_ip_packager_snapshot`에 만든 사본을 열기 때문에, GUI를 열거나
검사해도 HLS 자식 XCI에 임시 드라이브 경로가 기록되지 않는다. 제품 반영은 정본
XGUI, manifest 또는 HLS 원본을 수정한 뒤 패키징 실행기로 다시 생성한다.

## 6. 표준 유지보수 순서

1. `codex/tdc-gpx-lidar-v3-maintenance`를 최신 상태로 맞춘다.
2. 아래 비파괴 점검기로 기준 커밋, 정본 경로와 복사본 독립성을 확인한다.
3. 기능 변경은 유지보수 브랜치에서 짧은 기능 브랜치를 새로 만든다.
4. 정본 파일과 해당 HLS/RTL 테스트벤치를 함께 수정한다.
5. 가장 좁은 H1~H6 집중 회귀부터 수행한다.
6. IP 변경이면 생성 RTL 포함형과 HLS 하위 IP 참조형 중 영향받는 패키지를 갱신한다.
7. 32/64-bit 대표 OOC와 필요한 Parent 검증을 수행한다.
8. 점검기, `git diff --check`, 테스트 결과를 확인한 뒤 커밋한다.

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File scripts/maintenance/audit_workspace.ps1 `
  -Profile V3 -RequireCopyIsolation
```

V3 완료 기준과 재현 명령은
`system_integration/v3/V3_COMPLETE_BRANCH_KO.md`를 따른다.

## 7. 원격 저장소와 원본 보호

- `origin`은 GitHub 원격 저장소다.
- `source-local`은 보존 원본을 읽어 오는 로컬 원격이다.
- 복사본의 `source-local` push URL은 `disabled://read-only`로 잠근다.
- 원본 폴더에서 발생한 새 커밋이 꼭 필요할 때만 `source-local`에서 fetch한다.
- 유지보수 결과는 검증 후 `origin`에 push하며, 보존 원본으로 직접 push하지 않는다.

이 규칙은 V2/V3 복사본, 보존 원본과 Vivado 생성 프로젝트의 소유권이 섞이는 것을
방지한다.
