# TDC-GPX LiDAR 유지보수 작업공간 인덱스

이 문서는 `C:\fpga_ws\README_KO.md`의 정본이다. 기존 개발 폴더는 보존하고,
V2와 V3를 독립 복사본에서 유지보수한다.

## 버전별 작업공간

| 버전 | 작업공간 | 브랜치 | 시작 문서 |
|---|---|---|---|
| V2 | `C:\fpga_ws\tdc_gpx_v2` | `codex/tdc-gpx-lidar-v2-maintenance` | `README_KO.md` |
| V3 | `C:\fpga_ws\tdc_gpx_v3` | `codex/tdc-gpx-lidar-v3-maintenance` | `README_KO.md` |

## 주요 GUI 프로젝트

### V2

- IP Packager: `C:\fpga_ws\tdc_gpx_v2\.work\v2_ip_packager\tdc_gpx_lidar_ctrl_v2_ip_packager.xpr`
- 통합 IP 검증: `C:\fpga_ws\tdc_gpx_v2\.work\tdc_gpx_lidar_ctrl_v2_gui\tdc_gpx_lidar_ctrl_v2_gui.xpr`

V2 통합 IP 검증 Project에는 32-bit 비동기, 128-bit 비동기 및 64-bit 동기
Block Design과 각 IP의 OOC 합성 결과가 들어 있다.

### V3

- Embedded RTL IP Packager: `C:\fpga_ws\tdc_gpx_v3\.work\v3_ip_packager\tdc_gpx_lidar_ctrl_v3_ip_packager.xpr`
- HLS-IP IP Packager: `C:\fpga_ws\tdc_gpx_v3\.work\v3_hls_ip_packager\tdc_gpx_lidar_ctrl_v3_hls_ip_packager.xpr`
- 32-bit Parent: `C:\fpga_ws\tdc_gpx_v3\.work\v3_parent_l0\w32\project_4_lidar_v3_l0.xpr`
- 64-bit Parent: `C:\fpga_ws\tdc_gpx_v3\.work\v3_parent_l0\w64\project_4_lidar_v3_l0.xpr`
- HLS H1~H4: `C:\fpga_ws\tdc_gpx_v3\.work\v3_hls_*_component\vitis-comp.json`

V3 최종 배치·배선 증거는
`C:\fpga_ws\tdc_gpx_v3\.work\v3_parent_signoff\260813_200mhz_adaptive_reset_w32_impl1`과
`...w64_impl1`에 별도로 보존한다.

## 생성물 원칙

- `.work`의 Project, DCP, Bitstream, HLS report는 Git에 넣지 않는다.
- 삭제되면 각 버전 README에 기록된 실행기로 다시 만든다.
- 제품 소스는 각 복사본의 `system_integration`과 저장소 루트 RTL이다.
- Packager GUI의 임시 사본을 제품 정본으로 사용하지 않는다.
