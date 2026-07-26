# TDC-GPX IP Package 업데이트 가이드

## 1. Source of truth

패키지는 다음 canonical source에서 만들어진다.

- TDC-GPX RTL: `C:\Projects\my_sp\lib\IP\tdc_gpx_ctrl\HDL`
- CSR8 RTL: `C:\Projects\my_sp\lib\IP\my_axil_csr\HDL`
- CSR32 RTL: `C:\Projects\my_sp\lib\IP\my_axil_csr32\HDL`
- XGUI: `tdc_gpx_xgui.tcl`

출력 위치:

`C:\Projects\my_sp\lib\IP\tdc_gpx_ctrl\ip_repo`

패키지는 source-only이다. generated CSR `.xci`, project-relative `../HDL` 참조, testbench를 포함하지 않는다.

## 2. 권장 업데이트 순서

1. RTL과 XGUI를 수정한다.
2. standalone project의 generated CSR 참조를 canonical source로 동기화한다.
3. RTL regression을 수행한다.
4. package script를 실행한다.
5. package static check와 packaged-IP OOC synthesis를 수행한다.
6. 변경과 검증 결과를 Git commit으로 고정한다.

## 3. Standalone project 동기화

Vivado Tcl Console:

```tcl
source C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/scripts/sync_tdc_gpx_project_sources.tcl
```

이 스크립트는 다음 작업을 수행한다.

- TDC-GPX와 sibling IP에 남아 있는 generated CSR XCI 참조 제거
- 리팩터링 전에 사용하던 삭제된 sibling RTL 참조 제거
- canonical CSR, virtual encoder, motor decoder, laser controller, echo receiver RTL/TB 재등록
- compile order 갱신

generated directory와 원본 HDL 파일 자체는 삭제하지 않는다.

동기화 결과 확인:

```tcl
source C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/scripts/check_tdc_gpx_project_sources.tcl
```

정상 marker는 `TDC_GPX_PROJECT_SOURCE_CHECK_PASS`이다.

## 4. 패키지 생성

### PowerShell에서 실행

```powershell
& 'C:\AMDDesignTools\2025.2.1\Vivado\bin\vivado.bat' `
  -mode batch -nojournal `
  -source 'C:\Projects\my_sp\lib\IP\tdc_gpx_ctrl\HDL\package_tdc_gpx_ip.tcl'
```

PowerShell 방식은 shell에서 exit code, log 경로, 후속 test를 자동화하기 쉽다.

### Vivado Tcl Console에서 실행

```tcl
source C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/package_tdc_gpx_ip.tcl
```

GUI Tcl Console 방식은 현재 Vivado session에서 바로 package를 갱신할 때 편리하다. 스크립트 끝의 `exit` 때문에 현재 session을 종료하므로, GUI에서 계속 작업해야 한다면 별도 batch Vivado를 권장한다.

정상 marker:

```text
TDC_GPX_IP_PACKAGER_DRC=1
TDC_GPX_IP_PACKAGE_PASS
```

## 5. 자동 검사

### IP-XACT와 XGUI

```powershell
& 'C:\AMDDesignTools\2025.2.1\Vivado\bin\vivado.bat' `
  -mode batch -nojournal -nolog `
  -source 'C:\Projects\my_sp\lib\IP\tdc_gpx_ctrl\HDL\scripts\check_tdc_gpx_ip_package.tcl'
```

검사 항목:

- 두 AXI4-Lite, 두 AXI4-Stream, 세 clock, 두 interrupt interface
- clock generic에 종속된 `FREQ_HZ`
- `g_NUM_CHIPS`에 종속된 물리 핀 폭
- canonical CSR source 포함 및 `.xci`/외부 경로 부재
- 유효 topology와 잘못된 topology XGUI 예외처리
- 150/200 MHz ASYNC 허용, AXIS>TDC 및 split SYNC 거부

### Packaged-IP OOC synthesis

```powershell
& 'C:\AMDDesignTools\2025.2.1\Vivado\bin\vivado.bat' `
  -mode batch -nojournal `
  -source 'C:\Projects\my_sp\lib\IP\tdc_gpx_ctrl\HDL\scripts\run_tdc_gpx_packaged_ip_ooc.tcl'
```

기준 profile은 32-bit output, 4 chip, AXIS 150 MHz, TDC 200 MHz이다.

Tcl에서 vector generic을 직접 설정할 때는 VHDL 문자열의 따옴표까지
CONFIG 값에 포함한다. GUI Customize IP에서는 네 자리 binary 값만 입력한다.

```tcl
set_property -dict [list \
  CONFIG.g_PRESENT_CHIP_MASK {"1111"} \
  CONFIG.g_RISE_CHIP_MASK    {"0011"} \
  CONFIG.g_FALL_CHIP_MASK    {"1100"}] [get_ips tdc_gpx_top_0]
```

## 6. Vivado User Repository 반영

1. Project Settings > IP > Repository에서 `...\tdc_gpx_ctrl\ip_repo`를 추가한다.
2. Refresh 또는 `update_ip_catalog -rebuild`를 수행한다.
3. `VictekIP > TDC-GPX Multi-chip Acquisition Controller`를 선택한다.
4. Customize IP에서 clock/topology validation이 정상인지 확인한다.

Package IP GUI의 `Merge changes`를 source 동기화 방법으로 사용하지 않는다. 이 프로젝트는 canonical HDL을 package script가 `ip_repo/src`로 복사하고 file group을 다시 작성한다. 따라서 `src/...`가 `../HDL/...`로 바뀌는 문제를 피한다.

## 7. Git 관리 주의

현재 Git worktree root는 `...\tdc_gpx_ctrl\HDL`이다. 실제 package 출력 `...\tdc_gpx_ctrl\ip_repo`는 이 worktree 바깥이므로 이 repository commit에는 포함되지 않는다. package는 이 가이드의 deterministic script로 재생성하며, `ip_repo` 자체를 추적하려면 상위 폴더를 포함하는 별도 Git repository 정책이 필요하다.
