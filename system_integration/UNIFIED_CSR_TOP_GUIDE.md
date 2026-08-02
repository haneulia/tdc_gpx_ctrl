# LiDAR Unified CSR Top 사용 및 검증 가이드

문서 기준일: 2026-08-03

검증된 IP-XACT core revision: `3`

대상 RTL: `system_integration/rtl/lidar_unified_csr_top.vhd`

IPI 패키지 wrapper: `system_integration/rtl/lidar_unified_csr_ip_top.vhd`

## 1. 역할과 경계

`lidar_unified_csr_top`은 통합 모드에서 유일한 AXI4-Lite register owner다.
내부에 source-level `my_axil_csr32_top` 한 개만 두며 다음 기능만 수행한다.

1. 32 CTL / 32 STAT / 4 IRQ register 주소 decode
2. CTL0..25를 각 IP의 unified adapter 입력으로 고정 배선
3. IP status와 static TDC geometry를 STAT0..31에 고정 배선
4. config/reset epoch match, busy, reject, valid를 STAT31에 집계
5. Motor/Laser/Echo/TDC interrupt identity를 32-bit source에 고정 배선

Motor 위치, Laser fire/fire_done, TDC START/STOP, Echo STOP, GPX 28-bit bus,
VDMA AXIS는 이 모듈을 통과하지 않는다. 이 경계 덕분에 CSR 통합이 실시간 지연이나
200 MHz TDC 처리 경로를 늘리지 않는다.

## 2. 연결 원칙

- 모든 child IP는 `g_ENABLE_LOCAL_CSR=false`로 합성한다.
- `s_axi_csr_aclk`와 `s_axi_csr_aresetn`은 각 child unified configuration
  clock/reset에도 동일하게 연결한다.
- `o_sys_ctrl`은 Motor, Laser, Echo, TDC에 fan-out한다.
- `o_sys_cfg_apply`는 shared config epoch을 사용하는 Motor, Laser, TDC에
  fan-out한다. Echo profile은 CTL15/16의 독립 toggle/ack protocol을 사용한다.
- 각 IP의 status/transaction/IRQ output은 이 top의 동일 이름 input으로만 연결한다.
- 각 IP의 기존 local `s_axi`와 local IRQ는 Block Design에서 숨겨져야 한다.

Clock fan-out은 parent에서 직접 연결한다. 이 RTL은 AXI clock을 새 clock output으로
재생성하거나 buffer하지 않는다.

### IPI wrapper

`lidar_unified_csr_ip_top`은 기능 또는 레지스터를 추가하지 않는다. 공통
`SYS_CTRL`과 `SYS_CFG_APPLY`를 Motor/Laser/Echo/TDC별 물리 포트로 fan-out하여
Vivado가 다음 네 사용자 정의 Master 인터페이스를 각각 하나의 소유 포트 집합으로
묶을 수 있게 한다.

- `motor_unified_csr`
- `laser_unified_csr`
- `echo_unified_csr`
- `tdc_unified_csr`

XGUI에서는 ABI version/capability를 읽기 전용으로 표시하고, register/interrupt/IPI
연결 계약을 탭별로 보여준다. 변경 가능한 합성 파라미터는 없다.

## 3. Reset 이후 초기화

통합 CSR의 CTL0..31 reset 값은 모두 0이다. Child adapter의 generic 초기 active 값과
software-visible staging 값이 우연히 같은 것으로 가정하지 않는다. 따라서 Laser는
CTL0 `LASER_EN=0` 상태로 유지하고 다음 full-profile 절차를 반드시 수행한다.

```text
1. CTL2..CTL25 중 사용되는 모든 staging/config word 작성
2. Echo indexed delay가 필요하면 CTL15/16 write/apply ack 완료
3. CTL1.CFG_EPOCH 변경
4. STAT31[18] ALL_CFG_ACCEPTED = 1 확인
5. 필요 시 CTL0.RESET_EPOCH 변경
6. STAT31[19] ALL_RESET_ACCEPTED = 1 확인
7. STAT31[21:20] ANY_REJECT/ANY_BUSY = 0 확인
8. 마지막에 CTL0.LASER_EN 설정
```

Epoch는 8-bit modulo 값이며 0으로 wrap될 수 있다. 값이 0인지가 아니라 adapter의
match와 valid bit를 기준으로 완료를 판정한다.

## 4. System status

| STAT | 내용 |
|---:|---|
| 0 | `0x4C010000`: LiDAR signature, ABI 1.0, RTL revision 0 |
| 1 | IP/transaction capability와 active CTL/STAT/IRQ source 수 |
| 2 | requested cfg/reset epoch, Laser/TDC accepted cfg epoch |
| 3..5 | static TDC max rows, cell size, max HSIZE |
| 6..11 | Motor |
| 12..18 | Laser |
| 19..22 | Echo |
| 23..29 | TDC runtime/read result |
| 30 | CTL20 index가 선택한 active GPX image word readback |
| 31 | adapter transaction 요약 |

STAT31 bit 순서:

| Bits | 내용 |
|---:|---|
| 2:0 | Motor/Laser/TDC config match + valid |
| 6:3 | Motor/Laser/Echo/TDC reset match |
| 10:7 | Motor/Laser/Echo/TDC busy |
| 14:11 | Motor/Laser/Echo/TDC reject |
| 17:15 | Motor/Laser/TDC valid |
| 18 | shared config 전체 승인 |
| 19 | reset 전체 승인 |
| 20 | 하나 이상의 busy |
| 21 | 하나 이상의 reject |

Echo busy/reject는 STAT21의 profile apply pending과 command reject에서 나온다. TDC
busy/reject는 config, serialized command, indexed image 경로를 모두 포함한다.

## 5. Interrupt

| Global bits | Source |
|---:|---|
| 0..3 | system reserved, 현재 0 |
| 4..7 | Motor |
| 8..10 | Laser |
| 11..15 | reserved, 0 |
| 16..20 | Echo 5-bit cause bus; current source uses 16..17 and drives 18..20 low |
| 21..27 | TDC-GPX |
| 28..31 | reserved, 0 |

`my_axil_csr32_top`은 source를 2-FF로 동기화하고 안정된 두 번째 단계와 별도
history FF를 비교해 rising event를 `INTR_FLAG`에 저장한다. 첫 synchronizer FF는
edge detector가 직접 소비하지 않는다.
운용 기본은 manual mode이며 ISR이 `INTR_FLAG`를 W1C할 때까지 pending이 유지된다.
IP-XACT interrupt sensitivity는 기존 CSR32 호환성을 따라 `EDGE_RISING`이다. PS GIC와
parent interrupt fabric도 이 계약에 맞춰야 한다.

## 6. 검증 결과

| Gate | 결과 |
|---|---|
| Static address/capability contract | `UNIFIED_CSR_CONTRACT_PASS` |
| AXI/control/status/epoch/IRQ TB | `[SUMMARY] Passed=7 Failed=0` |
| xc7z020clg484-2 synthesis | 0 errors, 0 critical warnings |
| Packaged-IP source synchronization | `LIDAR_UNIFIED_CSR_IP_SOURCE_SYNC_PASS` |
| Canonical/package 구성 파일 | 18/18 byte-identical |
| Four child custom-bus connections | `LIDAR_UNIFIED_CSR_IP_CHILD_INTERFACE_PASS` |
| Parent topology: combined Motor/Laser + Echo + TDC | `LIDAR_UNIFIED_CSR_IP_PARENT_INTERFACE_PASS` |
| Packaged-IP OOC synthesis | `LIDAR_UNIFIED_CSR_PACKAGED_IP_OOC_PASS` |
| Black boxes | 0 |
| CSR32 owners | 1 |
| Legacy/local CSR owners | 0 |
| Post-synth utilization | 1,021 LUT, 2,250 FF |

합성 warning은 `my_axil_csr32`가 AXI `AWPROT/ARPROT` 값을 기능에 사용하지 않아
해당 register/load가 제거됐다는 내용뿐이다.

검증 명령:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/scripts/run_unified_csr_contract.ps1

powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/scripts/run_unified_csr_top.ps1

vivado.bat -mode batch `
  -source system_integration/scripts/test_unified_csr_top_synth.tcl `
  -tclargs C:/tmp/unified_csr_top_synth
```

## 7. 별도 IP repository

생성 위치:

```text
C:\Projects\my_sp\lib\IP\lidar_unified_csr\ip_repo
```

Revision 3은 register ABI를 변경하지 않는다. 이전 package revision 2에 남아 있던
`lidar_unified_csr_pkg.vhd`, `lidar_unified_csr_top.vhd`,
`lidar_unified_csr_ip_top.vhd`, `axil_intr_32.vhd`의 오래된 snapshot을 canonical
소스로 갱신한 배포 revision이다. Package check는 CSR32 6개, unified RTL 3개,
XGUI 1개, child interface XML 8개 등 총 18개 파일을 byte 단위로 비교한다.

canonical RTL과 패키징 스크립트는 이 Git 저장소에서 관리하고, 위 `ip_repo`는
재생성 가능한 배포 산출물로 관리한다. 패키지에는 `my_axil_csr32` 소스가 포함되며
child XCI나 `../HDL` 외부 참조는 포함되지 않는다.

실제 parent가 사용하는 `motor_laser_ctrl` 패키지는 local AXI 두 개 대신
`motor_unified_csr`과 `laser_unified_csr` 두 Slave interface를 동시에 제공한다.
`check_lidar_unified_csr_ip_package.tcl`은 중앙 Master 네 개를 결합 Motor/Laser,
Echo, TDC에 연결해 parent 배치 전 bus 호환성을 검사한다.

패키지 생성:

```powershell
& 'C:\AMDDesignTools\2025.2.1\Vivado\bin\vivado.bat' `
  -mode batch -nojournal `
  -source 'C:\Projects\my_sp\lib\IP\tdc_gpx_ctrl\HDL\package_lidar_unified_csr_ip.tcl'
```

패키징 작업 프로젝트는 기본적으로 `%TEMP%\lidar_unified_csr_package_work`에서
생성 후 삭제된다. 저장소 내부의 `.package_work_unified_csr`는 더 이상 기본
패키징 경로로 사용하지 않는다.

패키지/XGUI/source-sync 및 현재 child IP 연결 검사:

```powershell
& 'C:\AMDDesignTools\2025.2.1\Vivado\bin\vivado.bat' `
  -mode batch -nojournal -nolog `
  -source 'C:\Projects\my_sp\lib\IP\tdc_gpx_ctrl\HDL\system_integration\scripts\check_lidar_unified_csr_ip_package.tcl'
```

생성 패키지 소스 OOC 합성:

```powershell
& 'C:\AMDDesignTools\2025.2.1\Vivado\bin\vivado.bat' `
  -mode batch -nojournal -nolog `
  -source 'C:\Projects\my_sp\lib\IP\tdc_gpx_ctrl\HDL\system_integration\scripts\run_lidar_unified_csr_packaged_ip_ooc.tcl'
```

중앙과 child repository가 동일한 사용자 정의 interface XML을 포함하므로 Catalog
갱신 시 duplicate-interface warning이 나타날 수 있다. 정적 검사는 네 XML 쌍이
byte 단위로 동일한지 먼저 확인하므로 중앙 repository가 우선 선택되어도 연결 계약은
바뀌지 않는다.

Parent Project Settings의 IP Repository에는 중앙 repository와 실제로 사용하는
Motor/Laser/Echo/TDC repository를 함께 등록한다. Child IP의
`g_ENABLE_LOCAL_CSR=false` 설정 후 같은 이름의 Master/Slave 인터페이스를 직접
연결하고, `s_axi_csr_aclk/aresetn`을 모든 child unified configuration clock/reset에
fan-out한다.

## 8. Closure 범위

통합 **제어 평면 RTL과 배포 IP**는 sign-off 상태다. 다음 단계에서는 실제 parent
Block Design에 네 IP를 unified mode로 연결한 뒤 150/200 MHz 신호처리 회귀를
수행해야 한다. 그때 local/unified mode의 shot 수, START/STOP, Return-7, 28-bit
I-Mode, 17-bit Hit, VDMA geometry, IRQ와 HTML contract가 동일해야 전체 시스템
sign-off가 된다.
