# V2 Checkpoint L1 TDC-GPX IOB 및 PS Reference Closure

## 1. 결론

4-chip `tdc_gpx_lidar_ctrl_v2:2.0` Parent에서 TDC-GPX 병렬 버스 핀의 경계
레지스터를 실제 Zynq-7000 ILOGIC/OLOGIC에 고정하고, PS가 통합 CSR 설정부터
VDMA DDR 수집, PACKED17 복원 및 Viewer UDP 전송까지 수행하는 참조 코드를
추가했다.

판정은 다음과 같다.

- RTL 기능 및 4-chip Parent 구현: **PASS**
- TDC-GPX 병렬 버스 IOB 물리 배치 계약: **PASS**
- PS 참조 코드의 호스트 및 Cortex-A9 BSP 컴파일: **PASS**
- 실제 PCB 신호 무결성, DDR cache 장시간 운용 및 Ethernet 지속 처리량: **보드 검증 필요**

## 2. TDC-GPX 병렬 버스 경계

고정 PACKAGE_PIN만으로는 내부 레지스터가 매 합성마다 다른 위치에 배치될 수 있다.
따라서 `tdc_gpx_bus_phy`가 핀에 직접 연결되는 입력·출력·tri-state 레지스터에
`IOB=TRUE`를 부여하고, Parent Sign-off가 실제 LOC/BEL까지 검사한다.

| 기능 | 개수 | routed 물리 자원 |
|---|---:|---|
| 4 chips × GPX D[27:0] 입력 capture | 112 | `ILOGICE2.IFF` |
| 4 chips × GPX D[27:0] 출력 | 112 | `OLOGICE2.OUTFF` |
| 4 chips × GPX D[27:0] tri-state | 112 | `OLOGICE2.TFF` |
| 4 chips × ADR[3:0] | 16 | `OLOGICE2.OUTFF` |
| 4 chips × CSN | 4 | `OLOGICE2.OUTFF` |
| 4 chips × RDN | 4 | `OLOGICE2.OUTFF` |
| 4 chips × WRN | 4 | `OLOGICE2.OUTFF` |
| **합계** | **364** | **모두 IOB/LOC/BEL PASS** |

VT Parent는 `G_OEN_MODE=PULLUP_OR_NOT_CONNECTED`이므로 OEN을 외부로 내보내지
않는다. 다른 PCB가 동적 OEN을 사용하면 OEN도 XDC와 IOB 검사 대상에 추가한다.
`EF1/EF2/IRFLAG`는 비동기 상태 입력이므로 저지연 조합 입력으로 사용하지 않고
명시적 CDC 동기화기를 통과한다.

## 3. 200 MHz 경로 보완

초기 4-chip 구현의 유일한 위반은 Lane 상태와 BUS ready가 coordinator의
Register-read 요청 소유권 enable까지 한 사이클에 이어진 `WNS -0.007 ns`
경로였다. 외부 GPX 버스 파형이나 Runtime TDC-GPX 버스 읽기 타이밍
(`BUS_CLK_DIV/BUS_TICKS`)을 바꾸지 않고 다음처럼 보완했다.

```text
Lane ready 조합 판정
        ↓
lane_register_ready_r 1단 레지스터
        ↓
coordinator가 보존 중인 register_request_r 승인
```

요청 payload는 기존 1-entry `register_request_r`에 계속 보존되므로 Chip/Address가
유실되지 않는다. 선택 Lane은 ready가 될 때까지 요청 valid를 받으며, 응답은
outstanding 소유권이 설정될 때까지 Lane에서 보존된다.

최종 4-chip Parent 세션 `260809_iob_ps_ref_l0_impl_v2` 결과:

| 항목 | 결과 |
|---|---:|
| Synthesis WNS/WHS | `+0.357 / +0.036 ns` |
| Route WNS/WHS | `+0.240 / +0.030 ns` |
| CSR 100 MHz WNS | `+1.021 ns` |
| Processing 150 MHz WNS | `+0.686 ns` |
| TDC 200 MHz WNS | `+0.240 ns` |
| 최악 TDC 경로 | GPX bus FSM → D tri-state IOB FF, 2 LUT |
| 최악 경로 배선 비중 | `85.363%` |
| Critical CDC / bus-skew / blocking DRC | `0 / 0 / 0` |
| Bitstream | `4,045,708 bytes` |

## 4. PS 참조 데이터 흐름

```text
PS Runtime Shadow CTL 기록
  → STOP/DISARM
  → COMMIT
  → CTL25..29 VDMA profile 읽기
  → Rise/Fall VDMA HSIZE/VSIZE/STRIDE 적용
  → profile ACK
  → RUN/ARM
  → VDMA S2MM 완료 IRQ
  → DDR cache invalidate
  → PACKED17 Face/Footer/Shot/Cell 검증
  → 17-bit Hit 복원
  → Viewer Face Header + H-Line UDP 전송
  → Frame을 DMA-owned로 반환
```

참조 코드는 `system_integration/v2/sw_reference/zynq_ps_example`에 있다.

- `lidar_v2_ps_control`: CSR ABI 2.7, Shadow/COMMIT, VDMA profile, IRQ,
  외부 TDC-GPX Register readback
- `lidar_v2_xilinx_vdma`: XAxiVdma S2MM, Frame Store 소유권, cache flush/invalidate
- `lidar_v2_example_app`: CSR 설정부터 DDR/PACKED17/UDP까지 통합
- `lidar_v2_main_example`: Zynq-7000 GIC, 주소, DDR 영역을 연결하는 보드 골격
- `lidar_v2_posix_udp`: PetaLinux 사용자 공간 UDP sink

IP AXIS 출력 폭과 VDMA `S_AXIS_S2MM` 폭은 동일해야 한다. 현재 Parent는 32-bit
AXIS이다. VDMA의 DDR Master와 PS7 HP 포트가 64-bit인 것은 별도 AXI memory
계약이며 전용 AXI4→AXI3 converter가 처리한다.

## 5. IRQ 보수적 처리

통합 IP `o_irq`는 Zynq GIC의 level-high 입력에 연결할 수 있다. 그러나 IRQ가
어느 Shot에서 발생했는지는 IRQ bit 하나만으로 확정할 수 없으므로, ISR 또는
처리 task는 원인별 STAT와 누적 진단 카운터를 먼저 읽어야 한다.

`lidar_v2_main_example.c`는 다음 계약을 사용한다.

1. `last_irq_flags`에 pending 원인을 보존한다.
2. 보드 함수 `lidar_board_handle_irq_sources()`가 원인별 진단과 복구를 수행한다.
3. 보드 함수가 반환한, 실제 처리가 끝난 bit만 W1C한다.
4. 처리하지 않은 bit와 새로 발생한 다른 원인은 sticky 상태로 남긴다.

따라서 예제는 진단 정보를 읽기 전에 IRQ를 자동으로 지우지 않는다.

## 6. 자동 검증

| 검증 | 결과 |
|---|---|
| GPX coordinator 150/200 MHz 기능·구현 | PASS |
| Acquisition subsystem 150/200 및 200/150 MHz | PASS |
| B5-B8 PACKED17 subsystem 두 비동기 profile | PASS |
| 최신 IP package 32/64/128-bit OOC matrix | PASS |
| 4-chip Parent synth/place/route/bitstream | PASS |
| GPX IOB 364개 count/IOB/LOC/BEL | PASS |
| PS 가짜-MMIO CSR host test | PASS |
| Cortex-A9 공통 제어 코드 | PASS |
| Cortex-A9 XAxiVdma/lwIP/app/main BSP compile | PASS |

PS 회귀 세션은 `260809_final_iob_ps_v2_v2_ps_control_example`이며, Parent GUI
프로젝트는 다음 위치에 있다.

```text
C:\Projects\my_sp\ALINX\Logic\project_4_lidar_v2_l0\project_4_lidar_v2_l0.xpr
```

## 7. 보드 Release 전 남은 항목

1. 40 MHz TDC-GPX 기준 clock과 Reg7.MTimer readback
2. 150 MHz부터 시작한 외부 GPX Register read/write 및 IFIFO Drain 실측
3. 4 chips × 28-bit 데이터 버스의 PCB setup/hold와 온도·전압 장시간 시험
4. `레이저 목표 왕복시간 (2R/c, TARGET_RANGE_WINDOW_5NS)`과 실제 MTimer IRQ 비교
5. `Fire 명령 후 fire_done 수신 최대 대기시간`과 실제 laser feedback 분포 확인
6. VDMA Frame Store overwrite, cache ownership 및 DDR byte golden 비교
7. Viewer UDP 지속 처리량과 packet loss 측정
8. 실제 레이저 안전 인터록과 비정상 정지 복구

이 항목이 끝나기 전에는 물리 보드 Release Sign-off로 확대해서 해석하지 않는다.