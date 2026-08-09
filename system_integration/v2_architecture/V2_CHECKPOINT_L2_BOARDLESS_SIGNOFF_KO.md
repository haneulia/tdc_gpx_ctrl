# V2 Checkpoint L2 보드리스 Sign-off

## 1. 최종 판정

물리 PCB를 연결하지 않고 수행할 수 있는 `tdc_gpx_lidar_ctrl_v2:2.0` 검증은
**PASS**다. 이 판정은 RTL 기능, CDC, 32/64/128-bit AXIS, PACKED17 DDR 계약,
IP-XACT/XGUI, Zynq PS 참조 코드와 4-chip Parent 비트스트림 생성까지 포함한다.

이 결과를 실제 보드 Release Sign-off로 확대해서 해석하면 안 된다. TDC-GPX,
레이저, DDR 및 Ethernet의 전기적·지속 운용 특성은 물리 보드에서 별도로 닫아야
한다.

## 2. 이번 점검에서 보완한 항목

### 2.1 목표 왕복시간 종료 경로 파이프라인

`Processing 200 MHz / TDC 150 MHz / 64-bit` OOC 구현에서 모든 timing
constraint는 만족했지만, 내부 Sign-off 여유 기준 `+0.100 ns`보다 작은
`WNS +0.084 ns`가 검출됐다.

최악 경로는 `레이저 목표 왕복시간 (2R/c, TARGET_RANGE_WINDOW_5NS)`의 32-bit
카운터 종료 비교가 `stop_tdc` 펄스 폭 카운터까지 모듈 경계를 넘어 이어진
경로였다. 논리 4단보다 배선 지연 비중이 약 78%로 더 컸다.

`laser_executor_core`에 `range_last_r`를 추가해 종료 한 clock 전에 마지막 count
상태를 등록했다.

```text
수정 전
range_count_r[31:0] 비교 -> 모듈 경계 -> stop_tdc pulse counter enable

수정 후
range_count_r[31:0] 비교 -> range_last_r
range_last_r              -> stop_tdc pulse counter enable
```

`range_last_r`는 count가 2일 때 다음 마지막 count를 예고한다. 따라서 외부에서
관측되는 `stop_tdc` 발생 clock은 바뀌지 않고 긴 조합·배선 경로만 끊어진다.
Release 기능 회귀가 기존 `start_tdc`-to-`stop_tdc` clock 수를 다시 확인했다.

동일 구현 조건의 WNS는 `+0.084 ns`에서 `+0.146 ns`로 개선됐고, 기존 경로는
최악 경로 목록에서 제거됐다. 레이저 피드백 `fire_done`을 승인해
`측정 시작 기준시점 (T0)`과 `start_tdc`를 만드는 저지연 경로는 변경하지 않았다.

### 2.2 OOC 구현과 실제 Parent IOB 책임 분리

xc7z020의 PL pin은 200개다. 4-chip 외부 TDC-GPX 버스와 32쌍 Echo LVDS 입력을
동시에 공개 port로 둔 OOC fixture는 실제 Parent와 다른 구성이고 pin 수를
초과한다. K0-6 OOC 구현은 실제 4-chip Parent와 동일하게 Echo Receiver를
비활성화했다.

OOC 검증은 통합 Top 내부 timing/CDC를 소유하므로 GPX port의 `IOB=TRUE` 속성만
일시 완화한다. 대신 실제 XDC를 사용하는 4-chip Parent Sign-off가 364개 GPX
경계 레지스터의 `IOB`, `LOC`, `BEL`을 모두 검사한다. 즉 IOB 검증을 제거한 것이
아니라 실제 pin 위치를 아는 계층으로 이동한 것이다.

### 2.3 테스트벤치 유지보수 계약

`tb_lidar_vdma_profile_cdc`에 한글 목적, 핵심 계약, 실행 회귀, 관련 RTL 및
payload 변경 시 유지보수 주의를 명시했다. 테스트벤치 문서 검사 결과는 다음과
같다.

```text
files=72, primary=48, profiles=11, harnesses=13, PASS
```

## 3. 전체 RTL/IP 회귀

최종 전체 회귀:

```text
signoff_results/sessions/260809_preboard_final_v5_v2_k14_signoff
input snapshot SHA-256:
EB4A06E8811E5CD71A489812723FF6F824547957F5FF6C549C5A86BC8D846CE0
```

과거 PASS 표식을 재사용하지 않고 11개 Gate를 모두 다시 실행했다.

| Gate | 결과 | 주요 범위 |
|---|---|---|
| Testbench 문서 | PASS | 72개 자산의 목적·유지보수 설명 |
| GPX Register 유지보수 | PASS | Read/Write 소유권, 요청 보존, clear status |
| Release Top 기능 | PASS | 설정, 레이저 lifecycle, GPX, PACKED17 연결 |
| Top 구현 | PASS | 2개 비동기 clock 방향 x 3개 AXIS 폭 |
| GPX 획득 coordinator | PASS | 4-chip Shot, IFIFO Drain, 응답 backpressure |
| Release CDC | PASS | 설정, command, event, status, VDMA profile 원자성 |
| AXIS 폭 | PASS | 32/64/128-bit 동일 PACKED17 의미 |
| 상태/IRQ | PASS | Sticky, W1C, 원인 보존, 단일 소유권 |
| RTL-HTML 운용행렬 | PASS | Return, 거리, topology, clock 조합 |
| DDR/HTML/PS/Ethernet | PASS | Word 및 Viewer packet byte 비교 |
| IP package/XGUI/OOC | PASS | IP-XACT, XGUI, source sync, 3개 package profile |

### 3.1 OOC 배치·배선 결과

모든 profile은 latch 0, black box 0, Critical CDC 0, 예상 밖 blocking DRC 0이다.

| Processing/TDC | AXIS 폭 | Route WNS |
|---|---:|---:|
| 150/200 MHz | 32 bit | `+0.132 ns` |
| 150/200 MHz | 64 bit | `+0.354 ns` |
| 150/200 MHz | 128 bit | `+0.216 ns` |
| 200/150 MHz | 32 bit | `+0.157 ns` |
| 200/150 MHz | 64 bit | `+0.146 ns` |
| 200/150 MHz | 128 bit | `+0.252 ns` |

### 3.2 OEN 모드

| OEN 모드 | TDC clock | 기능·구현 | WNS |
|---|---:|---|---:|
| `DYNAMIC_CONNECTED` | 200 MHz | PASS | `+1.638 ns` |
| `PULLUP_OR_NOT_CONNECTED` | 150 MHz | PASS | `+2.903 ns` |

동적 OEN은 CSN/RDN/WRN과 같은 TDC clock FSM에 속하며 `IOB=TRUE`가 적용된다.
현재 VT Parent는 `PULLUP_OR_NOT_CONNECTED`를 사용하므로 OEN 외부 pin은 만들지
않는다.

### 3.3 IP package와 PS 참조 코드

최신 package 세션은 `260810_043753_k010_ip_package`다.

- self-contained source: RTL 88개 + XGUI + 한글 가이드 3개
- OOC: async 32-bit 150/200, async 128-bit 200/150 Echo off,
  sync 64-bit 150/150 모두 PASS
- 새 warning ID 또는 기존 warning 증가 시 실패하는 warning contract PASS

PS 참조 회귀 `260809_preboard_ps_final_v2_v2_ps_control_example`도 PASS다.

- CSR ABI 2.7 host MMIO 시험
- Cortex-A9 portable 제어 코드
- XAxiVdma adapter
- lwIP Viewer UDP 전송 경로
- Zynq-7000 BSP board main 골격

## 4. 최신 4-chip Parent 구현

GUI 프로젝트:

```text
C:\Projects\my_sp\ALINX\Logic\project_4_lidar_v2_l0\project_4_lidar_v2_l0.xpr
```

최신 IP package로 프로젝트를 재생성한 뒤 합성부터 bitstream까지 다시 실행했다.
세션은 `260809_preboard_parent_impl_commit_v1`이며, 자동 manifest는 검증 소스를
커밋 `4fb97303828462d14fe23a52e65a27cb768ff33a`으로 고정한다.

| 항목 | 결과 |
|---|---:|
| FPGA | `xc7z020clg484-2` |
| PL 서비스 pin | `171 / 171` |
| GPX IOB register | `364 / 364` |
| Synthesis WNS/WHS | `+0.357 / +0.036 ns` |
| Route WNS/WHS | `+0.218 / +0.010 ns` |
| Active Critical CDC | `0` |
| Bus-skew 위반 | `0` |
| Blocking DRC | `0` |
| Bitstream | `4,045,708 bytes` |

Parent의 AXIS와 VDMA `S_AXIS_S2MM`은 32-bit로 일치한다. VDMA의 DDR Master와
PS7 HP0/HP1은 64-bit이며 Rise/Fall별 AXI4-to-AXI3 converter가 이 별도 memory
interface 계약을 처리한다.

## 5. 물리 보드에서만 가능한 잔여 검증

다음 항목은 이번 보드리스 PASS에 포함되지 않는다.

1. 40 MHz TDC-GPX 기준 clock의 실제 정확도와 `Reg7.MTimer` readback
2. 외부 TDC-GPX Register Read/Write 및 IFIFO Drain의 실측 bus timing
3. 4 chips x 28-bit 데이터, ADR/CSN/RDN/WRN/OEN의 PCB setup/hold 및 신호 무결성
4. 전압·온도·배선 편차를 포함한 150/200 MHz 장시간 동작
5. 물리 `fire_done` 승인부터 `측정 시작 기준시점 (T0)`과 `start_tdc`까지의 지연
6. 실제 레이저 안전 interlock, timeout 및 비정상 정지 복구
7. VDMA Frame Store overwrite, Zynq cache ownership 및 HP port 지속 처리량
8. Viewer UDP/Ethernet의 장시간 처리량, packet loss와 복구
9. Echo Receiver를 활성화하는 별도 PCB의 LVDS pin, IBUFDS 및 채널 지연 실측
10. 동적 OEN 또는 LF/ERRFLAG를 배선하는 PCB의 XDC/IOB와 진단 정책

## 6. Sign-off 경계

현재 상태는 다음처럼 표현한다.

```text
RTL/IP/Parent 보드리스 Sign-off : PASS
물리 보드 Release Sign-off      : PENDING
```

다음 작업은 위 물리 항목 중 실제 보드 구성과 시험 우선순위가 확정된 뒤 시작한다.
