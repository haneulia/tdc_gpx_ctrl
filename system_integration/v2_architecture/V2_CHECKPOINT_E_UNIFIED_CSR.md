# Checkpoint E: Unified CSR and Atomic Configuration Boundary

## 1. 판정

Checkpoint E 범위는 **PASS**이다.

- 32 CTL / 32 STAT / 4 IRQ ABI 구현
- W1S command와 persistent shadow 분리
- build-aware reset shadow 구현
- AXI4-Lite frontend와 atomic configuration manager 통합
- Processing/TDC active version 일치 검증
- 150/200 MHz와 200/150 MHz 비동기 profile 기능 및 route 검증

이 판정은 v2의 CSR/configuration boundary에 대한 것이다. Processing event,
Echo, GPX acquisition 및 output datapath는 이후 checkpoint 범위이다.

## 2. 구현 파일

| 파일 | 역할 |
|---|---|
| `pkg/lidar_csr_map_pkg.vhd` | 주소, field, pack/unpack의 단일 ABI source |
| `pkg/lidar_config_types_pkg.vhd` | build-aware runtime reset 기본값 |
| `rtl/csr/lidar_csr_bank.vhd` | shadow, W1S, status, IRQ, active readback |
| `rtl/csr/lidar_csr_config_subsystem.vhd` | CSR와 atomic manager/gateway 통합 |
| `tb/tb_lidar_csr_map_pkg.vhd` | profile별 default와 pack round-trip |
| `tb/tb_lidar_csr_bank.vhd` | AXI/command/status/IRQ 단위 회귀 |
| `tb/tb_lidar_csr_config_subsystem.vhd` | AXI부터 양 domain activation까지 통합 회귀 |
| `scripts/run_v2_unified_csr.ps1` | 시뮬레이션, route, CDC, archive 자동화 |

CSR32의 검증된 `axil_fsm_32.vhd`와 `axil_intr_32.vhd`를 source-level로
재사용한다. persistent CTL용 `my_axil_csr32_top` 전체를 인스턴스화하지 않은
이유는 COMMIT/RESET 같은 명령이 level register가 아닌 AXI write event여야
하기 때문이다.

## 3. 검증 항목

| Test | 결과 |
|---|---|
| 5/4/3/1 Face·chip profile별 legal reset shadow | PASS |
| source record -> CSR words -> source record 무손실 round-trip | PASS |
| AW/W simultaneous, AW-first, W-first | PASS |
| WSTRB byte merge | PASS |
| BVALID/RVALID backpressure 안정성 | PASS |
| reserved/misaligned/invalid decode 보존 및 진단 | PASS |
| COMMIT/CLEAR/SOFT_RESET one-shot | PASS |
| multi-command 차단 | PASS |
| sticky status와 IRQ W1C/event-wins-clear | PASS |
| 최초 default commit과 active version 1 | PASS |
| BUSY 중 다음 shadow write와 snapshot 격리 | PASS |
| BUSY 중 추가 COMMIT reject `0x71` | PASS |
| invalid CPR commit의 active payload 보존 | PASS |
| unsafe PROC domain에서 PREPARE 대기 | PASS |
| PROC/TDC domain 동일 version/payload activation | PASS |
| Face lower/upper inclusive readback | PASS |

## 4. 구현 결과

최종 결과 archive:

```text
signoff_results/sessions/260804150405_v2_unified_csr
```

| Profile | WNS | Latch | CDC Critical | CDC-3 | ASYNC_REG |
|---|---:|---:|---:|---:|---:|
| PROC 150 / TDC 200 MHz | +0.656 ns | 0 | 0 | 14 | 92 |
| PROC 200 / TDC 150 MHz | +0.483 ns | 0 | 0 | 14 | 92 |

XC7Z020 OOC utilization은 2,325 LUT, 10,436 FF, BRAM 0, DSP 0이다.

## 5. 검토된 경고

### CDC-15 2,128건

두 destination gateway가 CSR-domain candidate record를 PREPARE handshake로
고정한 뒤 destination clock에서 한 번에 capture하는 multi-bit mailbox이다.
Vivado는 각 payload bit를 `Clock enable controlled CDC structure`로 보고하므로
2,128건이 발생한다. control crossing 14개는 ASYNC_REG synchronizer로
인식되며 Critical CDC는 0이다.

이 경고는 무시된 것이 아니라 다음 조건으로 closure했다.

- candidate payload는 PREPARE 전 CSR register에서 안정화된다.
- destination은 동기화된 request를 확인한 뒤 payload를 capture한다.
- ACK 전까지 source payload가 변하지 않는다.
- ACTIVATE/RELEASE가 payload capture와 분리된다.
- 두 비동기 clock 방향과 reset/replay 시나리오가 Checkpoint D/E TB에서
  검증됐다.

### OOC warning

- `ZPS7-1`: XC7Z020 OOC block에 PS7이 없어서 발생하며 parent design에서
  PS7/FCLK가 추가된다.
- `HD.CLK_SRC`, `HD.PARTPIN_LOCS`: parent placement 정보가 없는 OOC 특성이다.
- 최종 parent implementation에서 clock source와 port constraint를 포함해
  재검증해야 한다.

## 6. 설계상 개선점

현재 full active record를 manager, PROC gateway, TDC gateway가 각각 보관하여
FF 사용량이 10,436개이다. XC7Z020 용량에는 여유가 있지만 v2 목표인 단순성과
resource 절감을 위해 다음 checkpoint에서 consumer가 확정되면 payload를
다음처럼 분리하는 것이 합리적이다.

1. 공통 `version`과 geometry contract
2. Processing domain 전용 Motor/Laser snapshot
3. TDC domain 전용 GPX/range snapshot

분리는 software ABI나 atomic transaction을 바꾸지 않는다. 실제 consumer가
없는 지금 너무 일찍 field를 분리하면 다시 합치는 churn이 생기므로, Processing
event pipeline 연결과 함께 수행한다.

## 7. 잔여 계약

- `SOFT_RESET_REQUEST`는 parent reset supervisor가 아직 소비하지 않는다.
- `TDC_SCAN_TIMEOUT=0`의 GPX watchdog 의미는 acquisition stage에서 확정한다.
- Echo simulation delay는 Checkpoint G에서 CTL20 한 word의
  `CH0_DELAY + channel * STEP` 구조로 확정되었다. GPX image 16-entry indexed
  portal만 Stage 5에서 CTL21..31 일부를 사용해 추가한다.
- 최종 IP-XACT/XGUI packaging과 parent PS7 implementation은 full v2 top 이후
  수행한다.
