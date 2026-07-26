# TDC-GPX 구조 정리 및 최적화 검토

## 이번 단계에서 완료한 정리

1. Pipeline/Chip CSR의 generated `.xci` wrapper를 제거하고 `my_axil_csr`/`my_axil_csr32_top`을 source-level로 직접 instantiation했다.
2. standalone OOC, standalone XPR, parent reference, system integration smoke가 동일한 CSR source manifest를 사용하도록 맞췄다.
3. `tdc_gpx_top` 공개 entity의 generic default/range와 port 폭을 Vivado IPI가 해석 가능한 단순 표현으로 바꿨다. 내부 구현은 계속 package 상수와 함수를 사용한다.
4. chip 수에 따라 실제 물리 핀 폭이 IP-XACT에서 변하도록 확인했다.
5. XGUI에 topology, clock, counter-width 관계 validation을 추가했다.
6. 기본 통합 regression clock을 AXIS 150 MHz / TDC 200 MHz로 맞췄다.
7. standalone XPR의 오래된 sibling RTL과 generated CSR XCI 참조를 제거하고,
   공용 integration source manifest로 등록 목록을 단일화했다.

## 검증 결과

| 검증 | 결과 |
|---|---|
| Source RTL regression | 32/64/128-bit, topology, backpressure, CSR sticky PASS |
| Source OOC implementation | `xc7z020clg484-2`, WNS `+0.260 ns`, black box 0 |
| IP-XACT/XGUI static check | interface, clock, topology, variable pin width PASS |
| Packaged-IP OOC | 32-bit, 4 chip, AXIS/TDC 150/200 MHz PASS |
| Internal encoder full chain | 2 start/stop/result and rise/fall lines, abort 0 |
| Physical encoder full chain | 12 fire/start/stop/result and rise/fall lines, abort 0 |
| Existing XPR source audit | missing/legacy reference 0, canonical source 64개 등록 |
| PS FCLK parent reference | 22 generic parity, 95 block-design contract checks PASS |

통합 TB에서 최신 Laser `CTL1[31:16]`의 `FIRE_DONE_TIMEOUT`을 0으로
남겨 발사가 차단되던 오래된 설정도 발견해 수정했다. timeout은 5 ns tick이며
해당 통합 profile에서는 target round-trip 이하로 설정한다.

Parent reference의 `VALIDATE`는 연결 계약 확인용이다. 실제 parent를 만들 때는
Vivado 2025.2가 권고하는 inline `ilconcat` 전환, PS 양쪽 SmartConnect의 명시적
AXI 속성, board pin XDC를 별도 system sign-off 항목으로 닫아야 한다.

## 유지하는 것이 합리적인 구조

### 두 CSR bank

하나로 합치면 주소와 software ABI가 크게 바뀌고 chip control과 pipeline control의 CDC 소유권이 섞인다. 현재는 두 AXI-Lite interface를 유지하는 편이 안전하다. 상위 PS에서 연속 주소로 보이게 하는 것이 필요하면 Address Editor에서 base만 인접 배치한다.

### 네 logical chip slot

물리 핀은 1~4 chip으로 줄지만 status/header/CSR의 logical slot은 네 개를 유지한다. sparse present mask와 기존 software decode를 보존하기 위한 의도적인 고정 ABI이다.

### Rise/Fall compile-time mask

fall mask가 0이면 fall builder와 output chain이 합성에서 제거된다. runtime enable만 사용하는 구조보다 자원과 불필요한 상태를 줄인다. 양 edge가 필요한 chip은 두 mask를 겹치면 된다.

## 다음 최적화 후보

우선순위 1:

- `g_OEN_MODE`는 현재 허용값이 `DYNAMIC_CONNECTED` 하나뿐이다. 외부 호환성이 필요 없으면 generic을 제거하고 상수 정책으로 단순화할 수 있다.
- public entity literal default와 `tdc_gpx_pkg` default가 이중 기록된다. IP Packager 제한 때문에 직접 package 상수를 참조할 수 없으므로, 두 값의 일치를 검사하는 CI script를 추가하는 방식이 안전하다.
- Pipeline `o_irq_pipe`는 source가 reserved이다. 실제 pipeline interrupt source를 정의하거나 port를 deprecated 처리해 의미를 분명히 해야 한다.

우선순위 2:

- `g_STREAM_CLK_MODE=SYNC`의 실제 사용 계획이 없다면 ASYNC 전용으로 축소할 수 있다. 단, 동일-clock 경량 build 수요가 있다면 유지한다.
- `tdc_gpx_top` 내부의 geometry helper와 package geometry 함수 중 중복 산식을 다시 비교하고, VDMA 계약 함수 한 곳만 authoritative하게 유지한다.
- historical/soft-clear/per-shot sticky를 type 또는 field group으로 분리하면 status clear 정책 검토가 쉬워진다.

우선순위 3:

- 장기 backpressure 정책은 데이터 보존은 검증됐지만 시스템 timeout/recovery 정책까지 포함해 별도 운용 요구사항으로 확정할 필요가 있다.
- 128-bit output의 실제 DDR/VDMA 효율은 RTL beat 수뿐 아니라 parent interconnect burst, HP port, DDR arbitration 측정으로 최종 판단한다.

## 최적화 원칙

- software-visible address와 packet format은 검증 없이 바꾸지 않는다.
- clock 수 대신 물리 시간 generic을 우선 사용한다.
- runtime에서 필요한 거리/scan 값만 5 ns CSR tick으로 유지한다.
- compile-time topology는 generic/generate로 제거하고, runtime mux를 불필요하게 남기지 않는다.
- 최적화 전후에 150/200 MHz 통합 regression과 packaged-IP OOC를 모두 통과시킨다.
