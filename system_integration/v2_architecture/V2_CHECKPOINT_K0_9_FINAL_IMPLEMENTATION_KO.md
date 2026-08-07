# V2 Checkpoint K0-9 최종 구현 및 Golden 비교 결과

## 1. 판정

Checkpoint K0-9는 **L1 완료**로 판정한다.

- Processing/TDC `150/200 MHz`, `200/150 MHz`;
- AXIS 출력 폭 `32/64/128-bit`;
- 위 6개 조합에서 기능 회귀와 구현을 모두 수행;
- black box 0, latch 0, critical CDC 0;
- 예기치 않은 blocking DRC 0;
- 최저 WNS `+0.103 ns`로 K0-9 기준 `+0.100 ns` 이상 충족;
- DDR 캡처와 HTML Golden Vector의 모든 할당 Word exact compare PASS;
- PS cache 소유권 모델 이후 H-Line/Ethernet byte exact compare PASS.

이 판정은 독립 IP의 RTL/구현 및 실행 가능한 Golden 모델 범위다. 실제
Zynq-7000 parent의 AXI VDMA, HP port, DDR, Cortex-A9 cache API, Ethernet
MAC/PHY와 보드 핀은 Stage L0에서 별도로 닫아야 한다.

## 2. K0-9 타이밍 보완

| 경로 | 변경 | 기능 영향 |
|---|---|---|
| Frame-lane assembler | LUTRAM payload prefetch를 상태 decode에서 분리하고 Face-close를 capture/geometry/apply로 순차화 | Face-close 처리 `+2 proc clk`; Cell 수락 II와 AXIS 데이터 형식은 불변 |
| Echo diagnostics | 32채널 event와 Shot 결과를 진단 경로에서 1단 등록하고 snapshot CE fanout 제한 | 진단 snapshot만 `+1 proc clk`; `IBUFDS -> o_tdc_stop` 직접 경로는 불변 |
| Cell serializer | zero-bubble Cell accept를 명시적 단일 조건으로 만들고 fanout 제한 | Cell II=1 및 Word 순서 불변 |
| Face-close owner | B1이 이미 보장한 version/source/mask 비교를 실시간 CE 경로에서 제거 | 위반 검출 assertion 유지; 정상 기능과 지연 불변 |
| Processing status | 네 idle 관측값을 status 응답 전 1단 등록 | status 관측만 `+1 proc clk`; 제어에는 사용하지 않음 |
| Laser executor | request와 version/mode 적합 판정을 같은 cycle에 함께 등록 | fire/request latency 불변; wide compare가 lifecycle CE를 구동하지 않음 |

물리 Echo STOP은 이번 최적화의 예외다. 초저지연 요구 때문에 입력 버퍼에서
TDC STOP 출력까지 순차단을 추가하지 않았고, 최종 구현에서
`STOP_SEQUENTIAL_FANIN_COUNT=0`을 다시 확인했다.

## 3. 최종 기능 회귀

세션:

`signoff_results/sessions/260806_k09_final_functional_matrix_v2_k06_axis_integration`

| Processing/TDC | AXIS 폭 | 결과 |
|---|---:|---|
| 150/200 MHz | 32 bit | PASS |
| 150/200 MHz | 64 bit | PASS |
| 150/200 MHz | 128 bit | PASS |
| 200/150 MHz | 32 bit | PASS |
| 200/150 MHz | 64 bit | PASS |
| 200/150 MHz | 128 bit | PASS |

각 profile은 Footer 구간에 13-clock backpressure를 주입했고 Shot/Hole/T0,
Rise/Fall lane, Footer 완료와 출력 Word 순서를 보존했다.

## 4. 최종 구현 행렬

| Processing/TDC | AXIS 폭 | WNS | Latch | Black box | ASYNC_REG | Critical CDC | 예상 밖 blocking DRC |
|---|---:|---:|---:|---:|---:|---:|---:|
| 150/200 MHz | 32 bit | +0.108 ns | 0 | 0 | 498 | 0 | 0 |
| 200/150 MHz | 32 bit | +0.133 ns | 0 | 0 | 498 | 0 | 0 |
| 150/200 MHz | 64 bit | +0.180 ns | 0 | 0 | 498 | 0 | 0 |
| 200/150 MHz | 64 bit | +0.245 ns | 0 | 0 | 498 | 0 | 0 |
| 150/200 MHz | 128 bit | +0.114 ns | 0 | 0 | 498 | 0 | 0 |
| 200/150 MHz | 128 bit | +0.103 ns | 0 | 0 | 498 | 0 | 0 |

구현 세션:

- `260806_k09_final_w32_p150_v2_k06_top_implementation`;
- `260806_k09_laser_context_w32_p200_v2_k06_top_implementation`;
- `260806_k09_final_w64_v2_k06_top_implementation`;
- `260806_k09_final_w128_v2_k06_top_implementation`.

네 세션의 89개 production source manifest는 SHA-256 기준 차이가 0개다.
따라서 표의 6개 결과는 같은 최종 RTL에서 얻은 결과다.

각 행의 DRC에는 `IOSTDTYPE-1`과 `UCIO-1` 두 분류가 남아 있다. 이는 독립
Top 구현에서 LVDS 입력 32쌍의 parent `IBUFDS` 래퍼와 LOC XDC를 제외했기
때문이다. K0-9에서는 예상된 parent-XDC 제외 항목으로 분류했지만 bitstream
Sign-off에서는 허용하지 않는다.

## 5. DDR 캡처 대 HTML Golden Vector

세션:

`signoff_results/sessions/260806_k09_ddr_ps_ethernet_signoff_j9_v2_gpx_ddr_golden`

비교 단위는 **할당된 모든 DDR 주소의 32-bit Word**다. 시나리오는 실제 Shot
1개, Hole Shot 1개, 정렬된 Face Footer이며 HSIZE 밖 reserve 영역도 초기값
`0xA5A5A5A5`가 유지되는지 함께 검사했다.

| Proc MHz | 폭 | 비교 Word/Byte | Reserve Word | HSIZE | VSIZE | STRIDE | 결과 |
|---:|---:|---:|---:|---:|---:|---:|---|
| 150 | 32 | 36 / 144 B | 12 | 24 B | 4 | 36 B | PASS |
| 150 | 64 | 40 / 160 B | 16 | 24 B | 4 | 40 B | PASS |
| 150 | 128 | 36 / 144 B | 12 | 32 B | 3 | 48 B | PASS |
| 200 | 32 | 36 / 144 B | 12 | 24 B | 4 | 36 B | PASS |
| 200 | 64 | 40 / 160 B | 16 | 24 B | 4 | 40 B | PASS |
| 200 | 128 | 36 / 144 B | 12 | 32 B | 3 | 48 B | PASS |

따라서 출력 폭별 padding/line geometry 차이를 포함해 RTL DDR 이미지와 HTML
계산 모델이 Word 단위로 일치한다.

## 6. PS Cache 소유권 및 H-Line/Ethernet 비교

세션:

`signoff_results/sessions/260806_k09_ddr_ps_ethernet_signoff_v2_gpx_ps_hline`

검증 모델은 다음 순서를 강제한다.

1. DMA-owned buffer decode 거부;
2. cache 동기화 완료 후 CPU-owned 전환;
3. C reference decoder로 DDR Cell/Metadata 해석;
4. Face Header와 H-Line packet 생성;
5. HTML Golden packet과 byte exact compare;
6. DMA에 release한 buffer의 재해석 거부.

| Proc MHz | 폭 | Packet | Application payload | 결과 |
|---:|---:|---|---:|---|
| 150 | 32/64/128 | 1440 B Face Header + 38 B H-Line | 1478 B | 폭 간 동일, PASS |
| 200 | 32/64/128 | 1440 B Face Header + 38 B H-Line | 1478 B | 폭 간 동일, PASS |

동일 Processing clock에서는 32/64/128-bit DDR 형식이 모두 같은 Viewer wire
packet으로 정규화됐다. C decoder는 host 실행과 Cortex-A9 object compile을
통과했으며 Cortex-A9 object SHA-256은
`EE5578DE8F33BEFAA4AFF45A47164D01677503CB6D4B9B9C122459C043DF05F1`이다.

## 7. Sign-off 경계

| 항목 | K0-9 상태 | 최종 책임 단계 |
|---|---|---|
| RTL DDR Word 대 HTML Golden | 완료 | K0-9/L1 |
| PS 소유권 상태기계 및 H-Line/Ethernet byte Golden | 모델 완료 | K0-9/L1 |
| 실제 AXI VDMA S2MM/HP port DDR 기록 | 미검증 | L0 parent |
| FreeRTOS/PetaLinux cache invalidate API | 미검증 | L0 software/board |
| 실제 Ethernet MAC/PHY 전송 및 Viewer 수신 | 미검증 | L0 board |
| 실제 핀, LVDS IBUFDS, clock tree 포함 WNS | 미검증 | L0 parent implementation |

최저 OOC WNS가 `+0.103 ns`로 기준을 만족하지만 여유는 작다. 따라서 이 값은
보드 타이밍 보증이 아니다. parent의 실제 clock source, generated clock,
IBUFDS, LOC와 배선이 적용된 뒤 모든 profile에서 WNS를 다시 확인해야 한다.

## 8. 다음 Gate

다음 단계는 K0-10이다.

- 신규 VLNV `tdc_gpx_lidar_ctrl_v2:2.0` 생성;
- v1 패키지와 병존;
- production compile order와 XGUI parameter/port enablement 고정;
- package validation과 GUI smoke test;
- K0-9 source manifest와 package source manifest 일치 확인.

K0-10이 완료되기 전에는 K1 full RTL/HTML sweep이나 L0 parent 연결을 시작하지
않는다.
