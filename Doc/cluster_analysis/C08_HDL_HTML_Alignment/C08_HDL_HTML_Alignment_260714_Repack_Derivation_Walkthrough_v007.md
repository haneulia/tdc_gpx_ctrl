# C08-S10 Repack Derivation Walkthrough

## 변경 목적

C08-S9에서 `Distance packets`가 최종값으로는 맞지만, D02 이후 Raw VDMA 계산과 Ethernet Repack 계산이 한 흐름처럼 보여 입력 의존성을 따라가기 어려웠다. C08-S10은 두 경로를 분리하고, 거리 packet 수를 정수 몫과 나머지로 단계별 도출한다.

대상 HTML:

`C08_HDL_HTML_Alignment_260714_Repack_Derivation_Walkthrough_Simulator_v010.html`

## D01부터 D08까지의 계산 경로

| 단계 | 경로 | 이 단계에서 확정하는 값 | 다음 단계 사용 여부 |
|---|---|---|---|
| D01 | 공통 | shots/Face, shots/frame, 수직 line | D02에서 사용 |
| D02 | 공통 | TDC samples/shot, Distance samples/Face | D03 진단과 D05 Repack의 공통 원천 |
| D03 | Raw VDMA 진단 분기 | AXIS beat, row, cell, slope packet 구조 | Ethernet packet 수에는 미사용 |
| D04 | Raw VDMA 진단 분기 | Raw bytes/shot, Face, frame | Repack과 비교할 때만 사용 |
| D05 | Ethernet Repack | 거리 범위와 분해능에 따른 2 B 또는 3 B/sample | D06에서 사용 |
| D06 | Ethernet Repack | MTU당 sample, 몫, 나머지, 부분 packet, Distance packets | D07에서 사용 |
| D07 | Ethernet Repack | Face 전송 시간과 Face-rest margin | 최종 Frame rate 판단에 사용 |
| D08 | 교차 검증 | Raw/Repack 비교와 `g_OUTPUT_WIDTH` 독립성 | 진단 전용 |

D03/D04의 full-keep padding byte는 DDR 내부 저장량이다. Ethernet Repack은 D02에서 확정한 유효 거리 sample 수로 돌아가 D05부터 다시 계산한다.

## 기본값 Distance packets 도출

| 순서 | 대입식 | 결과 |
|---|---|---:|
| 1 | `16 APD x 7 echoes` | 112 echo points/shot |
| 2 | `112 points x 2 edges` | 224 distance samples/shot |
| 3 | `450 shots/Face x 224 samples/shot` | 100,800 distance samples/Face |
| 4 | `ceil(401 ticks x 5,000 ps / 81 ps/bin)` | 24,754 bins |
| 5 | `24,754 <= 65,535` | 2 B/sample |
| 6 | `floor(1,440 B / 2 B/sample)` | 720 samples/distance packet |
| 7 | `floor(100,800 / 720)` | 140 full distance packets |
| 8 | `100,800 mod 720` | 0 remainder samples |
| 9 | `remainder == 0` | 0 partial packet |
| 10 | `140 full + 0 partial` | 140 Distance packets |
| 11 | `1 Header + 140 Distance` | 141 packets/Face |
| 12 | `1,440 Header + 100,800 x 2` | 203,040 B/Face |

`Distance packets`는 첫 Header packet을 포함하지 않는다. 따라서 기본값에서 화면의 `140`은 거리 packet만의 개수이고, Face 전체 packet 수는 `141`이다.

## 경계값 확인

| 사례 | 주요 계산 | 결과 |
|---|---|---|
| Endpoint policy | 451 shots, 101,024 samples, `101,024 mod 720 = 224` | 140 full + 1 partial = 141 Distance, 142 packets/Face, 마지막 payload 448 B |
| 3-byte 전환 | `max_range_5ns_ticks=1062`, 81 ps/bin | 3 B/sample, 480 samples/packet, 210 Distance, 211 packets/Face, 303,840 B/Face |
| 출력 폭 변경 | 32/64/128-bit | Raw VDMA packet은 변하지만 Repack은 203,040 B/Face와 140 Distance packets로 동일 |
| 17-bit 초과 | required code > 131,071 | Ethernet 거리 word 계약 `CHECK` |

## HTML 표시 변경

- Data 표를 D01부터 D08까지 확장했다.
- D03/D04를 Raw VDMA 진단 분기로 표시하고 D05가 D02 결과에서 재개됨을 명시했다.
- Repack 표를 R01부터 R22까지 확장했다.
- R11부터 R14에서 full packet, remainder sample, partial packet, Distance packet 합계를 각각 표시한다.
- D06과 R11~R18에 실제 입력값을 대입한 산식을 표시한다.
- Data와 Repack 표는 한글 번역을 기본으로 표시한다.

## 회귀 검증

- JavaScript syntax 검사 통과
- 기본값 100,800 samples, 2 B/sample, 720 samples/packet, 140 Distance packets 확인
- Endpoint 451-shot에서 나머지 224 samples와 부분 packet 1개 확인
- 3-byte 경계에서 480 samples/packet과 210 Distance packets 확인
- 128-bit 출력 폭에서도 Repack Face payload와 Distance packets가 32-bit 기본값과 동일함을 확인

이번 단계는 HTML 검증 모델과 설명 구조 변경이다. 실제 Ethernet Repacker HDL 구현은 별도 개발 대상이다.
