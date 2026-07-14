# C08-S9 Repack Calculation Timeline

## 변경 목적

Repack 결과값만 보여 주던 C08-S8을 확장해, Face Ethernet payload 크기가 도출되는 모든 계산을 입력 의존성과 실제 처리 시간순으로 확인할 수 있게 했다.

대상 HTML:

`C08_HDL_HTML_Alignment_260714_Repack_Calculation_Timeline_Simulator_v009.html`

## Repack 계산 순서

| 단계 | 계산 | 기본값 결과 |
|---|---|---:|
| R01 | Ethernet application payload / MTU | 1,440 B |
| R02 | `ceil(H FoV / H resolution)` | 450 shots/Face |
| R03 | `APD x echoes x edge streams` | 224 samples/shot |
| R04 | `shots/Face x samples/shot` | 100,800 samples/Face |
| R05 | `max_range_5ns_ticks x 5 ns` | 2,005,000 ps |
| R06 | `ceil(range time / bin_resolution_ps)` | 24,754 bins |
| R07 | 16-bit 또는 17-bit 범위 판정 | 2 B/sample |
| R08 | `floor(1440 / word bytes)` | 720 samples/MTU |
| R09 | 첫 Face Header packet | 1,440 B |
| R10 | `samples/Face x word bytes` | 201,600 B |
| R11 | `ceil(distance bytes / 1440)` | 140 distance MTU |
| R12 | 마지막 거리 packet payload | 1,440 B |
| R13 | `Header + distance payload` | 203,040 B/Face |
| R14 | `1 Header + distance MTU` | 141 MTU/Face |
| R15 | `Face payload x facets` | 812,160 B/frame |
| R16 | `Face bytes / effective Ethernet` | 2.0304 ms |
| R17 | `Face rest - Repack time` | 4.2196 ms |
| R18 | 32/64/128-bit Repack 결과 비교 | 모두 203,040 B |

## 전체 표 정렬 기준

- Timing `T01 -> T08`: 기계 회전/scan window, 광학 기하, encoder, laser cadence, range/TDC, AXIS/DDR, shot closure, Face-rest Ethernet
- Frame/Data `D01 -> D05`: scan geometry, TDC sample 생성, RTL/VDMA, Repack packet, 전송률
- Repack `R01 -> R18`: MTU 계약부터 width 독립성 회귀까지 계산 의존성 순서
- Verdict `V01 -> V04`: 입력/설정, trigger/수집, Repack/전송, 최종 Frame rate
- Optical `O01 -> O04`: 정적 광학 입력, 회전/trigger, active-window 결과, 현재 ray 교차/반사
- Face 및 Encoder map: Face 0부터 N-1까지 회전 시간순

## Live table 동작

모든 표는 기본 live 상태다. 입력값을 바꾸면 정지된 canvas도 새 입력값으로 한 번 다시 계산되며, 이후 animation만 HOLD된다. 사용자가 각 표의 `표 Stop` 버튼을 누른 경우에만 해당 표가 현재 값으로 고정된다.

## 회귀 검증

- JavaScript syntax 검사 통과
- 초기 렌더에서 R01~R18 18개 행 생성 확인
- Timing T01~T08, Data D01~D05, Verdict V01~V04, Optical O01~O04 오름차순 확인
- 32/64/128-bit에서 Repack Face payload 203,040 B 동일 확인
- `max_range_5ns_ticks=1062`, 81 ps/bin에서 3 B/sample, 303,840 B/Face, 211 MTU 확인
- Endpoint 451 shots/Face에서 마지막 거리 packet 448 B, 미사용 MTU 용량 992 B, 총 142 MTU 확인

이번 단계는 HTML 검증 모델과 문서 정렬 변경이다. 실제 Ethernet Repacker HDL은 별도 구현 대상이다.
