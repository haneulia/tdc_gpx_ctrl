# V3 H1 GPX Raw28-to-Hit17 체크포인트 결과

> 현재 ABI 3.2의 전체 H0~H4 Header, Bit Map과 재검증 수치는
> [`V3_H0_H4_HEADER_CONTRACT_KO.md`](V3_H0_H4_HEADER_CONTRACT_KO.md)를 기준으로 한다.
> 이 문서의 수치는 H1 최초 체크포인트 이력으로 유지한다.

## 1. 판정

**H1 PASS**다. V3 HLS `gpx_hit_decoder_hls`는 외부 TDC-GPX I-Mode에서 읽은
28-bit word를 V2 RTL과 같은 Hit17 이벤트로 변환하며, 네 가지 Chip/STOP/Rise/Fall
구성에서 기능·오류·backpressure·abort 계약을 만족했다.

이 판정의 범위는 **Raw28-to-Hit17 경계 하나**다. 아직 구현하지 않은 Hit-to-Cell,
Cell-to-Frame, PACKED17/Shot/Hole/Footer, 혼합 Top과 Parent 프로젝트는 H2~H6에서
각각 검증한다. 따라서 H1 PASS를 V3 통합 IP 전체 Sign-off로 사용하면 안 된다.

## 2. 구현 경계

```text
TDC-GPX bus/IFIFO Drain RTL
        |
        | gpx_raw_event_t (V2 semantic 215 bit)
        v
RTL AXI adapter: reserved bit를 포함한 216-bit AXI4-Stream
        |
        v
gpx_hit_decoder_hls: Raw28 해석, Chip/STOP/slope 검사, Hit17 생성
        |
        | 224-bit AXI4-Stream result
        v
RTL AXI adapter: gpx_hit_event_t + fault pulse/sticky
```

HLS는 Processing clock 한 도메인에서만 동작한다. TDC-GPX bus PHY, IFIFO Drain,
Clock Domain Crossing (CDC), XPM FIFO, Reset 동기화는 계속 RTL이 소유한다.

## 3. 고정 Bit 계약

### 3.1 Raw 입력

| Bit | 의미 |
|---|---|
| `[1:0]` | 이벤트 종류: DATA/IFIFO_DONE/CHIP_DONE/SHOT_DONE |
| `[3:2]` | TDC-GPX Chip index, 0~3 |
| `[4]` | IFIFO index, 0 또는 1 |
| `[32:5]` | TDC-GPX I-Mode Raw word 28 bit |
| `[33]` | 앞 단계에서 검출한 fault |
| `[36:34]` | timeout 원인 |
| `[198:37]` | Shot context: Face, Shot, 측정 시작 기준시점 (T0) 등 |
| `[214:199]` | Chip별 Shot sequence |
| `[215]` | AXI 경계 reserved, 항상 0 |

Raw word 안에서 `channel_code=[27:26]`, `start_number=[25:18]`,
`slope=[17]`, `hit_value=[16:0]`로 해석한다. 외부 TDC-GPX가 제공하는 거리 원본은
`hit_value[16:0]`이며 HLS가 임의로 확장하거나 축소하지 않는다.

### 3.2 Hit 출력

| Bit | 의미 |
|---|---|
| `[1:0]` | 이벤트 종류 |
| `[3:2]` | Chip index |
| `[4]` | IFIFO index |
| `[6:5]` | Raw channel code 0~3 |
| `[9:7]` | STOP index 0~7: `channel_code + IFIFO×4` |
| `[17:10]` | START number |
| `[18]` | slope |
| `[35:19]` | Hit17 원본값 |
| `[36]` | 앞 단계 fault |
| `[39:37]` | timeout 원인 |
| `[201:40]` | Shot context 그대로 전달 |
| `[217:202]` | Chip별 Shot sequence |

결과 word의 `[218]`은 Hit 이벤트 출력 여부, `[219]`은 Chip index 오류,
`[220]`은 STOP index 오류, `[221]`은 slope 역할 오류다. AXI 정렬용 `[223:222]`는
항상 0이다.

## 4. 검증 구성

| Profile | Chip 수 | STOP/Chip | Rise mask | Fall mask | 목적 |
|---|---:|---:|---:|---:|---|
| dedicated | 4 | 8 | `0011` | `1100` | 2-Chip Rise + 2-Chip Fall |
| one_chip_dual | 1 | 8 | `0001` | `0001` | 한 Chip에서 Rise/Fall 동시 검출 |
| reduced | 3 | 6 | `0011` | `0100` | 3-Chip 및 감소된 STOP 범위 |
| all_dual | 4 | 8 | `1111` | `1111` | 네 Chip 모두 Rise/Fall 동시 검출 |

검사 항목은 다음과 같다.

- I-Mode 모든 field와 Hit17 최상위 bit `Hit[16]`의 정확한 전달
- DATA 외 제어 이벤트의 context와 sequence 보존
- 비활성 Chip, 범위 밖 STOP, 허용되지 않은 slope의 drop 및 fault 분류
- 출력 backpressure 중 전체 payload 고정
- abort에서 보류 이벤트 제거 후 다음 이벤트 정상 수신
- 긴 idle 뒤 한 개만 도착하는 희소 이벤트의 정상 배출
- 네 Profile 각각 150 MHz와 200 MHz에서 V2 RTL과 V3 HLS의 완전 일치

## 5. 결과

### 5.1 C simulation과 C/RTL co-simulation

- C simulation: 네 Profile의 256개 조합과 최대 Hit17 별도 경계값, 총 1,025개 PASS
- C synthesis: 목표 5.000 ns, 추정 4.365 ns, latency 2 clocks, II=1
- HLS 추정 자원: 321 LUT, 411 FF, BRAM 0, DSP 0
- 네 Profile C/RTL co-simulation: 모두 PASS
- AXI wrapper 포함 co-sim latency: 최소/평균/최대 3/3/4 clocks
- AXI wrapper 포함 co-sim interval: 최소/평균/최대 1/1/2 clocks

### 5.2 V2 RTL 차동 회귀

네 Profile을 150 MHz와 200 MHz에서 실행한 총 8개 시나리오가 모두 PASS했다.
V2 `lidar_gpx_hit_decoder`와 V3 HLS adapter의 Hit event 전체 record, fault pulse,
fault sticky를 Clock 단위로 비교했다.

V2 기준선 자체의 150/200 MHz 여섯 Profile 회귀도 모두 PASS하여 비교 기준이
깨지지 않았음을 확인했다.

### 5.3 xc7z020clg484-2 실제 OOC 배치·배선

| Processing clock | WNS | Latch | 차단 DRC |
|---:|---:|---:|---:|
| 150 MHz | +1.952 ns | 0 | 0 |
| 200 MHz | +0.768 ns | 0 | 0 |

200 MHz Routed 결과는 HLS adapter 전체 기준 582 LUT, 1,102 FF, BRAM 0, DSP 0이다.
이 수치는 HLS 알고리즘 추정치와 달리 216-bit 입력과 224-bit 출력 AXI register
slice를 포함한다.

200 MHz 최악 경로는 HLS 출력 register slice의 FSM state register에서 payload
register까지이며, data path 4.197 ns 중 logic 0.538 ns, route 3.659 ns다. LUT는
1단계이고 배선 비중은 87.2%다. 현재 WNS는 양수지만 H5/H6에서 Parent 배치 영향과
주변 경로를 다시 확인해야 한다.

## 6. 구현 중 발견하고 닫은 문제

처음 사용한 일반 pipeline은 입력이 빈 뒤 마지막 한 이벤트가 HLS 입력 register
slice에 남아 다음 입력을 기다렸다. TDC-GPX 이벤트는 연속 스트림이 아니라 긴 idle
사이에 하나만 올 수 있으므로 실제 데이터 손실로 이어질 수 있는 문제였다.

`PIPELINE II=1 style=flp`로 변경해 **희소 입력의 마지막 이벤트를 다음 입력 없이
배출**하면서 연속 입력은 Clock당 한 이벤트로 처리하도록 했다. C/RTL co-simulation과
V2 차동 테스트에 단일 희소 이벤트 검사를 포함하여 재발을 막았다.

## 7. 남은 경고와 한계

- `rise_mask[7:4]`, `fall_mask[7:4]` 미사용 합성 경고는 최대 4-Chip 계약 때문에
  의도된 것이다. C++ 표준 정수형을 우선 사용하기 위해 8-bit scalar로 전달하고
  하위 4 bit만 해석한다.
- OOC 설계라 일부 연결성 DRC를 실행할 수 없다는 Vivado 경고가 있다. H6 Parent
  검증에서 전체 연결 DRC를 다시 실행한다.
- 이번 배치·배선은 HLS decoder adapter 단독 결과다. V3 전체의 32/64-bit VDMA,
  CDC, Reset, Laser/측정 시작 기준시점 (T0) 계약을 아직 증명하지 않는다.

## 8. 재현 방법

```powershell
./system_integration/v3/scripts/run_v3_hls_hit_decoder.ps1 -Step all
./system_integration/v3/scripts/run_v3_gpx_hit_decoder_diff.ps1
./system_integration/v3/scripts/run_v3_gpx_hit_decoder_impl.ps1 -SkipHlsSynthesis
```

생성 프로젝트와 상세 로그는 저장소 루트 `.work/` 아래에만 생성하며 Git에는 넣지
않는다. 추적 대상은 소스, 테스트, 실행기와 이 판정 문서다.

## 9. 다음 단계

H2에서는 Hit17 이벤트를 Cell에 모으는 로직만 HLS로 옮긴다. 물리 IFIFO는
Runtime 직렬화(전시) Return 슬롯 수와 무관하게 EF 완료까지 모두 Drain하고,
H2는 Return 1~7 보존, Runtime 슬롯 초과분의 의도적 필터, 8번째 이상
`return_overflow`,
timeout과 abort를 V2 Golden 결과와 비교한다.
