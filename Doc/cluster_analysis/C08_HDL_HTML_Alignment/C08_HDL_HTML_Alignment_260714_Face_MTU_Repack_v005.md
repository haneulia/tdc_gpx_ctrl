# C08-S8 Face MTU Repack 정합 검토

## 1. 확정한 Ethernet application 계약

- Ethernet 기본 경로는 Raw VDMA가 아니라 Face 단위 Repack이다.
- UDP/TCP/IP header는 계산에서 제외한다.
- application payload 상한은 packet당 1440 byte다.
- Face의 첫 packet은 Header 전용이며, 여유 공간을 포함해 정확히 1440 byte로 padding한다.
- 두 번째 packet부터 Face에서 생성된 유효 TDC sample을 거리 word로 연속 전송한다.
- 마지막 거리 packet은 남은 payload만 전송하며, 사용하지 않은 MTU 용량은 Ethernet byte/time에 더하지 않는다.
- 거리 sample 수는 `shots/Face x APD x echoes x enabled edge streams`로 계산한다. Falling 검출이 켜지면 rising/falling sample을 각각 한 word로 센다.

기존 RTL의 48-byte VDMA Header 필드는 첫 1440-byte Ethernet Face Header에 포함할 수 있다. 나머지 1392 byte는 예약 영역으로 유지한다.

## 2. 2-byte / 3-byte 자동 선택

별도 CSR을 추가하지 않고 기존 값에서 파생한다.

```text
range_time_ps     = max_range_5ns_ticks x 5000
required_tdc_code = ceil(range_time_ps / bin_resolution_ps)

required_tdc_code <=  65535  -> 2 byte, raw code[15:0]
required_tdc_code <= 131071  -> 3 byte, {7'b0, raw code[16:0]}
required_tdc_code >  131071  -> CHECK (17-bit TDC 표현 범위 초과)
```

`max_range_5ns_ticks = 0`은 범위 제한 비활성이므로 bit 16이 0이라고 보장할 수 없다. 이때는 17-bit 전체를 보존하는 3 byte를 선택한다. Wire byte order는 기존 packet과 맞춘 little-endian을 권고한다.

1440 byte에는 2-byte word 720개 또는 3-byte word 480개가 정확히 들어간다.

## 3. 기본값 회귀 결과

기본 프로파일은 4 Face, 450 shots/Face, 16 APD, 7 echoes, rising/falling 검출, 300 m, 81 ps/bin, Ethernet effective 800 Mbps다.

| 항목 | 결과 |
|---|---:|
| 거리 sample / Face | 100,800 |
| `max_range_5ns_ticks` | 401 |
| 필요한 TDC code 상한 | 24,754 |
| 자동 거리 word | 2 byte |
| 거리 packet | 140 MTU |
| 총 Face packet | Header 1 + 거리 140 = 141 MTU |
| 실제 Face payload | 203,040 byte |
| Ethernet 시간 | 2.0304 ms |
| Face rest | 6.2500 ms |
| Ethernet margin | 4.2196 ms |

81 ps/bin에서 경계 회귀값은 다음과 같다.

| `max_range_5ns_ticks` | 필요한 code | 판정 |
|---:|---:|---|
| 1061 | 65,494 | 2 byte |
| 1062 | 65,556 | 3 byte |
| 2123 | 131,050 | 3 byte |
| 2124 | 131,112 | CHECK, 17-bit 초과 |

## 4. `g_OUTPUT_WIDTH` 독립성

기본 조건의 Raw VDMA Face 저장량은 32/64/128-bit에서 각각 619,200 / 734,400 / 964,800 byte다. 폭이 커질 때 full-keep padding이 늘어 Raw 값은 커질 수 있다.

Face MTU Repack은 유효 TDC sample만 전송하므로 세 폭 모두 203,040 byte, 2.0304 ms로 동일하다. 따라서 `g_OUTPUT_WIDTH`는 내부 AXIS/VDMA 처리량과 DDR 저장량에는 영향을 주지만 Ethernet Repack payload 크기에는 영향을 주지 않는다.

## 5. 권고 HDL 구조

`TDC_GPX_TOP`의 현재 VDMA packet ABI는 유지하고, DDR read/network 경계 또는 정규화된 Face stream 뒤에 `tdc_gpx_eth_repacker`를 독립 블록으로 두는 방향이 적합하다.

고정 상수:

- `c_ETH_PAYLOAD_BYTES = 1440`
- `c_TDC_RAW_HIT_BITS = 17`
- `c_TDC_SHORT_WORD_BITS = 16`

파생값:

- `distance_word_bytes`
- `samples_per_payload`
- `distance_packet_count`
- `face_payload_bytes`

첫 Face Header에는 최소한 format version, Face ID, `bin_resolution_ps`, `max_range_5ns_ticks`, distance word bytes, valid distance count, distance payload bytes, packet count, edge mode가 있어야 한다. 수신기는 이 Header만으로 뒤따르는 거리 packet을 해석할 수 있어야 한다.

검증 assertion 권고:

- 2-byte mode에서 모든 전송 code의 bit 16이 0인지 확인
- 설정 범위가 17-bit code 상한을 넘지 않는지 확인
- 첫 packet이 Header-only 1440 byte인지 확인
- 거리 데이터가 두 번째 packet에서 시작하는지 확인
- Face마다 선언한 sample count와 실제 전송 word 수가 같은지 확인
- packet payload가 1440 byte를 넘지 않는지 확인
- 32/64/128-bit 입력에서 Repack 결과 byte stream이 동일한지 확인

## 6. 남은 wire-format 확정 항목

Timing 계산에는 영향을 주지 않지만 구현 전에 다음 두 항목은 ABI로 고정해야 한다.

- 2/3-byte word의 byte order: 기존 Header와 동일한 little-endian 권고
- 마지막 거리 packet의 padding 정책: 현재 HTML은 실제 남은 byte만 전송하는 것으로 계산
