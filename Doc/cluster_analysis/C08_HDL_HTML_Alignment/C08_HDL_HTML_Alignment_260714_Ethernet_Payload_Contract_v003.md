# C08 Ethernet Payload Contract v003

- Date: 2026-07-14
- Stage: C08-S6
- Simulator: `C08_HDL_HTML_Alignment_260714_Ethernet_Payload_Contract_Simulator_v006.html`

## 원인

현재 `tdc_gpx_header_inserter`는 유효한 모든 AXIS beat에서 `TKEEP`와 `TSTRB`를 전부 1로 출력한다. `cell_builder`도 hit beat 뒤에 폭 전체를 사용하는 1개 metadata beat를 따로 배치한다. 따라서 `g_OUTPUT_WIDTH`가 커지면 AXIS beat 수는 줄지만, cell마다 full-keep으로 인정되는 padding byte가 증가할 수 있다.

```text
raw_vdma_cell_bytes = ceil(max_hits / (g_OUTPUT_WIDTH / 16)) * bus_bytes
                    + 1 * bus_bytes
```

`max_hits=4`이면 cell당 raw VDMA byte는 32/64/128-bit에서 각각 12/16/32 B다. 이 raw DDR frame을 그대로 Ethernet으로 보내면 출력 폭이 커질수록 Ethernet 전송량이 증가하는 현재 HTML 결과가 실제 RTL packet ABI와 일치한다.

동시에 AXIS beats/slope는 108/70/67 beat로 감소하므로 폭 확대에 따른 내부 병렬화 효과도 정상적으로 존재한다. 즉, 폭 확대가 AXIS 직렬화에는 유리하지만 현재 cell 단위 full-keep padding 때문에 DDR 저장량과 raw Ethernet 전송량에는 불리할 수 있는 구조다.

## 시스템 경계 분리

`TDC_GPX_TOP`은 VDMA AXI-Stream까지만 생성하며 Ethernet packetizer는 포함하지 않는다. 따라서 Ethernet budget은 다음 두 계약을 구분해야 한다.

- `Raw VDMA frame`: 현재 full-keep DDR frame을 그대로 송신한다. 폭별 padding 차이가 Ethernet 시간에 반영된다.
- `Compact ABI (repack)`: downstream logic 또는 software가 padding을 제거한 뒤 송신한다. Ethernet 의미 payload는 출력 폭과 무관하다.

Compact cell은 현재 의미 필드만 유지한다.

```text
compact_cell_bytes = max_hits * 2-byte hit_slot + 4-byte metadata
compact_slope_bytes = 48-byte header + rows * compact_cell_bytes
```

Compact 모드는 현재 `TDC_GPX_TOP` 출력 자체가 아니므로 downstream repack 구현과 처리시간 검증이 필요하다. HTML은 이를 `REPACK REQUIRED`로 표시하며 숨은 PASS 조건으로 사용하지 않는다.

## 20 Hz 예시

조건은 450 shots/face, 32 rows/slope, rise/fall 2 streams, `max_hits=4`, Ethernet effective 800 Mbps, rest 6.25 ms다.

| g_OUTPUT_WIDTH | Raw VDMA face | Raw Ethernet | Compact face | Compact Ethernet |
|---:|---:|---:|---:|---:|
| 32 bit | 388.8 kB | 3.888 ms | 388.8 kB | 3.888 ms |
| 64 bit | 504.0 kB | 5.040 ms | 388.8 kB | 3.888 ms |
| 128 bit | 964.8 kB | 9.648 ms | 388.8 kB | 3.888 ms |

따라서 버스 폭 확대가 Ethernet 자체를 느리게 만드는 것이 아니다. 현재 full-keep cell ABI의 폭별 padding을 Ethernet payload로 전달할 때만 그런 결과가 발생한다.

## 개발 판단

단기 검증 기준은 Raw VDMA와 Compact ABI를 모두 유지하는 것이다. 현재 구현 충족 여부는 Raw VDMA로 판단하고, 제품 목표가 width-neutral Ethernet이면 Compact ABI를 별도 downstream 계약으로 확정한다.

장기 HDL 최적화는 cell 경계를 넘는 byte packer 또는 정확한 `TKEEP` 전달을 검토할 수 있다. 이는 `cell_builder`, `face_assembler`, `header_inserter`, VDMA HSIZE와 software parser ABI를 함께 변경하므로 단순 HTML 산식 변경으로 확정해서는 안 된다.

## 검증 결과

- JavaScript syntax: PASS
- DOM/reference: 112 IDs, 36 dynamic controls PASS
- `max_hits=4`, 32/64/128-bit raw: 864/1120/2144 B per shot
- `max_hits=4`, 32/64/128-bit compact: 모두 864 B per shot
- 20 Hz, 800 Mbps, rest 6.25 ms: raw 32/64-bit PASS, raw 128-bit CHECK, compact 전 폭 PASS
