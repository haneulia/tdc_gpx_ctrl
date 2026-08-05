# V2 Checkpoint I5 - B8 Read Pipeline Optimization

## 1. 목적

B5-B8 통합 OOC에서 확인된 B8 최악 경로는 4 LUT 단계였고 지연의 약 70%가
배선이었다. 경로의 끝은 payload `D`가 아니라 빈 Cell을 0으로 만드는 넓은
synchronous-reset 입력이었다. 따라서 단순 물리 제약보다 데이터 유효성 경계를
앞단에서 등록하는 구조가 우선이다.

## 2. 적용 구조

Rise/Fall lane을 다음 3단 elastic pipeline으로 구성했다.

```text
Chip/STOP cursor
    -> registered read request(address, presence, identity)
    -> packed LUTRAM prefetch or zero blank word
    -> typed Cell output holding register
```

- reset/abort는 각 단계의 `valid`만 지운다.
- payload LUTRAM은 reset하지 않고 per-Shot presence가 유효성을 단독 소유한다.
- blank Cell은 prefetch 단계에서 명시적인 all-zero word로 만든다.
- Rise/Fall ready chain은 독립적이다.
- 파이프라인 latency는 1 clock 증가하지만 warm-up 뒤 II는 1 Cell/clock이다.
- `read address`와 `read presence`에는 `max_fanout=16`을 적용했다.

## 3. 기능 검증

`260806_j0_read_req_pipe_v2_gpx_frame_lane_assembler`에서 다음 14개 profile이
모두 PASS했다.

- dedicated, dual-edge, Falling OFF, fault injection, 4-Chip all-dual
- runtime slope mask와 runtime 4-Chip all-dual
- 각 profile의 150 MHz 및 200 MHz

`260806_j0_read_req_pipe_b5b8_v2_gpx_b5_b8_subsystem`에서도 dedicated와
4-Chip all-dual의 `proc150/tdc200`, `proc200/tdc150` 통합 회귀가 모두 PASS했다.
Cell 순서, payload, line end, Shot done, Face close, CDC, latch, blocking DRC의
회귀는 없었다.

## 4. 타이밍 결과

| profile | I4 baseline WNS | I5 WNS | 판정 |
|---|---:|---:|---|
| proc 150 / TDC 200 MHz | +0.366 ns | +0.164 ns | MET, 새 최악 경로는 TDC merge이며 B8 외부 |
| proc 200 / TDC 150 MHz | +0.478 ns | +0.496 ns | MET |

proc 200 MHz 통합 보고서에서 기존 `FSM -> Fall hit payload reset` 경로는 최악
경로 목록에서 제거됐다. B8의 다음 후보는 `FSM -> Rise gap_before reset`이며
3 LUT, WNS +0.572 ns다. 넓은 hit payload reset 경로보다 짧고 여유가 크다.

단독 B8 자원은 709 LUT / 1699 FF에서 727 LUT / 1729 FF로 증가했다. 증가는
Rise/Fall read-request metadata와 elastic valid 제어에 해당한다.

## 5. 판단 및 잔여 위험

목표 경로는 개선되었고 기능 계약과 II=1을 유지하므로 3단 구조를 채택한다.
다만 `proc150/tdc200` 전체 WNS는 배치 변화로 TDC-domain merge 경로가
+0.164 ns까지 이동했다. 이는 B8 논리 경로가 아니지만 최종 B9 및 parent 구현에서
반드시 다시 측정한다. B9 추가 후 WNS가 0 이하가 되면 TDC merge의 별도 순차
분할을 우선 검토한다.

`gap_before`를 모든 Cell에 반복해 reset 경로를 제거하는 시도는 기존 계약
(`line_start=0`이면 `gap_before=0`)을 위반해 TB가 즉시 검출했으며 반영하지 않았다.
타이밍 최적화가 데이터 의미를 바꾸지 않는다는 원칙을 유지한다.
