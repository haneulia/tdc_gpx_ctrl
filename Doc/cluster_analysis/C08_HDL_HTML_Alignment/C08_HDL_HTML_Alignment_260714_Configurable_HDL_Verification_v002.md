# C08 Configurable HDL Verification v002

- Date: 2026-07-14
- Stage: C08-S5
- Simulator: `C08_HDL_HTML_Alignment_260714_Configurable_HDL_Verification_Simulator_v005.html`

## 판정 분리

상단 `목표 Frame rate 판단`은 다음 두 시간 조건만 판정한다.

```text
frame_rate_pass = shot_pass and ethernet_pass
```

Release 기준값 차이, 광학 정렬, TDC 채널 수와 HDL field/clock/range 계약은 하단 검증표와 HDL verdict에서 별도로 판정한다. 따라서 두 margin이 양수이면 release baseline과 달라도 상단은 PASS이며, baseline 차이는 `SIM PASS / BASELINE DIFF`로 표시한다.

## 가변 HDL 입력

- Compile-time 시나리오: `g_OUTPUT_WIDTH`, `g_AXIS_CLK_MHZ`, `g_TDC_CLK_MHZ`, `g_STREAM_CLK_MODE`
- Runtime CSR 시나리오: `active_chip_mask`, `stops_per_chip`, `max_hits_cfg`, `n_faces`, `cols_per_face`, `max_range_5ns_ticks`
- HDL structure 영역의 `effective` readonly 값은 위 입력의 현재 적용값을 되비추는 표시이며 고정 파라미터가 아니다.

- `g_OUTPUT_WIDTH`: 32/64/128 bit
- `g_AXIS_CLK_MHZ`, `g_TDC_CLK_MHZ`: 50/100/125/150/200 MHz
- `g_STREAM_CLK_MODE`: ASYNC/SYNC
- `active_chip_mask`: 0x1..0xF
- `stops_per_chip`: 2..8
- `max_hits_cfg`: 1..7
- `n_faces`: 1..7
- `cols_per_face`: FoV, 해상도와 column policy에서 계산
- `max_range_5ns_ticks`: 거리 자동 계산 또는 CSR 수동 입력

위 값은 packet row/beat/byte 수, AXIS/DDR/Ethernet 시간, range clock 변환과 PASS/CHECK에 직접 연결된다. v004 저장값은 v005에서 자동 이관하며, 기존처럼 `echoes`로부터 `max_hits_cfg`를 파생하던 설정도 이관 시 보존한다.

## 고정 구조 계약

지원 출력 폭 집합, 지원 클럭 집합, 5 ns 기준 시간축, 최대 hit 7, header 크기, hit slot 비트폭과 두 slope stream은 현재 합성 HDL 구조를 나타내므로 시나리오 입력이 아니라 검증 한계로 고정한다. 이 값이 바뀌면 HTML 상수만 조정하는 것이 아니라 HDL 구조와 regression을 함께 갱신해야 한다.

## Range 시간 budget

상단 shot critical path와 파이프라인은 raw ToF 대신 다음 보수적 시간을 사용한다.

```text
range_budget_us = max(physical_tof_us, tdc_local_range_us, axis_local_range_us)
```

이 방식은 CSR 수동값이 물리 ToF보다 작을 때 timing margin이 과대평가되는 것을 막고, 각 도메인의 ceiling 변환으로 늘어난 시간도 반영한다.

## 확인 결과

- JavaScript syntax: PASS
- DOM ID/reference 및 35개 control wiring: PASS
- 양수 shot/Ethernet margin + baseline diff: Frame rate PASS
- 300 m, 401 tick, TDC 200/AXIS 150: range PASS
- 300 m, manual 100 tick: range CHECK
- SYNC, TDC 200/AXIS 150: clock CHECK
