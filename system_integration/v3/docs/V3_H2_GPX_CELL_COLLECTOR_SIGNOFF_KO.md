# V3 H2 GPX Hit-to-Cell 체크포인트 결과

> 현재 ABI 3.1의 전체 H0~H3 Header, Bit Map과 재검증 수치는
> [`V3_H0_H3_HEADER_CONTRACT_KO.md`](V3_H0_H3_HEADER_CONTRACT_KO.md)를 기준으로 한다.
> 이 문서의 수치는 H2 최초 체크포인트 이력으로 유지한다.

## 1. 판정

**H2 기능 및 단독 구현 PASS**다. V3 HLS `gpx_cell_collector_hls`는 H1에서
생성한 Hit17 이벤트를 V2 RTL과 같은 Cell 이벤트로 수집하며 다음 계약을
만족했다.

- 물리 Return 1~7의 순서와 17-bit Hit 원본값 보존
- Runtime 전시 Return 수 1~7 적용
- Runtime 전시 Return 수를 넘는 물리 Return의 의도적 필터링
- 8번째 이상 물리 Return의 `return_overflow` 분류
- IFIFO1 완료, 전체 Drain 완료, Timeout/Error-fill Cell 순서
- 전용 Rise/Fall Chip, 한 Chip 양 Edge, 네 Chip 전체 양 Edge
- Backpressure 중 출력 고정과 Abort 뒤 오래된 Shot 제거
- 150 MHz와 200 MHz Processing clock의 실제 OOC 배치·배선

이 판정은 **Hit-to-Cell 경계**만 닫는다. H3 이후의 Cell 정렬, PACKED17,
Shot Line, Face Footer, 32/64-bit AXI4-Stream, VDMA 및 Parent 프로젝트의
전체 처리율은 아직 Sign-off하지 않는다.

## 2. 역할과 데이터 흐름

```text
H1 Hit17 이벤트
  gpx_hit_event_t, 218 semantic bits
        |
        | RTL Adapter가 Reset Epoch를 붙이고 AXI4-Stream으로 변환
        v
  232-bit hit_in
        |
        v
gpx_cell_collector_hls
  - Shot 소유권/식별자 검사
  - Chip x STOP x slope별 Return 저장
  - Runtime 전시 Return 필터
  - IFIFO1/DRAIN/TIMEOUT Cell 생성
        |
        | 328-bit result_out
        v
RTL Adapter
  - 상태 응답을 fault pulse/sticky로 변환
  - Cell payload를 gpx_cell_event_t로 복원
  - Backpressure 보관
  - Abort 중 오래된 결과를 소비하되 외부에는 숨김
        |
        v
H3 Cell 정렬 및 Frame 경로
```

Cell 하나의 정의는 다음과 같다.

```text
Cell = Shot 하나 x TDC-GPX Chip 하나 x STOP 하나 x slope 하나
```

Rise와 Fall이 모두 활성인 Chip은 같은 STOP에 대해 Rise Cell과 Fall Cell이
각각 존재한다. Cell 내부의 Return 슬롯은 물리 도착 순서 0~6을 유지한다.

## 3. 물리 Drain과 Runtime 전시 Return 계약

H2는 TDC-GPX IFIFO를 직접 읽지 않는다. 물리 IFIFO Drain은 앞단 RTL이 EF
완료까지 수행한다. 따라서 Runtime 전시 Return 수가 1이어도 외부 TDC-GPX에
존재하는 나머지 Return을 읽지 않고 남겨두는 구조가 아니다.

```text
물리 Return 7개, Runtime 전시 Return 3개 예

외부 TDC-GPX IFIFO : R0 R1 R2 R3 R4 R5 R6  -> 모두 Drain
H2 Cell 저장/출력   : R0 R1 R2              -> 3개만 표시
의도적 필터         :          R3 R4 R5 R6   -> fault 아님
8번째 물리 Return   : R7                     -> return_overflow fault
```

`hit_dropped`는 Runtime 필터에 사용하지 않는다. 내부 용량 손실처럼 의도하지
않은 데이터 손실에만 예약한다.

## 4. 고정 Bit 계약

### 4.1 HLS 입력 232 bit

| Bit | 의미 |
|---|---|
| `[217:0]` | H1 Hit payload 전체 |
| `[223:218]` | 예약, 항상 0 |
| `[231:224]` | Reset Epoch |

Reset Epoch는 Abort 세대 번호다. Adapter가 Abort 상승 Edge마다 1 증가시키고,
다음에 승인되는 Hit와 함께 HLS로 전달한다. HLS가 Backpressure로 정지한 동안
짧은 Abort pulse가 지나가도 다음 Hit에서 세대 변경을 반드시 확인할 수 있다.

### 4.2 HLS 결과 328 bit

모든 승인 Hit는 먼저 상태 응답 한 개를 만든다. DATA Hit는 상태 응답으로
끝나며, IFIFO1/DRAIN/TIMEOUT 이벤트는 상태 응답 뒤에 Cell들을 출력한다.

| Bit | 의미 |
|---|---|
| `[318:0]` | Cell payload |
| `[319]` | Cell 출력 여부 |
| `[320]` | Shot context/설정 불일치 |
| `[321]` | 8번째 이상 물리 Return |
| `[322]` | START number가 0이 아님 |
| `[323]` | 의도하지 않은 내부 용량 손실, 현재 정상 경로에서는 0 |
| `[327:324]` | 예약, 항상 0 |

### 4.3 Cell payload 319 bit

| Bit | 의미 |
|---|---|
| `[1:0]` | DATA / IFIFO1_DONE / DRAIN_DONE / TIMEOUT |
| `[3:2]` | Chip index 0~3 |
| `[4]` | IFIFO index |
| `[7:5]` | STOP index 0~7 |
| `[8]` | slope: 0=Fall, 1=Rise |
| `[11:9]` | Runtime에 표시하는 Return 수 |
| `[14:12]` | 해당 Shot에 고정된 Runtime 전시 Return 설정값 |
| `[133:15]` | Hit0~Hit6, 각 17 bit |
| `[134]` | 의도하지 않은 Hit 손실 예약 Flag |
| `[135]` | 8번째 이상 물리 Return 발생 |
| `[136]` | Timeout/Error-fill Cell |
| `[137]` | Cell 또는 Shot fault |
| `[140:138]` | Timeout 원인 |
| `[302:141]` | Shot context 162 bit |
| `[318:303]` | Chip별 Shot sequence |

Shot identity 비교에는 Chip sequence, Active version, Shot index, Face index,
Simulation source가 포함된다. Shot 도중 Runtime 전시 Return 수나 identity가
바뀌면 그 Shot을 fault 처리하고 처음 승인한 Shot context를 Cell에 유지한다.

## 5. Cell 생성 순서

1. Rise가 활성인 경우 Rise Cell을 STOP 오름차순으로 출력한다.
2. Fall이 활성인 경우 Fall Cell을 STOP 오름차순으로 출력한다.
3. 마지막에 IFIFO1_DONE, DRAIN_DONE 또는 TIMEOUT 제어 Cell을 출력한다.
4. IFIFO1_DONE은 STOP 0~3만 출력하고 Shot 소유권을 유지한다.
5. 이후 DRAIN_DONE/TIMEOUT은 STOP 4 이상을 출력하고 Shot을 종료한다.
6. IFIFO1_DONE이 없었다면 DRAIN_DONE/TIMEOUT이 전체 STOP을 출력한다.

한 Chip에서 Rise/Fall을 모두 활성화할 수 있고, 최대 네 Chip 모두에 Rise/Fall을
동시에 활성화할 수 있다.

## 6. Abort와 Backpressure 안전성

Adapter는 HLS 입력 승인부터 `ap_done`까지 `hls_inflight`를 명시적으로 추적한다.
HLS의 입력 `TREADY`가 다시 1이 되었다는 사실만으로 이전 출력이 모두 끝났다고
판정하지 않는다.

Abort가 발생하면 다음 순서로 처리한다.

1. Reset Epoch를 증가시킨다.
2. 진행 중 호출이나 보류 결과가 있으면 Flush 상태로 진입한다.
3. HLS가 이미 만든 결과는 받아서 제거하되 외부 Cell valid는 0으로 유지한다.
4. `ap_done` 뒤 Flush를 종료한다.
5. 다음 Hit가 새 Reset Epoch를 전달하고 HLS의 모든 Chip Shot 소유권을 지운다.

초기 Adapter 구현은 HLS `TREADY`를 in-flight 종료 근거로 사용하여 Abort 직후
오래된 Cell이 다시 보일 수 있었다. 차등 테스트가 이를 검출했고, 현재의 명시적
in-flight 추적으로 수정했다.

## 7. 검증 행렬

| Profile | Chip | STOP/Chip | Rise mask | Fall mask | 핵심 목적 |
|---|---:|---:|---:|---:|---|
| dedicated | 4 | 8 | `0011` | `1100` | 2 Rise + 2 Fall Chip |
| one_chip_dual | 1 | 8 | `0001` | `0001` | 한 Chip 양 Edge, IFIFO1 분할, Abort |
| reduced | 3 | 6 | `0011` | `0100` | 축소 구성, overflow/start/context fault |
| all_dual | 4 | 8 | `1111` | `1111` | 네 Chip 모두 양 Edge |

### 7.1 독립 CSim

- Runtime 전시 Return 1~7 전체 Sweep
- 물리 Return 7개를 모두 입력한 뒤 노출 수만 변경
- 물리 8번째 Return의 overflow
- `Hit[16]`을 포함한 17-bit 값 보존
- START number 비정상, context 불일치
- IFIFO1 lower, DRAIN upper, TIMEOUT 전체 Error-fill
- Reset Epoch 변경 뒤 같은 Chip/STOP 주소 재사용
- 네 가지 Profile 모두 PASS

### 7.2 C/RTL Co-simulation

네 Profile 모두 Verilog Co-simulation PASS다. LUTRAM의 동기 읽기 지연과 HLS
스케줄링을 포함한 생성 RTL 결과가 C 모델과 일치했다.

### 7.3 V2/HLS 차등 회귀

네 Profile을 150 MHz와 200 MHz에서 실행한 총 8개 시나리오가 PASS했다.
두 구현이 동시에 Hit를 승인하도록 구동하고, Pipeline latency 차이는 허용하되
다음 항목을 완전 비교했다.

- Cell record 전체
- Cell 순서와 제어 Cell 위치
- fault pulse와 sticky
- Backpressure 중 payload 고정
- Abort 뒤 오래된 출력 차단과 같은 주소 재사용

### 7.4 H1 소급 무회귀

공통 HLS 계약 헤더 변경 뒤 H1도 다시 검증했다.

- H1 CSim, C synthesis, 네 Profile C/RTL Co-simulation PASS
- H1 V2/HLS 차등 8개 시나리오 PASS
- H1 OOC 150 MHz WNS `+1.952 ns`, 200 MHz WNS `+0.768 ns`
- H1 래치 0, 차단 DRC 0

## 8. 성능 및 구현 결과

### 8.1 HLS C synthesis

| 항목 | 결과 |
|---|---:|
| 목표 Clock | 5.000 ns, 200 MHz |
| HLS 추정 Clock | 4.335 ns |
| 함수 latency | 4~70 clocks |
| 다음 호출 interval | 5~71 clocks |
| HLS 추정 LUT | 1,783 |
| HLS 추정 FF | 1,691 |
| BRAM / DSP | 0 / 0 |

전체 함수가 한 입력마다 가변 개수 Cell을 방출하므로 Top 호출 자체는 II=1이
아니다. 내부 Metadata scrub과 Cell 출력 메모리 Loop는 II=1로 Pipeline된다.
V2 Collector도 `S_COLLECT`에서만 Hit를 승인하는 상태 직렬 구조이므로 H2에서
입력 II=1은 기존 계약이 아니다. 다만 상위 FIFO 점유율과 Shot 처리 여유는 H5
통합 회귀에서 반드시 다시 측정한다.

### 8.2 xc7z020clg484-2 OOC 배치·배선

| Processing clock | WNS | Latch | 차단 DRC |
|---:|---:|---:|---:|
| 150 MHz | `+0.342 ns` | 0 | 0 |
| 200 MHz | `+0.172 ns` | 0 | 0 |

200 MHz Routed 결과는 Adapter 포함 1,711 LUT, 2,368 FF, BRAM 0, DSP 0이다.
LUT 중 310개가 LUTRAM이다. 최악 경로는 HLS FSM에서 162-bit context 저장
레지스터의 Clock Enable까지이며 논리 단계 0, Data path 4.497 ns 중 배선이
4.118 ns(91.6%)다.

200 MHz WNS는 양수지만 여유가 크지 않다. H3~H5 통합 배치에서 주변 로직과
함께 다시 확인하며, 이 수치를 V3 전체 Timing Sign-off로 사용하지 않는다.

## 9. 채택한 최적화와 기각한 실험

채택:

- Hit 7개 Bank와 Metadata를 LUTRAM으로 구현
- Chip별 Shot active/context/sequence/max_hits를 LUTRAM으로 구현
- 새 Shot은 자신이 소유권을 정의하므로 같은 Event와 즉시 재비교하지 않음
- Abort in-flight를 Adapter가 명시적으로 추적

기각:

- HLS 목표 4.5 ns: HLS 추정치는 좋아졌으나 실제 200 MHz WNS가 `-0.396 ns`로
  악화되어 복구
- Vivado Aggressive 물리 전략: 기본 `+0.172 ns`보다 낮은 `+0.063 ns`여서 복구
- AXI4-Stream `register_mode=forward`: 결과 변화가 없고 도구 경고만 추가되어 복구

## 10. 남은 제한과 다음 Gate

- OOC Harness에는 외부 입력/출력 지연이 없다. 내부 무제약 Endpoint는 0이지만
  Parent I/O와 전체 배치 영향은 H6에서 확인한다.
- OOC 특성상 PS7 미포함 및 사용되지 않는 Harness 출력 경고가 남는다. 차단 DRC는
  없으며 Parent 통합에서 다시 검사한다.
- H2 처리 중 상위 Hit FIFO가 얼마나 차는지는 아직 실측하지 않았다. H5에서
  150/200 MHz, 200/150 MHz와 4:1 비동기 조합으로 최대 점유율을 확인한다.
- H3는 Cell의 Rise/Fall Lane 정렬과 Shot/Face 구조화를 구현한다.

## 11. 재현 명령

```powershell
# CSim, C synthesis, 네 Profile C/RTL Co-simulation
./system_integration/v3/scripts/run_v3_hls_cell_collector.ps1 -Step all

# V2 RTL과 HLS Adapter 차등: 네 Profile x 150/200 MHz
./system_integration/v3/scripts/run_v3_gpx_cell_collector_diff.ps1

# xc7z020clg484-2 OOC 합성/배치/배선: 150/200 MHz
./system_integration/v3/scripts/run_v3_gpx_cell_collector_impl.ps1 -SkipHlsSynthesis
```

생성 프로젝트와 로그는 저장소 루트 `.work/`에만 두며 Git에는 넣지 않는다.
