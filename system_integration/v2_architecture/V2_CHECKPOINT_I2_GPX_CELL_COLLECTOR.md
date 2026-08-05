# Checkpoint I2 GPX Cell Collector

## 1. 목적과 완료 범위

I2/B7은 B6의 `gpx_hit_event_t`를 폭에 의존하지 않는
`gpx_cell_event_t`로 수집한다. 이 단계의 Cell 정의는 다음과 같다.

```text
Cell = Shot 1개 x GPX Chip 1개 x STOP 1개 x slope 1개
```

Cell은 최대 7개의 17-bit Hit와 해당 Hit의 유효 개수, Drop, 오류 및
Shot 문맥을 가진다. AXIS `32/64/128-bit`, byte repack, line padding,
VDMA HSIZE/VSIZE는 B7 입력이나 저장 주소에 존재하지 않는다. 따라서 출력
폭을 바꿔도 Cell의 정보량과 canonical byte 수는 바뀌지 않는다.

## 2. 모듈 역할과 데이터 흐름

```mermaid
flowchart LR
    B6["B6 gpx_hit_event_t"] --> ID["Shot/Chip/STOP/slope identity"]
    ID --> META["Cell metadata read/apply pipeline"]
    ID --> RAM["7 x 64-address 17-bit LUTRAM banks"]
    META --> ORDER["Rise STOP order, then Fall STOP order"]
    RAM --> ORDER
    ORDER --> B7["B7 gpx_cell_event_t"]
    ORDER --> DIAG["pulse/sticky diagnostics"]
    B7 --> B8["Frame lane assembler"]
```

| 파일 | 역할 |
|---|---|
| `pkg/lidar_gpx_data_pkg.vhd` | Hit array, Cell record, Cell event kind와 fault record 정의 |
| `rtl/proc/lidar_gpx_cell_collector.vhd` | Hit 수집, runtime Hit 제한, Cell 정렬과 timeout 복구 |
| `tb/tb_lidar_gpx_cell_collector.vhd` | dedicated, dual-edge, fault, timeout, abort와 backpressure 검증 |
| `tb/lidar_gpx_cell_collector_impl.vhd` | OOC 합성용 평탄화 하네스 |
| `scripts/run_v2_gpx_cell_collector.ps1` | 150/200 MHz 기능 및 구현 회귀와 증거 보관 |

## 3. 저장 구조

최대 주소 공간은 다음 64개다.

```text
address = chip * 16 + slope_offset + stop
slope_offset = 0 for Rise, 8 for Fall
chip = 0..3, stop = 0..7
```

Hit payload는 B7이 결정한 Return 순서별 7개 LUTRAM bank에 저장한다.

```text
bank[return 0] : 64 x 17 bit
...
bank[return 6] : 64 x 17 bit
total          : 64 x 7 x 17 = 7,616 bit
```

입력은 한 번에 Hit 하나만 쓰고, Cell 출력은 선택된 주소의 7개 bank를 병렬로
읽는다. 별도 FF 배열이던 count/drop/error는 다음 64x5 metadata LUTRAM 하나로
통합했다.

```text
meta[2:0] = 해당 Cell이 수신한 물리 Return 수, 0..7 포화
meta[3]   = runtime max_hits 초과로 버린 Hit 존재
meta[4]   = 해당 Cell 오류
```

payload와 metadata RAM은 reset하지 않는다. 각 Chip의 새 Shot 첫 event를 입력
register에 보관한 채 그 Chip의 16개 metadata 주소를 한 clock에 하나씩 0으로
scrub한다. count가 0이면 stale payload가 보이지 않고, Cell을 읽을 때 count 밖의
slot을 0으로 마스킹한다. abort도 큰 RAM reset fanout을 만들지 않으며 다음 Shot의
동일 scrub 절차가 stale Hit 노출을 막는다.

### 3.1 LUTRAM을 선택한 이유

FIFO는 받은 순서대로 내보낼 때 적합하다. B7 입력은 Chip, STOP, slope가 섞여
들어오지만 출력은 Cell 주소 순서로 다시 정렬해야 하므로 random-access 저장소가
필요하다. FIFO를 사용하면 최대 64개 Cell별 FIFO 또는 별도 재정렬 RAM이 다시
필요해진다.

BRAM은 119-bit Cell 폭에서 17-bit Return slot 하나만 갱신하려면 여러 BRAM을
낭비하거나 read-modify-write가 필요하다. 현재 7,616-bit 용량에는 7개의
17-bit LUTRAM bank가 더 단순하다. 최종 합성은 `RAM64M` 42개, report 기준
LUTRAM 168개로 정상 추론됐다.

## 4. Hit 수집 규칙

### 4.1 Return과 visible Hit

B7은 같은 `Shot + Chip + STOP + slope` Cell에 정상 수신된 순서대로 payload bank
0..6을 선택한다. runtime `max_hits_per_stop`은 Cell에 보이는 Hit 수만 제한하며,
물리 GPX drain은 계속 최대 7 Return을 읽는다.

```text
accepted_count = 실제 수신 Return 수, 0..7 포화
visible_count  = min(accepted_count, runtime max_hits)
```

예를 들어 runtime 값이 3이면 수신 순서 0, 1, 2는 저장하고 3..6은 계속 소비하되
Cell에 노출하지 않는다. 해당 Cell의 `hit_dropped=1`과 `hit_capacity_drop`
진단을 남긴다. 8번째 event는 count를 wrap하지 않고 `return_overflow`와 drop/error
진단을 남긴다. GPX IFIFO drain 수를 줄이지 않으므로 다음 Shot에 이전 데이터가
남는 원인이 되지 않는다.

runtime 값은 첫 Chip/Shot event에서 snapshot한다. 같은 Chip Shot 중 값이나
active version이 바뀌면 `context_mismatch`로 진단한다.

### 4.2 Hit bit 보존

모든 Hit는 B7까지 `Hit[16:0]` 17 bit로 저장된다. `Hit[16]`을 payload에서
미리 제거하지 않는다. B8/B9 formatter가 canonical 32-bit word를 만들 때만
하위 16 bit payload와 Hit-MSB metadata를 구성한다.

### 4.3 현재 StartNum 정책

B6는 GPX `StartNum[7:0]`을 손실 없이 전달한다. 현재 Frame ABI는 한 Shot의
START 하나만 표현하므로 B7에서 `StartNum /= 0`을 unsupported multi-START로
진단하고 Cell을 faulted로 표시한다. 값을 조용히 무시하거나 다른 Shot에 합치지
않는다.

## 5. Cell 출력 순서

Chip별 IFIFO 제어 event가 Cell serialization 경계를 연다.

| 입력 제어 event | Cell 출력 | 마지막 제어 Cell |
|---|---|---|
| `GPX_HIT_IFIFO1_DONE` | 활성 Rise STOP 0..3, 활성 Fall STOP 0..3 | `GPX_CELL_IFIFO1_DONE` |
| `GPX_HIT_DRAIN_DONE` | 남은 Rise STOP 4..7, 활성 Fall STOP 4..7 | `GPX_CELL_DRAIN_DONE` |
| `GPX_HIT_TIMEOUT` | 아직 내보내지 않은 모든 slope/STOP, `error_fill=1` | `GPX_CELL_TIMEOUT` |

한 Chip이 Rise만 지원하면 Fall Cell은 생성하지 않는다. Fall만 지원하면 Rise
Cell은 생성하지 않는다. 한 Chip dual-edge이면 Rise STOP 오름차순을 먼저 내보낸
뒤 Fall STOP 오름차순을 내보낸다. 따라서 비활성 lane builder가 공회전하거나
blank Cell을 내부 FIFO에 쌓는 v1 문제가 재발하지 않는다.

출력 valid 동안 `i_cell_ready=0`이면 전체 Cell record가 안정적으로 유지된다.
abort는 입력 ready를 즉시 0으로 하고 pending Cell과 metadata를 폐기한다.

## 6. 순차 처리와 처리 예산

200 MHz 타이밍을 위해 한 Hit 처리를 다음 순차 단계로 나눴다.

```text
1. 입력 event와 runtime max_hits snapshot 등록
2. Chip별 owner 문맥 선택 및 등록
3. 52-bit identity를 16/16/16/4-bit 비교 register로 분할
4. 비교 결과 축약과 data/control 경로 선택
5. metadata LUTRAM read
6. Return 검사와 payload/metadata LUTRAM write
```

ready가 계속 높은 경우의 B7 단독 처리량은 다음과 같다.

| 항목 | Processing clocks |
|---|---:|
| 활성 Shot의 정상 Hit 다음 accept 간격 | 6 |
| 새 Chip Shot의 첫 event 추가 비용 | 16 metadata scrub clocks |
| Data Cell 하나 출력 | 4 + downstream wait |
| Group control Cell 출력 | 2 |

이 수치는 단일 B7의 상태 전이 비용이다. B6 입력/output register, B8/B9,
VDMA/Ethernet을 포함한 전체 Shot 예산은 I4와 Stage 8 HTML 정렬에서 합산한다.
순차화로 latency는 늘었지만 B6/B7 사이의 wide combinational 경로를 제거했고,
현재 GPX bus drain 속도와 Shot 간격 계약 안에서는 처리량을 제한하지 않는다.

## 7. 진단

| Fault | 의미 | 데이터 처리 |
|---|---|---|
| `context_mismatch` | 같은 Chip Shot에서 sequence/version/shot identity 또는 max_hits 불일치 | Shot fault 표시 |
| `return_overflow` | 같은 Cell의 8번째 이후 물리 Return | consume-drop, count no-wrap |
| `start_number_nonzero` | 현재 single-START Frame 계약 밖의 StartNum | Hit는 보존, Cell fault 표시 |
| `hit_capacity_drop` | runtime visible Hit 제한 초과 | Hit consume-drop, Cell dropped 표시 |

각 fault는 1-clock pulse와 explicit clear 전까지 유지되는 sticky를 제공한다. CSR
bit 배정은 B8/I4의 전체 데이터 진단을 함께 본 뒤 통합 status owner에서 확정한다.

## 8. Echo Receiver 비회귀 계약

B7은 Echo CSR나 합성 옵션을 변경하지 않았다. 최대 32채널 synthetic delay를
위해 채널별 CSR table을 만들지 않는다. **CTL20 한 개**에 두 필드만 둔다.

| CSR | Bit | 의미 |
|---|---:|---|
| CTL20 | `[15:0]` | Channel 0 기준 지연, 5 ns ticks |
| CTL20 | `[31:16]` | 다음 채널마다 더할 지연, 5 ns ticks/channel |

```text
delay[channel] = CHANNEL_0_DELAY + channel * CHANNEL_STEP
channel = 0..31
```

합성 전 옵션도 유지된다.

| Build option | 합성 결과 |
|---|---|
| `enable_echo_receiver=false` | physical receiver와 synthetic STOP 경로 제거 |
| receiver=true, simulation=false | physical LVDS-to-STOP만 합성 |
| receiver=true, simulation=true | physical 경로와 synthetic source 합성 |
| receiver=false, simulation=true | build validation에서 거부 |

CTL20은 synthetic source 전용이며 physical LVDS-to-STOP 초저지연 경로에는
들어가지 않는다.

## 9. 검증 결과

| Scenario | 150 MHz | 200 MHz | 확인 내용 |
|---|---|---|---|
| Dedicated 2-Rise/2-Fall | PASS | PASS | Rise/Fall 전용 Chip 정렬, Hit[16], runtime drop |
| One-Chip dual-edge | PASS | PASS | Rise/Fall 독립 Cell과 결정적 순서 |
| Fault/timeout/abort | PASS | PASS | 8번째 Return, 네 fault, error-fill, sticky clear, stale metadata scrub |
| Backpressure | PASS | PASS | 전체 Cell record 안정성 |
| B6-B7 직접 연결 | PASS | PASS | raw 28-bit 입력부터 Cell exact compare, max_hits=3, 8번째 Return, output stall |

`xc7z020clg484-2` OOC post-route 결과:

| Processing clock | WNS | Total LUT | LUTRAM | FF | Latch | Blocking DRC |
|---:|---:|---:|---:|---:|---:|---:|
| 150 MHz | +1.674 ns | 651 | 176 | 1,170 | 0 | 0 |
| 200 MHz | +0.864 ns | 651 | 176 | 1,170 | 0 | 0 |

B6+B7 직접 연결 OOC 결과:

| Processing clock | WNS | Total LUT | LUTRAM | FF | Latch | Blocking DRC |
|---:|---:|---:|---:|---:|---:|---:|
| 150 MHz | +2.195 ns | 666 | 176 | 1,473 | 0 | 0 |
| 200 MHz | +0.745 ns | 666 | 176 | 1,473 | 0 | 0 |

최종 증거:

- `signoff_results/sessions/260805_b6b7_input_select_final_v2_gpx_cell_collector`
- `signoff_results/sessions/260805_b6b7_sequential_final_v2_gpx_cell_collector`

OOC clock port에는 parent의 `HD.CLK_SRC`가 없으므로 최종 clock insertion/skew는
I4 parent 통합 구현에서 다시 확인한다. 최적화 전후 비교와 경계별 이유는
`V2_CHECKPOINT_I2A_GPX_PIPELINE_OPTIMIZATION.md`에 기록한다. I2는 B7 경계
완료이며 Stage 6 전체 sign-off는 아니다. 다음 단계는 I3/B8 Frame lane assembly다.
