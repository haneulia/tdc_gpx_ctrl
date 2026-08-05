# Stage 6 / I0 Hit, Cell and Frame Oracle

## 1. 목적

Stage 6은 GPX에서 읽은 28-bit raw word를 Hit, Cell, Frame으로 바꾸는
Processing-domain 경계를 마이그레이션한다. I0는 코드를 옮기기 전에 B6~B8의
의미와 v1에서 고쳐야 할 항목을 고정한다. H3의 raw acquisition 결과는 변경하지
않고 아래 순서로만 확장한다.

```text
gpx_raw_event_t -> gpx_hit_event_t -> cell_t -> frame_t
       B5                 B6            B7         B8
```

## 2. 동결한 v1 소스

| 파일 | SHA-256 | 관찰 역할 |
|---|---|---|
| `tdc_gpx_decoder_i_mode.vhd` | `7EF75BE0F7A09CB85ACEB6E3EEA550E680FD93A5463CFF4DAFA2E0F9EDE6FE1D` | 28-bit I-Mode field 분해와 STOP 복원 |
| `tdc_gpx_raw_event_builder.vhd` | `D014B69273546B2A3BE9D35EA807D01FF81D13F831480EF86DAE77FB7D6CA865` | Chip/Shot 문맥과 Return 순번 부여 |
| `tdc_gpx_cell_pipe.vhd` | `02CAC9919A9681891840A09AFA35F236318AD53D81D2760ABD265C6C03874409` | slope lane 분리와 비활성 lane 차단 |
| `tdc_gpx_cell_builder.vhd` | `7B6A6698243E34ABAC22430856531E85A97CF747D5AEB5FDFF208E5F59D4CDEE` | STOP/slope Cell 수집과 Hit[16] 보존 |
| `tdc_gpx_face_assembler.vhd` | `13BB5875B222EF07F9429590E8D7CE25AE2D84C04D26939AC672AA34EC11A347` | Chip slice를 slope별 Face line으로 조립 |

이 hash는 v1을 수정 대상으로 삼기 위한 것이 아니라, v2 비교 기준이 중간에
바뀌지 않았음을 확인하기 위한 것이다.

## 3. B6 Raw-to-Hit 계약

### 3.1 외부 GPX word

| Bit | 이름 | B6 처리 |
|---:|---|---|
| `[27:26]` | `ChaCode` | 선택된 IFIFO 내부 채널 0..3으로 보존 |
| `[25:18]` | `StartNum` | 현재 single-shot 기본값은 0이지만 8 bit 전체를 보존 |
| `[17]` | `Slope` | `1=Rise`, `0=Fall`로 보존 |
| `[16:0]` | `Hit` | 거리/시간 code 17 bit 전체를 보존 |

STOP 번호는 별도 CSR 없이 다음과 같이 복원한다.

```text
IFIFO1: stop = ChaCode       -- STOP 0..3
IFIFO2: stop = 4 + ChaCode   -- STOP 4..7
```

`stops_per_chip`보다 큰 복원 결과는 데이터로 내보내지 않고 identity fault로
기록한다.

### 3.2 Return 번호

Return 번호는 다음 네 값이 같은 Hit끼리 독립적으로 0부터 증가한다.

```text
key = chip + stop + slope + shot
return_index = 0..max_returns_per_stop-1
```

따라서 한 Chip dual-edge 구성에서 같은 STOP의 Rise Return 0과 Fall Return 0은
서로 독립적이다. 8번째 Return은 3-bit index를 0으로 wrap하지 않고 consume-drop
후 `return_overflow` 진단을 남긴다. runtime `max_hits_per_stop`은 이후 Cell이
노출할 Hit 수를 제한하는 값이며, 물리 raw drain이나 B6 Return identity를
줄이는 값으로 사용하지 않는다.

### 3.3 제어 event

| B5 event | B6 event | Return counter 처리 |
|---|---|---|
| `GPX_RAW_IFIFO1_DONE` | `GPX_HIT_IFIFO1_DONE` | 유지; IFIFO2가 이어짐 |
| `GPX_RAW_DRAIN_DONE` | `GPX_HIT_DRAIN_DONE` | 해당 Chip의 모든 STOP/slope counter clear |
| `GPX_RAW_TIMEOUT` | `GPX_HIT_TIMEOUT` | 해당 Chip counter clear, fault 문맥 보존 |
| abort/reset | 출력 valid와 모든 counter clear | 다음 Shot을 Return 0부터 시작 |

모든 제어 event도 Chip, IFIFO, Shot context, Chip shot sequence, fault/timeout
원인을 보존한다.

## 4. v1과 의도적으로 달라지는 부분

v1 `tdc_gpx_raw_event_builder`의 `hit_seq_local` counter는 STOP당 하나라서
Rise/Fall이 counter를 공유한다. 전용 slope Chip에서는 문제가 드러나지 않지만,
한 Chip dual-edge에서는 입력 순서가 Rise0, Fall0일 때 표시 순번이 0, 1로
갈라진다. 이후 v1 Cell builder는 slope별 별도 count를 다시 사용하므로 실제
Cell 저장 순서는 우연히 정상이고, 중간 B6 Return metadata만 일관되지 않다.

v2는 사용자가 확정한 "STOP/slope당 최대 7 Return" 계약에 맞춰 counter를
분리한다. 이것은 byte formatter 변경이 아니라 B6 identity 결함 교정이며,
dual-edge exact test로 별도 증명한다.

## 5. B7/B8 후속 불변조건

1. Hit `[15:0]`은 payload, Hit `[16]`은 Cell metadata에 손실 없이 남는다.
2. Cell의 canonical byte 수는 AXIS 32/64/128-bit 폭과 무관하다.
3. Rise/Fall lane은 같은 구현을 parameterize하며 비활성 lane builder는 합성 또는
   event 전달 단계에서 정지한다.
4. Frame line 순서는 Chip, STOP, slope topology에서 결정되고 padding으로
   HSIZE/VSIZE를 바꾸지 않는다.
5. B7/B8이 완료되기 전에는 Stage 6 전체를 sign-off로 표시하지 않는다.

## 6. B6 검증 벡터

| Scenario | 핵심 검사 |
|---|---|
| Dedicated 2-Rise/2-Fall | 4 Chip x 8 STOP x 7 Return, 32 물리 lane, field/Shot exactness |
| One-Chip dual-edge | 같은 STOP에 Rise/Fall 7개를 교차 입력해 각각 Return 0..6 확인 |
| Overflow | 8번째 Return consume-drop, no wrap, pulse/sticky 확인 |
| Identity fault | absent Chip, STOP 범위 초과, build slope 역할 위반을 구분 |
| Backpressure | output valid 동안 모든 typed field 안정성 확인 |
| Clock | Processing 150 MHz와 200 MHz 기능 및 OOC route 확인 |

Echo Receiver의 compact delay와 build option은 이 단계에서 변경하지 않는다.
Synthetic Echo는 계속 CTL20의 `CH0_DELAY + channel * STEP`만 사용하며,
`enable_echo_receiver`/`enable_echo_simulation` 조합도 기존 G/H 계약을 유지한다.
