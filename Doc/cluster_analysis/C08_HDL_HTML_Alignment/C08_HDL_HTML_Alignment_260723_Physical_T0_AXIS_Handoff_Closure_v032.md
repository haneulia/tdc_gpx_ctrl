# C08 Physical T0 and AXIS Handoff Closure v032

## 1. 목적

`motor_decoder -> laser_ctrl -> echo_receiver -> tdc_gpx_top` 연결에서 AXI4-Stream의 역압 정책과 실제 거리 기준점 T0의 경로를 분리해 확정한다. 핵심은 실제 발광 완료 신호인 `fire_done`이 물리 TDC-GPX START 핀을 즉시 구동하고, 동기화된 복사본만 Echo/TDC 제어 상태기에 전달되게 하는 것이다.

실제 parent Vivado 프로젝트는 아직 없다. 따라서 이번 closure는 각 IP RTL, 패키징된 IP, OOC 합성, GUI 프로젝트 생성, `tb_tdc_gpx_full_int` 실행형 연결 모델까지의 계약을 의미한다. 향후 parent Block Design은 이 문서의 배선을 그대로 구현하고 별도 implementation timing sign-off를 수행해야 한다.

## 2. 확정된 시간 경로

```text
motor position AXIS
    -> laser scheduler
    -> fire command
    -> laser driver
    -> raw i_fire_done = physical optical T0
         +-> async-set bridge -> laser_ctrl.o_start_tdc -> physical GPX START pin
         +-> 2FF synchronizer + edge detector -> laser_ctrl.o_shot_start
                                              -> echo_receiver.i_shot_start
                                              -> tdc_gpx_top.i_shot_start
```

### 2.1 물리 T0

- 기준 신호: `laser_ctrl.i_fire_done`
- 의미: 발사 명령 시점이 아니라 laser driver가 실제 발광 완료를 알린 시점
- 출력: `laser_ctrl.o_start_tdc`
- 구현: asynchronous set, synchronous release
- assertion 지연: AXIS clock cycle을 추가하지 않음
- 연결 대상: 실제 TDC-GPX START pin만
- 금지: Echo 또는 `tdc_gpx_top.i_shot_start`에 연결하지 않음

`start_tdc_width` 카운터는 pulse 종료와 최소 폭을 보장한다. 따라서 assertion은 raw edge를 보존하고 deassertion만 AXIS clock에 정렬된다.

### 2.2 논리 Shot 경계

- 출력: `laser_ctrl.o_shot_start`
- 생성: 물리 T0를 2FF로 동기화한 뒤 edge detector로 1-clock pulse 생성
- 예상 관측 지연: 비동기 입력 위상에 따라 물리 T0 뒤 약 2~3 AXIS clocks
- 용도: Echo window 시작, TDC Shot bookkeeping, face payload capture
- 금지: 물리 GPX START pin 구동

이 지연은 제어 상태기의 CDC 비용이며 거리 기준 오차에 포함되지 않는다. 거리값의 물리 기준은 `o_start_tdc` 경로다.

## 3. AXIS 계약

| 순서 | 경로 | 데이터 계약 | TREADY | 판단 |
|---|---|---|---|---|
| H00 | Motor -> Laser | TDATA[14:0] position, TUSER[9] face_valid, [8:4] active mask, [3] sim, [2:0] face, TLAST face end | 없음 | 실시간 위치 source는 정지시킬 수 없는 관측 스트림 |
| H01 | Laser -> Echo descriptor | TDATA[15:0] fire count, TUSER[20:6] position, [5] last, [4] shot-open, [3] sim, [2:0] face, TLAST=1 | 있음 | 수락된 Shot마다 1 beat, 미소비 beat는 안정적으로 유지 |
| H02 | Echo `m_stop_evt` | stop별 rise/fall 누적 count와 TKEEP | 없음 | 물리 edge 관측 진단 스트림 |
| H03 | Echo `m_fire_count` | event beat와 Shot-final TLAST beat | 없음 | Shot별 관측/검증 스트림 |
| H04 | TDC Rise/Fall VDMA | packed line data, TKEEP/TLAST/TUSER | 있음 | 저장 경로이므로 일반적인 backpressure 적용 |

Motor 스트림과 Echo 진단 스트림은 AXIS signal naming을 사용하지만 source 사건을 되돌릴 수 없으므로 `HAS_TREADY=0`이다. 반면 Laser descriptor와 TDC VDMA는 내부 보존 저장소가 있으므로 `TVALID/TREADY` 계약을 사용한다.

## 4. Shot descriptor와 T0의 관계

Laser descriptor는 Shot 수락 시 생성되어 물리 T0보다 먼저 Echo에 전달될 수 있다. Echo는 descriptor에서 fire count와 metadata를 보관하고, 실제 capture window는 별도 `o_shot_start` pulse에서 연다.

```text
accepted shot descriptor handshake
    -> fire command
    -> raw fire_done / physical GPX START
    -> synchronized o_shot_start / logical window open
    -> max_roundtrip
    -> o_stop_tdc / logical window close
```

Descriptor `TREADY`가 낮으면 pending descriptor가 다음 Shot의 admission을 막는다. 이미 발생한 물리 START/STOP timing은 descriptor `TREADY`에 의존하지 않는다.

## 5. Echo 출력 정책

`m_stop_evt`와 `m_fire_count`에서 사용되지 않던 ready 입력을 제거하고 IP metadata를 `HAS_TREADY=0`으로 고정했다. 그 이유는 다음과 같다.

1. LVDS로 들어온 물리 STOP edge는 downstream ready로 지연하거나 재생할 수 없다.
2. 기존 ready 입력은 RTL에서 소비되지 않아 Block Design 사용자에게 허위 역압 보장을 암시했다.
3. 이 스트림은 GPX IFIFO word 수나 drain 완료를 결정하지 않는 진단 출력이다.
4. 무손실 장기 저장이 필요하면 parent에서 충분한 폭의 event capture FIFO 또는 counter snapshot 인터페이스를 별도로 두어야 한다.

## 6. 검증 결과

### 6.1 단위 및 IP 패키징

| 범위 | 결과 |
|---|---|
| Laser Vivado regression | 7 TB, 145 checks, 0 failed |
| Laser IP package smoke | PASS |
| Motor IP package DRC/default recalculation | PASS, 80 ms와 8 ns 입력에서 694/695/6400 확인 |
| Echo core-only regression | 9 scenarios PASS |
| Echo IP package DRC | PASS, revision 4 |
| Echo xc7z020 OOC synthesis | PASS |
| Echo fresh GUI project | PASS |

### 6.2 전체 연결 회귀

| Encoder | AXIS/TDC MHz | Shot/START/STOP | Rise/Fall 출력 | 결과 |
|---|---:|---:|---:|---|
| external | 150/200 | 8/8/8 | 192/192 beats, TLAST 8/8 | PASS |
| external | 200/200 | 8/8/8 | 192/192 beats, TLAST 8/8 | PASS |
| internal simulation | 150/200 | logical 2/2, physical fire 0 | 48/48 beats, TLAST 2/2 | PASS |
| internal simulation | 200/200 | logical 2/2, physical fire 0 | 48/48 beats, TLAST 2/2 | PASS |

네 시나리오 모두 `face_valid_violations=0`, `stale_alias_gpx_shots=0`, `cfg_rejected=0`, `pipeline_abort=0`이다. 외부 모드는 `laser_fire = laser_start = laser_stop = laser_result_tlast = 8`을 만족한다.

## 7. HTML 반영

`C08_HDL_HTML_Alignment_260723_Physical_T0_AXIS_Handoff_Simulator_v021.html`에 다음을 추가했다.

- 첫 화면의 물리 T0와 논리 Shot 분기 도식
- H00..H04 인터페이스 표와 `TREADY` 유무
- D00 계산표의 Motor position, Shot descriptor, physical T0, logical Shot, Echo diagnostic 단계
- 판단표의 physical T0, logical Shot, Echo diagnostic backpressure 항목
- self-test의 TUSER 10/21-bit, ready/no-ready, zero-axis-cycle assertion 계약 검사

## 8. Parent Block Design 연결 규칙

```text
motor_decoder.m_axis_*          -> laser_ctrl.s_axis_*
motor_decoder.o_n_faces         -> tdc_gpx_top.i_n_faces

laser_ctrl.m_axis_*             -> echo_receiver.s_laser_evt_*
laser_ctrl.o_start_tdc          -> physical TDC-GPX START pin
laser_ctrl.o_shot_start         -> echo_receiver.i_shot_start
laser_ctrl.o_shot_start         -> tdc_gpx_top.i_shot_start
laser_ctrl.o_shot_face_index    -> tdc_gpx_top.i_shot_face_index
laser_ctrl.o_stop_tdc           -> echo_receiver.i_stop_tdc
laser_ctrl.o_stop_tdc           -> tdc_gpx_top.i_stop_tdc

echo_receiver.o_tdc_stop        -> physical TDC-GPX STOP pins
echo_receiver.m_stop_evt_*      -> optional diagnostics only
echo_receiver.m_fire_count_*    -> optional diagnostics only
```

`o_shot_start` 한 신호를 Echo와 TDC에 fan-out하는 것은 합리적이다. 두 소비자가 같은 AXIS clock domain에서 같은 Shot 경계를 보며, 이 pulse에는 ready가 없다. metadata stream만 Laser에서 Echo로 독립 handshake한다.

## 9. 잔여 항목

1. 실제 parent 프로젝트를 만든 뒤 위 배선을 Block Design validation과 post-route timing으로 재확인해야 한다.
2. `fire_done -> GPX START` 비동기 set 경로에는 보드 핀까지의 dedicated timing/placement constraint와 pulse-width 검증이 필요하다.
3. Laser의 기존 GUI `.xpr`은 오래된 수동 source order를 포함한다. RTL sign-off는 새 회귀 실행기와 package smoke로 통과했지만 GUI 프로젝트를 기준 산출물로 쓸 경우 source set 재생성이 필요하다.
4. 모든 Shot의 무손실 수락이 절대 요구사항이면 사후 ready가 아니라 발사 전 `tdc_ready/fire_admit` 계약을 별도 설계해야 한다.

## 10. 판정

수정 방향은 합리적이다. 물리 시간 기준을 AXIS handshake에서 분리했고, 저장 가능한 데이터에만 backpressure를 적용했으며, 관측 전용 물리 event에는 허위 ready를 제거했다. 현재 범위는 module/integration sign-off 가능하지만 실제 parent와 보드 timing sign-off까지 완료된 것으로 해석해서는 안 된다.
