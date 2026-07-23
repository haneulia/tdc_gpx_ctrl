# C08 Static/Dynamic IP Handoff Closure v031

## 1. 목적

`virtual_encoder -> motor_decoder -> laser_ctrl -> tdc_gpx_top` 통합에서 값의 소유권과 전달 방식을 하나의 규칙으로 정리한다. 같은 의미의 설정을 여러 IP가 독립적으로 소유하지 않게 하고, 변화하는 multi-bit 값이 갱신 중간 상태로 관측되는 문제를 막는 것이 목적이다.

> **후속 정정:** 이 문서의 기존 Section 4는 `laser_ctrl.o_start_tdc`를 논리 Shot valid처럼 기술했다. 현재 계약에서는 `o_start_tdc`가 물리 GPX START pin 전용이고, Echo/TDC 제어에는 동기화된 `o_shot_start`를 사용한다. 최신 근거는 `C08_HDL_HTML_Alignment_260723_Physical_T0_AXIS_Handoff_Closure_v032.md`를 따른다.

## 2. 적용 원칙

| 값의 수명 | 대표 값 | 인터페이스 | 판단 |
|---|---|---|---|
| 합성 후 불변 | polygon `n_faces`, build capability | generic 파생 정적 sideband | handshake 불필요 |
| 이벤트마다 변화 | Shot의 `face_index` | event valid + payload | event와 payload를 같은 경계에서 capture |
| 운용 중 설정 변경 | motor center/half width, TDC CSR bundle | request/apply 또는 CDC handshake | atomic apply와 applied readback 필요 |
| 연속 데이터 | GPX raw hit, VDMA stream | AXI4-Stream | `TVALID/TREADY` backpressure 필요 |

직접 연결이 허용되는 조건은 값이 bitstream 동작 중 바뀌지 않고 source와 destination이 같은 정적 시스템 계약을 공유하는 경우뿐이다. 정적이던 값이 향후 런타임 변경 대상이 되면 기존 sideband를 그대로 재사용하지 않고 request/apply 계약으로 승격한다.

## 3. `n_faces` 단일 소유권

- 단일 source: `motor_decoder_top.g_N_FACES`, 허용 범위 1..5
- 전달: `motor_decoder_top.o_n_faces -> tdc_gpx_top.i_n_faces`
- TDC readback: Pipeline `HW_CONFIG[31:29]`
- 제거된 중복 소유권: TDC `MAIN_CTRL[14:12]`
- `MAIN_CTRL[14:12]`는 reserved이며 write가 동작에 영향을 주지 않는다.
- TDC에는 별도 `g_N_FACES`를 두지 않는다.

TDC는 motor가 정한 면 수를 header/status와 face-ID legality에 사용한다. TDC가 자체 modulo counter로 face를 다시 생성하지 않으므로 motor와 TDC의 face 진행이 독립적으로 어긋나는 경로가 제거됐다.

## 4. Shot별 face ID 전달

`laser_ctrl_top.o_shot_face_index`는 동기화된 `o_shot_start`가 유효한 Shot에서 의미가 있다. Parent는 이를 다음 묶음으로 연결한다.

```text
laser_ctrl.o_shot_start      -> tdc_gpx_top.i_shot_start
laser_ctrl.o_shot_face_index -> tdc_gpx_top.i_shot_face_index
```

`laser_ctrl.o_start_tdc`는 raw `i_fire_done`에서 asynchronous set되는 물리 출력이며 실제 TDC-GPX START pin에만 연결한다.

`tdc_gpx_face_seq`는 direct, pending, deferred 경로마다 Shot event와 face payload를 함께 저장한다. Face header word3의 `face_id`는 이 payload에서 오며, 같은 word의 `n_faces`는 motor static sideband에서 온다. Simulation에서는 `face_index < n_faces`를 assertion으로 검사한다.

이미 발사된 레이저 사건에는 사후 `ready` backpressure를 적용할 수 없다. 현재 구조는 1-deep deferred Shot과 drop counter로 경계 충돌을 제한/관측한다. 향후 모든 Shot의 무손실 수락이 필수라면 `laser_ctrl`이 발사 전에 확인하는 `tdc_ready/fire_admit` 인터페이스를 추가해야 한다.

## 5. 런타임 geometry

motor center와 laser active half-width처럼 운용 중 바뀌는 다중 bit 값은 request/apply CDC를 유지한다. Software는 요청값을 쓴 뒤 apply 완료를 확인하고, 실제 적용 snapshot을 status로 읽는다. 단순 level sideband로 바꾸지 않는다.

## 6. 함께 수정한 결함

### 6.1 Laser echo CDC

`laser_ctrl_echo_capture`의 `xpm_cdc_single`이 `SRC_INPUT_REG=1`인데 `src_clk='0'`으로 연결돼 비동기 echo 입력이 갱신되지 않는 문제가 있었다. `SRC_INPUT_REG=0`으로 고쳐 destination synchronizer가 실제 입력을 관측하도록 했다.

### 6.2 통합 TB Face 종료 조건

`tb_tdc_gpx_full_int`가 Shot 수 0도 modulo 경계로 인정해 Face를 너무 일찍 닫을 수 있었다. `shot_count >= cols_per_face`를 먼저 만족한 뒤 modulo를 검사하도록 수정했다. 이 변경으로 AXIS 150 MHz / TDC 200 MHz 비동기 시나리오가 실제 Shot 수를 끝까지 검증한다.

### 6.3 Sign-off source/contract drift

기존 OOC flow가 오래된 `.xpr` source set을 기준으로 동작해 `tdc_gpx_cfg_image_override.vhd` 같은 현재 RTL을 누락할 수 있었다. `scripts/tdc_gpx_rtl_manifest.tcl`을 30개 RTL source의 단일 manifest로 만들고 OOC와 parent 생성 flow가 함께 사용하도록 변경했다. 따라서 프로젝트 파일의 source 목록이 오래돼도 sign-off 대상 RTL이 조용히 달라지지 않는다.

Parent contract verifier는 검사항목이 95개로 늘었는데 sign-off wrapper가 과거의 38개를 기대하고 있었다. 기대 개수를 95로 갱신하고 모든 줄이 `PASS`인지 함께 확인하도록 유지했다.

## 7. 검증 결과

| 범위 | 결과 |
|---|---|
| Laser 전체 Vivado TB | 41 passed, 0 failed |
| TDC C06 v002 전체 regression | PASS, 121 artifacts |
| `n_faces=1..5` system integration sweep | 전부 PASS |
| internal timing, AXIS/TDC 200/200 | PASS |
| internal timing, AXIS/TDC 150/200 | PASS |
| external timing, AXIS/TDC 200/200 | PASS, fire/start/stop 8/8/8 |
| external timing, AXIS/TDC 150/200 | PASS, fire/start/stop 8/8/8 |
| 5-face header/face seen mask | Rise/Fall `0x1F`, 9/9 header checks PASS |
| HTML 실제 Chrome DOM 실행 | 6개 contract self-test와 frame verdict 전부 PASS |
| Parent reference 재생성 | VALIDATE PASS, top/parent generic 22개 일치, 연결 계약 95개 PASS |
| xc7z020 OOC implementation | PASS, WNS 0.041 ns, TNS 0, WHS 0.065 ns, THS 0 |
| OOC timing/check_timing | AXIS WNS 0.506 ns, TDC WNS 0.041 ns, no-clock 0, unconstrained internal endpoint 0 |
| OOC CDC/route | Critical unsafe CDC 0, fully routed net 31,155, routing error 0 |

기본 implementation 전략은 post-route WNS `-0.084 ns`, TNS `-0.192 ns`로 5개 endpoint가 조금 부족했다. 동일 RTL/XDC에 `TIMING_EXPLORE` 전략을 적용한 최종 run은 WNS `+0.041 ns`로 닫혔다. 병목은 기존 `chip_run` drain-count 경로였고 이번 `n_faces`/Shot payload 경로가 아니므로 RTL에 별도 pipeline을 추가하지 않았다.

최종 OOC session은 `signoff_results/sessions/260723071000_ip_handoff_timing_explore_w32_a150_t200_p1111_r0011_f1100_impl`, parent 재검증 session은 `parent_ref/results/sessions/260723_manifest_validate_ps_fclk_parent_ref`이다.

## 8. 저장소 체크포인트

| IP | Commit | 내용 |
|---|---|---|
| virtual_encoder | `5384a72` | immediate encoder parameter atomic apply |
| motor_decoder | `3bbae9c` | fixed face geometry status와 applied geometry readback |
| laser_ctrl | `3cfd686` | Shot metadata valid 계약과 echo CDC 복구 |
| tdc_gpx_ctrl | 본 커밋 | motor face ownership, Shot payload, 문서/HTML/OOC closure |

## 9. HTML 대응

`C08_HDL_HTML_Alignment_260723_IP_Handoff_Contract_Simulator_v020.html`의 D00을 IP 전달 계약의 시작점으로 추가했다.

- 면 수 선택은 motor build 범위 1..5로 제한
- `HW_CONFIG[31:29]`과 reserved `MAIN_CTRL[14:12]` 표시
- Shot face ID를 event-qualified payload로 표시
- runtime 설정을 request/apply로 표시
- 사후 ready가 아닌 pre-fire admission이 필요한 이유 표시

이후 D01..D09 계산은 D00에서 확정한 동일 소유권을 사용한다.

## 10. 판정

현재 변경 방향은 합리적이며 이 범위의 sign-off gate는 통과했다. 정적값까지 무조건 handshake로 감싸 복잡도를 늘리지 않고, 실제로 변하는 값만 atomic 전달 대상으로 분류했다. 다만 모든 Shot의 무손실 수락이 시스템 요구가 되면 사후 `ready`가 아니라 레이저 발사 전 `tdc_ready/fire_admit`을 별도 기능으로 설계해야 한다.
