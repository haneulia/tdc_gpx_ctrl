# v2 테스트벤치 커버리지 및 유지보수 가이드

## 1. 문서 목적

이 문서는 `tdc_gpx_lidar_ctrl_v2`의 테스트벤치를 **실행 가능한 RTL 계약서**로
관리하기 위한 기준이다. 현재 테스트 자산은 다음 세 종류다.

| 종류 | 파일 수 | 의미 |
|---|---:|---|
| Primary self-checking TB | 48 | DUT를 구동하고 assertion 및 고유 PASS marker로 기능을 판정한다. |
| Profile wrapper | 11 | 공통 TB를 주파수, Face 수 또는 clock 관계별 top entity로 구체화한다. |
| Implementation harness | 13 | 합성/구현 timing, CDC, DRC와 자원 사용을 재현한다. 기능 PASS를 대신하지 않는다. |

테스트벤치는 RTL 이후에 작성하는 확인 코드가 아니다. 설정 의미, clock 경계,
데이터 bit 배열, 실패 복구와 외부 인터페이스를 사람이 다시 해석하지 않아도 되게
보존하는 첫 번째 유지보수 자산이다.

## 2. Sign-off에서의 우선순위

1. 버그 수정은 가능하면 **실패하는 재현 TB를 먼저 확보**한 뒤 RTL을 수정한다.
2. RTL 변경은 관련 단위 TB, 상위 통합 TB, 필요 시 Golden 비교 순서로 검증한다.
3. compile 또는 synthesis 성공만으로 기능 Sign-off를 선언하지 않는다.
4. PASS marker는 모든 assertion과 비교가 끝난 뒤 한 번만 출력한다.
5. assertion을 삭제하거나 조건을 느슨하게 해서 회귀를 통과시키지 않는다.
6. CSR/ABI 변경은 RTL, TB, CSR 문서, PS/HTML Golden을 하나의 변경으로 관리한다.
7. clock/CDC 변경은 routine 150/200, 200/150 MHz를 모두 실행하고, crossing 구조가
   바뀌면 150/150, 200/50, 50/200 MHz release profile도 추가한다.

## 3. Primary 테스트벤치 인벤토리

### 3.1 설정, CSR 및 활성화

| 테스트벤치 | 생성 목적과 핵심 커버 | 관련 RTL | 실행 | 변경 시 주의 |
|---|---|---|---|---|
| `tb_lidar_config_types_pkg.vhd` | 설정 record 기본값, pack/unpack, 기준 산식 round-trip | config/build/reference package | `run_v2_config_pkg.ps1` | field 추가 시 기본·최소·최대값을 추가한다. |
| `tb_lidar_csr_map_pkg.vhd` | 32 CTL/32 STAT bit map과 typed record 변환 | `lidar_csr_map_pkg` | `run_v2_unified_csr.ps1` | 주소/bit 문서와 벡터를 동시에 바꾼다. |
| `tb_lidar_commit_calculator.vhd` | 순차 validation, 시간 변환, 양자화, 오류 코드 | `lidar_commit_calculator` | `run_v2_commit_calculator.ps1` | 물리 단위별 경계/초과 벡터를 유지한다. |
| `tb_lidar_config_gateway.vhd` | PREPARE/ACTIVATE/RELEASE 및 abort | `lidar_config_gateway` | `run_v2_config_manager.ps1` | ACK 순서와 candidate 보존을 약화하지 않는다. |
| `tb_lidar_config_subsystem.vhd` | 양 domain atomic COMMIT, timeout, rollback, version | config manager/gateway 계층 | `run_v2_config_manager.ps1` | 실패 COMMIT에서 Active 불변을 확인한다. |
| `tb_lidar_csr_bank.vhd` | AXI AW/W 독립 도착, backpressure, W1S/W1C, portal, IRQ | `lidar_csr_bank` | `run_v2_unified_csr.ps1` | 새 bit마다 read/write/illegal/clear를 추가한다. |
| `tb_lidar_csr_config_subsystem.vhd` | AXI부터 Active 적용, GPX image snapshot, MTimer 자동 계산 | CSR/config/GPX activation 계층 | `run_v2_unified_csr.ps1` | Shadow 동시 수정과 실패 COMMIT을 함께 본다. |
| `tb_lidar_gpx_config_activation.vhd` | 모든 present Chip programming ACK 후 TDC enable | `lidar_gpx_config_activation` | `run_v2_unified_csr.ps1` | 조기 ACK와 fault activation을 유지한다. |
| `tb_lidar_processing_activation_barrier.vhd` | Echo/VDMA ready, busy, version을 묶는 barrier | processing activation barrier | `run_v2_k04_integration.ps1` | 새 의존 block에는 negative test가 필요하다. |
| `tb_lidar_system_command_cdc.vhd` | CLEAR/RESET one-shot CDC, busy/모호 명령 거부 | `lidar_system_command_cdc` | `run_v2_k03_integration.ps1` | destination pulse 폭과 reset 복구를 확인한다. |
| `tb_lidar_vdma_profile_cdc.vhd` | Processing→CSR 49-bit VDMA profile 원자성, 지연 ACK, stale-high ACK 거부, 다음 snapshot 분리 | `lidar_vdma_profile_cdc` | `run_v2_k03_integration.ps1` | payload field를 추가하면 모든 snapshot 비교를 함께 늘리고 150/200, 200/150 MHz를 모두 유지한다. |
| `tb_lidar_status_irq_integration.vhd` | native snapshot, CTL23/24, 물리 GPX read, 유지보수 pause-safe 2-edge 선행 조건, 상태 선택→응답 패킹 등록 경계, 모든 TDC lane fault, 같은-clock fault/clear, IRQ/CLEAR/W1C | CSR/status/command CDC | `run_v2_k08_status_irq.ps1` | GPX read는 pause 이전 safe 값으로 시작하면 안 된다. 상태 index 선택과 응답 word 패킹을 다시 한 조합 경로로 합치지 말고 owner clear는 아래 K1-1 전용 회귀와 함께 유지한다. |

### 3.2 모터, Face, Shot 및 레이저

| 테스트벤치 | 생성 목적과 핵심 커버 | 관련 RTL | 실행 | 변경 시 주의 |
|---|---|---|---|---|
| `tb_motor_position_core.vhd` | CW/CCW, x1/x2/x4, virtual x4, Z/wrap, latency, LO/HI별 Z 폭 제한 사전 계산 | `motor_position_core` | `run_v2_motor_position.ps1` | 동일 A/B/Z 벡터로 양 방향을 비교한다. 선택된 `ticks_next`에서 Z 폭 비교기를 다시 만들지 않는다. |
| `tb_face_tracker.vhd` | Face 1~5, lower/upper, wrap, 방향, overlap | `face_tracker` | `run_v2_face_tracker.ps1` | 경계의 inclusive 의미를 보존한다. |
| `tb_lidar_operation_subsystem.vhd` | RUN/STOP/ARM/DISARM, permit, mode, fail-safe reset, CSR 동기화 뒤 scheduler 상태 재도출 | operation manager/subsystem | `run_v2_operation.ps1` | Physical/Simulation 상호배제와 파생 scheduler 상태를 함께 확인하고 같은 source FF를 중복 synchronizer로 fan-out하지 않는다. |
| `tb_shot_scheduler.vhd` | 광학각 간격, 방향/wrap, 등록된 GPX admission(후보점 1클럭 전 안정), busy 억제, late-angle 금지 | `shot_scheduler` | `run_v2_shot_scheduler.ps1` | exact/overshoot 위치와 GPX busy 해제 후 off-grid 재시도 금지를 모두 확인한다. |
| `tb_laser_executor.vhd` | fire, fire_done, start/stop TDC, timeout, sim/physical 배제 | `laser_executor` | `run_v2_laser_executor.ps1` | fire_done 경로는 cycle 정확도를 유지한다. |
| `tb_v2_processing_control_chain.vhd` | operation→Face→scheduler B0~B2 직접 event 연결 | 세 Processing core | `run_v2_shot_scheduler.ps1` | AXIS monitor가 제어를 막지 않아야 한다. |
| `tb_v2_laser_control_chain.vhd` | operation→Face→scheduler→executor B0~B3 | 네 Processing core | `run_v2_laser_executor.ps1` | 단위 latency 변경을 end-to-end에 반영한다. |
| `tb_lidar_processing_subsystem.vhd` | 모터부터 laser/TDC 제어 및 monitor까지 통합 | `lidar_processing_subsystem` | `run_v2_processing_subsystem.ps1` | source 상호배제와 monitor 비의존성을 본다. |

### 3.3 Echo 및 GPX acquisition

| 테스트벤치 | 생성 목적과 핵심 커버 | 관련 RTL | 실행 | 변경 시 주의 |
|---|---|---|---|---|
| `tb_lidar_echo_subsystem.vhd` | physical/simulation/disabled, 16/32 lane, Return 1~7, delay, 10-bank 등록형 Shot 시작/채널 초기화, 등록된 any-event와 Shot snapshot finalize | `lidar_echo_subsystem` | `run_v2_echo.ps1` | physical STOP에는 CSR/AXIS/진단 파이프라인 지연을 넣지 않는다. 등록된 Shot 시작과 같은 cycle의 첫 Echo도 수락하며, 단계 분리 뒤에도 snapshot cycle과 32채널 count가 같아야 한다. |
| `tb_lidar_gpx_bus_engine.vhd` | typed wrapper와 v1 PHY의 pin/cycle/28-bit 등가 | GPX bus engine/legacy PHY | `run_v2_gpx_bus.ps1` | 물리 FSM 변경은 oracle 비교가 먼저다. |
| `tb_lidar_gpx_event_gateway.vhd` | SYNC/ASYNC command/result CDC, reset/stall과 K1-4 여섯 clock profile | GPX event gateways | `run_v2_gpx_event_gateway.ps1`, `run_v2_k14_signoff.ps1` | SYNC는 같은 주파수가 아니라 동일한 물리 clock을 사용한다. ASYNC 150/200, 200/150, 200/50, 50/200, 150/100 MHz를 보존한다. |
| `tb_lidar_stream_gateway_reset.vhd` | 200 MHz Source→150 MHz Destination 비동기 FIFO에서 Source/Destination Reset 전 보류 payload 폐기, Reset 중 ready/valid 차단, 복구 후 첫 새 payload 일치 | `lidar_stream_gateway` | `run_v2_stream_gateway_reset.ps1` | Destination Reset은 4단 XPM으로 Source에 전달된다. Reset pulse를 줄이거나 FIFO Reset 소유 clock을 바꾸면 stale payload와 Reset busy를 함께 재검증한다. |
| `tb_lidar_gpx_event_merge.vhd` | multi-lane ordered merge, terminal, backpressure | `lidar_gpx_event_merge` | `run_v2_gpx_acquisition_coordinator.ps1` | starvation과 terminal 누락을 검사한다. |
| `tb_lidar_gpx_acquisition_lane.vhd` | 한 Chip IFIFO1/2 drain, timeout, cap/purge | acquisition lane/chip run | `run_v2_gpx_acquisition_lane.ps1` | EF 정상 종료와 fault 종료를 분리한다. |
| `tb_lidar_gpx_acquisition_coordinator.vhd` | Shot fanout, runtime mask, terminal merge, 등록된 lane Shot/config-ready, config와 부팅 중 외부 GPX Register Read 보존 | acquisition coordinator | `run_v2_gpx_acquisition_coordinator.ps1`, `run_v2_k14_signoff.ps1` | inactive lane 무동작, ready의 1클럭 보수적 상승, Run 해제 즉시 stale-ready 차단과 재무장, 전역/강제 리셋 중 `ready=0`, held-valid 단일 수락, 요청 버퍼와 응답 stall 안정성을 함께 확인한다. |
| `tb_lidar_gpx_acquisition_subsystem.vhd` | external pin부터 B5, 32 physical STOP lane, fault/CDC | acquisition subsystem | `run_v2_gpx_acquisition_subsystem.ps1` | routine 두 clock 관계를 모두 유지한다. |
| `tb_lidar_face_close_owner.vhd` | trailing/all-hole Face close, overlap, exactly-once | `lidar_face_close_owner` | `run_v2_gpx_b5_b8_subsystem.ps1` | 조기 close와 누락 close를 함께 본다. |

#### 3.3.1 K1-1 legacy TDC 진단 소유자 회귀

| 검증 자산 | 생성 목적과 핵심 커버 | 관련 RTL | 실행 | 변경 시 주의 |
|---|---|---|---|---|
| `tb_tdc_gpx_chip_init_cfg_owner.vhd` | prefetch→prepare→issue register 순서/값, init/config coalesced set/clear, busy 중 clear가 초기화를 취소하지 않음, clear와 새 사건 동시 발생 | `tdc_gpx_chip_init` | `run_v2_gpx_clear_status.ps1` | 다음 index prefetch가 이전 register 값을 재사용하지 않는지와 pending 초기화/진단 sticky 분리를 함께 확인한다. |
| `tb_tdc_gpx_request_loss.vhd` | Register request overflow set/clear와 같은 clock 새 overflow 우선 | `tdc_gpx_chip_reg`, `tdc_gpx_cmd_arb` | `run_v2_gpx_clear_status.ps1` | queue/FSM은 clear 대상이 아니다. |
| `v2_gpx_clear_status_fault_injection.tcl` | response mismatch, raw drop, drain cap, command collision, bus fatal을 이름 있는 사건점에 개별 주입 | `tdc_gpx_chip_ctrl` | `run_v2_gpx_clear_status.ps1` | 내부 상태를 임의로 바꾸지 말고 진단 event predicate만 force한다. |
| `tb_tdc_gpx_chip_ctrl.vhd` | init/run/drain/backpressure/quarantine, range timeout one-shot, `0=disable`, 전체 진단 clear 무회귀 | Chip controller와 bus PHY | `run_v2_gpx_clear_status.ps1` | range countdown은 drain margin 전에 발화하지 않고 비영 예산당 한 번만 발화해야 한다. bus-fatal 기능 격리와 진단 이력도 별도 확인한다. |

전용 스크립트는 위 네 층을 모두 실행하고
`LIDAR_V2_K11_GPX_CLEAR_STATUS_REGRESSION_PASS`를 최종 표식으로 남긴다.

### 3.4 Hit, Cell 및 Frame

| 테스트벤치 | 생성 목적과 핵심 커버 | 관련 RTL | 실행 | 변경 시 주의 |
|---|---|---|---|---|
| `tb_lidar_gpx_hit_decoder.vhd` | I-Mode raw28→Hit17, bit16, topology/fault | `lidar_gpx_hit_decoder` | `run_v2_gpx_hit_decoder.ps1` | datasheet bit map과 벡터를 같이 관리한다. |
| `tb_lidar_gpx_cell_collector.vhd` | Cell grouping, Return 1~7, visible filter, overflow | `lidar_gpx_cell_collector` | `run_v2_gpx_cell_collector.ps1` | 의도적 필터와 실제 손실을 구분한다. |
| `tb_lidar_gpx_hit_cell_pipeline.vhd` | decoder→collector context/ready/Return 연결 | B6/B7 두 블록 | `run_v2_gpx_cell_collector.ps1` | pipeline 추가 시 latency와 identity를 본다. |
| `tb_lidar_gpx_runtime_slope_masks.vhd` | one/four-Chip dual-edge와 runtime mask 부분집합 | decoder/collector | `run_v2_gpx_frame_lane_assembler.ps1` | inactive slope가 출력되지 않아야 한다. |
| `tb_lidar_gpx_frame_lane_assembler.vhd` | Rise/Fall channel order, gap, topology, lane stall, Face-close context/geometry 단계 분리 | frame lane assembler | `run_v2_gpx_frame_lane_assembler.ps1` | canonical B8 순서와 all-hole/trailing-gap/fault 결과를 단계 분리 전후 동일하게 유지한다. |
| `tb_lidar_gpx_b5_b8_subsystem.vhd` | B5 raw부터 B8 Cell까지 identity와 Face close | B5~B8 subsystem | `run_v2_gpx_b5_b8_subsystem.ps1` | dedicated/all-dual 및 두 clock 관계를 본다. |

### 3.5 PACKED17, AXIS, VDMA 및 Golden

| 테스트벤치 | 생성 목적과 핵심 커버 | 관련 RTL | 실행 | 변경 시 주의 |
|---|---|---|---|---|
| `tb_lidar_gpx_cell_word_serializer.vhd` | Cell→PACKED17 word, metadata, stall | Cell serializer | `run_v2_gpx_cell_word_serializer.ps1` | PS/HTML ABI와 동시에 변경한다. |
| `tb_lidar_gpx_vdma_lane_formatter.vhd` | J2 legacy prefix formatter 기준선 | legacy lane formatter | `run_v2_gpx_vdma_lane_formatter.ps1` | 최종 ABI로 오해하지 않는다. |
| `tb_lidar_gpx_shot_line_builder.vhd` | 16-byte Shot Metadata와 Cell 연속 line | Shot line builder | `run_v2_gpx_shot_line_builder.ps1` | byte offset을 PS decoder와 맞춘다. |
| `tb_lidar_gpx_hole_line_expander.vhd` | leading/interior/trailing/all-Hole column | Hole line expander | `run_v2_gpx_hole_line_expander.ps1` | Hole과 데이터 손실 fault를 구분한다. |
| `tb_lidar_gpx_axis_word_packer.vhd` | 동일 byte열의 32/64/128 pack, TKEEP/TLAST | AXIS word packer | `run_v2_gpx_axis_word_packer.ps1` | 폭별 별도 ABI를 만들지 않는다. |
| `tb_lidar_gpx_vdma_profile_manager.vhd` | HSIZE/VSIZE/STRIDE/Footer line 및 safe activation | VDMA profile manager | `run_v2_gpx_face_footer.ps1` | 최소/최대 geometry를 모두 본다. |
| `tb_lidar_gpx_face_footer_builder.vhd` | 32-byte Face Footer field와 폭별 전송 | Footer builder/profile manager | `run_v2_gpx_face_footer.ps1` | version과 Viewer decoder를 같이 바꾼다. |
| `tb_lidar_gpx_frame_close_fork.vhd` | Rise/Fall Footer fork와 독립 stall | Frame close fork | `run_v2_gpx_face_footer.ps1` | disabled lane 교착을 막는다. |
| `tb_lidar_gpx_axis_output_subsystem.vhd` | dual-lane Shot/Hole/Footer/packer 통합, 등록된 Cell 1단계와 Shot-Line 1단계 dispatch, K1-3 최대 STOP/Return topology geometry telemetry | AXIS output subsystem | `run_v2_k06_axis_dual_lane.ps1`, `run_v2_k13_operating_matrix.ps1` | 한 lane stall의 격리와 세 topology의 슬롯/HSIZE/VSIZE/STRIDE/Beat 수를 함께 보존한다. 두 dispatch 단계의 고정 `+2 Processing clocks`를 K13 기준값에 포함하되 Cell/Word 순서와 정상상태 1개/clock 처리율은 유지한다. |
| `tb_lidar_gpx_ddr_golden.vhd` | STRIDE-aware DDR 모든 word와 HTML/PS Golden 비교 | 전체 data/output chain | `run_v2_gpx_ddr_golden.ps1` | capture와 외부 비교까지 성공해야 한다. |

### 3.6 Public Top 통합

| 테스트벤치 | 생성 목적과 핵심 커버 | 관련 RTL | 실행 | 변경 시 주의 |
|---|---|---|---|---|
| `tb_tdc_gpx_lidar_ctrl_v2_k03.vhd` | Top config/VDMA activation 및 command CDC | v2 Top + K0-3 계층 | `run_v2_k03_integration.ps1` | public port/generic 누락을 확인한다. |
| `tb_tdc_gpx_lidar_ctrl_v2_k04.vhd` | Top Processing/Echo, physical/simulation source | v2 Top + K0-4 계층 | `run_v2_k04_integration.ps1` | simulation이 physical 출력을 내지 않게 한다. |
| `tb_tdc_gpx_lidar_ctrl_v2_k05.vhd` | K0-5 B5~B8, K0-6 AXIS 출력, K1-2 Reg7 3계층, K1-3 운용 telemetry와 K1-4 릴리스 clock 사건 보존 | v2 Top + config/physical GPX/data/output 전체 계층 | `run_v2_k05_integration.ps1`, `run_v2_k06_axis_integration.ps1`, `run_v2_k13_operating_matrix.ps1`, `run_v2_k14_signoff.ps1` | 첫 완성 Frame은 Beat/Line/SOF/Footer/비결측을 정확히 비교한다. 빠른 가상 모터로 뒤이은 Face가 겹치면 추가 출력은 완전한 결측 Frame만 허용하며 부분 Frame이나 데이터 중복은 금지한다. K1-4에서는 동일 물리 clock 150/150과 ASYNC 200/50, 50/200, 150/100 MHz도 실행한다. |

## 4. Profile wrapper 인벤토리

Profile wrapper는 assertion을 소유하지 않는다. 공통 TB generic을 고정하여 회귀
top 이름을 제공한다.

| Wrapper 파일 | 펼치는 조합 | 실행 스크립트 |
|---|---|---|
| `tb_lidar_commit_calculator_profiles.vhd` | Processing 150/200 MHz | `run_v2_commit_calculator.ps1` |
| `tb_lidar_config_subsystem_profiles.vhd` | Processing/TDC 150/200, 200/150 | `run_v2_config_manager.ps1` |
| `tb_lidar_csr_config_profiles.vhd` | Processing/TDC 150/200, 200/150 | `run_v2_unified_csr.ps1` |
| `tb_motor_position_core_profiles.vhd` | Processing 150/200 MHz | `run_v2_motor_position.ps1` |
| `tb_face_tracker_profiles.vhd` | Processing 150/200 MHz × Face 1~5 | `run_v2_face_tracker.ps1` |
| `tb_lidar_operation_profiles.vhd` | Processing 50/150/200 MHz | `run_v2_operation.ps1` |
| `tb_shot_scheduler_profiles.vhd` | Processing 150/200 MHz | `run_v2_shot_scheduler.ps1` |
| `tb_laser_executor_profiles.vhd` | Processing 150/200 MHz | `run_v2_laser_executor.ps1` |
| `tb_v2_processing_control_chain_profiles.vhd` | Processing 150/200 MHz | `run_v2_shot_scheduler.ps1` |
| `tb_v2_laser_control_chain_profiles.vhd` | Processing 150/200 MHz | `run_v2_laser_executor.ps1` |
| `tb_lidar_processing_subsystem_profiles.vhd` | Processing/TDC 150/200, 200/150 | `run_v2_processing_subsystem.ps1` |

## 5. Implementation harness 인벤토리

| 하네스 | 구현 대상 | 실행 스크립트 |
|---|---|---|
| `lidar_echo_subsystem_impl.vhd` | Echo physical/simulation generate | `run_v2_echo.ps1` |
| `lidar_gpx_bus_engine_impl.vhd` | GPX physical bus timing | `run_v2_gpx_bus.ps1` |
| `lidar_gpx_event_gateway_impl.vhd` | SYNC/ASYNC CDC gateway | `run_v2_gpx_event_gateway.ps1` |
| `lidar_gpx_acquisition_lane_impl.vhd` | 단일 acquisition lane | `run_v2_gpx_acquisition_lane.ps1` |
| `lidar_gpx_acquisition_coordinator_impl.vhd` | multi-Chip coordinator | `run_v2_gpx_acquisition_coordinator.ps1` |
| `lidar_gpx_acquisition_subsystem_impl.vhd` | acquisition 전체 | `run_v2_gpx_acquisition_subsystem.ps1` |
| `lidar_gpx_hit_decoder_impl.vhd` | B6 decoder | `run_v2_gpx_hit_decoder.ps1` |
| `lidar_gpx_cell_collector_impl.vhd` | B7 collector | `run_v2_gpx_cell_collector.ps1` |
| `lidar_gpx_hit_cell_pipeline_impl.vhd` | B6/B7 연결 | `run_v2_gpx_cell_collector.ps1` |
| `lidar_gpx_frame_lane_assembler_impl.vhd` | B8 assembler | `run_v2_gpx_frame_lane_assembler.ps1` |
| `lidar_gpx_b5_b8_subsystem_impl.vhd` | B5~B8 전체 | `run_v2_gpx_b5_b8_subsystem.ps1` |
| `lidar_gpx_face_footer_builder_impl.vhd` | VDMA profile/Footer | `run_v2_gpx_face_footer.ps1` |
| `lidar_processing_subsystem_impl.vhd` | Processing 전체 | `run_v2_processing_subsystem.ps1` |

## 6. 권장 수행 순서

다음 순서는 앞 단계의 출력 계약을 뒤 단계의 입력 oracle로 사용한다.

1. 설정 package와 CSR: `run_v2_config_pkg.ps1`, `run_v2_unified_csr.ps1`
2. atomic configuration: `run_v2_commit_calculator.ps1`, `run_v2_config_manager.ps1`
3. Processing: motor, Face, operation, scheduler, executor, subsystem 순서
4. Echo: `run_v2_echo.ps1`
5. GPX physical path: bus, gateway, lane, coordinator, acquisition 순서
6. Data: decoder, collector, frame assembler, B5~B8 순서
7. Output: serializer, Shot/Hole, packer, Footer, dual lane 순서
8. Golden: `run_v2_gpx_ddr_golden.ps1`와 PS/HTML byte 비교
9. Public Top: K03, K04, K05/K06, K08 순서
10. K1-4 최종 Gate: `run_v2_k14_signoff.ps1`
11. 비동기 stream Reset 폐기 계약: `run_v2_stream_gateway_reset.ps1`
12. IP package와 OOC implementation

예시:

```powershell
powershell -ExecutionPolicy Bypass -File system_integration/v2/scripts/run_v2_unified_csr.ps1
powershell -ExecutionPolicy Bypass -File system_integration/v2/scripts/run_v2_gpx_b5_b8_subsystem.ps1
powershell -ExecutionPolicy Bypass -File system_integration/v2/scripts/run_v2_gpx_ddr_golden.ps1
```

문서 누락은 다음 정적 검사로 확인한다.

```powershell
powershell -ExecutionPolicy Bypass -File system_integration/v2/scripts/check_v2_testbench_docs.ps1
```

## 7. 현재 커버리지 공백과 예정 보완

| 우선순위 | 공백 | 현재 상태 | 필요한 회귀 |
|---|---|---|---|
| K1-4 Gate | 통합 RTL/IP release Gate | 입력 snapshot 기반 11개 Gate와 package OOC를 한 실행으로 판정 | routine 2개×폭 3개 구현, 직접 CDC 6개, Golden과 package 검사를 같은 runner로 유지한다. 최종 통과 세션은 K1-4 결과 문서에서 관리한다. |
| Closed K1-3 | K1 RTL/HTML 전체 operating matrix | 40개 RTL profile과 실행 가능한 HTML/Golden 자동 비교 완료 | Return 1~7, 32/64/128-bit, 목표거리, topology와 두 routine clock 관계를 유지 회귀 |
| Closed K1-2 | Reg7 Shadow/Active/Physical 3계층 단일 시나리오 | Top 연속 시나리오로 완료 | 잘못된 staging MTimer→진행 중 Shadow 수정→두 번의 성공 COMMIT→두 Chip 물리 readback→`0x33` 실패 rollback→복구 COMMIT |
| P1 | 같은 sticky가 high인 동안 반복된 사건 | IRQ bit만으로 횟수 구분 불가 | 관련 진단 count 증가/clear/new-event-wins 검증 |
| P2 | 실제 AXI VDMA, HP port, DDR cache coherency | RTL/Golden 모델 범위 밖 | Stage L parent/보드에서 DMA API와 cache invalidate 포함 측정 |
| P2 | 실제 PCB GPX timing, LVDS와 laser safety | simulation/implementation만으로 불충분 | laser-disabled capture 후 제한된 physical laser/GPX 보드 시험 |

이 표의 공백은 K1 RTL/IP PASS를 무효화하지 않는다. 다만 해당 범위를 검증했다고
확대 해석해서는 안 된다. 실제 VDMA/cache와 물리 신호 항목은 Stage L 보드
Sign-off에서 닫고, 반복 sticky 사건 횟수는 운용 진단 요구가 확정될 때 counter
coverage와 함께 보강한다.

legacy TDC sticky의 `CLEAR_STATUS` 통일은 K1-1에서 완료했다. lane fault
`[2],[3],[4],[10],[11],[12]`, clear, 같은 clock 새 fault 우선, 통합 IRQ source와
W1C 순서를 owner-level 회귀와 K08 회귀로 각각 검증했다.

K1-2 Reg7 시나리오는 `tb_tdc_gpx_lidar_ctrl_v2_k05.vhd` 안에서 다음 순서를
중간 reset 없이 한 번에 수행한다.

1. 첫 성공 COMMIT 전 `ACTIVE_VALID=0`과 Active view `0` 확인;
2. A 설정 COMMIT이 BUSY인 동안 B Shadow와 staging Reg7 기록;
3. A Active source, 자동 계산 MTimer, 두 Chip 실제 Reg7, `SHADOW_DIRTY=1` 확인;
4. B COMMIT 뒤 staging 수동값과 Active/물리 자동 계산값의 의도적 차이 확인;
5. MTimer 상한 초과 요청의 `0x33`, 물리 write 0회, Active version/image 보존 확인;
6. 유효한 B로 복구 COMMIT하고 acquisition/AXIS 회귀 계속 수행.

따라서 K1-2 marker는 CSR portal만 흉내 낸 단위 테스트가 아니라 실제 Top의
28-bit GPX pin write/read 모델을 통과했다는 의미다. 이 순서 중 하나를 분리하거나
중간 reset으로 상태를 지우면 COMMIT snapshot 격리 커버가 사라진다.

K1-3은 `tb_tdc_gpx_lidar_ctrl_v2_k05.vhd`에서 Shot 승인, TDC 측정 시작 신호(T0),
STOP_TDC와 첫 Shot Line의 마지막 handshake를 계측한다. 별도의 AXIS output TB는
전용 Rise/Fall, 한 chip 양 edge, 4 chip 양 edge의 슬롯 수와 geometry를 계측한다.
두 TB의 구조화된 marker를 HTML 실행 모델 및 체크인된 Golden JSON과 비교하므로,
marker 이름이나 사건 기준을 바꿀 때는 검증기와 K1-3 계약 문서를 같은 커밋에서
갱신해야 한다. Cell dispatch와 Shot-Line dispatch는 각각 한 Processing clock의
등록 경계이며, 이 고정 `+2 Processing clocks`는 HTML 기준값에도 포함한다. 빠른
가상 모터 때문에 첫 Frame 뒤에 다음 Face가 겹칠 수 있으므로 첫 Frame은 전체
Beat/Line/SOF/Footer를 정확히 비교하고, 이후 출력은 완전한 결측 Frame만 허용한다.
부분 Footer, 부분 Line 또는 중복 데이터는 계속 실패다. 실제 VDMA/HP/cache와 장기
backpressure는 이 PASS에 포함되지 않는다.

K1-4는 `run_v2_k14_signoff.ps1` 한 곳에서 테스트벤치 문서, GPX 유지보수,
Release Top 기능, 공개 Top 구현, coordinator, 직접 CDC, AXIS, 상태/IRQ,
RTL/HTML, DDR/PS/Ethernet, IP package를 순서대로 실행한다. Processing→TDC와
TDC→Processing의 두 비동기 경계 때문에 K13 시간 telemetry는 nominal baseline의
`-2..+2 Processing clocks`를 허용한다. `lidar_gpx_shot_gateway`의 TDC 출력
스키드는 Shot 지연을 1 TDC clock 늘리지만 1 Shot/clock 처리량과 원자성은 유지한다.
`tdc_gpx_chip_ctrl`의 등록형 Register 응답도 유지보수 응답만 1 TDC clock 늘리며
STOP/IFIFO 획득 경로를 변경하지 않는다. 상태 응답은 진단 index 선택과 word 패킹을
서로 다른 clock 단계로 나누고, Echo 진단은 Shot 시작과 채널 초기화를 10개 로컬
뱅크로 등록한다. 두 변경 모두 관측/진단 경로의 배선 집중을 줄이기 위한 것이며,
외부 LVDS STOP과 물리 fire_done→TDC 측정 시작 신호(T0) 경로에는 지연을 추가하지 않는다.

## 8. 테스트벤치 유지보수 체크리스트

- 파일 상단의 `테스트 자산 목적`, `핵심 검증 계약`, `관련 RTL`, `실행 회귀`,
  `유지보수 주의` 한글 주석을 최신 상태로 유지한다.
- 새 TB는 고유한 `LIDAR_V2_*_PASS` marker를 assertion 완료 뒤 출력한다.
- 정상 동작뿐 아니라 invalid input, timeout, reset, abort, backpressure를 포함한다.
- CDC TB는 양쪽 clock 관계와 destination reset 중 request를 확인한다.
- AXIS TB는 `TVALID=1, TREADY=0` 동안 payload/TKEEP/TLAST 안정성을 확인한다.
- sticky/count TB는 set, read, clear, 같은 cycle 새 사건 우선순위를 구분한다.
- profile wrapper에는 기능 assertion을 복제하지 않는다.
- implementation harness 결과를 기능 TB PASS 대신 사용하지 않는다.
- 생성 로그와 wave는 실패 재현 또는 Sign-off 증거만 보존한다.
- TB 변경도 RTL 변경과 같은 수준으로 code review하고 Git checkpoint에 포함한다.

## 9. Coverage 판정 용어

| 용어 | 의미 |
|---|---|
| `Unit covered` | 블록 입출력 계약과 주요 fault를 self-checking TB가 검증한다. |
| `Integration covered` | 인접 블록 사이 identity, handshake와 reset을 검증한다. |
| `Golden covered` | 독립 모델과 byte/word 단위 결과가 일치한다. |
| `Implementation covered` | 합성/배치배선/CDC/DRC 증거가 있다. 기능 검증과 별개다. |
| `Board covered` | 실제 pin, 외부 Chip, DMA/cache와 물리 시간 측정 증거가 있다. |

문서에서 단순히 `PASS`라고 쓰지 말고 어느 수준의 PASS인지 함께 기록한다.
