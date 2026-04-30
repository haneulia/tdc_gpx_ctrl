# C02 Chip Acquisition - Sequential Logic Rule Review v001

## 반영 추적

- 반영 문서: `Doc/cluster_analysis/C02_Chip_Acquisition/C02_Chip_Acquisition_260430202746_Sequential_Logic_Rule_Fix_Result_v001.md`
- 반영 시간: `2026-04-30 20:27:46 +09:00`
- 코드 반영 위치:
  - `tdc_gpx_face_seq.vhd`: `p_frame_done_both`, `p_face_closing`, `o_face_closing`, `s_shot_start_gated`
  - `tdc_gpx_face_assembler.vhd`: `s_can_produce`, `gen_in_tready`
  - `tdc_gpx_chip_ctrl.vhd`: `o_s_axis_tready`, `s_bus_rsp_fire`, bus response routing 주석 예외 기록
- 검증 결과: `tb_tdc_gpx_chip_ctrl`, `tb_tdc_gpx_face_seq`, `tb_tdc_gpx_downstream`, `tb_tdc_gpx_mask_sweep` PASS

- 문서 버전: `v001`
- 작성일: `2026-04-30`
- 최종 수정 시간: `2026-04-30 20:12:34 +09:00`
- 적용 규칙: `Doc/cluster_analysis/cluster_analysis_260430201013_operating_protocol_v009.md`
- 점검 기준: 순차논리 기본, 조합논리 최대 2-depth 제한, module boundary는 FF/register로 닫기
- 점검 범위: production RTL `tdc_gpx_*.vhd`, 특히 C02 관련 `chip_ctrl`, `chip_run`, `stop_cfg_decode`, `config_ctrl`, `face_seq`, `top`, 그리고 C02 후단 handshake에 영향을 주는 `face_assembler`

## 1. 점검 요약

기능 검증 PASS와 별개로, 새 v009 코딩 규칙 기준에서는 즉시성 때문에 남겨둔 조합 handshake/closing path가 review finding으로 분류된다.

| ID | 모듈 | 판단 | 우선순위 |
|---|---|---|---|
| R-C02-SEQ-01 | `tdc_gpx_face_seq` | face closing/shot gating 조합망이 2-depth를 초과하고 output boundary를 직접 구동 | P2 |
| R-C02-SEQ-02 | `tdc_gpx_face_assembler` | output FIFO ready에서 input FIFO ready까지 이어지는 조합 ready path가 2-depth 초과 후보 | P2 |
| R-C02-SEQ-03 | `tdc_gpx_chip_ctrl` | bus response ready/fire/routing 조합 chain이 sub-FSM 입력으로 직접 전달 | P2 |
| EX-C02-SEQ-01 | `tdc_gpx_stop_cfg_decode` | cfg image bit override는 pack/unpack 성격의 허용 예외 | Accept |
| EX-C02-SEQ-02 | `tdc_gpx_output_stage`, `tdc_gpx_face_assembler` | XPM FIFO reset qualify는 IP reset 경계 예외. 단, 예외 문서화 필요 | Accept with record |

## 2. Finding 상세

### R-C02-SEQ-01: face closing / shot gating 조합망

근거:

- `tdc_gpx_face_seq.vhd:413` ~ `tdc_gpx_face_seq.vhd:424`
- `tdc_gpx_face_seq.vhd:435`
- `tdc_gpx_face_seq.vhd:581`
- `tdc_gpx_face_seq.vhd:633` ~ `tdc_gpx_face_seq.vhd:651`

현재 구조:

```text
s_frame_done_both
  = (rise_done or frame_done or abort_rise)
    and (fall_done or frame_fall_done or abort_fall)

s_all_shots_fired
  = face_shot_cnt >= cols_per_face and cols_per_face != 0

s_face_closing
  = s_all_shots_fired
    or abort
    or rise_done/frame_done/hdr_draining
    or fall_done/frame_fall_done/hdr_fall_draining

s_shot_start_gated
  = s_shot_pending_r when s_face_closing=0 and stop/reset/abort/quiesce=0 else 0
```

판단:

- `s_face_closing`은 `s_all_shots_fired`라는 선행 조합 결과를 다시 사용하고, 여러 OR group을 합친 뒤 shot deferral과 shot gating에 직접 쓰인다.
- `o_face_closing`도 `s_face_closing`을 register 없이 직접 출력한다.
- 새 규칙 기준으로 source register와 destination register 사이 2-depth를 초과하는 조합망이며, module boundary 출력도 register로 닫히지 않는다.

권장 보완:

1. `s_frame_done_both_r`, `s_all_shots_fired_r`, `s_face_closing_r`를 별도 sequential process에서 생성한다.
2. `o_face_closing`은 `s_face_closing_r`로 구동한다.
3. `s_shot_start_gated`와 `o_face_start_gated`는 registered closing/abort qualify를 기준으로 재정의한다.
4. 변경 시 shot_start gating latency가 최대 1 clk 증가하므로, `shot_raw_pulse`, `shot_deferred`, `frame_done_both`와의 상호작용을 TB로 재검증한다.

Latency / Throughput / Pipeline / II 영향:

| 항목 | 예상 영향 |
|---|---|
| Latency | face closing 반영이 최대 1 clk 늦어질 수 있음 |
| Throughput | 정상 shot II는 유지 가능하나, frame close 직전 shot accept 조건 재검증 필요 |
| Pipeline | closing decision stage 1개 추가 |
| II | steady-state shot II에는 영향 없도록 설계해야 함 |

### R-C02-SEQ-02: face_assembler input ready 조합 path

근거:

- `tdc_gpx_face_assembler.vhd:472` ~ `tdc_gpx_face_assembler.vhd:486`
- `tdc_gpx_face_assembler.vhd:359`
- `tdc_gpx_face_assembler.vhd:395`

현재 구조:

```text
s_pipe_tready
  -> s_can_produce
  -> p_in_tready
  -> s_in_tready(chip)
  -> input FIFO m_axis_tready
```

판단:

- 코드 주석상 `s_pipe_tready`는 output skid/FIFO에서 오는 registered ready로 간주되어 있다.
- 그러나 새 규칙 기준에서는 `s_can_produce` 조합 계산 뒤 `p_in_tready` 조합 process에서 state/blank/current-chip decode와 결합되어 input FIFO ready로 직접 들어간다.
- ready path는 handshake boundary에 해당하므로, 순차논리 기본 규칙상 register로 닫는 것이 원칙이다.

권장 보완:

1. `s_can_produce_r` 또는 `s_in_tready_r`를 추가해 input FIFO ready 경계를 register로 닫는다.
2. output FIFO/pipe 여유를 1 clk registered-ready latency까지 흡수할 수 있는지 확인한다.
3. blank row, chip switch, row completion edge에서 stale ready가 1 clk 남아 잘못된 chip beat를 accept하지 않는지 negative test를 추가한다.

Latency / Throughput / Pipeline / II 영향:

| 항목 | 예상 영향 |
|---|---|
| Latency | input FIFO pop이 최대 1 clk 늦어질 수 있음 |
| Throughput | output pipe가 충분하면 steady-state II=1 유지 가능 |
| Pipeline | ready decision stage 1개 추가 |
| II | chip 전환/blank 구간의 II를 별도 측정 필요 |

### R-C02-SEQ-03: chip_ctrl bus response ready/fire/routing chain

근거:

- `tdc_gpx_chip_ctrl.vhd:604` ~ `tdc_gpx_chip_ctrl.vhd:626`
- `tdc_gpx_chip_ctrl.vhd:1273`

현재 구조:

```text
s_phase_r / s_raw_hold_busy / sub-FSM busy
  -> o_s_axis_tready
  -> s_bus_rsp_fire = i_s_axis_tvalid and o_s_axis_tready
  -> s_init_rsp_valid / s_run_rsp_valid / s_reg_rsp_valid
  -> chip_init/chip_run/chip_reg consume same cycle
```

판단:

- `o_s_axis_tready`는 upstream AXI-stream boundary 출력이다.
- `s_bus_rsp_fire`가 이 ready와 valid를 다시 조합한 뒤 sub-FSM valid로 즉시 분배된다.
- ready/fire/routing이 같은 cycle에 이어지는 구조라 새 2-depth 제한에서는 timing review 대상이다.

권장 보완:

1. bus response accept 시 `s_rsp_fire_r`, `s_rsp_data_r`, `s_rsp_phase_r`를 latch하는 1-stage response register를 둔다.
2. sub-FSM은 registered response fire/data만 consume한다.
3. `i_bus_rsp_pending`와 raw hold full 조건은 기존 backpressure 의미가 바뀌지 않도록 별도 pending register로 유지한다.
4. 변경 시 bus read II, PH_RESP_DRAIN stale response drain, raw bounded backpressure [16]을 재검증한다.

Latency / Throughput / Pipeline / II 영향:

| 항목 | 예상 영향 |
|---|---|
| Latency | bus response consume latency +1 clk 가능 |
| Throughput | bus read II가 증가하지 않도록 response register가 매 cycle accept 가능해야 함 |
| Pipeline | bus response routing stage 1개 추가 |
| II | C01/C02 read timing table와 bounded backpressure scenario 재측정 필요 |

## 3. 허용 예외 / 문서화 대상

### EX-C02-SEQ-01: stop_cfg_decode cfg_image override

근거:

- `tdc_gpx_stop_cfg_decode.vhd:405` ~ `tdc_gpx_stop_cfg_decode.vhd:416`

판단:

- `p_cfg_override`는 config image field를 pack/unpack하고 특정 bit를 constant로 force하는 구조다.
- datapath decision이나 handshake가 아니며, 실질적인 깊은 조합망이 아니다.
- v009 예외 항목의 `type conversion, resize, record/array field pack/unpack`에 해당한다.

관리:

- 현재 상태 유지 가능.
- 단, 분석 문서에는 `조합논리 예외 근거`로 남긴다.

### EX-C02-SEQ-02: XPM FIFO reset qualify

근거:

- `tdc_gpx_face_assembler.vhd:462`
- `tdc_gpx_output_stage.vhd:224` ~ `tdc_gpx_output_stage.vhd:225`

판단:

- XPM FIFO reset을 shot_start/abort와 qualify하는 물리 IP reset 경계 논리다.
- 일반 datapath/handshake 조합망과 다르게 IP reset 요구사항 때문에 존재한다.

관리:

- 현재 상태 유지 가능.
- 단, reset polarity, pulse width, simulation lock 회피 목적, Datasheet/AMD XPM 근거를 별도 문서 또는 코드 주석에 유지한다.

## 4. PASS로 본 항목

| 항목 | 판단 |
|---|---|
| `tdc_gpx_stop_cfg_decode` fire-count ownership | clocked process 내부 variable로 판정하고 registered expected/final/sticky로 닫힘 |
| `tdc_gpx_chip_run` ST_DRAIN_LATCH | fixed wait 제거 후에도 expected snapshot은 sequential FSM에서 처리 |
| `tdc_gpx_chip_ctrl` raw FIFO | `p_raw_fifo`와 `p_raw_busy_reg`가 순차 process로 닫힘 |
| `tdc_gpx_face_seq` `o_face_shot_count` | `s_face_shot_cnt_r` registered output |
| `tdc_gpx_top` chip error merge | 2-depth 이하 glue 수준. 다만 필요 시 status_agg 입력 전 register stage 검토 가능 |

## 5. 수정 계획 권고

새 규칙을 엄격히 적용하면 즉시 RTL 수정 대상은 다음 순서가 합리적이다.

1. `tdc_gpx_face_seq`: closing/shot gating register화
2. `tdc_gpx_chip_ctrl`: bus response routing register화
3. `tdc_gpx_face_assembler`: input ready register화
4. 예외 문서화: stop_cfg cfg override, XPM FIFO reset qualify

수정 전에는 별도 `Code_Fix_Plan` 문서를 작성하고 사용자 승인을 받은 뒤 진행한다. 특히 위 세 항목은 기능 latency와 II에 영향을 줄 수 있으므로, 수정 계획에는 testbench 영향과 재검증 matrix가 포함되어야 한다.

## 6. 검증 요구

수정이 진행될 경우 최소 검증 항목은 다음과 같다.

| 검증 ID | 목적 |
|---|---|
| VB-SEQ-01 | face closing register화 후 마지막 shot 중복 accept 없음 |
| VB-SEQ-02 | shot deferral이 1 clk latency 변화 후에도 phantom shot을 만들지 않음 |
| VB-SEQ-03 | chip_ctrl bus read count exact 유지 |
| VB-SEQ-04 | bounded raw backpressure [16] 재통과 |
| VB-SEQ-05 | PH_RESP_DRAIN stale response drain 유지 |
| VB-SEQ-06 | face_assembler chip switch / blank row / row completion에서 stale ready accept 없음 |
| VB-SEQ-07 | end-to-end latency / throughput / pipeline / II 재측정 |
