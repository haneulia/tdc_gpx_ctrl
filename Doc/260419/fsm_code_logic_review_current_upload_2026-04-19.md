# TDC-GPX FSM 코드 논리 점검 보고서 (코드 기준 전수검토)

## 범위
- 검토 기준은 **현재 업로드된 RTL/VHDL 코드만**입니다.
- 계약 문서, 요구 문서, 외부 데이터시트 내용은 판단 근거에서 **제외**했습니다.
- 초점은 **FSM 상태 전이**, **상태 간 인터페이스 정합성**, **timeout / backpressure / CDC / snapshot 시점**입니다.

## 총평
현재 업로드본은 이전보다 분명히 좋아졌습니다. 특히 아래 항목들은 실제로 개선된 상태입니다.

- `chip_run` 쪽 drain/pending-stuck watchdog 보강
- `err_handler`의 read/watchdog 보강
- `chip_init`의 deferred cfg snapshot 보강
- `header_inserter`의 non-idle `face_start` pending latch 추가
- `cell_builder`의 IFIFO2-timeout synthetic `tlast` 보강
- `chip_ctrl`의 raw FIFO depth 확대 및 overflow sticky 노출

그럼에도 불구하고, **코드 논리만 놓고 보면 FSM 안정화 완료라고 보기엔 아직 이릅니다.**
이번 기준으로는 아래 **20개 항목**이 남아 있습니다.

- 치명/높음: 8개
- 중간: 7개
- 낮음/유지보수: 5개

---

## A. 치명 / 높음

### 1. `face_seq`가 `packet_start` / `face_start` / `shot_start_gated`를 1-cycle pulse로 보장하지 못합니다
**위치**
- `tdc_gpx_face_seq.vhd:438-454`
- `tdc_gpx_face_seq.vhd:501-553`
- `tdc_gpx_face_seq.vhd:264-299`
- 소비자 측: `tdc_gpx_top.vhd:623-633`, `tdc_gpx_output_stage.vhd:211-216`, `tdc_gpx_output_stage.vhd:365-367`, `tdc_gpx_header_inserter.vhd:505-523`, `tdc_gpx_face_assembler.vhd:743-773`

**관찰**
`packet_start`는 `s_packet_start_comb`를 그대로 레지스터링한 구조이고, 같은 `WAIT_SHOT` 윈도에서 `i_shot_start_raw`가 들어오면 `s_shot_deferred_r`도 같이 세팅됩니다.
그 결과, 원래 한 번만 나가야 할 시작 이벤트가 다음 cycle에도 유지될 수 있습니다.

핵심은 아래 조합입니다.
- `s_packet_start_comb <= ... (i_shot_start_raw='1' or s_shot_deferred_r='1') ...`
- 같은 edge에서 `i_shot_start_raw='1'`이면 `s_shot_deferred_r <= '1'`
- 상태 전이는 `s_packet_start_r`를 보고 한 cycle 뒤에 일어나므로, 그 사이 `s_packet_start_comb`가 다시 1이 될 수 있음

**영향**
이건 단순 pulse 폭 문제가 아닙니다.
- `o_shot_start_gated`가 2-cycle 이상 올라가면 `p_global_shot_seq`, `p_face_shot_cnt`가 **중복 증가**합니다.
- `output_stage`/`face_assembler`는 `i_shot_start`를 새로운 shot으로 해석하므로, 두 번째 cycle을 **즉시 overrun**으로 볼 수 있습니다.
- `header_inserter`는 첫 `face_start`로 이미 `ST_PREFIX`에 들어간 다음, 다음 cycle의 `face_start`를 **pending**으로 잡아 버립니다. 그러면 현재 face가 끝난 뒤 **유령(face) 한 개가 추가 시작**될 수 있습니다.
- `cell_builder` 쪽 `i_shot_start_per_chip`도 2-cycle이면 buffer switch/drop 경로를 불필요하게 다시 밟습니다.

**권고**
`packet_start`와 `shot_start_gated`를 **level**이 아니라 **accepted-shot one-shot pulse**로 재구성해야 합니다.
가장 안전한 방식은 다음 둘 중 하나입니다.
1. `WAIT_SHOT`에서 “이번 cycle accepted”라는 내부 펄스를 만들고, 그 펄스를 모든 시작 신호의 유일한 source로 사용
2. `s_shot_deferred_r`는 `packet_start_comb=0`일 때만 set되도록 바꾸고, `packet_start_r`는 edge detect 또는 one-shot latch로 생성

---

### 2. 동일 frame/face 내부에서 config snapshot 시점이 모듈마다 다릅니다
**위치**
- `tdc_gpx_chip_ctrl.vhd:571-596`
- `tdc_gpx_face_seq.vhd:315-352`
- `tdc_gpx_top.vhd:562-567`, `tdc_gpx_top.vhd:629-636`
- `tdc_gpx_output_stage.vhd:211-216`, `tdc_gpx_output_stage.vhd:365-367`

**관찰**
현재 구조는 config를 하나의 run-snapshot으로 고정하지 않습니다.
- `chip_ctrl`는 `i_cmd_start` 시점에 run 관련 config snapshot을 잡습니다.
- `face_seq`는 `packet_start` 시점에 geometry / `cfg_face` snapshot을 잡습니다.
- 그런데 top은 `cell_pipe`, `face_assembler` 쪽에는 여전히 **live** `s_cfg.max_hits_cfg`, `s_cfg.max_scan_clks`를 넘기고, `header_inserter` 쪽에는 **snapshot** `s_cfg_face_r`를 넘깁니다.

즉, SW가 `cmd_start` 이후 ~ `packet_start` 사이에 config를 바꾸면,
- chip 측 동작 기준
- face/header 메타데이터 기준
- cell/face assembler의 beat 수, timeout 기준
이 서로 달라질 수 있습니다.

**영향**
한 frame 안에서
- header는 old config
- 데이터 beat 수는 new `max_hits_cfg`
- scan timeout은 new `max_scan_clks`
- chip-run drain 기준은 old snapshot
이런 식의 **혼합 프레임**이 발생할 수 있습니다.
VDMA 입장에서는 row/line 크기와 header metadata가 어긋나므로 치명적입니다.

**권고**
`cmd_start` acceptance 시점에 **pipeline 전체 공용 run_config snapshot**을 한 번만 만들고,
그 snapshot만 face_seq / cell_pipe / output_stage / header_inserter / chip_ctrl가 공통 사용하도록 바꾸는 것이 맞습니다.
최소한 top에서 `i_max_hits_cfg`, `i_max_scan_clks`는 live `s_cfg`가 아니라 `s_cfg_face_r` 또는 전용 run snapshot에서 꺼내야 합니다.

---

### 3. `header_inserter`는 drain 상태에 watchdog이 없습니다
**위치**
- `tdc_gpx_header_inserter.vhd:476-490`

**관찰**
- `ST_DRAIN_LAST`는 마지막 beat가 소비될 때까지 무기한 대기합니다.
- `ST_ABORT_DRAIN`도 `i_m_axis_tready='1'`이 될 때까지 무기한 대기합니다.

**영향**
하류 AXI sink가 영구 backpressure 상태가 되면,
- `frame_done`이 영원히 나오지 않고
- header FSM이 빠져나오지 못하며
- 결국 `face_seq`의 `frame_done_both`도 정렬이 깨집니다.

**권고**
`ST_DRAIN_LAST`, `ST_ABORT_DRAIN`에 timeout + sticky + 강제 복귀 경로를 넣는 것이 좋습니다.
강제 복귀 시에는 “frame truncated” 또는 “header drain timeout” 같은 별도 status가 필요합니다.

---

### 4. `cell_builder`의 `ST_C_QUARANTINE`는 영구 블랙홀 상태가 될 수 있습니다
**위치**
- `tdc_gpx_cell_builder.vhd:581-610`

**관찰**
현재 정책은 DROP timeout 뒤 QUARANTINE에 들어가면,
- final drain marker가 올 때까지
- 또는 `i_abort`가 올 때까지
절대 빠져나오지 않습니다.

이 정책은 “late stale beat가 IDLE에서 upstream stall을 만들지 않게 하자”는 의도 자체는 맞지만,
반대로 final marker가 끝내 안 오면 이 모듈은 **영구적으로 데이터를 흡수하는 sink**가 됩니다.

**영향**
- upstream은 막히지 않을 수 있지만,
- 해당 chip/slope의 이후 데이터는 전부 조용히 버려집니다.
- 시스템이 살아 있는 것처럼 보이면서 특정 채널만 영구적으로 죽는 형태라 디버깅이 어렵습니다.

**권고**
QUARANTINE 자체는 유지하되,
- shot/sequence tag가 맞는 drain marker만 종료 조건으로 인정하거나
- 상위 supervisor가 reset/reinit를 강제할 수 있도록 bounded quarantine + escalate 경로를 추가하는 편이 안전합니다.

---

### 5. `chip_ctrl`는 `PH_RESP_DRAIN`에서 bus가 아직 active인데도 `PH_INIT`로 강제 점프할 수 있습니다
**위치**
- `tdc_gpx_chip_ctrl.vhd:630-679`

**관찰**
`PH_RESP_DRAIN`이 15-cycle hard cap + 65K quarantine cap까지 도달하면,
`i_bus_busy` / `i_bus_rsp_pending`이 여전히 살아 있어도 `PH_INIT` + `s_init_start`로 넘어갑니다.

**영향**
이 시점에는 stale response가 bus_phy/skid/raw path에 남아 있을 수 있습니다.
그 상태에서 init/reg/cfg_write phase로 넘어가면,
다음 transaction이 이전 stale response를 소비해 버리는 **phase 오염**이 생길 수 있습니다.
현재 mismatch sticky는 진단용일 뿐, 예방은 아닙니다.

**권고**
강제 re-init 전에는 최소한
- response path 자체 reset/flush,
- 또는 phase routing 재개 전까지 별도 quarantine 유지
가 필요합니다.
단순히 phase만 `PH_INIT`로 바꾸는 건 충분하지 않습니다.

---

### 6. `chip_ctrl`의 raw FIFO는 depth를 늘렸어도 여전히 control/data beat를 같이 drop합니다
**위치**
- `tdc_gpx_chip_ctrl.vhd:798-842`

**관찰**
FIFO depth가 늘어나긴 했지만, 가득 차면 beat를 drop합니다.
문제는 여기로 들어오는 beat가 raw data만이 아니라
- `drain_done`
- `ififo1_done`
같은 **control beat**도 포함된다는 점입니다.

**영향**
overflow 시 단순 payload loss가 아니라,
상위 FSM이 completion / drain boundary 자체를 못 보는 상태가 됩니다.
즉 “데이터 한 beat 손실”이 아니라 “상태 의미 손실”입니다.

**권고**
control beat는 data beat와 분리하거나,
최소한 control beat 전용 reserved slot을 보장해야 합니다.
지금처럼 공용 FIFO에서 동등하게 drop되면 FSM 정합성 면에서 계속 위험합니다.

---

### 7. `stops_per_chip`의 상한(8) 검증이 빠져 있습니다
**위치**
- `tdc_gpx_face_seq.vhd:198-200`, `tdc_gpx_face_seq.vhd:306-319`
- `tdc_gpx_cell_builder.vhd:701-705`
- `tdc_gpx_face_assembler.vhd:509-513`

**관찰**
현재 시작 시 검증은 `stops_per_chip < 2`만 막고 있습니다. `> 8`은 막지 않습니다.
하지만 코드 전반은 `c_MAX_STOPS_PER_CHIP = 8`을 가정합니다.

구체적으로,
- `face_seq`의 `v_rows`는 `0..c_MAX_ROWS_PER_FACE` 범위 variable에 들어갑니다.
- `cell_builder`, `face_assembler`는 `i_stops_per_chip(2 downto 0) - 1`처럼 3-bit로 잘라서 씁니다.

**영향**
9~15 같은 값이 들어오면
- sim에선 range violation 가능성이 있고,
- synth에선 stop count가 wrap/truncate되어 완전히 다른 행 길이로 해석될 수 있습니다.

**권고**
`face_seq` start validation에서 `stops_per_chip > 8`도 즉시 reject해야 합니다.
이건 clamp보다 reject가 안전합니다.

---

### 8. `cell_pipe`는 실제 backpressure를 `cell_builder.tready`에 연결하지 않습니다
**위치**
- `tdc_gpx_cell_pipe.vhd:215-218`
- `tdc_gpx_cell_pipe.vhd:228-238`
- `tdc_gpx_cell_builder.vhd:326-332`, `tdc_gpx_cell_builder.vhd:406-431`

**관찰**
코드 주석 그대로, `cell_pipe`는 `cell_builder.o_s_axis_tready`를 **실제 upstream backpressure로 사용하지 않습니다.**
시스템이 `shot_start`가 data보다 먼저 온다고 가정하고, 이 가정이 깨지면 sim assert만 울립니다.

**영향**
예를 들어,
- `cell_builder`가 아직 `ST_C_IDLE`인데 raw beat가 들어오면
- 현재 cycle의 `tready`는 0이고
- 첫 beat 처리/유실 여부가 shot_start와 data의 정렬에 강하게 의존합니다.

이 가정은 위 1번(face_seq pulse stretching) 같은 문제와 결합되면 더 위험합니다.

**권고**
`cell_builder.tready`를 실제 backpressure chain에 반영하거나,
최소한 first-beat absorb용 skid/register를 두어 “shot_start와 첫 data 동시/근접 arrival”를 안전하게 처리해야 합니다.

---

## B. 중간

### 9. `config_ctrl`의 `cfg` / `cfg_image` CDC는 metastability는 줄였지만 atomic transfer는 아닙니다
**위치**
- `tdc_gpx_config_ctrl.vhd:452-459`
- `tdc_gpx_config_ctrl.vhd:893-901`

**관찰**
현재 `cfg`/`cfg_image`는 2-FF bundle sync입니다.
코드 주석에도 적혀 있듯, multi-bit atomicity는 source가 몇 cycle 동안 stable하다는 전제에 의존합니다.

**영향**
per-bit metastability 확률은 낮아지지만,
bundle 전체로 보면 “old/new field가 섞인 mixed snapshot” 가능성은 구조적으로 남습니다.
특히 live config를 계속 건드릴 수 있는 구조에서는 더 민감합니다.

**권고**
`cmd_reg`, `expected_ififo`처럼 handshake CDC로 올리는 게 가장 깔끔합니다.
최소한 run-critical subset만이라도 atomic CDC로 분리하는 편이 좋습니다.

---

### 10. `stop_cfg_decode`는 running-total 입력 형식에 강하게 의존하고, 위반은 sim-only assert뿐입니다
**위치**
- `tdc_gpx_stop_cfg_decode.vhd:10-19`
- `tdc_gpx_stop_cfg_decode.vhd:73-87`
- `tdc_gpx_stop_cfg_decode.vhd:92-139`

**관찰**
합성 로직은 stop event stream이 “shot 동안 단조 증가하는 running total”이라고 가정하고 overwrite합니다.
감소/깨짐은 sim-only assert가 잡을 뿐, synth 쪽 fail-safe가 없습니다.

**영향**
upstream 형식이 조금만 어긋나도 expected IFIFO count가 조용히 잘못 latch되고,
그 잘못된 count는 그대로 `chip_run` drain policy에 들어갑니다.

**권고**
합성 로직에도 최소한
- monotonic violation sticky,
- impossible transition reject,
- 또는 delta/running-total 형식 식별
중 하나는 넣는 것이 좋습니다.

---

### 11. `err_handler`는 서로 다른 실패를 `err_read_timeout` 하나로 뭉칩니다
**위치**
- `tdc_gpx_err_handler.vhd:315-322`
- `tdc_gpx_err_handler.vhd:391-406`

**관찰**
현재 `s_err_read_timeout_r`는
- 진짜 `ST_WAIT_READ` timeout에도 쓰이고,
- `ST_WAIT_FRAME_DONE` 장기 대기 escape에도 재사용됩니다.

**영향**
SW/디버거 입장에서는
- reg read path가 죽은 건지,
- frame boundary resync가 길어져 escape된 건지
구분이 안 됩니다.

**권고**
sticky를 분리해야 합니다.
예: `err_read_timeout`, `err_frame_wait_escape`.

---

### 12. `face_seq`는 `n_faces=0`을 reject하지 않고 특수값처럼 취급합니다
**위치**
- `tdc_gpx_face_seq.vhd:198-200`
- `tdc_gpx_face_seq.vhd:247-253`

**관찰**
start validation에서 `n_faces=0`은 막지 않습니다.
이후 state machine은 `s_face_n_faces_r = 0`이면 face_id를 무조건 0으로 되돌리는 특수 처리로 갑니다.

**영향**
의미가 명확하지 않습니다.
- 0을 “1개 face”처럼 취급하는 건지,
- “무한 반복”의 alias인지,
- 단순 invalid config를 살려둔 건지
코드만 봐서는 불분명합니다.

**권고**
정책을 명확히 해야 합니다.
- invalid면 reject
- 특별 의미가 있으면 전 모듈에 동일 semantics 명시

---

### 13. `header_inserter`의 `face_start` pending은 1-depth라 여러 pulse를 collapse합니다
**위치**
- `tdc_gpx_header_inserter.vhd:499-523`

**관찰**
non-idle `face_start`는 pending latch로 1개만 보존합니다.
추가 pulse는 collapse count만 증가하고 기능적으로는 합쳐집니다.

**영향**
짧은 burst성 `face_start`가 들어오면 일부 요청은 구조적으로 사라집니다.
특히 1번 문제처럼 upstream가 multi-cycle pulse를 만들면, 이 collapse 정책이 곧바로 ghost/replay 문제로 연결될 수 있습니다.

**권고**
upstream pulse를 반드시 one-shot로 고치고,
필요하면 `face_start`도 edge-queue 방식으로 바꾸는 것이 안전합니다.

---

### 14. `chip_init`의 busy-window `cfg_write`는 여러 요청을 1개로 합칩니다
**위치**
- `tdc_gpx_chip_init.vhd:170-173`
- `tdc_gpx_chip_init.vhd:311-327`

**관찰**
busy 중 들어온 `cfg_write_req`는 pending 1-depth로만 흡수합니다.
첫 snapshot만 유지되고, 같은 busy window의 이후 요청들은 별도 reject 없이 coalesce됩니다.

**영향**
SW가 init 중 여러 번 cfg_write pulse를 넣으면, 마지막 의도가 반영되지 않을 수 있습니다.
현재는 “묵시적 1-depth queue”이지, 명시적 serialize/reject 인터페이스가 아닙니다.

**권고**
- queue depth를 늘리거나,
- 추가 pulse는 reject sticky를 올리거나,
- 명시적으로 “busy 중 cfg_write는 coalesce된다” status를 내보내는 편이 낫습니다.

---

### 15. `face_assembler`는 `shot_start`마다 input FIFO를 flush하며 old-shot tail을 버립니다
**위치**
- `tdc_gpx_face_assembler.vhd:372-377`
- `tdc_gpx_face_assembler.vhd:454-460`

**관찰**
코드가 명시적으로 old-shot tail drop을 허용합니다.
정책 자체는 “shot boundary integrity 우선”으로 일관적이지만, 기능적으로는 lossy입니다.

**영향**
shot 경계 근처의 늦은 beat는 구조적으로 버려집니다.
현재는 sticky로 보이긴 하지만, 데이터 보존이 아니라 경계 강제 정렬 정책입니다.

**권고**
정책을 유지하더라도,
- flush 이유/건수/status를 더 분리해서 남기거나
- SW discard 판단을 더 쉽게 할 수 있게 cause를 세분화하는 편이 좋습니다.

---

## C. 낮음 / 유지보수 / 관찰성

### 16. `chip_reg`의 busy 중 read/write 동시 입력은 write-wins이며 read intent가 사라집니다
**위치**
- `tdc_gpx_chip_reg.vhd:191-208`

**관찰**
sim assert는 있지만 합성 동작은 write-wins입니다.
queue도 1-depth라 추가 충돌 시 overflow sticky만 남습니다.

**권고**
read/write를 one-hot command로 강제하거나, ambiguous overlap 전용 reject sticky를 두는 편이 좋습니다.

---

### 17. `cmd_arb`의 overlapping reg request queue는 1-depth입니다
**위치**
- `tdc_gpx_cmd_arb.vhd:450-469`

**관찰**
동시/연속 reg request를 1개만 queue하고, 그 이상은 reject sticky 처리합니다.

**권고**
현재 정책 자체는 논리적으로 일관적이지만, SW가 burst reg access를 할 가능성이 있으면 queue depth를 늘리거나 명시적 backpressure/reject 인터페이스가 필요합니다.

---

### 18. `chip_ctrl`의 `PH_IDLE` 동시 명령은 lower-priority command를 드롭합니다
**위치**
- `tdc_gpx_chip_ctrl.vhd:559-597`

**관찰**
우선순위는 정의되어 있지만, dropped command에 대한 runtime reject/status는 없습니다. sim assert만 있습니다.

**권고**
명령 충돌을 caller가 반드시 피해야 한다면 sticky라도 남기는 편이 좋습니다.

---

### 19. `face_assembler`의 `o_face_abort`는 사실상 죽은 포트입니다
**위치**
- `tdc_gpx_face_assembler.vhd:804-808`
- `tdc_gpx_top.vhd:664-670`

**관찰**
포트는 남아 있지만 더 이상 assertion되지 않고, top에서도 의도적으로 `open` 처리합니다.

**권고**
호환성 때문에 남겨둔 것이라면 주석/포트명에 deprecated 표시를 더 명확히 하거나, 이후 리비전에서 제거하는 편이 유지보수에 좋습니다.

---

### 20. `chip_ctrl` raw FIFO overflow 주석이 현재 구현과 어긋납니다
**위치**
- `tdc_gpx_chip_ctrl.vhd:831-833`

**관찰**
주석은 아직 “depth=3” 전제를 말하지만 실제 구현은 더 깊습니다.
기능 버그는 아니지만 리뷰/디버깅 때 혼선을 줍니다.

**권고**
설명 주석을 현재 구현과 맞추는 것이 좋습니다.

---

## 우선순위 제안

### 1차 수정 (가장 먼저)
1. `face_seq`의 shot/packet/face start pulse를 **진짜 one-shot**으로 재설계
2. run/face/header/cell/assembler가 모두 공통으로 보는 **단일 run_config snapshot** 도입
3. `header_inserter` drain watchdog 추가
4. `cell_builder` QUARANTINE에 bounded escape + 상위 복구 경로 추가
5. `chip_ctrl` PH_RESP_DRAIN 강제 re-init 전 **response path 정리** 추가
6. raw/control beat 분리 또는 control beat reserved slot 도입

### 2차 수정
7. `stops_per_chip > 8` reject
8. `cell_pipe`에 실제 builder-ready backpressure 또는 first-beat absorb 추가
9. `cfg/cfg_image` atomic CDC화
10. `stop_cfg_decode` monotonic 위반 runtime 보호 추가

### 3차 수정
11. `err_handler` sticky 분리
12. `n_faces=0` 정책 명문화
13. queue depth / reject observability 정리
14. dead/deprecated 포트와 stale comment 정리

---

## 한 줄 결론
가장 큰 미해결 문제는 **`face_seq`가 시작 펄스를 1-cycle로 보장하지 못한다는 점**과, **config snapshot 시점이 모듈별로 분산되어 있다는 점**입니다. 이 두 축만 먼저 바로잡아도 현재 남아 있는 FSM 불안정성의 상당 부분이 정리될 가능성이 큽니다.
