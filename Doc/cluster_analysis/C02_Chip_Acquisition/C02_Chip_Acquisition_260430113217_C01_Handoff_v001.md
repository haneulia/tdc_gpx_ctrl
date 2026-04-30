# C02_Chip_Acquisition C01 인계 판단 기록 v001

## 0. 문서 정보

| 항목 | 내용 |
| --- | --- |
| 문서 종류 | C01 -> C02 Cluster 인계 판단 기록 |
| 작성 일시 | 2026-04-30 11:32:17 +09:00 |
| 마지막 수정 일시 | 2026-04-30 11:32:17 +09:00 |
| 작성 목적 | C02 분석/수정에 들어가기 전에 C01에서 인계되어야 할 설계 범위, 구현 범위, 검증 방법, 검증 결과, 다음 Cluster 계약을 명시한다. |
| 절대 기준 | `Doc/TDC-GPX-Datasheet.pdf` |
| 직접 근거 | `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_20260429_v009.md`, `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_Code_Verify_20260429_v005.md` |
| 적용 대상 | `C02_Chip_Acquisition` 계획 v002 및 후속 분석/코딩/검증 문서 |

## 1. 사용자 피드백

사용자 피드백 요지:

> C01에서 C02로 넘어올 때 인계되어야 하는 기본 항목은 C01에서 설계된 범위, 구현, 검증 방법, 검증 결과이다. 또한 `C01_GPX_Bus_Read_20260429_v009`에서 다음 Cluster로 넘길 계약을 C02가 받아야 한다.

## 2. 판단 결론

사용자 판단에 동의한다.

C02는 단순히 "C01 bus primitive가 검증되었다"는 전제를 가져오면 부족하다. C02의 입력 조건은 C01의 산출물을 다음 다섯 묶음으로 받은 뒤 시작해야 한다.

| 인계 묶음 | C02에서 받아야 할 의미 | 판단 |
| --- | --- | --- |
| C01 설계 범위 | C01이 무엇을 책임지고, 무엇을 C02로 넘겼는지 | 필수 |
| C01 구현 범위 | 실제 RTL에서 그 책임이 어떤 모듈, generic, signal, FSM으로 구현되었는지 | 필수 |
| C01 검증 방법 | 어떤 testbench, xsim stage, positive/negative regression으로 닫았는지 | 필수 |
| C01 검증 결과 | PASS/FAIL, transcript, exit code, 남은 잔여 항목 | 필수 |
| 다음 Cluster 계약 | `C01_GPX_Bus_Read_20260429_v009.md` section 15의 C02 인계 계약 | 필수 |

기존 `C02_Chip_Acquisition_260429203421_Plan_v001.md`에는 section 3으로 C01 인계 계약 일부가 이미 들어 있다. 그러나 현재 사용자 피드백 기준으로는 그 정도로 충분하지 않다. C02 v002부터는 "C01 인계 패키지"를 C02의 입구 조건으로 두고, 분석 본문보다 앞에서 이 인계 내용을 먼저 닫아야 한다.

## 3. C01 설계 범위 인계

`C01_GPX_Bus_Read_20260429_v009.md`는 C01 목적을 "GPX IC로부터 값을 읽어 오는 가장 앞단", 즉 `tdc_gpx_bus_phy`의 물리 bus 접근 primitive 분석으로 정의한다.

| 구분 | C01 범위 | C02가 받아야 할 계약 |
| --- | --- | --- |
| 포함 | GPX bus READ/WRITE primitive, address/strobe/OEN/data bus 제어, `bus_ticks`/`bus_clk_div` timing, response hold, status pin 2-FF sync | C02는 bus primitive 내부 timing을 다시 설계하지 않고, 이 primitive가 제공하는 request/response/status 계약 위에서 acquisition FSM을 검증한다. |
| 포함 | 데이터시트 p.7~8 READ/WRITE/OEN timing, p.27 40 MHz readout 제한을 기준으로 한 timing legality | C02의 IFIFO drain은 C01의 `40 MHz` 이하 readout 계약을 깨면 안 된다. |
| 제외 | shot/capture/drain policy, IFIFO empty read 방지 정책, raw stream 의미론, PH_RUN/PH_RESP_DRAIN 운용 | C02가 이 범위를 새로 분석하고 검증해야 한다. |
| 제외 | 16-bit bus mode 실제 지원 | C02는 현재 28-bit mode 기준으로 분석한다. 16-bit mode는 별도 보완 항목이다. |

추적 근거:

| 근거 | 의미 |
| --- | --- |
| `C01_GPX_Bus_Read_20260429_v009.md:6` | C01 목적이 GPX bus 접근 primitive 분석임을 명시 |
| `C01_GPX_Bus_Read_20260429_v009.md:14` | shot/capture/drain 정책은 C02에서 확장한다고 명시 |
| `C01_GPX_Bus_Read_20260429_v009.md:1023-1058` | C02로 넘길 계약 32개 항목 |
| `C01_GPX_Bus_Read_20260429_v009.md:1076-1077` | `chip_ctrl`, `config_ctrl`는 연결 계약 확인 범위, 의미론은 C02로 넘김 |

## 4. C01 구현 범위 인계

C01 구현은 다음 RTL 축으로 C02에 인계된다.

| 구현 축 | C01 구현 내용 | C02에서 확인할 영향 |
| --- | --- | --- |
| `tdc_gpx_bus_phy.vhd` | GPX bus request accept, READ/WRITE FSM, tick 기반 strobe, IOBUF 제어, OEN 제어, response hold, status 2-FF sync | C02의 `chip_ctrl/chip_run`은 `bus_phy`가 새 request를 받을 수 있는 조건과 response/backpressure 조건을 따라야 한다. |
| `tdc_gpx_cfg_pkg.vhd` | bus timing constant, `bus_clk_div`, `bus_ticks` legality/clamp 기준 | `div=1` 허용 시 `ticks>=5` 또는 250 MHz에서 `ticks*div>=7` 계약을 C02 경계에서 깨지 않아야 한다. |
| `tdc_gpx_config_ctrl.vhd` | output stream clock mode `SYNC/ASYNC`, ASYNC mode의 `xpm_fifo_async` 적용 | C02 raw output은 TDC domain에서 생성되고, downstream stream CDC는 config/top 계약으로 분리된다. |
| testbench/script | C01 regression, negative test hook, ASYNC marker | C02 검증도 PASS 문구뿐 아니라 exit code, negative artifact, generate evidence를 남기는 방식으로 따라야 한다. |

추적 근거:

| 근거 | 의미 |
| --- | --- |
| `C01_GPX_Bus_Read_20260429_v009.md:158` | READ entry, OEN low, D-bus Hi-Z, tick 시작 위치 |
| `C01_GPX_Bus_Read_20260429_v009.md:322-324` | burst restart와 `o_rsp_pending` 개선 후보가 C02 II/PH_RESP_DRAIN에 영향 |
| `C01_GPX_Bus_Read_20260429_v009.md:628-644` | `c_BUS_CLK_DIV_MIN=1` 전환과 legality 계약 |
| `C01_GPX_Bus_Read_Code_Verify_20260429_v005.md:247-249` | 실제 수정/보강 파일과 범위 |

## 5. C01 검증 방법 인계

C01 검증 방법은 C02가 따라야 할 기본 검증 운영 방식이다.

| 검증 단계 | C01 검증 방법 | C02 적용 판단 |
| --- | --- | --- |
| Stage 1 direct TB | `tb_tdc_gpx_bus_phy_c01_contract`, `tb_tdc_gpx_bus_phy`, `tb_tdc_gpx_chip_ctrl`, `tb_tdc_gpx_config_ctrl` 직접 검증 | C02도 acquisition 단위 TB와 상위 연결 TB를 분리해 검증해야 한다. |
| Stage 2 mode TB | `g_DUT_STREAM_CLK_MODE=SYNC/ASYNC` 두 mode 검증 | C02 이후 stream CDC 영향을 볼 때 SYNC/ASYNC mode를 분리한다. |
| Stage 3 CSR clamp | CSR clamp 12 case 검증 | C02에서 acquisition parameter가 CSR/clamp 계약을 깨지 않는지 확인한다. |
| Positive regression | 통합 entrypoint에서 `INTEGRATED EXIT CODE = 0` 보존 | C02도 최종 regression transcript에 exit code를 남긴다. |
| Negative regression | 강제 실패 hook으로 `INTEGRATED EXIT CODE = 1` 및 실패 summary 보존 | C02도 empty FIFO read 같은 datasheet 위반을 negative scenario로 검출해야 한다. |
| ASYNC evidence | clean elaboration의 `xpm_fifo_async` compile evidence와 RTL marker를 모두 확인 | C02에서 CDC 관련 closure는 compile evidence와 runtime marker를 모두 본다. |

추적 근거:

| 근거 | 의미 |
| --- | --- |
| `C01_GPX_Bus_Read_Code_Verify_20260429_v005.md:176-185` | Stage 1 direct TB 4종 PASS와 sub-script exit code 0 |
| `C01_GPX_Bus_Read_Code_Verify_20260429_v005.md:187-194` | Stage 2 SYNC/ASYNC mode PASS |
| `C01_GPX_Bus_Read_Code_Verify_20260429_v005.md:196-198` | Stage 3 CSR clamp 12 case PASS |
| `C01_GPX_Bus_Read_Code_Verify_20260429_v005.md:209-211` | Positive integrated exit code 0 |
| `C01_GPX_Bus_Read_Code_Verify_20260429_v005.md:221-239` | Negative run exit code 1, forced fail artifact 보존 |
| `C01_GPX_Bus_Read_Code_Verify_20260429_v005.md:150-170` | ASYNC clean elab, marker evidence |

## 6. C01 검증 결과 인계

C01은 C02 진입 가능 상태로 닫혔다. 다만 "C02 진입 가능"은 C02가 C01 잔여 계약을 검증 책임으로 받는다는 뜻이지, C02 항목까지 이미 닫혔다는 뜻은 아니다.

| 항목 | 결과 | C02 해석 |
| --- | --- | --- |
| 기능 회귀 | Stage 1 4/4, Stage 2 2/2, Stage 3 12/12 PASS | C01 bus primitive와 config 보완은 baseline으로 사용 가능 |
| Positive transcript | `INTEGRATED EXIT CODE = 0` 보존 | C02 regression도 동일한 방식으로 최종 성공 근거를 남긴다. |
| Negative transcript | `INTEGRATED EXIT CODE = 1`, forced fail summary 보존 | C02에서 datasheet 위반 negative test가 실제 실패로 전파되는지 확인한다. |
| ASYNC FIFO evidence | Primary-A `xpm_fifo_async` compile 1 match, Primary-B runtime marker 1 match | C02 stream CDC 판단은 ASYNC mode evidence를 수용한다. |
| 정량 검증 | RDN low pulse, burst READ II, EF sync 2-FF latency, CSR clamp, CDC settling 등 PASS | C02 latency/throughput/pipeline/II 계산의 C01 입력값으로 사용한다. |
| 잔여 인계 | PH_RESP_DRAIN II, backpressure, IrFlag scenario, empty FIFO read 방지 | C02의 핵심 검증 대상이다. |

추적 근거:

| 근거 | 의미 |
| --- | --- |
| `C01_GPX_Bus_Read_Code_Verify_20260429_v005.md:73-78` | C01 보완 검증 closure 요약 |
| `C01_GPX_Bus_Read_Code_Verify_20260429_v005.md:304-324` | 정량 검증 PASS와 C02 추가 검증 필요 항목 |
| `C01_GPX_Bus_Read_Code_Verify_20260429_v005.md:378-382` | C02 진입 가능 및 인계 항목 |
| `C01_GPX_Bus_Read_Code_Verify_20260429_v005.md:417-426` | 사용자 review 기준으로 C02 진입 가능 |
| `C01_GPX_Bus_Read_Code_Verify_20260429_v005.md:494` | C01 cluster 종료, C02 시작 시 인계 항목을 검증 시나리오에 포함 |

## 7. C01 v009 다음 Cluster 계약 수락 항목

`C01_GPX_Bus_Read_20260429_v009.md` section 15의 계약은 C02의 입력 계약으로 수락한다. C02 문서 v002부터는 아래 항목을 시작 조건으로 명시해야 한다.

| C01 계약 | C02에서 받는 방식 |
| --- | --- |
| `bus_phy`는 `i_req_valid`를 `i_tick_en` 경계에서 받아 transaction을 시작한다. | C02 request issue timing과 pending wait 조건의 기준으로 사용 |
| READ/WRITE transaction은 `i_bus_ticks`, `i_bus_clk_div`에 의해 결정된다. | acquisition FSM은 40 MHz 초과 readout을 유발하지 않아야 함 |
| READ response는 deferred response로 전달되고, response는 handshake까지 hold된다. | C02 backpressure와 PH_RESP_DRAIN exit 조건 분석에 반영 |
| Backpressure가 있으면 새 transaction은 stall된다. | C02 throughput/II는 best case와 backpressure case를 분리 |
| `EF1/EF2 active HIGH`는 Interface FIFO empty이며, empty FIFO read 금지이다. | C02 최우선 검증 항목. TB에서 empty read를 fatal로 닫아야 함 |
| `div=1,ticks=5`는 200 MHz에서 40 MHz 경계값이다. | C02는 default/CSR 값이 이보다 빠른 readout을 만들지 않는지 확인 |
| OEN 미연결/pull-up 예외는 normal read/write만 허용한다. | C02는 `oen_permanent` 의존 운용을 board mode와 분리 |
| `o_rsp_pending` register boundary 개선은 1 clock 지연 가능성이 있다. | C02 PH_RESP_DRAIN, non-burst II, timeout 조건에 영향 분석 |
| 2-FF synchronizer는 level status 용도에는 충분하나 pulse event capture는 별도 판단이다. | IrFlag/ErrFlag 사용 의미를 C02에서 level/pulse로 구분 |
| output Stream clock은 `i_tdc_clk`와 별도 계약이다. | C02 raw generation domain과 downstream CDC boundary를 분리 |
| `tS-EF max 11.8 ns`와 2-FF sync latency를 drain stop 판단에 반영한다. | C02 empty FIFO read 방지 timing diagram과 assertion에 포함 |
| 250 MHz 변경 시 `div=1,ticks=5`는 50 MHz readout이라 금지한다. | C02는 200 MHz 기준을 기본으로 두고, clock 변경 시 legality table 재검토 |
| 모든 후속 Cluster 산출물은 latency, throughput, pipeline, II, timing diagram/block diagram을 포함한다. | C02 v002 이후 산출물의 필수 구조 |

## 8. C02 v001에 대한 보완 판단

기존 C02 v001은 C01 계약 일부를 분석에 반영했다. 특히 empty FIFO read 금지, `tS-EF + 2FF`, burst II, stream CDC는 이미 언급되어 있다.

그러나 사용자 피드백 기준으로 보면 C02 v001에는 다음 보완이 필요하다.

| 부족 항목 | 판단 | C02 v002 보완 방향 |
| --- | --- | --- |
| C01 설계 범위 인계 | C01 범위와 C02 범위가 완전한 handoff package로 정리되지 않음 | C02 v002 section 1에 C01 설계 범위 인계 표 추가 |
| C01 구현 범위 인계 | 어떤 RTL 수정과 구현 근거를 baseline으로 받아야 하는지 부족 | `bus_phy`, `cfg_pkg`, `config_ctrl`, regression script를 인계 표로 추가 |
| C01 검증 방법 인계 | C01 검증 운영 방식을 C02 검증 전략으로 충분히 연결하지 못함 | Stage/positive/negative/evidence 방식 재사용 명시 |
| C01 검증 결과 인계 | PASS 결과와 잔여 항목이 C02 입구 조건으로 구조화되지 않음 | C01 v005 closure와 residual handoff를 별도 표로 추가 |
| v009 section 15 계약 수락 | 일부 항목만 선별 수용됨 | section 15 계약 전체를 C02 entry contract로 수락하고 우선순위 부여 |

## 9. 다음 조치

다음 C02 작업은 바로 RTL 보완으로 들어가기보다, 먼저 `C02_Chip_Acquisition_260430114442_C01_Handoff_v002.md`와 `C02_Chip_Acquisition_260430124944_Code_Fix_Plan_v001.md`에 이 인계 패키지를 반영한 뒤 진행하는 것이 맞다.

권장 순서:

1. 본 문서를 C02 v002의 입력 근거로 등록한다.
2. C02 v002 시작부에 "C01 -> C02 인계 패키지" 절을 추가한다.
3. C02 finding 우선순위를 C01 계약 기준으로 재정렬한다.
4. C02 검증 계획에 C01 검증 운영 방식, 특히 positive/negative regression과 evidence 보존 방식을 반영한다.
5. 이후 empty FIFO read 방지, PH_RESP_DRAIN II, `o_rsp_pending` 지연 영향, stream CDC boundary를 RTL/TB 수정 계획으로 확장한다.

## 10. 결론

C02 진입 조건은 "C01이 PASS였다"가 아니라 "C01의 설계 범위, 구현, 검증 방법, 검증 결과, 그리고 C01 v009 section 15의 다음 Cluster 계약을 C02가 수락했다"로 정의해야 한다.

따라서 사용자 피드백은 타당하며, C02 v002부터 이 인계 구조를 공식 입구 조건으로 반영한다.

---

## v001 -> v002 반영 위치 기록

| 항목 | 기록 내용 |
|---|---|
| 변경 원인 | 사용자가 C01 v009 section 15에는 다음 Cluster 계약이 32건인데, C02 인계 문서 v001에는 13건만 보이는 이유를 질문함 |
| 반영된 다음 버전 파일 | `Doc/cluster_analysis/C02_Chip_Acquisition/C02_Chip_Acquisition_260430114442_C01_Handoff_v002.md` |
| 다음 버전 반영 위치 | section 2 `판단 결론`, section 3 `C01 v009 32건 계약 전체 수락 Matrix`, section 4 `v001의 13개 항목이 실제로 덮고 있던 범위` |
| 판단 변화 | v001의 13개 항목은 핵심 주제 요약으로만 인정하고, C02 공식 계약 수락 기준은 C01 v009 32건 전체로 수정 |
| 추적 근거 | `C01_GPX_Bus_Read_20260429_v009.md:1023-1058`, `C02_Chip_Acquisition_260430113217_C01_Handoff_v001.md:124-142`, 사용자 피드백 |
| 기존 문서 수정 시간 | `2026-04-30 11:44:42 +09:00` |
