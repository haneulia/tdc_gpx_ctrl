# C02_Chip_Acquisition C01 인계 판단 기록 v002

## 0. 문서 정보

| 항목 | 내용 |
| --- | --- |
| 문서 종류 | C01 -> C02 Cluster 인계 판단 보완 기록 |
| 문서 버전 | `v002` |
| 작성 일시 | 2026-04-30 11:44:42 +09:00 |
| 마지막 수정 일시 | 2026-04-30 11:44:42 +09:00 |
| 작성 목적 | C01 v009 section 15의 다음 Cluster 계약 32건이 C02 인계 문서 v001에서 13개 요약 항목으로 축약된 이유와 문제점을 판단하고, 32건 전체를 C02 수락 계약으로 재정리한다. |
| 절대 기준 | `Doc/TDC-GPX-Datasheet.pdf` |
| 직접 근거 | `Doc/cluster_analysis/C01_GPX_Bus_Read/C01_GPX_Bus_Read_20260429_v009.md:1023-1058`, `Doc/cluster_analysis/C02_Chip_Acquisition/C02_Chip_Acquisition_C01_Handoff_20260430_v001.md:124-142` |
| 이전 버전 | `C02_Chip_Acquisition_C01_Handoff_20260430_v001.md` |

## 1. 사용자 피드백

사용자는 `C01_GPX_Bus_Read_20260429_v009.md`의 다음 Cluster 계약이 32건인데, 실제 C02 인계 문서에서는 13건만 넘긴 이유가 있는지 질문했다.

## 2. 판단 결론

명확한 기술적 이유는 없다.

v001의 13개 항목은 32개 계약을 주제별로 압축한 요약이었다. 예를 들어 C01 계약 1~7은 request/response/pending 계약으로 묶었고, 21~22는 `tS-EF + 2FF` drain stop 계약으로 묶었다. 그러나 이 방식은 C02 인계 문서에는 부적절하다.

이유:

1. C01 v009가 section 15에서 32개 계약을 번호로 명시했기 때문에, C02는 32개 전부를 원본 번호와 함께 받아야 한다.
2. 요약 항목만 있으면 어떤 계약이 실제로 수락되었고, 어떤 계약이 단순히 생략되었는지 추적할 수 없다.
3. C02에서 검증해야 할 항목과 C03 이후로 넘길 항목을 구분하려면 원본 32개 계약이 모두 살아 있어야 한다.
4. 운영 프로토콜의 근거 추적 규칙과 Version lineage 규칙에도 맞지 않는다.

따라서 v001의 13개 항목은 "C02 핵심 주제 요약"으로는 의미가 있지만, "C01 v009 계약 수락 목록"으로는 부족하다. C02의 공식 인계 기준은 아래 32건 전체 수락 matrix로 보완한다.

## 3. C01 v009 32건 계약 전체 수락 Matrix

| C01 ID | C01 v009 계약 요지 | C02 수락 방식 | C02 검증/분석 반영 |
| ---: | --- | --- | --- |
| 1 | `bus_phy`는 `i_req_valid` transaction을 `i_tick_en` 경계에서 accept한다. | 수락 | C02 request issue는 tick 경계 accept와 pending 조건을 기준으로 분석한다. |
| 2 | READ/WRITE transaction은 `i_bus_ticks` tick phase로 수행된다. | 수락 | IFIFO drain, register write/read sequencing의 bus phase 계산에 사용한다. |
| 3 | READ response는 GPX data를 IOB FF로 capture한 뒤 deferred AXI response로 전달된다. | 수락 | C02 raw data beat 생성 latency의 C01 입력으로 사용한다. |
| 4 | WRITE response는 write ack이며 `tdata=0`, `tuser[0]=1`이다. | 수락 | C02 config/control write path가 data beat와 ack beat를 혼동하지 않도록 response type을 구분한다. |
| 5 | response는 `tvalid/tready` handshake 전까지 유지된다. | 수락 | C02 backpressure와 PH_RESP_DRAIN exit 조건의 기본 계약으로 사용한다. |
| 6 | 새 request는 pending response가 남아 있으면 accept되지 않는다. | 수락 | C02 FSM은 pending response 상태에서 새 bus request가 stall됨을 전제로 II를 계산한다. |
| 7 | `o_rsp_pending`은 internal deferred response 또는 AXI response hold를 의미한다. | 수락 | PH_RESP_DRAIN, timeout, non-burst II 분석의 핵심 관측 신호로 본다. |
| 8 | status pins는 2-FF sync된 level로 `chip_ctrl`에 들어간다. | 수락 | EF/IrFlag/ErrFlag는 level 관측 신호로만 해석한다. |
| 9 | bus timing legality는 `bus_phy` 단독이 아니라 CSR/SW 운용 계약까지 포함해 보장해야 한다. | 수락 | C02는 CSR/config path가 illegal timing 조합을 만들지 않는지 확인한다. |
| 10 | C01 bus timing 계산은 `i_tdc_clk=200 MHz`, `T_clk=5 ns` 기준이다. | 수락 | C02 기본 분석 clock은 200 MHz로 고정한다. 다른 clock은 별도 retiming 검토로 분리한다. |
| 11 | `EF1/EF2 active HIGH`는 empty FIFO이며, empty FIFO read는 금지된다. | 수락, 최우선 | C02의 가장 중요한 검증 항목이다. TB에서 empty read를 fatal/assertion으로 닫아야 한다. |
| 12 | 현재 RTL은 28-bit bus 운용만 지원한다. Reg14[4] 16-bit mode 차단 정책은 유지되어야 한다. | 수락 | C02 분석 범위는 28-bit mode이며, 16-bit mode는 unsupported/protected mode로 유지한다. |
| 13 | `div=1,ticks=5`는 `RDN low->capture=15 ns`, `tick0->capture=20 ns`, burst beat period `25 ns=40 MHz`로 구분한다. | 수락 | C02 timing diagram과 latency table에서 세 기준점을 분리해서 사용한다. |
| 14 | GPX data bus readout은 데이터시트 40 MHz transfer rate를 넘지 않는다. | 수락 | IFIFO drain throughput 상한은 40 MHz 이하로 제한한다. |
| 15 | OEN 미연결/pull-up 예외는 normal read/write만 허용하고 `oen_permanent` 의존을 제거하는 generic 정책이 필요하다. | 수락 | C02 board mode별 OEN 운용에서 burst/permanent 의존 경로를 분리한다. |
| 16 | OEN Low 고정은 WRITE bus contention 때문에 현재 구조에서 지원 불가로 본다. | 수락 | C02는 OEN Low fixed configuration을 valid 운용 모드로 취급하지 않는다. |
| 17 | `o_rsp_pending` 같은 조합 경계는 향후 FF/register boundary 개선 후보에 올린다. | 수락 | C02에서 register화 시 1 clock 지연이 PH_RESP_DRAIN/II에 미치는 영향을 분석한다. |
| 18 | 2-FF synchronizer는 level status에는 충분하나 pulse event capture 용도로 해석하지 않는다. | 수락 | IrFlag/ErrFlag가 pulse 의미로 쓰이는지 검토하고, pulse라면 별도 capture 전략을 finding으로 분리한다. |
| 19 | `c_BUS_CLK_DIV_MIN=1` 변경은 목표지만 `div=1=>ticks>=5`를 강제해야 한다. | 수락 | C02/cfg/CSR path에서 legalize 계약이 깨지지 않는지 확인한다. |
| 20 | output Stream clock은 `i_tdc_clk`와 별도 계약이며, Sync/Async generic 및 Async FIFO가 필요하다. | 수락 | C02 raw generation domain과 downstream stream CDC boundary를 분리한다. |
| 21 | `tS-EF max 11.8 ns`와 2-FF sync latency를 C02 drain stop 판단에 반영해야 한다. | 수락 | empty FIFO read 방지 timing diagram과 assertion guard에 포함한다. |
| 22 | 200 MHz에서 raw `tS-EF` guard는 3 clock, `EF_sync`는 최소 4 clock 관측 지연이다. | 수락 | `ST_DRAIN_SETTLE` 3 clock이 충분한지 C02에서 재검증한다. |
| 23 | 250 MHz 변경 시 `div=1,ticks=5`는 50 MHz readout이므로 금지하고, `bus_ticks * bus_clk_div >= 7` 계약이 필요하다. | 수락 | C02 기본은 200 MHz이며, 250 MHz 검토 시 legality table을 별도 적용한다. |
| 24 | 250 MHz 변경은 `cfg_pkg`, CSR clamp, TB clock, timeout/count 기반 상수를 함께 재검토해야 한다. | 수락 | clock retiming은 C02 기본 보완과 분리된 영향성 항목으로 둔다. |
| 25 | 200 MHz/250 MHz 비교는 정상 READ 운용 시퀀스 기준으로 같은 표에서 비교한다. | 수락 | C02 timing 비교 자료도 같은 기준점으로 작성한다. |
| 26 | 125~200 MHz 범위 정수 `Tclk` 후보는 200, 166.667, 142.857, 125 MHz이다. | 수락 | clock 후보 비교가 필요할 경우 C01 표를 C02 입력으로 사용한다. |
| 27 | 후보 clock별 최단 정상 READ 설정은 C01 v009에 정의된 값을 사용한다. | 수락 | C02 throughput 비교에서 임의 설정을 만들지 않고 C01 후보표를 기준으로 한다. |
| 28 | `bus_phy` local latency는 `tick0->capture`, `tick0->bus_phy AXI tvalid`로 분리하고 skid/chip_run latency를 추가한다. | 수락 | C02 latency 분석의 구조를 이 분해 방식으로 작성한다. |
| 29 | Burst READ throughput 상한은 `bus_ticks * bus_clk_div * Tclk`이고 실제 throughput은 response/raw/downstream backpressure로 낮아질 수 있다. | 수락 | C02 throughput은 best case와 backpressure case를 분리한다. |
| 30 | 250 MHz 정상 후보는 142.857 MHz 후보와 같은 35.7 MHz readout이므로 throughput 이득이 제한적이다. | 수락 | C02에서 clock 상향을 성능 개선 근거로 단정하지 않는다. |
| 31 | Burst READ II와 non-burst READ II는 별도로 계산한다. | 수락 | C02 II 분석에서 burst drain과 개별 transaction을 분리한다. |
| 32 | 모든 후속 Cluster 산출물은 latency, throughput, pipeline, II와 timing diagram/block diagram을 포함한다. | 수락 | C02 Markdown/PPT v002 이후 필수 산출물 기준으로 적용한다. |

## 4. v001의 13개 항목이 실제로 덮고 있던 범위

v001의 13개 항목은 다음처럼 32개 계약 일부를 묶어 표현했다.

| v001 요약 항목 | 대응되는 C01 원본 ID | 문제점 |
| --- | --- | --- |
| request issue timing | 1, 2 | WRITE/READ tick phase와 accept edge가 한 줄로 합쳐짐 |
| response/deferred/hold | 3, 5, 6, 7 | WRITE ack 계약 4번이 명확히 드러나지 않음 |
| empty FIFO read | 11 | 정상 |
| `div=1,ticks=5` timing | 13, 14, 일부 19 | 40 MHz 제한과 legality 계약이 섞임 |
| OEN 예외 | 15, 일부 16 | OEN Low fixed unsupported가 명시적으로 분리되지 않음 |
| `o_rsp_pending` register boundary | 17, 일부 31 | II와 PH_RESP_DRAIN 영향이 함께 묶임 |
| 2-FF synchronizer | 8, 18 | status level 계약과 pulse 금지가 분리되지 않음 |
| Stream clock | 20 | 정상 |
| `tS-EF + 2FF` | 21, 22 | 정상이나 settle 검증 항목과 연결 필요 |
| 250 MHz illegal | 23, 일부 24, 30 | retiming 영향성 24번과 성능 판단 30번이 누락처럼 보임 |
| 모든 후속 산출물 규칙 | 32 | 정상 |

이 표에서 보듯 v001은 "누락 의도"가 아니라 "압축 표현"이었다. 하지만 C02가 계약을 검증해야 하는 단계에서는 압축 표현만으로 충분하지 않다.

## 5. C02 문서 보완 원칙

앞으로 C02 v002 및 후속 문서는 다음 원칙으로 작성한다.

1. C01 v009 section 15의 32개 계약은 모두 원본 ID를 유지한다.
2. C02에서 직접 검증할 계약과 C03 이후로 넘길 계약을 분리하되, 원본 번호는 삭제하지 않는다.
3. PPT에는 32개 전체를 전부 나열하지 않더라도, Markdown에는 full matrix를 두고 PPT에는 "핵심 그룹 + 원본 ID 범위"를 표시한다.
4. C02 finding은 가능한 한 C01 원본 계약 ID를 함께 적는다. 예: `F-C02-01`은 `C01-C11`, `C01-C21`, `C01-C22`와 연결.
5. 계약을 요약할 때는 "요약 항목"과 "원본 계약 ID"를 별도 column으로 둔다.

## 6. 결론

C02 인계 문서 v001에서 13건만 보인 것은 기술적 제외가 아니라 문서 요약 방식 때문이다. 그러나 사용자가 지적한 대로 이 방식은 계약 수락 문서로는 부족하다.

C02의 공식 판단은 다음과 같이 수정한다.

> C01 v009 section 15의 다음 Cluster 계약 32건은 모두 C02 입력 계약으로 수락한다. C02에서 직접 검증할 항목과 후속 Cluster로 넘길 항목은 구분하되, 원본 32개 계약 번호는 삭제하거나 압축하지 않는다.

## 7. v001 -> v002 반영 요약

| 항목 | 기록 내용 |
| --- | --- |
| 변경 원인 | 사용자가 C01 v009의 32개 계약 중 C02 인계 문서에 13개만 보이는 이유를 질문 |
| 반영된 이전 버전 | `C02_Chip_Acquisition_C01_Handoff_20260430_v001.md` |
| 반영된 다음 버전 파일 | `C02_Chip_Acquisition_C01_Handoff_20260430_v002.md` |
| 다음 버전 반영 위치 | section 3 `C01 v009 32건 계약 전체 수락 Matrix` |
| 판단 변화 | 13개 요약은 C02 핵심 주제 요약으로만 인정하고, 공식 계약 수락 기준은 32개 full matrix로 수정 |
| 추적 근거 | `C01_GPX_Bus_Read_20260429_v009.md:1023-1058`, 사용자 피드백 |

## 8. Handoff v002 -> Code Fix Plan v001 반영 위치 기록

| 항목 | 기록 내용 |
| --- | --- |
| 변경 원인 | 사용자가 Handoff 문서를 기반으로 C02 수정계획 수립을 요청 |
| 반영된 다음 계획 파일 | `C02_Chip_Acquisition_Code_Fix_Plan_20260430_v001.md` |
| 다음 계획 반영 위치 | section 2 `Handoff 계약 우선순위화`, section 3 `수정 목표`, section 5 `검증 항목`, section 10 `사용자 승인 요청 항목` |
| 판단 변화 | C01 v009 32건 전체 수락 matrix를 C02 code-fix 우선순위, RTL/TB 수정 목표, xsim 검증 항목으로 확장 |
| 추적 근거 | 본 문서 section 3, `C02_Chip_Acquisition_20260429_v001.md` F-C02-01~05 |
| 기존 문서 수정 시간 | `2026-04-30 12:49:44 +09:00` |
