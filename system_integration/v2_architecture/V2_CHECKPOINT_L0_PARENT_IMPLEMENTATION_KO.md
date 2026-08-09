# V2 Checkpoint L0 Parent 구현 Sign-off

## 1. 결론

`tdc_gpx_lidar_ctrl_v2:2.0`을 Zynq-7000 `xc7z020clg484-2` Parent에 넣고
Rise/Fall VDMA, HP0/HP1, CSR, IRQ 및 VT_HRL_2 계열 서비스 핀까지 연결한
GUI 프로젝트가 합성·배치·배선·비트스트림 검증을 통과했다.

판정은 **Parent 구현 PASS / 물리 보드 Release 보류**다.

## 2. 이번 단계에서 닫은 문제

| 항목 | 보완 내용 |
|---|---|
| Processing↔TDC stream Reset | 비동기 FIFO Reset을 Source clock 순차회로가 소유하고 Destination Reset을 4단 XPM으로 전달 |
| stale payload | Source 또는 Destination Reset 전 보류 payload를 폐기하는 전용 TB 추가 |
| system command Reset | Processing/TDC sink pulse 생성 FSM을 각 목적지 clock의 동기 Reset으로 변경 |
| Shot ingress Reset | FIFO write-enable을 구동하는 pending slot을 Processing 동기 Reset으로 변경 |
| GPX 양방향 핀 | 공개 Top에 Chip/bit별 명시적 `IOBUF` 적용 |
| OEN IOB | `DYNAMIC_CONNECTED`일 때만 IOB 강제, pull-up/미연결 구성은 FALSE |
| Parent CDC | blanket clock group 제거, handshake/FIFO별 max-delay·bus-skew·narrow false path 적용 |
| TDC 출력 | false path 대신 8 ns register-to-pad 계약 적용 |
| Sign-off gate | endpoint 수, 89핀, CDC, bus skew, methodology, DRC, WNS/WHS, bitstream 크기 검사 |
| 패키지 회귀 | 사용자 Tcl Store와 분리된 임시 캐시, Critical Warning hard-fail, 일반 Warning ID/최대 수 계약 추가 |

## 3. 검증 근거

| 검증 | 세션/마커 | 결과 |
|---|---|---|
| K03 150/200, 200/150 | `260809163557_v2_k03_integration` | PASS |
| Stream Reset 200→150 | `260809190043_v2_stream_gateway_reset` | PASS |
| VDMA profile bridge 150→100 | `260809_190428_l0_vdma_profile_bridge` | 49-bit atomic 전달, ACK 순서 PASS |
| Parent synth | `260809_l0_parent_timing_contract_synth02` | WNS `+0.379 ns` |
| 6 ns 출력 실험 | `260809_l0_parent_timing_contract_impl01` | `-0.049 ns`, 부당하게 짧은 외부 예산 확인 |
| 7 ns 출력 실험 | `260809_l0_parent_timing_contract_impl02` | WNS `+0.060 ns`, 구현 변동 여유 부족 |
| 최종 8 ns Parent/GUI Release | `260809_l0_parent_release_impl04` | WNS/WHS `+0.174/+0.018 ns`, bitstream PASS |
| 격리 캐시 재현성 | `260809_l0_parent_isolated_cache_synth05` | WNS/WHS `+0.379/+0.035 ns`, Critical Warning 0 |
| K0~K10 package/OOC | `260809_185237_k010_ip_package` | 최신 91개 자산, 3 profile, Critical Warning 0, Error 0, Warning 계약 PASS |

8 ns는 기능을 느슨하게 만든 임의 숫자가 아니다. 최악 register-to-pad 실측은
`7.826 ns`이고, 별도 RTL/버스 회귀가 보장하는 최소 25 ns Runtime TDC-GPX
버스 읽기 유지시간 안에 `17.174 ns`의 핀 안정 구간을 남긴다.

## 4. CDC 예외 감사

- 일반 clock-domain 전체 false path: 없음
- GPX 데이터 waiver: 56개 포트→56개 IOB capture FF에만 적용
- 비동기 상태 입력 waiver: EF1/EF2/IRFLAG 6개
- 일반 비동기 서비스 입력: Encoder 3개, `fire_done` 1개, GPX 상태 6개
- 활성 Critical CDC: 0
- mailbox bus-skew 위반: 0

GPX 28-bit 응답은 2FF 동기화 버스가 아니다. RDN 선행, 외부 데이터 안정화,
최소 25 ns 유지시간과 IOB capture를 묶은 비동기 병렬 버스 프로토콜이다.

## 5. 구현 결과

| Clock/path | 여유 |
|---|---:|
| CSR 100 MHz | `+0.77 ns` |
| Processing 150 MHz | `+0.66 ns` |
| TDC 200 MHz | `+0.23 ns` |
| 전체 Setup | `+0.174 ns` |
| 전체 Hold | `+0.018 ns` |

Methodology의 vendor AXI bridge `LUTAR-1`, violation이 없는 `TIMING-9`,
mailbox fanout `TIMING-37`, DRC `REQP-181`은 원본 보고서에서 검토했으며
Critical 또는 blocking 항목은 0이다. 보드 서비스 핀 89개는 모두
`PACKAGE_PIN`과 `IOSTANDARD`를 가진다.

## 6. 범위 밖 항목

이 Checkpoint는 다음을 증명하지 않는다.

- 실제 GPX Chip의 40 MHz Tref, MTimer 및 Register readback
- PCB 신호 무결성과 TDC 200/150/125/100/50 MHz 장기 운용
- VDMA Frame Buffer를 통한 실제 DDR byte 무결성
- FreeRTOS/PetaLinux cache 동기화 API
- 실제 레이저 안전 인터록
- 지속 Ethernet 처리량

따라서 Stage L은 아직 끝나지 않았다. L0 Parent 구현은 닫혔고, 다음은 보드
Runtime 및 PS 소프트웨어 증거를 수집하는 L1 작업이다.

## 7. 유지보수 규칙

1. 공개 Top/Generic 변경 뒤에는 패키지 소스 동기화와 Parent 재생성을 모두 수행한다.
2. Clock-domain 전체 false path를 추가하지 않는다.
3. GPX pin 수 또는 capability mask 변경 시 89핀 계약을 새 profile에 맞게 갱신한다.
4. TDC 출력 8 ns를 변경할 때는 최소 버스 유지시간과 실제 최악 경로를 함께 기록한다.
5. Reset/CDC 변경 시 `tb_lidar_stream_gateway_reset`을 먼저 실행한다.
6. 최종 Parent 결과는 고유 세션 이름과 비트스트림 크기를 기록한다.
7. K0~K10 package runner의 허용 Warning ID나 최대 수를 늘릴 때는 원인 RTL과
   profile별 증가량을 먼저 문서화한다. 단순히 PASS를 얻기 위한 상향은 금지한다.
