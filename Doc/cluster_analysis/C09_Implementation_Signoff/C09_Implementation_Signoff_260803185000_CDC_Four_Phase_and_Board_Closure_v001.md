# TDC-GPX CDC 4상 handshake 및 실제 parent 구현 종결

## 1. 목적과 최종 판정

이번 점검은 TDC clock이 AXIS clock보다 빠르거나 느린 양방향 비동기 조합과
동일 clock net의 SYNC 조합을 기능, 패키지, 합성 및 실제 parent 배치·배선까지
연결해 검증하는 데 목적이 있다.

다음 범위는 **sign-off PASS**로 판정한다.

- `tdc_gpx_top` RTL 기능 및 local/unified CSR 경로
- AXIS 200 MHz / TDC 50 MHz ASYNC
- AXIS 50 MHz / TDC 200 MHz ASYNC
- 동일 clock net AXIS 150 MHz / TDC 150 MHz SYNC
- AXIS 150 MHz / TDC 200 MHz, 32/64/128-bit 패키지
- 통합 IP revision 20, Echo enable/external STOP, 32/64/128-bit OOC
- 실제 `project_4`의 2-chip rising 구성 합성 및 배치·배선
- `project_4_tdc_gpx_4chip`의 4-chip rising 2 + falling 2 합성 및 배치·배선

4-chip dedicated rising 2 + falling 2 구성도 revision 20, 171핀 XDC를 적용한
별도 GUI 확인용 parent에서 구현했다. 따라서 1~4-chip RTL 지원과 2/4-chip
보드 정적 구현 범위가 현재 revision 기준으로 닫혔다.

## 2. 발견된 CDC 결함

### 2.1 현상

local CSR, AXIS 200 MHz / TDC 50 MHz에서 `max_range_5ns_ticks`를 갱신했을 때
TDC domain이 최신값 대신 이전값을 유지하는 현상이 재현됐다. 예를 들어 기대
환산값이 334 TDC clocks인데 stale 값 67이 관측됐다.

### 2.2 원인

`xpm_cdc_handshake` sender가 다음 순서를 지키지 않았다.

1. source가 `src_send`를 올린다.
2. destination이 요청을 받고 `src_rcv`가 올라온다.
3. source가 `src_send`를 내린다.
4. destination acknowledgement가 해제되어 `src_rcv`가 내려간다.

기존 sender는 3단계 직후 `src_rcv='1'`인 동안에도 변경값을 보고
`src_send`를 다시 올릴 수 있었다. source clock이 destination보다 빠르면 이
재전송이 이전 acknowledgement에 흡수되어 최신 snapshot이 유실될 수 있다.

### 2.3 수정 계약

- 새 전송은 `src_send='0' and src_rcv='0'`일 때만 시작한다.
- `src_in`에는 live bus가 아니라 전송 시작 시 고정한 hold register를 연결한다.
- 전송 중 변경된 최신값은 acknowledgement가 완전히 내려온 뒤 재전송한다.
- CDC idle은 `src_send=0`뿐 아니라 모든 `src_rcv=0`과 image pending 상태 해제를
  함께 확인한다.

이 계약을 `tdc_gpx_config_ctrl`, `tdc_gpx_csr_chip`,
`tdc_gpx_csr_pipeline`의 XPM handshake sender에 동일하게 적용했다.

## 3. 클럭 및 기능 회귀

모든 시나리오는 4 GPX chip, 16 APD channel, rising mask `0011`, falling mask
`1100`, STOP당 Return 7, 외부 28-bit I-Mode bus model을 사용했다.

| Control | Stream | AXIS/TDC | 폭 | Face | TDC range clocks | Fire | Rise/Fall TLAST | raw word | point margin | face-gap margin | 결과 |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| local | ASYNC | 200/50 MHz | 32 | 1 | 334 | 3 | 3/3 | 672 | 5,828 | 8,331,970 | PASS |
| unified | ASYNC | 200/50 MHz | 32 | 1 | 334 | 3 | 3/3 | 672 | 5,830 | 8,331,970 | PASS |
| unified | ASYNC | 50/200 MHz | 128 | 1 | 1,335 | 3 | 3/3 | 672 | 1,858 | 2,082,986 | PASS |
| unified | SYNC | 150/150 MHz | 64 | 1 | 1,002 | 9 | 9/9 | 2,016 | 159 | 6,248,976 | PASS |
| unified | SYNC | 150/150 MHz | 64 | 5 | 1,002 | 15 | 15/15 | 3,360 | 2,242 | 248,976 | PASS |

모든 결과는 다음 상태 계약을 만족했다.

- `STAT5 = 0x00000001`
- `STAT6 = 0xF0000000`
- `STAT7 = 0x00000000`
- `schedule_overrun = 0`
- `pipeline_abort = 0`
- `cfg_rejected = 0`

`SYNC`는 주파수 숫자만 같은 두 독립 clock이 아니라, testbench와 parent에서
AXIS/TDC에 동일한 물리 clock net을 연결하는 모드다.

## 4. Face 경계 계약

point 간격과 Face 경계의 비활성 간격은 서로 다른 두 예산이다.

```text
point interval >= fire_done timeout + target round-trip + guard + FSM overhead
face inactive gap >= 같은 worst re-arm 시간
```

Face active width는 decoded-state half-width에서 기계각으로 환산하고, Face pitch
`360 / N_FACES`에서 빼서 inactive gap을 계산한다. 시뮬레이션 가속용 회전주기와
실제 운용 RPM의 Face gap도 별도로 계산한다. 이 보완으로 point margin이 양수여도
Face 전환 첫 점에서 busy가 남는 구성을 PASS로 오판하지 않는다.

Laser scheduler 단위 회귀에는 다음 경계를 추가했다.

- 비활성 간격 0에서 다음 Face request가 들어오는 overrun: 의도된 검출 PASS
- 충분한 비활성 간격 뒤 다음 Face request: 정상 재무장 PASS

## 5. HDL/HTML 정렬

C08 v025 HTML은 다음 항목을 RTL과 같은 식으로 계산하도록 수정했다.

- point interval과 Face inactive gap을 별도 표시
- 현재 Face 수와 공통 active half-width로 실제 운용 Face gap 계산
- Face gap과 Laser worst re-arm 시간의 독립 PASS/CHECK 판정
- raw hit 수, line/beat 및 폭별 payload 계산의 RTL contract 비교

자동 checker는 5개 contract를 모두 통과했고
`C08_V025_HTML_SELF_TEST_PASS` marker를 확인했다. 로컬 `file://` 페이지의 수동
브라우저 제어는 브라우저 보안 정책으로 제한됐지만, 같은 JavaScript 계산 경로를
실행하는 checker 결과는 PASS다.

## 6. IP 패키지와 독립 구현

### 6.1 Standalone TDC-GPX

- core revision: 10
- package source 일치: 45 files
- packaged OOC: 32/64/128-bit 모두 PASS
- 대표 4-chip ASYNC 150/200 MHz, 64-bit post-route:
  - WNS `+0.249 ns`
  - WHS `+0.066 ns`
  - black box 0
  - route error 0

OOC의 NSTD/UCIO 경고는 보드 핀 XDC가 없는 독립 IP 구현에서 예상되는 항목이며,
실제 parent 관문에서 별도로 닫았다.

### 6.2 통합 LiDAR IP

- core revision: 20
- package source: 88 files
- Echo enable/external STOP 각각 32/64/128-bit OOC PASS
- Echo enable은 `IBUFDS=32`, external STOP은 `IBUFDS=0`
- 모든 ASYNC build에서 raw CDC root 4, black box 0, LUTAR-1 0

사용자 Tcl Store cache가 손상된 환경에서도 재현되도록 package/board script는
Vivado 설치 Tcl Store의 `appinit 1.2`를 명시적으로 로드한다. 시작 시 출력되는
사용자 cache 경고는 환경 경고이며, sign-off script 자체는 설치 copy로 동작한다.

## 7. 실제 project_4 구현

대상은 `C:/Projects/my_sp/ALINX/Logic/project_4`의
`design_1_lidar_ctrl.bd` 하나다. 기존 `design_1`과 `design_1_unified`는 갱신하지
않았고 실행 전후 SHA-256이 각각 다음 값으로 유지됐다.

- `design_1`: `11E1645464C23749B18370C0E50D00A6589FB26D02360B21768207203F0FE344`
- `design_1_unified`: `BF4B2DFCB0016096CC922181950AF98CAA5590377BFA5DF17E057C1CF68E4FE0`

통합 instance만 revision 17에서 20으로 갱신했다. 4-chip GUI 확인용 프로젝트
`C:/Projects/my_sp/ALINX/Logic/project_4_tdc_gpx_4chip`도 같은 revision으로
갱신하고 171핀 XDC를 다시 구현했다.

| 항목 | 2-chip Stage 1 | 4-chip Stage 2 |
|---|---:|---:|
| Device | `xc7z020clg484-2` | `xc7z020clg484-2` |
| Slope | rising `0011`, falling `0000` | rising `0011`, falling `1100` |
| Mandatory PL pins | 89 | 171 |
| Pin audit free / duplicate | 111 / 0 | 29 / 0 |
| 합성 WNS | `+0.396 ns` | `+0.396 ns` |
| route WNS / WHS | `+0.128 / +0.036 ns` | `+0.080 / +0.025 ns` |
| unrouted/partial net | 0 / 0 | 0 / 0 |
| black box | 0 | 0 |
| pulse/structural timing violation | 0 / 0 | 0 / 0 |
| unexpected CDC | 0 | 0 |
| severe DRC / methodology violation | 0 / 0 | 0 / 0 |
| bus-skew violation | 0 | 0 |
| GPX read window | 25 ns | 25 ns |
| GPX read input / RDN output | 1.536 / 3.747 ns | 1.536 / 3.691 ns |
| GPX read board budget | `+7.917 ns` | `+7.973 ns` |
| GPX write setup / margin | 12.341 / `+7.341 ns` | 10.702 / `+5.702 ns` |
| GPX write hold / margin | 17.803 / `+13.803 ns` | 17.955 / `+13.955 ns` |

외부 protocol port 66 input, 79 output은 board/interface 지연을 하나의 임의
기본값으로 숨기지 않고 `EXTERNAL_PROTOCOL_AUDIT` 대상으로 분류한다. GPX bus는
별도의 명시적 board budget으로 판정했으며 모두 양수다.

## 8. 최종 release 경계

### 닫힌 항목

- CSR snapshot의 atomicity, hold 안정성, 4상 재전송 및 idle 계약
- TDC가 AXIS보다 4배 빠른 조합과 AXIS가 TDC보다 4배 빠른 조합
- 동일-net SYNC 150/150 MHz
- Return 7, 16 APD channel, 28-bit I-Mode/17-bit Hit 보존
- 32/64/128-bit VDMA output contract
- point 및 Face 경계 Laser re-arm 가능성
- standalone/integrated package와 2/4-chip parent 구현

### 별도 관문

- 실제 TDC-GPX와 PCB trace를 사용한 lab read/write margin 측정
- Ethernet/VDMA downstream 장기 backpressure의 시스템 운용 정책

따라서 revision 10 TDC-GPX IP와 revision 20 통합 IP는 현재 검증 범위에서
release 가능하다. 2-chip과 4-chip parent 모두 bitstream 생성 전 단계의 정적
sign-off를 통과했다. 제품 보드 sign-off는 실제 TDC-GPX와 PCB에서 전기 조건 및
read/write margin을 측정한 뒤 승격한다.
