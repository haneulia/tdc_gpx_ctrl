# Checkpoint K1-4: 통합 RTL/IP 최종 Sign-off

## 1. 최종 판정

Checkpoint K1-4의 **RTL, Golden, 구현 및 패키지 범위는 PASS**다.

| 판정 계층 | 결과 | 의미 |
|---|---|---|
| Unit | PASS | GPX 유지보수, 획득 coordinator, CDC, AXIS, 상태/IRQ 단위 계약 통과 |
| Integration | PASS | 공개 Top의 설정, 물리 GPX, Hit/Cell/Frame, Rise/Fall AXIS 연결 통과 |
| Golden | PASS | RTL 측정값과 HTML, DDR Word, PS H-Line/Ethernet Byte 비교 통과 |
| Implementation | PASS | `xc7z020clg484-2` OOC 배치·배선 6개 routine 프로필 통과 |
| IP package | PASS | IP-XACT, XGUI, package source와 3개 packaged OOC 프로필 통과 |
| Board | **PENDING** | 실제 VDMA/HP port, DMA cache, PCB GPX/LVDS, laser 및 Ethernet 실측 필요 |

최종 증거 세션은 다음과 같다.

```text
signoff_results/sessions/260809_k14_final13_v2_k14_signoff
PASS marker: LIDAR_V2_K14_SIGNOFF_PASS
input snapshot SHA-256:
E148F42982B51C40D89DE5594829D3C81C6A87B08421C3F0C679619360CC61AF
```

최종 세션은 이전 로그를 재사용하지 않고 11개 Gate를 모두 `executed` 상태로
새로 수행했다. Gate 누적 실행시간은 `4023.432초`이며, 각 Gate의 호출 로그와
증거 디렉터리 SHA-256을 `step_summary.json`에 기록했다. 따라서 부분 실행이나
과거 PASS 표식을 현재 결과로 간주하지 않는다.

## 2. K1-4에서 닫은 구조적 결함

### 2.1 부팅 중 GPX Register Read 요청 보존

기존 coordinator는 선택한 Chip이 초기화 중이면 외부 GPX Register Read의
`ready`가 조합 경로로 길어지고, 요청자가 한 clock만 `valid`를 유지할 경우 요청을
잃을 가능성이 있었다.

`lidar_gpx_acquisition_coordinator`에 1-entry 등록형 요청 버퍼를 추가했다.

- 선택 Chip이 부팅 중이어도 승인한 요청을 보존한다.
- 해당 Chip이 서비스 가능해진 뒤 정확히 한 번 전달한다.
- 응답 backpressure 중 payload와 `valid`를 유지한다.
- Soft reset 또는 강제 재초기화는 보류/진행 중 유지보수 요청을 명시적으로 abort한다.
- 보류 요청은 Run 시작, 설정 적용 및 `o_safe` 판정에 포함된다.

직접 TB는 전역 리셋과 강제 재초기화 중 `ready=0`을 확인하고, 재초기화와 겹쳐
유지한 Read 요청이 리셋 해제 후 수락되어 부팅 완료 뒤 응답으로 돌아오는 것과
응답 stall 안정성을 검사한다.

### 2.2 GPX Register 응답 경로 등록

`tdc_gpx_chip_ctrl`의 Register 응답을 등록해 `PH_REG` 상태에서 timeout/reset
카운터로 이어지던 조합 경로를 끊었다. 유지보수 Read 응답 지연은 1 TDC clock
늘어나지만, STOP 입력부터 IFIFO drain까지의 실시간 획득 경로에는 들어가지 않는다.

### 2.3 Raw FIFO 등록형 credit

128-bit 구현에서 downstream ready와 현재 clock pop을 같은 clock의 LUTRAM write
결정에 재사용하던 경로를 제거했다.

```text
Raw FIFO depth                    = 9
정상 데이터 backlog              = 4
ready 반응 지연 예약             = 2
terminal/control 예약            = 2
등록형 credit 탄력 슬롯          = 1
```

현재 clock pop은 다음 clock의 credit으로만 반영한다. 처리량은 1 Raw Word/clock을
유지하고, 추가 슬롯 하나가 등록 경계의 반응 지연을 흡수한다.

### 2.4 Shot CDC 출력 스키드

TDC 도메인의 전체 활성 Chip ready 축약 결과가 Shot async FIFO의 BRAM enable로
직접 이어지던 장거리 경로를 제거했다. `lidar_gpx_shot_gateway`의 CDC 출력 뒤에
2-entry registered-ready 스키드 버퍼를 배치했다.

- 승인된 Shot은 coordinator가 받을 때까지 원자적으로 보존된다.
- 처리량은 1 Shot/clock을 유지한다.
- Shot 전달 지연만 1 TDC clock 늘어난다.
- 동기화된 물리 `fire_done`을 승인하고 측정 시작 기준시점 (T0)에 `start_tdc`를
  만드는 실시간 경로를 변경하지 않는다.

### 2.5 비동기 위상 허용 범위

전체 측정에는 Processing→TDC Shot CDC와 TDC→Processing 결과 CDC의 독립 경계가
있다. 각 경계는 상대 clock 위상에 따라 최대 1 Processing clock 차이를 만들 수
있으므로 RTL 측정 허용 범위는 nominal baseline의 `-2..+2 Processing clocks`다.

HTML 운용 예산은 측정 baseline에 `+2 Processing clocks`를 **한 번만** 더한다.
토폴로지 배율에 위상 guard를 곱하지 않으므로 같은 불확실성을 중복 계상하지 않는다.

### 2.6 TDC-GPX 초기화, Shot 분배 및 Drain watchdog

`tdc_gpx_chip_init`의 설정 쓰기는 `prefetch → prepare → issue` 세 단계로 분리했다.
설정 이미지에서 주소와 28-bit 값을 선택하는 조합논리가 GPX 버스 요청 `valid`와
같은 clock에 연결되지 않으며, 다음 Register가 이전 Register 값을 재사용하지 않는다.

`lidar_gpx_acquisition_coordinator`는 승인된 Shot을 한 번 등록한 뒤 모든 활성 Chip에
동일한 context로 분배한다. 각 lane의 설정 수락 가능 상태도 먼저 등록한 뒤 축약하며,
부팅 중 수락한 외부 TDC-GPX Register Read는 1-entry 요청 버퍼에 보존한다.

레이저 목표 왕복시간 (2R/c, `TARGET_RANGE_WINDOW_5NS`) 뒤의 IFIFO Drain watchdog은
상향 카운터 대신 등록형 카운트다운을 사용한다. 예산에는
`G_DRAIN_MARGIN_TIME_NS`가 추가되고, 0이 아닌 예산에서는 timeout pulse가 한 번만
발생한다.
예산 `0`은 watchdog 비활성 계약이다. `BUS_CLK_DIV`와 `BUS_TICKS`는 이 watchdog이
아니라 Runtime TDC-GPX 버스 읽기 타이밍만 결정한다.

### 2.7 Processing 실시간 경로와 출력 파이프라인

- 가상 Encoder의 LO/HI state 유지시간별 Z pulse 제한값을 설정 정지 구간에서 미리
  계산하고, 운용 중에는 선택값만 등록한다. Z 출력 사건의 clock 위치는 유지한다.
- `shot_scheduler`의 GPX 수락 가능 상태는 요청 광학각 후보점 한 clock 전에 등록한다.
  이 경로는 Shot 발사 허가 조건이며, 물리 `fire_done` 승인에서 측정 시작 기준시점
  `(T0)`과 `start_tdc`를 만드는 저지연 경로가 아니다.
- B8 Frame-lane 조립의 context/geometry와 Cell 방출을 단계 분리했다.
- AXIS lane에는 Cell dispatch 한 단계와 Shot-Line dispatch 한 단계를 추가했다.
  두 단계는 완료시간에 고정 `+2 Processing clocks`를 더하지만 정상상태
  `1 Cell/clock`, `1 Word/clock` 처리율과 32/64/128-bit Byte ABI는 유지한다.

### 2.8 Echo 진단 fanout 분산

Echo 진단의 Shot 시작 초기화를 10개 로컬 register bank로 복제했다. bank 0은
window, bank 1은 전체 합계, bank 2~9는 각각 4채널의 Rise/Fall count를 담당한다.
Shot 시작과 같은 원시 clock에 들어온 첫 Echo는 등록 경계 뒤 같은 window에
수락하므로 유실되지 않는다. 이 한 clock은 CSR/진단 표시 지연일 뿐이며 외부 LVDS
STOP에서 TDC-GPX STOP pin으로 가는 경로에는 들어가지 않는다.

### 2.9 상태 선택과 응답 패킹 분리

Processing 상태 응답은 진단 index 선택과 32-bit 응답 패킹을 서로 다른 등록
단계로 나눴다. TDC 상태의 외부 Register Read는 안전 정지 확인 뒤 요청하고 응답을
보존한다. 따라서 상태 읽기 지연은 늘 수 있지만, Shot 수집이나 물리
`fire_done → 측정 시작 기준시점 (T0)` 경로에는 영향을 주지 않는다.

## 3. 최종 실행 Gate

| 순서 | Gate | 핵심 결과 |
|---:|---|---|
| 1 | 테스트벤치 문서 | 70개 자산: Primary 46, Profile 11, Harness 13 |
| 2 | GPX 유지보수 | timeout, CLEAR_STATUS, Register service 회귀 PASS |
| 3 | Release Top 기능 | routine, 동일 물리 150 MHz, 4:1·1:4, 150/100 MHz PASS |
| 4 | Top 구현 | 두 routine clock × 32/64/128-bit 6개 구현 PASS |
| 5 | 획득 coordinator | 부팅 중 Read 보존, event merge, 150/200 MHz 구현 PASS |
| 6 | 직접 CDC | 6개 clock profile 기능·구현·CDC·DRC PASS |
| 7 | AXIS 폭 | 32/64/128-bit Beat, TKEEP, TLAST, geometry, stall PASS |
| 8 | 상태/IRQ | native snapshot, CTL23/24, IRQ set/W1C/reset 복구 PASS |
| 9 | RTL/HTML | 획득 22 + topology 18 profile 자동 비교 PASS |
| 10 | DDR/PS/Ethernet | 2 clocks × 3 widths의 Word/Byte 비교 PASS |
| 11 | IP package | source/XGUI/IP-XACT 및 packaged OOC 3 profile PASS |

각 Gate의 경로와 SHA-256은 최종 archive의 `step_summary.json`,
`source_manifest.json`에 기록돼 있다.

## 4. 구현 타이밍 결과

### 4.1 공개 Top

부품은 `xc7z020clg484-2`, 기준은 `WNS >= +0.100 ns`다.

| Processing/TDC | AXIS 폭 | WNS | Latch | Black Box | CDC Critical | 예상 외 차단 DRC |
|---|---:|---:|---:|---:|---:|---:|
| 150/200 MHz | 32 | +0.285 ns | 0 | 0 | 0 | 0 |
| 150/200 MHz | 64 | **+0.133 ns** | 0 | 0 | 0 | 0 |
| 150/200 MHz | 128 | +0.291 ns | 0 | 0 | 0 | 0 |
| 200/150 MHz | 32 | +0.170 ns | 0 | 0 | 0 | 0 |
| 200/150 MHz | 64 | +0.177 ns | 0 | 0 | 0 | 0 |
| 200/150 MHz | 128 | +0.140 ns | 0 | 0 | 0 | 0 |

최소 WNS는 150/200 MHz, 64-bit의 `+0.133 ns`다. 기준은 통과했지만 여유가
크지 않으므로 GPX 상태/진단 축약, Raw FIFO credit 또는 출력 dispatch 경로 변경 시
6개 구현 매트릭스를 반드시 다시 실행한다.

### 4.2 직접 CDC 게이트웨이

| 모드 | Processing/TDC | WNS | Async FIFO cell | CDC Critical |
|---|---|---:|---:|---:|
| ASYNC | 150/200 MHz | +0.671 ns | 610 | 0 |
| ASYNC | 200/150 MHz | +0.812 ns | 610 | 0 |
| ASYNC | 200/50 MHz | +0.786 ns | 610 | 0 |
| ASYNC | 50/200 MHz | +0.676 ns | 610 | 0 |
| ASYNC | 150/100 MHz | +2.459 ns | 610 | 0 |
| SYNC | 동일 물리 150 MHz | +3.500 ns | 0 | 0 |

SYNC는 숫자만 같은 별도 clock이 아니라 **동일한 물리 clock source**를 공유하는
설정이다. 서로 독립된 150 MHz clock이면 ASYNC로 설정해야 한다.

### 4.3 GPX 획득 coordinator

직접 구현 WNS는 150 MHz에서 `+1.530 ns`, 200 MHz에서 `+0.388 ns`이며 Latch는
모두 0이다.

## 5. Golden 및 소프트웨어 경계

### 5.1 RTL/HTML 운용 매트릭스

- 획득 profile 22개와 topology profile 18개를 비교했다.
- Return 1..7, 레이저 목표 왕복시간 (`TARGET_RANGE_WINDOW_5NS`)
  288/668/1335개의 5 ns ticks, 32/64/128-bit를 포함한다.
- 전용 2 Rise/2 Fall, 한 Chip 양 edge, 네 Chip 모두 양 edge geometry를 포함한다.
- Cell과 Shot-Line 등록 경계의 고정 `+2 Processing clocks`를 HTML 기준값에 포함한다.
- 첫 완성 Frame은 전체 Beat/Line/SOF/Footer와 실제 데이터를 정확히 비교한다.
  빠른 가상 모터로 추가 Face가 겹치면 완전한 결측 Frame만 허용하고 부분 Frame이나
  데이터 중복은 실패 처리한다.
- 의도된 운용 예제는 PASS 5개, CHECK 3개다.
- 최악 예제 `Return 7, 레이저 목표 편도 거리 1 km, Processing/TDC 200/150 MHz`는
  margin `-526 Processing clocks`로 CHECK다.

CHECK는 RTL 오류가 아니라 요청한 RPM, 인접한 레이저 발사 후보점 사이의 요청
광학각 (`OPTICAL_SHOT_INTERVAL_UDEG`), 레이저 목표 편도 거리 (R)에서 파생한
레이저 목표 왕복시간 (2R/c, `TARGET_RANGE_WINDOW_5NS`) 및 Return 수가 안전한
다음 Shot 시간을 제공하지 못한다는 운용 판정이다.

### 5.2 DDR→PS→Ethernet

Processing 150/200 MHz와 AXIS 32/64/128-bit의 6개 캡처를 비교했다.

```text
packet 0  : 1440-byte Face Header
packet 1+ : 32-byte H-Line Header + 3-byte PACKED17 sample
검증 fixture: packet 2개, 1440 + 38 byte, application payload 1478 byte
```

동일 Processing clock에서 세 AXIS 폭의 DDR 논리 Byte열 SHA-256이 같았다. 즉
AXIS 폭은 전송 Beat 경계만 바꾸며 PS/Viewer ABI를 바꾸지 않는다. Cortex-A9용
decoder object도 생성했지만 실제 DMA cache invalidate/ownership 전환은 보드
범위다.

## 6. Package 및 XGUI

패키지 경로와 VLNV는 다음과 같다.

```text
system_integration/v2/ip_repo/tdc_gpx_lidar_ctrl_v2_2_0
victek.co.kr:lidar:tdc_gpx_lidar_ctrl_v2:2.0
```

검증한 packaged OOC profile:

1. ASYNC 150/200 MHz, 32-bit, Echo receiver 활성;
2. ASYNC 200/150 MHz, 128-bit, Echo receiver 비활성;
3. SYNC 동일 물리 150 MHz, 64-bit, Echo receiver 활성.

패키지 Gate 증거 세션은
`signoff_results/sessions/260809_075704_k010_ip_package`다. canonical source와
package-local 복사본 87개 RTL, XGUI 1개 및 한글 가이드 3개의 binary equality를
검사한 뒤 세 profile을 package-local source만으로 OOC 합성했다.

Vivado의 VHDL-2008 top packager 안내는 알려진 도구 제한이다. 패키지에 source를
명시적으로 등록하고, 패키지 source 자체로 OOC 합성을 다시 수행해 Black Box 없이
통과했으므로 이 Checkpoint의 차단 항목은 아니다.

## 7. 보드 단계로 남긴 항목

K1-4 PASS를 다음 항목의 PASS로 확대 해석하지 않는다.

1. 실제 AXI VDMA와 Zynq HP port의 지속 전송률 및 장기 backpressure;
2. FreeRTOS/PetaLinux DMA ownership과 cache invalidate/flush 순서;
3. PCB의 외부 GPX 40 MHz 기준 클럭(Tref 25 ns), 병렬 버스 및 LVDS 신호 무결성;
4. 물리 laser의 fire/fire_done 지연, 안전 interlock과 실제 STOP 입력;
5. 실제 Ethernet 지속 전송률, packet loss와 Viewer 종단 표시;
6. routine 이외 임의 clock 조합의 공개 Top 구현 타이밍.

Stage L0는 먼저 laser-disabled 상태에서 VDMA/DDR/GPX readback을 검증하고, 그 뒤
제한된 물리 laser 시험으로 확장한다.

## 8. 재실행 방법

전체 Sign-off:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/scripts/run_v2_k14_signoff.ps1 `
  -Stamp <새로운_고유_Stamp>
```

외부 실행 중단 후 같은 입력을 재사용할 때만:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/scripts/run_v2_k14_signoff.ps1 `
  -Stamp <기존_Stamp> -Resume
```

RTL, TB, PowerShell/Tcl 실행 스크립트, XGUI/package 원본, PS 참조 코드,
Golden JSON 또는 K13/DDR HTML이 바뀌면 `-Resume`은 거부된다. 매 실행에서
재생성되는 `ip_repo/component.xml`의 날짜와 checksum은 입력 해시에서 제외한다.
입력이 바뀌었다면 반드시 새로운 Stamp로 전체 Gate를 다시 실행한다.
