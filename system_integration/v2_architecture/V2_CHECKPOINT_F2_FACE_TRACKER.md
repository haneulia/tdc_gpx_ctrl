# v2 Checkpoint F2: Face Tracker

## 1. 판정

Stage 3 / Checkpoint F의 두 번째 기능 경계인 F2 `face_tracker`를 완료했다.
v1의 Face membership 의미를 보존하면서 v2의 직접 등록 이벤트 경로에
enter/exit, overlap 진단, source latency 문맥과 atomic configuration version을
추가했다.

**결론:** B1은 통과했다. Checkpoint F 전체는 아직 진행 중이며 다음 허용
단계는 F3a operation/safety state owner뿐이다. F3b `shot_scheduler`는 F3a가
RUN/STOP/ARM/DISARM과 laser permit을 소유하기 전에는 시작하지 않는다.

## 2. v1 비교 기준

고정 소스는
`C:/Projects/my_sp/lib/IP/motor_decoder/HDL/mirror_active_detect.vhd`이며
SHA-256은 다음과 같다.

```text
389F00678C3FB09B4DEE1A7CCE63AB401C968450D0BCDB9A27E9000EE9E91CD2
```

v1과 동일하게 non-wrap은 `lower <= position <= upper`, wrap은
`position >= lower OR position <= upper`로 판정한다. 여러 Face가 일치하면
가장 낮은 Face index를 선택한다. v1의 AXIS adapter와 probe 조합 경로는
복제하지 않았다.

## 3. 구현 구조

| 단계 | 처리 | 등록 결과 |
|---|---|---|
| Stage 1 | build/runtime mask 적용, 최대 5개 inclusive 비교, 최저 index 선택, overlap 계산 | membership와 B0 문맥 |
| Stage 2 | 직전 membership 및 방향과 비교, traversal 전환 계산 | `face_event_t` |

입력 B0 이벤트에서 출력 B1 이벤트까지 지연은 1 Processing clock이다.
파이프라인 initiation interval은 1이므로 B0 이벤트가 매 클럭 들어와도 손실
없이 처리한다. 설정 lower/upper, mask와 version은 `i_enable=0`인 quiescent
구간에서만 지역 레지스터에 복사한다.

`face_event_t`는 다음 정보를 한 번에 전달한다.

| 필드 | 의미 |
|---|---|
| `valid` | B0 입력에 대응하는 1-clock B1 결과 |
| `inside` | 현재 위치가 선택된 활성 Face 안에 있음 |
| `enter_event` | 새 traversal 시작 |
| `exit_event` | 이전 traversal 종료 |
| `face_index` | 현재 Face, 또는 현재가 밖이면 방금 이탈한 Face |
| `overlap` | 활성 Face 둘 이상이 동시에 일치 |
| B0 context | position, direction, source, latency, Z, active version을 그대로 전달 |

## 4. 방향과 경계 규칙

- Geometry는 방향과 무관한 inclusive `[lower, upper]`로 저장한다.
- CW에서는 lower에서 진입하고 upper 다음 상태에서 이탈한다.
- CCW에서는 upper에서 진입하고 lower 이전 상태에서 이탈한다.
- Face 안에서 방향이 바뀌면 같은 이벤트에서 `exit_event=1`과
  `enter_event=1`로 이전 traversal을 닫고 새 traversal을 연다.
- gap 없이 Face A에서 Face B로 이동해도 두 이벤트가 모두 1이며
  `face_index`는 현재 Face B이다.
- 정상 commit은 overlap을 거부한다. 방어 경로는 최저 Face를 선택하고
  per-event `overlap`, sticky와 32-bit 누적 count를 함께 남긴다.
- build의 `G_BUILD_CONFIG.num_faces` 밖 비트는 runtime mask가 잘못 1이어도
  무시한다. 별도 Face-count generic을 만들지 않아 build 설정의 소유권을
  한 record로 유지한다.

## 5. 기능 검증

`tb_face_tracker`를 150 MHz와 200 MHz에서 각각 build Face 수 1, 2, 3, 4,
5로 실행해 총 10개 조합을 검사했다.

| ID | 시나리오 | 결과 |
|---|---|---|
| P10 | non-wrap lower/upper inclusive, 1-clock latency, 연속 II=1 입력 | PASS |
| P11 | 0 상태를 가로지르는 modular wrap | PASS |
| P12 | CW/CCW 경계, Face 내부 reversal, zero-gap Face 전환 | PASS |
| P13 | build Face 1..5, runtime mask 1..N, build-mask 방어, overlap 진단/clear | PASS |

최종 marker는 각 조합에서 다음 형식으로 확인했다.

```text
LIDAR_V2_FACE_TRACKER_PASS proc_mhz=<150|200> faces=<1..5>
```

## 6. 배치 및 타이밍 결과

최종 보관 세션:
`signoff_results/sessions/260804184000_v2_face_tracker`

Worst-case `G_BUILD_CONFIG.num_faces=5`, `xc7z020clg484-2` OOC 구현 결과는
다음과 같다.

| Processing clock | WNS | Latch | ASYNC_REG | Critical CDC |
|---:|---:|---:|---:|---:|
| 150 MHz | `+2.736 ns` | 0 | 0 | 0 |
| 200 MHz | `+1.126 ns` | 0 | 0 | 0 |

200 MHz route 기준 자원은 168 LUT, 294 FF, BRAM 0, DSP 0이다. F2는 단일
Processing clock 블록이므로 ASYNC_REG 0이 정상이다. OOC DRC의 `ZPS7-1`
PS7-required warning과 `HD.CLK_SRC` warning은 parent가 없는 독립 Zynq
모듈 구현에서 발생하는 제한이며, 최종 parent 통합에서 다시 판정한다.

## 7. 재현 명령

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File system_integration/v2/scripts/run_v2_face_tracker.ps1
```

스크립트는 모든 P10..P13 marker, 150/200 MHz WNS, latch와 Critical CDC를
검사한다. 실패 실행은 PASS 근거로 보관하지 않으며 최종 세션에는 source
manifest와 필요한 로그/report만 남긴다.

`face_event_t` 추가 후 B0 회귀도
`signoff_results/sessions/260804183000_v2_motor_position`에서 다시 실행했다.
P00..P04, 150/200 MHz route, latch 0, ASYNC_REG 12, Critical CDC 0이 모두
유지되어 F2 package 확장이 F1 입력 경계를 변경하지 않았음을 확인했다.

## 8. 다음 단계

F3a에서는 설정 유효성과 운용 허가를 혼동하지 않도록 operation/safety state
owner를 먼저 만든다. 이 블록이 reset, RUN/STOP, ARM/DISARM, 외부 permit과
fail-safe 출력을 단독 소유한 뒤에만 F3b `shot_scheduler`가 B1 이벤트를
소비할 수 있다.
