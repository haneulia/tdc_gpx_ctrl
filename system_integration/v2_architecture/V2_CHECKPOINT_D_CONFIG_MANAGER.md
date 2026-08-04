# V2 Checkpoint D: Atomic Configuration Manager

## 1. 판정

Checkpoint D의 원자적 설정 관리자와 Processing/TDC 도메인 gateway는
**기능 및 OOC 구조 검증 PASS**이다.

이 판정은 통합 CSR bank나 전체 LiDAR 데이터 경로의 최종 sign-off를 뜻하지
않는다. 이번 단계는 runtime 설정 한 세트가 두 기능 도메인에 서로 다른
버전으로 노출되지 않도록 하는 설정 트랜잭션 경계만 닫는다.

## 2. 구현 구성

| 모듈 | 역할 |
|---|---|
| `lidar_config_manager` | COMMIT snapshot, 계산기 실행, 버전 부여 및 트랜잭션 상태 소유 |
| `lidar_config_gateway` | 한 기능 도메인의 PREPARE/ACTIVATE/RELEASE와 coherent mailbox 수신 |
| `lidar_config_subsystem` | manager와 Processing/TDC gateway 두 개를 연결하는 구조 전용 wrapper |
| `lidar_commit_calculator` | source 설정 검증 및 모든 derived 설정 계산 |

설정 payload는 `lidar_active_config_t` 한 레코드로 전달된다. 이 레코드는
내부 version, 원본 runtime source, 계산된 derived 설정을 함께 가진다.

## 3. 트랜잭션 순서

1. reset 후 `active_valid=0`이며 두 기능 도메인은 비활성 상태다.
2. IDLE에서 COMMIT을 받으면 shadow 전체를 한 clock에 snapshot한다.
3. calculator가 snapshot만 사용하여 검증과 derived 계산을 수행한다.
4. 오류이면 기존 active 설정과 두 도메인을 변경하지 않고 종료한다.
5. 정상 결과에는 내부 version을 부여하고 candidate를 고정한다.
6. PREPARE를 두 gateway에 보내고 두 도메인의 신규 작업 허용을 닫는다.
7. 각 gateway는 로컬 payload 사본을 만든 후 safe point에서 ACK한다.
8. 두 PREPARE ACK가 모이면 ACTIVATE를 보낸다.
9. 두 ACTIVATE ACK가 모이면 RELEASE를 보낸다.
10. 두 RELEASE ACK가 모이면 중앙 active 설정도 같은 candidate로 게시한다.
11. `RELEASE -> ACTIVATE -> PREPARE` 역순으로 요청을 하나씩 내리고 각 ACK
    하강을 확인한 뒤 DONE을 한 clock pulse로 보고한다.

BUSY 중 추가 COMMIT은 실행하지 않고 `CFG_TRANSACTION_BUSY`로 명시적으로
거절한다. 외부 shadow가 BUSY 중 바뀌어도 진행 중인 snapshot에는 영향을
주지 않는다.

## 4. 오류와 복구 정책

| 코드 | 의미 | 정책 |
|---:|---|---|
| `0x71` | COMMIT while BUSY | 현재 트랜잭션 유지, 추가 명령만 거절 |
| `0x72` | PREPARE timeout | 활성화 전 abort, 기존 설정과 enable 복원 |
| `0x73` | gateway protocol fault | 활성화 전에는 abort, 이후에는 복구 잠금 |
| `0x74` | ACTIVATE timeout | 혼합 가능성을 격리하고 `recovery_required=1` |
| `0x75` | RELEASE timeout | 부분 release 가능성을 격리하고 복구 요구 |
| `0x76` | ACK clear/abort-clear timeout | 트랜잭션 제어가 정리되지 않아 복구 요구 |

모든 단계는 하나의 `G_PHASE_TIMEOUT_US`를 사용한다. CSR clock 주파수와
곱하여 clock 수를 만들며, phase가 바뀔 때 counter를 0으로 다시 시작한다.
활성화 이후에는 어느 도메인이 새 설정을 이미 사용했는지 확정할 수 없으므로
자동 rollback을 하지 않는다. 이 경우 coordinated reset 뒤 재-commit해야 한다.

## 5. CDC 계약

manager는 candidate를 PREPARE보다 먼저 등록하고, PREPARE assertion부터 모든
요청과 ACK가 0으로 돌아올 때까지 변경하지 않는다. 각 gateway는 동기화된
PREPARE를 받은 뒤 이 stable bundled payload를 한 번만 로컬 레코드로 캡처한다.
ACTIVATE에서는 비동기 원본을 다시 읽지 않고 로컬 prepared 사본만 사용한다.

각 gateway의 동기화 구성은 다음과 같다.

- CSR -> domain: PREPARE, ACTIVATE, RELEASE 3개, 각 2-FF 동기화
- domain -> CSR: PREPARE ACK, ACTIVATE ACK, RELEASE ACK, FAULT 4개, 각 2-FF 동기화
- 두 gateway 합계: 동기화 경로 14개, `ASYNC_REG` FF 28개

Vivado `report_cdc`는 stable bundled-data mailbox의 1,064-bit payload를
gateway 두 개에서 `CDC-15` warning 2,128건으로 보고한다. 이는 clock-enable
handshake를 인식한 의도된 분류다. 회귀는 이 warning을 기록하면서도
`Critical CDC = 0`을 필수 통과 조건으로 강제한다.

초기 구현은 ACTIVATE 시점에 비동기 source version을 다시 비교하여 active
레코드 전체의 CE에 Critical CDC를 만들었다. 이 중복 비교를 제거하고 prepared
사본만 사용하도록 수정한 뒤 Critical 2,136건이 0건으로 감소했다.

## 6. 기능 검증

동일한 self-checking TB를 다음 비동기 조합에서 실행했다.

| Processing clock | TDC clock | 결과 |
|---:|---:|---|
| 150 MHz | 200 MHz | PASS |
| 200 MHz | 150 MHz | PASS |

검증 항목은 다음과 같다.

- startup inhibit와 첫 atomic commit
- BUSY 거절 및 외부 shadow 변경에 대한 snapshot 원자성
- invalid commit 시 모든 active payload 보존
- Processing safe-point 대기
- PREPARE timeout 후 기존 설정과 enable 복원
- ACTIVATE 중 TDC reset 후 request-level replay
- PREPARE 중 coordinated reset 및 재시작
- ACTIVATE timeout의 recovery lock
- RELEASE timeout의 partial-activation quarantine
- lock 이후 coordinated reset과 version 1부터의 정상 재시작
- `ACTIVATE => PREPARE`, `RELEASE => PREPARE and ACTIVATE` 불변식

기본 정상 commit은 calculator를 포함해 1,118 CSR clocks가 걸렸다. 테스트용
phase timeout은 100 MHz CSR clock에서 1 us, 즉 100 clocks로 확인했다.

## 7. 구현 결과

대상은 `xc7z020clg484-2`, out-of-context post-route 결과다.

| 항목 | Proc 150 / TDC 200 | Proc 200 / TDC 150 |
|---|---:|---:|
| WNS | +1.013 ns | +1.225 ns |
| Inferred latch | 0 | 0 |
| ASYNC_REG FF | 28 | 28 |
| Critical CDC | 0 | 0 |
| Recognized sync paths | 14 | 14 |
| LUT | 1,223 | 1,223 |
| FF | 9,468 | 9,468 |
| BRAM / DSP | 0 / 0 | 0 / 0 |

FF 수가 큰 이유는 source와 derived를 포함한 전체 active record를 manager,
Processing gateway, TDC gateway에 각각 보관하기 때문이다. 기능 도메인별
필드 분할은 향후 면적 최적화 항목이며, 원자성 검증이 끝나기 전에 payload를
잘라내지는 않는다.

## 8. reset 경계와 잔여 제한

다음 reset 동작은 검증했다.

- 모든 도메인의 coordinated reset
- ACTIVATE 진행 중 한 destination reset과 request-level replay
- recovery lock 뒤 coordinated reset 및 재-commit

아직 자동 복구하지 않는 경계가 하나 있다. 트랜잭션이 없는 IDLE 상태에서
Processing 또는 TDC 도메인만 단독 reset하면 해당 gateway는 로컬 active 설정을
잃고 enable을 0으로 내리지만, CSR manager는 이를 자동으로 재배포하지 않는다.
따라서 현재 통합 계약은 기능 도메인 reset을 coordinated reset으로 사용하거나,
단독 reset 뒤 소프트웨어가 다시 COMMIT해야 한다.

향후 reset supervisor를 구현할 때 domain-online/reset-sequence를 manager가
확인하고 마지막 active version을 자동 replay하도록 보완한다. 이 항목이 닫히기
전까지 독립 destination reset 복구는 전체 IP sign-off 범위가 아니다.

OOC의 `HD.CLK_SRC`, `PARTPIN_LOCS`, PS7 요구 warning은 상위 Zynq 프로젝트의
clock buffer, I/O 및 PS7 배치가 없는 검증 경계에서 발생한다. 최종 parent 구현은
실제 clock source와 top-level constraints를 포함해 다시 timing/CDC/DRC를 닫아야 한다.

## 9. 재현 정보

- Tool: Vivado 2025.2.1
- Package regression: `260804135106_v2_config_pkg`
- Manager regression: `260804135927_v2_config_manager`
- 결과: `signoff_results/sessions/260804135927_v2_config_manager`
- 실행: `system_integration/v2/scripts/run_v2_config_manager.ps1`
- PASS marker: `LIDAR_V2_CONFIG_MANAGER_PASS`

## 10. 다음 단계

다음 Checkpoint는 unified AXI4-Lite CSR bank를 이 manager 앞에 연결한다.

1. 32 CTL / 32 STAT 외부 geometry를 유지한다.
2. CTL write는 shadow만 바꾸고 active readback과 분리한다.
3. COMMIT, RESET, CLEAR는 W1S event로 만든다.
4. BUSY, DONE, ERROR, ERROR_CODE, ACTIVE_VERSION을 직접 상태로 노출한다.
5. 이번 Checkpoint의 manager/gateway TB를 CSR write/read 시나리오로 확장한다.
