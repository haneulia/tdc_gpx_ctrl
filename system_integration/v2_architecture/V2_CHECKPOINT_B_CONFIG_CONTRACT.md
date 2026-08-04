# V2 Checkpoint B: Configuration Contract

## 1. 판정

Checkpoint B의 설정 데이터 모델과 독립 참조 계산 모델은 **PASS**이다.
다만 이 판정은 설정 계약의 산술·예외 조건을 확정한 것이며, 실제 합성
가능한 순차형 commit calculator의 sign-off를 의미하지 않는다.

## 2. 확정된 구조

| 항목 | 구현 | 역할 |
|---|---|---|
| Build configuration | `lidar_build_pkg.vhd` | 합성 전에 정해지는 칩·클럭·출력폭·slope topology |
| Runtime source configuration | `lidar_config_types_pkg.vhd` | CSR에서 입력되는 원본 물리 설정값 |
| Derived configuration | `lidar_config_types_pkg.vhd` | commit 시 한 번 계산하여 각 기능 블록에 전달할 값 |
| Reference arithmetic | `lidar_config_reference_pkg.vhd` | 테스트용 정답 모델이며 합성 데이터 경로에서 사용 금지 |

물리적으로 동일한 값은 한 곳에서만 입력받는다. 예를 들어 목표 거리
시간은 `target_range_5ns` 한 값으로 관리하고, processing/TDC clock 수는
derived 값으로 생성한다. Present-chip mask는 칩 수에서 자동 도출한다.

각도는 부동소수점 하드웨어 대신 microdegree 정수로 표현한다. 미러의
광학 반사 이득 2는 이름 있는 상수로 분리하여, 의미가 불명확한 숫자
`720`을 직접 사용하지 않는다.

## 3. 핵심 계약

- 지원 clock: 50, 100, 125, 150, 200 MHz
- 동기 모드: processing clock과 TDC clock 수치가 같아야 함
- 출력 폭: 32, 64, 128 bit
- TDC chip: 1..4, STOP/chip: 1..8, Return/STOP: 1..7
- Falling 비활성: 모든 활성 chip을 rising 경로로 승격
- Falling 활성: `rise chip 수 >= fall chip 수`
- 1-chip dual-edge와 3-chip 2-rise/1-fall 구성 허용
- Face 수: 1..5, face half-width는 공통 설정
- Face 경계는 lower/upper 모두 inclusive
- 스캔 lattice는 `[entry, exit)`로 정의하여 양 끝의 동일 지점 중복 발사 방지
- `columns = ceil(2 * half_width / shot_interval_states)`
- 요청 광학 shot angle은 한 state의 광학각보다 작을 수 없음
- `fire_done_timeout_5ns <= target_range_5ns`
- 5 ns CSR 시간은 각 clock domain에서 ceil 방식으로 clock 수로 변환

## 4. 기본값 계산 결과

CPR 3600, x4 decode, 5 faces, 공통 half-width 1200 states,
요청 광학 shot 간격 0.05 degree 기준 결과는 다음과 같다.

| 결과 | 값 |
|---|---:|
| Decoded states/revolution | 14,400 |
| Mechanical angle/state | 0.025 degree |
| Optical angle/state | 0.050 degree |
| Inclusive positions/face | 2,401 |
| Angular intervals/face | 2,400 |
| Shot interval | 1 state |
| Output columns/face | 2,400 |
| 288 x 5 ns at 150 MHz | 216 clocks |
| 288 x 5 ns at 200 MHz | 288 clocks |

`2,401 positions`와 `2,400 intervals`는 모순이 아니다. 양 끝 position을
포함하면 위치는 2,401개지만, 인접 위치 사이의 각도 간격은 2,400개다.
레이저 열 수는 간격 기준이므로 기본값은 2,400 columns이다.

## 5. 검증 범위

Self-checking TB는 정상 기본값과 다음 오류를 확인한다.

- 불법 clock, 동기 clock 불일치, 불법 출력 폭과 face 수
- 존재하지 않는 chip mask bit, slope 미지정 chip, rise/fall 수 위반
- CPR·ticks·Z 설정 범위 및 face window overlap
- 빈 face mask, 1-state 미만 shot angle
- timeout/range 관계 및 signed capture 보정 underflow/overflow
- runtime active-chip mask, max hits, bus divider/ticks 범위
- shot 간격 ceil 양자화, wrap-around face 경계, 방향 독립 geometry

## 6. 실행 결과

- Tool: Vivado Simulator 2025.2.1
- Run: `260804120235_v2_config_pkg`
- Marker: `LIDAR_V2_CONFIG_TYPES_PASS`
- Compile/elaboration/simulation: PASS
- WDB: 생성하지 않음
- 보관 위치: `signoff_results/sessions/260804120235_v2_config_pkg`

## 7. 다음 단계

다음 Checkpoint C에서는 실제 합성 가능한 순차형 commit calculator와
configuration manager를 구현한다. 계산기는 CSR shadow 값을 직접 배포하지
않고 `validate -> derive -> prepare -> acknowledge -> activate` 순서로 처리한다.
그 결과를 이 Checkpoint B reference package와 cycle-independent 비교한다.
