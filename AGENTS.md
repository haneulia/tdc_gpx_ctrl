# TDC-GPX LiDAR 작업 용어 규칙

이 저장소에서 사용자에게 설명하거나 보고서를 작성할 때 다음 규칙을 따른다.

1. 의미를 합쳐 줄이지 말고, 처음에는 정식 물리 의미와 RTL/CSR 이름을 함께 쓴다.
   예: `레이저 목표 왕복시간 (TARGET_RANGE_WINDOW_5NS)`.
2. `물리 시간`, `레이저 시간`, `거리 설정`, `각도 설정`처럼 여러 계약을 가리킬 수
   있는 포괄 표현을 단독으로 사용하지 않는다.
3. 사용자가 입력하는 거리는 `레이저 목표 편도 거리 (R)`라고 쓴다. RTL이 저장하고
   사용하는 시간은 `레이저 목표 왕복시간 (2R/c, TARGET_RANGE_WINDOW_5NS)`이라고 쓴다.
4. `FIRE_DONE_TIMEOUT_5NS_TICKS`는 `Fire 명령 후 fire_done 수신 최대 대기시간`으로
   설명한다. 레이저 목표 왕복시간과 같은 뜻으로 쓰지 않는다.
5. `OPTICAL_SHOT_INTERVAL_UDEG`는 `인접한 레이저 발사 후보점 사이의 요청 광학각`으로
   설명한다. 실제 발사 보장은 모터 속도, 실행기 Busy, 왕복시간 및 재무장 여유와 함께
   판단한다고 명시한다.
6. `BUS_CLK_DIV`와 `BUS_TICKS`는 `Runtime TDC-GPX 버스 읽기 타이밍`으로 설명한다.
   Processing/AXIS 속도나 레이저 왕복시간과 혼용하지 않는다.
7. 다음 Generic 묶음은 반드시 개별 이름과 목적을 적는다.
   - `G_POWERUP_TIME_NS`: TDC-GPX 초기 전원/리셋 후 대기시간
   - `G_RECOVERY_TIME_NS`: TDC-GPX 제어 펄스 후 회복 대기시간
   - `G_ALU_PULSE_TIME_NS`: ALUTrigger 펄스 폭
   - `G_BUS_IDLE_STABLE_TIME_NS`: 버스 장애 후 유휴 안정 판정시간
   - `G_DRAIN_MARGIN_TIME_NS`: 목표 왕복시간 뒤 IFIFO Drain watchdog 추가 여유시간
   - `G_PHASE_TIMEOUT_US`: 통합 설정 PREPARE/ACTIVATE/RELEASE 단계 제한시간
8. `측정 시작 기준시점 (T0)`은 물리 모드에서 동기화된 `fire_done`을 승인하고
   `start_tdc`를 발생시키는 사건으로 설명한다. 약어만 단독으로 먼저 쓰지 않는다.
9. TDC-GPX `MTimer`와 `TARGET_RANGE_WINDOW_5NS`는 별도 설정원임을 명시한다.
   둘의 일치가 RTL에서 자동 보장되는지, 소프트웨어 계약인지 항상 구분해서 보고한다.

