# V2 Checkpoint K1-2 Reg7 Shadow/Active/Physical 종결 보고서

## 1. 목적과 판정

K1-2는 목표 왕복시간이 software Shadow에서 시작해 외부 TDC-GPX Chip의
`Reg7.MTimer[27:15]`까지 이동하는 전체 경로를 하나의 연속 시나리오로
검증한다. 개별 module의 계산 PASS가 아니라 다음 세 값을 같은 transaction
기준으로 비교하는 것이 목적이다.

1. CTL21/22 staging view의 software 후보 Reg7;
2. 마지막 성공 COMMIT의 CTL21/22 Active effective Reg7;
3. CTL23/24를 통해 외부 GPX pin에서 실제 다시 읽은 물리 Reg7.

판정은 **Complete**다. 진행 중 Shadow 변경, 정상 COMMIT 두 번, 범위 오류와
rollback, 복구 COMMIT을 중간 reset 없이 수행했고 두 routine clock profile과
32/64/128-bit AXIS 통합 회귀를 통과했다.

## 2. 시간 기준 계약

| 항목 | 값 | 소유자 | 의미 |
|---|---:|---|---|
| Runtime 시간 단위 | 5 ns | `C_5NS_TICK_RATE_MHZ=200` | CTL12 등 PS 시간 ABI |
| 외부 GPX Tref | 25 ns | `C_GPX_REFERENCE_CLK_MHZ=40` | PCB가 GPX reference pin에 공급 |
| 환산비 | 5 | `C_GPX_REFERENCE_TICK_5NS=200/40` | MTimer 1 tick = 5개의 5 ns tick |
| MTimer 폭 | 13 bit | GPX Reg7[27:15] | 0..8191 |

```text
Reg7.MTimer = ceil(CTL12.TARGET_RANGE_5NS / 5)
실효 목표 왕복시간 = Reg7.MTimer * 25 ns
```

`lidar_build_pkg`가 200 MHz 공통 시간축과 40 MHz GPX Tref를 함께 소유하고,
환산비는 두 상수에서 파생한다. `lidar_config_validator_seq`의 simulation
assertion은 이 비율이 정확한 정수인지 확인한다. 따라서 향후 한쪽 상수만
바꾸어 `/5`가 조용히 틀어지는 유지보수 오류를 막는다.

## 3. 세 값의 층과 접근 권한

| 값의 층 | 읽는 방법 | 의미 | 쓰기 |
|---|---|---|---|
| Shadow/Staging | CTL21 `INDEX=7`, `VIEW_ACTIVE=0`; CTL22 | 다음 COMMIT 후보. 수동 MTimer도 그대로 저장 | 가능 |
| Active effective | CTL21 `INDEX=7`, `VIEW_ACTIVE=1`; CTL22 | CTL12로 MTimer를 자동 대체한 마지막 성공 COMMIT image | 불가 |
| Physical Chip | CTL23 `CAPTURE + 11CC0111`; CTL24 | 실제 28-bit 외부 bus에서 다시 읽은 Chip Reg7 | 진단 portal은 read-only |

첫 성공 COMMIT 전에는 `ACTIVE_VALID=0`이고 Active view가 `0`을 반환한다.
이는 reset 후보 image를 실제 적용값으로 오인하지 않게 하는 안전 계약이다.

## 4. 연속 시나리오

### 4.1 후보 A와 B

| 후보 | CTL12 요청 | software staging MTimer | COMMIT MTimer | 실효 시간 |
|---|---:|---:|---:|---:|
| A | 48 x 5 ns = 240 ns | 1, 의도적으로 잘못 기록 | `ceil(48/5)=10` | 250 ns |
| B | 53 x 5 ns = 265 ns | 8191, 의도적으로 잘못 기록 | `ceil(53/5)=11` | 275 ns |

두 후보는 MTimer 이외의 Reg7 bit도 서로 다르게 만들어 자동 대체가 13-bit
MTimer field에만 적용되고 HSDiv 등 나머지 field는 보존되는지 확인한다.

### 4.2 실행 순서와 기대값

| 순서 | 동작 | 반드시 관측해야 하는 결과 |
|---:|---|---|
| 1 | A를 staging하고 첫 COMMIT 전 view 확인 | staging=A, Active=0, ACTIVE_VALID=0 |
| 2 | A COMMIT이 BUSY인 동안 CTL12와 staging Reg7을 B로 변경 | 진행 transaction snapshot은 A, 다음 Shadow는 B |
| 3 | 첫 COMMIT 완료 | Active/Chip0/Chip1=A effective MTimer 10, staging=B, SHADOW_DIRTY=1 |
| 4 | 두 번째 COMMIT | Active/Chip0/Chip1=B effective MTimer 11, staging 수동값 8191 유지, SHADOW_DIRTY=0 |
| 5 | CTL12=40,956 ticks로 COMMIT | 오류 `0x33`, Active version 불변, 물리 write 0회, Reg7 B 보존 |
| 6 | CTL12를 B로 복구해 COMMIT | 성공, clean Shadow/Active/Physical 일치 후 acquisition 계속 |

40,956 ticks는 표현 가능한 최대 요청 40,955 ticks보다 한 tick 크다. 실패
transaction이 물리 Chip에 일부 register라도 쓰면 원자성 실패이므로, TB는
Chip별 write count가 변하지 않았는지도 검사한다.

## 5. 테스트벤치와 실제 물리 bus 모델

주 검증은 `tb_tdc_gpx_lidar_ctrl_v2_k05.vhd`가 소유한다. 이 TB의 외부 Chip
모델은 각 Chip의 16개 28-bit register를 저장한다. WRN edge에서 실제 Top의
주소와 data bus를 기록하고, RDN edge에서는 저장된 값을 다시 구동한다.
따라서 CTL23/24 비교는 Active image를 내부에서 바로 되돌려 주는 모형이 아니라
실제 `CSN/ADR/WRN/RDN/D[27:0]` 경로를 통과한다.

TB는 Chip 2개, Chip당 STOP 8개, 최대 Return 7개 구성으로 수행한다. K1-2가
끝난 뒤에도 같은 실행에서 IFIFO drain, B5~B8, Rise/Fall, Hole/Footer, AXIS
출력을 계속 검증하므로 설정 검증이 데이터 경로를 훼손하지 않았는지도 본다.

성공 marker는 다음과 같다.

```text
LIDAR_V2_K12_REG7_SHADOW_ACTIVE_PHYSICAL_PASS proc_mhz=150 tdc_mhz=200
LIDAR_V2_K12_REG7_SHADOW_ACTIVE_PHYSICAL_PASS proc_mhz=200 tdc_mhz=150
```

`run_v2_k05_integration.ps1`은 기존 K05 marker와 위 K1-2 marker를 모두 요구한다.
`run_v2_k06_axis_integration.ps1`도 동일 TB를 32/64/128-bit로 반복한다.

## 6. 회귀와 구현 결과

| 검증 | 조합 | 결과 |
|---|---|---|
| K05 Top 연속 Reg7/GPX/data 회귀 | 처리 150/TDC 200 MHz | PASS |
| K05 Top 연속 Reg7/GPX/data 회귀 | 처리 200/TDC 150 MHz | PASS |
| K06 AXIS 통합 | 두 clock profile x 32/64/128 bit | 6/6 PASS |
| 통합 CSR 기능/구현 | 처리 150/TDC 200 MHz | WNS `+0.395 ns`, latch 0, Critical CDC 0 |
| 통합 CSR 기능/구현 | 처리 200/TDC 150 MHz | WNS `+0.608 ns`, latch 0, Critical CDC 0 |
| GPX acquisition lane 구현 | TDC 150 MHz | WNS `+1.532 ns`, latch 0 |
| GPX acquisition lane 구현 | TDC 200 MHz | WNS `+0.541 ns`, latch 0 |
| IP package source/XGUI/catalog/OOC | ASYNC32, ASYNC128, SYNC64 | 3/3 PASS, Black Box 0 |

주요 보존 세션은 다음과 같다.

- `260808_k12_third_v2_k05_integration`
- `260808_k12_axis_v2_k06_axis_integration`
- `260808_k12_csr_final_v2_unified_csr`
- `260808_k12_timing_v2_gpx_acquisition_lane`
- `260808_164618_k010_ip_package`

## 7. 유지보수 규칙

- 목표 왕복시간의 Runtime 원본을 CTL12 외에 추가하지 않는다.
- staging MTimer write를 금지하거나 숨기지 않는다. 저장은 허용하되 COMMIT에서
  CTL12 파생값으로 정확히 대체한다.
- Active view를 물리 readback으로 이름 붙이지 않는다.
- `ACTIVE_VALID=0`의 Active view 0 반환을 기본 image 적용으로 해석하지 않는다.
- COMMIT 중 Shadow write를 현재 snapshot에 섞지 않는다.
- validation 실패가 Active version, Active image 또는 물리 GPX write를
  변경하지 않게 한다.
- 5 ns 공통 주파수와 GPX Tref 상수를 다른 package에 중복 선언하지 않는다.
- Reg7 검증을 단위 계산 TB만으로 대체하지 않고 Top 물리 bus readback까지 유지한다.

## 8. 다음 단계

K1-3에서 RPM, 요청 광학 Shot 간격, 목표 왕복시간, Runtime Return 1..7,
32/64/128-bit, slope topology와 두 routine clock profile을 HTML Golden model과
자동 비교한다. K1-2의 세 층 Reg7 계약은 K1-3의 선행 조건이며 HTML 계산으로
대체하지 않는다.
