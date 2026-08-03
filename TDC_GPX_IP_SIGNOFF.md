# TDC-GPX IP Sign-off 기록

## 1. 판정

- 대상 IP: `victek.co.kr:my_ip:tdc_gpx_top:1.0`
- 패키지 core revision: `7`
- FPGA: `xc7z020clg484-2`
- 검증 기준일: 2026-08-03
- RTL/IP 패키지 판정: **PASS**
- 실보드 전기·PCB 타이밍 판정: **조건부 보류**

이 문서의 PASS는 패키지 원본 정합성, 합성, RTL 기능, CDC/CSR 구조와 기록된
운용점의 처리 예산을 의미한다. 실제 TDC-GPX 칩, PCB 배선 지연, I/O 전압,
START/STOP 파형을 사용한 보드 검증까지 완료됐다는 의미는 아니다.

## 2. 고정 검증 구성

| 항목 | 값 |
|---|---:|
| AXIS 처리 클럭 | 150 MHz |
| TDC 버스 클럭 | 200 MHz |
| 출력 폭 | 32, 64, 128 bit |
| GPX 칩 수 | 4 |
| slope 구성 | chip 0/1 rising, chip 2/3 falling |
| STOP 수 | 칩당 최대 8 |
| Return 수 | STOP당 최대 7 |
| 외부 GPX 데이터 | 칩당 28 bit, 하위 17 bit가 거리 hit |
| GPX 동작 모델 | I-Mode single measurement |

## 3. 패키지 무결성

- canonical RTL 31개, CSR RTL 11개, XGUI 1개, XML 2개를 포함한 총 45개
  파일을 패키지와 byte 단위로 비교했다.
- XGUI에서 32/64/128-bit만 허용하고 96-bit는 거부되는 것을 확인했다.
- 1-chip dual-edge, 3-chip 2-rise/1-fall, 4-chip 2-rise/2-fall 토폴로지와
  `rise chip 수 >= fall chip 수` 제한을 확인했다.
- 로컬 CSR와 통합 CSR 구성 모두 IP Integrator 속성 검사를 통과했다.

검사 명령:

```powershell
vivado.bat -mode batch -source scripts/check_tdc_gpx_ip_package.tcl
```

## 4. 합성 판정

패키지의 복사 소스만 읽어 OOC 합성했다. 세 출력 폭 모두 포트 폭 일치,
4-chip 토폴로지, 최대 8 STOP/7 Return 설정과 black box 0 조건을 통과했다.

| 출력 폭 | AXIS/TDC | 결과 |
|---:|---:|---|
| 32 bit | 150/200 MHz | PASS |
| 64 bit | 150/200 MHz | PASS |
| 128 bit | 150/200 MHz | PASS |

CSR 소유권도 별도로 합성 확인했다.

| 모드 | 기대 구조 | 결과 |
|---|---|---|
| `g_ENABLE_LOCAL_CSR=true` | 로컬 CSR 1개 | PASS |
| `g_ENABLE_LOCAL_CSR=false` | 로컬 CSR 제거, 통합 CSR 포트 사용 | PASS |

## 5. 기능 회귀 판정

`scripts/run_c06_v002_regression.ps1 -Signoff150200Only`로 다음 항목을 한 번에
검증한다. 이 모드는 150/200 MHz 외 주파수 조합을 실행하지 않는다.

- GPX bus PHY 기본 응답 88개 및 request one-shot 계약
- C01 read/write 캡처 계약
- 통합 CSR 설정 전달
- 32/64/128-bit 출력의 beat 수와 `tlast`
- 세 출력 폭의 shot 경계 40-clock `tready` stall
- 64-bit 출력의 주기적 17-clock bounded backpressure
- masked-slope 입력의 소비·드롭과 `STAT7[15]` sticky/soft-clear

최종 마커는 `TDC_GPX_FUNCTIONAL_SIGNOFF_150_200_PASS`이다.

## 6. 7 Return 물리 처리 예산

Return PASS는 다음 운용점에 한정한다.

| 항목 | 값 |
|---|---:|
| 모터 속도 | 1200 RPM |
| 광학 수평 분해능 | 0.2 deg |
| 최대거리 | 1000 m |
| 주입 목표거리 | 950 m |
| APD 채널 | 16 |
| 물리 LVDS Return | 채널당 1~7 |
| 처리 클럭 | AXIS 150 MHz, TDC 200 MHz |
| 물리 예산 런의 출력 backpressure | 없음 |

시뮬레이션 시간을 줄이기 위해 회전 주기를 200 us로 압축하고 광학 샷 간격을
50 deg로 확대했다. 두 값은 같은 비율로 변환되어 실제 운용점의 point 간 시간
약 13.889 us를 보존한다.

7 Return에서 GPX raw word는 `4 chips x 8 STOP x 7 Return = 224`개다.
32-bit 최악 경계에서도 다음 샷까지 130 AXIS clocks의 여유가 남았다.

| 폭 | fire-to-output 최대 | point budget 여유 | 판정 |
|---:|---:|---:|---|
| 32 bit | 1954 clocks | 130 clocks | PASS |
| 64 bit | 1930 clocks | 154 clocks | PASS |
| 128 bit | 1922 clocks | 162 clocks | PASS |

추가 확인값은 다음과 같다.

- planned shot interval: 2084 AXIS clocks
- measured minimum shot interval: 2083 AXIS clocks
- range wait maximum: 1010 AXIS clocks
- raw bus checks: 2016
- rising hit checks: 1008
- falling hit checks: 1008
- `STAT5=0x00000001`, `STAT6=0xF0000000`, `STAT7=0x00000000`
- schedule overrun: 0

물리 예산 런에는 backpressure를 넣지 않았다. 대신 기능 회귀에서 17-clock
bounded backpressure와 shot 경계를 가로지르는 40-clock stall을 독립 검증했다.
두 결과를 합쳐도 무제한 stall까지 보장하는 것은 아니다.

## 7. 실보드에서 닫아야 할 항목

다음 항목이 완료되기 전에는 제품 전체의 최종 sign-off로 해석하지 않는다.

1. 보드 VCCO와 TDC/STOP I/O 표준 전압 일치 확인
2. PCB GPX 데이터·주소·제어 skew를 포함한 read capture 최소 7.917 ns 보장
3. 실칩 reset, register read/write, Reg12 진단, I-Mode 28-bit 읽기 확인
4. 하위 17-bit 거리값과 chip/STOP/Return/slope 순서 대조
5. 실제 START/STOP, EF/LF/IrFlag/ErrFlag/OEN 파형 확인
6. 1~7 Return 실측 및 16 APD 채널 전수 확인
7. 출력 AXIS에서 VDMA/DDR/Ethernet까지의 장기 backpressure 정책 확인

## 8. 재실행 명령

```powershell
vivado.bat -mode batch -source scripts/run_tdc_gpx_packaged_ip_ooc.tcl
vivado.bat -mode batch -source scripts/run_tdc_gpx_csr_mode_ooc.tcl
powershell.exe -NoProfile -ExecutionPolicy Bypass -File scripts/run_c06_v002_regression.ps1 -Signoff150200Only
powershell.exe -NoProfile -ExecutionPolicy Bypass -File system_integration/scripts/run_return_feasibility_matrix.ps1
```
