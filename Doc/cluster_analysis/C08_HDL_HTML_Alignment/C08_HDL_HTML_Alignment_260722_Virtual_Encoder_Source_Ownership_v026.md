# C08 Virtual Encoder Source Ownership v026

## 결정

Virtual Encoder의 canonical source는 다음 저장소다.

```text
C:/Projects/my_sp/lib/IP/virtual_encoder/HDL
```

`motor_decoder/HDL` 안의 `enc_*.vhd`는 Motor Decoder가 내부 시뮬레이션용 Encoder를 포함하던 구조에서 생긴 과도기 복사본이다. 앞으로 Virtual Encoder 기능 변경은 canonical source에서 먼저 구현하고 검증한다.

## 근거

1. 시스템 목표 연결 순서는 `Virtual Encoder -> Motor Decoder -> Laser Controller -> Echo Receiver -> TDC GPX`다.
2. 별도 `virtual_encoder` Vivado 프로젝트와 IP package가 이미 존재한다.
3. 두 디렉터리의 `enc_phase_counter`와 `enc_position_counter`는 동일했지만 package, Top, parameter controller, TB는 서로 다른 시점으로 분기되어 있었다.
4. 복사본을 독립적으로 수정하면 동일 결함을 두 번 수정하거나 서로 다른 동작 계약을 만들 위험이 있다.

## 이번 정렬

- 독립 Virtual Encoder에 통합 `enc_timing_generator`를 반영했다.
- `enc_pkg`, `enc_param_apply_ctrl`, `enc_top`, `enc_top_tb`를 검증된 버전으로 정렬했다.
- `tb_enc_timing_generator`와 `enc_top_tb`가 독립 소스에서 PASS했다.
- 독립 Top 합성 결과는 475 LUT, 569 FF, WNS -1.508 ns였다.
- 최악 경로는 `enc_position_counter`의 position register에서 Z counter enable로 이어지는 경로다.

## 실행기 정리

`run_virtual_encoder_unit.ps1`의 기본 소스 위치를 canonical Virtual Encoder로 변경했다. 테스트마다 필요한 파일만 컴파일한다.

| 테스트 | 소스 소유자 |
|---|---|
| `tb_enc_timing_generator` | Virtual Encoder |
| `enc_top_tb` | Virtual Encoder |
| `tb_motor_cfg_commit_atomic` | Motor Decoder |

## 과도기 정책

1. Position/Z 2차 최적화는 canonical Virtual Encoder에서 수행한다.
2. 통합 회귀가 아직 Motor Decoder 내부 Encoder를 사용하므로, 외부 연결 전까지 검증된 변경을 복사본에도 동기화한다.
3. Parent 통합에서 독립 Virtual Encoder 출력 A/B/Z를 Motor Decoder 입력에 연결한 뒤, Motor Decoder 내부 Encoder와 중복 소스를 제거한다.
4. 기존 XPR 및 `ip_repo` 생성물은 source commit과 분리한다. 새 parent 프로젝트 및 IP 재패키징 단계에서 canonical manifest로 재생성한다.

## Git 체크포인트

- Virtual Encoder: `11ab956 refactor: simplify encoder timing path`
- Motor Decoder: `e89c7ae refactor: simplify virtual encoder timing`
- Integration verification: `a3612d0 test: verify virtual encoder timing optimization`
