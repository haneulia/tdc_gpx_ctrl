# C08 Physical Chip Pin Contract Closure v019

## 1. 결론

`tdc_gpx_top`의 chip 수 의미를 다음 두 축으로 분리하고 검증했다.

- `c_MAX_CHIPS = 4`: CSR, header, status, 내부 array가 공유하는 고정 논리 chip-slot ABI 상한
- `g_NUM_CHIPS = 1..4`: 합성 시 결정하는 실제 외부 TDC-GPX 물리 lane 수와 top port 폭
- `g_PRESENT_CHIP_MASK`: 4-slot 논리 ABI에서 구현할 chip ID 선택
- 필수 불변식: `g_NUM_CHIPS = popcount(g_PRESENT_CHIP_MASK)`

따라서 1-chip build는 4-chip 핀을 요구하지 않는다. `io_tdc_d`, `o_tdc_adr`, 모든 TDC control/status vector는 `g_NUM_CHIPS`에 비례해 축소된다. 논리 metadata ID는 sparse mask에서도 보존된다.

## 2. 전수 조사 결과

2026-07-22 현재, `backup`, simulation 결과, sign-off 결과를 제외한 활성 VHDL을 조사했다.

| 항목 | 결과 |
|---|---:|
| `c_MAX_CHIPS` 사용 | 647회 / 33파일 |
| 제품 RTL | 16파일 |
| Testbench | 17파일 |
| 활성 VHDL의 `c_N_CHIPS` | 0건 |

남은 `c_MAX_CHIPS` 사용은 다음 고정 논리 계약에 해당한다.

- 4-bit chip mask와 chip ID
- CSR/header/status record와 packed field
- stop-event 입력의 4-slot ABI
- per-chip 내부 array와 disabled logical slot
- 최대 geometry와 정적 legality bound

외부 sibling `echo_receiver_top`의 formal generic 이름 `g_N_CHIPS`는 해당 IP 인터페이스 소유이므로 변경하지 않았다. `tb_tdc_gpx_full_int`의 중복 로컬 chip-count 별칭은 제거하고 `c_MAX_CHIPS`를 직접 연결했다. 실제 parent가 echo receiver를 포함할 때는 이 formal에 `g_NUM_CHIPS` 또는 동일 build-profile 값을 전달해야 한다.

## 3. 구현 계약

### 3.1 외부 포트 폭

```text
io_tdc_d width       = 28 * g_NUM_CHIPS
o_tdc_adr width      =  4 * g_NUM_CHIPS
control/status width =      g_NUM_CHIPS
```

Vivado IP Integrator module-reference parser는 사용자 함수를 port range에서 처리하지 못했다. 따라서 `fn_physical_chip_count(g_PRESENT_CHIP_MASK)`를 port 범위에 직접 쓰지 않고 단순 정수 generic `g_NUM_CHIPS`를 사용하며, top과 `tdc_gpx_config_ctrl`의 elaboration assertion이 mask popcount 일치를 강제한다.

### 3.2 Sparse mapping

물리 lane은 Present mask의 낮은 논리 chip ID부터 연속 배치된다.

| Present | 물리 mapping | metadata/CSR chip ID |
|---|---|---|
| `0001` | P0 -> C0 | 0 |
| `0101` | P0 -> C0, P1 -> C2 | 0, 2 |
| `0111` | P0/P1/P2 -> C0/C1/C2 | 0, 1, 2 |
| `1111` | P0/P1/P2/P3 -> C0/C1/C2/C3 | 0, 1, 2, 3 |

`tdc_gpx_config_ctrl`이 compact lane adapter와 IOBUF를 소유한다. `tdc_gpx_bus_phy`는 split `i_d/o_d/o_d_tri` 인터페이스만 사용한다. Present가 아닌 논리 slot은 reset 상태로 고정되고 물리 IOBUF가 생성되지 않는다.

## 4. 검증 결과

### 4.1 Xsim 기능 회귀

`scripts/run_c06_v002_regression.ps1` 전체 회귀를 stamp `260722_physical_pin_num_generic`으로 실행했고 정상 종료했다.

| Profile | Xsim 확인 핀 폭 | 결과 |
|---|---|---|
| 1-chip dual edge | D=28, ADR=4, control=1 | PASS |
| sparse 2-chip Rise-only `0101` | D=56, ADR=8, control=2 | PASS |
| 3-chip 2R+1F | D=84, ADR=12, control=3 | PASS |
| 4-chip profiles, 32/64/128-bit | D=112, ADR=16, control=4 | PASS |

기존 CSR, slope topology, backpressure, sticky status, 5 ns tick conversion, line-packer 폭 회귀도 함께 통과했다.

### 4.2 OOC synthesis

Part는 `xc7z020clg484-2`, AXIS/TDC clock은 150/200 MHz, output width는 32-bit다.

| Build | Present / Rise / Fall | D / ADR / control | Timing |
|---|---|---:|---|
| 1-chip dual | `0001 / 0001 / 0001` | 28 / 4 / 1 | PASS |
| sparse 2-chip Rise-only | `0101 / 0101 / 0000` | 56 / 8 / 2 | PASS |
| 3-chip 2R+1F | `0111 / 0011 / 0100` | 84 / 12 / 3 | PASS |
| 4-chip default | `1111 / 0011 / 1100` | 112 / 16 / 4 | PASS |

각 session의 `physical_port_contract.txt`가 15개 TDC port 폭을 netlist에서 직접 확인했고, 네 timing report 모두 `All user specified timing constraints are met.`를 기록했다.

### 4.3 Parent reference

새 parent reference에서 다음을 확인했다.

- `xc7z020clg484-2`, PS FCLK 100/150/200 MHz
- top/parent generic parity 26/26 PASS
- `g_NUM_CHIPS` 전달 PASS
- 4-chip 외부 포트: D `[111:0]`, ADR `[15:0]`, control/status `[3:0]`
- block-design validation 및 parent contract 89개 PASS

검증 session: `parent_ref/results/sessions/260722_physical_pin_parent_validate_v3_ps_fclk_parent_ref`

### 4.4 HTML S17

`C08_HDL_HTML_Alignment_260722_Physical_Chip_Pin_Contract_Simulator_v017.html`에 다음을 반영했다.

- `g_NUM_CHIPS` 편집과 Present popcount 불일치 CHECK
- 1/2/3/4-chip 및 sparse `0101` compact mapping 자가시험
- D/ADR/control 핀 폭과 lane mapping 표시
- 구버전 저장 설정의 `g_NUM_CHIPS` 자동 이관
- desktop 및 실제 390 px mobile viewport 렌더링

Chrome 실행 결과 slope/clock/physical-pin 자가시험 모두 PASS였다. 390 px viewport에서 `clientWidth=390`, `scrollWidth=390`으로 가로 overflow가 없었다.

## 5. Git checkpoint

- `40029b1 feat: scale TDC physical pins by chip count`
- `ba1c2b8 refactor: clarify logical chip slot ownership`

## 6. 남은 시스템 통합 책임

RTL physical-pin contract는 닫혔다. 다만 실제 board parent와 XDC가 아직 없으므로 다음은 시스템 sign-off 전 필수다.

1. build profile별 `g_NUM_CHIPS`, Present/Rise/Fall mask를 하나의 설정 묶음으로 고정한다.
2. sparse lane mapping을 schematic와 XDC pin assignment에 명시한다.
3. GPX I/O standard, bank voltage, drive/slew, input/output delay를 실제 PCB와 datasheet 기준으로 설정한다.
4. parent가 echo receiver를 포함하면 PD/STOP channel 수와 `g_NUM_CHIPS`를 동일 build profile에서 파생한다.

이 네 항목은 현재 RTL 기능 결함이 아니라 아직 존재하지 않는 실제 board integration의 closure 항목이다.
