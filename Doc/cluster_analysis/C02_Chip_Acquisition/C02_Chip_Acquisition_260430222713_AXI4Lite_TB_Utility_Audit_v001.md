# C02 Chip Acquisition - AXI4-Lite TB Utility Audit

문서 버전: `v001`  
작성/수정 시간: `2026-04-30 22:27:13 +09:00`  
목적: 모든 테스트벤치의 AXI4-Lite write/read transaction이 자체 생성 BFM이 아니라 `px_utility_pkg.vhd` 공통 절차를 사용하도록 규칙을 반영하고, 현재 TB 상태를 점검한다.

---

## 1. 신규 운영 규칙

모든 테스트벤치에서 AXI4-Lite interface로 CSR/register를 쓰거나 읽을 때는 `px_utility_pkg.vhd`에 정의된 공통 procedure를 사용한다.

| 용도 | 기준 procedure | 근거 |
|---|---|---|
| AXI4-Lite write | `px_axi_lite_writer` | `px_utility_pkg.vhd:71-85`, body `px_utility_pkg.vhd:383-463` |
| AXI4-Lite read | `px_axi_lite_reader` | `px_utility_pkg.vhd:90-104`, body `px_utility_pkg.vhd:469-532` |

허용되는 wrapper 범위:

- 여러 AXI4-Lite port bundle 중 어느 module CSR을 접근할지 선택하는 얇은 wrapper는 허용한다.
- wrapper 내부는 `px_axi_lite_writer` 또는 `px_axi_lite_reader` 호출만 수행한다.
- wrapper 내부에서 `awvalid <=`, `wvalid <=`, `arvalid <=`, `bready <=`, `rready <=`, `while awready...` 같은 handshake를 직접 구현하면 위반으로 본다.

운영 프로토콜 반영:

- 신규 파일: `Doc/cluster_analysis/cluster_analysis_260430222713_operating_protocol_v010.md`
- v009 forward-trace: `Doc/cluster_analysis/cluster_analysis_260430201013_operating_protocol_v009.md`의 `v009 -> v010 반영 위치 기록`

---

## 2. 점검 결과

| 파일 | 기존 상태 | 보완/판단 | 근거 |
|---|---|---|---|
| `tb_tdc_gpx_csr_chip_clamp.vhd` | 이미 공통 package 사용 | 변경 없음. write/read 모두 기준 충족 | `tb_tdc_gpx_csr_chip_clamp.vhd:39`, writer `:219`, reader `:303` |
| `tb_tdc_gpx_top_int.vhd` | `pipe_wr`, `chip_wr`가 직접 AXI4-Lite handshake 구현 | wrapper 내부를 `px_axi_lite_writer` 호출로 전환 | package import `:53`, `pipe_wr` `:572-591`, `chip_wr` `:597-616` |
| `tb_tdc_gpx_full_int.vhd` | `axilw_7b`, `axilw_9b`, `pipe_rd`가 직접 AXI4-Lite handshake 구현 | domain별 wrapper `md_wr/lc_wr/er_wr/td_wr/tp_wr/tp_rd`로 전환. 각 wrapper는 package procedure만 호출 | package import `:61`, writer wrappers `:1273-1376`, reader wrapper `:1379-1398` |
| 기타 `tb_tdc_gpx_*.vhd` | AXI4-Lite transaction helper 미검출 또는 AXI-Stream/비AXI TB | 변경 없음 | `Select-String` scan 기준 |

판단:

- 현 시점에서 C02 관련 TB 안에 AXI4-Lite valid/ready handshake를 직접 구현한 write/read helper는 남기지 않았다.
- `pipe_wr`, `chip_wr`, `md_wr`, `lc_wr`, `er_wr`, `td_wr`, `tp_wr`, `tp_rd`는 transaction BFM이 아니라 port bundle 선택용 wrapper다.
- 이후 새 TB에서 read data를 변수로 받아 후속 판단에 써야 한다면, TB 내부 자체 reader를 만들지 말고 `px_utility_pkg.vhd` 확장안을 먼저 문서화해야 한다.

---

## 3. 검증

정적 점검:

- `tb_tdc_gpx_full_int.vhd`에서 기존 `axilw_7b`, `axilw_9b`, `pipe_rd` 직접 handshake 절차가 제거됨.
- `tb_tdc_gpx_top_int.vhd`의 `pipe_wr`, `chip_wr`는 `px_axi_lite_writer`만 호출하도록 변경됨.
- `tb_tdc_gpx_csr_chip_clamp.vhd`는 기존부터 `px_axi_lite_writer`/`px_axi_lite_reader`를 사용 중이다.
- 전체 `tb*.vhd` scan에서 남은 AXI valid/ready 직접 대입은 `tb_tdc_gpx_csr_chip_clamp.vhd:259-263`의 시나리오 종료 후 signal cleanup뿐이며, AXI4-Lite read/write transaction helper가 아니다.

Vivado `xvhdl` 점검:

| 명령 목적 | 결과 |
|---|---|
| `px_utility_pkg.vhd`, `tb_tdc_gpx_top_int.vhd` 분석 | PASS. `tb_tdc_gpx_top_int` entity 분석 완료 |
| motor/laser/echo 의존 package/entity compile order 포함 후 `tb_tdc_gpx_full_int.vhd` 분석 | PASS. `tb_tdc_gpx_full_int` entity 분석 완료 |

남은 검증:

- 본 점검은 VHDL analyze 단계까지 수행했다. 전체 system integration xsim run은 별도 simset/source order 기준으로 수행한다.
- 본 변경은 TB AXI4-Lite transaction driver만 공통 package로 바꾼 것이며 DUT RTL 동작을 변경하지 않는다.

---

## 4. 후속 관리

- 새 테스트벤치 추가 시 review checklist에 `px_utility_pkg.vhd` 사용 여부를 포함한다.
- 기존 테스트벤치에서 AXI4-Lite write/read helper가 추가 발견되면 C02 audit 문서의 다음 버전으로 기록하고 즉시 전환한다.
- `px_axi_lite_reader`가 read value를 caller에게 반환하지 않는 한계가 실제 검증에서 문제가 되면, 자체 helper를 만들지 않고 `px_utility_pkg.vhd` package 확장 계획으로 다룬다.
