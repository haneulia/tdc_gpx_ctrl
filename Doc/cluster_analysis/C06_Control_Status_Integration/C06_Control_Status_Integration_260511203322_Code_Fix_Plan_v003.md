# C06 Control/Status Integration Code Fix Plan v003

| 항목 | 내용 |
|---|---|
| 문서 버전 | v003 |
| 생성 시간 | 2026-05-11 20:33:22 KST |
| 수정 시간 | 2026-05-11 20:33:22 KST |
| Cluster | C06 Control / Status Integration |
| 절대 기준 문서 | `Doc/TDC-GPX-Datasheet.pdf` |
| 선행 계획 | `C06_Control_Status_Integration_260511200655_Code_Fix_Plan_v002.md` |
| 선행 결과 | `C06_Control_Status_Integration_260511203322_Code_Fix_Result_v002.md` |
| 목적 | v002에서 남은 recovery, project syntax policy, pipeline IRQ 정책 결정을 닫고 C06 handoff 가능 여부를 판단한다. |

## 1. 결론

v003의 핵심은 새 기능 추가가 아니라 C06 완료 판정을 위해 남은 운영 계약을 닫는 것이다. 가장 중요한 항목은 top-level recovery 검증이다.

## 2. v002 승계 항목

| v003 ID | v002 원인 | 우선순위 | 목표 |
|---|---|---:|---|
| FP3-C06-01 | `FP2-C06-08` Partial | P1 | AXI4-Lite CSR write 기반 `soft_reset`/`force_reinit` top recovery 검증 |
| FP3-C06-02 | `FP2-C06-01` Accepted Environment Exception | P2 | project syntax flow를 `.xpr` 갱신으로 닫을지 direct compile/elab 회귀로 표준화할지 결정 |
| FP3-C06-03 | `FP2-C06-06` Accepted Exception | P2 | `o_irq_pipe` reserved 유지 또는 dedicated pipeline IRQ 요구사항 분리 |

## 3. Datasheet 기준

| Datasheet 근거 | 위치 | v003 적용 |
|---|---|---|
| Single measurement sequence 후 master reset이 필요하다. | PDF page 29, section `2.11.1 Single measurement` | recovery 검증은 측정 완료 후 재초기화/재시작 가능성을 확인한다. |
| IFIFO empty read 금지 | PDF page 27 | recovery command 이후 다음 run에서 empty read 없이 정상 drain 되는지 확인한다. |
| I-Mode single 운용 | PDF page 25, section `2.3 I-Mode Basics` | Quiet/M-mode 또는 continuous mode는 범위 제외. |

## 4. 작업 계획

### FP3-C06-01: CSR recovery top regression

목표:

1. `tb_tdc_gpx_top_int.vhd` 또는 별도 focused TB에서 정상 run을 1회 완료한다.
2. `px_utility_pkg.vhd`의 AXI4-Lite write/read helper를 사용해 `soft_reset` 또는 `force_reinit` command를 주입한다.
3. reset/reinit 이후 두 번째 run을 수행한다.
4. output beat/tlast, STAT5/6/7, IRQ no-spurious를 재검증한다.

PASS 기준:

| 항목 | PASS 기준 |
|---|---|
| hard reset | 기존 TB와 동일하게 PASS |
| soft_reset | command 이후 deadlock 없이 다음 run PASS |
| force_reinit | command 이후 chip/control 상태가 재진입 가능 |
| status | recovery 후 fatal/timeout/sequence error가 새로 발생하지 않음 |
| output | 두 번째 run에서도 expected beats/tlast 보존 |

### FP3-C06-02: project syntax policy

선택지:

| 선택지 | 장점 | 단점 | 권고 |
|---|---|---|---|
| `.xpr` source list 영구 갱신 | Vivado GUI/project flow와 일치 | `.xpr`가 HDL writable root 밖에 있으면 별도 승인/관리 필요 | 사용자가 project file 관리까지 원하면 선택 |
| direct compile/elab 회귀 표준화 | repo-local script로 재현 가능, CI 친화적 | Vivado GUI compile-order warning은 남을 수 있음 | 현재 권고 |

### FP3-C06-03: pipeline IRQ policy

현재 설계는 `o_irq_pipe`를 reserved로 두고 STAT5/6/7 readback으로 fault observability를 제공한다. dedicated pipeline IRQ가 필요하면 다음을 별도 요구사항으로 정의해야 한다.

| 결정 항목 | 선택지 |
|---|---|
| IRQ source | fatal only / any sticky fault / run complete / reserved 유지 |
| clear | AXI4-Lite read clear / explicit soft clear / hard reset only |
| SW 계약 | polling 중심 / interrupt 중심 |

v003 기본 권고는 reserved 유지다. C06 handoff 문서에는 "pipeline fault는 STAT5/6/7 polling으로 확인"을 명시한다.

## 5. Timing / Latency / Throughput / Pipeline / II 영향 예측

| 항목 | 예상 영향 |
|---|---|
| Latency | recovery command는 normal data path latency에 직접 영향 없음 |
| Throughput | run 사이 recovery gap만 증가, run 내부 throughput은 v002와 동일 |
| Pipeline | recovery command가 상태 register boundary를 통과하므로 stale state 제거 여부가 핵심 |
| II | 정상 shot-to-shot II가 아니라 run-to-run 재진입 II를 계측해야 함 |
| Timing diagram | 정상 run -> recovery command -> 다음 run의 T0~T6를 한 그림에 표시 |

```mermaid
flowchart LR
    A["Run #1 PASS"] --> B["AXI4-Lite recovery command"]
    B --> C["status/idle 확인"]
    C --> D["Run #2 start"]
    D --> E["T0~T6 재계측"]
    E --> F["output/status/IRQ PASS"]
```

## 6. v003 검증 Matrix

| 검증 ID | 연결 작업 | 시나리오 | PASS 기준 |
|---|---|---|---|
| V3-C06-01 | FP3-C06-01 | normal run -> soft_reset -> normal run | 두 번째 run output/status PASS |
| V3-C06-02 | FP3-C06-01 | normal run -> force_reinit -> normal run | chip/control 재진입 PASS |
| V3-C06-03 | FP3-C06-01 | recovery 후 T0~T6 marker | marker 누락 없음 |
| V3-C06-04 | FP3-C06-01 | recovery 후 IRQ counter | no-spurious 또는 정의된 IRQ만 발생 |
| V3-C06-05 | FP3-C06-02 | compile policy | 선택한 flow의 재현 명령과 근거 문서화 |
| V3-C06-06 | FP3-C06-03 | IRQ policy | reserved 유지 또는 dedicated IRQ 요구사항 확정 |

## 7. 결과 문서 작성 규칙

v003 실행 후 다음 파일을 생성한다.

| 산출물 | 파일명 규칙 |
|---|---|
| Result Markdown | `C06_Control_Status_Integration_<YYMMDDHHMMSS>_Code_Fix_Result_v003.md` |
| Result PPT | `C06_Control_Status_Integration_<YYMMDDHHMMSS>_Code_Fix_Result_v003.pptx` |
| Handoff 판단 | 모든 항목 PASS/Accepted Exception이면 C06 handoff 문서 생성 |

## 8. 진행 판정

판정: v003은 진행 가능하다. 우선순위는 `FP3-C06-01`이며, 사용자의 별도 지시가 없으면 `o_irq_pipe`는 reserved 유지 정책으로 문서화한다.
