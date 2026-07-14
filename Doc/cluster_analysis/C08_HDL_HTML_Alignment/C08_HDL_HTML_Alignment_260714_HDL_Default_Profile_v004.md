# C08 HDL Default Profile v004

- Date: 2026-07-14
- Stage: C08-S7
- Simulator: `C08_HDL_HTML_Alignment_260714_HDL_Default_Profile_Simulator_v007.html`

## 결론

`TDC_GPX_TOP`의 `g_OUTPUT_WIDTH` generic 기본값은 32-bit다.

- `tdc_gpx_top.vhd`: `g_OUTPUT_WIDTH : natural := 32`
- `tdc_gpx_cell_pipe.vhd`: `g_OUTPUT_WIDTH : natural := 32`
- `tdc_gpx_output_stage.vhd`: `g_OUTPUT_WIDTH : natural := 32`

v006까지의 HTML은 C07 검토에서 사용한 128-bit 대표 설정을 HTML 초기값으로 사용했다. 이는 RTL generic 기본값과 비교 프로파일을 혼동한 것이다.

## v007 수정

- `g_OUTPUT_WIDTH` select의 초기 선택값을 32-bit로 변경
- `DEFAULT_PARAMS.values.axisWidth`를 32로 변경
- `HDL_MODEL.defaultOutputWidth = 32` 계약 추가
- effective width 표시에 `default` 또는 `override`를 명시
- 판정표에 `g_OUTPUT_WIDTH default` 행 추가
- 기존 128-bit release baseline 표현을 `C07 comparison profile`로 재명명
- C07 profile의 128-bit는 비교 정보일 뿐 Frame rate 실패 조건이나 HDL 기본값이 아님

## 저장 설정 마이그레이션

v006/v005/v004는 HTML 기본값 자체가 128-bit였으므로 v007이 구버전 저장값을 읽을 때 다음 정책을 적용한다.

- 구버전 저장값이 128-bit이거나 width 필드가 없으면 32-bit로 한 번 교정
- 구버전에서 명시적으로 32/64-bit를 사용했다면 그 값을 보존
- v007에서 사용자가 다시 128-bit를 선택해 저장하면 이후에는 그 override를 보존

이 정책은 새 기본값이 localStorage에 가려져 계속 128-bit로 보이는 문제를 막는다.

## 기본 20 Hz 수치

기본 `max_hits=7`, 450 shots/face, 32 rows/slope, Raw VDMA, Ethernet effective 800 Mbps 조건이다.

- 32-bit raw packet: 1,376 B/shot
- face payload: 619.2 kB
- Ethernet time: 6.192 ms
- face rest: 6.250 ms
- Ethernet margin: 0.058 ms

따라서 RTL 기본 32-bit Raw VDMA 조건은 20 Hz에서 PASS지만 Ethernet 여유가 매우 작다. Compact ABI를 선택하면 전송시간은 5.616 ms이며, downstream repack 구현이 필요하다는 기존 계약은 유지한다.

## 검증 항목

- HTML/JavaScript syntax
- DOM/reference 및 dynamic control wiring
- DOM selected option = 32
- `DEFAULT_PARAMS.values.axisWidth = 32`
- legacy 128 -> 32 migration
- legacy 64 -> 64 preservation
- v007 saved 128 override preservation
- 32/64/128-bit raw/compact packet matrix
