# C02_Chip_Acquisition Code Fix Result v002

문서 버전: `v002`  
작성일: `2026-04-30`  
최종 수정 시간: `2026-04-30 15:23:18 +09:00`  
작성 목적: `C02_Chip_Acquisition_Code_Fix_Result_20260430_v001.pptx`에서 한글이 `??`로 보이는 문제를 반영해, PPT를 UTF-8 안전 생성 경로로 재생성하고 추적 근거를 남긴다.

---

## 1. 수정 사유

사용자 검토에서 `C02_Chip_Acquisition_Code_Fix_Result_20260430_v001.pptx`의 한글 글자가 `??`로 표시되는 문제가 확인되었다.

판단:

- Markdown `C02_Chip_Acquisition_Code_Fix_Result_20260430_v001.md`는 UTF-8 한글이 정상이다.
- PPT v001은 PowerShell inline script 경로를 통해 생성되었고, 이 과정에서 한글 문자열이 안전하게 보존되지 않았을 가능성이 높다.
- 따라서 동일한 기술 결과를 유지하되, PPT 생성 스크립트를 UTF-8 파일로 저장한 뒤 Node.js에서 직접 실행하는 방식으로 `v002` PPT를 새로 생성했다.

---

## 2. 산출물

| 산출물 | 내용 |
|---|---|
| `C02_Chip_Acquisition_Code_Fix_Result_20260430_v002.md` | 본 추적 문서 |
| `C02_Chip_Acquisition_Code_Fix_Result_20260430_v002.pptx` | 한글 UTF-8 재생성 PPT |

---

## 3. 기술 내용 변경 여부

RTL/TB 기술 내용은 `v001`과 동일하다.

| 항목 | 상태 |
|---|---|
| count-known expected hard bound | 변경 없음 |
| count-unknown EF fallback guard | 변경 없음 |
| empty IFIFO read strict monitor | 변경 없음 |
| raw AXI `tuser` contract monitor | 변경 없음 |
| xsim 결과 | `tb_tdc_gpx_chip_ctrl` PASS, `total_raw_words=231` |

---

## 4. PPT 한글 확인 방법

PPT 내부 XML을 zip으로 풀어 확인했을 때, slide XML에 다음 한글 문자열이 직접 포함되어야 한다.

```text
C02 보완 결과 v002
결론
근거
검증
변경 파일과 역할
```

이 확인은 PowerPoint 렌더링 전에 파일 내부 문자열이 깨지지 않았는지 보는 1차 검증이다.

---

## 5. Lineage

| 이전 문서 | 본 문서 반영 |
|---|---|
| `C02_Chip_Acquisition_Code_Fix_Result_20260430_v001.md` | 기술 내용 유지 |
| `C02_Chip_Acquisition_Code_Fix_Result_20260430_v001.pptx` | 한글 깨짐 문제로 PPT만 `v002` 재생성 |
| `C02_Chip_Acquisition_Code_Fix_Plan_20260430_v004.md` | 계획/검증 근거 유지 |
