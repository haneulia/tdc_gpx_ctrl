# C07 Claude Simulator Review v001

| 항목 | 내용 |
|---|---|
| 문서 종류 | 외부 작성 HTML 비교 검토 |
| 문서 버전 | v001 |
| 생성 시간 | 2026-07-08 19:34:29 KST |
| 검토 대상 | `Doc/cluster_analysis/C07_System_Integration/C07_System_Integration_260708130837_Polygon_Mirror_Optical_Timing_Simulator_v001_by_Claude.html` |
| 기준 파일 | `Doc/cluster_analysis/C07_System_Integration/C07_System_Integration_260708130837_Polygon_Mirror_Optical_Timing_Simulator_v001.html` |

## 1. 결론

Claude 버전은 전체 방향이 좋다. 현재 C07에서 합의한 핵심 모델인 `고정 180도 광원`, `고정 광원->face center`, `Q2 gate`, `active window`, `PRF/화각/분해능 coupling`이 대부분 반영되어 있다.

다만 최신 기준 파일과 비교하면 Claude 버전은 두 가지 edge case가 약하다.

| 항목 | 판단 |
|---|---|
| 4각/5각 polygon 기본 동작 | 대체로 양호 |
| 1각/2각 flat mirror | 기준 파일이 더 안전 |
| occlusion edge 판정 | 기준 파일이 더 안전 |
| PRF/active window coupling | 양호 |
| 고정 광원 180도 모델 | 양호 |

따라서 `by_Claude.html`을 그대로 기준본으로 교체하지 말고, 현재 기준 HTML을 유지하는 것이 좋다.

## 2. 정적 검증

두 파일 모두 JavaScript syntax는 통과했다.

```text
C07..._v001.html            : JS syntax OK
C07..._v001_by_Claude.html  : JS syntax OK
```

파일 diff는 매우 작다.

```text
1 file changed, 2 insertions(+), 6 deletions(-)
```

즉, 구조적으로는 거의 같은 파일이고 판단 차이는 일부 기하 edge case에 집중된다.

## 3. 차이점

### 3.1 Occlusion edge 판정

Claude 버전:

```js
if (!rayHit || rayHit.edgeIdx !== targetFace.edgeIdx) {
  incidentQ2Allowed = false;
}
```

기준 파일:

```js
if (!rayHit || (verts.length > 2 && rayHit.edgeIdx !== targetFace.edgeIdx)) {
  incidentQ2Allowed = false;
}
```

판단:

1각/2각 flat mirror는 segment가 2-vertex로 표현되며, `findRayHit()`가 같은 물리 segment를 edge 0 또는 edge 1로 반환할 수 있다. Claude 버전은 이 경우를 다른 edge로 오판해서 laser gate를 OFF로 만들 수 있다.

기준 파일처럼 `verts.length > 2`일 때만 edgeIdx 일치를 강제하는 편이 안전하다.

### 3.2 1각/2각 flat mirror tangent 정렬

Claude 버전:

```js
const a = rotationDeg * Math.PI / 180;
```

기준 파일:

```js
const a = (rotationDeg + 45 - 180 / m.facets) * Math.PI / 180;
```

판단:

Claude 버전은 1각/2각 flat mirror에서 단순 회전각을 그대로 mirror tangent로 사용한다. 기준 파일은 polygon 모델의 active-center 조건과 45..135도 target sector 중심을 맞추기 위해 offset을 둔다.

따라서 4각/5각만 볼 때는 큰 차이가 작지만, 사용자가 요청한 1각/2각 모드까지 검토하려면 기준 파일이 더 일관적이다.

## 4. 좋은 점

Claude 버전은 다음 항목을 잘 반영했다.

| 항목 | 평가 |
|---|---|
| 고정 광원 180도 | 좋음 |
| face center 지향 | 좋음 |
| Q2 gate | 좋음 |
| active window와 laser gate 연결 | 좋음 |
| Timing / PRF coupling 진단표 | 좋음 |
| 하단 Optical diagnostics | 좋음 |

특히 `laserFired = scan.active && Q2` 형태로 묶은 방향은 현재 모델에서 맞다.

## 5. 권고

| 선택지 | 권고 |
|---|---|
| Claude 파일을 그대로 채택 | 비권장 |
| 현재 기준 파일 유지 | 권장 |
| Claude 파일을 참고용으로 보관 | 가능 |
| Claude 파일의 UI/표현 중 좋은 부분만 수동 반영 | 이미 대부분 반영됨 |

최종 판단:

```text
현재 기준 파일이 Claude 버전보다 최신이고, 1각/2각 및 occlusion edge case에 더 안전하다.
```
