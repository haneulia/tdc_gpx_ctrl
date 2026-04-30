const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C04_Output_Stage/C04_Output_Stage_260501035419_Distance_Time_Recheck_v002.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C04 Distance Time Recheck v002";
pptx.title = "C04 Distance Time Recheck v002";
pptx.lang = "ko-KR";
pptx.theme = { headFontFace: "Malgun Gothic", bodyFontFace: "Malgun Gothic", lang: "ko-KR" };

const font = "Malgun Gothic";
const C = {
  bg: "FAFBFD",
  ink: "172033",
  muted: "64748B",
  line: "CBD5E1",
  white: "FFFFFF",
  blue: "2563EB",
  blueFill: "EFF6FF",
  green: "16A34A",
  greenFill: "ECFDF5",
  red: "DC2626",
  redFill: "FEF2F2",
  orange: "EA580C",
  orangeFill: "FFF7ED",
  cyan: "0891B2",
  cyanFill: "ECFEFF",
  slate: "F1F5F9",
  purple: "7C3AED",
  purpleFill: "F5F3FF"
};

function bg(slide) {
  slide.background = { color: C.bg };
}

function addText(slide, value, x, y, w, h, opt = {}) {
  slide.addText(value, {
    x,
    y,
    w,
    h,
    fontFace: font,
    fontSize: opt.size || 11,
    bold: opt.bold || false,
    color: opt.color || C.ink,
    align: opt.align || "left",
    valign: "mid",
    fit: "shrink",
    margin: opt.margin === undefined ? 0.04 : opt.margin,
    breakLine: opt.breakLine || false
  });
}

function title(slide, main, sub) {
  bg(slide);
  addText(slide, main, 0.55, 0.25, 12.2, 0.48, { size: 21, bold: true });
  addText(slide, sub, 0.58, 0.72, 12.1, 0.32, { size: 9.5, color: C.muted });
  slide.addShape(pptx.ShapeType.line, {
    x: 0.55,
    y: 1.08,
    w: 12.2,
    h: 0,
    line: { color: C.line, width: 0.8 }
  });
}

function footer(slide, text) {
  addText(slide, text, 0.65, 6.97, 12.0, 0.23, { size: 8, color: C.muted, align: "center" });
}

function box(slide, value, x, y, w, h, opt = {}) {
  slide.addShape(pptx.ShapeType.roundRect, {
    x,
    y,
    w,
    h,
    rectRadius: 0.06,
    fill: { color: opt.fill || C.white },
    line: { color: opt.line || C.line, width: opt.lineWidth || 1 }
  });
  addText(slide, value, x + 0.08, y + 0.05, w - 0.16, h - 0.1, {
    size: opt.size || 11,
    bold: opt.bold || false,
    color: opt.color || C.ink,
    align: opt.align || "center"
  });
}

function line(slide, x1, y1, x2, y2, color = C.muted, width = 1.4) {
  slide.addShape(pptx.ShapeType.line, {
    x: x1,
    y: y1,
    w: x2 - x1,
    h: y2 - y1,
    line: { color, width, endArrowType: "triangle" }
  });
}

function simpleTable(slide, rows, x, y, widths, rowH, size = 8.8) {
  rows.forEach((row, r) => {
    let curX = x;
    row.forEach((cell, c) => {
      const isHead = r === 0;
      const fill = isHead ? "E2E8F0" : C.white;
      slide.addShape(pptx.ShapeType.rect, {
        x: curX,
        y: y + r * rowH,
        w: widths[c],
        h: rowH,
        fill: { color: fill },
        line: { color: C.line, width: 0.6 }
      });
      addText(slide, cell, curX + 0.04, y + r * rowH + 0.02, widths[c] - 0.08, rowH - 0.04, {
        size,
        bold: isHead,
        align: c === 0 ? "center" : "left"
      });
      curX += widths[c];
    });
  });
}

{
  const s = pptx.addSlide();
  title(s, "150 m 왕복 시간 재검토", "v001은 C04 drain이 측정 창과 완전히 겹친다는 낙관 모델이었다. v002는 stop_tdc 이후 drain 예산을 분리한다.");
  box(s, "물리 시간\n150 m 왕복 = 약 1.0007 us", 0.8, 1.45, 3.0, 0.95, { fill: C.blueFill, line: C.blue, bold: true, size: 13 });
  box(s, "TB 근사\n6671 ps/m -> 1,000,650 ps", 4.15, 1.45, 3.3, 0.95, { fill: C.cyanFill, line: C.cyan, bold: true, size: 13 });
  box(s, "운용 계약\nshot_period = 1.5 x 왕복", 7.8, 1.45, 3.2, 0.95, { fill: C.orangeFill, line: C.orange, bold: true, size: 13 });
  addText(s, "정확한 물리식: round_trip = 2 x distance / c, c = 299,792,458 m/s", 1.05, 2.85, 11.2, 0.32, { size: 12, bold: true });
  simpleTable(s, [
    ["시각", "값", "의미"],
    ["T0", "0 ps", "start_tdc 발생"],
    ["T1", "1,000,650 ps", "150 m 왕복 후 stop_tdc 발생"],
    ["T2", "1,500,975 ps", "1.5 x 왕복 기준 next start_tdc"],
    ["Budget", "500,325 ps", "stop_tdc 이후 C04 drain 가능 시간"]
  ], 1.15, 3.55, [1.4, 2.5, 7.5], 0.48, 10.5);
  footer(s, "근거: tdc_gpx_pkg.vhd:278,281 / tb_tdc_gpx_distance_maxhits_matrix.vhd:44,68");
}

{
  const s = pptx.addSlide();
  title(s, "Timing Diagram", "post-stop 보수 모델: 데이터가 stop_tdc 이후 C04에서 drain된다고 보면 150 m의 실제 여유는 약 500 ns다.");
  const y = 2.2;
  slideTimeAxis(s, y);
  box(s, "start_tdc\n0 ps", 0.8, 1.35, 1.45, 0.75, { fill: C.blueFill, line: C.blue, bold: true });
  box(s, "stop_tdc\n1,000,650 ps", 4.75, 1.35, 1.95, 0.75, { fill: C.orangeFill, line: C.orange, bold: true });
  box(s, "next start\n1,500,975 ps", 8.45, 1.35, 2.0, 0.75, { fill: C.purpleFill, line: C.purple, bold: true });
  line(s, 1.55, 2.1, 5.7, 2.1, C.blue, 2.0);
  addText(s, "측정/왕복 구간: 약 1.00065 us", 2.6, 2.22, 2.5, 0.28, { size: 9.2, color: C.blue, align: "center" });
  line(s, 5.75, 2.55, 9.45, 2.55, C.orange, 2.0);
  addText(s, "post-stop drain budget: 500,325 ps", 6.35, 2.66, 2.95, 0.28, { size: 9.2, color: C.orange, align: "center" });
  box(s, "32bit cfg=6 drain\n933,380 ps\nFAIL", 1.1, 4.0, 2.6, 1.15, { fill: C.redFill, line: C.red, bold: true, size: 12 });
  box(s, "64bit cfg=6 drain\n680,034 ps\nFAIL", 4.25, 4.0, 2.6, 1.15, { fill: C.redFill, line: C.red, bold: true, size: 12 });
  box(s, "128bit cfg=6 drain\n446,689 ps\nPASS", 7.4, 4.0, 2.6, 1.15, { fill: C.greenFill, line: C.green, bold: true, size: 12 });
  addText(s, "핵심: 1 us 안에 6 echo가 가능하다는 표현은 width와 overlap 증명 없이는 성립하지 않는다.", 1.1, 5.75, 11.2, 0.35, { size: 13, bold: true, color: C.ink, align: "center" });
  footer(s, "150 m / shot_period=1.5xRT / output clock=150MHz / full line=4 chips x 8 stops");
}

function slideTimeAxis(slide, y) {
  slide.addShape(pptx.ShapeType.line, {
    x: 0.8,
    y,
    w: 9.8,
    h: 0,
    line: { color: C.ink, width: 1.4 }
  });
  [
    [1.5, "T0"],
    [5.7, "T1"],
    [9.45, "T2"]
  ].forEach(([x, label]) => {
    slide.addShape(pptx.ShapeType.line, {
      x,
      y: y - 0.25,
      w: 0,
      h: 0.5,
      line: { color: C.ink, width: 1.2 }
    });
    addText(slide, label, x - 0.25, y + 0.15, 0.5, 0.25, { size: 9, bold: true, align: "center" });
  });
}

{
  const s = pptx.addSlide();
  title(s, "150 m cfg=6 Width 비교", "C04 drain이 stop_tdc 이후 시작된다고 보면 6 echo 보장은 128bit에서만 닫힌다.");
  simpleTable(s, [
    ["Width", "line beats", "drain time", "drain end", "margin", "판정"],
    ["32", "140", "933,380 ps", "1,934,030 ps", "-433,055 ps", "FAIL"],
    ["64", "102", "680,034 ps", "1,680,684 ps", "-179,709 ps", "FAIL"],
    ["128", "67", "446,689 ps", "1,447,339 ps", "+53,636 ps", "PASS"]
  ], 0.75, 1.55, [1.05, 1.55, 2.0, 2.2, 1.75, 1.15], 0.6, 11);
  box(s, "v001 해석\nC04 drain <= 왕복 시간\n낙관 overlap 참고값", 0.95, 4.35, 3.3, 1.0, { fill: C.slate, line: C.line, bold: true });
  line(s, 4.45, 4.85, 5.35, 4.85, C.muted);
  box(s, "v002 해석\nC04 drain <= stop 이후 예산\n주 판정 기준", 5.55, 4.35, 3.3, 1.0, { fill: C.orangeFill, line: C.orange, bold: true });
  line(s, 9.05, 4.85, 9.95, 4.85, C.muted);
  box(s, "운용 결론\n150m cfg=6은\n128bit만 보수 PASS", 10.15, 4.35, 2.35, 1.0, { fill: C.greenFill, line: C.green, bold: true });
  footer(s, "xsim 근거: line 60~94, cfg=6 파생 계산은 동일 line_beats 기반");
}

{
  const s = pptx.addSlide();
  title(s, "거리 스윕 결과", "FAIL이 발생하면 같은 cfg를 다음 거리에서 재시도했다. 주 판정은 post-stop 모델이다.");
  simpleTable(s, [
    ["Width", "150 m 판정", "cfg=6 최초 PASS", "cfg=7 최초 PASS", "최종 거리", "근거"],
    ["32", "cfg=1도 FAIL", "300 m", "350 m", "350 m", "log:34,50,54,56"],
    ["64", "cfg=1..4 PASS", "250 m", "250 m", "250 m", "log:60,68,74,76,78"],
    ["128", "cfg=1..7 PASS", "150 m", "150 m", "150 m", "log:82,92,94,96"]
  ], 0.75, 1.55, [1.1, 2.2, 2.0, 2.0, 1.5, 2.15], 0.58, 10.3);
  box(s, "32bit\n150m에서는 post-stop 예산 500,325 ps보다 최소 line drain 506,692 ps가 6,367 ps 길다.", 0.9, 4.15, 3.7, 1.15, { fill: C.redFill, line: C.red, size: 10.5, bold: true });
  box(s, "64bit\ncfg=5부터 line drain이 680,034 ps로 증가해 150m/200m에서 실패한다.", 4.85, 4.15, 3.7, 1.15, { fill: C.orangeFill, line: C.orange, size: 10.5, bold: true });
  box(s, "128bit\n모든 cfg가 67 beats로 446,689 ps에 닫혀 150m에서도 여유가 있다.", 8.8, 4.15, 3.7, 1.15, { fill: C.greenFill, line: C.green, size: 10.5, bold: true });
  footer(s, "TB: tb_tdc_gpx_distance_maxhits_matrix.vhd / xsim_distance_maxhits_matrix.log:98 PASS");
}

{
  const s = pptx.addSlide();
  title(s, "운용 계약 제안", "C04만의 serialization PASS와 전체 shot 처리 PASS를 분리해서 말해야 한다.");
  simpleTable(s, [
    ["선택", "의미", "필요 조치"],
    ["보수 모델", "stop_tdc 이후 C04 drain 완료", "v002 경계를 운용 기준으로 사용"],
    ["overlap 허용", "측정 중 C02/C03/C04가 동시에 drain", "end-to-end 시간 추적 TB 추가"],
    ["PRF 완화", "next start_tdc를 더 늦춤", "거리별 shot_period 테이블 보완"],
    ["128bit 채택", "150m cfg=6/7 보수 PASS", "VDMA/메모리 128bit 계약 유지"]
  ], 0.8, 1.45, [1.55, 3.3, 6.55], 0.58, 10.2);
  box(s, "권장 임시 결론", 1.05, 5.0, 2.0, 0.48, { fill: C.blueFill, line: C.blue, bold: true });
  addText(s, "150 m에서 max_hits_cfg=6 이상을 보수적으로 보장하려면 output width는 128bit가 필요하다. 32/64bit는 end-to-end overlap 검증 전까지 150m cfg=6 PASS로 선언하지 않는다.", 3.25, 4.95, 8.8, 0.75, { size: 12.2, bold: true });
  footer(s, "문서: C04_Output_Stage_260501035419_Distance_Time_Recheck_v002.md");
}

pptx.writeFile({ fileName: out });
