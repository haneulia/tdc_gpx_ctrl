const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C04_Output_Stage/C04_Output_Stage_260501042543_Polygon_Budget_Sweep_Result_v001.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C04 Polygon Budget Sweep Result v001";
pptx.title = "C04 Polygon Budget Sweep Result v001";
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

function text(slide, value, x, y, w, h, opt = {}) {
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
    margin: opt.margin === undefined ? 0.04 : opt.margin
  });
}

function title(slide, main, sub) {
  bg(slide);
  text(slide, main, 0.55, 0.25, 12.2, 0.5, { size: 21, bold: true });
  text(slide, sub, 0.58, 0.73, 12.1, 0.32, { size: 9.5, color: C.muted });
  slide.addShape(pptx.ShapeType.line, {
    x: 0.55,
    y: 1.08,
    w: 12.2,
    h: 0,
    line: { color: C.line, width: 0.8 }
  });
}

function footer(slide, value) {
  text(slide, value, 0.65, 6.96, 12, 0.25, { size: 8, color: C.muted, align: "center" });
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
  text(slide, value, x + 0.07, y + 0.05, w - 0.14, h - 0.1, {
    size: opt.size || 11,
    bold: opt.bold || false,
    color: opt.color || C.ink,
    align: opt.align || "center"
  });
}

function arrow(slide, x1, y1, x2, y2, color = C.muted, width = 1.4) {
  slide.addShape(pptx.ShapeType.line, {
    x: x1,
    y: y1,
    w: x2 - x1,
    h: y2 - y1,
    line: { color, width, endArrowType: "triangle" }
  });
}

function table(slide, rows, x, y, widths, rowH, size = 8.8) {
  rows.forEach((row, r) => {
    let curX = x;
    row.forEach((cell, c) => {
      const head = r === 0;
      slide.addShape(pptx.ShapeType.rect, {
        x: curX,
        y: y + r * rowH,
        w: widths[c],
        h: rowH,
        fill: { color: head ? "E2E8F0" : C.white },
        line: { color: C.line, width: 0.55 }
      });
      text(slide, cell, curX + 0.04, y + r * rowH + 0.02, widths[c] - 0.08, rowH - 0.04, {
        size,
        bold: head,
        align: c === 0 ? "center" : "left"
      });
      curX += widths[c];
    });
  });
}

{
  const s = pptx.addSlide();
  title(s, "Polygon Mirror Budget 모델", "13.888889us start_tdc 간격에서 후단 8us를 예약하고 남은 시간으로 C04 drain 가능성을 판단");
  box(s, "10Hz\n1회전 100ms", 0.75, 1.45, 1.65, 0.8, { fill: C.blueFill, line: C.blue, bold: true, size: 12 });
  arrow(s, 2.45, 1.85, 3.05, 1.85, C.blue);
  box(s, "5각\n72도 = 20ms", 3.1, 1.45, 1.75, 0.8, { fill: C.cyanFill, line: C.cyan, bold: true, size: 12 });
  arrow(s, 4.9, 1.85, 5.5, 1.85, C.cyan);
  box(s, "60도 사용\n16.666667ms", 5.55, 1.45, 1.9, 0.8, { fill: C.purpleFill, line: C.purple, bold: true, size: 12 });
  arrow(s, 7.5, 1.85, 8.1, 1.85, C.purple);
  box(s, "0.05도\n1200 points", 8.15, 1.45, 1.75, 0.8, { fill: C.orangeFill, line: C.orange, bold: true, size: 12 });
  arrow(s, 9.95, 1.85, 10.55, 1.85, C.orange);
  box(s, "start 간격\n13.888889us", 10.6, 1.45, 2.0, 0.8, { fill: C.greenFill, line: C.green, bold: true, size: 12 });
  table(s, [
    ["항목", "값"],
    ["후단 예약", "VDMA + PS + Ethernet = 8.000000 us"],
    ["pre-ToF 예산", "13.888889 us - 8.000000 us = 5.888889 us"],
    ["거리별 C04 예산", "5.888889 us - 왕복시간(distance)"],
    ["판정", "C04 drain time <= 거리별 C04 예산"]
  ], 1.1, 3.2, [2.4, 8.8], 0.48, 10.3);
  footer(s, "TB: tb_tdc_gpx_polygon_budget_matrix.vhd");
}

{
  const s = pptx.addSlide();
  title(s, "max_hits_cfg 스왑 결과", "각 거리에서 PASS 가능한 최대 cfg를 선택한다. 거리가 길어질수록 왕복시간이 커져 budget이 감소한다.");
  table(s, [
    ["Width", "cfg=7 유지", "스왑 구간", "완전 FAIL"],
    ["32bit", "10m~710m", "720~740m cfg6, 750~770m cfg4, 780~800m cfg2", "810m"],
    ["64bit", "10m~780m", "790~810m cfg4", "820m"],
    ["128bit", "10m~810m", "없음", "820m"]
  ], 0.8, 1.45, [1.3, 2.3, 6.2, 1.4], 0.65, 11);
  box(s, "32bit\npayload beat가 많아서\n가장 먼저 cfg를 낮춤", 1.0, 4.25, 3.0, 1.1, { fill: C.redFill, line: C.red, bold: true });
  box(s, "64bit\n790m부터 cfg4로 제한", 4.35, 4.25, 2.8, 1.1, { fill: C.orangeFill, line: C.orange, bold: true });
  box(s, "128bit\n810m까지 cfg7 유지", 7.5, 4.25, 2.8, 1.1, { fill: C.greenFill, line: C.green, bold: true });
  footer(s, "Boundary logs: xsim_polygon_budget_matrix.log:36,180,188,196,204,212,370,378,386,550");
}

{
  const s = pptx.addSlide();
  title(s, "32bit 경계", "32bit는 cfg group이 촘촘히 나뉘어 거리 증가에 따라 단계적으로 cfg를 낮춘다.");
  table(s, [
    ["거리", "budget", "권장 cfg", "cfg7 margin", "의미"],
    ["710m", "1,152,479ps", "7", "+5,755ps", "cfg7 마지막 PASS"],
    ["720m", "1,085,769ps", "6", "-60,955ps", "cfg7 FAIL"],
    ["750m", "885,639ps", "4", "-261,085ps", "cfg5 이상 FAIL"],
    ["780m", "685,509ps", "2", "-461,215ps", "cfg3 이상 FAIL"],
    ["810m", "485,379ps", "0", "-661,345ps", "cfg1도 FAIL"]
  ], 0.75, 1.35, [1.1, 2.0, 1.3, 1.8, 4.7], 0.52, 10.5);
  footer(s, "근거: xsim_polygon_budget_matrix.log:176,178,186,194,202");
}

{
  const s = pptx.addSlide();
  title(s, "64bit / 128bit 경계", "넓은 output width는 같은 payload를 더 적은 beat로 전송하므로 더 먼 거리까지 cfg7을 유지한다.");
  table(s, [
    ["Width", "거리", "budget", "권장 cfg", "margin", "의미"],
    ["64", "780m", "685,509ps", "7", "+5,475ps", "cfg7 마지막 PASS"],
    ["64", "790m", "618,799ps", "4", "-61,235ps", "cfg5 이상 FAIL"],
    ["64", "820m", "418,669ps", "0", "-48,021ps(cfg1)", "cfg1도 FAIL"],
    ["128", "810m", "485,379ps", "7", "+38,690ps", "cfg7 마지막 PASS"],
    ["128", "820m", "418,669ps", "0", "-28,020ps", "cfg1도 FAIL"]
  ], 0.75, 1.35, [0.9, 1.05, 1.75, 1.2, 1.9, 3.9], 0.52, 10);
  footer(s, "근거: xsim_polygon_budget_matrix.log:366,368,376,546,548");
}

{
  const s = pptx.addSlide();
  title(s, "운용 결론", "이번 모델에서는 10m~수백m까지 여유가 크다. 이유는 start_tdc 간격이 고정 13.888889us이기 때문이다.");
  box(s, "32bit 운용", 0.9, 1.4, 2.2, 0.5, { fill: C.redFill, line: C.red, bold: true });
  text(s, "710m 이하 cfg7, 720~740m cfg6, 750~770m cfg4, 780~800m cfg2, 810m 이상 FAIL", 3.35, 1.35, 8.5, 0.65, { size: 12, bold: true });
  box(s, "64bit 운용", 0.9, 2.65, 2.2, 0.5, { fill: C.orangeFill, line: C.orange, bold: true });
  text(s, "780m 이하 cfg7, 790~810m cfg4, 820m 이상 FAIL", 3.35, 2.6, 8.5, 0.65, { size: 12, bold: true });
  box(s, "128bit 운용", 0.9, 3.9, 2.2, 0.5, { fill: C.greenFill, line: C.green, bold: true });
  text(s, "810m 이하 cfg7, 820m 이상 FAIL", 3.35, 3.85, 8.5, 0.65, { size: 12, bold: true });
  box(s, "주의", 0.9, 5.15, 1.45, 0.5, { fill: C.blueFill, line: C.blue, bold: true });
  text(s, "8us reserve는 사용자 운용 가정이다. 실제 closure는 VDMA backpressure, PS/Ethernet worst-case, C02/C03/C04 end-to-end latency로 재확인해야 한다.", 2.65, 5.05, 9.4, 0.8, { size: 11.6, bold: true });
  footer(s, "xsim 최종 PASS: xsim_polygon_budget_matrix.log:554");
}

pptx.writeFile({ fileName: out });
