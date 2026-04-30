const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C04_Output_Stage/C04_Output_Stage_260501040101_Distance_10m_Sweep_Result_v003.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C04 100m 10m Distance Sweep Result v003";
pptx.title = "C04 100m 10m Distance Sweep Result v003";
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

function line(slide, x1, y1, x2, y2, color = C.muted, width = 1.4) {
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
  title(s, "100m 시작 10m 거리 스윕", "post-stop 보수 모델: stop_tdc 이후 C04 final stream drain이 next start 전 완료되는지 확인");
  box(s, "start\n100m", 0.75, 1.45, 1.55, 0.8, { fill: C.blueFill, line: C.blue, bold: true, size: 13 });
  line(s, 2.35, 1.85, 3.0, 1.85, C.blue);
  box(s, "step\n10m", 3.05, 1.45, 1.45, 0.8, { fill: C.cyanFill, line: C.cyan, bold: true, size: 13 });
  line(s, 4.55, 1.85, 5.2, 1.85, C.cyan);
  box(s, "width\n32/64/128", 5.25, 1.45, 1.9, 0.8, { fill: C.purpleFill, line: C.purple, bold: true, size: 13 });
  line(s, 7.2, 1.85, 7.85, 1.85, C.purple);
  box(s, "cfg\n1..7", 7.9, 1.45, 1.45, 0.8, { fill: C.orangeFill, line: C.orange, bold: true, size: 13 });
  line(s, 9.4, 1.85, 10.05, 1.85, C.orange);
  box(s, "PASS/FAIL\npost-stop", 10.1, 1.45, 2.1, 0.8, { fill: C.greenFill, line: C.green, bold: true, size: 13 });
  table(s, [
    ["항목", "값"],
    ["100m 왕복", "667,100 ps = 667.100 ns"],
    ["100m shot_period", "1,000,650 ps = 1.000650 us"],
    ["100m post-stop budget", "333,550 ps = 333.550 ns"],
    ["판정", "C04 drain_ps <= post_stop_budget_ps"]
  ], 1.1, 3.2, [2.55, 8.8], 0.48, 10.5);
  footer(s, "근거: tb_tdc_gpx_distance_maxhits_matrix_100m_10m.vhd / xsim_distance_maxhits_matrix_100m_10m.log");
}

{
  const s = pptx.addSlide();
  title(s, "100m 판정", "100m에서는 stop 이후 budget이 333.550 ns라서 모든 width가 cfg=1부터 FAIL");
  table(s, [
    ["Width", "cfg", "drain_ps", "budget_ps", "margin_ps", "판정"],
    ["32", "1", "506,692", "333,550", "-173,142", "FAIL"],
    ["64", "1", "466,690", "333,550", "-133,140", "FAIL"],
    ["128", "1", "446,689", "333,550", "-113,139", "FAIL"]
  ], 1.0, 1.55, [1.05, 0.9, 2.05, 2.05, 1.85, 1.25], 0.62, 12);
  box(s, "100m에서의 핵심", 1.05, 4.35, 2.15, 0.48, { fill: C.redFill, line: C.red, bold: true });
  text(s, "C04 출력이 stop_tdc 이후에만 drain된다는 보수 모델에서는 128bit라도 cfg=1을 만족하지 못한다. 100m 운용을 닫으려면 overlap 검증, PRF 완화, payload 축소, 또는 더 빠른 output path 검토가 필요하다.", 3.45, 4.25, 8.65, 0.95, { size: 12, bold: true });
  footer(s, "로그: 32 log:34 / 64 log:102 / 128 log:142");
}

{
  const s = pptx.addSlide();
  title(s, "최초 PASS 거리", "10m 단위 경계에서 width별 최소 운용 거리가 더 정확해졌다.");
  table(s, [
    ["목표", "32bit", "64bit", "128bit"],
    ["cfg=1 최소 PASS", "160m", "140m", "140m"],
    ["cfg=6 최소 PASS", "280m", "210m", "140m"],
    ["cfg=7 최소 PASS", "350m", "210m", "140m"]
  ], 1.0, 1.4, [2.5, 2.1, 2.1, 2.1], 0.66, 12);
  box(s, "32bit\ncfg1:160m\ncfg6:280m\ncfg7:350m", 1.0, 4.0, 2.75, 1.25, { fill: C.redFill, line: C.red, bold: true, size: 12 });
  box(s, "64bit\ncfg1:140m\ncfg6:210m\ncfg7:210m", 4.2, 4.0, 2.75, 1.25, { fill: C.orangeFill, line: C.orange, bold: true, size: 12 });
  box(s, "128bit\ncfg1:140m\ncfg6:140m\ncfg7:140m", 7.4, 4.0, 2.75, 1.25, { fill: C.greenFill, line: C.green, bold: true, size: 12 });
  footer(s, "경계 로그: 32 log:46,80,96 / 64 log:110,136 / 128 log:150,162");
}

{
  const s = pptx.addSlide();
  title(s, "Width별 Beat Group", "max_hits_cfg가 증가해도 width에 따라 같은 beat group을 공유한다.");
  table(s, [
    ["Width", "cfg group", "line_beats", "drain_ps", "최초 PASS"],
    ["32", "1..2", "76", "506,692", "160m"],
    ["32", "3..4", "108", "720,036", "220m"],
    ["32", "5..6", "140", "933,380", "280m"],
    ["32", "7", "172", "1,146,724", "350m"],
    ["64", "1..4", "70", "466,690", "140m"],
    ["64", "5..7", "102", "680,034", "210m"],
    ["128", "1..7", "67", "446,689", "140m"]
  ], 0.85, 1.35, [1.0, 2.0, 1.45, 1.85, 1.45], 0.45, 9.5);
  box(s, "해석", 8.55, 1.45, 1.3, 0.45, { fill: C.blueFill, line: C.blue, bold: true });
  text(s, "넓은 width는 같은 cell payload를 더 적은 beat로 내보낸다. 그래서 post-stop budget이 작은 근거리에서는 128bit가 거리 경계를 크게 낮춘다.", 8.45, 2.05, 3.5, 1.7, { size: 12, bold: true });
  footer(s, "계산 근거: tdc_gpx_pkg.vhd fn_beats_per_cell_rt / 150MHz output period");
}

{
  const s = pptx.addSlide();
  title(s, "운용 판단", "100m부터 10m 단위로 보면 100m 보수 운용은 현재 full-line payload 기준으로 닫히지 않는다.");
  box(s, "100m", 0.9, 1.55, 1.6, 0.8, { fill: C.redFill, line: C.red, bold: true, size: 14 });
  text(s, "32/64/128bit 모두 cfg=1부터 FAIL", 2.75, 1.55, 8.9, 0.8, { size: 16, bold: true });
  box(s, "cfg=6 이상 보수 사용", 0.9, 3.0, 2.4, 0.65, { fill: C.orangeFill, line: C.orange, bold: true, size: 13 });
  table(s, [
    ["Width", "최소 거리"],
    ["32bit", "280m 이상"],
    ["64bit", "210m 이상"],
    ["128bit", "140m 이상"]
  ], 3.6, 2.8, [1.6, 2.4], 0.5, 11);
  box(s, "다음 검토", 8.25, 2.9, 1.55, 0.48, { fill: C.blueFill, line: C.blue, bold: true });
  text(s, "100m 운용이 필요하면 end-to-end overlap 검증 또는 shot_period 완화가 별도 필요하다.", 8.15, 3.55, 3.7, 0.8, { size: 12, bold: true });
  footer(s, "xsim 최종 PASS: xsim_distance_maxhits_matrix_100m_10m.log:166");
}

pptx.writeFile({ fileName: out });
