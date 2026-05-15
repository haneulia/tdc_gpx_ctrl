const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/.pnpm/pptxgenjs@4.0.1/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C07_System_Integration/C07_System_Integration_260515174538_Reserve_Budget_Result_v001.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C07 Reserve Budget Result";
pptx.title = "C07 Reserve Budget Result v001";
pptx.lang = "ko-KR";
pptx.theme = { headFontFace: "Malgun Gothic", bodyFontFace: "Malgun Gothic", lang: "ko-KR" };

const font = "Malgun Gothic";
const C = {
  bg: "F8FAFC", ink: "111827", muted: "64748B", line: "CBD5E1",
  dark: "0F172A", white: "FFFFFF", blue: "2563EB", blueFill: "DBEAFE",
  green: "15803D", greenFill: "DCFCE7", orange: "C2410C", orangeFill: "FFEDD5",
  red: "B91C1C", redFill: "FEE2E2", purple: "6D28D9", purpleFill: "EDE9FE",
  teal: "0F766E", tealFill: "CCFBF1", slate: "E2E8F0",
};

function tx(slide, value, x, y, w, h, opt = {}) {
  slide.addText(value, {
    x, y, w, h, fontFace: font, fontSize: opt.size || 10,
    bold: !!opt.bold, color: opt.color || C.ink,
    align: opt.align || "left", valign: "mid",
    fit: "shrink", margin: opt.margin === undefined ? 0.04 : opt.margin,
  });
}
function title(slide, main, sub) {
  slide.background = { color: C.bg };
  tx(slide, main, 0.55, 0.28, 12.1, 0.46, { size: 20.5, bold: true });
  tx(slide, sub, 0.58, 0.78, 12.0, 0.3, { size: 9.0, color: C.muted });
  slide.addShape(pptx.ShapeType.line, { x: 0.58, y: 1.13, w: 12.1, h: 0, line: { color: C.line, width: 0.8 } });
}
function footer(slide, value) {
  tx(slide, value, 0.58, 7.05, 12.15, 0.24, { size: 7.2, color: C.muted, align: "center" });
}
function box(slide, label, x, y, w, h, opt = {}) {
  slide.addShape(pptx.ShapeType.roundRect, {
    x, y, w, h, rectRadius: 0.04,
    fill: { color: opt.fill || C.white },
    line: { color: opt.line || C.line, width: 0.9 },
  });
  tx(slide, label, x + 0.08, y + 0.06, w - 0.16, h - 0.12, {
    size: opt.size || 9, bold: opt.bold, color: opt.color || C.ink, align: opt.align || "center",
  });
}
function arrow(slide, x1, y1, x2, y2, color = C.muted) {
  slide.addShape(pptx.ShapeType.line, { x: x1, y: y1, w: x2 - x1, h: y2 - y1, line: { color, width: 1.15, endArrowType: "triangle" } });
}
function table(slide, rows, x, y, widths, rowH, size = 7.0) {
  rows.forEach((row, r) => {
    let curX = x;
    row.forEach((cell, c) => {
      slide.addShape(pptx.ShapeType.rect, {
        x: curX, y: y + r * rowH, w: widths[c], h: rowH,
        fill: { color: r === 0 ? C.slate : C.white },
        line: { color: C.line, width: 0.45 },
      });
      tx(slide, cell, curX + 0.05, y + r * rowH + 0.02, widths[c] - 0.10, rowH - 0.04, {
        size, bold: r === 0, align: c === 0 ? "center" : "left",
      });
      curX += widths[c];
    });
  });
}

{
  const s = pptx.addSlide();
  s.background = { color: C.dark };
  tx(s, "C07 Reserve Budget", 0.72, 0.75, 7.8, 0.42, { size: 18, bold: true, color: "93C5FD" });
  tx(s, "8us는 통과지만\n실측 없는 margin은 작다", 0.72, 1.28, 9.6, 1.15, { size: 29, bold: true, color: C.white });
  tx(s, "8/9/10/11/12us reserve sweep으로 polygon 운용 거리 경계를 다시 산출했습니다.", 0.78, 2.88, 11.6, 0.55, { size: 13, color: "E5E7EB" });
  box(s, "8us\n128b 810m PASS\nmargin 38.69ns", 0.95, 4.1, 2.65, 0.95, { fill: "14532D", line: "86EFAC", color: C.white, bold: true, size: 11.1 });
  box(s, "9us\n128b cfg7\n660m까지", 3.9, 4.1, 2.55, 0.95, { fill: "713F12", line: "FDE68A", color: C.white, bold: true, size: 11.1 });
  box(s, "10us\n128b cfg7\n510m까지", 6.75, 4.1, 2.55, 0.95, { fill: "7F1D1D", line: "FCA5A5", color: C.white, bold: true, size: 11.1 });
  box(s, "결론\n810m는 측정 계약 필요", 9.6, 4.1, 2.55, 0.95, { fill: "1E3A8A", line: "93C5FD", color: C.white, bold: true, size: 11.1 });
  footer(s, "Archive: sim_results/vivado_xsim/sessions/260515174807_c07_v001_reserve_budget/");
}

{
  const s = pptx.addSlide();
  title(s, "Budget Model", "point interval 안에서 ToF, output drain, system reserve가 모두 들어와야 합니다.");
  const xs = [0.9, 3.1, 5.3, 7.7, 10.0];
  const labels = ["start_tdc\n13.888889us", "ToF\ndistance * 6671ps", "C04 drain\nwidth/cfg", "Reserve\nVDMA/PS/Eth", "next\nstart_tdc"];
  labels.forEach((label, i) => {
    box(s, label, xs[i], 2.05, 1.65, 0.85, { fill: i === 3 ? C.orangeFill : C.blueFill, line: i === 3 ? C.orange : C.blue, bold: true, size: 9.1 });
    if (i < labels.length - 1) arrow(s, xs[i] + 1.68, 2.48, xs[i + 1] - 0.08, 2.48);
  });
  box(s, "PASS 조건: 13.888889us - reserve - round_trip(distance) >= output_drain(width, max_hits_cfg)", 1.0, 4.6, 11.25, 0.72, { fill: C.tealFill, line: C.teal, bold: true, size: 12 });
  footer(s, "full load: 4 active chips x 8 stops/chip, output clock 150MHz");
}

{
  const s = pptx.addSlide();
  title(s, "8us Current Assumption", "기존 C04 결과와 같은 reserve 기준입니다.");
  table(s, [
    ["Width", "cfg7 유지", "cfg swap", "FAIL 시작"],
    ["32", "710m까지", "720-740 cfg6, 750-770 cfg4, 780-800 cfg2", "810m"],
    ["64", "780m까지", "790-810 cfg4", "820m"],
    ["128", "810m까지", "없음", "820m"],
  ], 0.75, 1.45, [1.2, 2.0, 6.4, 2.0], 0.58, 8.0);
  box(s, "128-bit 810m cfg7은 PASS지만 margin은 38.69ns뿐입니다. 실측 없는 release margin으로는 매우 얇습니다.", 1.0, 5.2, 11.3, 0.75, { fill: C.orangeFill, line: C.orange, bold: true, size: 12 });
  footer(s, "근거: xsim_c07_v001_reserve_8us_260515174807.log");
}

{
  const s = pptx.addSlide();
  title(s, "Reserve Sensitivity", "reserve가 1us 늘어날 때마다 거리 경계가 크게 내려갑니다.");
  table(s, [
    ["Reserve", "32-bit cfg7", "64-bit cfg7", "128-bit cfg7", "128-bit FAIL"],
    ["8us", "710m", "780m", "810m", "820m"],
    ["9us", "560m", "630m", "660m", "670m"],
    ["10us", "410m", "480m", "510m", "520m"],
    ["11us", "260m", "330m", "360m", "370m"],
    ["12us", "110m", "180m", "210m", "220m"],
  ], 0.75, 1.35, [1.6, 2.2, 2.2, 2.2, 2.2], 0.52, 8.0);
  footer(s, "8/9/10/11/12us sweep PASS, artifact count 26");
}

{
  const s = pptx.addSlide();
  title(s, "Release Decision", "P0-04는 xsim sensitivity로 닫고, system 실측은 release gate로 분리합니다.");
  table(s, [
    ["선택", "조건", "의미"],
    ["8us 유지", "보드/PS/Ethernet measured reserve <= 8us", "810m는 128-bit에서만 cfg7 가능"],
    ["9us 보수", "실측 전 임시 guardband", "128-bit cfg7은 660m까지"],
    ["810m + margin", "0.5us margin이면 reserve <= 약 7.54us", "측정 계약 없이는 권장 어려움"],
  ], 0.75, 1.45, [2.0, 5.1, 4.5], 0.62, 8.0);
  box(s, "다음 작업은 P1: C03 direct matrix, C04 ready/header pending, Hit[16] SW/range 계약입니다.", 1.05, 5.55, 11.1, 0.68, { fill: C.greenFill, line: C.green, bold: true, size: 12 });
  footer(s, "P0-04 closure: RTL/xsim reserve sensitivity complete, system measurement still required for release");
}

pptx.writeFile({ fileName: out });
