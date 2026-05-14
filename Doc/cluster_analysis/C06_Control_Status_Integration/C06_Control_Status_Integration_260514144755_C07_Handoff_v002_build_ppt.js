const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/.pnpm/pptxgenjs@4.0.1/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C06_Control_Status_Integration/C06_Control_Status_Integration_260514144755_C07_Handoff_v002.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C06 to C07 Handoff v002";
pptx.title = "C06 to C07 Handoff v002";
pptx.lang = "ko-KR";
pptx.theme = { headFontFace: "Malgun Gothic", bodyFontFace: "Malgun Gothic", lang: "ko-KR" };

const font = "Malgun Gothic";
const C = {
  bg: "F8FAFC", ink: "111827", muted: "64748B", line: "CBD5E1",
  white: "FFFFFF", dark: "0F172A", blue: "2563EB", blueFill: "DBEAFE",
  green: "15803D", greenFill: "DCFCE7", orange: "C2410C", orangeFill: "FFEDD5",
  red: "B91C1C", redFill: "FEE2E2", purple: "6D28D9", purpleFill: "EDE9FE",
  teal: "0F766E", tealFill: "CCFBF1", slate: "E2E8F0",
};
function tx(slide, value, x, y, w, h, opt = {}) {
  slide.addText(value, { x, y, w, h, fontFace: font, fontSize: opt.size || 10, bold: !!opt.bold, color: opt.color || C.ink, align: opt.align || "left", valign: "mid", fit: "shrink", margin: opt.margin === undefined ? 0.04 : opt.margin });
}
function title(slide, main, sub) {
  slide.background = { color: C.bg };
  tx(slide, main, 0.55, 0.28, 12.1, 0.46, { size: 21, bold: true });
  tx(slide, sub, 0.58, 0.78, 12.0, 0.3, { size: 9.2, color: C.muted });
  slide.addShape(pptx.ShapeType.line, { x: 0.58, y: 1.13, w: 12.1, h: 0, line: { color: C.line, width: 0.8 } });
}
function footer(slide, value) {
  tx(slide, value, 0.58, 7.05, 12.15, 0.24, { size: 7.2, color: C.muted, align: "center" });
}
function box(slide, label, x, y, w, h, opt = {}) {
  slide.addShape(pptx.ShapeType.roundRect, { x, y, w, h, rectRadius: 0.04, fill: { color: opt.fill || C.white }, line: { color: opt.line || C.line, width: 0.9 } });
  tx(slide, label, x + 0.08, y + 0.06, w - 0.16, h - 0.12, { size: opt.size || 9, bold: opt.bold, color: opt.color || C.ink, align: opt.align || "center" });
}
function arrow(slide, x1, y1, x2, y2, color = C.muted) {
  slide.addShape(pptx.ShapeType.line, { x: x1, y: y1, w: x2 - x1, h: y2 - y1, line: { color, width: 1.2, endArrowType: "triangle" } });
}
function table(slide, rows, x, y, widths, rowH, size = 7.4) {
  rows.forEach((row, r) => {
    let curX = x;
    row.forEach((cell, c) => {
      slide.addShape(pptx.ShapeType.rect, { x: curX, y: y + r * rowH, w: widths[c], h: rowH, fill: { color: r === 0 ? C.slate : C.white }, line: { color: C.line, width: 0.45 } });
      tx(slide, cell, curX + 0.05, y + r * rowH + 0.02, widths[c] - 0.10, rowH - 0.04, { size, bold: r === 0, align: c === 0 ? "center" : "left" });
      curX += widths[c];
    });
  });
}

{
  const s = pptx.addSlide();
  s.background = { color: C.dark };
  tx(s, "C06 -> C07 Handoff v002", 0.72, 0.88, 9.8, 0.56, { size: 27, bold: true, color: C.white });
  tx(s, "GO_WITH_HARDENED_CONTRACT", 0.76, 1.62, 8.7, 0.42, { size: 16, color: "CBD5E1" });
  box(s, "C06 closed\ncontrol/status/recovery", 0.95, 3.75, 2.6, 0.95, { fill: "14532D", line: "86EFAC", color: C.white, bold: true });
  box(s, "C07 must close\nsystem timing", 3.95, 3.75, 2.6, 0.95, { fill: "713F12", line: "FDE68A", color: C.white, bold: true });
  box(s, "SW must accept\n16-bit hit slot", 6.95, 3.75, 2.6, 0.95, { fill: "1E3A8A", line: "93C5FD", color: C.white, bold: true });
  box(s, "Board must close\nSTA / reserve", 9.95, 3.75, 2.6, 0.95, { fill: "7F1D1D", line: "FCA5A5", color: C.white, bold: true });
  footer(s, "archive: sim_results/vivado_xsim/sessions/260514144755_c06_v006_hardening/");
}

{
  const s = pptx.addSlide();
  title(s, "C06 Closed Contracts", "다음 단계가 그대로 받아야 하는 닫힌 계약입니다.");
  table(s, [
    ["ID", "계약", "근거"],
    ["H2-03", "output width 32/64/128", "baseline PASS"],
    ["H2-04", "32/64/128 force/soft recovery", "v006 recovery PASS"],
    ["H2-05", "bounded backpressure width sweep", "line 143/145"],
    ["H2-06", "rise-only/fall-only output stall", "64-bit lane PASS"],
    ["H2-07", "Hit[16] final stream 폐기", "SW parser contract"],
    ["H2-08", "sticky clear map", "soft_clear vs reset-only"],
  ], 0.68, 1.45, [1.0, 5.3, 5.1], 0.50, 7.2);
  footer(s, "근거: C06_Control_Status_Integration_260514144755_C07_Handoff_v002.md section 2");
}

{
  const s = pptx.addSlide();
  title(s, "C07 System Items", "C06 xsim PASS가 시스템 release PASS는 아닙니다.");
  table(s, [
    ["System ID", "항목", "이유"],
    ["SYS-01", "VDMA/PS/Ethernet reserve 실측", "8 us는 가정값"],
    ["SYS-02", "polygon budget 최종 거리표", "810 m @ 8 us margin 부족"],
    ["SYS-03", "SW parser 16-bit hit slot", "Hit[16] 폐기 수락 필요"],
    ["SYS-04", "board STA / constraints", "xsim 범위 밖"],
    ["SYS-05", "장기 output stall policy", "bounded stall만 검증"],
  ], 0.75, 1.45, [1.6, 4.2, 5.55], 0.56, 7.6);
  footer(s, "권고: 다음 Cluster는 C07_System_Integration 또는 C07_Release_Readiness");
}

{
  const s = pptx.addSlide();
  title(s, "Data / Timing Handoff", "C06은 final AXIS와 status를 시스템 소비자에게 넘깁니다.");
  const nodes = [
    ["C06 RTL/xsim\nclosed", 0.95, C.greenFill, C.green],
    ["final AXIS\n32/64/128", 3.35, C.blueFill, C.blue],
    ["SW parser\n16-bit hit", 5.95, C.purpleFill, C.purple],
    ["system timing\nreserve/STA", 8.55, C.orangeFill, C.orange],
  ];
  nodes.forEach(([label, x, fill, line], i) => {
    box(s, label, x, 2.0, 1.9, 0.95, { fill, line, bold: true, size: 8.4 });
    if (i < nodes.length - 1) arrow(s, x + 1.92, 2.47, nodes[i + 1][1] - 0.08, 2.47);
  });
  table(s, [
    ["Metric", "인계값"],
    ["recovery CDC", "4 clk / 20 ns"],
    ["throughput", "32:72, 64:44, 128:38 beats/run"],
    ["conservative T2->T5", "32:0.540 us, 64/128:0.500 us"],
    ["bounded stall", "bp_gap=17 pass"],
  ], 1.05, 4.05, [3.1, 7.8], 0.48, 8.0);
  footer(s, "C07은 시스템 소비자 관점에서 reserve와 parser를 닫음");
}

pptx.writeFile({ fileName: out });
