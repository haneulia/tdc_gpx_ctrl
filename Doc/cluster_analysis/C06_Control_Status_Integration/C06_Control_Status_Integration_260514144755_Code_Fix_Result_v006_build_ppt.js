const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/.pnpm/pptxgenjs@4.0.1/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C06_Control_Status_Integration/C06_Control_Status_Integration_260514144755_Code_Fix_Result_v006.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C06 Code Fix Result v006";
pptx.title = "C06 Code Fix Result v006";
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
  slide.addText(value, {
    x, y, w, h, fontFace: font, fontSize: opt.size || 10, bold: !!opt.bold,
    color: opt.color || C.ink, align: opt.align || "left", valign: "mid",
    fit: "shrink", margin: opt.margin === undefined ? 0.04 : opt.margin,
  });
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
  slide.addShape(pptx.ShapeType.roundRect, {
    x, y, w, h, rectRadius: 0.04,
    fill: { color: opt.fill || C.white },
    line: { color: opt.line || C.line, width: 0.9 },
  });
  tx(slide, label, x + 0.08, y + 0.06, w - 0.16, h - 0.12, {
    size: opt.size || 9.0, bold: opt.bold, color: opt.color || C.ink, align: opt.align || "center",
  });
}
function arrow(slide, x1, y1, x2, y2, color = C.muted) {
  slide.addShape(pptx.ShapeType.line, { x: x1, y: y1, w: x2 - x1, h: y2 - y1, line: { color, width: 1.15, endArrowType: "triangle" } });
}
function table(slide, rows, x, y, widths, rowH, size = 7.2) {
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
  tx(s, "C06 Result v006", 0.72, 0.82, 7.8, 0.55, { size: 28, bold: true, color: C.white });
  tx(s, "Release hardening regression closed", 0.76, 1.58, 8.8, 0.42, { size: 15, color: "CBD5E1" });
  box(s, "PASS\n32/64/128 recovery", 0.92, 3.05, 2.55, 1.0, { fill: "14532D", line: "86EFAC", color: C.white, bold: true, size: 12 });
  box(s, "PASS\nwidth backpressure", 3.92, 3.05, 2.55, 1.0, { fill: "1E3A8A", line: "93C5FD", color: C.white, bold: true, size: 12 });
  box(s, "PASS\nlane stall", 6.92, 3.05, 2.55, 1.0, { fill: "0F766E", line: "5EEAD4", color: C.white, bold: true, size: 12 });
  box(s, "CONTRACT\nsystem reserve", 9.92, 3.05, 2.55, 1.0, { fill: "713F12", line: "FDE68A", color: C.white, bold: true, size: 12 });
  tx(s, "Archive: sim_results/vivado_xsim/sessions/260514144755_c06_v006_hardening/", 0.78, 5.35, 11.8, 0.36, { size: 11.5, color: C.white });
  footer(s, "생성 2026-05-14 14:47:55 KST | artifact count 65");
}

{
  const s = pptx.addSlide();
  title(s, "Plan ID Closure", "v006 계획을 PASS, contract, document audit으로 분리해 닫았습니다.");
  table(s, [
    ["Plan", "결과", "근거"],
    ["FP6-01", "Recovery width sweep PASS", "32/64/128 force/soft"],
    ["FP6-02", "Backpressure width PASS", "32/64/128 bp_gap=17"],
    ["FP6-03", "Reserve sweep 계산 완료", "8 us는 system measurement"],
    ["FP6-04", "Marker audit 문서화", "source/dest/effect 기준"],
    ["FP6-05", "Hit[16] 계약화", "16-bit slot / SW parser"],
    ["FP6-06", "Sticky map 확정", "soft_clear vs reset-only"],
    ["FP6-07", "Lane stall PASS", "rise-only/fall-only"],
  ], 0.62, 1.36, [1.15, 4.3, 5.9], 0.43, 6.8);
  footer(s, "근거: C06_Control_Status_Integration_260514144755_Code_Fix_Result_v006.md section 4");
}

{
  const s = pptx.addSlide();
  title(s, "Recovery Width Sweep", "각 width에서 source, destination, effect, output preservation marker를 확인했습니다.");
  table(s, [
    ["Width", "force beats/tlast", "soft beats/tlast", "PASS marker"],
    ["32", "144 / 4", "144 / 4", "236/238, 244/246"],
    ["64", "88 / 4", "88 / 4", "236/238, 244/246"],
    ["128", "76 / 4", "76 / 4", "236/238, 244/246"],
  ], 0.9, 1.55, [1.3, 3.2, 3.2, 3.3], 0.56, 8.2);
  const xs = [1.2, 3.9, 6.6, 9.3];
  const labels = [
    ["source pulse\nline 142", C.orangeFill, C.orange],
    ["TDC pulse\nline 146", C.blueFill, C.blue],
    ["FSM effect\nline 148/156", C.purpleFill, C.purple],
    ["stream PASS\nline 236/244", C.greenFill, C.green],
  ];
  labels.forEach(([label, fill, line], i) => {
    box(s, label, xs[i], 4.2, 1.75, 0.78, { fill, line, bold: true, size: 8.4 });
    if (i < labels.length - 1) arrow(s, xs[i] + 1.78, 4.59, xs[i + 1] - 0.08, 4.59);
  });
  footer(s, "false positive 방지: source + destination + effect + output marker");
}

{
  const s = pptx.addSlide();
  title(s, "Backpressure / Lane Imbalance", "bounded stall과 rise-only/fall-only stall 모두 beat/tlast를 보존했습니다.");
  table(s, [
    ["Scenario", "Width", "Beats/tlast", "Timing"],
    ["both-lane bp", "32", "72 / 2", "last cycle 5195"],
    ["both-lane bp", "64", "44 / 2", "last cycle 5189"],
    ["both-lane bp", "128", "38 / 2", "last cycle 5189"],
    ["rise-only bp", "64", "44 / 2", "rise last 5189, fall 5187"],
    ["fall-only bp", "64", "44 / 2", "rise last 5187, fall 5189"],
  ], 0.75, 1.45, [2.5, 1.5, 2.2, 5.1], 0.50, 7.4);
  box(s, "bp_gap=17, 2-cycle stall 반복. 장기/무한 stall은 C07 system fault policy 대상입니다.", 0.95, 5.65, 11.2, 0.65, { fill: C.orangeFill, line: C.orange, bold: true, size: 10.5 });
  footer(s, "근거: xsim_c06_v006_top_int_w*_bp*_260514144755.log line 143/145");
}

{
  const s = pptx.addSlide();
  title(s, "Polygon Reserve", "8 us reserve 기준 810 m는 C06 보수 worst 계산에서 margin 부족입니다.");
  table(s, [
    ["Width", "T2->T5", "6 us reserve", "8 us reserve", "10 us reserve"],
    ["32", "0.540 us", "1101.6 m", "801.8 m", "502.0 m"],
    ["64", "0.500 us", "1107.6 m", "807.8 m", "508.0 m"],
    ["128", "0.500 us", "1107.6 m", "807.8 m", "508.0 m"],
  ], 0.82, 1.55, [1.35, 2.15, 2.55, 2.55, 2.55], 0.58, 8.0);
  box(s, "판단: 8 us는 RTL/xsim 실측값이 아닙니다. C07에서 VDMA/PS/Ethernet 측정값으로 다시 산출해야 합니다.", 0.95, 5.25, 11.25, 0.75, { fill: C.redFill, line: C.red, bold: true, size: 10.5 });
  footer(s, "PASS 조건: processing + stall <= 13.888889 us - round_trip - reserve");
}

{
  const s = pptx.addSlide();
  title(s, "Handoff Contracts", "C06 내부 blocker는 닫혔고, 다음 단계는 system 계약을 닫아야 합니다.");
  table(s, [
    ["계약", "내용"],
    ["Hit[16]", "final VDMA stream에서는 버림. SW는 16-bit hit slot 수락"],
    ["Sticky", "주요 sticky는 err_soft_clear. quarantine_escape는 reset-only"],
    ["Width", "32/64/128 baseline + recovery + bounded backpressure PASS"],
    ["System", "8 us reserve, board STA, 장기 VDMA stall은 C07에서 닫음"],
  ], 0.85, 1.45, [2.4, 8.8], 0.62, 8.0);
  box(s, "결론: GO_WITH_HARDENED_CONTRACT", 3.3, 5.55, 6.7, 0.70, { fill: C.greenFill, line: C.green, bold: true, size: 15 });
  footer(s, "후속: C06_Control_Status_Integration_260514144755_C07_Handoff_v002.md");
}

pptx.writeFile({ fileName: out });
