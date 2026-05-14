const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/.pnpm/pptxgenjs@4.0.1/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C07_System_Integration/C07_System_Integration_260514153208_Chain_Stress_Result_v001.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C07 Chain Stress Result";
pptx.title = "C07 Chain Stress Result v001";
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
  tx(s, "C07 Chain Stress", 0.72, 0.75, 7.5, 0.42, { size: 18, bold: true, color: "93C5FD" });
  tx(s, "P0-01 / P0-02\nExecution Result", 0.72, 1.28, 8.7, 1.05, { size: 30, bold: true, color: C.white });
  tx(s, "RTL 내부 output stream CDC는 architecture contract로 닫고, 32/64/128 x max_hits 1/3/5/7 x bounded backpressure chain stress는 PASS했습니다.", 0.78, 2.72, 11.6, 0.6, { size: 13, color: "E5E7EB" });
  box(s, "PASS\n13 C07 sims", 0.95, 4.15, 2.3, 0.9, { fill: "14532D", line: "86EFAC", color: C.white, bold: true, size: 12 });
  box(s, "PASS\n32/64/128", 3.75, 4.15, 2.3, 0.9, { fill: "1E3A8A", line: "93C5FD", color: C.white, bold: true, size: 12 });
  box(s, "PASS\nmax_hits 1/3/5/7", 6.55, 4.15, 2.3, 0.9, { fill: "4C1D95", line: "C4B5FD", color: C.white, bold: true, size: 12 });
  box(s, "OPEN\nreserve/marker audit", 9.35, 4.15, 2.55, 0.9, { fill: "713F12", line: "FDE68A", color: C.white, bold: true, size: 12 });
  footer(s, "Archive: sim_results/vivado_xsim/sessions/260514153208_c07_v001_chain_stress/");
}

{
  const s = pptx.addSlide();
  title(s, "Architecture Closure", "top 내부는 raw CDC 이후 전부 i_axis_aclk 단일 stream domain입니다.");
  const xs = [0.9, 3.1, 5.3, 7.5, 9.7];
  const labels = [
    ["i_tdc_clk\nbus/chip_ctrl", C.orangeFill, C.orange],
    ["raw CDC\nxpm_fifo_async", C.blueFill, C.blue],
    ["i_axis_aclk\ndecode/cell", C.tealFill, C.teal],
    ["output_stage\nface/header", C.purpleFill, C.purple],
    ["final AXIS\nVDMA contract", C.greenFill, C.green],
  ];
  labels.forEach(([label, fill, line], i) => {
    box(s, label, xs[i], 2.1, 1.55, 0.9, { fill, line, bold: true, size: 9.2 });
    if (i < labels.length - 1) arrow(s, xs[i] + 1.58, 2.55, xs[i + 1] - 0.08, 2.55);
  });
  box(s, "Closure: RTL scope에서는 현재 구조 수락. 단, VDMA가 다른 clock이면 top 외부 AXIS CDC FIFO 또는 clocking 계약이 필요합니다.", 1.0, 4.55, 11.25, 0.78, { fill: C.orangeFill, line: C.orange, bold: true, size: 12.3 });
  footer(s, "근거: tdc_gpx_config_ctrl.vhd:1909, tdc_gpx_top.vhd:735/741, tdc_gpx_header_inserter.vhd:326");
}

{
  const s = pptx.addSlide();
  title(s, "Verification Matrix", "공통 조건: faces=2, cols=2, active chips=4, stops_per_chip=2, bp_gap=17, ASYNC raw CDC");
  table(s, [
    ["Width", "max_hits", "Beats/lane", "TLAST", "T2->T5 worst"],
    ["32", "1 / 3 / 5 / 7", "112 / 144 / 176 / 208", "4", "0.500 / 0.540 / 0.575 / 0.620 us"],
    ["64", "1 / 3 / 5 / 7", "88 / 88 / 120 / 120", "4", "0.500 / 0.500 / 0.540 / 0.540 us"],
    ["128", "1 / 3 / 5 / 7", "76 / 76 / 76 / 76", "4", "0.500 us all"],
  ], 0.65, 1.45, [1.25, 2.4, 3.7, 1.2, 3.1], 0.64, 8.1);
  box(s, "해석: 넓은 bus는 빠르지만, 이득은 output packing 경계 때문에 계단식으로 나타납니다.", 1.15, 5.25, 10.9, 0.65, { fill: C.blueFill, line: C.blue, bold: true, size: 12 });
  footer(s, "대표 증거: w32/mh7 log line 161/167/169, w128/mh7 log line 161/167/169");
}

{
  const s = pptx.addSlide();
  title(s, "Timing / Pipeline", "T2는 IrFlag/drain condition, T5는 final TLAST입니다.");
  const xs = [1.0, 2.85, 4.7, 6.55, 8.4, 10.25];
  const labels = ["T0\nstart", "T1\nfire final", "T2\nIrFlag", "C02/C03\nraw/cell", "C04\noutput", "T5\nTLAST"];
  labels.forEach((label, i) => {
    box(s, label, xs[i], 2.05, 1.25, 0.78, { fill: C.blueFill, line: C.blue, bold: true, size: 8.7 });
    if (i < labels.length - 1) arrow(s, xs[i] + 1.28, 2.44, xs[i + 1] - 0.08, 2.44);
  });
  table(s, [
    ["Case", "T2 cycle", "T5 cycle", "Delta"],
    ["32-bit / mh7", "2983", "3107", "124 clk / 0.620 us"],
    ["64-bit / mh7", "2983", "3091", "108 clk / 0.540 us"],
    ["128-bit / mh7", "2983", "3083", "100 clk / 0.500 us"],
  ], 1.15, 4.05, [3.0, 2.0, 2.0, 3.6], 0.48, 8.2);
  footer(s, "주의: T4 first beat는 header prefix일 수 있어 raw data first beat와 동일하지 않습니다.");
}

{
  const s = pptx.addSlide();
  title(s, "CTL21 Late Write", "max_hits_cfg는 face/shot 시작 전에 설정하는 전략이 여전히 권장됩니다.");
  table(s, [
    ["Scenario", "Evidence", "Result"],
    ["early write 생략", "log line 55", "observed"],
    ["first packet_start 이후 CTL21 write", "log line 98", "observed"],
    ["64-bit late mh3 summary", "log line 162", "104 beats/lane"],
    ["bounded BP + output", "line 168 / 170", "PASS"],
  ], 0.85, 1.5, [3.3, 3.3, 4.4], 0.56, 8.2);
  box(s, "late write는 동작상 PASS지만 첫 face와 이후 face의 beat budget이 달라지므로 release 기본 운용으로는 권장하지 않습니다.", 1.05, 5.35, 11.1, 0.72, { fill: C.orangeFill, line: C.orange, bold: true, size: 12 });
  footer(s, "근거: xsim_c07_v001_top_int_w64_late_mh3_260514153208.log");
}

{
  const s = pptx.addSlide();
  title(s, "Remaining Work", "C07 P0 중 두 항목은 닫혔고, system/marker 성격의 항목이 남았습니다.");
  table(s, [
    ["ID", "Status", "Next"],
    ["P0-01", "RTL scope closed", "외부 VDMA clock CDC/STA는 system item"],
    ["P0-02", "PASS", "matrix 결과 유지"],
    ["P0-03", "OPEN", "C02/C04 source/dest/effect/output marker audit"],
    ["P0-04", "OPEN", "8 us reserve 실측 또는 보수치 갱신"],
    ["P1", "OPEN", "C03/C04 direct matrix TB"],
  ], 0.75, 1.45, [1.2, 2.5, 7.5], 0.55, 7.8);
  footer(s, "다음 권고: Marker Audit Result v001 작성 후 Reserve Measurement로 진행");
}

pptx.writeFile({ fileName: out });
