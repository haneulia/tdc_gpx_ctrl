const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C03_Cell_Pipe/C03_Cell_Pipe_260501021013_Plan_v001.pptx";
const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.title = "C03 Cell Pipe Plan v001";
pptx.lang = "ko-KR";
pptx.theme = { headFontFace: "Malgun Gothic", bodyFontFace: "Malgun Gothic", lang: "ko-KR" };

const font = "Malgun Gothic";
const C = {
  bg: "FAFBFD", ink: "172033", muted: "64748B", line: "CBD5E1",
  table: "E2E8F0", card: "FFFFFF", blue: "2563EB", blueFill: "EFF6FF",
  green: "16A34A", greenFill: "ECFDF5", orange: "EA580C", orangeFill: "FFF7ED",
  red: "DC2626", redFill: "FEF2F2", cyan: "0891B2", cyanFill: "ECFEFF",
  violet: "7C3AED", violetFill: "F5F3FF"
};

function bg(slide) { slide.background = { color: C.bg }; }
function text(slide, value, x, y, w, h, opt = {}) {
  slide.addText(value, {
    x, y, w, h, fontFace: font, fontSize: opt.size || 12,
    bold: opt.bold || false, color: opt.color || C.ink,
    align: opt.align || "left", valign: "mid", fit: "shrink",
    margin: opt.margin === undefined ? 0.04 : opt.margin, breakLine: false
  });
}
function box(slide, value, x, y, w, h, opt = {}) {
  slide.addShape(pptx.ShapeType.roundRect, {
    x, y, w, h, rectRadius: 0.05,
    fill: { color: opt.fill || C.card },
    line: { color: opt.line || C.line, width: 1 }
  });
  text(slide, value, x + 0.07, y + 0.05, w - 0.14, h - 0.1, {
    size: opt.size || 11.5, bold: opt.bold || false,
    color: opt.color || C.ink, align: opt.align || "center"
  });
}
function arrow(slide, x1, y1, x2, y2, color = C.muted) {
  slide.addShape(pptx.ShapeType.line, {
    x: x1, y: y1, w: x2 - x1, h: y2 - y1,
    line: { color, width: 1.4, beginArrowType: "none", endArrowType: "triangle" }
  });
}
function title(slide, main, sub) {
  bg(slide);
  text(slide, main, 0.55, 0.27, 12.2, 0.46, { size: 21.5, bold: true });
  text(slide, sub, 0.58, 0.74, 12.0, 0.28, { size: 9.4, color: C.muted });
  slide.addShape(pptx.ShapeType.line, { x: 0.55, y: 1.08, w: 12.2, h: 0, line: { color: C.line, width: 0.8 } });
}
function foot(slide, value) { text(slide, value, 0.62, 6.95, 12.1, 0.25, { size: 8.1, color: C.muted, align: "center" }); }
function table(slide, rows, x, y, widths, rowH = 0.42, size = 10.2) {
  rows.forEach((row, r) => {
    let cx = x;
    row.forEach((cell, c) => {
      box(slide, String(cell), cx, y + r * rowH, widths[c], rowH - 0.03, {
        fill: r === 0 ? C.table : C.card, line: C.line,
        size: r === 0 ? size : size - 0.4, bold: r === 0
      });
      cx += widths[c] + 0.04;
    });
  });
}

let s = pptx.addSlide();
bg(s);
text(s, "C03 Cell Pipe Plan", 0.75, 0.62, 12.0, 0.58, { size: 28, bold: true, color: C.blue });
text(s, "slope demux + cell_builder x8 분석 시작 계획", 0.78, 1.25, 11.8, 0.42, { size: 17, bold: true });
box(s, "입력\nC02 event stream", 0.95, 2.25, 2.35, 0.92, { fill: C.blueFill, line: C.blue, color: C.blue, size: 14, bold: true });
arrow(s, 3.3, 2.71, 4.0, 2.71, C.blue);
box(s, "C03\ncell_pipe / cell_builder", 4.0, 2.25, 3.05, 0.92, { fill: C.cyanFill, line: C.cyan, color: C.cyan, size: 14, bold: true });
arrow(s, 7.05, 2.71, 7.75, 2.71, C.cyan);
box(s, "출력\nC04 cell stream", 7.75, 2.25, 2.55, 0.92, { fill: C.greenFill, line: C.green, color: C.green, size: 14, bold: true });
box(s, "절대 기준: Datasheet. 범위: I-Mode single, 32/64/128-bit, 합성 가능한 VHDL.", 1.0, 4.5, 11.25, 0.60, { fill: C.orangeFill, line: C.orange, color: C.orange, size: 13, bold: true });
text(s, "작성/수정: 2026-05-01 02:10:13 KST", 0.95, 6.20, 8.0, 0.28, { size: 10.8, color: C.muted });
foot(s, "근거: tdc_gpx_top.vhd: Cluster 3, C02->C03 Handoff v001");

s = pptx.addSlide();
title(s, "분석 대상", "중심 RTL과 경계 RTL을 분리해서 본다.");
table(s, [
  ["구분", "파일", "확인 포인트"],
  ["중심", "tdc_gpx_cell_pipe.vhd", "registered slope demux, drain_done"],
  ["중심", "tdc_gpx_cell_builder.vhd", "dual buffer, shot/drop/timeout"],
  ["중심", "tdc_gpx_pkg.vhd", "cell helper, 16-bit slot"],
  ["경계", "tdc_gpx_decode_pipe.vhd", "event AXIS producer"],
  ["경계", "tdc_gpx_output_stage.vhd", "cell stream consumer"],
], 0.75, 1.35, [1.45, 3.2, 5.7], 0.50, 10.2);
box(s, "C03 분석은 C02 output 의미와 C04 input 요구를 동시에 만족하는지 보는 경계 분석이다.", 0.9, 5.75, 11.55, 0.58, { fill: C.greenFill, line: C.green, color: C.green, size: 12.8, bold: true });
foot(s, "문서에는 파일/라인 근거와 xsim 근거를 남긴다.");

s = pptx.addSlide();
title(s, "C03 핵심 타이밍", "shot_start, drain_done, IFIFO split output, tlast를 분리한다.");
box(s, "shot_start\nbuffer allocate", 0.85, 1.85, 2.0, 0.72, { fill: C.blueFill, line: C.blue, color: C.blue, size: 12.2, bold: true });
arrow(s, 2.85, 2.21, 3.45, 2.21, C.blue);
box(s, "event collect\nhit slots", 3.45, 1.85, 2.0, 0.72, { fill: C.cyanFill, line: C.cyan, color: C.cyan, size: 12.2, bold: true });
arrow(s, 5.45, 2.21, 6.05, 2.21, C.cyan);
box(s, "IFIFO1 done\noutput stops 0..3", 6.05, 1.85, 2.35, 0.72, { fill: C.greenFill, line: C.green, color: C.green, size: 12.2, bold: true });
arrow(s, 8.4, 2.21, 9.0, 2.21, C.green);
box(s, "wait IFIFO2\nor timeout", 9.0, 1.85, 2.0, 0.72, { fill: C.orangeFill, line: C.orange, color: C.orange, size: 12.2, bold: true });
arrow(s, 11.0, 2.21, 11.55, 2.21, C.orange);
box(s, "tlast\nslice done", 11.55, 1.85, 1.25, 0.72, { fill: C.violetFill, line: C.violet, color: C.violet, size: 11.5, bold: true });
box(s, "측정 항목: latency / throughput / pipeline / II를 event accept, collect, output serialize로 분리한다.", 1.0, 4.65, 11.25, 0.65, { fill: C.redFill, line: C.red, color: C.red, size: 12.7, bold: true });
foot(s, "첫 분석 산출물: C03_Cell_Pipe_<timestamp>_Analysis_v001.md/.pptx");

pptx.writeFile({ fileName: out });
