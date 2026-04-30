const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C02_Chip_Acquisition/C02_Chip_Acquisition_260501021013_C03_Handoff_v001.pptx";
const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.title = "C02 to C03 Handoff v001";
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
text(s, "C02 -> C03 Handoff", 0.75, 0.62, 12.0, 0.58, { size: 28, bold: true, color: C.blue });
text(s, "C02는 닫고 C03 Cell Pipe로 진입 가능", 0.78, 1.25, 11.8, 0.42, { size: 17, bold: true });
box(s, "C02 완료\nacquisition / data flow\nwidth timing 검증", 0.9, 2.2, 3.0, 1.05, { fill: C.greenFill, line: C.green, color: C.green, size: 14, bold: true });
box(s, "인계 계약\nCTL21 / face snapshot\nI-Mode single", 4.25, 2.2, 3.0, 1.05, { fill: C.blueFill, line: C.blue, color: C.blue, size: 14, bold: true });
box(s, "C03 범위\ncell_pipe / cell_builder\nslope demux + cell slice", 7.6, 2.2, 3.55, 1.05, { fill: C.cyanFill, line: C.cyan, color: C.cyan, size: 14, bold: true });
box(s, "판단: C03 진입 가능. 단, C02 산출물과 현재 변경분은 git commit으로 닫고 시작한다.", 1.0, 4.45, 11.25, 0.65, { fill: C.orangeFill, line: C.orange, color: C.orange, size: 13.2, bold: true });
text(s, "작성/수정: 2026-05-01 02:10:13 KST", 0.95, 6.20, 8.0, 0.28, { size: 10.8, color: C.muted });
foot(s, "근거: tdc_gpx_top.vhd Cluster 주석, C02 xsim PASS logs, C02 width/timing/CTL21 문서");

s = pptx.addSlide();
title(s, "C02 종료 근거", "기능 PASS와 운용 계약이 C03 입력 조건으로 정리되었다.");
table(s, [
  ["검증", "결과", "근거"],
  ["32-bit top", "PASS 72 beats", "xsim_top_int_width32.log:92"],
  ["64-bit top", "PASS 44 beats", "xsim_top_int_width64.log:92"],
  ["128-bit top", "PASS 38 beats", "xsim_top_int_width128.log:92"],
  ["width matrix", "PASS", "xsim_width_timing_matrix.log:110"],
  ["CTL21 early/unset/zero/late", "PASS", "xsim_top_ctl21_*.log"],
], 0.75, 1.35, [2.45, 2.5, 5.5], 0.50, 10.5);
box(s, "C02의 남은 항목은 blocker가 아니라 C03 입력 또는 후속 Cluster 검토 항목으로 분리한다.", 0.9, 5.6, 11.55, 0.58, { fill: C.greenFill, line: C.green, color: C.green, size: 12.8, bold: true });
foot(s, "C02 산출: Data Flow v002, Timing/Pipeline/II v001, Width Verification v001, CTL21 Contract v001");

s = pptx.addSlide();
title(s, "C02 -> C03 Data Boundary", "C03는 decoded event stream을 per-slope/per-chip cell slice로 변환한다.");
box(s, "C02 decode_pipe\nAXIS event x4", 0.8, 2.0, 2.35, 0.78, { fill: C.blueFill, line: C.blue, color: C.blue, size: 12.5, bold: true });
arrow(s, 3.15, 2.39, 3.85, 2.39, C.blue);
box(s, "C03 cell_pipe\nregistered slope demux", 3.85, 2.0, 2.65, 0.78, { fill: C.cyanFill, line: C.cyan, color: C.cyan, size: 12.5, bold: true });
arrow(s, 6.5, 2.22, 7.2, 1.75, C.cyan);
arrow(s, 6.5, 2.56, 7.2, 3.02, C.cyan);
box(s, "cell_builder rising x4", 7.2, 1.35, 2.35, 0.68, { fill: C.greenFill, line: C.green, color: C.green, size: 12.5, bold: true });
box(s, "cell_builder falling x4", 7.2, 2.82, 2.35, 0.68, { fill: C.greenFill, line: C.green, color: C.green, size: 12.5, bold: true });
arrow(s, 9.55, 1.70, 10.4, 2.25, C.green);
arrow(s, 9.55, 3.16, 10.4, 2.65, C.green);
box(s, "C04 output_stage\nface/header", 10.4, 2.0, 2.0, 0.78, { fill: C.orangeFill, line: C.orange, color: C.orange, size: 12.5, bold: true });
box(s, "중요 tuser: slope, chip_id, stop_id, ififo_id, drain_done, hit_seq, shot_seq", 1.0, 4.6, 11.25, 0.55, { fill: C.violetFill, line: C.violet, color: C.violet, size: 12.5, bold: true });
foot(s, "근거: tdc_gpx_cell_pipe.vhd ports, tdc_gpx_cell_builder.vhd AXIS contract comments");

s = pptx.addSlide();
title(s, "C03 인계 계약", "C02에서 닫힌 계약을 C03 분석의 입력 조건으로 고정한다.");
table(s, [
  ["계약", "C03 적용"],
  ["I-Mode single only", "cell conversion 검증 범위"],
  ["CTL21 before packet_start", "runtime beat count 안정성"],
  ["000 -> 7 alias", "effective max_hits 1..7"],
  ["face-local shot count", "shot_start / buffer ownership 기준"],
  ["lower 16-bit hit slot", "full 17-bit는 finding 후보"],
  ["32/64/128 only", "256-bit 제외"],
], 1.0, 1.35, [3.4, 7.5], 0.50, 10.5);
box(s, "C03 질문: slope demux ready, drain_done 양 slope 전달, dual buffer ownership, timeout/drop/fault tuser 정합성", 1.0, 5.65, 11.25, 0.62, { fill: C.redFill, line: C.red, color: C.red, size: 12.4, bold: true });
foot(s, "다음 산출물: C03_Cell_Pipe Plan v001");

pptx.writeFile({ fileName: out });
