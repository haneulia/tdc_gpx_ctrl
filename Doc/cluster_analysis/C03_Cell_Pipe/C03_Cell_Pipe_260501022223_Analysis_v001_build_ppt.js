const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C03_Cell_Pipe/C03_Cell_Pipe_260501022223_Analysis_v001.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C03 Cell Pipe Analysis";
pptx.title = "C03 Cell Pipe Analysis v001";
pptx.lang = "ko-KR";
pptx.theme = { headFontFace: "Malgun Gothic", bodyFontFace: "Malgun Gothic", lang: "ko-KR" };

const font = "Malgun Gothic";
const C = {
  bg: "FAFBFD",
  ink: "172033",
  muted: "64748B",
  line: "CBD5E1",
  table: "E2E8F0",
  card: "FFFFFF",
  blue: "2563EB",
  blueFill: "EFF6FF",
  green: "16A34A",
  greenFill: "ECFDF5",
  orange: "EA580C",
  orangeFill: "FFF7ED",
  red: "DC2626",
  redFill: "FEF2F2",
  cyan: "0891B2",
  cyanFill: "ECFEFF",
  violet: "7C3AED",
  violetFill: "F5F3FF"
};

function bg(slide) { slide.background = { color: C.bg }; }
function text(slide, value, x, y, w, h, opt = {}) {
  slide.addText(value, {
    x, y, w, h,
    fontFace: font,
    fontSize: opt.size || 12,
    bold: opt.bold || false,
    color: opt.color || C.ink,
    align: opt.align || "left",
    valign: "mid",
    fit: "shrink",
    margin: opt.margin === undefined ? 0.04 : opt.margin,
    breakLine: false
  });
}
function box(slide, value, x, y, w, h, opt = {}) {
  slide.addShape(pptx.ShapeType.roundRect, {
    x, y, w, h,
    rectRadius: 0.05,
    fill: { color: opt.fill || C.card },
    line: { color: opt.line || C.line, width: 1 }
  });
  text(slide, value, x + 0.07, y + 0.05, w - 0.14, h - 0.1, {
    size: opt.size || 11.5,
    bold: opt.bold || false,
    color: opt.color || C.ink,
    align: opt.align || "center"
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
  text(slide, sub, 0.58, 0.74, 12.0, 0.28, { size: 9.3, color: C.muted });
  slide.addShape(pptx.ShapeType.line, { x: 0.55, y: 1.08, w: 12.2, h: 0, line: { color: C.line, width: 0.8 } });
}
function foot(slide, value) {
  text(slide, value, 0.62, 6.95, 12.1, 0.25, { size: 8.1, color: C.muted, align: "center" });
}
function table(slide, rows, x, y, widths, rowH = 0.42, size = 10.0) {
  rows.forEach((row, r) => {
    let cx = x;
    row.forEach((cell, c) => {
      box(slide, String(cell), cx, y + r * rowH, widths[c], rowH - 0.03, {
        fill: r === 0 ? C.table : C.card,
        line: C.line,
        size: r === 0 ? size : size - 0.35,
        bold: r === 0
      });
      cx += widths[c] + 0.04;
    });
  });
}

let s = pptx.addSlide();
bg(s);
text(s, "C03 Cell Pipe Analysis", 0.72, 0.62, 12.0, 0.58, { size: 28, bold: true, color: C.blue });
text(s, "decoded event를 per-chip/per-slope cell slice로 변환하는 구간", 0.74, 1.23, 11.8, 0.42, { size: 17, bold: true });
box(s, "Datasheet 기준\nHit[16:0]", 0.9, 2.15, 2.35, 0.95, { fill: C.blueFill, line: C.blue, color: C.blue, size: 14, bold: true });
box(s, "C03 구조\ncell_pipe + cell_builder", 3.55, 2.15, 2.9, 0.95, { fill: C.cyanFill, line: C.cyan, color: C.cyan, size: 14, bold: true });
box(s, "baseline xsim\nsmoke PASS", 6.75, 2.15, 2.35, 0.95, { fill: C.greenFill, line: C.green, color: C.green, size: 14, bold: true });
box(s, "close 전 위험\n3개 finding", 9.4, 2.15, 2.35, 0.95, { fill: C.redFill, line: C.red, color: C.red, size: 14, bold: true });
box(s, "판단: C03 분석은 진행 가능하지만, 17-bit 보존과 handshake 안전화 없이는 C03 close로 보기 어렵다.", 1.0, 4.35, 11.25, 0.72, { fill: C.orangeFill, line: C.orange, color: C.orange, size: 13.2, bold: true });
text(s, "작성/수정: 2026-05-01 02:22:23 KST", 0.95, 6.18, 8.0, 0.28, { size: 10.7, color: C.muted });
foot(s, "근거: Doc/TDC-GPX-Datasheet.pdf, tdc_gpx_cell_pipe.vhd, tdc_gpx_cell_builder.vhd, xsim_c03_cell_pipe_base.log");

s = pptx.addSlide();
title(s, "Data Flow", "C02 event stream을 slope별 cell_builder로 나누고 C04 output_stage로 넘긴다.");
box(s, "C02\nIFIFO raw read", 0.65, 2.0, 1.65, 0.72, { fill: C.blueFill, line: C.blue, color: C.blue, size: 11.5, bold: true });
arrow(s, 2.3, 2.36, 2.85, 2.36, C.blue);
box(s, "decoder\n28-bit fields", 2.85, 2.0, 1.75, 0.72, { fill: C.blueFill, line: C.blue, color: C.blue, size: 11.5, bold: true });
arrow(s, 4.6, 2.36, 5.15, 2.36, C.blue);
box(s, "raw_event\nshot/hit seq", 5.15, 2.0, 1.75, 0.72, { fill: C.violetFill, line: C.violet, color: C.violet, size: 11.5, bold: true });
arrow(s, 6.9, 2.36, 7.45, 2.36, C.violet);
box(s, "cell_pipe\nslope demux", 7.45, 2.0, 1.75, 0.72, { fill: C.cyanFill, line: C.cyan, color: C.cyan, size: 11.5, bold: true });
arrow(s, 9.2, 2.18, 9.75, 1.75, C.cyan);
arrow(s, 9.2, 2.55, 9.75, 2.95, C.cyan);
box(s, "rise\nbuilder x4", 9.75, 1.35, 1.5, 0.62, { fill: C.greenFill, line: C.green, color: C.green, size: 11, bold: true });
box(s, "fall\nbuilder x4", 9.75, 2.75, 1.5, 0.62, { fill: C.greenFill, line: C.green, color: C.green, size: 11, bold: true });
arrow(s, 11.25, 2.04, 11.8, 2.36, C.green);
arrow(s, 11.25, 3.06, 11.8, 2.52, C.green);
box(s, "C04\nface/header", 11.8, 2.0, 1.05, 0.72, { fill: C.orangeFill, line: C.orange, color: C.orange, size: 10, bold: true });
box(s, "event tuser 핵심: slope, chip_id, stop_id, ififo_id, drain_done, hit_seq, shot_seq", 1.0, 4.7, 11.25, 0.55, { fill: C.table, line: C.line, size: 12.1, bold: true });
foot(s, "근거: tdc_gpx_raw_event_builder.vhd:63-73, tdc_gpx_cell_pipe.vhd:149-237");

s = pptx.addSlide();
title(s, "Timing / Pipeline", "정상 경로에서 ififo1_done 이후 cell output이 시작되고 IFIFO2 경계에서 wait 가능하다.");
box(s, "shot_start\nBUF_COLLECT", 0.8, 1.6, 1.7, 0.65, { fill: C.blueFill, line: C.blue, color: C.blue, size: 11.5, bold: true });
arrow(s, 2.5, 1.93, 3.0, 1.93, C.blue);
box(s, "hit store\n+1 clk", 3.0, 1.6, 1.45, 0.65, { fill: C.cyanFill, line: C.cyan, color: C.cyan, size: 11.5, bold: true });
arrow(s, 4.45, 1.93, 4.95, 1.93, C.cyan);
box(s, "ififo1_done\nBUF_SHARED", 4.95, 1.6, 1.8, 0.65, { fill: C.greenFill, line: C.green, color: C.green, size: 11.5, bold: true });
arrow(s, 6.75, 1.93, 7.25, 1.93, C.green);
box(s, "ST_O_LOAD\n1 clk", 7.25, 1.6, 1.45, 0.65, { fill: C.orangeFill, line: C.orange, color: C.orange, size: 11.5, bold: true });
arrow(s, 8.7, 1.93, 9.2, 1.93, C.orange);
box(s, "first beat\nvalid", 9.2, 1.6, 1.45, 0.65, { fill: C.violetFill, line: C.violet, color: C.violet, size: 11.5, bold: true });
arrow(s, 10.65, 1.93, 11.15, 1.93, C.violet);
box(s, "tlast\nfaulted", 11.15, 1.6, 1.35, 0.65, { fill: C.redFill, line: C.red, color: C.red, size: 11.3, bold: true });
table(s, [
  ["구간", "최소 지연 / II"],
  ["input hit -> buffer store", "1 clk"],
  ["input ififo1_done -> first valid", "약 3 clk"],
  ["same-cell beat", "II=1"],
  ["cell boundary", "1 invalid cycle"],
  ["stop3 -> stop4", "IFIFO2 wait 가능"],
], 1.1, 3.35, [4.2, 5.0], 0.42, 10.2);
foot(s, "근거: tdc_gpx_cell_builder.vhd:654-698, 915-1052");

s = pptx.addSlide();
title(s, "Throughput", "16-bit slot + metadata 1 beat 기준의 C03 cell slice 비용");
table(s, [
  ["max_hits", "Bcell 32", "Bcell 64", "Bcell 128"],
  ["1", "2", "2", "2"],
  ["3", "3", "2", "2"],
  ["5", "4", "3", "2"],
  ["7", "5", "3", "2"],
], 0.75, 1.35, [1.8, 1.8, 1.8, 1.8], 0.46, 10.5);
table(s, [
  ["case", "valid beats", "cycles", "time @200MHz"],
  ["max3 32b", "24", "31", "155 ns"],
  ["max3 64/128b", "16", "23", "115 ns"],
  ["max7 32b", "40", "47", "235 ns"],
  ["max7 64b", "24", "31", "155 ns"],
  ["max7 128b", "16", "23", "115 ns"],
], 6.0, 1.35, [2.0, 1.55, 1.25, 1.7], 0.46, 10.0);
box(s, "주의: C03 cell stream에는 stop cell 사이 1-cycle bubble이 있다. C04 final AXIS beat 수와 C03 valid cycle 수는 같은 값이 아니다.", 0.9, 5.85, 11.55, 0.58, { fill: C.orangeFill, line: C.orange, color: C.orange, size: 12.3, bold: true });
foot(s, "근거: tdc_gpx_pkg.vhd:805-820, tdc_gpx_cell_builder.vhd:948-1011");

s = pptx.addSlide();
title(s, "Findings", "C03 close 전 code-fix plan으로 넘겨야 할 항목");
table(s, [
  ["ID", "Priority", "판단"],
  ["F-C03-01", "P1", "17-bit raw hit 중 bit16이 cell slot에서 손실 가능"],
  ["F-C03-02", "P1", "registered ready stale 상태에서 event beat 손실 가능"],
  ["F-C03-03", "P2", "per-slope abort가 demux holding register를 clear하지 않음"],
  ["F-C03-04", "P2", "기존 TB는 단일 rising 64-bit smoke test 수준"],
], 0.75, 1.35, [1.6, 1.3, 8.1], 0.55, 10.5);
box(s, "권장 다음 단계\n1. 17-bit 보존 방식 결정\n2. demux ready/valid skid 보완\n3. per-slope abort reset 보완\n4. width/max_hits/backpressure/dual-buffer TB 추가", 1.0, 5.0, 11.25, 1.05, { fill: C.redFill, line: C.red, color: C.red, size: 13.0, bold: true });
foot(s, "baseline: xsim_c03_cell_pipe_base.log:28 PASS, closure 증거로는 부족");

pptx.writeFile({ fileName: out });
