const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C02_Chip_Acquisition/C02_Chip_Acquisition_260501012435_Timing_Pipeline_II_Analysis_v001.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C02 timing pipeline II analysis";
pptx.title = "C02 Timing Pipeline II v001";
pptx.lang = "ko-KR";
pptx.theme = { headFontFace: "Malgun Gothic", bodyFontFace: "Malgun Gothic", lang: "ko-KR" };

const C = {
  bg: "FAFBFD", ink: "172033", muted: "64748B", gray: "CBD5E1", header: "E2E8F0", card: "FFFFFF",
  blue: "2563EB", blueFill: "EFF6FF", green: "16A34A", greenFill: "ECFDF5",
  orange: "EA580C", orangeFill: "FFF7ED", red: "DC2626", redFill: "FEF2F2",
  violet: "7C3AED", violetFill: "F5F3FF", cyan: "0891B2", cyanFill: "ECFEFF"
};
const font = "Malgun Gothic";

function bg(slide) { slide.background = { color: C.bg }; }
function txt(slide, text, x, y, w, h, size = 15, bold = false, color = C.ink, align = "left") {
  slide.addText(text, { x, y, w, h, fontFace: font, fontSize: size, bold, color, align, valign: "mid", fit: "shrink", margin: 0.03 });
}
function box(slide, text, x, y, w, h, fill = C.card, line = C.gray, size = 13, bold = false, color = C.ink) {
  slide.addShape(pptx.ShapeType.roundRect, { x, y, w, h, rectRadius: 0.04, fill: { color: fill }, line: { color: line, width: 1 } });
  txt(slide, text, x + 0.06, y + 0.05, w - 0.12, h - 0.1, size, bold, color, "center");
}
function arrow(slide, x1, y1, x2, y2, color = C.muted, width = 1.5) {
  slide.addShape(pptx.ShapeType.line, { x: x1, y: y1, w: x2 - x1, h: y2 - y1, line: { color, width, beginArrowType: "none", endArrowType: "triangle" } });
}
function title(slide, main, sub) {
  bg(slide);
  txt(slide, main, 0.55, 0.3, 12.3, 0.45, 22, true, C.ink);
  txt(slide, sub, 0.58, 0.78, 12.2, 0.3, 9.5, false, C.muted);
  slide.addShape(pptx.ShapeType.line, { x: 0.55, y: 1.15, w: 12.2, h: 0, line: { color: C.gray, width: 0.8 } });
}
function table(slide, rows, x, y, widths, rowH = 0.45, size = 10.3) {
  rows.forEach((row, r) => {
    let cx = x;
    row.forEach((cell, c) => {
      box(slide, cell, cx, y + r * rowH, widths[c], rowH - 0.035, r === 0 ? C.header : C.card, C.gray, r === 0 ? size : size - 0.8, r === 0);
      cx += widths[c] + 0.04;
    });
  });
}
function foot(slide, text) { txt(slide, text, 0.65, 6.95, 12.0, 0.25, 8.3, false, C.muted, "center"); }

let s = pptx.addSlide();
bg(s);
txt(s, "C02 Timing / Pipeline / II", 0.75, 0.7, 12.0, 0.55, 30, true, C.blue);
txt(s, "32 / 64 / 128-bit output width timing analysis", 0.75, 1.32, 12.0, 0.55, 23, true, C.ink);
box(s, "GPX READ\n폭 영향 없음", 0.95, 2.35, 2.65, 1.0, C.blueFill, C.blue, 16, true, C.blue);
box(s, "Raw/Event\nII=1 처리 여유", 3.95, 2.35, 2.65, 1.0, C.cyanFill, C.cyan, 16, true, C.cyan);
box(s, "Cell/Header\nbeat 수 감소", 6.95, 2.35, 2.65, 1.0, C.greenFill, C.green, 16, true, C.green);
box(s, "Final AXIS\n2x / 4x capacity", 9.95, 2.35, 2.35, 1.0, C.orangeFill, C.orange, 16, true, C.orange);
txt(s, "핵심: 넓은 bus는 output 구간을 빠르게 한다. end-to-end 개선율은 GPX READ와 serializer overhead가 함께 결정한다.", 0.95, 4.25, 11.4, 0.55, 15, true, C.orange, "center");
txt(s, "작성 시간: 2026-05-01 01:24:35 KST", 0.9, 6.35, 8.0, 0.3, 12, false, C.muted);

s = pptx.addSlide();
title(s, "Pipeline Stage", "폭 영향은 cell_builder 이후에 집중된다.");
const nodes = [
  ["GPX READ\n200MHz ctrl", 0.55, C.blueFill, C.blue],
  ["raw CDC\n40b", 2.2, C.card, C.gray],
  ["decode\n~4 clk", 3.75, C.cyanFill, C.cyan],
  ["cell\nserialize", 5.35, C.greenFill, C.green],
  ["face\nchip order", 7.0, C.violetFill, C.violet],
  ["header\n12/6/3", 8.75, C.orangeFill, C.orange],
  ["AXIS\n32/64/128", 10.55, C.redFill, C.red],
];
nodes.forEach(([label, x, fill, line], i) => { box(s, label, x, 2.0, 1.25, 0.78, fill, line, 10.5, true, line); if (i < nodes.length - 1) arrow(s, x + 1.25, 2.39, nodes[i + 1][1], 2.39); });
box(s, "폭 영향 없음\nGPX/read/raw/event", 1.05, 3.75, 4.35, 0.75, C.blueFill, C.blue, 14, true, C.blue);
box(s, "폭 영향 큼\ncell/header/final AXIS", 6.2, 3.75, 5.1, 0.75, C.greenFill, C.green, 14, true, C.green);
foot(s, "근거: config_ctrl raw CDC, decode_pipe skid, cell_builder, face_assembler, output_stage, header_inserter");

s = pptx.addSlide();
title(s, "Beat 수 모델", "LineBeats = Header + active_chips * stops * Bcell(max_hits, W)");
table(s, [
  ["max_hits", "32-bit", "64-bit", "128-bit"],
  ["1", "76", "70", "67"],
  ["3", "108", "70", "67"],
  ["5", "140", "102", "67"],
  ["7", "172", "102", "67"],
], 1.0, 1.45, [2.2, 2.3, 2.3, 2.3], 0.55, 12);
box(s, "Header beats\n32=12, 64=6, 128=3", 1.0, 4.45, 3.3, 0.72, C.blueFill, C.blue, 13.5, true, C.blue);
box(s, "Cell beats max7\n32=5, 64=3, 128=2", 4.95, 4.45, 3.3, 0.72, C.greenFill, C.green, 13.5, true, C.green);
box(s, "Low max_hits에서는\n개선율이 작아짐", 8.9, 4.45, 3.3, 0.72, C.orangeFill, C.orange, 13.5, true, C.orange);
foot(s, "조건: C=4 chips, N=8 stops, data already available, downstream ready=1");

s = pptx.addSlide();
title(s, "II 분석", "beat II=1과 cell/line 완료 시간은 다르다.");
table(s, [
  ["구간", "II", "폭 영향"],
  ["GPX READ", ">=25ns/read", "없음"],
  ["decode/event", "1 axis clk/beat", "없음"],
  ["cell 내부", "1 axis clk/beat", "Bcell 감소"],
  ["cell 간", "Bcell + 1 근사", "bubble 포함"],
  ["header/data", "1 axis clk/beat", "beat 수 감소"],
  ["final AXIS", "1 beat/clk", "byte/beat 증가"],
], 0.75, 1.35, [3.0, 3.0, 5.2], 0.5, 11);
txt(s, "넓은 bus는 beat II를 줄이는 것이 아니라, 같은 데이터를 더 적은 beat로 보내 cell/line 완료 시간을 줄인다.", 0.95, 5.55, 11.4, 0.45, 15, true, C.orange, "center");

s = pptx.addSlide();
title(s, "Latency / Throughput", "150MHz output 기준 final wire capacity와 line time.");
table(s, [
  ["Width", "Byte/beat", "Wire throughput", "max7 line cycles", "max7 line time"],
  ["32-bit", "4", "600 MB/s", "172", "1.147 us"],
  ["64-bit", "8", "1.2 GB/s", "102", "0.680 us"],
  ["128-bit", "16", "2.4 GB/s", "67", "0.447 us"],
], 0.65, 1.35, [1.6, 1.8, 2.3, 2.3, 2.3], 0.55, 10.5);
box(s, "Final AXIS capacity\n32->64 = 2x\n32->128 = 4x", 1.0, 4.75, 3.35, 0.9, C.blueFill, C.blue, 14, true, C.blue);
box(s, "Live serializer\nmetadata + bubble 때문에\n개선율은 workload 의존", 4.95, 4.75, 3.35, 0.9, C.greenFill, C.green, 13.5, true, C.green);
box(s, "GPX READ 병목이면\nend-to-end 개선 제한", 8.9, 4.75, 3.35, 0.9, C.redFill, C.red, 14, true, C.red);

s = pptx.addSlide();
title(s, "검증 경계", "분석 모델을 실제 xsim 계측으로 닫아야 하는 항목.");
table(s, [
  ["항목", "현재 상태", "다음 검증"],
  ["64-bit top", "PASS 로그 있음", "유지"],
  ["32-bit top", "최신 full top 로그 필요", "line/frame beat 계측"],
  ["128-bit top", "최신 full top 로그 필요", "line/frame beat 계측"],
  ["max_hits matrix", "수식 분석 완료", "cfg별 output beat count"],
  ["주석 debt", "17-bit comment stale", "주석 보완"],
], 0.75, 1.35, [2.6, 4.0, 5.0], 0.52, 10.8);
box(s, "판단\n32/64/128 폭 증가는 output 구간 timing 개선이 맞다.\n최종 선택은 workload별 max_hits와 GPX READ 병목 여부로 결정한다.", 1.0, 5.35, 11.35, 0.85, C.greenFill, C.green, 15, true, C.green);

pptx.writeFile({ fileName: out });
