const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/pptxgenjs");

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C02 AXI4-Stream Phase A Fix Result";
pptx.title = "C02 AXI4-Stream Width Standardization Phase A";
pptx.lang = "ko-KR";
pptx.theme = {
  headFontFace: "Malgun Gothic",
  bodyFontFace: "Malgun Gothic",
  lang: "ko-KR",
};

const C = {
  bg: "FAFBFD",
  ink: "212529",
  muted: "5A6068",
  blue: "2463EB",
  green: "16A34A",
  orange: "EA580C",
  red: "DC2626",
  line: "C4CCD7",
  card: "FFFFFF",
  blueFill: "EFF6FF",
  greenFill: "F0FDF4",
  orangeFill: "FFF7ED",
  redFill: "FEF2F2",
  header: "F1F5F9",
};
const font = "Malgun Gothic";

function bg(slide) {
  slide.background = { color: C.bg };
}

function txt(slide, text, x, y, w, h, size = 20, bold = false, color = C.ink, align = "left") {
  slide.addText(text, {
    x, y, w, h,
    fontFace: font,
    fontSize: size,
    bold,
    color,
    align,
    valign: "mid",
    margin: 0.02,
    fit: "shrink",
  });
}

function box(slide, text, x, y, w, h, fill = C.card, line = C.line, size = 16, bold = false, color = C.ink) {
  slide.addShape(pptx.ShapeType.roundRect, {
    x, y, w, h,
    rectRadius: 0.06,
    fill: { color: fill },
    line: { color: line, width: 1 },
  });
  slide.addText(text, {
    x: x + 0.05,
    y: y + 0.04,
    w: w - 0.1,
    h: h - 0.08,
    fontFace: font,
    fontSize: size,
    bold,
    color,
    align: "center",
    valign: "mid",
    fit: "shrink",
    margin: 0.02,
  });
}

function arrow(slide, x1, y1, x2, y2, color = C.blue) {
  slide.addShape(pptx.ShapeType.line, {
    x: x1,
    y: y1,
    w: x2 - x1,
    h: y2 - y1,
    line: { color, width: 2, beginArrowType: "none", endArrowType: "triangle" },
  });
}

function bullets(slide, items, x, y, w, h, size = 17) {
  const runs = items.map((text) => ({ text, options: { bullet: { type: "bullet" } } }));
  slide.addText(runs, {
    x, y, w, h,
    fontFace: font,
    fontSize: size,
    color: C.ink,
    fit: "shrink",
    valign: "top",
    margin: 0.03,
    paraSpaceAfterPt: 8,
  });
}

let slide = pptx.addSlide();
bg(slide);
txt(slide, "C02 AXI4-Stream Width Standardization", 0.7, 0.65, 11.8, 0.5, 27, true, C.blue);
txt(slide, "Phase A 보완 결과 v001", 0.7, 1.18, 11.8, 0.7, 36, true, C.ink);
box(slide, "32 / 64 / 128-bit\nfull TKEEP · full TSTRB", 0.9, 2.25, 3.4, 1.25, C.blueFill, C.blue, 21, true, C.blue);
box(slide, "GPX READ timing\n변경 없음", 4.8, 2.25, 3.0, 1.25, C.greenFill, C.green, 21, true, C.green);
box(slide, "Payload latency\n추가 없음", 8.3, 2.25, 3.0, 1.25, C.orangeFill, C.orange, 21, true, C.orange);
txt(slide, "문서 시간: 2026-05-01 00:21:08 KST", 0.9, 6.45, 8.0, 0.35, 14, false, C.muted);
txt(slide, "기준: Datasheet 계약 유지 + downstream AXIS 표준화", 0.9, 6.85, 10.5, 0.35, 14, false, C.muted);

slide = pptx.addSlide();
bg(slide);
txt(slide, "데이터 흐름", 0.6, 0.45, 12, 0.45, 28, true, C.ink);
const labels = [
  ["cell_builder\nTDATA generic", 0.6],
  ["cell_pipe\n폭 guard", 2.7],
  ["face_assembler\nrow pack", 4.8],
  ["output_stage\nFIFO + header", 6.9],
  ["header_inserter\nTKEEP/TSTRB", 9.0],
  ["tdc_gpx_top\nfinal AXIS", 11.1],
];
labels.forEach(([label, x]) => box(slide, label, x, 2.0, 1.65, 1.05, C.card, C.line, 13, true));
for (let i = 0; i < labels.length - 1; i += 1) {
  arrow(slide, labels[i][1] + 1.65, 2.52, labels[i + 1][1], 2.52);
}
box(slide, "Phase A 변경점", 0.8, 4.2, 2.2, 0.55, C.blueFill, C.blue, 18, true, C.blue);
bullets(slide, [
  "top/output_stage/header_inserter에 최종 TKEEP/TSTRB 포트 추가",
  "32/64/128 이외 폭은 elaboration failure로 차단",
  "accepted beat 기준 keep/strb는 항상 all-ones",
], 0.9, 4.9, 11.8, 1.1, 18);

slide = pptx.addSlide();
bg(slide);
txt(slide, "폭별 구조 비교", 0.6, 0.45, 12, 0.45, 28, true, C.ink);
const cols = ["TDATA", "TKEEP/TSTRB", "Header prefix", "Max cell beats", "상태"];
const xs = [0.9, 2.8, 5.2, 7.7, 10.2];
const ws = [1.5, 1.9, 2.0, 2.0, 1.6];
cols.forEach((col, i) => box(slide, col, xs[i], 1.4, ws[i], 0.55, C.header, C.line, 15, true));
[
  ["32", "4", "12 beats", "8", "지원"],
  ["64", "8", "6 beats", "4", "지원"],
  ["128", "16", "3 beats", "2", "신규"],
].forEach((row, r) => {
  const y = 2.1 + r * 0.8;
  row.forEach((cell, i) => box(slide, cell, xs[i], y, ws[i], 0.55, i === 4 ? C.greenFill : C.card, C.line, 16, cell === "신규", i === 4 ? C.green : C.ink));
});
txt(slide, "동일 48B header prefix: 폭이 커질수록 beat 수만 감소", 0.95, 5.2, 8.5, 0.35, 20, true, C.blue);
txt(slide, "Partial TKEEP는 Phase A 제외. 256-bit 이상은 Phase C 후보.", 0.95, 5.65, 10.5, 0.35, 17, false, C.muted);

slide = pptx.addSlide();
bg(slide);
txt(slide, "Timing / Latency / Throughput / II", 0.6, 0.45, 12, 0.45, 28, true, C.ink);
box(slide, "face_start", 0.9, 1.7, 1.5, 0.55, C.blueFill, C.blue, 16, true, C.blue);
arrow(slide, 2.4, 1.98, 3.0, 1.98);
box(slide, "header ROM\nbuild", 3.0, 1.55, 1.7, 0.85, C.card, C.line, 15, true);
arrow(slide, 4.7, 1.98, 5.4, 1.98);
box(slide, "prefix\n12/6/3 beats", 5.4, 1.55, 2.0, 0.85, C.orangeFill, C.orange, 15, true, C.orange);
arrow(slide, 7.4, 1.98, 8.0, 1.98);
box(slide, "data beats\nfull keep", 8.0, 1.55, 1.8, 0.85, C.card, C.line, 15, true);
arrow(slide, 9.8, 1.98, 10.4, 1.98);
box(slide, "TLAST\nframe_done", 10.4, 1.55, 1.7, 0.85, C.greenFill, C.green, 15, true, C.green);
bullets(slide, [
  "Latency: TKEEP/TSTRB는 registered valid에서 파생, payload 지연 추가 없음",
  "Throughput: downstream ready 유지 시 1 beat/clk",
  "Pipeline: 기존 cell -> face -> FIFO -> header 구조 유지",
  "II: beat 단위 II=1 유지, byte payload 기준 점유 beat는 128-bit에서 감소",
], 1.0, 3.2, 11.2, 2.0, 18);

slide = pptx.addSlide();
bg(slide);
txt(slide, "검증 상태와 다음 Phase", 0.6, 0.45, 12, 0.45, 28, true, C.ink);
box(slide, "PASS 기준 추가", 0.9, 1.35, 2.6, 0.55, C.greenFill, C.green, 17, true, C.green);
bullets(slide, [
  "tb_tdc_gpx_header_inserter_widths: 32/64/128 동시 확인",
  "기존 downstream/output/top/full TB에 keep/strb all-one assert 추가",
  "git diff --check: 공백 오류 없음, CRLF warning만 존재",
], 1.0, 2.05, 11.4, 1.35, 18);
box(slide, "미실행", 0.9, 4.0, 2.0, 0.55, C.redFill, C.red, 17, true, C.red);
bullets(slide, [
  "현재 PATH에 xvhdl/vivado/ghdl 없음: xsim은 로컬에서 실행 필요",
  "다음 Phase: raw/event stream 폭 상수화 및 Vivado sim set 반영",
], 1.0, 4.7, 11.3, 0.9, 18);

pptx.writeFile({
  fileName: "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C02_Chip_Acquisition/C02_Chip_Acquisition_260501002108_AXI4_Stream_PhaseA_Fix_Result_v001.pptx",
});
