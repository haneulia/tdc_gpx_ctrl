const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/pptxgenjs");

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C02 AXI4-Stream Phase C Contract Close";
pptx.title = "C02 AXI4-Stream Phase C";
pptx.lang = "ko-KR";
pptx.theme = {
  headFontFace: "Malgun Gothic",
  bodyFontFace: "Malgun Gothic",
  lang: "ko-KR",
};

const C = {
  bg: "FAFBFD",
  ink: "1F2937",
  muted: "64748B",
  blue: "2563EB",
  blueFill: "EFF6FF",
  green: "16A34A",
  greenFill: "ECFDF5",
  orange: "EA580C",
  orangeFill: "FFF7ED",
  red: "DC2626",
  redFill: "FEF2F2",
  gray: "CBD5E1",
  card: "FFFFFF",
  header: "F1F5F9",
};
const font = "Malgun Gothic";

function bg(slide) {
  slide.background = { color: C.bg };
}

function txt(slide, text, x, y, w, h, size = 18, bold = false, color = C.ink, align = "left") {
  slide.addText(text, {
    x, y, w, h,
    fontFace: font,
    fontSize: size,
    bold,
    color,
    align,
    valign: "mid",
    fit: "shrink",
    margin: 0.02,
  });
}

function box(slide, text, x, y, w, h, fill = C.card, line = C.gray, size = 15, bold = false, color = C.ink) {
  slide.addShape(pptx.ShapeType.roundRect, {
    x, y, w, h,
    rectRadius: 0.05,
    fill: { color: fill },
    line: { color: line, width: 1 },
  });
  slide.addText(text, {
    x: x + 0.06,
    y: y + 0.04,
    w: w - 0.12,
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
  slide.addText(items.map((text) => ({ text, options: { bullet: { type: "bullet" } } })), {
    x, y, w, h,
    fontFace: font,
    fontSize: size,
    color: C.ink,
    fit: "shrink",
    valign: "top",
    margin: 0.03,
    paraSpaceAfterPt: 7,
  });
}

function table(slide, rows, x, y, widths, rowH = 0.55) {
  rows.forEach((row, r) => {
    let cx = x;
    row.forEach((cell, c) => {
      const isHeader = r === 0;
      box(
        slide,
        cell,
        cx,
        y + r * rowH,
        widths[c],
        rowH - 0.04,
        isHeader ? C.header : C.card,
        C.gray,
        isHeader ? 13 : 12,
        isHeader,
        C.ink,
      );
      cx += widths[c] + 0.04;
    });
  });
}

let slide = pptx.addSlide();
bg(slide);
txt(slide, "C02 AXI4-Stream Phase C", 0.7, 0.62, 11.8, 0.5, 28, true, C.blue);
txt(slide, "32/64/128-bit 계약 종료", 0.7, 1.15, 11.8, 0.75, 36, true, C.ink);
box(slide, "256-bit\n미반영", 0.9, 2.35, 2.55, 1.1, C.redFill, C.red, 22, true, C.red);
box(slide, "지원 폭\n32 / 64 / 128", 4.0, 2.35, 3.1, 1.1, C.blueFill, C.blue, 22, true, C.blue);
box(slide, "Full TKEEP\nFull TSTRB 유지", 7.65, 2.35, 3.1, 1.1, C.greenFill, C.green, 22, true, C.green);
txt(slide, "문서 시각: 2026-05-01 01:02:57 KST", 0.9, 6.35, 7.5, 0.32, 14, false, C.muted);
txt(slide, "기준: Doc/TDC-GPX-Datasheet.pdf + Phase A/B 구현 계약", 0.9, 6.73, 10.5, 0.32, 14, false, C.muted);

slide = pptx.addSlide();
bg(slide);
txt(slide, "계약 구조", 0.6, 0.45, 12, 0.45, 27, true, C.ink);
box(slide, "tdc_gpx_pkg\nfn_output_width_supported()", 0.7, 1.45, 3.0, 0.95, C.blueFill, C.blue, 16, true, C.blue);
arrow(slide, 3.7, 1.93, 4.45, 1.93);
box(slide, "공식 지원\n32 / 64 / 128", 4.45, 1.45, 2.35, 0.95, C.greenFill, C.green, 16, true, C.green);
arrow(slide, 6.8, 1.93, 7.55, 1.93);
box(slide, "모듈 assert\n동일 함수 사용", 7.55, 1.45, 2.4, 0.95, C.card, C.gray, 16, true);
arrow(slide, 9.95, 1.93, 10.7, 1.93);
box(slide, "비지원 폭\n차단", 10.7, 1.45, 1.7, 0.95, C.redFill, C.red, 16, true, C.red);
table(slide, [
  ["적용 모듈", "역할"],
  ["top / csr_pipeline / face_seq", "시스템 계약과 geometry 계산"],
  ["cell_builder / cell_pipe", "cell serialization 계약"],
  ["face_assembler / output_stage", "row/frame output 계약"],
  ["header_inserter", "header + final AXIS 계약"],
], 0.8, 3.25, [4.0, 7.5], 0.58);

slide = pptx.addSlide();
bg(slide);
txt(slide, "Timing Block", 0.6, 0.45, 12, 0.45, 27, true, C.ink);
const nodes = [
  ["T0\nGPX READ 완료", 0.75],
  ["T1\nraw/event decode", 2.8],
  ["T2\ncell/face", 4.95],
  ["T3\nheader", 6.95],
  ["T4\nfinal AXIS", 8.85],
];
nodes.forEach(([label, x]) => box(slide, label, x, 2.0, 1.5, 0.85, C.card, C.gray, 15, true));
for (let i = 0; i < nodes.length - 1; i += 1) arrow(slide, nodes[i][1] + 1.5, 2.43, nodes[i + 1][1], 2.43);
box(slide, "width guard\n계산/elaboration 계약", 4.65, 4.0, 3.2, 0.8, C.orangeFill, C.orange, 16, true, C.orange);
txt(slide, "guard는 datapath FF/FIFO/state를 추가하지 않으므로 지연과 II를 바꾸지 않는다.", 1.05, 5.45, 11.2, 0.4, 18, true, C.blue);
bullets(slide, [
  "Latency: 변화 없음",
  "Throughput: 32/64/128 full-keep 기준 유지",
  "Pipeline: C01/C02/C03/C04 구조 유지",
  "II: AXIS beat 기준 II=1 조건 유지",
], 1.1, 5.9, 10.8, 1.1, 16);

slide = pptx.addSlide();
bg(slide);
txt(slide, "검증 결과", 0.6, 0.45, 12, 0.45, 27, true, C.ink);
table(slide, [
  ["검증", "결과", "핵심 확인"],
  ["compile", "PASS", "top_int까지 분석 완료"],
  ["header widths", "PASS", "32/64/128 지원, 256 미지원 정책 확인"],
  ["output stage", "PASS", "Scenario 1/2 PASS"],
  ["top_int 64-bit", "PASS", "rising/falling beats=60, tlast=2"],
], 0.6, 1.25, [3.2, 1.4, 7.5], 0.62);
box(slide, "Phase C 판단\nAXI4-Stream 폭 표준화는 32/64/128-bit 범위에서 close", 1.05, 5.4, 11.2, 0.85, C.greenFill, C.green, 19, true, C.green);

pptx.writeFile({
  fileName: "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C02_Chip_Acquisition/C02_Chip_Acquisition_260501010257_AXI4_Stream_PhaseC_Contract_Close_v001.pptx",
});
