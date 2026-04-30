const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C04_Output_Stage/C04_Output_Stage_260501030046_Plan_v002.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C04 Output Stage Plan v002";
pptx.title = "C04 Output Stage Plan v002";
pptx.lang = "ko-KR";
pptx.theme = { headFontFace: "Malgun Gothic", bodyFontFace: "Malgun Gothic", lang: "ko-KR" };

const font = "Malgun Gothic";
const C = { bg: "FAFBFD", ink: "172033", muted: "64748B", line: "CBD5E1", card: "FFFFFF", blue: "2563EB", blueFill: "EFF6FF", green: "16A34A", greenFill: "ECFDF5", orange: "EA580C", orangeFill: "FFF7ED", red: "DC2626", redFill: "FEF2F2", cyan: "0891B2", cyanFill: "ECFEFF", slate: "F8FAFC" };

function bg(slide) { slide.background = { color: C.bg }; }
function text(slide, value, x, y, w, h, opt = {}) {
  slide.addText(value, { x, y, w, h, fontFace: font, fontSize: opt.size || 12, bold: opt.bold || false, color: opt.color || C.ink, align: opt.align || "left", valign: "mid", fit: "shrink", margin: opt.margin === undefined ? 0.04 : opt.margin, breakLine: false });
}
function title(slide, main, sub) {
  bg(slide);
  text(slide, main, 0.55, 0.28, 12.2, 0.45, { size: 21.5, bold: true });
  text(slide, sub, 0.58, 0.74, 12.0, 0.3, { size: 9.2, color: C.muted });
  slide.addShape(pptx.ShapeType.line, { x: 0.55, y: 1.08, w: 12.2, h: 0, line: { color: C.line, width: 0.8 } });
}
function foot(slide, value) { text(slide, value, 0.65, 6.96, 12, 0.25, { size: 8.2, color: C.muted, align: "center" }); }
function box(slide, value, x, y, w, h, opt = {}) {
  slide.addShape(pptx.ShapeType.roundRect, { x, y, w, h, rectRadius: 0.05, fill: { color: opt.fill || C.card }, line: { color: opt.line || C.line, width: 1 } });
  text(slide, value, x + 0.08, y + 0.06, w - 0.16, h - 0.12, { size: opt.size || 11, bold: opt.bold || false, color: opt.color || C.ink, align: opt.align || "center" });
}
function arrow(slide, x1, y1, x2, y2, color = C.muted) {
  slide.addShape(pptx.ShapeType.line, { x: x1, y: y1, w: x2 - x1, h: y2 - y1, line: { color, width: 1.5, beginArrowType: "none", endArrowType: "triangle" } });
}
function table(slide, rows, x, y, widths, rowH, size) {
  rows.forEach((row, r) => {
    let curX = x;
    row.forEach((cell, c) => {
      slide.addShape(pptx.ShapeType.rect, { x: curX, y: y + r * rowH, w: widths[c], h: rowH, fill: { color: r === 0 ? "E2E8F0" : C.card }, line: { color: C.line, width: 0.7 } });
      text(slide, cell, curX + 0.05, y + r * rowH + 0.04, widths[c] - 0.1, rowH - 0.08, { size, bold: r === 0, align: c === 0 ? "center" : "left" });
      curX += widths[c];
    });
  });
}

{
  const s = pptx.addSlide();
  title(s, "C04 Plan v002", "최종 VDMA stream에서는 Hit[16]을 버리는 계획으로 변경");
  box(s, "C03 input\nmetadata [6:0]\nHit[16]", 0.8, 1.8, 2.0, 1.0, { fill: C.orangeFill, line: C.orange, bold: true });
  arrow(s, 2.85, 2.3, 3.55, 2.3, C.orange);
  box(s, "C04-C\nsanitize\n[6:0]=0", 3.6, 1.8, 2.0, 1.0, { fill: C.blueFill, line: C.blue, bold: true });
  arrow(s, 5.65, 2.3, 6.35, 2.3, C.blue);
  box(s, "VDMA output\nHit[15:0] only", 6.4, 1.8, 2.2, 1.0, { fill: C.greenFill, line: C.green, bold: true });
  arrow(s, 8.65, 2.3, 9.35, 2.3, C.green);
  box(s, "Next gen\n17-bit 검토", 9.4, 1.8, 2.1, 1.0, { fill: C.slate, line: C.line, bold: true });
  table(s, [
    ["변경", "내용"],
    ["Q-C04-01", "pass-through 확인 -> sanitize 확인"],
    ["VB-C04-01", "final metadata [6:0]=0 검증"],
    ["범위", "full 17-bit output은 다음 generation"]
  ], 1.0, 4.25, [2.2, 9.0], 0.48, 9.7);
  foot(s, "절대 기준은 Datasheet, 운용 결정은 이번 generation final stream 정책");
}

{
  const s = pptx.addSlide();
  title(s, "Sub-cluster v002", "metadata sanitize 단계를 C04-C로 명시");
  table(s, [
    ["단계", "대상", "분석 핵심"],
    ["C04-A", "face_assembler input FIFO", "C03 cell stream 수락, faulted tuser"],
    ["C04-B", "face_assembler scheduling", "active mask, strict order, blank-fill"],
    ["C04-C", "metadata sanitize", "real metadata beat [6:0] clear"],
    ["C04-D", "output FIFO", "ready/valid 경계, abort flush"],
    ["C04-E", "header_inserter", "48B header, SOF, EOL, frame_done"],
    ["C04-F", "final VDMA", "32/64/128 width별 protocol, Hit[16] 폐기 확인"]
  ], 0.7, 1.35, [1.55, 3.45, 6.4], 0.48, 9.1);
  foot(s, "C04-C가 v002의 핵심 추가 분석 단위");
}

{
  const s = pptx.addSlide();
  title(s, "Timing Block v002", "sanitize는 metadata beat cycle 안에서 처리하고 bubble을 만들지 않는다");
  box(s, "T0\ncell accepted", 0.55, 2.2, 1.45, 0.72, { fill: C.greenFill, line: C.green, bold: true });
  arrow(s, 2.05, 2.56, 2.55, 2.56);
  box(s, "T1\ninput FIFO", 2.6, 2.2, 1.45, 0.72, { fill: C.blueFill, line: C.blue, bold: true });
  arrow(s, 4.1, 2.56, 4.6, 2.56);
  box(s, "T2\nscan/resolve", 4.65, 2.2, 1.55, 0.72, { fill: C.cyanFill, line: C.cyan, bold: true });
  arrow(s, 6.25, 2.56, 6.75, 2.56);
  box(s, "T3\nforward", 6.8, 2.2, 1.35, 0.72, { fill: C.orangeFill, line: C.orange, bold: true });
  arrow(s, 8.2, 2.56, 8.7, 2.56);
  box(s, "S\n[6:0] clear", 8.75, 2.2, 1.35, 0.72, { fill: C.redFill, line: C.red, bold: true });
  arrow(s, 10.15, 2.56, 10.65, 2.56);
  box(s, "T4\nVDMA", 10.7, 2.2, 1.35, 0.72, { fill: C.slate, line: C.line, bold: true });
  table(s, [
    ["Metric", "v002 확인"],
    ["Latency", "sanitize가 추가 cycle을 만들지 않는지"],
    ["Throughput", "cell/header beat count 변경 없음"],
    ["II", "row forward II=1 유지 여부"],
    ["Pipeline", "face_assembler output register 내 clear 가능성"]
  ], 1.0, 4.25, [2.0, 9.1], 0.44, 9.6);
  foot(s, "검증: C03 input [6:0]=1 패턴 -> final VDMA [6:0]=0");
}

pptx.writeFile({ fileName: out });
