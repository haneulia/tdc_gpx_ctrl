const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C03_Cell_Pipe/C03_Cell_Pipe_260501025221_C04_Handoff_v001.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C03 to C04 Handoff";
pptx.title = "C03 to C04 Handoff v001";
pptx.lang = "ko-KR";
pptx.theme = { headFontFace: "Malgun Gothic", bodyFontFace: "Malgun Gothic", lang: "ko-KR" };

const font = "Malgun Gothic";
const C = { bg: "FAFBFD", ink: "172033", muted: "64748B", line: "CBD5E1", card: "FFFFFF", blue: "2563EB", blueFill: "EFF6FF", green: "16A34A", greenFill: "ECFDF5", orange: "EA580C", orangeFill: "FFF7ED", red: "DC2626", redFill: "FEF2F2", slate: "F8FAFC" };

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
  title(s, "C03 종료 판단", "C03 P1/P2는 닫혔고, C04는 cell metadata 계약을 수락해야 한다");
  table(s, [
    ["항목", "상태", "근거"],
    ["Hit[16] 보존", "Close", "metadata [6:0] = hit_msb_vec"],
    ["stale-ready", "Close", "C03 input skid boundary"],
    ["per-slope abort", "Close", "slope별 clear/drop"],
    ["검증", "PASS", "smoke + C03 fix regression"]
  ], 0.85, 1.45, [2.2, 1.4, 8.1], 0.52, 10.2);
  box(s, "C03 추가 검토 없음\n잔여는 C04 수락 계약", 3.5, 4.6, 6.2, 0.9, { fill: C.greenFill, line: C.green, bold: true, size: 16 });
  foot(s, "근거: fff793d / xsim_c03_cell_pipe_fix.log:36");
}

{
  const s = pptx.addSlide();
  title(s, "C03 -> C04 데이터 계약", "Hit[16:0] 복원 의미를 C04 최종 output까지 보존한다");
  box(s, "C03 cell\nHit[15:0]", 0.8, 1.7, 2.0, 0.85, { fill: C.greenFill, line: C.green, bold: true });
  box(s, "metadata [6:0]\nHit[16]", 0.8, 3.0, 2.0, 0.85, { fill: C.orangeFill, line: C.orange, bold: true });
  arrow(s, 2.85, 2.1, 4.0, 2.1, C.green);
  arrow(s, 2.85, 3.43, 4.0, 3.43, C.orange);
  box(s, "C04\nface_assembler", 4.05, 2.25, 2.2, 1.0, { fill: C.blueFill, line: C.blue, bold: true });
  arrow(s, 6.3, 2.75, 7.2, 2.75, C.blue);
  box(s, "FIFO + header", 7.25, 2.25, 2.0, 1.0, { fill: C.slate, line: C.line, bold: true });
  arrow(s, 9.3, 2.75, 10.2, 2.75, C.blue);
  box(s, "VDMA stream\nfull hit 복원", 10.25, 2.25, 2.1, 1.0, { fill: C.card, line: C.line, bold: true });
  table(s, [
    ["계약", "내용"],
    ["C03-C04-01", "metadata [6:0]은 hit_msb_vec[6:0]"],
    ["C03-C04-03", "full hit = Hit[16] & Hit[15:0]"],
    ["C03-C04-06", "tuser(0) faulted는 chip slice tlast와 동행"]
  ], 1.0, 4.55, [2.4, 8.8], 0.45, 9.6);
  foot(s, "근거: Doc/TDC-GPX-Datasheet.pdf page 20, 27 / C03 Fix Result v001");
}

{
  const s = pptx.addSlide();
  title(s, "Timing 인계", "C04는 face assembly, FIFO, header 단계의 latency/II를 이어서 측정한다");
  box(s, "C2 event skid", 0.7, 2.2, 1.7, 0.7, { fill: C.slate, bold: true });
  arrow(s, 2.45, 2.55, 3.0, 2.55);
  box(s, "C3 input skid\n+1 clk", 3.05, 2.1, 1.9, 0.9, { fill: C.blueFill, line: C.blue, bold: true });
  arrow(s, 5.0, 2.55, 5.55, 2.55);
  box(s, "cell_builder\nserializer", 5.6, 2.1, 2.0, 0.9, { fill: C.greenFill, line: C.green, bold: true });
  arrow(s, 7.65, 2.55, 8.2, 2.55);
  box(s, "C04 input\nper-chip stream", 8.25, 2.1, 2.1, 0.9, { fill: C.orangeFill, line: C.orange, bold: true });
  table(s, [
    ["Metric", "C04 확인"],
    ["Latency", "cell accepted -> row first beat -> header data -> final tlast"],
    ["Throughput", "cell beats + header beats / output clock"],
    ["II", "연속 row/shot에서 row start 간격"],
    ["Pipeline", "face_assembler FSM, FIFO, header FSM register boundary"]
  ], 0.95, 4.15, [2.0, 9.25], 0.44, 9.5);
  foot(s, "C04 진입 기준: I-Mode single, 32/64/128-bit output width");
}

pptx.writeFile({ fileName: out });
