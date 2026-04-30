const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C03_Cell_Pipe/C03_Cell_Pipe_260501030046_C04_Handoff_v002.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C03 to C04 Handoff v002";
pptx.title = "C03 to C04 Handoff v002";
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
  title(s, "C03 -> C04 인계 v002", "이번 generation: C03 내부 Hit[16] 보존, C04 최종 VDMA에서는 폐기");
  box(s, "C03 내부\nHit[16] metadata", 0.8, 2.15, 2.2, 0.9, { fill: C.orangeFill, line: C.orange, bold: true });
  arrow(s, 3.05, 2.6, 3.8, 2.6, C.orange);
  box(s, "C04\nmetadata sanitize", 3.85, 2.15, 2.4, 0.9, { fill: C.blueFill, line: C.blue, bold: true });
  arrow(s, 6.3, 2.6, 7.05, 2.6, C.blue);
  box(s, "VDMA stream\nHit[16] 없음", 7.1, 2.15, 2.3, 0.9, { fill: C.greenFill, line: C.green, bold: true });
  arrow(s, 9.45, 2.6, 10.2, 2.6, C.green);
  box(s, "다음 generation\n17-bit 복원 검토", 10.25, 2.15, 2.0, 0.9, { fill: C.slate, line: C.line, bold: true });
  table(s, [
    ["항목", "v002 결정"],
    ["metadata [6:0]", "C04 final VDMA 전 0으로 clear"],
    ["Hit slot", "Hit[15:0]만 최종 유효"],
    ["Full hit", "이번 generation 범위 제외"]
  ], 1.0, 4.4, [2.1, 9.1], 0.46, 9.7);
  foot(s, "근거: 사용자 결정 2026-05-01 / Datasheet page 20, 27");
}

{
  const s = pptx.addSlide();
  title(s, "권장 처리 위치", "metadata beat를 아는 face_assembler에서 clear하는 것이 가장 단순하다");
  box(s, "C03 cell stream", 0.75, 2.25, 1.8, 0.75, { fill: C.slate, bold: true });
  arrow(s, 2.6, 2.62, 3.2, 2.62);
  box(s, "face_assembler\nmetadata beat 식별", 3.25, 2.05, 2.5, 1.15, { fill: C.blueFill, line: C.blue, bold: true });
  arrow(s, 5.8, 2.62, 6.4, 2.62, C.blue);
  box(s, "[6:0] clear\nbubble 없음", 6.45, 2.05, 2.1, 1.15, { fill: C.greenFill, line: C.green, bold: true });
  arrow(s, 8.6, 2.62, 9.2, 2.62, C.green);
  box(s, "FIFO + header\npass-through", 9.25, 2.05, 2.0, 1.15, { fill: C.card, line: C.line, bold: true });
  table(s, [
    ["위치", "판단"],
    ["face_assembler", "권장: s_fwd_beat_r = s_rt_last_beat_r 시점에 clear"],
    ["header 이후", "비권장: header/data가 섞여 metadata 식별 복잡"],
    ["blank-fill", "이미 [6:0]=0, 계약만 명시"]
  ], 0.9, 4.35, [2.2, 9.0], 0.46, 9.5);
  foot(s, "C04 Plan v002: C04-C metadata sanitize 항목 신설");
}

pptx.writeFile({ fileName: out });
