const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C04_Output_Stage/C04_Output_Stage_260501025221_Plan_v001.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C04 Output Stage Plan";
pptx.title = "C04 Output Stage Plan v001";
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
  title(s, "C04 Output Stage 분석 계획", "face_assembler -> FIFO -> header_inserter -> VDMA output");
  box(s, "C03 rise\ncell x4", 0.65, 1.95, 1.35, 0.75, { fill: C.greenFill, line: C.green, bold: true });
  box(s, "C03 fall\ncell x4", 0.65, 3.35, 1.35, 0.75, { fill: C.orangeFill, line: C.orange, bold: true });
  arrow(s, 2.05, 2.32, 2.65, 2.32, C.green);
  arrow(s, 2.05, 3.72, 2.65, 3.72, C.orange);
  box(s, "face_assembler\nrise/fall", 2.7, 2.45, 2.05, 1.0, { fill: C.blueFill, line: C.blue, bold: true });
  arrow(s, 4.8, 2.95, 5.45, 2.95, C.blue);
  box(s, "xpm_fifo_axis\n2 slopes", 5.5, 2.45, 2.05, 1.0, { fill: C.cyanFill, line: C.cyan, bold: true });
  arrow(s, 7.6, 2.95, 8.25, 2.95, C.cyan);
  box(s, "header_inserter\n48B prefix", 8.3, 2.45, 2.15, 1.0, { fill: C.slate, line: C.line, bold: true });
  arrow(s, 10.5, 2.95, 11.1, 2.95);
  box(s, "VDMA\nstream", 11.15, 2.45, 1.45, 1.0, { fill: C.card, line: C.line, bold: true });
  table(s, [
    ["목표", "핵심"],
    ["metadata", "C03 hit_msb_vec [6:0] 보존"],
    ["timing", "latency / throughput / pipeline / II 산출"],
    ["protocol", "tlast / tuser / tkeep / tstrb 정합"]
  ], 1.0, 5.05, [2.0, 9.1], 0.42, 9.6);
  foot(s, "근거: tdc_gpx_output_stage.vhd:8-17, 238-481");
}

{
  const s = pptx.addSlide();
  title(s, "Sub-cluster 분해", "C04는 5개 분석 단위로 순차 확장한다");
  table(s, [
    ["단계", "대상", "분석 핵심"],
    ["C04-A", "face_assembler input FIFO", "C03 cell stream 수락, tuser/faulted 동행"],
    ["C04-B", "face_assembler scheduling", "active mask, strict order, blank-fill, row_done"],
    ["C04-C", "output FIFO", "ready/valid 경계, abort flush"],
    ["C04-D", "header_inserter", "48B header, SOF, EOL, frame_done"],
    ["C04-E", "final VDMA", "32/64/128 width별 HSIZE/VSIZE/STRIDE"]
  ], 0.75, 1.4, [1.5, 3.2, 7.4], 0.55, 9.5);
  foot(s, "각 단계마다 Datasheet와 C03 인계 계약을 우선 기준으로 사용");
}

{
  const s = pptx.addSlide();
  title(s, "검증 Matrix 초안", "분석 뒤 RTL/TB 보완으로 닫을 기준");
  table(s, [
    ["ID", "검증 항목", "기대 결과"],
    ["VB-C04-01", "metadata [6:0] pass-through", "final output에서 Hit[16] vector 동일"],
    ["VB-C04-02", "blank-fill metadata", "error_fill=1, chip_id 정상, hit_msb_vec=0"],
    ["VB-C04-03", "32/64/128 row size", "cell/header beat count와 tlast 위치 일치"],
    ["VB-C04-05", "faulted tuser", "row_done_faulted/frame_done_faulted 연결"],
    ["VB-C04-10", "VDMA protocol", "SOF, EOL, tkeep/tstrb, frame_done 정합"]
  ], 0.7, 1.45, [1.55, 4.05, 6.0], 0.52, 9.2);
  foot(s, "C04 분석 후 VB 항목은 PASS/FAIL 근거와 로그 line으로 추적");
}

{
  const s = pptx.addSlide();
  title(s, "Timing Block 계획", "C04에서 측정할 latency, throughput, pipeline, II 경계");
  box(s, "T0\ncell accepted", 0.7, 2.25, 1.55, 0.75, { fill: C.greenFill, line: C.green, bold: true });
  arrow(s, 2.3, 2.62, 2.9, 2.62);
  box(s, "T1\ninput FIFO", 2.95, 2.25, 1.55, 0.75, { fill: C.blueFill, line: C.blue, bold: true });
  arrow(s, 4.55, 2.62, 5.15, 2.62);
  box(s, "T2\nscan/resolve", 5.2, 2.25, 1.65, 0.75, { fill: C.cyanFill, line: C.cyan, bold: true });
  arrow(s, 6.9, 2.62, 7.5, 2.62);
  box(s, "T3\nrow forward", 7.55, 2.25, 1.65, 0.75, { fill: C.orangeFill, line: C.orange, bold: true });
  arrow(s, 9.25, 2.62, 9.85, 2.62);
  box(s, "T4\nheader + VDMA", 9.9, 2.25, 2.05, 0.75, { fill: C.slate, line: C.line, bold: true });
  table(s, [
    ["Metric", "측정 경계"],
    ["Latency", "T0 -> first row beat -> first data after header -> final tlast"],
    ["Throughput", "cell beats + header beats / output clock"],
    ["II", "연속 shot/row 입력에서 row start 간격"],
    ["Pipeline", "FSM/register/FIFO boundary별 분리"]
  ], 1.0, 4.25, [2.0, 9.1], 0.44, 9.6);
  foot(s, "C04 분석 산출물에는 timing diagram 또는 timing block diagram을 필수 포함");
}

pptx.writeFile({ fileName: out });
