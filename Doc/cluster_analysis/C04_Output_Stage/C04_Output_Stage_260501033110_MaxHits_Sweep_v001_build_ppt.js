const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C04_Output_Stage/C04_Output_Stage_260501033110_MaxHits_Sweep_v001.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C04 max_hits_cfg Sweep v001";
pptx.title = "C04 max_hits_cfg Sweep v001";
pptx.lang = "ko-KR";
pptx.theme = { headFontFace: "Malgun Gothic", bodyFontFace: "Malgun Gothic", lang: "ko-KR" };

const font = "Malgun Gothic";
const C = {
  bg: "FAFBFD", ink: "172033", muted: "64748B", line: "CBD5E1", card: "FFFFFF",
  blue: "2563EB", blueFill: "EFF6FF", green: "16A34A", greenFill: "ECFDF5",
  orange: "EA580C", orangeFill: "FFF7ED", cyan: "0891B2", cyanFill: "ECFEFF",
  slate: "F8FAFC"
};

function bg(slide) { slide.background = { color: C.bg }; }
function text(slide, value, x, y, w, h, opt = {}) {
  slide.addText(value, {
    x, y, w, h, fontFace: font, fontSize: opt.size || 11, bold: opt.bold || false,
    color: opt.color || C.ink, align: opt.align || "left", valign: "mid",
    fit: "shrink", margin: opt.margin === undefined ? 0.04 : opt.margin
  });
}
function title(slide, main, sub) {
  bg(slide);
  text(slide, main, 0.55, 0.28, 12.2, 0.45, { size: 21, bold: true });
  text(slide, sub, 0.58, 0.74, 12.0, 0.3, { size: 9.2, color: C.muted });
  slide.addShape(pptx.ShapeType.line, { x: 0.55, y: 1.08, w: 12.2, h: 0, line: { color: C.line, width: 0.8 } });
}
function footer(slide, value) { text(slide, value, 0.65, 6.96, 12, 0.25, { size: 8, color: C.muted, align: "center" }); }
function box(slide, value, x, y, w, h, opt = {}) {
  slide.addShape(pptx.ShapeType.rect, { x, y, w, h, fill: { color: opt.fill || C.card }, line: { color: opt.line || C.line, width: 1 } });
  text(slide, value, x + 0.08, y + 0.06, w - 0.16, h - 0.12, { size: opt.size || 11, bold: opt.bold || false, align: opt.align || "center" });
}
function arrow(slide, x1, y1, x2, y2, color = C.muted) {
  slide.addShape(pptx.ShapeType.line, { x: x1, y: y1, w: x2 - x1, h: y2 - y1, line: { color, width: 1.5, endArrowType: "triangle" } });
}
function table(slide, rows, x, y, widths, rowH, size = 9.3) {
  rows.forEach((row, r) => {
    let curX = x;
    row.forEach((cell, c) => {
      slide.addShape(pptx.ShapeType.rect, {
        x: curX, y: y + r * rowH, w: widths[c], h: rowH,
        fill: { color: r === 0 ? "E2E8F0" : C.card },
        line: { color: C.line, width: 0.7 }
      });
      text(slide, cell, curX + 0.05, y + r * rowH + 0.03, widths[c] - 0.1, rowH - 0.06, {
        size, bold: r === 0, align: c === 0 ? "center" : "left"
      });
      curX += widths[c];
    });
  });
}

{
  const s = pptx.addSlide();
  title(s, "max_hits_cfg Sweep v001", "cfg=0..7, width=32/64/128에서 C04 final stream 계약 검증");
  box(s, "cfg=0\nalias 7 hits", 0.9, 1.7, 1.75, 0.9, { fill: C.orangeFill, line: C.orange, bold: true });
  arrow(s, 2.7, 2.15, 3.35, 2.15, C.orange);
  box(s, "runtime\nbeats/cell", 3.4, 1.7, 1.85, 0.9, { fill: C.blueFill, line: C.blue, bold: true });
  arrow(s, 5.3, 2.15, 5.95, 2.15, C.blue);
  box(s, "rise/fall\nframe_done", 6.0, 1.7, 1.95, 0.9, { fill: C.cyanFill, line: C.cyan, bold: true });
  arrow(s, 8.0, 2.15, 8.65, 2.15, C.cyan);
  box(s, "metadata[6:0]\n= 0", 8.7, 1.7, 1.85, 0.9, { fill: C.greenFill, line: C.green, bold: true });
  arrow(s, 10.6, 2.15, 11.25, 2.15, C.green);
  box(s, "PASS\n3 widths", 11.3, 1.7, 1.4, 0.9, { fill: C.greenFill, line: C.green, bold: true });
  table(s, [
    ["검증", "내용"],
    ["대상", "max_hits_cfg 0..7, output width 32/64/128"],
    ["공식", "output_delta = header_beats + 2 * beats_per_cell"],
    ["보강", "rise/fall final metadata sanitize 직접 확인"]
  ], 1.0, 4.1, [2.0, 9.2], 0.5, 10);
  footer(s, "기준: Doc/TDC-GPX-Datasheet.pdf / 결과 문서: MaxHits_Sweep_v001");
}

{
  const s = pptx.addSlide();
  title(s, "Pipeline / II", "cfg는 beat 수를 줄이고, beat II는 유지");
  box(s, "cfg latch", 0.8, 1.55, 1.45, 0.7, { fill: C.slate, bold: true });
  arrow(s, 2.3, 1.9, 2.95, 1.9);
  box(s, "cell stream", 3.0, 1.55, 1.6, 0.7, { fill: C.card, bold: true });
  arrow(s, 4.65, 1.9, 5.3, 1.9);
  box(s, "face_assembler\nruntime last beat", 5.35, 1.45, 2.1, 0.9, { fill: C.blueFill, line: C.blue, bold: true, size: 9.5 });
  arrow(s, 7.5, 1.9, 8.15, 1.9);
  box(s, "FIFO", 8.2, 1.55, 1.15, 0.7, { fill: C.cyanFill, line: C.cyan, bold: true });
  arrow(s, 9.4, 1.9, 10.05, 1.9);
  box(s, "header + VDMA\nII=1 beat/clk", 10.1, 1.45, 2.2, 0.9, { fill: C.greenFill, line: C.green, bold: true, size: 9.5 });
  table(s, [
    ["항목", "판단"],
    ["Latency", "cfg가 작아지면 line output beat 수 감소"],
    ["Throughput", "width가 넓을수록 line beat 수 감소"],
    ["II", "backpressure가 없으면 final output 1 beat/clk"],
    ["Metadata", "마지막 beat 위치는 runtime beats/cell 기준"]
  ], 1.0, 3.45, [2.0, 9.2], 0.48, 9.7);
  footer(s, "metadata clear는 등록 출력 전 bit mask라 pipeline stage 추가 없음");
}

{
  const s = pptx.addSlide();
  title(s, "32bit / 64bit 결과", "cfg=0은 cfg=7과 같은 effective 7 hits");
  table(s, [
    ["cfg", "32b b/cell", "32b out", "64b b/cell", "64b out"],
    ["0", "5", "22", "3", "12"],
    ["1", "2", "16", "2", "10"],
    ["2", "2", "16", "2", "10"],
    ["3", "3", "18", "2", "10"],
    ["4", "3", "18", "2", "10"],
    ["5", "4", "20", "3", "12"],
    ["6", "4", "20", "3", "12"],
    ["7", "5", "22", "3", "12"]
  ], 0.75, 1.35, [1.0, 2.1, 1.8, 2.1, 1.8], 0.43, 9.2);
  box(s, "PASS: xsim_output_stage_maxhits_w32.log / w64.log", 1.0, 5.65, 11.2, 0.55, { fill: C.greenFill, line: C.green, bold: true });
  footer(s, "조건: active chip 1개, stops_per_chip=2");
}

{
  const s = pptx.addSlide();
  title(s, "128bit 결과", "모든 cfg에서 2 beats/cell, output_delta 7");
  table(s, [
    ["cfg", "effective", "beats/cell", "output_delta", "결과"],
    ["0", "7", "2", "7", "PASS"],
    ["1", "1", "2", "7", "PASS"],
    ["2", "2", "2", "7", "PASS"],
    ["3", "3", "2", "7", "PASS"],
    ["4", "4", "2", "7", "PASS"],
    ["5", "5", "2", "7", "PASS"],
    ["6", "6", "2", "7", "PASS"],
    ["7", "7", "2", "7", "PASS"]
  ], 1.05, 1.25, [1.0, 1.8, 2.0, 2.1, 1.8], 0.43, 9.2);
  box(s, "PASS: xsim_output_stage_maxhits_w128.log", 1.0, 5.65, 11.2, 0.55, { fill: C.greenFill, line: C.green, bold: true });
  footer(s, "128bit에서는 7 hit + metadata가 2 beats/cell 안에 들어간다");
}

{
  const s = pptx.addSlide();
  title(s, "결론", "max_hits_cfg runtime scaling과 Hit[16] 폐기 정책 검증 완료");
  table(s, [
    ["항목", "결과"],
    ["cfg=0 alias", "effective 7 hits로 검증 완료"],
    ["width scaling", "32/64/128 모두 기대 output_delta 일치"],
    ["metadata sanitize", "rise/fall final stream 모두 metadata[6:0]=0"],
    ["기존 회귀", "64bit smoke/abort PASS 유지"],
    ["후속", "full 17-bit output 복원은 다음 generation"]
  ], 1.05, 1.45, [2.15, 9.0], 0.58, 10);
  footer(s, "Vivado/xsim 경로: C:\\AMDDesignTools\\2025.2.1\\Vivado");
}

pptx.writeFile({ fileName: out });
