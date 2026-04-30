const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C04_Output_Stage/C04_Output_Stage_260501043106_C05_Handoff_v001.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C04 to C05 Handoff v001";
pptx.title = "C04 to C05 Handoff v001";
pptx.lang = "ko-KR";
pptx.theme = { headFontFace: "Malgun Gothic", bodyFontFace: "Malgun Gothic", lang: "ko-KR" };

const font = "Malgun Gothic";
const C = {
  bg: "FAFBFD", ink: "172033", muted: "64748B", line: "CBD5E1", white: "FFFFFF",
  blue: "2563EB", blueFill: "EFF6FF", green: "16A34A", greenFill: "ECFDF5",
  red: "DC2626", redFill: "FEF2F2", orange: "EA580C", orangeFill: "FFF7ED",
  cyan: "0891B2", cyanFill: "ECFEFF", purple: "7C3AED", purpleFill: "F5F3FF",
  slate: "F1F5F9"
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
  text(slide, main, 0.55, 0.25, 12.2, 0.5, { size: 21, bold: true });
  text(slide, sub, 0.58, 0.73, 12.1, 0.32, { size: 9.5, color: C.muted });
  slide.addShape(pptx.ShapeType.line, { x: 0.55, y: 1.08, w: 12.2, h: 0, line: { color: C.line, width: 0.8 } });
}
function footer(slide, value) { text(slide, value, 0.65, 6.96, 12, 0.25, { size: 8, color: C.muted, align: "center" }); }
function box(slide, value, x, y, w, h, opt = {}) {
  slide.addShape(pptx.ShapeType.roundRect, {
    x, y, w, h, rectRadius: 0.06,
    fill: { color: opt.fill || C.white },
    line: { color: opt.line || C.line, width: opt.lineWidth || 1 }
  });
  text(slide, value, x + 0.07, y + 0.05, w - 0.14, h - 0.1, {
    size: opt.size || 11, bold: opt.bold || false, color: opt.color || C.ink,
    align: opt.align || "center"
  });
}
function arrow(slide, x1, y1, x2, y2, color = C.muted) {
  slide.addShape(pptx.ShapeType.line, { x: x1, y: y1, w: x2 - x1, h: y2 - y1, line: { color, width: 1.4, endArrowType: "triangle" } });
}
function table(slide, rows, x, y, widths, rowH, size = 8.8) {
  rows.forEach((row, r) => {
    let curX = x;
    row.forEach((cell, c) => {
      const head = r === 0;
      slide.addShape(pptx.ShapeType.rect, {
        x: curX, y: y + r * rowH, w: widths[c], h: rowH,
        fill: { color: head ? "E2E8F0" : C.white },
        line: { color: C.line, width: 0.55 }
      });
      text(slide, cell, curX + 0.04, y + r * rowH + 0.02, widths[c] - 0.08, rowH - 0.04, {
        size, bold: head, align: c === 0 ? "center" : "left"
      });
      curX += widths[c];
    });
  });
}

{
  const s = pptx.addSlide();
  title(s, "C04 종료 판단", "Output Stage는 다음 Cluster로 넘길 수 있다. 남은 위험은 system/top 계약에서 검증한다.");
  box(s, "C03 cell stream", 0.8, 1.55, 2.0, 0.75, { fill: C.blueFill, line: C.blue, bold: true });
  arrow(s, 2.85, 1.92, 3.45, 1.92, C.blue);
  box(s, "face_assembler", 3.5, 1.55, 2.0, 0.75, { fill: C.cyanFill, line: C.cyan, bold: true });
  arrow(s, 5.55, 1.92, 6.15, 1.92, C.cyan);
  box(s, "FIFO + header", 6.2, 1.55, 2.0, 0.75, { fill: C.purpleFill, line: C.purple, bold: true });
  arrow(s, 8.25, 1.92, 8.85, 1.92, C.purple);
  box(s, "final AXIS", 8.9, 1.55, 2.0, 0.75, { fill: C.greenFill, line: C.green, bold: true });
  table(s, [
    ["완료 항목", "근거"],
    ["32/64/128bit output width", "tdc_gpx_output_stage.vhd:34,216"],
    ["Hit[16] final stream 제거", "tdc_gpx_face_assembler.vhd:829"],
    ["header/tuser/frame_done 계약", "tdc_gpx_header_inserter.vhd:127~136"],
    ["polygon budget xsim PASS", "xsim_polygon_budget_matrix.log:554"]
  ], 1.0, 3.25, [4.2, 6.9], 0.46, 10);
  footer(s, "C04_Output_Stage_260501043106_C05_Handoff_v001.md");
}

{
  const s = pptx.addSlide();
  title(s, "C04 -> C05 계약", "C05는 C04 결과를 top-level sequencer/status/downstream system 계약으로 검증한다.");
  table(s, [
    ["계약", "C05 확인"],
    ["output width는 32/64/128", "VDMA width, memory layout, parser 일치"],
    ["rising/falling lane 분리", "VDMA channel mapping"],
    ["tuser(0)=SOF", "SW/VDMA packet boundary 해석"],
    ["Hit[16] 제거", "현재 generation parser는 Hit[15:0]만 사용"],
    ["polygon budget은 8us reserve 가정", "실측 worst-case와 backpressure 반영"],
    ["C04 PASS는 ready 유지 조건", "ready stall negative test"]
  ], 0.75, 1.35, [4.0, 7.6], 0.48, 9.6);
  footer(s, "다음 Cluster: C05_Top_Sequencer_Status");
}

{
  const s = pptx.addSlide();
  title(s, "Polygon Budget 인계", "13.888889us point 간격에서 8us를 예약한 뒤 남은 예산으로 C04 drain을 판단했다.");
  table(s, [
    ["Width", "cfg=7 유지", "스왑 구간", "FAIL"],
    ["32bit", "710m까지", "720~740 cfg6, 750~770 cfg4, 780~800 cfg2", "810m"],
    ["64bit", "780m까지", "790~810 cfg4", "820m"],
    ["128bit", "810m까지", "없음", "820m"]
  ], 0.8, 1.55, [1.3, 2.2, 6.1, 1.3], 0.62, 11);
  box(s, "C05 핵심", 1.05, 4.55, 1.7, 0.5, { fill: C.orangeFill, line: C.orange, bold: true });
  text(s, "8us reserve와 output ready high 조건이 실제 시스템에서 유지되는지 확인해야 C04 timing 결과를 운용 계약으로 확정할 수 있다.", 3.05, 4.45, 8.8, 0.75, { size: 12.2, bold: true });
  footer(s, "근거: C04_Output_Stage_260501042543_Polygon_Budget_Sweep_Result_v001.md");
}

pptx.writeFile({ fileName: out });
