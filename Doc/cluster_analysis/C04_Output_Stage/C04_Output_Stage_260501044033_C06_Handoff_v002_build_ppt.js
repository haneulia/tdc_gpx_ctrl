const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C04_Output_Stage/C04_Output_Stage_260501044033_C06_Handoff_v002.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C04 to C06 Handoff v002";
pptx.title = "C04 to C06 Handoff v002";
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
  title(s, "C04 -> C06 정정 인계", "초기 소통 계획 기준으로 C04 Output Stage 다음은 C06 Control/Status Integration이다.");
  box(s, "초기 C05\nFace_Output", 0.8, 1.55, 2.0, 0.8, { fill: C.blueFill, line: C.blue, bold: true });
  arrow(s, 2.85, 1.95, 3.35, 1.95, C.blue);
  box(s, "실제 수행 완료\nC04 Output Stage", 3.4, 1.55, 2.15, 0.8, { fill: C.greenFill, line: C.green, bold: true });
  arrow(s, 5.6, 1.95, 6.1, 1.95, C.green);
  box(s, "다음 실제 Cluster\nC06 Control/Status", 6.15, 1.55, 2.35, 0.8, { fill: C.orangeFill, line: C.orange, bold: true });
  box(s, "v001 C04->C05 표기는 기능 방향은 맞지만 번호 체계가 불일치하여 v002에서 superseded 처리", 1.15, 3.1, 10.7, 0.85, { fill: C.slate, line: C.line, bold: true, size: 13 });
  table(s, [
    ["문서", "상태"],
    ["C04_Output_Stage_260501043106_C05_Handoff_v001", "명칭 불일치로 superseded"],
    ["C04_Output_Stage_260501044033_C06_Handoff_v002", "정식 인계 문서"],
    ["C06_Control_Status_Integration_260501044033_Plan_v001", "정식 다음 Cluster 계획"]
  ], 1.25, 4.45, [5.1, 5.9], 0.46, 9.6);
  footer(s, "근거: cluster_analysis_communication_plan_20260429_v001.md");
}

{
  const s = pptx.addSlide();
  title(s, "C04 완료 판단", "Output serialization, header insertion, width/cfg sweep, Hit[16] 정책을 닫고 C06으로 인계한다.");
  table(s, [
    ["완료 항목", "근거"],
    ["32/64/128bit output width", "tdc_gpx_output_stage.vhd:34, 216-217"],
    ["rising/falling lane 분리", "tdc_gpx_output_stage.vhd:97-111"],
    ["runtime max_hits_cfg beat 계산", "tdc_gpx_face_assembler.vhd:673-680"],
    ["최종 VDMA stream Hit[16] 제거", "tdc_gpx_face_assembler.vhd:829"],
    ["header/tuser/frame_done 계약", "tdc_gpx_header_inserter.vhd:127-136, 302-308"],
    ["polygon budget xsim PASS", "xsim_polygon_budget_matrix.log:554"]
  ], 0.85, 1.35, [4.4, 7.1], 0.44, 9.2);
  footer(s, "절대 기준: Doc/TDC-GPX-Datasheet.pdf");
}

{
  const s = pptx.addSlide();
  title(s, "C04 Data Flow 계약", "C06은 payload 재설계가 아니라 최종 stream 소비, ready, status/IRQ 계약을 검토한다.");
  box(s, "C03\ncell stream", 0.7, 1.65, 1.55, 0.72, { fill: C.blueFill, line: C.blue, bold: true });
  arrow(s, 2.3, 2.0, 2.85, 2.0, C.blue);
  box(s, "face\nassembler", 2.9, 1.65, 1.6, 0.72, { fill: C.cyanFill, line: C.cyan, bold: true });
  arrow(s, 4.55, 2.0, 5.1, 2.0, C.cyan);
  box(s, "output\nFIFO", 5.15, 1.65, 1.4, 0.72, { fill: C.purpleFill, line: C.purple, bold: true });
  arrow(s, 6.6, 2.0, 7.15, 2.0, C.purple);
  box(s, "header\ninserter", 7.2, 1.65, 1.55, 0.72, { fill: C.greenFill, line: C.green, bold: true });
  arrow(s, 8.8, 2.0, 9.35, 2.0, C.green);
  box(s, "final AXIS\nrise/fall", 9.4, 1.65, 1.65, 0.72, { fill: C.orangeFill, line: C.orange, bold: true });
  arrow(s, 11.1, 2.0, 11.6, 2.0, C.orange);
  box(s, "C06\nsystem/status", 11.65, 1.65, 1.0, 0.72, { fill: C.redFill, line: C.red, bold: true, size: 9.5 });
  table(s, [
    ["계약", "C06 확인"],
    ["width 32/64/128", "VDMA 폭, memory layout, parser 일치"],
    ["tuser(0)=SOF", "SOF/tlast 해석"],
    ["Hit[16] 제거", "현 generation SW parser 16-bit hit slot"],
    ["ready high 가정", "stall negative test"],
    ["max_hits_cfg stable before shot", "CSR snapshot 적용 시점"]
  ], 1.0, 3.45, [4.1, 7.2], 0.46, 9.5);
  footer(s, "C04_Output_Stage_260501044033_C06_Handoff_v002.md");
}

{
  const s = pptx.addSlide();
  title(s, "Timing / Throughput 인계", "13.888889us point interval에서 8us system reserve를 제외하고 C04 drain margin을 평가했다.");
  table(s, [
    ["Width", "cfg=7 유지", "cfg swap 구간", "FAIL 시작"],
    ["32bit", "710m까지", "720~740 cfg6, 750~770 cfg4, 780~800 cfg2", "810m"],
    ["64bit", "780m까지", "790~810 cfg4", "820m"],
    ["128bit", "810m까지", "없음", "820m"]
  ], 0.75, 1.35, [1.3, 2.0, 6.5, 1.3], 0.58, 10.6);
  box(s, "C06 핵심: output tready stall, status update, next start gating을 포함해 C04 PASS 조건이 유지되는지 확인", 1.0, 4.65, 11.1, 0.85, { fill: C.orangeFill, line: C.orange, bold: true, size: 12.4 });
  footer(s, "근거: C04_Output_Stage_260501042543_Polygon_Budget_Sweep_Result_v001.md");
}

{
  const s = pptx.addSlide();
  title(s, "C06 진입 조건", "C06은 top-level sequencing, status aggregation, IRQ, recovery, downstream 계약을 닫는다.");
  table(s, [
    ["C06 검토", "주요 질문"],
    ["face_seq", "다음 start_tdc 허용 조건은 무엇인가?"],
    ["status_agg", "fault/sticky/counter/IRQ가 추적 가능한가?"],
    ["tdc_gpx_top", "C01~C04 연결과 clock/ready 경계가 닫혔는가?"],
    ["VDMA/PS/Ethernet", "8us reserve와 ready high 가정은 유지되는가?"],
    ["end-to-end timing", "Latency/Throughput/Pipeline/II가 하나의 그림으로 닫히는가?"]
  ], 0.85, 1.35, [3.6, 7.7], 0.52, 10.3);
  footer(s, "다음 폴더: Doc/cluster_analysis/C06_Control_Status_Integration");
}

pptx.writeFile({ fileName: out });
