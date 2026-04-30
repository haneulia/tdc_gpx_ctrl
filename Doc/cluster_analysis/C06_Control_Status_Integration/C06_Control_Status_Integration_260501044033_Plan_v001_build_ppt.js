const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C06_Control_Status_Integration/C06_Control_Status_Integration_260501044033_Plan_v001.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C06 Control Status Integration Plan v001";
pptx.title = "C06 Control Status Integration Plan v001";
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
  title(s, "C06 분석 목적", "C01~C04 데이터 경로를 top-level control, status, IRQ, recovery 계약으로 닫는다.");
  box(s, "start_tdc\nCSR/reset", 0.7, 1.55, 1.65, 0.75, { fill: C.blueFill, line: C.blue, bold: true });
  arrow(s, 2.4, 1.92, 2.95, 1.92, C.blue);
  box(s, "face_seq\nshot/face", 3.0, 1.55, 1.65, 0.75, { fill: C.cyanFill, line: C.cyan, bold: true });
  arrow(s, 4.7, 1.92, 5.25, 1.92, C.cyan);
  box(s, "C01~C04\ndata pipe", 5.3, 1.55, 1.65, 0.75, { fill: C.purpleFill, line: C.purple, bold: true });
  arrow(s, 7.0, 1.92, 7.55, 1.92, C.purple);
  box(s, "VDMA/PS\nEthernet", 7.6, 1.55, 1.75, 0.75, { fill: C.orangeFill, line: C.orange, bold: true });
  arrow(s, 9.4, 1.92, 9.95, 1.92, C.orange);
  box(s, "status/IRQ\nrecovery", 10.0, 1.55, 1.9, 0.75, { fill: C.greenFill, line: C.green, bold: true });
  table(s, [
    ["핵심 질문", "판단 기준"],
    ["다음 start_tdc 허용 조건", "face_closing, frame_done_both, output drain"],
    ["fault/status 추적성", "sticky/counter/IRQ/clear semantic"],
    ["width/cfg 적용 시점", "face/shot 시작 전 snapshot"],
    ["8us reserve 유지", "VDMA/PS/Ethernet ready와 stall"]
  ], 1.0, 3.35, [4.3, 6.8], 0.48, 9.8);
  footer(s, "C06_Control_Status_Integration_260501044033_Plan_v001.md");
}

{
  const s = pptx.addSlide();
  title(s, "C06 분석 범위", "Top integration, face sequencer, status aggregation, downstream 계약을 동시에 본다.");
  table(s, [
    ["대상", "역할", "근거"],
    ["tdc_gpx_top.vhd", "top 연결", "lines 16-17, 853, 917, 962+"],
    ["tdc_gpx_face_seq.vhd", "shot/face sequence", "lines 31, 59, 75-93"],
    ["tdc_gpx_status_agg.vhd", "status/timestamp/overrun", "lines 32, 94, 161-179"],
    ["CSR/config", "width, max_hits_cfg, clear", "C06에서 line 확정"],
    ["VDMA/PS/Ethernet", "8us reserve", "C04 polygon budget 인계"]
  ], 0.65, 1.35, [2.6, 3.5, 5.4], 0.5, 9.4);
  footer(s, "절대 기준: Doc/TDC-GPX-Datasheet.pdf");
}

{
  const s = pptx.addSlide();
  title(s, "상태 관계도 초안", "C06 첫 분석 문서는 실제 RTL line 근거로 이 상태 관계를 검증한다.");
  const y = 1.45;
  box(s, "Idle", 0.55, y, 1.05, 0.55, { fill: C.slate, line: C.line, bold: true });
  arrow(s, 1.62, y + 0.27, 2.1, y + 0.27, C.muted);
  box(s, "ArmFace", 2.15, y, 1.25, 0.55, { fill: C.blueFill, line: C.blue, bold: true });
  arrow(s, 3.42, y + 0.27, 3.9, y + 0.27, C.blue);
  box(s, "ShotOpen", 3.95, y, 1.35, 0.55, { fill: C.cyanFill, line: C.cyan, bold: true });
  arrow(s, 5.32, y + 0.27, 5.8, y + 0.27, C.cyan);
  box(s, "GPXBusy", 5.85, y, 1.35, 0.55, { fill: C.purpleFill, line: C.purple, bold: true });
  arrow(s, 7.22, y + 0.27, 7.7, y + 0.27, C.purple);
  box(s, "OutputDrain", 7.75, y, 1.55, 0.55, { fill: C.orangeFill, line: C.orange, bold: true });
  arrow(s, 9.32, y + 0.27, 9.8, y + 0.27, C.orange);
  box(s, "StatusUpdate", 9.85, y, 1.65, 0.55, { fill: C.greenFill, line: C.green, bold: true });
  arrow(s, 10.65, y + 0.65, 1.1, y + 1.65, C.green);
  box(s, "FaultHold / Recovery", 4.6, 3.35, 2.2, 0.65, { fill: C.redFill, line: C.red, bold: true });
  arrow(s, 4.65, 2.05, 5.3, 3.3, C.red);
  arrow(s, 6.5, 2.05, 6.0, 3.3, C.red);
  arrow(s, 8.5, 2.05, 6.6, 3.3, C.red);
  table(s, [
    ["검증 포인트", "이유"],
    ["face_closing registered boundary", "조합 depth와 timing 분석성"],
    ["frame_done_both 의미", "rise/fall lane 모두 종료 확인"],
    ["next start gating", "C04 drain 중 shot 충돌 방지"]
  ], 1.0, 5.05, [4.4, 6.8], 0.48, 9.6);
  footer(s, "상태명은 분석 초안이며 C06 Analysis v001에서 RTL 기준으로 확정");
}

{
  const s = pptx.addSlide();
  title(s, "Data Flow와 Control Flow 분리", "C06은 데이터 beat 자체보다 제어와 status 경계가 닫혔는지 판단한다.");
  box(s, "Data Flow\nC01 read -> C02 raw -> C03 cell -> C04 AXIS", 0.9, 1.45, 5.1, 1.0, { fill: C.blueFill, line: C.blue, bold: true, size: 13 });
  box(s, "Control/Status Flow\nstart_tdc -> face_seq -> busy/closing/status/IRQ", 7.1, 1.45, 5.1, 1.0, { fill: C.greenFill, line: C.green, bold: true, size: 13 });
  table(s, [
    ["경로", "C06 판단"],
    ["Data", "end-to-end boundary, tready stall, final packet map"],
    ["Control", "shot accept, defer/drop/overrun, face close"],
    ["Status", "fault propagation, sticky clear, IRQ, counter"],
    ["System", "8us reserve, VDMA/PS/Ethernet timing"]
  ], 1.0, 3.3, [2.4, 8.8], 0.52, 10.2);
  footer(s, "C06에서는 data/control/status를 같은 그림에 섞지 않고 구분");
}

{
  const s = pptx.addSlide();
  title(s, "Timing / Pipeline / II 계획", "T0->T6 marker로 전체 운용을 하나의 timing block diagram으로 닫는다.");
  table(s, [
    ["Marker", "의미"],
    ["T0", "external start_tdc 또는 shot request accepted"],
    ["T1", "face_seq shot accept / lower cluster enable"],
    ["T2", "GPX stop/read/drain 완료 전파"],
    ["T3", "C04 final AXIS first beat"],
    ["T4", "C04 final AXIS tlast"],
    ["T5", "frame_done_both / status update"],
    ["T6", "다음 start_tdc 허용"]
  ], 0.75, 1.2, [1.2, 6.0], 0.42, 9.4);
  table(s, [
    ["Metric", "C06 산출"],
    ["Latency", "T0->T6 구간 분해"],
    ["Throughput", "13.888889us point interval 만족 여부"],
    ["Pipeline", "registered/FIFO/CDC stage 표시"],
    ["II", "다음 shot 시작 최소 간격"]
  ], 8.05, 1.2, [1.4, 3.0], 0.5, 9.4);
  footer(s, "C04 인계: 32 cfg7 710m, 64 cfg7 780m, 128 cfg7 810m");
}

{
  const s = pptx.addSlide();
  title(s, "검증 Matrix 초안", "C06 검증은 정상 sequence와 backpressure/fault negative test를 함께 닫는다.");
  table(s, [
    ["ID", "검증 항목", "목적"],
    ["VB-C06-01", "I-Mode single 정상 sequence", "기본 운용 close"],
    ["VB-C06-02", "C04 drain 중 next start", "defer/drop/overrun 정책"],
    ["VB-C06-03", "rise/fall lane 완료 불균형", "frame_done_both 확인"],
    ["VB-C06-04", "output tready stall", "II=1 가정 붕괴 확인"],
    ["VB-C06-05", "sticky clear / IRQ", "SW 추적성"],
    ["VB-C06-06", "max_hits_cfg snapshot", "width 이득 적용 시점"],
    ["VB-C06-07", "32/64/128 width sweep", "system-visible 계약"]
  ], 0.65, 1.25, [1.55, 4.3, 5.2], 0.43, 8.8);
  footer(s, "AXI4-Lite TB helper는 px_utility_pkg.vhd 사용");
}

pptx.writeFile({ fileName: out });
