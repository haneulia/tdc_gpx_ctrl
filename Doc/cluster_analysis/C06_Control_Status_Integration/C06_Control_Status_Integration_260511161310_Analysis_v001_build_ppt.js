const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C06_Control_Status_Integration/C06_Control_Status_Integration_260511161310_Analysis_v001.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C06 Control Status Integration Analysis v001";
pptx.title = "C06 Control Status Integration Analysis v001";
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
  title(s, "C06 분석 목적", "C01~C04 data path를 top-level control/status/IRQ 운용 계약으로 닫기 위한 첫 분석이다.");
  box(s, "Datasheet\nI-Mode single", 0.75, 1.55, 1.9, 0.78, { fill: C.blueFill, line: C.blue, bold: true });
  arrow(s, 2.7, 1.94, 3.25, 1.94, C.blue);
  box(s, "tdc_gpx_top\nface_seq", 3.3, 1.55, 1.9, 0.78, { fill: C.cyanFill, line: C.cyan, bold: true });
  arrow(s, 5.25, 1.94, 5.8, 1.94, C.cyan);
  box(s, "C01~C04\nData Pipe", 5.85, 1.55, 1.75, 0.78, { fill: C.purpleFill, line: C.purple, bold: true });
  arrow(s, 7.65, 1.94, 8.2, 1.94, C.purple);
  box(s, "status_agg\nIRQ/CSR", 8.25, 1.55, 1.8, 0.78, { fill: C.greenFill, line: C.green, bold: true });
  arrow(s, 10.1, 1.94, 10.65, 1.94, C.green);
  box(s, "VDMA/PS\nEthernet", 10.7, 1.55, 1.7, 0.78, { fill: C.orangeFill, line: C.orange, bold: true });
  table(s, [
    ["이번 v001 결론", "상태"],
    ["C06 분석 시작", "가능"],
    ["C06 검증", "미완료"],
    ["다음 산출물", "Code Review v001 + VB-C06 검증 계획"]
  ], 1.0, 3.35, [3.5, 7.4], 0.5, 10.4);
  footer(s, "C06_Control_Status_Integration_260511161310_Analysis_v001.md");
}

{
  const s = pptx.addSlide();
  title(s, "Datasheet 운용 기준", "I-Mode single은 IrFlag 후 EF를 확인하고 IFIFO를 read한 뒤 reset/rearm하는 흐름이다.");
  table(s, [
    ["Datasheet 근거", "C06 적용"],
    ["PDF p25 2.3 I-Mode Basics", "8 stop channels against 1 start"],
    ["PDF p25 Single Start", "StartTimer=0, continuous 제외"],
    ["PDF p27 2.4 Data structure", "ChaCode/Start#/Slope/Hit[16:0]"],
    ["PDF p20 1.7.2 Read registers", "Reg8/9 IFIFO hit data"],
    ["PDF p27 IFIFO/data bus", "40MHz transfer, empty read 금지"],
    ["PDF p29 2.11.1 Single measurement", "IrFlag -> EF check -> IFIFO read -> reset"]
  ], 0.75, 1.25, [4.1, 7.1], 0.52, 9.4);
  footer(s, "절대 기준: Doc/TDC-GPX-Datasheet.pdf");
}

{
  const s = pptx.addSlide();
  title(s, "RTL 상태 관계", "실제 face_seq state는 3개지만, 운용 stage는 packet/start/drain/status로 확장해서 봐야 한다.");
  box(s, "ST_IDLE", 0.7, 1.55, 1.25, 0.6, { fill: C.slate, line: C.line, bold: true });
  arrow(s, 2.0, 1.85, 2.5, 1.85, C.muted);
  box(s, "ST_WAIT_SHOT", 2.55, 1.55, 1.65, 0.6, { fill: C.blueFill, line: C.blue, bold: true });
  arrow(s, 4.25, 1.85, 4.75, 1.85, C.blue);
  box(s, "packet_start_r", 4.8, 1.55, 1.65, 0.6, { fill: C.cyanFill, line: C.cyan, bold: true });
  arrow(s, 6.5, 1.85, 7.0, 1.85, C.cyan);
  box(s, "ST_IN_FACE", 7.05, 1.55, 1.5, 0.6, { fill: C.purpleFill, line: C.purple, bold: true });
  arrow(s, 8.6, 1.85, 9.1, 1.85, C.purple);
  box(s, "frame_done_both", 9.15, 1.55, 1.85, 0.6, { fill: C.greenFill, line: C.green, bold: true });
  arrow(s, 10.1, 2.22, 3.2, 3.0, C.green);
  table(s, [
    ["Registered boundary", "근거"],
    ["packet_start_r", "tdc_gpx_face_seq.vhd:526-535"],
    ["frame_done_both_r", "tdc_gpx_face_seq.vhd:400-424"],
    ["face_closing_r", "tdc_gpx_face_seq.vhd:431-469"],
    ["status sticky/counter", "tdc_gpx_status_agg.vhd:110-156"]
  ], 1.2, 3.65, [4.0, 6.8], 0.48, 9.6);
  footer(s, "RTL 근거: tdc_gpx_face_seq.vhd");
}

{
  const s = pptx.addSlide();
  title(s, "Data / Control / Status 분리", "C06은 payload 재설계가 아니라 시작, 종료, backpressure, 관측성을 닫는다.");
  box(s, "Data Flow\nGPX IFIFO -> C01 -> C02 -> C03 -> C04 -> AXIS", 0.85, 1.35, 5.3, 0.92, { fill: C.blueFill, line: C.blue, bold: true, size: 12.5 });
  box(s, "Control Flow\nstart_tdc -> packet_start -> shot_start_gated -> frame_done", 0.85, 2.75, 5.3, 0.92, { fill: C.cyanFill, line: C.cyan, bold: true, size: 12.5 });
  box(s, "Status Flow\nC01~C04 fault -> status_agg/top -> CSR/IRQ", 0.85, 4.15, 5.3, 0.92, { fill: C.greenFill, line: C.green, bold: true, size: 12.5 });
  table(s, [
    ["C06 질문", "왜 필요한가"],
    ["next start 정책", "II와 drop/defer 결정"],
    ["tready stall", "C04 timing PASS 조건"],
    ["status/IRQ 의미", "SW 추적성"],
    ["8us reserve", "system throughput"]
  ], 6.8, 1.35, [2.5, 3.4], 0.52, 9.8);
  footer(s, "C06 경계: tdc_gpx_top.vhd + face_seq + status_agg");
}

{
  const s = pptx.addSlide();
  title(s, "T0~T6 Timing Block", "정확한 cycle은 xsim marker로 닫고, v001에서는 관측 지점을 확정한다.");
  table(s, [
    ["Marker", "관측 후보", "의미"],
    ["T0", "i_shot_start_raw", "start_tdc 입력"],
    ["T1", "o_packet_start", "face 수락"],
    ["T2", "o_shot_start_gated", "lower cluster enable"],
    ["T3", "final AXIS first beat", "VDMA 첫 payload"],
    ["T4", "final AXIS tlast", "lane output 완료"],
    ["T5", "o_frame_done_both", "rise/fall 종료"],
    ["T6", "next packet_start/drop", "II 판단"]
  ], 0.75, 1.25, [1.25, 4.2, 5.6], 0.45, 9.2);
  box(s, "정적 판독: packet_start는 register되고, shot_start_gated는 shot_pending path를 거쳐 나온다. T0->T2는 zero-cycle이 아니다.", 0.95, 5.1, 11.2, 0.7, { fill: C.orangeFill, line: C.orange, bold: true, size: 11.8 });
  footer(s, "검증 필요: VB-C06-01 marker log");
}

{
  const s = pptx.addSlide();
  title(s, "Next Start 정책", "현재 RTL은 1-deep defer + 추가 start drop 구조로 읽힌다.");
  box(s, "raw start edge", 0.9, 1.55, 1.6, 0.62, { fill: C.blueFill, line: C.blue, bold: true });
  arrow(s, 2.55, 1.86, 3.05, 1.86, C.blue);
  box(s, "accept\npacket_start", 3.1, 1.55, 1.65, 0.62, { fill: C.greenFill, line: C.green, bold: true });
  arrow(s, 4.8, 1.86, 5.3, 1.86, C.green);
  box(s, "if blocked\ns_deferred", 5.35, 1.55, 1.65, 0.62, { fill: C.orangeFill, line: C.orange, bold: true });
  arrow(s, 7.05, 1.86, 7.55, 1.86, C.orange);
  box(s, "if already deferred\ndrop count", 7.6, 1.55, 1.9, 0.62, { fill: C.redFill, line: C.red, bold: true });
  table(s, [
    ["근거", "해석"],
    ["tdc_gpx_face_seq.vhd:616-627", "수락 불가 start를 deferred latch로 보존"],
    ["tdc_gpx_face_seq.vhd:629-640", "추가 start는 drop count 가능"],
    ["tdc_gpx_face_seq.vhd:674-693", "face_closing/abort/stop이 shot_start gate"]
  ], 1.05, 3.3, [4.6, 6.3], 0.5, 9.6);
  footer(s, "검증 필요: VB-C06-02 drain 중 next start");
}

{
  const s = pptx.addSlide();
  title(s, "Preliminary Findings", "분석 단계에서 다음 code review와 검증으로 넘길 항목이다.");
  table(s, [
    ["ID", "등급", "내용"],
    ["F-C06-A01", "P1", "end-to-end II 미검증"],
    ["F-C06-A02", "P1", "C04 timing PASS는 ready-high 조건부"],
    ["F-C06-A03", "P2", "o_irq / o_irq_pipe SW 의미 분리 필요"],
    ["F-C06-A04", "P2", "status source/clear bit map 필요"],
    ["F-C06-A05", "P2", "status_agg concurrent path review 필요"],
    ["F-C06-A06", "P2", "Datasheet Reg12 clear와 RTL sticky 의미 분리 필요"]
  ], 0.75, 1.25, [1.55, 1.1, 8.6], 0.47, 9.4);
  footer(s, "다음 산출물: C06 Code Review v001");
}

{
  const s = pptx.addSlide();
  title(s, "검증 Matrix v001", "VB-C06-01..10을 순차 실행해 control/status closure를 닫는다.");
  table(s, [
    ["ID", "목적", "핵심 관측"],
    ["VB-C06-01", "I-Mode single 정상 sequence", "T0..T6"],
    ["VB-C06-02", "C04 drain 중 next start", "defer/drop"],
    ["VB-C06-03", "rise/fall 완료 불균형", "frame_done_both"],
    ["VB-C06-04", "output tready stall", "overrun/status"],
    ["VB-C06-05", "sticky clear", "soft_clear"],
    ["VB-C06-06", "max_hits_cfg snapshot", "s_cfg_face_r"],
    ["VB-C06-09", "IRQ policy", "o_irq/o_irq_pipe"]
  ], 0.75, 1.2, [1.55, 4.0, 5.6], 0.43, 8.8);
  box(s, "AXI4-Lite CSR TB는 px_utility_pkg.vhd의 px_axi_lite_writer/reader 사용", 0.95, 5.8, 11.1, 0.45, { fill: C.slate, line: C.line, bold: true, size: 11.5 });
  footer(s, "C06 Analysis v001 -> Code Review v001 -> Verify v001");
}

pptx.writeFile({ fileName: out });
