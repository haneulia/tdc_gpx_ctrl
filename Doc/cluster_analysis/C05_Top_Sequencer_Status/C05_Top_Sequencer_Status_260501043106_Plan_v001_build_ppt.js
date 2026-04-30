const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C05_Top_Sequencer_Status/C05_Top_Sequencer_Status_260501043106_Plan_v001.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C05 Top Sequencer Status Plan v001";
pptx.title = "C05 Top Sequencer Status Plan v001";
pptx.lang = "ko-KR";
pptx.theme = { headFontFace: "Malgun Gothic", bodyFontFace: "Malgun Gothic", lang: "ko-KR" };

const font = "Malgun Gothic";
const C = {
  bg: "FAFBFD", ink: "172033", muted: "64748B", line: "CBD5E1", white: "FFFFFF",
  blue: "2563EB", blueFill: "EFF6FF", green: "16A34A", greenFill: "ECFDF5",
  red: "DC2626", redFill: "FEF2F2", orange: "EA580C", orangeFill: "FFF7ED",
  cyan: "0891B2", cyanFill: "ECFEFF", purple: "7C3AED", purpleFill: "F5F3FF"
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
  title(s, "C05 분석 범위", "C01~C04 데이터 파이프라인을 top-level 운용 제어와 status/IRQ 계약으로 연결한다.");
  box(s, "start_tdc", 0.8, 1.55, 1.45, 0.7, { fill: C.blueFill, line: C.blue, bold: true });
  arrow(s, 2.3, 1.9, 2.9, 1.9, C.blue);
  box(s, "face_seq", 2.95, 1.55, 1.6, 0.7, { fill: C.cyanFill, line: C.cyan, bold: true });
  arrow(s, 4.6, 1.9, 5.2, 1.9, C.cyan);
  box(s, "C01~C04", 5.25, 1.55, 1.6, 0.7, { fill: C.purpleFill, line: C.purple, bold: true });
  arrow(s, 6.9, 1.9, 7.5, 1.9, C.purple);
  box(s, "VDMA/PS", 7.55, 1.55, 1.65, 0.7, { fill: C.orangeFill, line: C.orange, bold: true });
  arrow(s, 9.25, 1.9, 9.85, 1.9, C.orange);
  box(s, "status/IRQ", 9.9, 1.55, 1.8, 0.7, { fill: C.greenFill, line: C.green, bold: true });
  table(s, [
    ["대상", "역할"],
    ["tdc_gpx_face_seq.vhd", "shot/face sequencer, face_closing, frame_done_both"],
    ["tdc_gpx_status_agg.vhd", "status, timestamp, overrun aggregation"],
    ["tdc_gpx_top.vhd", "C01~C04와 외부 AXIS/IRQ 연결"],
    ["downstream contract", "VDMA/PS/Ethernet 8us reserve 검증"]
  ], 1.0, 3.25, [3.4, 7.7], 0.48, 10);
  footer(s, "C05_Top_Sequencer_Status_260501043106_Plan_v001.md");
}

{
  const s = pptx.addSlide();
  title(s, "C05 핵심 질문", "다음 start_tdc를 언제 받아도 되는지 top-level 기준으로 닫는다.");
  table(s, [
    ["질문", "확인 방법"],
    ["C04 drain 중 새 start_tdc가 들어오면?", "face_seq deferral/drop/overrun 분석"],
    ["rising/falling lane이 모두 닫혀야 하는가?", "frame_done_both와 face_closing timing 확인"],
    ["status/IRQ가 fault를 추적 가능한가?", "status bit map, sticky clear, IRQ 동작 검증"],
    ["polygon 8us reserve가 안전한가?", "VDMA ready stall과 PS/Ethernet worst-case 반영"],
    ["width별 II가 유지되는가?", "32/64/128 backpressure TB"]
  ], 0.75, 1.35, [4.2, 7.2], 0.58, 10.5);
  footer(s, "Datasheet 기준은 계속 Doc/TDC-GPX-Datasheet.pdf");
}

{
  const s = pptx.addSlide();
  title(s, "Timing 분석 축", "C05 결과에는 latency, throughput, pipeline, II와 timing diagram을 반드시 포함한다.");
  table(s, [
    ["시각", "의미"],
    ["T0", "start_tdc"],
    ["T1", "shot_start_gated / packet_start"],
    ["T2", "GPX stop_tdc / raw drain 완료"],
    ["T3", "C04 final AXIS first beat"],
    ["T4", "C04 final AXIS tlast"],
    ["T5", "frame_done_both / status update"],
    ["T6", "next start_tdc 허용"]
  ], 1.0, 1.35, [1.2, 8.5], 0.48, 10.8);
  box(s, "검증 축", 1.0, 5.25, 1.5, 0.45, { fill: C.blueFill, line: C.blue, bold: true });
  text(s, "Latency / Throughput / Pipeline / II / Timing Diagram / Backpressure negative test", 2.75, 5.18, 8.7, 0.55, { size: 12, bold: true });
  footer(s, "C05 첫 산출물: Analysis v001");
}

pptx.writeFile({ fileName: out });
