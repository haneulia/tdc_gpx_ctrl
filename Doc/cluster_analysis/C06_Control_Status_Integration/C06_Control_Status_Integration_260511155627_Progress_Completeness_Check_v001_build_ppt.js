const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C06_Control_Status_Integration/C06_Control_Status_Integration_260511155627_Progress_Completeness_Check_v001.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C06 Progress and Module Completeness Check v001";
pptx.title = "C06 Progress and Module Completeness Check v001";
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
  title(s, "진행률 / 완성도 요약", "C01~C04 data path는 닫혔고, C06 control/status closure가 남아 있다.");
  table(s, [
    ["구분", "판단", "근거"],
    ["Cluster 진행률", "약 80%", "C01~C04 close, C06 plan만 완료"],
    ["Data path 완성도", "약 90%", "bus/acquisition/cell/output 검증 존재"],
    ["Control/status 완성도", "약 55%", "C06 VB-C06-01..10 미실행"],
    ["전체 운용 완성도", "약 75%", "system ready/status/IRQ closure 전"]
  ], 0.85, 1.35, [3.0, 1.7, 6.8], 0.58, 10.3);
  box(s, "판정: C06을 건너뛰면 안 된다. top-level sequence, status, IRQ, backpressure가 닫혀야 운용 완료로 볼 수 있다.", 0.95, 5.0, 11.2, 0.82, { fill: C.orangeFill, line: C.orange, bold: true, size: 12.2 });
  footer(s, "기준: C04_Output_Stage_260501044033_C06_Handoff_v002.md");
}

{
  const s = pptx.addSlide();
  title(s, "계획 대비 진행 상황", "초기 C05 Face_Output은 실제 C04에서 수행되었고, 현재는 C06 진입 상태다.");
  const y = 1.6;
  box(s, "C01\nBus Read\nClose", 0.55, y, 1.45, 0.8, { fill: C.greenFill, line: C.green, bold: true });
  arrow(s, 2.05, y + 0.4, 2.45, y + 0.4, C.green);
  box(s, "C02\nAcquisition\nClose", 2.5, y, 1.55, 0.8, { fill: C.greenFill, line: C.green, bold: true });
  arrow(s, 4.1, y + 0.4, 4.5, y + 0.4, C.green);
  box(s, "C03\nCell Pipe\nClose", 4.55, y, 1.5, 0.8, { fill: C.greenFill, line: C.green, bold: true });
  arrow(s, 6.1, y + 0.4, 6.5, y + 0.4, C.green);
  box(s, "C04\nOutput\nClose", 6.55, y, 1.45, 0.8, { fill: C.greenFill, line: C.green, bold: true });
  arrow(s, 8.05, y + 0.4, 8.45, y + 0.4, C.orange);
  box(s, "C06\nControl/Status\nOpen", 8.5, y, 1.85, 0.8, { fill: C.orangeFill, line: C.orange, bold: true });
  table(s, [
    ["Cluster", "현재 상태"],
    ["C05_Face_Output", "C04_Output_Stage에 흡수, numbering superseded"],
    ["C06_Control_Status_Integration", "Plan v001 완료, 분석/검증 미완료"],
    ["다음 필수 산출", "Analysis v001 + Code Review + Verify"]
  ], 1.1, 3.45, [3.4, 7.1], 0.52, 10.2);
  footer(s, "기준 계획: cluster_analysis_communication_plan_20260429_v001.md");
}

{
  const s = pptx.addSlide();
  title(s, "RTL 완성도", "구조는 분리되어 있으나 C06 관점의 system 검증이 아직 필요하다.");
  box(s, "tdc_gpx_top", 0.85, 1.45, 1.6, 0.65, { fill: C.blueFill, line: C.blue, bold: true });
  arrow(s, 2.5, 1.78, 3.0, 1.78, C.blue);
  box(s, "face_seq\nregistered", 3.05, 1.45, 1.75, 0.65, { fill: C.cyanFill, line: C.cyan, bold: true });
  arrow(s, 4.85, 1.78, 5.35, 1.78, C.cyan);
  box(s, "C01~C04\ndata pipe", 5.4, 1.45, 1.75, 0.65, { fill: C.purpleFill, line: C.purple, bold: true });
  arrow(s, 7.2, 1.78, 7.7, 1.78, C.purple);
  box(s, "status_agg\nsticky/counter", 7.75, 1.45, 1.95, 0.65, { fill: C.greenFill, line: C.green, bold: true });
  arrow(s, 9.75, 1.78, 10.25, 1.78, C.green);
  box(s, "SW/IRQ\nVDMA", 10.3, 1.45, 1.55, 0.65, { fill: C.orangeFill, line: C.orange, bold: true });
  table(s, [
    ["좋은 점", "남은 점"],
    ["g_OUTPUT_WIDTH assert 32/64/128", "status_agg busy/overrun concurrent path review"],
    ["packet_start / frame_done_both / face_closing registered", "C04 drain 중 next start 정책 검증"],
    ["sticky/error counter sequential", "IRQ pulse/level, sticky clear 검증"],
    ["C04 output/polygon PASS", "tready stall + 8us reserve 결합 검증"]
  ], 0.8, 3.15, [5.3, 5.8], 0.48, 9.5);
  footer(s, "RTL 근거: tdc_gpx_top.vhd, tdc_gpx_face_seq.vhd, tdc_gpx_status_agg.vhd");
}

{
  const s = pptx.addSlide();
  title(s, "검증 Closure 상태", "C01~C04는 PASS 근거가 있고, C06 검증 matrix는 아직 open이다.");
  table(s, [
    ["Cluster", "검증 상태", "대표 근거"],
    ["C01", "PASS", "C01 regression ALL PASS + negative evidence"],
    ["C02", "PASS", "width32/64/128 top int PASS, width matrix PASS"],
    ["C03", "PASS", "cell pipe fix / handoff v002"],
    ["C04", "PASS", "polygon budget matrix PASS"],
    ["C06", "OPEN", "VB-C06-01..10 미실행"]
  ], 0.8, 1.35, [1.4, 2.2, 7.7], 0.55, 10.2);
  box(s, "C06 완료 조건: normal sequence, drain 중 next start, tready stall, sticky clear, width sweep, polygon+backpressure를 xsim으로 닫기", 0.95, 5.1, 11.1, 0.72, { fill: C.redFill, line: C.red, bold: true, size: 11.8 });
  footer(s, "검증 계획: C06_Control_Status_Integration_260501044033_Plan_v001.md:175-186");
}

{
  const s = pptx.addSlide();
  title(s, "다음 진행 권고", "C06 Analysis v001부터 시작해 end-to-end 운용 개념을 닫는다.");
  table(s, [
    ["순서", "작업"],
    ["1", "Datasheet I-Mode/status/interrupt page-table 근거 재확정"],
    ["2", "tdc_gpx_top / face_seq / status_agg 관계도 작성"],
    ["3", "T0~T6 latency, throughput, pipeline, II timing block diagram 작성"],
    ["4", "VB-C06-01..10 검증 matrix 실행"],
    ["5", "Code review finding을 수정계획으로 변환"],
    ["6", "C06 Verify 문서로 PASS/FAIL과 잔여 위험 분리"]
  ], 1.0, 1.35, [1.2, 9.8], 0.55, 10.2);
  footer(s, "판정: 지금 다음 단계는 C06 실제 분석이며, C06을 닫기 전 최종 완료 판단은 보류");
}

pptx.writeFile({ fileName: out });
