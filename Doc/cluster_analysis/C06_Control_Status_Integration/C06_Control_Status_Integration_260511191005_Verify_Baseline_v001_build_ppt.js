const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C06_Control_Status_Integration/C06_Control_Status_Integration_260511191005_Verify_Baseline_v001.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C06 Control Status Integration Verify Baseline v001";
pptx.title = "C06 Control Status Integration Verify Baseline v001";
pptx.lang = "ko-KR";
pptx.theme = {
  headFontFace: "Malgun Gothic",
  bodyFontFace: "Malgun Gothic",
  lang: "ko-KR",
};

const font = "Malgun Gothic";
const C = {
  bg: "F8FAFC",
  ink: "111827",
  muted: "64748B",
  line: "CBD5E1",
  white: "FFFFFF",
  blue: "2563EB",
  blueFill: "EFF6FF",
  green: "16A34A",
  greenFill: "ECFDF5",
  red: "DC2626",
  redFill: "FEF2F2",
  orange: "EA580C",
  orangeFill: "FFF7ED",
  cyan: "0891B2",
  cyanFill: "ECFEFF",
  purple: "7C3AED",
  purpleFill: "F5F3FF",
  slate: "E2E8F0",
  dark: "0F172A",
};

function bg(slide) {
  slide.background = { color: C.bg };
}

function text(slide, value, x, y, w, h, opt = {}) {
  slide.addText(value, {
    x, y, w, h,
    fontFace: font,
    fontSize: opt.size || 11,
    bold: opt.bold || false,
    color: opt.color || C.ink,
    align: opt.align || "left",
    valign: opt.valign || "mid",
    fit: "shrink",
    margin: opt.margin === undefined ? 0.04 : opt.margin,
  });
}

function title(slide, main, sub) {
  bg(slide);
  text(slide, main, 0.55, 0.24, 12.2, 0.48, { size: 21, bold: true });
  text(slide, sub, 0.58, 0.74, 12.15, 0.34, { size: 9.4, color: C.muted });
  slide.addShape(pptx.ShapeType.line, {
    x: 0.55, y: 1.1, w: 12.2, h: 0,
    line: { color: C.line, width: 0.8 },
  });
}

function footer(slide, value) {
  text(slide, value, 0.65, 6.98, 12.0, 0.24, { size: 7.8, color: C.muted, align: "center" });
}

function box(slide, value, x, y, w, h, opt = {}) {
  slide.addShape(pptx.ShapeType.roundRect, {
    x, y, w, h,
    rectRadius: 0.06,
    fill: { color: opt.fill || C.white },
    line: { color: opt.line || C.line, width: opt.lineWidth || 1.0 },
  });
  text(slide, value, x + 0.07, y + 0.05, w - 0.14, h - 0.1, {
    size: opt.size || 11,
    bold: opt.bold || false,
    color: opt.color || C.ink,
    align: opt.align || "center",
    valign: opt.valign || "mid",
  });
}

function arrow(slide, x1, y1, x2, y2, color = C.muted, width = 1.4) {
  slide.addShape(pptx.ShapeType.line, {
    x: x1, y: y1, w: x2 - x1, h: y2 - y1,
    line: { color, width, endArrowType: "triangle" },
  });
}

function line(slide, x1, y1, x2, y2, color = C.line, width = 1) {
  slide.addShape(pptx.ShapeType.line, {
    x: x1, y: y1, w: x2 - x1, h: y2 - y1,
    line: { color, width },
  });
}

function table(slide, rows, x, y, widths, rowH, size = 8.2) {
  rows.forEach((row, r) => {
    let curX = x;
    row.forEach((cell, c) => {
      const head = r === 0;
      slide.addShape(pptx.ShapeType.rect, {
        x: curX, y: y + r * rowH, w: widths[c], h: rowH,
        fill: { color: head ? C.slate : C.white },
        line: { color: C.line, width: 0.5 },
      });
      text(slide, cell, curX + 0.04, y + r * rowH + 0.02, widths[c] - 0.08, rowH - 0.04, {
        size,
        bold: head,
        align: c === 0 ? "center" : "left",
      });
      curX += widths[c];
    });
  });
}

{
  const s = pptx.addSlide();
  title(s, "C06 Baseline 검증", "현재 RTL 수정 전 기준선을 잡기 위한 xsim snapshot 재실행 결과다.");

  box(s, "검증 결과\n일부 PASS / 일부 Open", 0.75, 1.55, 2.45, 0.92, { fill: C.orangeFill, line: C.orange, bold: true });
  box(s, "PASS 영역\nface_seq, top width 32/64/128, CTL21 early/late, polygon model", 3.45, 1.55, 4.0, 0.92, { fill: C.greenFill, line: C.green, bold: true, size: 9.7 });
  box(s, "Open 영역\ntready stall, sticky clear, IRQ, rise/fall imbalance, exact T0~T6", 7.7, 1.55, 4.45, 0.92, { fill: C.redFill, line: C.red, bold: true, size: 9.7 });

  text(s, "실행 조건", 0.85, 3.05, 1.55, 0.3, { size: 14, bold: true });
  box(s, "xsim path\nC:\\AMDDesignTools\\2025.2.1\\Vivado", 0.85, 3.48, 3.25, 0.74, { fill: C.white, line: C.line, size: 9.2 });
  box(s, "방식\n기존 2025.2.1 elaborated snapshot 재실행", 4.35, 3.48, 3.25, 0.74, { fill: C.white, line: C.line, size: 9.2 });
  box(s, "주의\n수정 후에는 fresh compile/elab/xsim 필요", 7.85, 3.48, 3.55, 0.74, { fill: C.orangeFill, line: C.orange, size: 9.2, bold: true });

  text(s, "Datasheet 기준", 0.85, 4.75, 1.7, 0.3, { size: 14, bold: true });
  box(s, "I-Mode single\nStartTimer=0, external start 수락/보류/드롭", 0.85, 5.17, 3.25, 0.7, { fill: C.blueFill, line: C.blue, size: 9.0 });
  box(s, "IFIFO read\nIrFlag 이후 EF 확인과 read sequence", 4.35, 5.17, 3.25, 0.7, { fill: C.purpleFill, line: C.purple, size: 9.0 });
  box(s, "C06 관점\nstream 완료와 status/IRQ/rearm까지 포함", 7.85, 5.17, 3.55, 0.7, { fill: C.cyanFill, line: C.cyan, size: 9.0 });

  footer(s, "C06_Control_Status_Integration_260511191005_Verify_Baseline_v001");
}

{
  const s = pptx.addSlide();
  title(s, "Baseline PASS 근거", "기본 정상 흐름과 폭별 출력은 재실행 로그에서 PASS를 확인했다.");

  table(s, [
    ["검증", "결과", "핵심 근거"],
    ["face_seq", "PASS", "Scenario A~E, ALL SCENARIOS PASSED"],
    ["Top 32-bit", "PASS", "rising/falling beats=72, tlast=2"],
    ["Top 64-bit", "PASS", "rising/falling beats=44, tlast=2"],
    ["Top 128-bit", "PASS", "rising/falling beats=38, tlast=2"],
    ["CTL21 early", "PASS", "early max_hits write 후 stream PASS"],
    ["CTL21 late", "PASS", "late write 후 2 faces/4 cols stream PASS"],
    ["Polygon model", "PASS", "32/64/128 boundary sweep complete"],
  ], 0.65, 1.35, [2.25, 1.35, 8.85], 0.55, 8.8);

  box(s, "중요 해석\n이 PASS는 ready-high 중심의 baseline이다. C06 완료 판정에는 backpressure와 status/IRQ 전용 검증이 더 필요하다.", 0.95, 6.05, 11.35, 0.62, { fill: C.orangeFill, line: C.orange, bold: true, size: 10.0 });
  footer(s, "Evidence logs: xsim_c06_baseline_*_260511191005.log");
}

{
  const s = pptx.addSlide();
  title(s, "T0~T6 Timing Baseline", "현재 log는 stage별 exact marker가 아니라 shot/drain/output summary 기반이다.");

  const y = 2.1;
  const xs = [0.65, 2.05, 3.45, 5.05, 6.7, 8.35, 10.0];
  const labels = ["T0\nstart", "T1\npacket", "T2\nchip start", "C01/C02\ndrain", "C03/C04\noutput", "T5\nstatus", "T6\nnext"];
  for (let i = 0; i < xs.length; i++) {
    const fill = i < 3 ? C.blueFill : i < 5 ? C.purpleFill : C.greenFill;
    const stroke = i < 3 ? C.blue : i < 5 ? C.purple : C.green;
    box(s, labels[i], xs[i], y, 1.1, 0.72, { fill, line: stroke, bold: true, size: 8.8 });
    if (i < xs.length - 1) arrow(s, xs[i] + 1.12, y + 0.36, xs[i + 1] - 0.08, y + 0.36, C.muted);
  }

  line(s, 0.7, 3.35, 11.25, 3.35, C.line, 1.0);
  text(s, "width64 log marker", 0.8, 3.55, 2.0, 0.25, { size: 8.5, color: C.muted, bold: true });
  table(s, [
    ["Marker", "Time", "의미"],
    ["S5", "14112.5 ns", "START command pulse"],
    ["Shot1 start", "14582.5 ns", "col0/shot1 start"],
    ["Shot1 drain done", "20117.5 ns", "expected-count PASS"],
    ["Shot2 start", "25117.5 ns", "col1/shot2 start"],
    ["Shot2 drain done", "30652.5 ns", "expected-count PASS"],
    ["Integrated end", "43162.5 ns", "output summary 완료"],
  ], 0.95, 3.9, [2.15, 2.1, 6.4], 0.36, 7.8);

  box(s, "미해결\nexact packet_start, final tlast time, frame_done_both, next-start admission은 전용 marker가 필요하다.", 0.95, 6.35, 10.65, 0.42, { fill: C.redFill, line: C.red, bold: true, size: 8.8 });
  footer(s, "Timing / Latency / Throughput / Pipeline / II");
}

{
  const s = pptx.addSlide();
  title(s, "Polygon Boundary", "C04 계산 모델 기준 폭별 거리와 max_hits_cfg 경계는 PASS지만, C06 ready stall은 미포함이다.");

  table(s, [
    ["Width", "Boundary", "MAX_PASS_CFG", "의미"],
    ["32", "720 m", "6", "cfg=7 실패 시작"],
    ["32", "750 m", "4", "cfg=6 실패"],
    ["32", "780 m", "2", "cfg=4 실패"],
    ["32", "810 m", "0", "cfg=1 실패"],
    ["64", "790 m", "4", "cfg=7/6 실패"],
    ["64", "820 m", "0", "cfg=1 실패"],
    ["128", "820 m", "0", "cfg=1 실패"],
  ], 0.8, 1.35, [1.2, 1.7, 1.9, 6.65], 0.48, 8.3);

  box(s, "계산식\nusable time = 13.888889 us - 8 us reserve - round_trip_time(distance)", 0.95, 5.65, 5.45, 0.62, { fill: C.blueFill, line: C.blue, bold: true, size: 9.4 });
  box(s, "C06에서 추가할 항목\nready-stall penalty를 T0→T6 latency에 더해 PASS/FAIL 재판정", 6.75, 5.65, 5.1, 0.62, { fill: C.orangeFill, line: C.orange, bold: true, size: 9.4 });
  footer(s, "Evidence: xsim_c06_baseline_polygon_260511191005.log");
}

{
  const s = pptx.addSlide();
  title(s, "VB-C06 Matrix", "이번 baseline으로 닫힌 항목과 다음에 닫아야 할 항목을 분리한다.");

  table(s, [
    ["ID", "상태", "판단"],
    ["VB-C06-01", "Partial PASS", "정상 top 흐름 PASS, exact T0~T6 미관측"],
    ["VB-C06-02", "Partial PASS", "face_seq deferred PASS, top drain 중 next start 미검증"],
    ["VB-C06-03", "Open", "rise/fall imbalance 전용 scenario 필요"],
    ["VB-C06-04", "Open", "output tready stall 미검증"],
    ["VB-C06-05", "Open", "sticky clear/readback 미검증"],
    ["VB-C06-06", "Partial PASS", "CTL21 early/late PASS, field-level snapshot 추가 필요"],
    ["VB-C06-07", "PASS", "32/64/128 top width sweep PASS"],
    ["VB-C06-08", "Open", "reset/soft_reset/force_reinit 미검증"],
    ["VB-C06-09", "Open", "o_irq/o_irq_pipe 미검증"],
    ["VB-C06-10", "Partial PASS", "polygon model PASS, backpressure 미포함"],
  ], 0.55, 1.25, [1.45, 1.65, 9.25], 0.43, 7.5);

  footer(s, "Baseline status drives next Code Fix Plan");
}

{
  const s = pptx.addSlide();
  title(s, "다음 단계", "정상 data flow는 기준선이 잡혔고, 이제 control/status/IRQ/backpressure 계약을 보완한다.");

  box(s, "1\nstatus_agg register화\nbusy/overrun output FF boundary", 0.75, 1.65, 2.25, 1.0, { fill: C.blueFill, line: C.blue, bold: true, size: 9.5 });
  arrow(s, 3.05, 2.15, 3.55, 2.15, C.muted);
  box(s, "2\nface_seq start 경계\nregister화 또는 예외 문서화", 3.6, 1.65, 2.25, 1.0, { fill: C.purpleFill, line: C.purple, bold: true, size: 9.5 });
  arrow(s, 5.9, 2.15, 6.4, 2.15, C.muted);
  box(s, "3\nsticky clear 정책\nsoft_clear 통일/예외 목록", 6.45, 1.65, 2.25, 1.0, { fill: C.greenFill, line: C.green, bold: true, size: 9.5 });
  arrow(s, 8.75, 2.15, 9.25, 2.15, C.muted);
  box(s, "4\nIRQ 계약\no_irq_pipe reserved 또는 pipeline IRQ", 9.3, 1.65, 2.55, 1.0, { fill: C.orangeFill, line: C.orange, bold: true, size: 9.3 });

  box(s, "추천 산출물\nC06_Code_Fix_Plan_v001: baseline 결과를 기반으로 수정 범위와 검증 항목을 고정한다.", 1.05, 3.65, 10.85, 0.78, { fill: C.white, line: C.line, bold: true, size: 10.2 });
  box(s, "수정 후 Exit\nfresh compile/elab/xsim으로 VB-C06-01/03/04/05/09/10 PASS를 확인해야 C06 완료로 볼 수 있다.", 1.05, 4.82, 10.85, 0.78, { fill: C.redFill, line: C.red, bold: true, size: 10.2 });

  footer(s, "Next: C06_Code_Fix_Plan_v001");
}

pptx.writeFile({ fileName: out });

