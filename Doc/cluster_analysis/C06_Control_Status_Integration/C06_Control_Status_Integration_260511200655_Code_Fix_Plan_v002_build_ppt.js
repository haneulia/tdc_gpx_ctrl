const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C06_Control_Status_Integration/C06_Control_Status_Integration_260511200655_Code_Fix_Plan_v002.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C06 Code Fix Plan v002";
pptx.title = "C06 Code Fix Plan v002";
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
  orange: "EA580C",
  orangeFill: "FFF7ED",
  red: "DC2626",
  redFill: "FEF2F2",
  purple: "7C3AED",
  purpleFill: "F5F3FF",
  slate: "E2E8F0",
};

function setBg(slide) {
  slide.background = { color: C.bg };
}

function text(slide, value, x, y, w, h, opt = {}) {
  slide.addText(value, {
    x,
    y,
    w,
    h,
    fontFace: font,
    fontSize: opt.size || 10,
    bold: opt.bold || false,
    color: opt.color || C.ink,
    align: opt.align || "left",
    valign: opt.valign || "mid",
    fit: "shrink",
    margin: opt.margin === undefined ? 0.04 : opt.margin,
    breakLine: false,
  });
}

function title(slide, main, sub) {
  setBg(slide);
  text(slide, main, 0.58, 0.24, 12.2, 0.5, { size: 20.5, bold: true });
  text(slide, sub, 0.6, 0.77, 12.0, 0.34, { size: 9.4, color: C.muted });
  slide.addShape(pptx.ShapeType.line, {
    x: 0.58,
    y: 1.12,
    w: 12.2,
    h: 0,
    line: { color: C.line, width: 0.8 },
  });
}

function footer(slide, value) {
  text(slide, value, 0.7, 7.0, 11.9, 0.24, {
    size: 7.6,
    color: C.muted,
    align: "center",
  });
}

function box(slide, value, x, y, w, h, opt = {}) {
  slide.addShape(pptx.ShapeType.roundRect, {
    x,
    y,
    w,
    h,
    rectRadius: 0.06,
    fill: { color: opt.fill || C.white },
    line: { color: opt.line || C.line, width: opt.lineWidth || 1 },
  });
  text(slide, value, x + 0.08, y + 0.05, w - 0.16, h - 0.1, {
    size: opt.size || 9.4,
    bold: opt.bold || false,
    color: opt.color || C.ink,
    align: opt.align || "center",
    valign: "mid",
  });
}

function arrow(slide, x1, y1, x2, y2, color = C.muted) {
  slide.addShape(pptx.ShapeType.line, {
    x: x1,
    y: y1,
    w: x2 - x1,
    h: y2 - y1,
    line: { color, width: 1.35, endArrowType: "triangle" },
  });
}

function table(slide, rows, x, y, widths, rowH, size = 8.2) {
  rows.forEach((row, r) => {
    let curX = x;
    row.forEach((cell, c) => {
      const head = r === 0;
      slide.addShape(pptx.ShapeType.rect, {
        x: curX,
        y: y + r * rowH,
        w: widths[c],
        h: rowH,
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
  setBg(s);
  text(s, "C06 Code Fix Plan v002", 0.75, 0.82, 8.4, 0.62, { size: 27, bold: true });
  text(
    s,
    "Fix Plan v001 실행 결과에서 닫히지 않은 검증/계약 항목을 v002 추적 ID로 승계한다.",
    0.78,
    1.52,
    10.8,
    0.44,
    { size: 13, color: C.muted },
  );
  box(s, "Fix Plan v001\n수정 계약", 0.95, 3.02, 2.35, 0.92, { fill: C.blueFill, line: C.blue, bold: true });
  arrow(s, 3.42, 3.48, 4.25, 3.48);
  box(s, "Result v001\n실행 결과", 4.25, 3.02, 2.35, 0.92, { fill: C.greenFill, line: C.green, bold: true });
  arrow(s, 6.78, 3.48, 7.62, 3.48);
  box(s, "Fix Plan v001\n반영 위치 기록", 7.62, 3.02, 2.55, 0.92, { fill: C.orangeFill, line: C.orange, bold: true });
  arrow(s, 10.28, 3.48, 11.05, 3.48);
  box(s, "Fix Plan v002\n승계 관리", 11.05, 3.02, 1.85, 0.92, { fill: C.purpleFill, line: C.purple, bold: true, size: 8.4 });
  footer(s, "C06_Control_Status_Integration_260511200655_Code_Fix_Plan_v002");
}

{
  const s = pptx.addSlide();
  title(s, "오늘 문서 업데이트 반영", "v001 실행 결과를 C06 분석 문서와 v002 계획 문서에 연결했다.");
  table(
    s,
    [
      ["문서", "반영 내용"],
      ["Progress Check", "v001 실행 뒤 C06 진행도와 남은 검증 항목을 v002로 승계"],
      ["Analysis", "초기 finding F-C06-A01..A06의 현재 상태와 v002 추적 ID 기록"],
      ["Code Review", "F-C06-CR-01..07을 Verified/Partial/Open으로 재분류"],
      ["Verify Baseline", "VB-C06-01..10의 baseline 대비 Result v001 이후 상태 기록"],
      ["Fix Plan v001", "실행 결과와 v002 승계 여부를 원 계획에 역기록"],
      ["Result v001", "FP2-C06-01..08 승계 근거 기록"],
    ],
    0.72,
    1.38,
    [2.35, 9.75],
    0.55,
    8.0,
  );
  footer(s, "근거: Code_Fix_Plan_v002 section 3");
}

{
  const s = pptx.addSlide();
  title(s, "v001 실행 결과 요약", "닫힌 항목과 다음 계획으로 넘어갈 항목을 분리한다.");
  table(
    s,
    [
      ["항목", "Result v001 상태", "v002 처리"],
      ["status_agg register", "Fixed + Verified", "Close"],
      ["face_seq output register", "Fixed + Verified", "T0~T6 영향 계측"],
      ["sticky soft_clear", "Fixed + Verified", "status map 상세 승계"],
      ["o_irq_pipe reserved", "Accepted Exception", "o_irq 실제 계약 확인"],
      ["Phase E 검증", "Partial", "v002 핵심 작업"],
      ["sync_fifo warning", "New Open", "compile-order 정리"],
    ],
    0.8,
    1.55,
    [3.0, 3.0, 5.4],
    0.52,
    8.4,
  );
  footer(s, "근거: Code_Fix_Result_v001 section 5~8");
}

{
  const s = pptx.addSlide();
  title(s, "v002 작업 항목", "FP2-C06-01..08로 남은 검증과 계약을 추적한다.");
  table(
    s,
    [
      ["ID", "작업", "완료 기준"],
      ["01", "sync_fifo compile-order", "syntax warning 제거 또는 예외 근거"],
      ["02", "32/64/128 fresh xsim", "width별 beat/tlast PASS"],
      ["03", "T0~T6 marker", "cycle/time 표 생성"],
      ["04", "tready backpressure", "bounded stall PASS/FAIL"],
      ["05", "rise/fall imbalance", "PASS 또는 Accepted Exception"],
      ["06", "o_irq 계약", "source/read/clear 검증"],
      ["07", "status map", "field/source/clear/domain 표"],
      ["08", "recovery", "deadlock 없음 또는 인계 예외"],
    ],
    0.72,
    1.42,
    [1.05, 4.0, 6.2],
    0.43,
    7.9,
  );
  footer(s, "근거: Fix Plan v002 section 6, 9");
}

{
  const s = pptx.addSlide();
  title(s, "Timing / Latency / Throughput / Pipeline / II", "v002는 +1 clock 예측을 실제 T0~T6와 backpressure II로 닫는다.");
  const y = 1.85;
  box(s, "T0\nstart_tdc", 0.8, y, 1.35, 0.65, { fill: C.white, line: C.line, bold: true });
  arrow(s, 2.2, y + 0.32, 3.0, y + 0.32);
  box(s, "T1\npacket/start accept", 3.0, y, 1.75, 0.65, { fill: C.blueFill, line: C.blue, bold: true });
  arrow(s, 4.8, y + 0.32, 5.6, y + 0.32);
  box(s, "T2\nregistered start", 5.6, y, 1.75, 0.65, { fill: C.greenFill, line: C.green, bold: true });
  arrow(s, 7.4, y + 0.32, 8.2, y + 0.32);
  box(s, "T3~T5\nGPX/C02~C04", 8.2, y, 1.9, 0.65, { fill: C.purpleFill, line: C.purple, bold: true });
  arrow(s, 10.15, y + 0.32, 10.95, y + 0.32);
  box(s, "T6\nnext start allowed", 10.95, y, 1.75, 0.65, { fill: C.orangeFill, line: C.orange, bold: true });
  table(
    s,
    [
      ["분석", "v002 보완"],
      ["Latency", "T0~T6 cycle/time 실측"],
      ["Throughput", "32/64/128 + stall 조건 반영"],
      ["Pipeline", "start FF, status FF, output drain stage 표시"],
      ["II", "ready-high와 bounded stall 조건을 분리 산출"],
    ],
    1.05,
    4.1,
    [2.2, 9.0],
    0.45,
    8.5,
  );
  footer(s, "근거: Fix Plan v002 section 8~9");
}

{
  const s = pptx.addSlide();
  title(s, "운영 규칙 반영", "앞으로 Plan, Result, 다음 Plan 승계를 문서 자체에 남긴다.");
  box(s, "Plan vN\n실행 계약", 1.0, 1.7, 2.0, 0.78, { fill: C.blueFill, line: C.blue, bold: true });
  arrow(s, 3.1, 2.08, 4.1, 2.08);
  box(s, "Result vN\n검증 결과", 4.1, 1.7, 2.0, 0.78, { fill: C.greenFill, line: C.green, bold: true });
  arrow(s, 6.2, 2.08, 7.2, 2.08);
  box(s, "Plan vN\nforward trace", 7.2, 1.7, 2.0, 0.78, { fill: C.orangeFill, line: C.orange, bold: true });
  arrow(s, 9.3, 2.08, 10.3, 2.08);
  box(s, "Plan vN+1\ncarry-over", 10.3, 1.7, 2.0, 0.78, { fill: C.purpleFill, line: C.purple, bold: true });
  table(
    s,
    [
      ["문서", "반영"],
      ["operating_protocol_v012", "Fix Plan 실행/승계 사이클 규칙"],
      ["communication_plan_v002", "Cluster별 Plan-Result-Rollover 소통 절차"],
      ["Fix Plan v001", "실행 결과 및 v002 승계 기록"],
      ["Result v001", "Result에서 Fix Plan v002로 넘기는 근거"],
    ],
    1.05,
    3.65,
    [3.2, 8.0],
    0.47,
    8.5,
  );
  footer(s, "v002 이후에도 같은 패턴 반복");
}

pptx.writeFile({ fileName: out });
