const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/.pnpm/pptxgenjs@4.0.1/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C06_Control_Status_Integration/C06_Control_Status_Integration_260514134116_Code_Fix_Plan_v006.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C06 Code Fix Plan v006";
pptx.title = "C06 Code Fix Plan v006";
pptx.lang = "ko-KR";
pptx.theme = { headFontFace: "Malgun Gothic", bodyFontFace: "Malgun Gothic", lang: "ko-KR" };

const font = "Malgun Gothic";
const C = {
  bg: "F8FAFC", ink: "111827", muted: "64748B", line: "CBD5E1",
  white: "FFFFFF", dark: "0F172A", blue: "2563EB", blueFill: "DBEAFE",
  green: "15803D", greenFill: "DCFCE7", orange: "C2410C", orangeFill: "FFEDD5",
  red: "B91C1C", redFill: "FEE2E2", purple: "6D28D9", purpleFill: "EDE9FE",
  teal: "0F766E", tealFill: "CCFBF1", slate: "E2E8F0",
};

function tx(slide, value, x, y, w, h, opt = {}) {
  slide.addText(value, {
    x, y, w, h,
    fontFace: font,
    fontSize: opt.size || 10,
    bold: !!opt.bold,
    color: opt.color || C.ink,
    align: opt.align || "left",
    valign: "mid",
    fit: "shrink",
    margin: opt.margin === undefined ? 0.04 : opt.margin,
  });
}

function title(slide, main, sub) {
  slide.background = { color: C.bg };
  tx(slide, main, 0.55, 0.28, 12.1, 0.46, { size: 21, bold: true });
  tx(slide, sub, 0.58, 0.78, 12.0, 0.3, { size: 9.2, color: C.muted });
  slide.addShape(pptx.ShapeType.line, { x: 0.58, y: 1.13, w: 12.1, h: 0, line: { color: C.line, width: 0.8 } });
}

function footer(slide, value) {
  tx(slide, value, 0.58, 7.05, 12.15, 0.24, { size: 7.2, color: C.muted, align: "center" });
}

function box(slide, label, x, y, w, h, opt = {}) {
  slide.addShape(pptx.ShapeType.roundRect, {
    x, y, w, h, rectRadius: 0.04,
    fill: { color: opt.fill || C.white },
    line: { color: opt.line || C.line, width: 0.9 },
  });
  tx(slide, label, x + 0.08, y + 0.06, w - 0.16, h - 0.12, {
    size: opt.size || 9.0,
    bold: opt.bold,
    color: opt.color || C.ink,
    align: opt.align || "center",
  });
}

function arrow(slide, x1, y1, x2, y2, color = C.muted) {
  slide.addShape(pptx.ShapeType.line, {
    x: x1, y: y1, w: x2 - x1, h: y2 - y1,
    line: { color, width: 1.2, endArrowType: "triangle" },
  });
}

function table(slide, rows, x, y, widths, rowH, size = 7.4) {
  rows.forEach((row, r) => {
    let curX = x;
    row.forEach((cell, c) => {
      slide.addShape(pptx.ShapeType.rect, {
        x: curX, y: y + r * rowH, w: widths[c], h: rowH,
        fill: { color: r === 0 ? C.slate : C.white },
        line: { color: C.line, width: 0.45 },
      });
      tx(slide, cell, curX + 0.05, y + r * rowH + 0.02, widths[c] - 0.10, rowH - 0.04, {
        size, bold: r === 0, align: c === 0 ? "center" : "left",
      });
      curX += widths[c];
    });
  });
}

{
  const s = pptx.addSlide();
  s.background = { color: C.dark };
  tx(s, "Code Fix Plan v006", 0.72, 0.82, 8.4, 0.55, { size: 28, bold: true, color: C.white });
  tx(s, "Review-driven release hardening plan", 0.76, 1.58, 8.8, 0.42, { size: 15, color: "CBD5E1" });
  box(s, "P0\nwidth recovery", 0.92, 3.05, 2.55, 1.0, { fill: "1E3A8A", line: "93C5FD", color: C.white, bold: true, size: 12 });
  box(s, "P0\nreserve sweep", 3.92, 3.05, 2.55, 1.0, { fill: "713F12", line: "FDE68A", color: C.white, bold: true, size: 12 });
  box(s, "P0\nbackpressure", 6.92, 3.05, 2.55, 1.0, { fill: "7F1D1D", line: "FCA5A5", color: C.white, bold: true, size: 12 });
  box(s, "P1\nmarker audit", 9.92, 3.05, 2.55, 1.0, { fill: "14532D", line: "86EFAC", color: C.white, bold: true, size: 12 });
  footer(s, "생성 2026-05-14 13:41:16 KST | 선행: C06 Review v001");
}

{
  const s = pptx.addSlide();
  title(s, "v006 작업 항목", "v005에서 남은 계약 위험을 검증 가능한 Plan ID로 분리합니다.");
  table(s, [
    ["Plan ID", "작업", "완료 기준"],
    ["FP6-01", "recovery width sweep", "32/64/128 force/soft fresh logs"],
    ["FP6-02", "output backpressure + width", "tready stall 후 beats/tlast 보존"],
    ["FP6-03", "polygon reserve sweep", "8 us baseline + 보수 reserve pass/fail"],
    ["FP6-04", "C01~C04 marker audit", "source/destination/effect 재분류"],
    ["FP6-05", "Hit[16] SW/range contract", "16-bit range와 810 m 위험 명시"],
    ["FP6-06", "sticky clear policy map", "soft-clear/reset-only table"],
    ["FP6-07", "lane imbalance + stall", "frame_done/face_closing/tlast 보존"],
  ], 0.62, 1.36, [1.25, 4.2, 6.0], 0.43, 6.75);
  footer(s, "근거: C06_Control_Status_Integration_260514134116_Code_Fix_Plan_v006.md section 2");
}

{
  const s = pptx.addSlide();
  title(s, "검증 Matrix", "width, recovery, backpressure, polygon budget을 분리해서 닫습니다.");
  table(s, [
    ["검증", "Width", "Scenario", "PASS marker"],
    ["VB6-01", "32/64/128", "force recovery", "source, TDC, PH_INIT, beats/tlast"],
    ["VB6-02", "32/64/128", "soft recovery", "source, TDC, PH_RESP_DRAIN, PH_INIT"],
    ["VB6-03", "32/64/128", "bounded backpressure", "no loss, no duplicate tlast"],
    ["VB6-04", "32/64/128", "polygon reserve", "processing <= window"],
    ["VB6-05", "selected", "lane imbalance + stall", "frame_done_both, next shot"],
    ["VB6-06", "N/A", "sticky clear", "CSR readback map"],
  ], 0.62, 1.45, [1.2, 2.2, 3.1, 5.0], 0.50, 7.1);
  footer(s, "PASS marker는 source/destination/effect 분리 원칙을 따름");
}

{
  const s = pptx.addSlide();
  title(s, "Pipeline / Timing Marker", "v006 결과는 T0~T7 marker를 같은 이름으로 기록합니다.");
  const nodes = [
    ["AXI\nT3 source", 0.85, C.orangeFill, C.orange],
    ["CDC\nT4 dest", 2.82, C.blueFill, C.blue],
    ["chip_ctrl\nT5 effect", 4.92, C.purpleFill, C.purple],
    ["face/status\nlane sticky", 7.12, C.tealFill, C.teal],
    ["output\nT6 TLAST", 9.35, C.greenFill, C.green],
    ["next shot\nT7 II", 11.25, C.slate, C.muted],
  ];
  nodes.forEach(([label, x, fill, line], i) => {
    box(s, label, x, 2.0, 1.45, 0.86, { fill, line, bold: true, size: 7.8 });
    if (i < nodes.length - 1) arrow(s, x + 1.48, 2.43, nodes[i + 1][1] - 0.05, 2.43);
  });
  table(s, [
    ["Metric", "기록 방법"],
    ["Latency", "T3 source, T4 dest, T5 effect 분리"],
    ["Throughput", "expected beats/run, total beats, tlast count"],
    ["Pipeline", "AXI -> CDC -> chip -> face/status -> output"],
    ["II", "T0/T7 accepted/deferred/dropped 분리"],
  ], 1.0, 3.75, [2.2, 8.8], 0.50, 8.0);
  footer(s, "Datasheet I-Mode single sequence 기준 marker와 RTL marker를 분리");
}

{
  const s = pptx.addSlide();
  title(s, "Polygon Budget Sweep", "8 us reserve는 실측값이 아니므로 sweep 또는 board measurement로 닫습니다.");
  box(s, "13.888889 us\nshot interval", 0.9, 1.62, 2.45, 0.9, { fill: C.blueFill, line: C.blue, bold: true });
  box(s, "minus\nround trip time", 3.75, 1.62, 2.45, 0.9, { fill: C.tealFill, line: C.teal, bold: true });
  box(s, "minus\nsystem reserve", 6.6, 1.62, 2.45, 0.9, { fill: C.orangeFill, line: C.orange, bold: true });
  box(s, ">=\nprocessing time", 9.45, 1.62, 2.45, 0.9, { fill: C.greenFill, line: C.green, bold: true });
  table(s, [
    ["Reserve", "목적"],
    ["6 us", "optimistic margin 확인"],
    ["8 us", "사용자 baseline"],
    ["10 us", "보수 case"],
    ["measured", "최종 board/system 기준"],
  ], 1.15, 3.55, [2.2, 8.4], 0.50, 8.2);
  footer(s, "PASS: processing_time + stall_penalty <= 13.888889 us - round_trip - reserve");
}

{
  const s = pptx.addSlide();
  title(s, "실행 순서", "문서 계획을 fresh xsim evidence와 system 계약으로 연결합니다.");
  table(s, [
    ["순서", "작업"],
    ["1", "run_c06_v004_recovery.ps1 기준으로 width sweep 명령 구성"],
    ["2", "32/64/128 force/soft recovery fresh stamp 실행"],
    ["3", "backpressure stress를 width별로 확장"],
    ["4", "polygon reserve sweep 결과 문서화"],
    ["5", "Hit[16], sticky clear, lane imbalance 계약 보완"],
    ["6", "C01~C04 marker audit 결과 기록"],
    ["7", "v006 Result에서 Plan ID별 close/partial/open 판정"],
  ], 1.0, 1.45, [1.0, 10.0], 0.55, 8.2);
  footer(s, "다음 산출물: Code_Fix_Result_v006 + 필요한 xsim session archive");
}

pptx.writeFile({ fileName: out });
