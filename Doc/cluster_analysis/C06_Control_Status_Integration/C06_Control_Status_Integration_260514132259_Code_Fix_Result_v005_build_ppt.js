const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/.pnpm/pptxgenjs@4.0.1/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C06_Control_Status_Integration/C06_Control_Status_Integration_260514132259_Code_Fix_Result_v005.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C06 Code Fix Result v005";
pptx.title = "C06 Code Fix Result v005";
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
  dark: "101820",
  blue: "2563EB",
  blueFill: "DBEAFE",
  green: "15803D",
  greenFill: "DCFCE7",
  orange: "C2410C",
  orangeFill: "FFEDD5",
  red: "B91C1C",
  redFill: "FEE2E2",
  purple: "6D28D9",
  purpleFill: "EDE9FE",
  teal: "0F766E",
  tealFill: "CCFBF1",
  slate: "E2E8F0",
};

function tx(slide, value, x, y, w, h, opt = {}) {
  slide.addText(value, {
    x, y, w, h,
    fontFace: font,
    fontSize: opt.size || 10,
    bold: !!opt.bold,
    color: opt.color || C.ink,
    align: opt.align || "left",
    valign: opt.valign || "mid",
    fit: "shrink",
    margin: opt.margin === undefined ? 0.04 : opt.margin,
    breakLine: false,
  });
}

function title(slide, main, sub) {
  slide.background = { color: C.bg };
  tx(slide, main, 0.55, 0.28, 12.1, 0.46, { size: 21, bold: true });
  tx(slide, sub, 0.58, 0.78, 12.0, 0.3, { size: 9.2, color: C.muted });
  slide.addShape(pptx.ShapeType.line, {
    x: 0.58, y: 1.13, w: 12.1, h: 0,
    line: { color: C.line, width: 0.8 },
  });
}

function footer(slide, value) {
  tx(slide, value, 0.58, 7.05, 12.15, 0.24, {
    size: 7.2, color: C.muted, align: "center",
  });
}

function box(slide, label, x, y, w, h, opt = {}) {
  slide.addShape(pptx.ShapeType.roundRect, {
    x, y, w, h,
    rectRadius: 0.04,
    fill: { color: opt.fill || C.white },
    line: { color: opt.line || C.line, width: opt.width || 0.9 },
  });
  tx(slide, label, x + 0.08, y + 0.06, w - 0.16, h - 0.12, {
    size: opt.size || 9.2,
    bold: opt.bold,
    color: opt.color || C.ink,
    align: opt.align || "center",
  });
}

function arrow(slide, x1, y1, x2, y2, color = C.muted, width = 1.25) {
  slide.addShape(pptx.ShapeType.line, {
    x: x1, y: y1, w: x2 - x1, h: y2 - y1,
    line: { color, width, endArrowType: "triangle" },
  });
}

function table(slide, rows, x, y, widths, rowH, size = 7.6) {
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
  tx(s, "C06", 0.72, 0.58, 1.2, 0.42, { size: 15, bold: true, color: "93C5FD" });
  tx(s, "Code Fix Plan v005 실행 결과", 0.72, 1.12, 10.8, 0.58, { size: 26, bold: true, color: C.white });
  tx(s, "C06 recovery/control/status 계약을 공식 회귀, handoff, 완료도 판단으로 고정했습니다.", 0.76, 1.82, 11.2, 0.42, { size: 13, color: "CBD5E1" });
  box(s, "GO\nWITH CONTRACT", 0.88, 3.08, 2.55, 1.05, { fill: "14532D", line: "86EFAC", color: C.white, bold: true, size: 13 });
  box(s, "PASS\nv004 recovery", 3.88, 3.08, 2.55, 1.05, { fill: "1E3A8A", line: "93C5FD", color: C.white, bold: true, size: 12 });
  box(s, "KEEP\nsim reports", 6.88, 3.08, 2.55, 1.05, { fill: "713F12", line: "FDE68A", color: C.white, bold: true, size: 12 });
  box(s, "NEXT\nC07 / release", 9.88, 3.08, 2.55, 1.05, { fill: "4C1D95", line: "C4B5FD", color: C.white, bold: true, size: 12 });
  tx(s, "공식 archive: sim_results/vivado_xsim/sessions/260514132259_c06_v004_recovery/", 0.78, 5.52, 11.8, 0.35, { size: 10.2, color: C.white });
  footer(s, "C06 Result v005 | 생성 2026-05-14 13:22:59 KST | 수정 13:30:33 KST");
}

{
  const s = pptx.addSlide();
  title(s, "Plan v005 Closure", "FP5-C06-01부터 05까지 모두 Closed 또는 Accepted로 처리했습니다.");
  table(s, [
    ["Plan ID", "목표", "결과"],
    ["FP5-01", "C06 handoff 문서 갱신", "Closed"],
    ["FP5-02", "공식 recovery regression 지정", "Closed: run_c06_v004_recovery.ps1"],
    ["FP5-03", "simulation-only report 정책", "Accepted: 유지"],
    ["FP5-04", "C06 완료도 체크 갱신", "Closed"],
    ["FP5-05", "다음 cluster 진입 판단", "GO_WITH_CONTRACT"],
  ], 0.82, 1.58, [1.25, 6.2, 4.55], 0.58, 8.1);
  box(s, "v005는 RTL 기능 변경이 아니라, v004에서 닫힌 recovery 계약을 공식 운영 계약으로 고정하는 단계입니다.", 1.05, 5.55, 11.2, 0.70, { fill: C.blueFill, line: C.blue, bold: true, size: 11.2 });
  footer(s, "근거: C06_Control_Status_Integration_260514115000_Code_Fix_Plan_v005.md section 7");
}

{
  const s = pptx.addSlide();
  title(s, "공식 회귀 Evidence", "fresh compile/elab/xsim 후 세션 archive로 38개 artifact가 이동되었습니다.");
  table(s, [
    ["검증", "로그", "PASS line"],
    ["face_seq", "xsim_c06_v002_face_seq_260514132259.log", "54"],
    ["status_agg", "xsim_c06_v002_status_agg_260514132259.log", "34"],
    ["width 32/64/128", "xsim_c06_v002_top_int_w*.log", "143"],
    ["64-bit backpressure", "xsim_c06_v002_top_int_w64_bp_260514132259.log", "143,145"],
    ["force recovery", "xsim_c06_v004_top_int_force_260514132259.log", "236,238"],
    ["soft recovery", "xsim_c06_v004_top_int_soft_260514132259.log", "244,246"],
  ], 0.68, 1.45, [2.0, 6.75, 2.65], 0.52, 7.1);
  tx(s, "명령", 0.82, 5.35, 0.9, 0.25, { size: 10, bold: true });
  box(s, "powershell -NoProfile -ExecutionPolicy Bypass -File scripts\\run_c06_v004_recovery.ps1 -Stamp 260514132259", 1.65, 5.22, 10.95, 0.52, { fill: C.white, line: C.line, size: 8.9 });
  footer(s, "archive session: sim_results/vivado_xsim/sessions/260514132259_c06_v004_recovery/");
}

{
  const s = pptx.addSlide();
  title(s, "Recovery Timing", "200 MHz 기준 source pulse에서 TDC pulse까지 20 ns / 4 clk로 관측되었습니다.");
  const xs = [0.78, 2.95, 5.12, 7.29, 9.46];
  const labels = [
    ["T8 marker\n43.5875 us", C.slate, C.muted],
    ["source pulse\n43.6575 us", C.orangeFill, C.orange],
    ["TDC pulse\n43.6775 us", C.blueFill, C.blue],
    ["chip action\n43.6775 us", C.purpleFill, C.purple],
    ["run #2 PASS", C.greenFill, C.green],
  ];
  labels.forEach(([label, fill, line], i) => {
    box(s, label, xs[i], 2.0, 1.55, 0.95, { fill, line, bold: true, size: 8.3 });
    if (i < labels.length - 1) arrow(s, xs[i] + 1.60, 2.47, xs[i + 1] - 0.05, 2.47);
  });
  table(s, [
    ["Metric", "force", "soft", "의미"],
    ["source -> TDC", "20 ns", "20 ns", "CDC latency"],
    ["soft TDC -> PH_INIT", "-", "20 ns", "idle drain 후 PH_INIT"],
    ["T8 -> run2 T0", "14.965 us", "74.565 us", "TB sequence 포함"],
  ], 0.85, 4.03, [2.2, 2.1, 2.1, 5.0], 0.50, 7.7);
  footer(s, "근거: force log line 135/142/146/148/236, soft log line 135/142/146/148/156/244");
}

{
  const s = pptx.addSlide();
  title(s, "Pipeline 계약", "Recovery는 run 내부 payload를 바꾸지 않고 run-to-run control/status boundary를 닫습니다.");
  const nodes = [
    ["AXI CSR\nsoft/force", 0.78, C.orangeFill, C.orange],
    ["config_ctrl\nsource toggle", 2.78, C.blueFill, C.blue],
    ["3-stage sync\nTDC pulse", 4.98, C.blueFill, C.blue],
    ["chip_ctrl\nPH_INIT", 7.18, C.purpleFill, C.purple],
    ["face/status\nreset", 9.38, C.tealFill, C.teal],
    ["next run\nstream PASS", 11.18, C.greenFill, C.green],
  ];
  nodes.forEach(([label, x, fill, line], i) => {
    box(s, label, x, 1.92, 1.35, 0.90, { fill, line, bold: true, size: 7.8 });
    if (i < nodes.length - 1) arrow(s, x + 1.38, 2.37, nodes[i + 1][1] - 0.05, 2.37);
  });
  table(s, [
    ["Boundary", "상태"],
    ["AXI -> TDC command CDC", "toggle bridge로 close"],
    ["chip_ctrl phase", "register phase로 close"],
    ["face/status recovery", "top-level recovery reset으로 close"],
    ["output stream", "force/soft recovery 모두 beats/tlast PASS"],
  ], 1.0, 3.65, [3.2, 7.7], 0.50, 8.0);
  footer(s, "근거: tdc_gpx_config_ctrl.vhd, tdc_gpx_chip_ctrl.vhd, tdc_gpx_top.vhd");
}

{
  const s = pptx.addSlide();
  title(s, "다음 단계 진입 판단", "C06 내부 blocker는 닫혔고, 다음 단계는 시스템 소비자 계약을 수락하는 일입니다.");
  box(s, "C06 Scope\nI-Mode single only", 0.85, 1.60, 2.5, 0.90, { fill: C.blueFill, line: C.blue, bold: true });
  box(s, "Output Width\n32/64/128 baseline", 3.75, 1.60, 2.5, 0.90, { fill: C.greenFill, line: C.green, bold: true });
  box(s, "Recovery\n64-bit force/soft PASS", 6.65, 1.60, 2.5, 0.90, { fill: C.greenFill, line: C.green, bold: true });
  box(s, "System Risk\nVDMA/PS reserve", 9.55, 1.60, 2.5, 0.90, { fill: C.orangeFill, line: C.orange, bold: true });
  table(s, [
    ["남은 항목", "다음 단계 처리"],
    ["VDMA/PS/Ethernet 8 us reserve", "system measurement 또는 reserve sweep"],
    ["recovery width sweep 32/128", "필요 시 C07에서 추가"],
    ["Hit[16] final output", "현재 generation에서는 출력하지 않음"],
    ["simulation-only report", "유지. release 로그 정책 변경은 별도 plan"],
  ], 0.92, 3.35, [3.9, 7.6], 0.54, 8.0);
  footer(s, "결론: C06 -> 다음 단계 GO_WITH_CONTRACT");
}

pptx.writeFile({ fileName: out });
