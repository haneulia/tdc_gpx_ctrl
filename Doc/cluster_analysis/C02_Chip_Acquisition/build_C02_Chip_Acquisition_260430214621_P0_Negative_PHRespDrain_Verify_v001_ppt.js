const pptxgen = require("pptxgenjs");
const path = require("path");

const pptx = new pptxgen();
pptx.layout = "LAYOUT_WIDE";
pptx.author = "Codex";
pptx.company = "OpenAI";
pptx.subject = "C02 P0 negative and PH_RESP_DRAIN verification";
pptx.title = "C02 P0 Negative / PH_RESP_DRAIN Verify v001";
pptx.lang = "ko-KR";
pptx.theme = {
  headFontFace: "Malgun Gothic",
  bodyFontFace: "Malgun Gothic",
  lang: "ko-KR",
};
pptx.defineLayout({ name: "LAYOUT_WIDE", width: 13.333, height: 7.5 });
pptx.margin = 0;

const C = {
  bg: "F8FAFC",
  ink: "172033",
  muted: "566174",
  line: "CBD5E1",
  white: "FFFFFF",
  blue: "2563EB",
  blueSoft: "DBEAFE",
  green: "059669",
  greenSoft: "D1FAE5",
  amber: "D97706",
  amberSoft: "FEF3C7",
  red: "DC2626",
  redSoft: "FEE2E2",
  slate: "334155",
  slateSoft: "E2E8F0",
};

function header(slide, title, sub) {
  slide.background = { color: C.bg };
  slide.addText(title, {
    x: 0.45, y: 0.22, w: 12.35, h: 0.42,
    fontFace: "Malgun Gothic", fontSize: 18, bold: true,
    color: C.ink, margin: 0, fit: "shrink",
  });
  slide.addText(sub, {
    x: 0.46, y: 0.68, w: 12.2, h: 0.28,
    fontFace: "Malgun Gothic", fontSize: 8.5,
    color: C.muted, margin: 0, fit: "shrink",
  });
  slide.addShape(pptx.ShapeType.line, {
    x: 0.45, y: 1.03, w: 12.42, h: 0,
    line: { color: C.line, width: 0.8 },
  });
}

function text(slide, x, y, w, h, value, opts = {}) {
  slide.addText(value, {
    x, y, w, h,
    fontFace: "Malgun Gothic",
    fontSize: opts.fontSize || 11,
    bold: !!opts.bold,
    color: opts.color || C.ink,
    align: opts.align || "left",
    valign: opts.valign || "top",
    margin: 0,
    fit: "shrink",
    breakLine: false,
  });
}

function box(slide, x, y, w, h, value, fill, opts = {}) {
  slide.addShape(pptx.ShapeType.roundRect, {
    x, y, w, h,
    rectRadius: 0.04,
    fill: { color: fill },
    line: { color: opts.line || C.line, width: opts.lineWidth || 1 },
  });
  slide.addText(value, {
    x: x + 0.12, y: y + 0.08, w: w - 0.24, h: h - 0.16,
    fontFace: "Malgun Gothic",
    fontSize: opts.fontSize || 10.5,
    bold: !!opts.bold,
    color: opts.color || C.ink,
    align: opts.align || "center",
    valign: "mid",
    margin: 0,
    fit: "shrink",
  });
}

function arrow(slide, x1, y1, x2, y2, color = C.slate) {
  slide.addShape(pptx.ShapeType.line, {
    x: x1, y: y1, w: x2 - x1, h: y2 - y1,
    line: { color, width: 1.4, beginArrowType: "none", endArrowType: "triangle" },
  });
}

function pill(slide, x, y, w, value, fill, color = C.ink) {
  box(slide, x, y, w, 0.42, value, fill, { color, fontSize: 9.5, bold: true });
}

{
  const s = pptx.addSlide();
  header(s, "C02 P0 검증 결론", "기준: Datasheet / Code Fix Plan Open Items v001 / 2026-04-30 21:46 KST");
  box(s, 0.7, 1.45, 3.75, 1.4, "OP-C02-01\nNegative monitor evidence\nPASS", C.greenSoft, { line: C.green, bold: true });
  box(s, 4.78, 1.45, 3.75, 1.4, "OP-C02-02\nPH_RESP_DRAIN fatal recovery\nPASS", C.greenSoft, { line: C.green, bold: true });
  box(s, 8.86, 1.45, 3.75, 1.4, "Positive regression\nALL TESTS PASSED\n796 raw words", C.greenSoft, { line: C.green, bold: true });
  text(s, 0.85, 3.35, 11.7, 0.75,
    "핵심 수정: fatal latch 이후에는 기존 PH_IDLE 정상 탈출을 차단하고, bus-idle stable window를 만족한 뒤 PH_INIT으로 재진입하도록 경로를 단일화했다.",
    { fontSize: 13, bold: true });
  text(s, 0.85, 4.45, 11.7, 0.5,
    "주의: Vivado xsim은 VHDL severity failure 로그가 있어도 process exit code를 0으로 반환했다. CI는 로그 스캔 wrapper를 exit-code 계약으로 사용해야 한다.",
    { fontSize: 11, color: C.red, bold: true });
  pill(s, 0.85, 5.4, 3.7, "RTL: tdc_gpx_chip_ctrl.vhd:839, :882, :905..919", C.blueSoft, C.blue);
  pill(s, 4.8, 5.4, 3.7, "TB: tb_tdc_gpx_chip_ctrl.vhd:52, :839..877", C.blueSoft, C.blue);
  pill(s, 8.75, 5.4, 3.7, "Logs: xsim_chip_ctrl*.log", C.blueSoft, C.blue);
}

{
  const s = pptx.addSlide();
  header(s, "PH_RESP_DRAIN Fatal Recovery", "fatal 이후 PH_IDLE 즉시 탈출 금지, stable window 이후 PH_INIT 재진입");
  const y = 2.0;
  box(s, 0.55, y, 1.65, 0.8, "PH_RUN\n완료", C.white);
  arrow(s, 2.2, y + 0.4, 2.78, y + 0.4);
  box(s, 2.78, y, 2.05, 0.8, "PH_RESP_DRAIN\nstale response drain", C.blueSoft, { line: C.blue });
  arrow(s, 4.83, y + 0.4, 5.38, y + 0.4);
  box(s, 5.38, y, 1.9, 0.8, "hard cap\nquarantine", C.amberSoft, { line: C.amber });
  arrow(s, 7.28, y + 0.4, 7.86, y + 0.4);
  box(s, 7.86, y, 1.75, 0.8, "bus fatal\nsticky", C.redSoft, { line: C.red });
  arrow(s, 9.61, y + 0.4, 10.16, y + 0.4);
  box(s, 10.16, y, 2.15, 0.8, "idle stable\n16clk TB / 4096clk RTL", C.greenSoft, { line: C.green });
  arrow(s, 11.25, y + 0.82, 11.25, 3.3, C.green);
  box(s, 10.16, 3.3, 2.15, 0.8, "PH_INIT\nforce_reinit sticky", C.greenSoft, { line: C.green, bold: true });
  text(s, 0.75, 4.75, 11.8, 0.8,
    "xsim 근거: [19] drain_done -> hard cap -> bus fatal -> idle stable force_reinit -> idle recovery 순서가 모두 PASS.",
    { fontSize: 12, bold: true });
  text(s, 0.75, 5.6, 11.8, 0.6,
    "로그: xsim_chip_ctrl.log:1009, :1010, :1338, :1340, :1341, :1343",
    { fontSize: 10.5, color: C.muted });
}

{
  const s = pptx.addSlide();
  header(s, "Negative Monitor Evidence", "기능 경로와 감시망 검증을 분리하여 실패 전파를 확인");
  box(s, 0.85, 1.45, 5.45, 1.15, "N1 empty IFIFO read monitor\nforced monitor violation", C.redSoft, { line: C.red, bold: true });
  box(s, 7.03, 1.45, 5.45, 1.15, "N2 raw tuser monitor\nforced sideband violation", C.redSoft, { line: C.red, bold: true });
  arrow(s, 3.58, 2.6, 3.58, 3.25, C.red);
  arrow(s, 9.75, 2.6, 9.75, 3.25, C.red);
  box(s, 1.35, 3.25, 4.45, 0.9, "monitor PASS\n위반 감지", C.greenSoft, { line: C.green });
  box(s, 7.53, 3.25, 4.45, 0.9, "monitor PASS\n위반 감지", C.greenSoft, { line: C.green });
  arrow(s, 3.58, 4.15, 3.58, 4.85, C.red);
  arrow(s, 9.75, 4.15, 9.75, 4.85, C.red);
  box(s, 1.35, 4.85, 4.45, 0.9, "intentional failure\nFailure: C02 negative evidence", C.redSoft, { line: C.red });
  box(s, 7.53, 4.85, 4.45, 0.9, "intentional failure\nFailure: C02 negative evidence", C.redSoft, { line: C.red });
  text(s, 0.8, 6.25, 11.8, 0.55,
    "N1/N2 xsim 반환값은 0이었다. CI에서는 로그 wrapper가 해당 Failure 패턴을 발견하면 exit 1을 반환하도록 운용한다.",
    { fontSize: 10.5, color: C.red, bold: true });
}

{
  const s = pptx.addSlide();
  header(s, "Latency / Throughput / Pipeline / II", "정상 데이터 경로 영향과 fatal recovery 영향 분리");
  const rows = [
    ["Latency", "정상 drain 영향 없음. fatal recovery에서만 stable window 추가."],
    ["Throughput", "정상 I-Mode single throughput 영향 없음. fatal 중 신규 shot 차단."],
    ["Pipeline", "PH_RESP_DRAIN exit이 fatal=0 normal / fatal=1 auto-reinit로 분리."],
    ["II", "정상 shot II 영향 없음. recovery 상태는 측정 파이프라인 밖으로 격리."],
  ];
  rows.forEach((r, i) => {
    const y = 1.45 + i * 1.05;
    box(s, 0.8, y, 2.15, 0.72, r[0], C.slateSoft, { line: C.slate, bold: true });
    box(s, 3.15, y, 9.25, 0.72, r[1], C.white, { align: "left", fontSize: 10.5 });
  });
  text(s, 0.85, 6.05, 11.7, 0.55,
    "남은 open: OP-C02-03 top/config expected CDC, OP-C02-04 downstream tuser boundary, OP-C02-05 illegal timing matrix, OP-C02-06 stale ready negative.",
    { fontSize: 10.5, color: C.muted });
}

const out = path.join(__dirname, "C02_Chip_Acquisition_260430214621_P0_Negative_PHRespDrain_Verify_v001.pptx");
pptx.writeFile({ fileName: out });
