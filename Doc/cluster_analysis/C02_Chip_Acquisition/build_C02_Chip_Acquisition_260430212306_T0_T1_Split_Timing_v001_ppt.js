const pptxgen = require("pptxgenjs");
const path = require("path");

const pptx = new pptxgen();
pptx.layout = "LAYOUT_WIDE";
pptx.author = "Codex";
pptx.company = "OpenAI";
pptx.subject = "C02 T0/T1 split timing";
pptx.title = "C02 T0/T1 Split Timing v001";
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
    fontSize: opts.fontSize || 11,
    bold: !!opts.bold,
    color: opts.color || C.ink,
    align: opts.align || "center",
    valign: "mid",
    margin: 0.02,
    fit: "shrink",
    breakLine: false,
  });
}

function arrow(slide, x, y, w, h, color = C.slate) {
  slide.addShape(pptx.ShapeType.rightArrow, {
    x, y, w, h,
    fill: { color },
    line: { color },
  });
}

function line(slide, x, y, w, h, color = C.line, width = 1) {
  slide.addShape(pptx.ShapeType.line, {
    x, y, w, h,
    line: { color, width },
  });
}

function metric(slide, x, y, title, value, note, fill, color = C.ink) {
  box(slide, x, y, 2.85, 1.25, "", fill, { line: C.line });
  text(slide, x + 0.18, y + 0.15, 2.45, 0.26, title, {
    fontSize: 9.5, color: C.muted, bold: true, align: "center",
  });
  text(slide, x + 0.18, y + 0.43, 2.45, 0.42, value, {
    fontSize: 18, color, bold: true, align: "center",
  });
  text(slide, x + 0.18, y + 0.88, 2.45, 0.20, note, {
    fontSize: 8.5, color: C.muted, align: "center",
  });
}

// Slide 1
{
  const s = pptx.addSlide();
  header(s, "C02 T0/T1 분리 Timing", "2026-04-30 21:23:06 KST | 기준: TDC-GPX Datasheet + C01 bus timing 계약 + xsim");
  text(s, 0.70, 1.35, 6.0, 0.48, "핵심 판단", { fontSize: 18, bold: true });
  text(s, 0.70, 1.95, 5.9, 1.18,
    "기존 T0 -> T1 = 40clk는 내부 첫 데이터 준비 시간이 아니라 downstream accepted 기준 값이다.\n첫 raw data는 T0+16clk에 이미 valid 상태가 된다.",
    { fontSize: 15, color: C.ink });
  metric(s, 7.05, 1.42, "T1a", "16 clk", "first raw valid", C.greenSoft, C.green);
  metric(s, 10.00, 1.42, "T1b", "40 clk", "first raw accepted", C.amberSoft, C.amber);
  box(s, 0.72, 4.05, 3.55, 1.0, "Datasheet 기준\nGPX 물리 READ timing", C.blueSoft, { line: C.blue, fontSize: 13, bold: true, color: C.blue });
  arrow(s, 4.48, 4.25, 0.55, 0.42, C.slate);
  box(s, 5.25, 4.05, 3.35, 1.0, "C02 내부\nfirst raw valid", C.greenSoft, { line: C.green, fontSize: 13, bold: true, color: C.green });
  arrow(s, 8.82, 4.25, 0.55, 0.42, C.slate);
  box(s, 9.60, 4.05, 2.95, 1.0, "AXI ready 영향\nfirst accepted", C.amberSoft, { line: C.amber, fontSize: 13, bold: true, color: C.amber });
  text(s, 0.72, 6.35, 11.8, 0.35, "문서 반영: T1은 더 이상 단일 의미로 사용하지 않고 T1a/T1b로 분리 표기한다.", { fontSize: 12, color: C.slate });
}

// Slide 2
{
  const s = pptx.addSlide();
  header(s, "Timing Block", "T0 기준 상대 clock | 200 MHz capture clock");
  const y = 2.25;
  box(s, 0.70, y, 1.55, 0.74, "T0\nIrFlag", C.slateSoft, { fontSize: 12, bold: true });
  arrow(s, 2.45, y + 0.18, 0.55, 0.36);
  box(s, 3.20, y, 1.75, 0.74, "T1a\n16clk valid", C.greenSoft, { line: C.green, fontSize: 12, bold: true, color: C.green });
  arrow(s, 5.15, y + 0.18, 0.55, 0.36, C.amber);
  box(s, 5.95, y, 2.10, 0.74, "ready=0\n24clk hold", C.amberSoft, { line: C.amber, fontSize: 12, bold: true, color: C.amber });
  arrow(s, 8.25, y + 0.18, 0.55, 0.36, C.amber);
  box(s, 9.05, y, 1.95, 0.74, "T1b\n40clk accept", C.amberSoft, { line: C.amber, fontSize: 12, bold: true, color: C.amber });
  arrow(s, 11.18, y + 0.18, 0.55, 0.36);
  box(s, 11.95, y, 0.75, 0.74, "...", C.white, { fontSize: 14, bold: true });

  line(s, 0.85, 4.05, 11.2, 0, C.line, 1.2);
  line(s, 0.85, 3.88, 0, 0.34, C.line, 1.2);
  line(s, 3.82, 3.88, 0, 0.34, C.green, 1.5);
  line(s, 5.55, 3.88, 0, 0.34, C.amber, 1.5);
  line(s, 10.00, 3.88, 0, 0.34, C.amber, 1.5);
  text(s, 0.55, 4.35, 0.9, 0.25, "T0", { fontSize: 9, align: "center", color: C.muted });
  text(s, 3.32, 4.35, 1.0, 0.25, "+16", { fontSize: 9, align: "center", color: C.green, bold: true });
  text(s, 5.02, 4.35, 1.1, 0.25, "+10 low", { fontSize: 9, align: "center", color: C.amber, bold: true });
  text(s, 9.42, 4.35, 1.15, 0.25, "+40 release", { fontSize: 9, align: "center", color: C.amber, bold: true });
  text(s, 1.05, 5.25, 10.9, 0.7,
    "[16b] 조건에서는 ready가 T0+10clk부터 low이고, T0+40clk에 release된다. 데이터 valid는 T0+16clk에 이미 올라오므로 accepted가 24clk 밀린다.",
    { fontSize: 13, color: C.ink, align: "center" });
}

// Slide 3
{
  const s = pptx.addSlide();
  header(s, "측정값 비교", "xsim_chip_ctrl.log:921, 987..991, 1006");
  metric(s, 0.70, 1.45, "[16a] first_valid", "16 clk", "backpressure 없음", C.greenSoft, C.green);
  metric(s, 3.85, 1.45, "[16a] first_accept", "16 clk", "valid_to_accept=0", C.greenSoft, C.green);
  metric(s, 7.00, 1.45, "[16b] first_accept", "40 clk", "ready release와 동일", C.amberSoft, C.amber);
  metric(s, 10.15, 1.45, "output_done", "428 clk", "[16a]/[16b] 동일", C.blueSoft, C.blue);
  box(s, 0.90, 3.55, 3.55, 1.12, "Latency\nT1a=16clk\nT1b=40clk", C.white, { fontSize: 13, bold: true });
  box(s, 4.90, 3.55, 3.55, 1.12, "Throughput\n56 words / 428clk\n전체 평균 유지", C.white, { fontSize: 13, bold: true });
  box(s, 8.90, 3.55, 3.55, 1.12, "II\nII_min=1clk\nII_max=15clk", C.white, { fontSize: 13, bold: true });
  text(s, 0.85, 5.45, 11.8, 0.58,
    "초기 accepted latency는 backpressure에 의해 달라지지만, raw FIFO/hold 경계가 흡수하여 전체 drain 완료 시점은 변하지 않았다.",
    { fontSize: 13, color: C.ink, align: "center" });
}

// Slide 4
{
  const s = pptx.addSlide();
  header(s, "근거와 후속 규칙", "추적 대상: TB 코드, xsim 로그, C02 Markdown");
  box(s, 0.80, 1.35, 3.60, 1.08, "코드 근거\nTB [16a]/[16b]\nline 1720..2064", C.slateSoft, { fontSize: 12.5, bold: true });
  box(s, 4.85, 1.35, 3.60, 1.08, "시뮬레이션 근거\nALL TESTS PASSED\ntotal_raw_words=795", C.greenSoft, { line: C.green, fontSize: 12.5, bold: true, color: C.green });
  box(s, 8.90, 1.35, 3.60, 1.08, "문서 근거\nT0/T1 Split Timing\nv001", C.blueSoft, { line: C.blue, fontSize: 12.5, bold: true, color: C.blue });
  text(s, 0.95, 3.25, 11.3, 1.25,
    "앞으로 C02 timing 결과에서 '첫 데이터'는 반드시 T1a(first valid)와 T1b(first accepted)를 분리한다.\nDatasheet 기준의 물리 READ timing은 C01 bus timing 계약으로 추적하고, C02 stream timing은 AXI ready 상태를 함께 기록한다.",
    { fontSize: 14, color: C.ink, align: "center" });
  text(s, 0.95, 5.50, 11.3, 0.42,
    "다음 PPT/Markdown 결과: Timing Diagram 또는 Timing Block에 T1a/T1b를 함께 표기",
    { fontSize: 12.5, color: C.slate, align: "center", bold: true });
}

async function main() {
  const out = path.join(__dirname, "C02_Chip_Acquisition_260430212306_T0_T1_Split_Timing_v001.pptx");
  await pptx.writeFile({ fileName: out });
  console.log(out);
}

main().catch((err) => {
  console.error(err);
  process.exit(1);
});
