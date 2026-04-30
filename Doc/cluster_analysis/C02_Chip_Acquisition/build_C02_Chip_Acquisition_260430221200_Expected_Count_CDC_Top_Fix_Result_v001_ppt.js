const pptxgen = require("pptxgenjs");
const path = require("path");

const pptx = new pptxgen();
pptx.layout = "LAYOUT_WIDE";
pptx.author = "Codex";
pptx.subject = "C02 expected-count CDC/top integration fix";
pptx.title = "C02 Expected Count CDC / Top Fix Result v001";
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
  muted: "5B667A",
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
    x: 0.45, y: 0.24, w: 12.35, h: 0.42,
    fontFace: "Malgun Gothic", fontSize: 18, bold: true,
    color: C.ink, margin: 0, fit: "shrink",
  });
  slide.addText(sub, {
    x: 0.46, y: 0.7, w: 12.2, h: 0.28,
    fontFace: "Malgun Gothic", fontSize: 8.5,
    color: C.muted, margin: 0, fit: "shrink",
  });
  slide.addShape(pptx.ShapeType.line, {
    x: 0.45, y: 1.04, w: 12.42, h: 0,
    line: { color: C.line, width: 0.8 },
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

function label(slide, x, y, w, h, value, opts = {}) {
  slide.addText(value, {
    x, y, w, h,
    fontFace: "Malgun Gothic",
    fontSize: opts.fontSize || 10,
    bold: !!opts.bold,
    color: opts.color || C.ink,
    align: opts.align || "left",
    valign: opts.valign || "top",
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

function chip(slide, x, y, txt, fill = C.white) {
  box(slide, x, y, 2.05, 0.72, txt, fill, { fontSize: 9.3 });
}

// Slide 1
{
  const s = pptx.addSlide();
  header(s, "C02 Expected Count CDC / Top 통합 보완", "생성: 2026-04-30 22:12 KST | 기준: Datasheet I-Mode single 운용 흐름");
  label(s, 0.75, 1.45, 6.0, 0.4, "핵심 판단", { fontSize: 16, bold: true });
  box(s, 0.75, 2.05, 5.7, 1.15, "기존 3개 CDC는 IFIFO1 / IFIFO2 / final_valid가 서로 다른 snapshot으로 섞일 수 있음", C.redSoft, { line: C.red, fontSize: 13 });
  box(s, 0.75, 3.55, 5.7, 1.15, "보완: 65-bit 단일 tuple CDC로 chip_run ST_DRAIN_LATCH 입력을 원자화", C.greenSoft, { line: C.green, fontSize: 13 });
  box(s, 7.05, 2.05, 5.25, 2.65, "검증 결과\nconfig_ctrl 통합: data=16, ctrl=8 PASS\nchip_ctrl 회귀: ALL TESTS PASSED", C.blueSoft, { line: C.blue, fontSize: 16, bold: true });
}

// Slide 2
{
  const s = pptx.addSlide();
  header(s, "문제 구조와 보완 구조", "chip_run은 세 값을 같은 사이클에 래치하므로 CDC도 하나의 논리 tuple이어야 함");
  label(s, 0.65, 1.35, 5.8, 0.3, "기존", { bold: true, color: C.red });
  chip(s, 0.7, 2.0, "expected_ififo1\n32b CDC", C.redSoft);
  chip(s, 0.7, 3.05, "expected_ififo2\n32b CDC", C.redSoft);
  chip(s, 0.7, 4.1, "final_valid\n1b CDC", C.redSoft);
  box(s, 3.55, 2.55, 2.35, 1.35, "ST_DRAIN_LATCH\n동시 래치", C.white);
  arrow(s, 2.75, 2.36, 3.55, 3.0, C.red);
  arrow(s, 2.75, 3.41, 3.55, 3.2, C.red);
  arrow(s, 2.75, 4.46, 3.55, 3.4, C.red);
  label(s, 0.7, 5.35, 5.2, 0.6, "위험: count와 final qualifier가 서로 다른 update일 수 있음", { color: C.red, fontSize: 11 });

  label(s, 7.05, 1.35, 5.8, 0.3, "보완", { bold: true, color: C.green });
  box(s, 7.1, 2.25, 2.8, 1.2, "[31:0] IFIFO1\n[63:32] IFIFO2\n[64] final", C.greenSoft, { line: C.green, fontSize: 10.5 });
  box(s, 10.35, 2.25, 1.7, 1.2, "65b\nCDC", C.blueSoft, { line: C.blue, fontSize: 15, bold: true });
  box(s, 9.0, 4.25, 2.35, 1.2, "ST_DRAIN_LATCH\n동일 snapshot", C.white);
  arrow(s, 9.9, 2.85, 10.35, 2.85, C.green);
  arrow(s, 11.2, 3.45, 10.25, 4.25, C.green);
  label(s, 7.1, 5.35, 5.3, 0.6, "효과: count/final skew 제거, GPX read timing 영향 없음", { color: C.green, fontSize: 11 });
}

// Slide 3
{
  const s = pptx.addSlide();
  header(s, "Timing / Pipeline Block", "control path만 보강. GPX bus read II와 throughput은 변경하지 않음");
  const y = 2.55;
  chip(s, 0.6, y, "shot_start\nfire_count=1", C.slateSoft);
  chip(s, 2.95, y, "stop_evt\nrunning count", C.amberSoft);
  chip(s, 5.3, y, "fire_count\nfinal beat", C.amberSoft);
  chip(s, 7.65, y, "65b tuple\nCDC", C.blueSoft);
  chip(s, 10.0, y, "IrFlag\nST_DRAIN_LATCH", C.greenSoft);
  arrow(s, 2.65, y + 0.36, 2.95, y + 0.36);
  arrow(s, 5.0, y + 0.36, 5.3, y + 0.36);
  arrow(s, 7.35, y + 0.36, 7.65, y + 0.36);
  arrow(s, 9.7, y + 0.36, 10.0, y + 0.36);
  label(s, 0.75, 4.25, 11.9, 0.4, "운용 계약: final tuple은 IrFlag 이전에 TDC domain으로 도착해야 한다.", { fontSize: 13, bold: true });
  box(s, 0.75, 4.9, 3.4, 0.8, "Latency\ncontrol CDC latency 존재", C.white);
  box(s, 4.35, 4.9, 3.4, 0.8, "Throughput\nread data path 영향 없음", C.white);
  box(s, 7.95, 4.9, 3.4, 0.8, "II\nIFIFO read II 변경 없음", C.white);
}

// Slide 4
{
  const s = pptx.addSlide();
  header(s, "Verification Evidence", "xsim 로그 기반 closure");
  box(s, 0.85, 1.55, 5.6, 1.25, "config_ctrl 직접 통합 TB\n4 chip x IFIFO1/2 x 2 word\nraw data=16, ctrl=8", C.greenSoft, { line: C.green, fontSize: 14, bold: true });
  box(s, 6.9, 1.55, 5.6, 1.25, "chip_ctrl 회귀\nALL TESTS PASSED\ntotal_raw_words=796", C.blueSoft, { line: C.blue, fontSize: 14, bold: true });
  label(s, 0.95, 3.45, 11.6, 0.42, "추적 근거", { fontSize: 15, bold: true });
  box(s, 0.95, 4.1, 11.3, 1.1, "RTL: tdc_gpx_config_ctrl.vhd:682, 690, 1374, 1388, 1408\nTB: tb_tdc_gpx_config_ctrl.vhd:216, 464, 528, 610\nLog: xsim_config_expected_cdc.log:32 / xsim_chip_ctrl_regression.log:1343", C.white, { align: "left", fontSize: 11.2 });
}

// Slide 5
{
  const s = pptx.addSlide();
  header(s, "Closure / Next Items", "OP-C02-03은 code + TB + log evidence 기준 close");
  box(s, 0.8, 1.55, 3.75, 1.2, "닫힘\nOP-C02-03\nexpected-count tuple CDC", C.greenSoft, { line: C.green, fontSize: 14, bold: true });
  box(s, 4.8, 1.55, 3.75, 1.2, "유지\nDatasheet read timing\n변경 없음", C.blueSoft, { line: C.blue, fontSize: 14, bold: true });
  box(s, 8.8, 1.55, 3.75, 1.2, "후속\nfull_int echo packing\n별도 검토", C.amberSoft, { line: C.amber, fontSize: 14, bold: true });
  label(s, 0.9, 3.45, 11.6, 1.35, "남은 경계: echo_receiver stop_evt packing은 C02 stop_cfg_decode의 per-chip [IFIFO2|IFIFO1] layout과 별도 계약 검토가 필요하다. output stream CDC 전체 재설계는 C03/C04 경계 검토 항목으로 유지한다.", { fontSize: 13 });
}

const out = path.join(__dirname, "C02_Chip_Acquisition_260430221200_Expected_Count_CDC_Top_Fix_Result_v001.pptx");
pptx.writeFile({ fileName: out });
