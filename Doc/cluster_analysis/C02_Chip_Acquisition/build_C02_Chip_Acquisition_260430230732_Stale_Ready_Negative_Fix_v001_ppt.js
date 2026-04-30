const pptxgen = require("pptxgenjs");
const path = require("path");

const pptx = new pptxgen();
pptx.layout = "LAYOUT_WIDE";
pptx.author = "Codex";
pptx.subject = "C02 stale ready negative verification";
pptx.title = "C02 Stale Ready Negative Fix v001";
pptx.lang = "ko-KR";
pptx.theme = {
  headFontFace: "Malgun Gothic",
  bodyFontFace: "Malgun Gothic",
  lang: "ko-KR",
};
pptx.defineLayout({ name: "LAYOUT_WIDE", width: 13.333, height: 7.5 });

const C = {
  bg: "F8FAFC",
  ink: "172033",
  muted: "64748B",
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
  slateSoft: "E2E8F0",
};

function header(slide, title, sub) {
  slide.background = { color: C.bg };
  slide.addText(title, {
    x: 0.45, y: 0.22, w: 12.2, h: 0.45,
    fontFace: "Malgun Gothic", fontSize: 18, bold: true,
    color: C.ink, margin: 0, fit: "shrink",
  });
  slide.addText(sub, {
    x: 0.47, y: 0.72, w: 12.1, h: 0.28,
    fontFace: "Malgun Gothic", fontSize: 8.5,
    color: C.muted, margin: 0, fit: "shrink",
  });
  slide.addShape(pptx.ShapeType.line, {
    x: 0.45, y: 1.05, w: 12.42, h: 0,
    line: { color: C.line, width: 0.8 },
  });
}

function label(slide, x, y, w, h, text, opts = {}) {
  slide.addText(text, {
    x, y, w, h,
    fontFace: "Malgun Gothic",
    fontSize: opts.fontSize || 10,
    bold: !!opts.bold,
    color: opts.color || C.ink,
    align: opts.align || "left",
    valign: opts.valign || "top",
    margin: 0.03,
    fit: "shrink",
    breakLine: false,
  });
}

function box(slide, x, y, w, h, text, fill, opts = {}) {
  slide.addShape(pptx.ShapeType.roundRect, {
    x, y, w, h,
    rectRadius: 0.04,
    fill: { color: fill },
    line: { color: opts.line || C.line, width: opts.lineWidth || 1 },
  });
  label(slide, x + 0.12, y + 0.08, w - 0.24, h - 0.16, text, {
    fontSize: opts.fontSize || 10.5,
    bold: opts.bold,
    color: opts.color || C.ink,
    align: opts.align || "center",
    valign: "mid",
  });
}

function arrow(slide, x1, y1, x2, y2, color = C.ink) {
  slide.addShape(pptx.ShapeType.line, {
    x: x1, y: y1, w: x2 - x1, h: y2 - y1,
    line: { color, width: 1.35, endArrowType: "triangle" },
  });
}

function row(slide, y, a, b, c, fill = C.white) {
  box(slide, 0.75, y, 2.45, 0.48, a, fill, { fontSize: 9.3, bold: true });
  box(slide, 3.25, y, 4.35, 0.48, b, fill, { fontSize: 9.3, align: "left" });
  box(slide, 7.65, y, 4.95, 0.48, c, fill, { fontSize: 9.3, align: "left" });
}

{
  const s = pptx.addSlide();
  header(s, "C02 Stale Ready Negative 보완", "작성: 2026-04-30 23:07 KST | 기준: Datasheet 기반 C02 stream 보존 계약");
  box(s, 0.8, 1.55, 3.25, 1.05, "OP-C02-06\nstale ready negative", C.blueSoft, { line: C.blue, bold: true, fontSize: 14 });
  box(s, 5.05, 1.55, 3.25, 1.05, "skid buffer\n2 beat blocked PASS", C.greenSoft, { line: C.green, bold: true, fontSize: 13 });
  box(s, 9.15, 1.55, 3.25, 1.05, "sync FIFO\n6 beat blocked PASS", C.greenSoft, { line: C.green, bold: true, fontSize: 13 });
  arrow(s, 4.05, 2.08, 5.05, 2.08, C.green);
  arrow(s, 8.3, 2.08, 9.15, 2.08, C.green);
  label(s, 0.95, 3.45, 11.5, 0.5, "결론: downstream tready가 low일 때 registered-ready가 늦게 내려가도 elastic capacity 안에서만 수용되고, release 후 순서/중복/누락 없이 drain된다.", { fontSize: 14, bold: true, align: "center" });
  box(s, 1.1, 4.75, 5.15, 0.9, "위험\nstale ready로 초과 accept", C.redSoft, { line: C.red, fontSize: 13, bold: true });
  box(s, 7.1, 4.75, 5.15, 0.9, "검증\n수용 상한 + 순서 보존", C.greenSoft, { line: C.green, fontSize: 13, bold: true });
  arrow(s, 6.25, 5.2, 7.1, 5.2, C.green);
}

{
  const s = pptx.addSlide();
  header(s, "Timing Block", "registered-ready stale-high window와 capacity close 시점을 분리");
  box(s, 0.7, 2.1, 2.25, 0.72, "Source\nvalid/data", C.white, { fontSize: 11, bold: true });
  box(s, 3.65, 2.1, 2.45, 0.72, "Boundary\nskid/sync FIFO", C.blueSoft, { line: C.blue, fontSize: 11, bold: true });
  box(s, 6.9, 2.1, 2.35, 0.72, "Sink\ntready=0", C.redSoft, { line: C.red, fontSize: 11, bold: true });
  box(s, 10.1, 2.1, 2.35, 0.72, "Sink\ntready=1", C.greenSoft, { line: C.green, fontSize: 11, bold: true });
  arrow(s, 2.95, 2.46, 3.65, 2.46, C.ink);
  arrow(s, 6.1, 2.46, 6.9, 2.46, C.red);
  arrow(s, 9.25, 2.46, 10.1, 2.46, C.green);
  box(s, 1.0, 4.0, 2.25, 0.62, "beat0 accept", C.greenSoft, { line: C.green, fontSize: 10 });
  box(s, 3.55, 4.0, 2.25, 0.62, "beat1 accept", C.amberSoft, { line: C.amber, fontSize: 10 });
  box(s, 6.1, 4.0, 2.25, 0.62, "ready=0", C.redSoft, { line: C.red, fontSize: 10 });
  box(s, 8.65, 4.0, 2.25, 0.62, "release", C.greenSoft, { line: C.green, fontSize: 10 });
  box(s, 11.0, 4.0, 1.55, 0.62, "drain", C.greenSoft, { line: C.green, fontSize: 10 });
  arrow(s, 3.25, 4.31, 3.55, 4.31);
  arrow(s, 5.8, 4.31, 6.1, 4.31);
  arrow(s, 8.35, 4.31, 8.65, 4.31);
  arrow(s, 10.9, 4.31, 11.0, 4.31);
  label(s, 1.0, 5.55, 11.4, 0.45, "sync FIFO는 input skid 2 + core FIFO 2 + output skid 2 구조라 blocked 상태에서 최대 6 beat가 합법 수용 상한이다.", { fontSize: 13, bold: true, align: "center" });
}

{
  const s = pptx.addSlide();
  header(s, "검증 구조", "RTL 변경 없이 전용 testbench로 boundary만 격리 검증");
  box(s, 0.85, 1.55, 3.1, 1.0, "TB source\ncontinuous valid", C.white, { fontSize: 12, bold: true });
  box(s, 5.1, 1.55, 3.1, 1.0, "DUT\nskid / sync FIFO", C.blueSoft, { line: C.blue, fontSize: 12, bold: true });
  box(s, 9.35, 1.55, 3.1, 1.0, "TB sink\nready low then high", C.white, { fontSize: 12, bold: true });
  arrow(s, 3.95, 2.05, 5.1, 2.05);
  arrow(s, 8.2, 2.05, 9.35, 2.05);
  row(s, 3.25, "Check", "조건", "판단", C.slateSoft);
  row(s, 3.8, "blocked accept", "ready low 중 수용 수량 계수", "skid=2, fifo<=6");
  row(s, 4.35, "ready phase", "ready high/low 모두 관측", "stale window 실제 exercise");
  row(s, 4.9, "order", "release 후 데이터 증가 순서 확인", "drop/duplicate 차단");
  row(s, 5.45, "extra beat", "drain 후 valid=0 확인", "잔여 beat 없음");
}

{
  const s = pptx.addSlide();
  header(s, "Latency / Throughput / Pipeline / II", "이번 변경은 TB 추가이며 RTL timing path는 유지");
  row(s, 1.35, "Metric", "영향", "근거", C.slateSoft);
  row(s, 1.9, "Latency", "RTL 변화 없음", "전용 TB 추가만 수행");
  row(s, 2.45, "Throughput", "ready 유지 시 1 beat/clk", "skid/sync FIFO 기존 계약 유지");
  row(s, 3.0, "Pipeline", "registered boundary 검증", "source -> elastic boundary -> sink");
  row(s, 3.55, "II", "정상 ready 구간 II=1", "backpressure 구간은 sink ready가 제한");
  row(s, 4.1, "Negative", "초과 accept 없음", "xsim: blocked accepts=6");
  box(s, 1.2, 5.35, 3.1, 0.75, "skid\n2 slots", C.greenSoft, { line: C.green, fontSize: 13, bold: true });
  box(s, 5.1, 5.35, 3.1, 0.75, "core FIFO\n2 slots", C.blueSoft, { line: C.blue, fontSize: 13, bold: true });
  box(s, 9.0, 5.35, 3.1, 0.75, "out skid\n2 slots", C.greenSoft, { line: C.green, fontSize: 13, bold: true });
  arrow(s, 4.3, 5.72, 5.1, 5.72);
  arrow(s, 8.2, 5.72, 9.0, 5.72);
}

{
  const s = pptx.addSlide();
  header(s, "검증 결과와 다음 항목", "OP-C02-06 close 가능");
  box(s, 0.9, 1.35, 5.4, 1.0, "xsim_stale_ready.log:30\nskid_buffer PASS", C.greenSoft, { line: C.green, fontSize: 13, bold: true });
  box(s, 7.0, 1.35, 5.4, 1.0, "xsim_stale_ready.log:34\nsync_fifo blocked accepts=6 PASS", C.greenSoft, { line: C.green, fontSize: 12.5, bold: true });
  box(s, 3.15, 3.0, 7.0, 0.85, "xsim_stale_ready.log:36\nALL TESTS PASSED", C.greenSoft, { line: C.green, fontSize: 14, bold: true });
  label(s, 0.9, 4.75, 11.5, 0.35, "다음 우선순위", { fontSize: 15, bold: true, align: "center" });
  box(s, 2.6, 5.25, 8.2, 0.9, "OP-C02-03\nconfig_ctrl/top expected-count CDC integration", C.amberSoft, { line: C.amber, fontSize: 14, bold: true });
}

pptx.writeFile({
  fileName: path.join(__dirname, "C02_Chip_Acquisition_260430230732_Stale_Ready_Negative_Fix_v001.pptx"),
});
