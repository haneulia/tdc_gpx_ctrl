const pptxgen = require("pptxgenjs");
const path = require("path");

const pptx = new pptxgen();
pptx.layout = "LAYOUT_WIDE";
pptx.author = "Codex";
pptx.subject = "C02 timing legality matrix fix";
pptx.title = "C02 Timing Legality Matrix Fix v001";
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
  slate: "334155",
  slateSoft: "E2E8F0",
};

function header(slide, title, sub) {
  slide.background = { color: C.bg };
  slide.addText(title, {
    x: 0.45, y: 0.23, w: 12.35, h: 0.44,
    fontFace: "Malgun Gothic", fontSize: 18, bold: true,
    color: C.ink, margin: 0, fit: "shrink",
  });
  slide.addText(sub, {
    x: 0.46, y: 0.72, w: 12.2, h: 0.26,
    fontFace: "Malgun Gothic", fontSize: 8.5,
    color: C.muted, margin: 0, fit: "shrink",
  });
  slide.addShape(pptx.ShapeType.line, {
    x: 0.45, y: 1.04, w: 12.42, h: 0,
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
    margin: 0.02,
    fit: "shrink",
  });
}

function box(slide, x, y, w, h, text, fill, opts = {}) {
  slide.addShape(pptx.ShapeType.roundRect, {
    x, y, w, h,
    rectRadius: 0.04,
    fill: { color: fill },
    line: { color: opts.line || C.line, width: opts.lineWidth || 1 },
  });
  label(slide, x + 0.1, y + 0.08, w - 0.2, h - 0.16, text, {
    fontSize: opts.fontSize || 10.5,
    bold: opts.bold,
    color: opts.color || C.ink,
    align: opts.align || "center",
    valign: "mid",
  });
}

function arrow(slide, x1, y1, x2, y2, color = C.slate) {
  slide.addShape(pptx.ShapeType.line, {
    x: x1, y: y1, w: x2 - x1, h: y2 - y1,
    line: { color, width: 1.4, beginArrowType: "none", endArrowType: "triangle" },
  });
}

function row(slide, y, a, b, c, fill = C.white) {
  box(slide, 0.8, y, 2.4, 0.46, a, fill, { fontSize: 9.5, bold: true });
  box(slide, 3.25, y, 4.3, 0.46, b, fill, { fontSize: 9.5, align: "left" });
  box(slide, 7.6, y, 4.9, 0.46, c, fill, { fontSize: 9.5, align: "left" });
}

// Slide 1
{
  const s = pptx.addSlide();
  header(s, "C02 Timing Legality Matrix 보완", "작성: 2026-04-30 22:56 KST | 기준: Datasheet READ timing + C01 v009 계약");
  box(s, 0.75, 1.5, 3.4, 1.1, "OP-C02-05\nillegal div/ticks matrix", C.blueSoft, { line: C.blue, bold: true, fontSize: 14 });
  box(s, 4.9, 1.5, 3.5, 1.1, "CSR boundary\n68 cases PASS", C.greenSoft, { line: C.green, bold: true, fontSize: 14 });
  box(s, 9.1, 1.5, 3.4, 1.1, "Bus_Phy boundary\nphysical timing PASS", C.greenSoft, { line: C.green, bold: true, fontSize: 13 });
  arrow(s, 4.15, 2.05, 4.9, 2.05);
  arrow(s, 8.4, 2.05, 9.1, 2.05);
  label(s, 0.9, 3.45, 11.8, 0.35, "결론: RTL 변경 없이 TB evidence를 확장했고, 200 MHz에서 GPX 40 MHz readout 한계를 넘는 조합은 clamp로 차단된다.", { fontSize: 14, bold: true, align: "center" });
  box(s, 1.1, 4.55, 5.1, 0.9, "대표 illegal\nraw div=1,ticks=4", C.redSoft, { line: C.red, fontSize: 13 });
  box(s, 7.1, 4.55, 5.1, 0.9, "safe output\ndiv=1,ticks=5", C.greenSoft, { line: C.green, fontSize: 13 });
  arrow(s, 6.2, 5.0, 7.1, 5.0, C.green);
}

// Slide 2
{
  const s = pptx.addSlide();
  header(s, "Datasheet 기준식", "C01 v009가 정리한 p.7 READ timing과 p.27 40 MHz readout 계약");
  row(s, 1.45, "항목", "공식", "판단 기준", C.slateSoft);
  row(s, 2.0, "capture", "((ticks-3)*div+1)*5ns", "tV-DR max 11.8ns 뒤에 sample");
  row(s, 2.55, "RDN low", "(ticks-2)*div*5ns", "tPW-RL >= 6ns");
  row(s, 3.1, "burst II", "ticks*div*5ns", "25ns 이상, 40MHz 이하");
  row(s, 3.65, "div=1", "ticks >= 5", "200MHz 최단 정상 경계");
  row(s, 4.2, "div>=2", "ticks >= 4", "absolute minimum");
  box(s, 1.1, 5.35, 5.2, 0.85, "div=1,ticks=5\n25ns / 40MHz 정상 경계", C.greenSoft, { line: C.green, fontSize: 13 });
  box(s, 7.0, 5.35, 5.2, 0.85, "div=1,ticks=4\n20ns / 50MHz 금지", C.redSoft, { line: C.red, fontSize: 13 });
}

// Slide 3
{
  const s = pptx.addSlide();
  header(s, "검증 구조", "CSR 정책과 Bus_Phy leaf safety를 분리 검증");
  box(s, 0.65, 2.05, 2.1, 0.9, "AXI4-Lite\nCTL1 raw write", C.white, { fontSize: 10 });
  box(s, 3.05, 2.05, 2.1, 0.9, "CSR clamp\nsafe cfg", C.blueSoft, { line: C.blue, fontSize: 10 });
  box(s, 5.45, 2.05, 2.1, 0.9, "chip_ctrl\nsnapshot", C.slateSoft, { fontSize: 10 });
  box(s, 7.85, 2.05, 2.1, 0.9, "Bus_Phy\nlocal clamp", C.amberSoft, { line: C.amber, fontSize: 10 });
  box(s, 10.25, 2.05, 2.1, 0.9, "GPX pins\nRDN timing", C.greenSoft, { line: C.green, fontSize: 10 });
  arrow(s, 2.75, 2.5, 3.05, 2.5);
  arrow(s, 5.15, 2.5, 5.45, 2.5);
  arrow(s, 7.55, 2.5, 7.85, 2.5);
  arrow(s, 9.95, 2.5, 10.25, 2.5);
  box(s, 1.0, 4.15, 5.1, 1.1, "CSR matrix\nmanual 대표 10 + sweep 48 + div63 edge 8\n총 68 PASS", C.greenSoft, { line: C.green, fontSize: 12 });
  box(s, 7.0, 4.15, 5.1, 1.1, "Bus_Phy timing\nillegal 3건 warning + legal 1건\nRDN low width 확인", C.greenSoft, { line: C.green, fontSize: 12 });
}

// Slide 4
{
  const s = pptx.addSlide();
  header(s, "Timing / Latency / Throughput / II", "이번 수정은 테스트 보강이며 RTL data path 변경 없음");
  row(s, 1.35, "Metric", "영향", "근거", C.slateSoft);
  row(s, 1.9, "Latency", "변화 없음", "RTL path 미수정");
  row(s, 2.45, "Throughput", "40MHz 초과 방지", "div=1,ticks<5 -> ticks=5");
  row(s, 3.0, "Pipeline", "구조 유지", "CSR -> chip_ctrl -> bus_phy");
  row(s, 3.55, "II", "최단 burst II 25ns", "1*5*5ns = 25ns");
  box(s, 1.15, 5.0, 3.0, 0.85, "raw\n1,4", C.redSoft, { line: C.red, fontSize: 13, bold: true });
  box(s, 5.15, 5.0, 3.0, 0.85, "clamp\n1,5", C.greenSoft, { line: C.green, fontSize: 13, bold: true });
  box(s, 9.15, 5.0, 3.0, 0.85, "burst II\n25ns", C.greenSoft, { line: C.green, fontSize: 13, bold: true });
  arrow(s, 4.15, 5.42, 5.15, 5.42, C.green);
  arrow(s, 8.15, 5.42, 9.15, 5.42, C.green);
}

// Slide 5
{
  const s = pptx.addSlide();
  header(s, "검증 결과와 다음 단계", "OP-C02-05 close 가능");
  box(s, 0.85, 1.55, 5.55, 1.15, "xsim_csr_chip_timing_matrix.log\n68 pass, 0 fail\nALL TESTS PASSED", C.greenSoft, { line: C.green, fontSize: 13, bold: true });
  box(s, 6.95, 1.55, 5.55, 1.15, "xsim_bus_phy_c01_timing_matrix.log\nlocal clamp warning + PASS", C.greenSoft, { line: C.green, fontSize: 13, bold: true });
  label(s, 1.0, 3.6, 11.4, 0.4, "남은 open item", { fontSize: 15, bold: true });
  box(s, 3.0, 4.25, 7.3, 1.0, "OP-C02-06\nstale ready negative test", C.amberSoft, { line: C.amber, fontSize: 14, bold: true });
  label(s, 0.75, 5.85, 11.8, 0.35, "추천: 다음 수정은 stale ready로 잘못 pop/drop되지 않는 negative TB 보강", { fontSize: 13, bold: true, align: "center" });
}

pptx.writeFile({
  fileName: path.join(__dirname, "C02_Chip_Acquisition_260430225644_Timing_Legality_Matrix_Fix_v001.pptx"),
});
