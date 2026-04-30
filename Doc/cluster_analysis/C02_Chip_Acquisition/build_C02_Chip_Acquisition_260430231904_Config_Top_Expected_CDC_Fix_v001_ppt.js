const pptxgen = require("pptxgenjs");
const path = require("path");

const pptx = new pptxgen();
pptx.layout = "LAYOUT_WIDE";
pptx.author = "Codex";
pptx.subject = "C02 config/top expected count CDC integration";
pptx.title = "C02 Config Top Expected CDC Fix v001";
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

function text(slide, x, y, w, h, value, opts = {}) {
  slide.addText(value, {
    x, y, w, h,
    fontFace: "Malgun Gothic",
    fontSize: opts.fontSize || 10,
    bold: !!opts.bold,
    color: opts.color || C.ink,
    align: opts.align || "left",
    valign: opts.valign || "top",
    margin: 0.03,
    fit: "shrink",
  });
}

function box(slide, x, y, w, h, value, fill, opts = {}) {
  slide.addShape(pptx.ShapeType.roundRect, {
    x, y, w, h,
    rectRadius: 0.04,
    fill: { color: fill },
    line: { color: opts.line || C.line, width: opts.lineWidth || 1 },
  });
  text(slide, x + 0.11, y + 0.08, w - 0.22, h - 0.16, value, {
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
  header(s, "C02 OP-C02-03 보완", "작성: 2026-04-30 23:19 KST | 기준: Datasheet 기반 I-Mode single expected-count 계약");
  box(s, 0.8, 1.55, 3.25, 1.05, "OP-C02-03\n부분반영 -> 반영", C.blueSoft, { line: C.blue, bold: true, fontSize: 14 });
  box(s, 5.05, 1.55, 3.25, 1.05, "config_ctrl\nexpected CDC PASS", C.greenSoft, { line: C.green, bold: true, fontSize: 13 });
  box(s, 9.15, 1.55, 3.25, 1.05, "tdc_gpx_top\ncount-bound PASS", C.greenSoft, { line: C.green, bold: true, fontSize: 13 });
  arrow(s, 4.05, 2.08, 5.05, 2.08, C.green);
  arrow(s, 8.3, 2.08, 9.15, 2.08, C.green);
  text(s, 0.95, 3.4, 11.5, 0.55, "결론: fire-count final expected가 top/config CDC를 지나 chip_run까지 도달했고, EF fallback 대신 expected 개수에서 READ가 멈추는 것을 확인했다.", { fontSize: 14, bold: true, align: "center" });
  box(s, 1.2, 4.75, 3.1, 0.85, "physical IFIFO\n3 words", C.redSoft, { line: C.red, fontSize: 13, bold: true });
  box(s, 5.1, 4.75, 3.1, 0.85, "expected final\n2 words", C.amberSoft, { line: C.amber, fontSize: 13, bold: true });
  box(s, 9.0, 4.75, 3.1, 0.85, "leftover\n1 word", C.greenSoft, { line: C.green, fontSize: 13, bold: true });
  arrow(s, 4.3, 5.18, 5.1, 5.18, C.ink);
  arrow(s, 8.2, 5.18, 9.0, 5.18, C.green);
}

{
  const s = pptx.addSlide();
  header(s, "검증 경로", "tdc_gpx_top 포트부터 chip_run drain decision까지 end-to-end 확인");
  box(s, 0.55, 2.15, 2.0, 0.72, "TB\nstop_evt", C.white, { fontSize: 11, bold: true });
  box(s, 3.05, 2.15, 2.0, 0.72, "top\nport map", C.blueSoft, { line: C.blue, fontSize: 11, bold: true });
  box(s, 5.55, 2.15, 2.0, 0.72, "stop_cfg\nfire match", C.blueSoft, { line: C.blue, fontSize: 11, bold: true });
  box(s, 8.05, 2.15, 2.0, 0.72, "expected\nCDC", C.amberSoft, { line: C.amber, fontSize: 11, bold: true });
  box(s, 10.55, 2.15, 2.0, 0.72, "chip_run\nREAD bound", C.greenSoft, { line: C.green, fontSize: 11, bold: true });
  arrow(s, 2.55, 2.51, 3.05, 2.51);
  arrow(s, 5.05, 2.51, 5.55, 2.51);
  arrow(s, 7.55, 2.51, 8.05, 2.51);
  arrow(s, 10.05, 2.51, 10.55, 2.51);
  row(s, 4.0, "입력", "expected=2, fire_count tlast=1", "현재 face shot count와 일치");
  row(s, 4.55, "물리", "IFIFO physical=3", "fallback이면 3개를 읽어야 함");
  row(s, 5.1, "판정", "READ=2, leftover=1", "expected-count bounded drain 증명");
}

{
  const s = pptx.addSlide();
  header(s, "Timing Block", "IrFlag 전에 expected tuple CDC 안정화");
  box(s, 0.8, 3.15, 1.8, 0.62, "shot_start", C.blueSoft, { line: C.blue, fontSize: 10, bold: true });
  box(s, 3.0, 3.15, 1.8, 0.62, "load\n3 words", C.redSoft, { line: C.red, fontSize: 10, bold: true });
  box(s, 5.2, 3.15, 1.8, 0.62, "expected\n2 final", C.amberSoft, { line: C.amber, fontSize: 10, bold: true });
  box(s, 7.4, 3.15, 1.8, 0.62, "CDC\n80 clk", C.amberSoft, { line: C.amber, fontSize: 10, bold: true });
  box(s, 9.6, 3.15, 1.8, 0.62, "IrFlag", C.blueSoft, { line: C.blue, fontSize: 10, bold: true });
  box(s, 11.6, 3.15, 1.2, 0.62, "READ 2", C.greenSoft, { line: C.green, fontSize: 10, bold: true });
  arrow(s, 2.6, 3.46, 3.0, 3.46);
  arrow(s, 4.8, 3.46, 5.2, 3.46);
  arrow(s, 7.0, 3.46, 7.4, 3.46);
  arrow(s, 9.2, 3.46, 9.6, 3.46);
  arrow(s, 11.4, 3.46, 11.6, 3.46);
  text(s, 1.0, 5.0, 11.4, 0.45, "ST_DRAIN_LATCH가 expected tuple을 snapshot하기 전에 final_valid와 IFIFO counts가 같은 CDC payload로 도착한다.", { fontSize: 13, bold: true, align: "center" });
}

{
  const s = pptx.addSlide();
  header(s, "Latency / Throughput / Pipeline / II", "RTL timing path 변화 없이 검증 경계를 강화");
  row(s, 1.35, "Metric", "영향", "근거", C.slateSoft);
  row(s, 1.9, "Latency", "RTL 변화 없음", "TB 주입/검증 강화");
  row(s, 2.45, "Throughput", "READ throughput 유지", "C01/C02 timing legality 계약 유지");
  row(s, 3.0, "Pipeline", "top->config->CDC->run 검증", "end-to-end stop/fire path");
  row(s, 3.55, "II", "shot II 기존 유지", "face_seq shot_period 조건 그대로");
  row(s, 4.1, "CDC", "atomic tuple", "IFIFO1/IFIFO2/final_valid 단일 payload");
  box(s, 2.1, 5.3, 3.3, 0.8, "config_ctrl\nxsim PASS", C.greenSoft, { line: C.green, fontSize: 13, bold: true });
  box(s, 7.9, 5.3, 3.3, 0.8, "top_int\nxsim PASS", C.greenSoft, { line: C.green, fontSize: 13, bold: true });
}

{
  const s = pptx.addSlide();
  header(s, "검증 결과와 다음 단계", "OP-C02-03 close 가능");
  box(s, 0.75, 1.35, 5.7, 1.0, "xsim_config_ctrl_op_c02_03.log\nexpected tuple CDC PASS\ndata=16 ctrl=8", C.greenSoft, { line: C.green, fontSize: 12.5, bold: true });
  box(s, 6.9, 1.35, 5.7, 1.0, "xsim_top_int_op_c02_03.log\nshot1/shot2 read=2 leftover=1", C.greenSoft, { line: C.green, fontSize: 12.5, bold: true });
  box(s, 2.85, 3.2, 7.6, 0.9, "OP-C02-01 ~ OP-C02-06\n모두 반영 상태", C.blueSoft, { line: C.blue, fontSize: 15, bold: true });
  text(s, 0.95, 5.05, 11.5, 0.45, "다음은 C02 closure 문서와 C03/C04 인계 계약 정리", { fontSize: 14, bold: true, align: "center" });
}

pptx.writeFile({
  fileName: path.join(__dirname, "C02_Chip_Acquisition_260430231904_Config_Top_Expected_CDC_Fix_v001.pptx"),
});
