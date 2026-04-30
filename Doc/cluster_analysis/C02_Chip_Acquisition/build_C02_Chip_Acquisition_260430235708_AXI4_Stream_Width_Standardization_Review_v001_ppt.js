const pptxgen = require("pptxgenjs");
const path = require("path");

const pptx = new pptxgen();
pptx.layout = "LAYOUT_WIDE";
pptx.author = "Codex";
pptx.subject = "C02 AXI4-Stream width standardization review";
pptx.title = "C02 AXI4-Stream Width Standardization Review v001";
pptx.company = "OpenAI";
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
  violet: "7C3AED",
  violetSoft: "EDE9FE",
  cyan: "0891B2",
  cyanSoft: "CFFAFE",
  slateSoft: "E2E8F0",
};

function header(slide, title, sub) {
  slide.background = { color: C.bg };
  slide.addText(title, {
    x: 0.42, y: 0.22, w: 12.5, h: 0.42,
    fontFace: "Malgun Gothic", fontSize: 18, bold: true,
    color: C.ink, margin: 0, fit: "shrink",
  });
  slide.addText(sub, {
    x: 0.44, y: 0.72, w: 12.2, h: 0.27,
    fontFace: "Malgun Gothic", fontSize: 8.5,
    color: C.muted, margin: 0, fit: "shrink",
  });
  slide.addShape(pptx.ShapeType.line, {
    x: 0.42, y: 1.05, w: 12.5, h: 0,
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
    margin: opts.margin === undefined ? 0.04 : opts.margin,
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
  text(slide, x + 0.08, y + 0.06, w - 0.16, h - 0.12, value, {
    fontSize: opts.fontSize || 10,
    bold: opts.bold,
    color: opts.color || C.ink,
    align: opts.align || "center",
    valign: "mid",
    margin: 0.02,
  });
}

function arrow(slide, x1, y1, x2, y2, color = C.muted, width = 1.2) {
  slide.addShape(pptx.ShapeType.line, {
    x: x1, y: y1, w: x2 - x1, h: y2 - y1,
    line: { color, width, beginArrowType: "none", endArrowType: "triangle" },
  });
}

function row(slide, y, cells, widths, fills = []) {
  let x = 0.65;
  cells.forEach((cell, i) => {
    box(slide, x, y, widths[i], 0.43, cell, fills[i] || C.white, {
      fontSize: 8.5,
      bold: y < 1.8,
      align: i === 0 ? "center" : "left",
    });
    x += widths[i] + 0.04;
  });
}

{
  const s = pptx.addSlide();
  header(s, "AXI4-Stream 폭 표준화 결론", "수정 가능. 단, output full-keep 표준화와 partial-keep 완전 유연화를 분리");

  box(s, 0.8, 1.45, 3.45, 1.0, "Phase A\n32/64/128-bit\nfull-keep", C.greenSoft, { line: C.green, fontSize: 15, bold: true });
  box(s, 4.95, 1.45, 3.45, 1.0, "Phase B\nraw/event 폭\n상수화", C.blueSoft, { line: C.blue, fontSize: 15, bold: true });
  box(s, 9.1, 1.45, 3.45, 1.0, "Phase C\n256-bit 이상\npartial tkeep", C.amberSoft, { line: C.amber, fontSize: 15, bold: true });

  text(s, 0.9, 3.1, 11.55, 0.6,
    "추천 결정: C02에서는 먼저 32/64/128-bit 표준 AXIS를 닫는다. 256-bit 이상은 header 48B 정렬 문제 때문에 partial tkeep 설계가 필요하다.",
    { fontSize: 15, bold: true, align: "center" });

  box(s, 1.1, 4.65, 2.7, 0.65, "TDATA: 8의 배수", C.slateSoft, { fontSize: 12, bold: true });
  box(s, 4.0, 4.65, 2.7, 0.65, "TKEEP: TDATA/8", C.slateSoft, { fontSize: 12, bold: true });
  box(s, 6.9, 4.65, 2.7, 0.65, "TSTRB: TDATA/8", C.slateSoft, { fontSize: 12, bold: true });
  box(s, 9.8, 4.65, 2.7, 0.65, "TUSER: 역할별", C.slateSoft, { fontSize: 12, bold: true });

  text(s, 0.95, 6.15, 11.5, 0.35,
    "Datasheet 기준 GPX raw word는 28-bit로 고정. Stream 폭 확장은 output drain 효율을 바꾸며 GPX READ II를 바꾸지는 않는다.",
    { fontSize: 11.5, bold: true, color: C.muted, align: "center" });
}

{
  const s = pptx.addSlide();
  header(s, "현재 구조의 폭 유연성", "output/cell은 일부 generic, raw/event는 고정 폭");

  row(s, 1.35, ["구간", "현재 폭", "문제", "판정"], [2.0, 2.45, 4.25, 3.3], [C.slateSoft, C.slateSoft, C.slateSoft, C.slateSoft]);
  row(s, 1.85, ["bus_phy", "32 data / 8 user", "GPX 28-bit raw 응답", "유지 + constant화"], [2.0, 2.45, 4.25, 3.3]);
  row(s, 2.35, ["chip_ctrl", "32/8 raw", "raw FIFO pack 고정", "constant화"], [2.0, 2.45, 4.25, 3.3]);
  row(s, 2.85, ["decode", "32/8 -> 32/16", "40/48-bit skid pack", "constant화"], [2.0, 2.45, 4.25, 3.3]);
  row(s, 3.35, ["cell", "g_OUTPUT_WIDTH", "32/64 중심 검증", "128 추가 검증"], [2.0, 2.45, 4.25, 3.3]);
  row(s, 3.85, ["final", "data/user/last", "tkeep/tstrb 없음", "Phase A 수정"], [2.0, 2.45, 4.25, 3.3]);
  row(s, 4.35, ["stop/fire", "32 중심", "tuser가 data와 결합", "Phase B 분리"], [2.0, 2.45, 4.25, 3.3]);

  box(s, 1.0, 5.55, 5.25, 0.75, "유연한 곳\ncell/output datapath", C.greenSoft, { line: C.green, fontSize: 13, bold: true });
  box(s, 7.0, 5.55, 5.25, 0.75, "고정된 곳\nraw/event/stop sideband", C.redSoft, { line: C.red, fontSize: 13, bold: true });
}

{
  const s = pptx.addSlide();
  header(s, "폭별 Header / Cell 영향", "48B header와 32B cell이 full-keep 지원 폭을 결정");

  row(s, 1.35, ["TDATA", "Byte/beat", "Header", "Cell", "판정"], [1.8, 2.0, 2.2, 2.2, 3.7], [C.slateSoft, C.slateSoft, C.slateSoft, C.slateSoft, C.slateSoft]);
  row(s, 1.85, ["32", "4", "12 beats", "8 beats", "가능"], [1.8, 2.0, 2.2, 2.2, 3.7]);
  row(s, 2.35, ["64", "8", "6 beats", "4 beats", "가능"], [1.8, 2.0, 2.2, 2.2, 3.7]);
  row(s, 2.85, ["128", "16", "3 beats", "2 beats", "가능, 신규 검증"], [1.8, 2.0, 2.2, 2.2, 3.7]);
  row(s, 3.35, ["256", "32", "1.5 beats", "1 beat", "partial tkeep 필요"], [1.8, 2.0, 2.2, 2.2, 3.7]);

  box(s, 0.95, 4.75, 3.5, 0.8, "Row 1072B @32b\n268 beats", C.blueSoft, { line: C.blue, fontSize: 12.5, bold: true });
  arrow(s, 4.45, 5.15, 5.05, 5.15);
  box(s, 5.05, 4.75, 3.5, 0.8, "Row 1072B @64b\n134 beats", C.greenSoft, { line: C.green, fontSize: 12.5, bold: true });
  arrow(s, 8.55, 5.15, 9.15, 5.15);
  box(s, 9.15, 4.75, 3.5, 0.8, "Row 1072B @128b\n67 beats", C.violetSoft, { line: C.violet, fontSize: 12.5, bold: true });
}

{
  const s = pptx.addSlide();
  header(s, "Phase A 수정 범위", "최종 output을 표준 AXI4-Stream 형태로 닫는 저위험 변경");

  const xs = [0.75, 2.55, 4.35, 6.15, 7.95, 9.75, 11.35];
  const labels = ["cell\nbuilder", "cell\npipe", "face\nassembler", "face\nFIFO", "header\ninserter", "top\noutput", "next\nCluster"];
  labels.forEach((label, i) => {
    box(s, xs[i], 1.65, 1.25, 0.75, label, i < 5 ? C.blueSoft : C.greenSoft, { line: i < 5 ? C.blue : C.green, fontSize: 9.4, bold: true });
    if (i < labels.length - 1) arrow(s, xs[i] + 1.25, 2.02, xs[i + 1], 2.02);
  });

  box(s, 0.95, 3.25, 3.45, 0.65, "추가: tkeep/tstrb", C.greenSoft, { line: C.green, fontSize: 13, bold: true });
  box(s, 4.95, 3.25, 3.45, 0.65, "제약: 32/64/128", C.amberSoft, { line: C.amber, fontSize: 13, bold: true });
  box(s, 8.95, 3.25, 3.45, 0.65, "정책: full keep", C.cyanSoft, { line: C.cyan, fontSize: 13, bold: true });

  row(s, 4.75, ["파일", "핵심 수정"], [3.1, 9.0], [C.slateSoft, C.slateSoft]);
  row(s, 5.25, ["tdc_gpx_top", "rise/fall output에 tkeep/tstrb 포트 추가"], [3.1, 9.0]);
  row(s, 5.75, ["output_stage", "header_inserter의 keep/strb를 top으로 전달"], [3.1, 9.0]);
  row(s, 6.25, ["header_inserter", "full keep 생성과 width assert 추가"], [3.1, 9.0]);
}

{
  const s = pptx.addSlide();
  header(s, "Phase B 수정 범위", "raw/event/stop/fire 폭 의미를 이름 있는 계약으로 관리");

  box(s, 0.9, 1.45, 2.55, 0.8, "bus response\n32/8", C.slateSoft, { fontSize: 12, bold: true });
  arrow(s, 3.45, 1.85, 4.15, 1.85);
  box(s, 4.15, 1.45, 2.55, 0.8, "raw axis\n32/8", C.blueSoft, { line: C.blue, fontSize: 12, bold: true });
  arrow(s, 6.7, 1.85, 7.4, 1.85);
  box(s, 7.4, 1.45, 2.55, 0.8, "event axis\n32/16", C.greenSoft, { line: C.green, fontSize: 12, bold: true });
  arrow(s, 9.95, 1.85, 10.65, 1.85);
  box(s, 10.65, 1.45, 1.85, 0.8, "cell", C.amberSoft, { line: C.amber, fontSize: 12, bold: true });

  row(s, 3.1, ["Constant", "의미"], [3.4, 8.7], [C.slateSoft, C.slateSoft]);
  row(s, 3.6, ["c_RAW_AXIS_TDATA_WIDTH", "chip_ctrl raw word carrier, 32-bit 표준 내부 stream"], [3.4, 8.7]);
  row(s, 4.1, ["c_RAW_AXIS_TUSER_WIDTH", "IFIFO/control/fault marker, 8-bit"], [3.4, 8.7]);
  row(s, 4.6, ["c_EVT_AXIS_TDATA_WIDTH", "decoded hit carrier, 32-bit"], [3.4, 8.7]);
  row(s, 5.1, ["c_EVT_AXIS_TUSER_WIDTH", "slope/chip/stop/shot/hit_seq, 16-bit"], [3.4, 8.7]);
  row(s, 5.6, ["g_STOP_EVT_TUSER_WIDTH", "stop_evt data width와 tuser width 분리"], [3.4, 8.7]);
}

{
  const s = pptx.addSlide();
  header(s, "Timing / Throughput / II 영향", "폭 확장은 output drain을 줄이고 GPX READ II는 유지");

  row(s, 1.35, ["항목", "영향"], [3.0, 9.1], [C.slateSoft, C.slateSoft]);
  row(s, 1.85, ["GPX READ II", "변경 없음. Datasheet 기반 40MHz 이하 READ timing 유지"], [3.0, 9.1]);
  row(s, 2.35, ["raw/event latency", "Phase A에서는 변경 없음"], [3.0, 9.1]);
  row(s, 2.85, ["output latency", "64/128로 갈수록 row/frame beat 수 감소"], [3.0, 9.1]);
  row(s, 3.35, ["AXIS beat II", "각 stage 1 beat/clk 유지 목표"], [3.0, 9.1]);
  row(s, 3.85, ["timing closure", "128-bit부터 header/cell mux timing 재검증 필요"], [3.0, 9.1]);

  box(s, 0.95, 5.25, 3.45, 0.72, "READ 병목\n그대로", C.redSoft, { line: C.red, fontSize: 13, bold: true });
  box(s, 4.95, 5.25, 3.45, 0.72, "Output drain\nbeat 수 감소", C.greenSoft, { line: C.green, fontSize: 13, bold: true });
  box(s, 8.95, 5.25, 3.45, 0.72, "Pipeline II\n1 clk 유지", C.blueSoft, { line: C.blue, fontSize: 13, bold: true });
}

{
  const s = pptx.addSlide();
  header(s, "검증 Matrix", "폭 변경 후 닫아야 하는 기능 경계");

  row(s, 1.35, ["ID", "검증", "Phase"], [1.3, 8.7, 2.1], [C.slateSoft, C.slateSoft, C.slateSoft]);
  row(s, 1.85, ["W-01", "g_OUTPUT_WIDTH=32 회귀", "A"], [1.3, 8.7, 2.1]);
  row(s, 2.35, ["W-02", "g_OUTPUT_WIDTH=64 회귀", "A"], [1.3, 8.7, 2.1]);
  row(s, 2.85, ["W-03", "g_OUTPUT_WIDTH=128 신규 sim", "A"], [1.3, 8.7, 2.1]);
  row(s, 3.35, ["W-04", "final tkeep/tstrb all ones", "A"], [1.3, 8.7, 2.1]);
  row(s, 3.85, ["W-05", "SOF/EOL 의미 유지", "A"], [1.3, 8.7, 2.1]);
  row(s, 4.35, ["W-06", "raw/event constant화 후 decode PASS", "B"], [1.3, 8.7, 2.1]);
  row(s, 4.85, ["W-07", "stop_evt data/user width 분리", "B"], [1.3, 8.7, 2.1]);
  row(s, 5.35, ["W-08", "partial tkeep positive/negative", "C"], [1.3, 8.7, 2.1]);
}

{
  const s = pptx.addSlide();
  header(s, "추천 수정 결정", "C02에서 지금 선택할 수 있는 합리적인 길");

  box(s, 0.95, 1.55, 3.35, 1.0, "지금 진행\nPhase A", C.greenSoft, { line: C.green, fontSize: 16, bold: true });
  box(s, 4.95, 1.55, 3.35, 1.0, "Phase A PASS 후\nPhase B", C.blueSoft, { line: C.blue, fontSize: 16, bold: true });
  box(s, 8.95, 1.55, 3.35, 1.0, "다음 Cluster 계약 후\nPhase C", C.amberSoft, { line: C.amber, fontSize: 15, bold: true });

  text(s, 1.0, 3.25, 11.3, 0.52,
    "결론: 표준화는 가능하다. 다만 raw GPX 의미는 28-bit로 고정하고, AXI 폭 유연성은 output/cell 쪽부터 안전하게 확장하는 것이 맞다.",
    { fontSize: 15, bold: true, align: "center" });

  box(s, 1.25, 4.8, 10.85, 0.82, "다음 코딩 작업 제안: top/output_stage/header_inserter에 tkeep/tstrb 추가 + 32/64/128 width matrix TB 작성", C.violetSoft, { line: C.violet, fontSize: 13.5, bold: true });
}

async function main() {
  await pptx.writeFile({
    fileName: path.join(__dirname, "C02_Chip_Acquisition_260430235708_AXI4_Stream_Width_Standardization_Review_v001.pptx"),
  });
}

main().catch((err) => {
  console.error(err);
  process.exit(1);
});
