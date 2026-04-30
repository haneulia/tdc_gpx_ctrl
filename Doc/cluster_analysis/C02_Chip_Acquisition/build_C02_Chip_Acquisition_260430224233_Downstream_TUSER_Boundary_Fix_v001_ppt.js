const pptxgen = require("pptxgenjs");
const path = require("path");

const pptx = new pptxgen();
pptx.layout = "LAYOUT_WIDE";
pptx.author = "Codex";
pptx.subject = "C02 downstream TUSER boundary fix";
pptx.title = "C02 Downstream TUSER Boundary Fix v001";
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

function bullet(slide, x, y, w, text, color = C.ink) {
  label(slide, x, y, w, 0.26, "• " + text, { fontSize: 10, color });
}

// Slide 1
{
  const s = pptx.addSlide();
  header(s, "C02 Downstream TUSER Boundary 보완", "작성: 2026-04-30 22:42 KST | 기준: Datasheet + C02 내부 AXI-stream 계약");
  box(s, 0.75, 1.45, 3.6, 1.1, "OP-C02-04\nDownstream TUSER boundary", C.blueSoft, { line: C.blue, bold: true, fontSize: 14 });
  box(s, 4.9, 1.45, 3.6, 1.1, "발견 1\nsame-cycle tlast/tuser 누락", C.redSoft, { line: C.red, bold: true, fontSize: 13 });
  box(s, 9.05, 1.45, 3.6, 1.1, "발견 2\nrow_done_faulted pulse clear 누락", C.redSoft, { line: C.red, bold: true, fontSize: 12.5 });
  arrow(s, 4.35, 2.0, 4.9, 2.0, C.slate);
  arrow(s, 8.5, 2.0, 9.05, 2.0, C.slate);
  box(s, 0.95, 3.25, 5.3, 1.2, "보완\nface_assembler 순차 프로세스 안에서\nsame-cycle fault OR + 1clk default clear", C.greenSoft, { line: C.green, fontSize: 13 });
  box(s, 7.1, 3.25, 5.0, 1.2, "검증\ncell_pipe / output_stage xsim PASS\nSOF와 row fault 의미 분리 확인", C.greenSoft, { line: C.green, fontSize: 13 });
  label(s, 0.75, 5.45, 12.0, 0.4, "판단: GPX Datasheet timing은 변경하지 않고, 내부 stream sideband 계약만 명확화했다.", { fontSize: 14, bold: true, color: C.ink, align: "center" });
}

// Slide 2
{
  const s = pptx.addSlide();
  header(s, "TUSER 의미 경계", "cell fault는 row status pulse로, header stream tuser는 SOF 전용으로 분리");
  box(s, 0.55, 2.15, 2.1, 0.86, "raw/config\nraw tuser", C.slateSoft, { fontSize: 10 });
  box(s, 3.0, 2.15, 2.45, 0.86, "cell_pipe\nslice fault on tlast", C.blueSoft, { line: C.blue, fontSize: 10 });
  box(s, 5.8, 2.15, 2.45, 0.86, "face_assembler\nfault latch", C.amberSoft, { line: C.amber, fontSize: 10 });
  box(s, 8.6, 1.48, 2.15, 0.86, "row_done_faulted\n1clk pulse", C.greenSoft, { line: C.green, fontSize: 10 });
  box(s, 8.6, 2.82, 2.15, 0.86, "header\nm_axis_tuser(0)=SOF", C.greenSoft, { line: C.green, fontSize: 10 });
  box(s, 11.05, 2.82, 1.65, 0.86, "VDMA\nstream", C.white, { fontSize: 10 });
  arrow(s, 2.65, 2.58, 3.0, 2.58);
  arrow(s, 5.45, 2.58, 5.8, 2.58);
  arrow(s, 8.25, 2.43, 8.6, 1.92, C.green);
  arrow(s, 8.25, 2.73, 8.6, 3.25, C.green);
  arrow(s, 10.75, 3.25, 11.05, 3.25);
  bullet(s, 0.75, 4.75, 11.8, "cell fault는 m_axis_tuser로 재사용하지 않는다. downstream stream의 tuser(0)는 SOF 식별만 담당한다.");
  bullet(s, 0.75, 5.18, 11.8, "row fault는 o_row_done_faulted_rise/fall 1클럭 pulse로 관측한다.");
  bullet(s, 0.75, 5.61, 11.8, "frame_done_faulted는 header drain watchdog synthetic close 전용으로 유지한다.");
}

// Slide 3
{
  const s = pptx.addSlide();
  header(s, "수정 포인트", "조합 출력 경로 추가 없이 registered boundary 안에서 보완");
  box(s, 0.7, 1.55, 3.4, 1.0, "기존 문제\npending register만 참조", C.redSoft, { line: C.red, fontSize: 12, bold: true });
  box(s, 4.95, 1.55, 3.4, 1.0, "마지막 chip fault\nsame-cycle tlast/tuser", C.amberSoft, { line: C.amber, fontSize: 12, bold: true });
  box(s, 9.2, 1.55, 3.2, 1.0, "결과\nrow fault 누락 가능", C.redSoft, { line: C.red, fontSize: 12, bold: true });
  arrow(s, 4.1, 2.05, 4.95, 2.05, C.red);
  arrow(s, 8.35, 2.05, 9.2, 2.05, C.red);
  label(s, 0.75, 3.35, 5.2, 0.35, "보완 코드", { bold: true, fontSize: 14 });
  box(s, 0.75, 3.85, 5.25, 1.2, "v_faulted_this_cycle\n이번 클럭에 소비된 faulted tlast를 변수로 보존", C.greenSoft, { line: C.green, fontSize: 12 });
  box(s, 6.45, 3.85, 5.6, 1.2, "s_row_done_faulted_r <= '0'\n정상 클럭 기본 clear로 1클럭 pulse 계약 복구", C.greenSoft, { line: C.green, fontSize: 12 });
  label(s, 0.75, 5.8, 11.8, 0.32, "근거: tdc_gpx_face_assembler.vhd:543, :580..597, :896", { fontSize: 11, color: C.muted, align: "center" });
}

// Slide 4
{
  const s = pptx.addSlide();
  header(s, "Timing / Pipeline 영향", "데이터 path latency, throughput, II 변화 없음");
  box(s, 0.75, 1.55, 2.25, 0.8, "T0\ncell final beat\nvalid/tready/tlast/tuser", C.blueSoft, { line: C.blue, fontSize: 9.5 });
  box(s, 3.55, 1.55, 2.25, 0.8, "T0 edge\nsame-cycle fault OR", C.greenSoft, { line: C.green, fontSize: 10 });
  box(s, 6.35, 1.55, 2.25, 0.8, "T0/T1\nrow_done_faulted\n1clk pulse", C.greenSoft, { line: C.green, fontSize: 10 });
  box(s, 9.15, 1.55, 2.9, 0.8, "stream path\nheader SOF tuser unchanged", C.slateSoft, { fontSize: 10 });
  arrow(s, 3.0, 1.95, 3.55, 1.95);
  arrow(s, 5.8, 1.95, 6.35, 1.95);
  arrow(s, 8.6, 1.95, 9.15, 1.95);
  const rows = [
    ["Latency", "데이터 beat latency 변화 없음"],
    ["Throughput", "tvalid/tready backpressure 경로 변화 없음"],
    ["Pipeline", "face_assembler 내부 sequential 판정만 보완"],
    ["II", "신규 wait state 없음. row/frame 시작 간격 영향 없음"],
  ];
  rows.forEach((r, i) => {
    const y = 3.25 + i * 0.58;
    box(s, 1.0, y, 2.0, 0.42, r[0], C.white, { fontSize: 10, bold: true });
    box(s, 3.15, y, 8.9, 0.42, r[1], i % 2 ? C.white : C.slateSoft, { fontSize: 10, align: "left" });
  });
}

// Slide 5
{
  const s = pptx.addSlide();
  header(s, "검증 결과와 다음 단계", "xsim evidence 기준으로 OP-C02-04 close 가능");
  box(s, 0.75, 1.45, 5.5, 1.1, "cell_pipe boundary\nPASS\nxsim_cell_pipe_tuser.log:28", C.greenSoft, { line: C.green, bold: true, fontSize: 13 });
  box(s, 7.0, 1.45, 5.5, 1.1, "output_stage boundary\nScenario 1/2 PASS\nxsim_output_stage_tuser.log:42, :56", C.greenSoft, { line: C.green, bold: true, fontSize: 12.5 });
  label(s, 0.9, 3.25, 11.7, 0.4, "남은 Open Item", { fontSize: 15, bold: true });
  box(s, 1.0, 3.85, 5.15, 0.85, "OP-C02-05\ntiming legality illegal matrix", C.amberSoft, { line: C.amber, fontSize: 12 });
  box(s, 7.0, 3.85, 5.15, 0.85, "OP-C02-06\nstale ready negative test", C.amberSoft, { line: C.amber, fontSize: 12 });
  label(s, 0.75, 5.75, 11.8, 0.35, "추천: 다음 단계는 OP-C02-05 timing legality matrix 검증", { fontSize: 13, bold: true, align: "center" });
}

pptx.writeFile({
  fileName: path.join(__dirname, "C02_Chip_Acquisition_260430224233_Downstream_TUSER_Boundary_Fix_v001.pptx"),
});
