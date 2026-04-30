const pptxgen = require("pptxgenjs");
const path = require("path");

const pptx = new pptxgen();
pptx.layout = "LAYOUT_WIDE";
pptx.author = "Codex";
pptx.subject = "C02 Cluster2 readiness review";
pptx.title = "C02 Cluster2 Readiness Review v001";
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
  slateSoft: "E2E8F0",
  cyanSoft: "CFFAFE",
  cyan: "0891B2",
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
  });
}

function arrow(slide, x1, y1, x2, y2, color = C.ink, width = 1.2) {
  slide.addShape(pptx.ShapeType.line, {
    x: x1, y: y1, w: x2 - x1, h: y2 - y1,
    line: { color, width, endArrowType: "triangle" },
  });
}

function line(slide, x1, y1, x2, y2, color = C.line, width = 1) {
  slide.addShape(pptx.ShapeType.line, {
    x: x1, y: y1, w: x2 - x1, h: y2 - y1,
    line: { color, width },
  });
}

function row(slide, y, cols, fills = []) {
  const xs = [0.62, 2.95, 6.45, 9.65];
  const ws = [2.2, 3.35, 3.05, 2.95];
  cols.forEach((value, i) => box(slide, xs[i], y, ws[i], 0.43, value, fills[i] || C.white, {
    fontSize: i === 0 ? 8.7 : 8.4,
    bold: i === 0,
    align: "left",
  }));
}

{
  const s = pptx.addSlide();
  header(s, "C02 종합 판정", "작성: 2026-04-30 23:32 KST | 절대 기준: Doc/TDC-GPX-Datasheet.pdf | 상세 근거: Readiness Review v001");

  box(s, 0.85, 1.55, 3.55, 1.2, "조건부 GO\nC02 내부는 닫힘", C.greenSoft, { line: C.green, fontSize: 17, bold: true });
  box(s, 4.95, 1.55, 3.55, 1.2, "I-Mode single\n28-bit / 200 MHz", C.blueSoft, { line: C.blue, fontSize: 16, bold: true });
  box(s, 9.05, 1.55, 3.55, 1.2, "Handoff\n통합 계약 유지", C.amberSoft, { line: C.amber, fontSize: 16, bold: true });

  text(s, 0.9, 3.35, 11.8, 0.52,
    "C02는 expected-count final 계약으로 GPX IFIFO를 empty-read 없이 drain하고, timing legality / CDC / tuser / stale-ready 검증을 통과했다.",
    { fontSize: 14, bold: true, align: "center" });

  box(s, 1.0, 4.55, 3.3, 0.75, "닫힌 범위\nOP-C02-01~06 PASS", C.greenSoft, { line: C.green, fontSize: 12, bold: true });
  box(s, 5.0, 4.55, 3.3, 0.75, "상속 기준\nDatasheet READ 40 MHz", C.blueSoft, { line: C.blue, fontSize: 12, bold: true });
  box(s, 9.0, 4.55, 3.3, 0.75, "후속 항목\nfull_int packing / output CDC", C.amberSoft, { line: C.amber, fontSize: 12, bold: true });

  text(s, 0.9, 6.25, 11.8, 0.35,
    "No-Go 조건: 다음 Cluster가 실제 echo_receiver full_int end-to-end 연결을 C02 closure 필수조건으로 요구하는 경우",
    { fontSize: 10.5, color: C.red, bold: true, align: "center" });
}

{
  const s = pptx.addSlide();
  header(s, "Cluster2 Pipeline", "shot 소유권 -> expected 확정 -> GPX drain -> raw/downstream stream");

  box(s, 0.45, 1.45, 1.65, 0.68, "laser_ctrl\nfire count", C.white, { fontSize: 9.5, bold: true });
  box(s, 2.4, 1.45, 1.65, 0.68, "face_seq\nshot count", C.blueSoft, { line: C.blue, fontSize: 9.5, bold: true });
  box(s, 4.35, 1.45, 1.85, 0.68, "stop_cfg_decode\nfire match", C.blueSoft, { line: C.blue, fontSize: 9.2, bold: true });
  box(s, 6.55, 1.45, 1.7, 0.68, "expected\nCDC tuple", C.amberSoft, { line: C.amber, fontSize: 9.5, bold: true });
  box(s, 8.6, 1.45, 1.65, 0.68, "chip_run\nlatch/check", C.greenSoft, { line: C.green, fontSize: 9.5, bold: true });
  box(s, 10.55, 1.45, 1.8, 0.68, "chip_ctrl\nresponse skid", C.greenSoft, { line: C.green, fontSize: 9.5, bold: true });

  arrow(s, 2.1, 1.79, 2.4, 1.79);
  arrow(s, 4.05, 1.79, 4.35, 1.79);
  arrow(s, 6.2, 1.79, 6.55, 1.79);
  arrow(s, 8.25, 1.79, 8.6, 1.79);
  arrow(s, 10.25, 1.79, 10.55, 1.79);

  box(s, 0.85, 3.15, 2.15, 0.75, "Datasheet\nREAD timing", C.redSoft, { line: C.red, fontSize: 11, bold: true });
  box(s, 3.65, 3.15, 2.15, 0.75, "bus_phy\nRDN/D[27:0]", C.redSoft, { line: C.red, fontSize: 11, bold: true });
  box(s, 6.45, 3.15, 2.15, 0.75, "raw_cdc\nSYNC or ASYNC", C.cyanSoft, { line: C.cyan, fontSize: 11, bold: true });
  box(s, 9.25, 3.15, 2.15, 0.75, "decode/cell/output\ntuser verified", C.violetSoft, { line: C.violet, fontSize: 10.2, bold: true });

  arrow(s, 3.0, 3.52, 3.65, 3.52, C.red);
  arrow(s, 5.8, 3.52, 6.45, 3.52, C.cyan);
  arrow(s, 8.6, 3.52, 9.25, 3.52, C.violet);
  arrow(s, 11.45, 2.15, 4.75, 3.15, C.green);

  text(s, 0.8, 5.15, 11.8, 0.55,
    "중요한 경계: expected tuple은 AXIS clock에서 TDC clock으로 atomic CDC되고, raw data는 TDC clock에서 AXI-Stream clock으로 CDC된다.",
    { fontSize: 13, bold: true, align: "center" });
}

{
  const s = pptx.addSlide();
  header(s, "상태머신 관계도", "C02는 face_seq, stop_cfg_decode, chip_run, chip_ctrl가 역할을 나누어 닫는다");

  box(s, 0.7, 1.45, 2.2, 0.65, "shot_start_gated\ncurrent_fire_count", C.blueSoft, { line: C.blue, fontSize: 10, bold: true });
  box(s, 3.4, 1.45, 2.1, 0.65, "ST_CAPTURE\nrange window", C.blueSoft, { line: C.blue, fontSize: 10, bold: true });
  box(s, 6.0, 1.45, 2.2, 0.65, "expected final\nfire-count match", C.amberSoft, { line: C.amber, fontSize: 10, bold: true });
  box(s, 8.75, 1.45, 2.1, 0.65, "IrFlag\nI-Mode single end", C.redSoft, { line: C.red, fontSize: 10, bold: true });
  box(s, 11.1, 1.45, 1.55, 0.65, "LATCH", C.greenSoft, { line: C.green, fontSize: 11, bold: true });
  arrow(s, 2.9, 1.78, 3.4, 1.78);
  arrow(s, 5.5, 1.78, 6.0, 1.78);
  arrow(s, 8.2, 1.78, 8.75, 1.78);
  arrow(s, 10.85, 1.78, 11.1, 1.78);

  box(s, 1.0, 3.15, 1.8, 0.58, "DRAIN_CHECK", C.greenSoft, { line: C.green, fontSize: 9.5, bold: true });
  box(s, 3.25, 2.55, 1.65, 0.58, "EF1 READ", C.white, { fontSize: 9.5, bold: true });
  box(s, 3.25, 3.75, 1.65, 0.58, "EF2 READ", C.white, { fontSize: 9.5, bold: true });
  box(s, 5.45, 3.15, 1.8, 0.58, "SETTLE", C.amberSoft, { line: C.amber, fontSize: 9.5, bold: true });
  box(s, 7.75, 3.15, 1.8, 0.58, "DONE CTRL", C.greenSoft, { line: C.green, fontSize: 9.5, bold: true });
  box(s, 10.05, 3.15, 1.8, 0.58, "ALU / RECOVERY", C.slateSoft, { fontSize: 9.5, bold: true });

  arrow(s, 2.8, 3.44, 3.25, 2.84, C.green);
  arrow(s, 2.8, 3.44, 3.25, 4.04, C.green);
  arrow(s, 4.9, 2.84, 5.45, 3.44, C.green);
  arrow(s, 4.9, 4.04, 5.45, 3.44, C.green);
  arrow(s, 7.25, 3.44, 1.0, 3.44, C.amber);
  arrow(s, 2.8, 3.44, 7.75, 3.44, C.green);
  arrow(s, 9.55, 3.44, 10.05, 3.44, C.green);

  text(s, 0.85, 5.4, 11.6, 0.45,
    "ST_DRAIN_LATCH는 expected tuple을 snapshot한다. ST_DRAIN_SETTLE은 expected wait가 아니라 READ 후 EF/status settle guard이다.",
    { fontSize: 12.5, bold: true, align: "center" });
}

{
  const s = pptx.addSlide();
  header(s, "Timing Diagram", "Datasheet READ timing과 expected-count drain timing을 분리해서 판단");

  text(s, 0.55, 1.28, 2.2, 0.25, "Bus READ @200 MHz", { fontSize: 11, bold: true });
  const xs = [0.75, 2.25, 3.75, 5.25, 6.75, 8.25];
  ["tick0\nADR", "tick1\nRDN↓", "tick2", "tick3\nsample", "tick4\nRDN↑", "tick5\nrsp"].forEach((label, i) => {
    box(s, xs[i], 1.75, 1.05, 0.55, label, i === 1 || i === 4 ? C.redSoft : C.white, { line: i === 1 || i === 4 ? C.red : C.line, fontSize: 8.5, bold: true });
    line(s, xs[i] + 0.52, 2.34, xs[i] + 0.52, 2.78, C.line);
  });
  line(s, xs[1] + 0.52, 2.55, xs[4] + 0.52, 2.55, C.red, 2);
  text(s, 2.25, 2.82, 5.6, 0.25, "RDN low width = 15 ns, capture delay = 15 ns", { fontSize: 9.5, bold: true, align: "center", color: C.red });
  line(s, xs[0] + 0.52, 3.08, xs[5] + 0.52, 3.08, C.blue, 2);
  text(s, 2.35, 3.28, 5.7, 0.25, "Burst READ II = 25 ns = 40 MHz max", { fontSize: 9.5, bold: true, align: "center", color: C.blue });

  text(s, 0.55, 4.25, 2.8, 0.25, "Expected-count Drain", { fontSize: 11, bold: true });
  box(s, 0.8, 4.72, 1.65, 0.52, "shot_start", C.blueSoft, { line: C.blue, fontSize: 9.2, bold: true });
  box(s, 2.9, 4.72, 1.65, 0.52, "stop_evt\nrunning", C.white, { fontSize: 9, bold: true });
  box(s, 5.0, 4.72, 1.65, 0.52, "fire final\nmatch", C.amberSoft, { line: C.amber, fontSize: 9, bold: true });
  box(s, 7.1, 4.72, 1.65, 0.52, "IrFlag", C.redSoft, { line: C.red, fontSize: 9.2, bold: true });
  box(s, 9.2, 4.72, 1.65, 0.52, "LATCH", C.greenSoft, { line: C.green, fontSize: 9.2, bold: true });
  box(s, 11.3, 4.72, 1.25, 0.52, "READ N", C.greenSoft, { line: C.green, fontSize: 9.2, bold: true });
  [2.45, 4.55, 6.65, 8.75, 10.85].forEach((x) => arrow(s, x, 4.98, x + 0.45, 4.98, C.ink));
  text(s, 0.9, 5.95, 11.5, 0.38,
    "top 검증: physical=expected+1 조건에서도 read=expected, leftover=1 -> EF fallback이 아니라 count-known drain",
    { fontSize: 11.5, bold: true, align: "center" });
}

{
  const s = pptx.addSlide();
  header(s, "Latency / Throughput / II", "측정값은 bus-level과 stream-level을 분리해서 해석");

  row(s, 1.35, ["구간", "측정/계약", "값", "판단"], [C.slateSoft, C.slateSoft, C.slateSoft, C.slateSoft]);
  row(s, 1.85, ["T0->T1a", "IrFlag -> first raw valid", "16 clk", "내부 first-valid latency"]);
  row(s, 2.35, ["T0->T1b", "IrFlag -> first accepted", "40 clk", "TB backpressure 포함"]);
  row(s, 2.85, ["T0->T7", "IrFlag -> output done", "428 clk", "56 data word drain"]);
  row(s, 3.35, ["Bus READ II", "ticks*div*Tclk", "25 ns", "Datasheet 40 MHz 만족"]);
  row(s, 3.85, ["Raw II", "output accepted interval", "1..15 clk", "FIFO/skid 방출 포함"]);
  row(s, 4.35, ["Throughput", "56 word / 428 clk", "약 26.2 Mword/s", "전체 overhead 포함"]);

  box(s, 0.85, 5.35, 3.35, 0.75, "first_valid=16clk\nfirst_accept=40clk", C.blueSoft, { line: C.blue, fontSize: 11, bold: true });
  box(s, 5.0, 5.35, 3.35, 0.75, "II_min=1clk\nII_max=15clk", C.cyanSoft, { line: C.cyan, fontSize: 11, bold: true });
  box(s, 9.15, 5.35, 3.35, 0.75, "run_complete=427clk\noutput_done=428clk", C.greenSoft, { line: C.green, fontSize: 11, bold: true });
}

{
  const s = pptx.addSlide();
  header(s, "검증 완성도", "OP-C02-01~06은 PASS evidence로 닫힘");

  const items = [
    ["OP-C02-01", "forced empty/tuser negative", "CLOSE"],
    ["OP-C02-02", "PH_RESP_DRAIN stuck/fatal", "CLOSE"],
    ["OP-C02-03", "expected CDC/top integration", "CLOSE"],
    ["OP-C02-04", "downstream tuser boundary", "CLOSE"],
    ["OP-C02-05", "timing legality matrix", "CLOSE"],
    ["OP-C02-06", "stale-ready boundary", "CLOSE"],
  ];
  items.forEach((it, idx) => {
    const y = 1.35 + idx * 0.62;
    box(s, 0.75, y, 1.65, 0.45, it[0], C.slateSoft, { fontSize: 9.5, bold: true });
    box(s, 2.55, y, 5.65, 0.45, it[1], C.white, { fontSize: 9.5, align: "left" });
    box(s, 8.45, y, 1.55, 0.45, it[2], C.greenSoft, { line: C.green, fontSize: 9.5, bold: true });
  });

  box(s, 10.45, 1.35, 1.95, 0.85, "68 pass\n0 fail", C.greenSoft, { line: C.green, fontSize: 13, bold: true });
  box(s, 10.45, 2.55, 1.95, 0.85, "read=2\nleftover=1", C.greenSoft, { line: C.green, fontSize: 12.5, bold: true });
  box(s, 10.45, 3.75, 1.95, 0.85, "blocked accepts=6\norder kept", C.greenSoft, { line: C.green, fontSize: 10.5, bold: true });

  text(s, 0.9, 5.85, 11.6, 0.35,
    "상세 로그 위치: Readiness Review v001 section 9.2",
    { fontSize: 10.5, color: C.muted, align: "center", bold: true });
}

{
  const s = pptx.addSlide();
  header(s, "다음 Cluster 인계", "GO 판정과 함께 넘겨야 할 계약");

  box(s, 0.7, 1.35, 3.6, 0.88, "받아야 할 계약\nI-Mode single / 28-bit / 200 MHz", C.blueSoft, { line: C.blue, fontSize: 13, bold: true });
  box(s, 4.85, 1.35, 3.6, 0.88, "count-known drain\nfire-count final expected tuple", C.greenSoft, { line: C.green, fontSize: 13, bold: true });
  box(s, 9.0, 1.35, 3.6, 0.88, "후속 검토\noutput CDC / echo packing", C.amberSoft, { line: C.amber, fontSize: 13, bold: true });

  row(s, 3.0, ["항목", "다음 단계 처리", "C02 판정", "근거"], [C.slateSoft, C.slateSoft, C.slateSoft, C.slateSoft]);
  row(s, 3.5, ["output CDC", "C03/C04 구조 판단", "Handoff", "raw_cdc는 검증됨"]);
  row(s, 4.0, ["echo_receiver", "full_int packing 정합", "Handoff", "top/config path는 PASS"]);
  row(s, 4.5, ["OEN", "연결/High 고정 검토", "Handoff", "Low fixed 금지"]);
  row(s, 5.0, ["16-bit bus", "28-bit close 이후", "Excluded", "현재 unsupported"]);

  text(s, 0.85, 6.25, 11.7, 0.42,
    "결론: C02 내부 closure 기준으로는 다음 Cluster 진입 가능. 단, 통합 항목은 명시적으로 인계해야 한다.",
    { fontSize: 12.5, bold: true, align: "center" });
}

pptx.writeFile({
  fileName: path.join(__dirname, "C02_Chip_Acquisition_260430233213_Cluster2_Readiness_Review_v001.pptx"),
});
