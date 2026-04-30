const pptxgen = require("pptxgenjs");
const path = require("path");

const pptx = new pptxgen();
pptx.layout = "LAYOUT_WIDE";
pptx.author = "Codex";
pptx.company = "OpenAI";
pptx.subject = "C02 fire count ownership fix";
pptx.title = "C02 Fire Count Ownership Fix v001";
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
    x: 0.45, y: 0.22, w: 12.3, h: 0.42,
    fontFace: "Malgun Gothic", fontSize: 18, bold: true,
    color: C.ink, margin: 0, fit: "shrink",
  });
  if (sub) {
    slide.addText(sub, {
      x: 0.46, y: 0.70, w: 12.2, h: 0.25,
      fontFace: "Malgun Gothic", fontSize: 8.5,
      color: C.muted, margin: 0, fit: "shrink",
    });
  }
  slide.addShape(pptx.ShapeType.line, {
    x: 0.45, y: 1.04, w: 12.42, h: 0,
    line: { color: C.line, width: 0.8 },
  });
}

function box(slide, x, y, w, h, text, fill, opts = {}) {
  slide.addShape(pptx.ShapeType.roundRect, {
    x, y, w, h,
    rectRadius: 0.05,
    fill: { color: fill },
    line: { color: opts.line || C.line, width: opts.lineWidth || 1 },
  });
  slide.addText(text, {
    x: x + 0.12, y: y + 0.08, w: w - 0.24, h: h - 0.16,
    fontFace: "Malgun Gothic",
    fontSize: opts.fontSize || 10.5,
    bold: !!opts.bold,
    color: opts.color || C.ink,
    align: opts.align || "center",
    valign: "mid",
    margin: 0.02,
    fit: "shrink",
    breakLine: false,
  });
}

function text(slide, x, y, w, h, t, opts = {}) {
  slide.addText(t, {
    x, y, w, h,
    fontFace: "Malgun Gothic",
    fontSize: opts.fontSize || 10,
    bold: !!opts.bold,
    color: opts.color || C.ink,
    align: opts.align || "left",
    valign: opts.valign || "top",
    margin: 0,
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

function pill(slide, x, y, w, h, t, fill, color = C.ink) {
  box(slide, x, y, w, h, t, fill, { fontSize: 9, color, bold: true });
}

// Slide 1
{
  const s = pptx.addSlide();
  header(s, "C02 Fire Count Ownership Fix", "2026-04-30 20:03:58 KST | 기준: TDC-GPX Datasheet + C02 계약 문서");
  text(s, 0.70, 1.38, 5.8, 0.52, "판단", { fontSize: 18, bold: true });
  text(s, 0.70, 1.92, 5.95, 1.50,
    "tlast final beat는 zero-stop을 닫는 종료 표식이다.\n실제 ownership key는 fire_count_tvalid beat의 tdata[15:0] shot count이다.",
    { fontSize: 15, color: C.ink });
  box(s, 7.05, 1.50, 2.25, 0.86, "tlast=1\nfinal marker", C.amberSoft, { line: C.amber, fontSize: 13, bold: true });
  arrow(s, 9.42, 1.73, 0.62, 0.28, C.slate);
  box(s, 10.18, 1.50, 2.45, 0.86, "tdata[15:0]=N\nownership key", C.greenSoft, { line: C.green, fontSize: 13, bold: true });
  text(s, 0.70, 4.25, 11.85, 0.9,
    "수정 방향: stop_evt count 갱신과 final 확정 모두 current face shot count와 fire_count_tdata[15:0]가 일치할 때만 수락한다.",
    { fontSize: 15, bold: true, color: C.blue });
}

// Slide 2
{
  const s = pptx.addSlide();
  header(s, "Ownership Contract", "tlast는 final 표식, tdata[15:0] 매치가 현재 shot 소유권");
  box(s, 0.60, 1.34, 2.25, 0.78, "현재 shot\nN", C.blueSoft, { line: C.blue, bold: true, fontSize: 13 });
  arrow(s, 3.05, 1.58, 0.55, 0.25, C.slate);
  box(s, 3.82, 1.20, 3.20, 1.08, "fire_count_tvalid=1\nfire_count_tdata[15:0]=N", C.greenSoft, { line: C.green, bold: true, fontSize: 12 });
  arrow(s, 7.23, 1.58, 0.55, 0.25, C.slate);
  box(s, 8.00, 1.20, 4.60, 1.08, "stop_evt 수락 또는 final 확정\nexpected_ififo / expected_final_valid 갱신", C.white, { line: C.line, bold: true, fontSize: 12 });

  box(s, 0.60, 3.10, 2.25, 0.78, "현재 shot\nN", C.blueSoft, { line: C.blue, bold: true, fontSize: 13 });
  arrow(s, 3.05, 3.34, 0.55, 0.25, C.slate);
  box(s, 3.82, 2.96, 3.20, 1.08, "fire_count_tvalid=1\nfire_count_tdata[15:0]!=N", C.redSoft, { line: C.red, bold: true, fontSize: 12 });
  arrow(s, 7.23, 3.34, 0.55, 0.25, C.slate);
  box(s, 8.00, 2.96, 4.60, 1.08, "expected count 갱신 금지\nownership/orphan sticky", C.white, { line: C.red, bold: true, fontSize: 12 });

  pill(s, 0.72, 5.34, 3.05, 0.42, "stop_evt: tlast=0 + count match", C.slateSoft, C.slate);
  pill(s, 4.02, 5.34, 3.05, 0.42, "zero-stop: tlast=1 + count match", C.amberSoft, C.amber);
  pill(s, 7.32, 5.34, 3.80, 0.42, "tlast 단독으로는 ownership 불가", C.redSoft, C.red);
}

// Slide 3
{
  const s = pptx.addSlide();
  header(s, "RTL 보완 구조", "face_seq에서 current shot count 생성, stop_cfg_decode에서 match gate");
  box(s, 0.55, 1.34, 2.15, 0.88, "face_seq\no_face_shot_count", C.blueSoft, { line: C.blue, bold: true });
  arrow(s, 2.88, 1.62, 0.50, 0.24);
  box(s, 3.55, 1.34, 2.15, 0.88, "top/config_ctrl\ni_current_fire_count", C.slateSoft, { line: C.slate, bold: true });
  arrow(s, 5.88, 1.62, 0.50, 0.24);
  box(s, 6.55, 1.34, 2.45, 0.88, "stop_cfg_decode\nfire_count match", C.greenSoft, { line: C.green, bold: true });
  arrow(s, 9.18, 1.62, 0.50, 0.24);
  box(s, 9.85, 1.34, 2.60, 0.88, "chip_run\nimmediate latch", C.white, { line: C.line, bold: true });

  box(s, 0.90, 3.36, 3.15, 0.84, "Removed\nc_EXP_LATCH_SETTLE_LAST", C.redSoft, { line: C.red, bold: true });
  arrow(s, 4.26, 3.64, 0.50, 0.24, C.slate);
  box(s, 4.98, 3.36, 3.30, 0.84, "ST_DRAIN_LATCH\nsnapshot immediately", C.greenSoft, { line: C.green, bold: true });
  arrow(s, 8.50, 3.64, 0.50, 0.24, C.slate);
  box(s, 9.22, 3.36, 3.20, 0.84, "raw FIFO depth\n6 -> 8", C.amberSoft, { line: C.amber, bold: true });

  text(s, 0.75, 5.38, 11.80, 0.62,
    "의미: CDC/settle 추정값을 기다리지 않고, upstream이 제공한 shot count 계약을 직접 사용한다.",
    { fontSize: 14, bold: true, color: C.ink });
}

// Slide 4
{
  const s = pptx.addSlide();
  header(s, "Timing Block", "200 MHz testbench 관측 기준");
  const y = 1.55;
  line(s, 0.75, y, 11.7, 0, C.slate, 1.2);
  const xs = [0.90, 2.45, 4.20, 5.95, 7.45, 9.10, 10.95];
  const labs = [
    "shot_start\ncount=N",
    "stop_evt\nvalid,data=N",
    "final\nlast=1,data=N",
    "IrFlag",
    "ST_DRAIN_LATCH\n0 wait",
    "raw FIFO\naccept",
    "drain_done"
  ];
  xs.forEach((x, i) => {
    line(s, x, y - 0.10, 0, 0.40, C.blue, 1.5);
    box(s, x - 0.52, y + 0.32, 1.05, 0.88, labs[i], i === 4 ? C.greenSoft : C.white, {
      line: i === 4 ? C.green : C.line,
      fontSize: 8.5,
      bold: i === 4,
    });
  });
  pill(s, 1.28, 3.32, 3.35, 0.48, "count 갱신: valid beat의 shot count match", C.greenSoft, C.green);
  pill(s, 4.72, 3.32, 2.35, 0.48, "final: tlast + same count", C.amberSoft, C.amber);
  pill(s, 7.22, 3.32, 3.15, 0.48, "blind wait 제거: latch 1 cycle path", C.blueSoft, C.blue);
  text(s, 0.90, 5.18, 11.5, 0.9,
    "검증 관측: zero-stop output_done=7 clk. bounded raw backpressure에서는 first_data=40 clk, run_complete=152 clk, output_done=153 clk.",
    { fontSize: 14, bold: true, color: C.ink });
}

// Slide 5
{
  const s = pptx.addSlide();
  header(s, "검증 결과", "Vivado xsim 결과 요약");
  const rows = [
    ["stop_cfg_decode", "ALL TESTS PASSED", "mismatch count 차단, zero-stop final match"],
    ["chip_ctrl", "ALL TESTS PASSED", "total_raw_words=258, raw/drop fatal clean"],
    ["bounded backpressure", "PASS", "20 data words exact, no empty IFIFO read"],
    ["top analysis", "PASS", "tdc_gpx_top VHDL analysis 완료"],
    ["config elab", "환경 이슈", "CSR black box, XPM glbl 미바인딩"]
  ];
  let y = 1.24;
  box(s, 0.70, y, 2.60, 0.42, "범위", C.slateSoft, { bold: true, fontSize: 10 });
  box(s, 3.30, y, 2.35, 0.42, "결과", C.slateSoft, { bold: true, fontSize: 10 });
  box(s, 5.65, y, 6.95, 0.42, "핵심 확인", C.slateSoft, { bold: true, fontSize: 10 });
  y += 0.48;
  rows.forEach((r) => {
    const fill = r[1].startsWith("ALL") || r[1] === "PASS" ? C.greenSoft : C.amberSoft;
    box(s, 0.70, y, 2.60, 0.54, r[0], C.white, { fontSize: 9.5 });
    box(s, 3.30, y, 2.35, 0.54, r[1], fill, { fontSize: 9.5, bold: true });
    box(s, 5.65, y, 6.95, 0.54, r[2], C.white, { fontSize: 9.5, align: "left" });
    y += 0.60;
  });
  text(s, 0.78, 5.25, 11.7, 0.7,
    "Latency/II: zero-stop 7 clk, bounded case first_data 40 clk / output_done 153 clk / II_min 1 clk / II_max 14 clk",
    { fontSize: 13, bold: true, color: C.blue });
}

// Slide 6
{
  const s = pptx.addSlide();
  header(s, "결론", "C02 expected count는 fire count ownership 계약으로 닫음");
  box(s, 0.78, 1.26, 3.55, 1.05, "1\nfire_count_tdata[15:0]가\n현재 shot count와 일치해야 함", C.greenSoft, { line: C.green, bold: true, fontSize: 12 });
  box(s, 4.90, 1.26, 3.55, 1.05, "2\nstop_evt는 같은 cycle의\nnon-final fire_count beat 필요", C.blueSoft, { line: C.blue, bold: true, fontSize: 12 });
  box(s, 9.02, 1.26, 3.55, 1.05, "3\ntlast=1은 final marker\nownership key 아님", C.amberSoft, { line: C.amber, bold: true, fontSize: 12 });

  box(s, 1.25, 3.35, 4.90, 0.92, "ST_DRAIN_LATCH fixed wait 제거", C.white, { line: C.line, bold: true, fontSize: 13 });
  arrow(s, 6.34, 3.63, 0.55, 0.25, C.slate);
  box(s, 7.10, 3.35, 4.90, 0.92, "raw FIFO depth 8로 early drain 흡수", C.white, { line: C.line, bold: true, fontSize: 13 });

  text(s, 0.92, 5.52, 11.60, 0.55,
    "후속 주의: echo_receiver는 stop_evt와 fire_count beat를 같은 cycle로 정렬하거나, 별도 skid/hold 계약을 제공해야 한다.",
    { fontSize: 13.5, bold: true, color: C.red });
}

pptx.writeFile({
  fileName: path.join(__dirname, "C02_Chip_Acquisition_260430200358_Fire_Count_Ownership_Fix_v001.pptx"),
});
