const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/.pnpm/pptxgenjs@4.0.1/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C06_Control_Status_Integration/C06_Control_Status_Integration_260514115000_Code_Fix_Result_v004.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C06 Code Fix Result v004";
pptx.title = "C06 Code Fix Result v004";
pptx.lang = "ko-KR";
pptx.theme = {
  headFontFace: "Malgun Gothic",
  bodyFontFace: "Malgun Gothic",
  lang: "ko-KR",
};

const font = "Malgun Gothic";
const C = {
  bg: "F8FAFC",
  ink: "111827",
  muted: "64748B",
  line: "CBD5E1",
  white: "FFFFFF",
  dark: "101820",
  blue: "2563EB",
  blueFill: "DBEAFE",
  green: "15803D",
  greenFill: "DCFCE7",
  orange: "C2410C",
  orangeFill: "FFEDD5",
  red: "B91C1C",
  redFill: "FEE2E2",
  purple: "6D28D9",
  purpleFill: "EDE9FE",
  teal: "0F766E",
  tealFill: "CCFBF1",
  slate: "E2E8F0",
};

function tx(slide, value, x, y, w, h, opt = {}) {
  slide.addText(value, {
    x, y, w, h,
    fontFace: font,
    fontSize: opt.size || 10,
    bold: !!opt.bold,
    color: opt.color || C.ink,
    align: opt.align || "left",
    valign: opt.valign || "mid",
    fit: "shrink",
    margin: opt.margin === undefined ? 0.04 : opt.margin,
    breakLine: false,
  });
}

function title(slide, main, sub) {
  slide.background = { color: C.bg };
  tx(slide, main, 0.55, 0.28, 12.1, 0.46, { size: 21, bold: true });
  tx(slide, sub, 0.58, 0.78, 12.0, 0.3, { size: 9.4, color: C.muted });
  slide.addShape(pptx.ShapeType.line, {
    x: 0.58, y: 1.13, w: 12.1, h: 0,
    line: { color: C.line, width: 0.8 },
  });
}

function footer(slide, value) {
  tx(slide, value, 0.58, 7.05, 12.15, 0.24, {
    size: 7.4, color: C.muted, align: "center",
  });
}

function box(slide, label, x, y, w, h, opt = {}) {
  slide.addShape(pptx.ShapeType.roundRect, {
    x, y, w, h,
    rectRadius: 0.04,
    fill: { color: opt.fill || C.white },
    line: { color: opt.line || C.line, width: opt.width || 1 },
  });
  tx(slide, label, x + 0.08, y + 0.06, w - 0.16, h - 0.12, {
    size: opt.size || 10,
    bold: opt.bold,
    color: opt.color || C.ink,
    align: opt.align || "center",
  });
}

function pill(slide, label, x, y, w, fill, line, color = C.ink) {
  slide.addShape(pptx.ShapeType.roundRect, {
    x, y, w, h: 0.36,
    rectRadius: 0.07,
    fill: { color: fill },
    line: { color: line, width: 0.8 },
  });
  tx(slide, label, x + 0.04, y + 0.04, w - 0.08, 0.26, {
    size: 8.6, bold: true, color, align: "center",
  });
}

function arrow(slide, x1, y1, x2, y2, color = C.muted, width = 1.3) {
  slide.addShape(pptx.ShapeType.line, {
    x: x1, y: y1, w: x2 - x1, h: y2 - y1,
    line: { color, width, endArrowType: "triangle" },
  });
}

function table(slide, rows, x, y, widths, rowH, size = 8.0) {
  rows.forEach((row, r) => {
    let curX = x;
    row.forEach((cell, c) => {
      slide.addShape(pptx.ShapeType.rect, {
        x: curX, y: y + r * rowH, w: widths[c], h: rowH,
        fill: { color: r === 0 ? C.slate : C.white },
        line: { color: C.line, width: 0.45 },
      });
      tx(slide, cell, curX + 0.05, y + r * rowH + 0.02, widths[c] - 0.10, rowH - 0.04, {
        size, bold: r === 0, align: c === 0 ? "center" : "left",
      });
      curX += widths[c];
    });
  });
}

{
  const s = pptx.addSlide();
  s.background = { color: C.dark };
  tx(s, "C06", 0.72, 0.56, 1.25, 0.42, { size: 15, bold: true, color: "93C5FD" });
  tx(s, "Control / Status Integration", 0.72, 1.10, 10.3, 0.56, {
    size: 27, bold: true, color: C.white,
  });
  tx(s, "Code Fix Result v004", 0.76, 1.74, 4.6, 0.35, {
    size: 15, color: "CBD5E1",
  });
  tx(s, "soft_reset과 force_reinit recovery가 모두 실제 TDC 도메인까지 도달하고, run-to-run output stream을 보존했습니다.", 0.78, 2.42, 11.0, 0.55, {
    size: 13.0, color: C.white,
  });
  box(s, "PASS\nforce_reinit", 0.86, 3.60, 2.35, 0.95, {
    fill: "14532D", line: "86EFAC", color: C.white, bold: true,
  });
  box(s, "PASS\nsoft_reset", 3.72, 3.60, 2.35, 0.95, {
    fill: "14532D", line: "86EFAC", color: C.white, bold: true,
  });
  box(s, "PASS\n32/64/128 baseline", 6.58, 3.60, 2.70, 0.95, {
    fill: "1E3A8A", line: "93C5FD", color: C.white, bold: true, size: 9.0,
  });
  box(s, "NEXT\nhandoff 갱신", 9.72, 3.60, 2.35, 0.95, {
    fill: "713F12", line: "FDE68A", color: C.white, bold: true,
  });
  footer(s, "C06 Code Fix Result v004 | 생성/수정 2026-05-14 11:50 KST | 실행 stamp 260514115000");
}

{
  const s = pptx.addSlide();
  title(s, "원인과 보완", "단순한 soft reset 대기가 아니라, recovery pulse 보존과 recovery boundary가 핵심이었습니다.");
  const xs = [0.75, 3.20, 5.65, 8.10, 10.55];
  const labels = [
    ["AXI 1-cycle\ncommand", C.orangeFill, C.orange],
    ["기존 pulse CDC\n누락 가능", C.redFill, C.red],
    ["toggle CDC\n보존", C.blueFill, C.blue],
    ["chip/face/status\nboundary reset", C.purpleFill, C.purple],
    ["next run\nstream PASS", C.greenFill, C.green],
  ];
  labels.forEach(([label, fill, line], i) => {
    box(s, label, xs[i], 2.0, 1.55, 1.05, { fill, line, bold: true, size: 8.8 });
    if (i < labels.length - 1) arrow(s, xs[i] + 1.60, 2.52, xs[i + 1] - 0.08, 2.52);
  });
  table(s, [
    ["원인", "v004 보완", "코드 근거"],
    ["source pulse는 보였지만 TDC pulse가 누락될 수 있음", "soft/force command를 toggle CDC로 변경", "tdc_gpx_config_ctrl.vhd:1172, :1189"],
    ["force_reinit이 sub-FSM/face state를 완전히 닫지 못함", "sub reset 및 top recovery reset 추가", "tdc_gpx_chip_ctrl.vhd:490, tdc_gpx_top.vhd:416"],
    ["soft_reset 후 다음 run 계약이 닫히지 않음", "PH_RESP_DRAIN 후 PH_INIT 재진입 검증", "soft log :148, :156, :244"],
  ], 0.72, 4.04, [1.45, 5.15, 4.85], 0.56, 7.4);
  footer(s, "Datasheet 기준 GPX bus timing은 변경하지 않고, C06 내부 control/status recovery boundary만 보완");
}

{
  const s = pptx.addSlide();
  title(s, "Recovery Timing", "200 MHz 기준 source pulse에서 TDC pulse까지 20 ns, 4 clk로 관측되었습니다.");
  const y = 2.05;
  const points = [
    ["T8 marker\n43.5875 us", 0.85, C.slate, C.muted],
    ["source pulse\n43.6575 us", 3.10, C.orangeFill, C.orange],
    ["TDC pulse\n43.6775 us", 5.35, C.blueFill, C.blue],
    ["chip action\n43.6775 us", 7.60, C.purpleFill, C.purple],
    ["run #2 PASS", 9.85, C.greenFill, C.green],
  ];
  points.forEach(([label, x, fill, line], i) => {
    box(s, label, x, y, 1.55, 0.95, { fill, line, bold: true, size: 8.4 });
    if (i < points.length - 1) arrow(s, x + 1.60, y + 0.48, points[i + 1][1] - 0.08, y + 0.48);
  });
  pill(s, "source -> TDC = 20 ns / 4 clk", 3.65, 3.35, 2.55, C.blueFill, C.blue);
  pill(s, "soft: TDC -> PH_INIT = 20 ns / 4 clk", 6.55, 3.35, 3.05, C.purpleFill, C.purple);
  table(s, [
    ["경로", "force_reinit", "soft_reset", "판단"],
    ["T8 marker", "43.5875 us", "43.5875 us", "command write 시작"],
    ["source pulse", "43.6575 us", "43.6575 us", "AXI-domain pulse"],
    ["TDC pulse", "43.6775 us", "43.6775 us", "CDC 보존 확인"],
    ["chip action", "43.6775 us", "43.6775 us", "동일 edge에서 phase 전이"],
  ], 0.76, 4.25, [1.55, 2.3, 2.3, 5.25], 0.42, 7.5);
  footer(s, "근거: xsim_c06_v004_top_int_force/soft_260514115000.log :142~:156");
}

{
  const s = pptx.addSlide();
  title(s, "Pipeline / State 관계", "Recovery는 run 내부 pipeline이 아니라 run-to-run control boundary에만 영향을 줍니다.");
  const row1 = [
    ["AXI CSR", 0.70, 1.85, C.orangeFill, C.orange],
    ["config_ctrl\nsource toggle", 2.55, 1.85, C.blueFill, C.blue],
    ["3-stage sync\nTDC pulse", 4.65, 1.85, C.blueFill, C.blue],
    ["chip_ctrl\nPH_INIT", 6.75, 1.85, C.purpleFill, C.purple],
    ["face/status\nreset", 8.85, 1.85, C.tealFill, C.teal],
    ["next START\naccepted", 10.95, 1.85, C.greenFill, C.green],
  ];
  row1.forEach(([label, x, y, fill, line], i) => {
    box(s, label, x, y, 1.45, 0.95, { fill, line, bold: true, size: 8.2 });
    if (i < row1.length - 1) arrow(s, x + 1.48, y + 0.48, row1[i + 1][1] - 0.05, y + 0.48);
  });
  tx(s, "Boundary 원칙", 0.80, 3.45, 2.0, 0.28, { size: 12, bold: true });
  tx(s, "CDC register, chip phase register, top-level recovery reset으로 경계를 닫아 조합 path를 길게 만들지 않습니다.", 0.80, 3.82, 11.4, 0.42, { size: 11 });
  table(s, [
    ["항목", "결과", "의미"],
    ["Latency", "T0->T1/T2/T5/T3 동일", "steady-state shot pipeline 보존"],
    ["Throughput", "44 beats/run, 88 beats total", "recovery 후 stream 총량 보존"],
    ["II", "shot-to-shot 2107 clk", "TB shot period 기준, recovery 영향 없음"],
  ], 0.82, 4.70, [1.65, 4.05, 5.65], 0.48, 8.0);
  footer(s, "C06 수정은 GPX external timing이 아니라 control/status recovery pipeline을 보완");
}

{
  const s = pptx.addSlide();
  title(s, "Latency / Throughput / II", "숫자는 TB marker 기준이며, 순수 RTL 최소 지연과 TB sequence 지연을 분리해서 봅니다.");
  table(s, [
    ["측정", "값", "판단"],
    ["T0 -> T1 fire_count final", "11 clk / 55 ns", "recovery 전후 동일"],
    ["T0 -> T2 IrFlag assert", "92 clk / 460 ns", "I-Mode single TB timing 동일"],
    ["T0 -> T5 TLAST", "192 또는 189 clk", "stream drain latency 보존"],
    ["T0 -> T3 drain wait end", "1092 clk / 5.460 us", "drain window 보존"],
    ["shot-to-shot II", "2107 clk / 10.535 us", "TB shot period 기준"],
    ["force T8 -> next T0", "2993 clk / 14.965 us", "status/read/reinit 포함"],
    ["soft T8 -> next T0", "14913 clk / 74.565 us", "TB 12000 clk 대기 포함"],
  ], 0.72, 1.48, [3.35, 3.05, 5.0], 0.52, 8.2);
  pill(s, "steady-state pipeline unchanged", 0.96, 6.18, 2.85, C.greenFill, C.green);
  pill(s, "recovery affects run-to-run interval only", 4.05, 6.18, 3.25, C.blueFill, C.blue);
  pill(s, "Datasheet GPX timing unchanged", 7.55, 6.18, 3.05, C.orangeFill, C.orange);
  footer(s, "근거: C06_MARKER T0/T1/T2/T3/T5/T8 in xsim_c06_v004_top_int_*_260514115000.log");
}

{
  const s = pptx.addSlide();
  title(s, "검증 완료와 다음 단계", "C06 recovery는 닫혔고, 다음은 handoff 문서 갱신과 release 전 로그 정책 결정입니다.");
  table(s, [
    ["검증", "결과", "로그"],
    ["face_seq/status_agg baseline", "PASS", "v002 face/status logs"],
    ["top width 32/64/128", "PASS", "v002 top_int_w32/w64/w128 logs"],
    ["64-bit backpressure", "PASS", "v002 top_int_w64_bp log"],
    ["force_reinit recovery", "PASS", "v004 force log :236"],
    ["soft_reset recovery", "PASS", "v004 soft log :244"],
  ], 0.72, 1.45, [3.3, 1.55, 6.05], 0.52, 8.0);
  const next = [
    ["1", "C06 handoff 갱신"],
    ["2", "v004 script 공식 regression 지정"],
    ["3", "simulation-only report 유지 정책 결정"],
    ["4", "다음 cluster 진입 판단"],
  ];
  next.forEach(([n, text], i) => {
    pill(s, `${n}. ${text}`, 0.95 + i * 2.95, 5.42, 2.45, i < 2 ? C.blueFill : C.orangeFill, i < 2 ? C.blue : C.orange);
  });
  footer(s, "산출물: Code_Fix_Result_v004.md / Code_Fix_Plan_v005.md / scripts/run_c06_v004_recovery.ps1");
}

pptx.writeFile({ fileName: out });
