const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/.pnpm/pptxgenjs@4.0.1/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C07_System_Integration/C07_System_Integration_260515162050_Marker_Audit_Result_v001.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C07 Marker Audit Result";
pptx.title = "C07 Marker Audit Result v001";
pptx.lang = "ko-KR";
pptx.theme = { headFontFace: "Malgun Gothic", bodyFontFace: "Malgun Gothic", lang: "ko-KR" };

const font = "Malgun Gothic";
const C = {
  bg: "F8FAFC", ink: "111827", muted: "64748B", line: "CBD5E1",
  dark: "0F172A", white: "FFFFFF", blue: "2563EB", blueFill: "DBEAFE",
  green: "15803D", greenFill: "DCFCE7", orange: "C2410C", orangeFill: "FFEDD5",
  purple: "6D28D9", purpleFill: "EDE9FE", teal: "0F766E", tealFill: "CCFBF1",
  slate: "E2E8F0",
};

function tx(slide, value, x, y, w, h, opt = {}) {
  slide.addText(value, {
    x, y, w, h, fontFace: font, fontSize: opt.size || 10,
    bold: !!opt.bold, color: opt.color || C.ink,
    align: opt.align || "left", valign: "mid",
    fit: "shrink", margin: opt.margin === undefined ? 0.04 : opt.margin,
  });
}
function title(slide, main, sub) {
  slide.background = { color: C.bg };
  tx(slide, main, 0.55, 0.28, 12.1, 0.46, { size: 20.5, bold: true });
  tx(slide, sub, 0.58, 0.78, 12.0, 0.3, { size: 9.0, color: C.muted });
  slide.addShape(pptx.ShapeType.line, { x: 0.58, y: 1.13, w: 12.1, h: 0, line: { color: C.line, width: 0.8 } });
}
function footer(slide, value) {
  tx(slide, value, 0.58, 7.05, 12.15, 0.24, { size: 7.2, color: C.muted, align: "center" });
}
function box(slide, label, x, y, w, h, opt = {}) {
  slide.addShape(pptx.ShapeType.roundRect, {
    x, y, w, h, rectRadius: 0.04,
    fill: { color: opt.fill || C.white },
    line: { color: opt.line || C.line, width: opt.width || 0.9 },
  });
  tx(slide, label, x + 0.08, y + 0.06, w - 0.16, h - 0.12, {
    size: opt.size || 9, bold: opt.bold, color: opt.color || C.ink, align: opt.align || "center",
  });
}
function arrow(slide, x1, y1, x2, y2, color = C.muted) {
  slide.addShape(pptx.ShapeType.line, { x: x1, y: y1, w: x2 - x1, h: y2 - y1, line: { color, width: 1.15, endArrowType: "triangle" } });
}
function table(slide, rows, x, y, widths, rowH, size = 7.0) {
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
  tx(s, "C07 Marker Audit", 0.72, 0.75, 7.5, 0.42, { size: 18, bold: true, color: "93C5FD" });
  tx(s, "PASS marker를\n4단계로 다시 닫기", 0.72, 1.28, 8.7, 1.05, { size: 30, bold: true, color: C.white });
  tx(s, "C06 v003 false positive 패턴을 막기 위해 C02/C04의 과거 PASS 항목을 Source, Destination, Effect, Output 기준으로 재점검했습니다.", 0.78, 2.78, 11.6, 0.55, { size: 13, color: "E5E7EB" });
  box(s, "C02 tuser\nPASS_WITH_TRACE", 0.95, 4.15, 2.5, 0.9, { fill: "1E3A8A", line: "93C5FD", color: C.white, bold: true, size: 11.5 });
  box(s, "C04 Hit[16]\nSanitize PASS", 3.85, 4.15, 2.5, 0.9, { fill: "14532D", line: "86EFAC", color: C.white, bold: true, size: 11.5 });
  box(s, "Width beat/tlast\nPASS", 6.75, 4.15, 2.5, 0.9, { fill: "4C1D95", line: "C4B5FD", color: C.white, bold: true, size: 11.5 });
  box(s, "8us reserve\nOPEN", 9.65, 4.15, 2.35, 0.9, { fill: "713F12", line: "FDE68A", color: C.white, bold: true, size: 11.5 });
  footer(s, "Doc/TDC-GPX-Datasheet.pdf 기준, 문서 생성 2026-05-15 16:20:50 KST");
}

{
  const s = pptx.addSlide();
  title(s, "Audit Rule", "PASS는 source만으로 닫지 않고 destination, effect, output까지 관측해야 합니다.");
  const xs = [1.0, 3.55, 6.1, 8.65];
  const labels = [
    ["Source\n입력/명령/의미 발생", C.blueFill, C.blue],
    ["Destination\n다음 stage 수신", C.tealFill, C.teal],
    ["Effect\n상태/데이터 변화", C.purpleFill, C.purple],
    ["Output\n최종 stream/status", C.greenFill, C.green],
  ];
  labels.forEach(([label, fill, line], i) => {
    box(s, label, xs[i], 2.2, 1.95, 0.9, { fill, line, bold: true, size: 10.2 });
    if (i < labels.length - 1) arrow(s, xs[i] + 1.98, 2.65, xs[i + 1] - 0.08, 2.65);
  });
  box(s, "판정 기준: 네 marker가 모두 분리되어 있으면 PASS, source/output만 있고 중간 effect가 없으면 false-positive risk로 남깁니다.", 1.0, 4.65, 11.3, 0.75, { fill: C.orangeFill, line: C.orange, bold: true, size: 12.1 });
  footer(s, "C06 v003 force_reinit false positive의 재발 방지 규칙");
}

{
  const s = pptx.addSlide();
  title(s, "A. C02 downstream tuser", "faulted cell tuser와 final SOF tuser의 의미가 섞이지 않는지 확인했습니다.");
  const xs = [0.8, 3.05, 5.3, 7.55, 9.8];
  const labels = ["faulted\ndrain_done", "cell tlast\n+tuser", "row_done\nfault latch", "SOF tuser\nseparated", "xsim\nPASS"];
  labels.forEach((label, i) => {
    box(s, label, xs[i], 2.0, 1.55, 0.8, { fill: i === 4 ? C.greenFill : C.blueFill, line: i === 4 ? C.green : C.blue, bold: true, size: 9.2 });
    if (i < labels.length - 1) arrow(s, xs[i] + 1.58, 2.4, xs[i + 1] - 0.08, 2.4);
  });
  table(s, [
    ["Marker", "Evidence"],
    ["Source", "tb_tdc_gpx_cell_pipe.vhd:166, :172, :228"],
    ["Destination", "cell_pipe checker + face_assembler tlast/tuser latch"],
    ["Effect", "tdc_gpx_face_assembler.vhd row_done_faulted_r"],
    ["Output", "xsim_cell_pipe_tuser.log:28, output_stage_tuser.log:42/:56"],
  ], 1.0, 4.05, [2.0, 9.35], 0.45, 7.5);
  footer(s, "판정: PASS_WITH_TRACE, C07 fresh archive 재패키징은 optional P2");
}

{
  const s = pptx.addSlide();
  title(s, "B. Hit[16] final sanitize", "이번 generation의 final VDMA stream에서는 Hit[16]을 버리는 정책입니다.");
  const xs = [1.1, 3.8, 6.5, 9.2];
  const labels = ["cell_builder\nHit[16] metadata", "face_assembler\nmetadata beat", "sanitize\n[6:0]=0", "final AXIS\nmonitor PASS"];
  labels.forEach((label, i) => {
    box(s, label, xs[i], 2.05, 1.85, 0.85, { fill: i >= 2 ? C.greenFill : C.purpleFill, line: i >= 2 ? C.green : C.purple, bold: true, size: 9.1 });
    if (i < labels.length - 1) arrow(s, xs[i] + 1.88, 2.48, xs[i + 1] - 0.08, 2.48);
  });
  table(s, [
    ["근거", "위치"],
    ["Hit[16] 생성", "tdc_gpx_cell_builder.vhd:79, :403, :428"],
    ["sanitize 구현", "tdc_gpx_face_assembler.vhd:826-831"],
    ["final monitor", "tb_tdc_gpx_output_stage.vhd:350-352, :803-804"],
  ], 1.05, 4.15, [2.55, 8.75], 0.5, 7.8);
  footer(s, "주의: sanitize PASS는 16-bit hit slot 정책 구현 확인이며, 거리/wrap 계약은 C07 P1-03에서 별도 고정");
}

{
  const s = pptx.addSlide();
  title(s, "C. Width beat/tlast", "넓은 bus는 output packing beat 수를 줄이지만, packing 경계 때문에 계단형으로 반영됩니다.");
  table(s, [
    ["Width", "max_hits", "Beats/lane", "TLAST", "T2->T5 worst"],
    ["32", "1 / 3 / 5 / 7", "112 / 144 / 176 / 208", "4", "0.620 us"],
    ["64", "1 / 3 / 5 / 7", "88 / 88 / 120 / 120", "4", "0.540 us"],
    ["128", "1 / 3 / 5 / 7", "76 / 76 / 76 / 76", "4", "0.500 us"],
  ], 0.75, 1.55, [1.2, 2.35, 3.65, 1.15, 2.95], 0.58, 8.0);
  box(s, "C07 fresh evidence: 32/64/128 x max_hits 1/3/5/7 x 2 faces x bounded backpressure PASS", 1.0, 5.25, 11.3, 0.72, { fill: C.greenFill, line: C.green, bold: true, size: 12 });
  footer(s, "Archive: sim_results/vivado_xsim/sessions/260514153208_c07_v001_chain_stress/");
}

{
  const s = pptx.addSlide();
  title(s, "Timing / Pipeline / II", "이번 audit는 RTL 변경이 없으므로 timing 수치는 바꾸지 않습니다.");
  table(s, [
    ["항목", "영향", "판단"],
    ["Latency", "변경 없음", "sanitize는 기존 register path의 bit clear"],
    ["Throughput", "변경 없음", "width 이득은 C07 chain stress beat 감소로 확인"],
    ["Pipeline", "변경 없음", "stage 추가 없음, marker 해석만 보강"],
    ["II", "변경 없음", "reserve/VDMA 결합 판단은 P0-04"],
  ], 0.8, 1.45, [1.8, 1.9, 7.7], 0.58, 8.2);
  box(s, "P0-03은 Closed by marker audit입니다. 다음 P0는 8 us reserve 측정 또는 보수치 갱신입니다.", 1.05, 5.35, 11.15, 0.72, { fill: C.orangeFill, line: C.orange, bold: true, size: 12 });
  footer(s, "남은 P1: C03 direct matrix, C04 ready/header pending, Hit[16] SW/range 계약");
}

pptx.writeFile({ fileName: out });
