const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/.pnpm/pptxgenjs@4.0.1/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C07_System_Integration/C07_System_Integration_260515191624_C03_Direct_Matrix_Result_v001.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C07 C03 Direct Matrix Result";
pptx.title = "C07 C03 Direct Matrix Result v001";
pptx.lang = "ko-KR";
pptx.theme = { headFontFace: "Malgun Gothic", bodyFontFace: "Malgun Gothic", lang: "ko-KR" };

const font = "Malgun Gothic";
const C = {
  bg: "F8FAFC",
  ink: "111827",
  muted: "64748B",
  line: "CBD5E1",
  dark: "0F172A",
  white: "FFFFFF",
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
    valign: "mid",
    fit: "shrink",
    margin: opt.margin === undefined ? 0.04 : opt.margin,
    breakLine: !!opt.breakLine,
  });
}

function title(slide, main, sub) {
  slide.background = { color: C.bg };
  tx(slide, main, 0.58, 0.28, 12.1, 0.46, { size: 20.5, bold: true });
  tx(slide, sub, 0.60, 0.78, 12.0, 0.3, { size: 9.0, color: C.muted });
  slide.addShape(pptx.ShapeType.line, {
    x: 0.60, y: 1.13, w: 12.1, h: 0,
    line: { color: C.line, width: 0.8 },
  });
}

function footer(slide, value) {
  tx(slide, value, 0.58, 7.06, 12.15, 0.24, { size: 7.2, color: C.muted, align: "center" });
}

function box(slide, label, x, y, w, h, opt = {}) {
  slide.addShape(pptx.ShapeType.roundRect, {
    x, y, w, h, rectRadius: 0.04,
    fill: { color: opt.fill || C.white },
    line: { color: opt.line || C.line, width: opt.lineWidth || 0.9 },
  });
  tx(slide, label, x + 0.08, y + 0.05, w - 0.16, h - 0.10, {
    size: opt.size || 9,
    bold: opt.bold,
    color: opt.color || C.ink,
    align: opt.align || "center",
  });
}

function arrow(slide, x1, y1, x2, y2, color = C.muted) {
  slide.addShape(pptx.ShapeType.line, {
    x: x1, y: y1, w: x2 - x1, h: y2 - y1,
    line: { color, width: 1.15, endArrowType: "triangle" },
  });
}

function table(slide, rows, x, y, widths, rowH, size = 7.2) {
  rows.forEach((row, r) => {
    let curX = x;
    row.forEach((cell, c) => {
      slide.addShape(pptx.ShapeType.rect, {
        x: curX, y: y + r * rowH, w: widths[c], h: rowH,
        fill: { color: r === 0 ? C.slate : C.white },
        line: { color: C.line, width: 0.45 },
      });
      tx(slide, cell, curX + 0.05, y + r * rowH + 0.02, widths[c] - 0.10, rowH - 0.04, {
        size,
        bold: r === 0,
        align: c === 0 ? "center" : "left",
      });
      curX += widths[c];
    });
  });
}

{
  const s = pptx.addSlide();
  s.background = { color: C.dark };
  tx(s, "C07 / CHAIN-P1-01", 0.72, 0.72, 6.0, 0.35, { size: 16, bold: true, color: "93C5FD" });
  tx(s, "C03 direct matrix가 닫혔습니다", 0.72, 1.22, 10.9, 0.9, { size: 30, bold: true, color: C.white });
  tx(s, "C04 통합 PASS로 우회됐던 width/max_hits, IFIFO2 timeout, dual-buffer II, drop/quarantine, slope abort를 C03 레벨에서 직접 검증했습니다.", 0.78, 2.48, 11.7, 0.7, { size: 13, color: "E5E7EB" });
  box(s, "17개 신규 xsim\nPASS", 0.95, 4.1, 2.25, 0.9, { fill: "14532D", line: "86EFAC", color: C.white, bold: true, size: 15 });
  box(s, "32/64/128\nwidth sweep", 3.55, 4.1, 2.25, 0.9, { fill: "1E3A8A", line: "93C5FD", color: C.white, bold: true, size: 12 });
  box(s, "max_hits\n1/3/5/7", 6.15, 4.1, 2.25, 0.9, { fill: "581C87", line: "D8B4FE", color: C.white, bold: true, size: 12 });
  box(s, "P1-01\nClosed", 8.75, 4.1, 2.25, 0.9, { fill: "064E3B", line: "5EEAD4", color: C.white, bold: true, size: 14 });
  footer(s, "Archive: sim_results/vivado_xsim/sessions/260515191624_c07_v001_c03_direct_matrix/");
}

{
  const s = pptx.addSlide();
  title(s, "검증 경계", "이번 검증은 top 통합이 아니라 C03 내부 FSM과 output serializer를 직접 자극합니다.");
  const nodes = [
    ["C02 decoded event\ntdata 32 / tuser 16", 0.7, 2.2, 2.05, C.blueFill, C.blue],
    ["Collect FSM\nACTIVE / DROP /\nQUARANTINE", 3.15, 2.05, 2.0, C.tealFill, C.teal],
    ["Dual cell buffer\nBUF0 / BUF1", 5.55, 2.2, 2.0, C.greenFill, C.green],
    ["Output FSM\nLOAD / ACTIVE /\nWAIT_IFIFO2", 7.95, 2.05, 2.15, C.purpleFill, C.purple],
    ["C04 cell stream\n32 / 64 / 128 bit", 10.6, 2.2, 2.05, C.orangeFill, C.orange],
  ];
  nodes.forEach(([label, x, y, w, fill, line]) => box(s, label, x, y, w, 0.95, { fill, line, bold: true, size: 9.5 }));
  arrow(s, 2.78, 2.68, 3.08, 2.68);
  arrow(s, 5.18, 2.68, 5.48, 2.68);
  arrow(s, 7.58, 2.68, 7.88, 2.68);
  arrow(s, 10.13, 2.68, 10.53, 2.68);
  table(s, [
    ["TB", "직접 확인"],
    ["tb_tdc_gpx_cell_builder_c07_direct", "width/max_hits, IFIFO2 timeout, dual-buffer II, drop/quarantine"],
    ["tb_tdc_gpx_cell_pipe_c03_fix", "Hit[16] metadata, input skid, per-slope abort"],
  ], 1.0, 4.55, [3.2, 8.3], 0.52, 7.4);
  footer(s, "근거: tdc_gpx_cell_builder.vhd collect/output FSM, tdc_gpx_pkg.vhd fn_beats_per_cell_rt()");
}

{
  const s = pptx.addSlide();
  title(s, "Width / Max Hits Matrix", "내부 raw/event 의미는 고정이고, output serializer의 beat 수가 width에 따라 줄어듭니다.");
  table(s, [
    ["max_hits_cfg", "32-bit", "64-bit", "128-bit", "결과"],
    ["1", "2 clk/cell", "2 clk/cell", "2 clk/cell", "PASS"],
    ["3", "3 clk/cell", "2 clk/cell", "2 clk/cell", "PASS"],
    ["5", "4 clk/cell", "3 clk/cell", "2 clk/cell", "PASS"],
    ["7", "5 clk/cell", "3 clk/cell", "2 clk/cell", "PASS"],
  ], 0.85, 1.55, [1.8, 2.15, 2.15, 2.15, 1.8], 0.6, 8.4);
  box(s, "검증 항목: data slot packing / metadata last beat / hit_valid / slope_vec / hit_count / Hit[16] metadata", 1.0, 5.2, 11.3, 0.72, { fill: C.greenFill, line: C.green, bold: true, size: 11.2 });
  footer(s, "PASS marker: xsim_c07_v001_c03_matrix_w*_mh*_260515191624.log:28");
}

{
  const s = pptx.addSlide();
  title(s, "Timing / II 의미", "dual-buffer는 다음 shot을 받아주지만, 두 buffer가 모두 막히면 drop/quarantine이 정상 경계입니다.");
  box(s, "shot1 collect", 0.8, 1.8, 1.65, 0.52, { fill: C.blueFill, line: C.blue, bold: true, size: 8.8 });
  arrow(s, 2.5, 2.06, 3.05, 2.06);
  box(s, "IFIFO1 done", 3.1, 1.8, 1.55, 0.52, { fill: C.tealFill, line: C.teal, bold: true, size: 8.8 });
  arrow(s, 4.72, 2.06, 5.35, 2.06);
  box(s, "output_req", 5.4, 1.8, 1.45, 0.52, { fill: C.purpleFill, line: C.purple, bold: true, size: 8.8 });
  arrow(s, 6.92, 2.06, 7.5, 2.06);
  box(s, "first valid\n+2 clk 구조", 7.55, 1.68, 1.65, 0.74, { fill: C.orangeFill, line: C.orange, bold: true, size: 8.8 });
  box(s, "shot2 collect 가능\nwhile shot1 output stalls", 2.0, 3.15, 3.4, 0.76, { fill: C.greenFill, line: C.green, bold: true, size: 10 });
  box(s, "shot3 when BUF0/BUF1 occupied\n=> DROP -> QUARANTINE", 6.8, 3.15, 3.6, 0.76, { fill: C.redFill, line: C.red, bold: true, size: 10 });
  table(s, [
    ["경계", "판정"],
    ["dual-buffer II", "64-bit max_hits=3 stall 중 shot2 수집 PASS"],
    ["IFIFO2 timeout", "32/64/128 bit synthetic EOS + faulted tuser PASS"],
    ["drop/quarantine", "no-free-buffer shot3는 output slice 없이 clean exit PASS"],
  ], 1.0, 4.65, [2.55, 8.9], 0.52, 7.4);
  footer(s, "주의: 생산 margin은 RTL generic 기본값, TB는 상태 전이 단축을 위해 margin=8 clk 사용");
}

{
  const s = pptx.addSlide();
  title(s, "Closure", "C03 직접 검증은 닫고, 다음은 C04 ready/header pending과 Hit[16] SW/range 계약입니다.");
  table(s, [
    ["항목", "상태", "근거"],
    ["CHAIN-P1-01 C03 direct matrix", "Closed", "17개 xsim PASS + 기존 C03 regression PASS"],
    ["C07-VB-04 dual-buffer II", "Closed", "dual-buffer next-shot stall scenario PASS"],
    ["C07-VB-05 IFIFO2 wait/timeout", "Closed", "32/64/128 IFIFO2 timeout PASS"],
    ["CHAIN-P1-02 C04 ready/header pending", "Open", "다음 단계 direct TB"],
    ["CHAIN-P1-03 Hit[16] SW/range 계약", "Open", "final 16-bit policy와 range/wrap 명시 필요"],
  ], 0.72, 1.35, [3.25, 1.45, 6.75], 0.52, 7.1);
  box(s, "다음 진행 권고: C04 face_assembler ready path와 header_inserter face_start pending을 direct TB로 닫습니다.", 1.0, 5.8, 11.3, 0.62, { fill: C.blueFill, line: C.blue, bold: true, size: 11.4 });
  footer(s, "문서: C07_System_Integration_260515191624_C03_Direct_Matrix_Result_v001.md");
}

pptx.writeFile({ fileName: out });
