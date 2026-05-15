const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/.pnpm/pptxgenjs@4.0.1/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C07_System_Integration/C07_System_Integration_260515194251_C04_Direct_Matrix_Result_v001.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C07 C04 Direct Matrix Result";
pptx.title = "C07 C04 Direct Matrix Result v001";
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
  tx(s, "C07 / CHAIN-P1-02", 0.72, 0.72, 6.0, 0.35, { size: 16, bold: true, color: "93C5FD" });
  tx(s, "C04 ready와 header pending이 닫혔습니다", 0.72, 1.22, 11.0, 0.9, { size: 29, bold: true, color: C.white });
  tx(s, "face_assembler ready boundary와 header_inserter ST_DRAIN_LAST pending을 32/64/128 bit 직접 TB로 검증했습니다.", 0.78, 2.48, 11.7, 0.7, { size: 13, color: "E5E7EB" });
  box(s, "6개 xsim\nPASS", 0.95, 4.1, 2.25, 0.9, { fill: "14532D", line: "86EFAC", color: C.white, bold: true, size: 15 });
  box(s, "32/64/128\nwidth sweep", 3.55, 4.1, 2.25, 0.9, { fill: "1E3A8A", line: "93C5FD", color: C.white, bold: true, size: 12 });
  box(s, "ready stall +\npending latch", 6.15, 4.1, 2.25, 0.9, { fill: "581C87", line: "D8B4FE", color: C.white, bold: true, size: 12 });
  box(s, "P1-02\nClosed", 8.75, 4.1, 2.25, 0.9, { fill: "064E3B", line: "5EEAD4", color: C.white, bold: true, size: 14 });
  footer(s, "Archive: sim_results/vivado_xsim/sessions/260515194251_c07_v001_c04_direct_matrix/");
}

{
  const s = pptx.addSlide();
  title(s, "검증 경계", "C04 내부 구성요소를 직접 자극해 source, effect, output을 한 번에 관측했습니다.");
  const nodes = [
    ["C03 cell stream\nchip slice input", 0.65, 2.25, 1.95, C.blueFill, C.blue],
    ["input FIFO /\nelastic FIFO", 3.0, 2.1, 1.95, C.tealFill, C.teal],
    ["face_assembler\nrow output", 5.35, 2.1, 2.0, C.greenFill, C.green],
    ["header_inserter\nprefix/data/drain", 7.8, 2.1, 2.25, C.purpleFill, C.purple],
    ["final AXIS\n32/64/128 bit", 10.55, 2.25, 1.95, C.orangeFill, C.orange],
  ];
  nodes.forEach(([label, x, y, w, fill, line]) => box(s, label, x, y, w, 0.95, { fill, line, bold: true, size: 9.3 }));
  arrow(s, 2.63, 2.66, 2.93, 2.66);
  arrow(s, 4.98, 2.66, 5.28, 2.66);
  arrow(s, 7.38, 2.66, 7.73, 2.66);
  arrow(s, 10.08, 2.66, 10.48, 2.66);
  table(s, [
    ["TB", "직접 확인"],
    ["tb_tdc_gpx_face_assembler_c07_direct", "downstream stall 중 order, TLAST, row_done, metadata sanitize"],
    ["tb_tdc_gpx_header_inserter_c07_direct", "ST_PREFIX stall, ST_DRAIN_LAST pending latch, 2-frame completion"],
  ], 1.0, 4.55, [3.4, 8.1], 0.52, 7.4);
  footer(s, "근거: tdc_gpx_face_assembler.vhd ready/FIFO boundary, tdc_gpx_header_inserter.vhd pending queue");
}

{
  const s = pptx.addSlide();
  title(s, "face_assembler Ready Boundary", "downstream stall 중에도 두 chip slice의 순서와 row boundary가 유지됐습니다.");
  table(s, [
    ["Width", "beats/chip", "total beats", "결과"],
    ["32-bit", "40", "80", "PASS"],
    ["64-bit", "24", "48", "PASS"],
    ["128-bit", "16", "32", "PASS"],
  ], 0.85, 1.45, [1.8, 2.2, 2.2, 1.8], 0.6, 8.4);
  box(s, "조건: chip0 + chip1, stops_per_chip=8, max_hits_cfg=7, 초기 2 us stall 후 7 clk ready / 3 clk stall 반복", 1.0, 4.05, 11.3, 0.62, { fill: C.blueFill, line: C.blue, bold: true, size: 10.6 });
  box(s, "확인: chip order, TLAST final-only, row_done 1회, metadata[6:0]=0, fault/abort 없음", 1.0, 4.95, 11.3, 0.62, { fill: C.greenFill, line: C.green, bold: true, size: 10.6 });
  footer(s, "PASS marker: xsim_c07_v001_c04_face_w*_260515194251.log:28");
}

{
  const s = pptx.addSlide();
  title(s, "header_inserter Pending Timing", "마지막 TLAST가 보류된 상태에서 다음 face_start가 pending으로 latch됐습니다.");
  box(s, "face_start #1", 0.65, 2.15, 1.45, 0.5, { fill: C.blueFill, line: C.blue, bold: true, size: 8.5 });
  arrow(s, 2.15, 2.40, 2.65, 2.40);
  box(s, "ST_PREFIX\nstall", 2.70, 2.02, 1.45, 0.72, { fill: C.orangeFill, line: C.orange, bold: true, size: 8.5 });
  arrow(s, 4.20, 2.40, 4.70, 2.40);
  box(s, "data TLAST", 4.75, 2.15, 1.35, 0.5, { fill: C.greenFill, line: C.green, bold: true, size: 8.5 });
  arrow(s, 6.15, 2.40, 6.65, 2.40);
  box(s, "ST_DRAIN_LAST\nholds TLAST", 6.70, 1.98, 1.65, 0.8, { fill: C.redFill, line: C.red, bold: true, size: 8.4 });
  arrow(s, 8.40, 2.40, 8.90, 2.40);
  box(s, "face_start #2\npending latch", 8.95, 1.98, 1.65, 0.8, { fill: C.purpleFill, line: C.purple, bold: true, size: 8.4 });
  arrow(s, 10.65, 2.40, 11.15, 2.40);
  box(s, "frame #2\nstarts", 11.20, 2.15, 1.35, 0.5, { fill: C.tealFill, line: C.teal, bold: true, size: 8.5 });
  table(s, [
    ["Width", "header/frame", "frames", "total beats", "결과"],
    ["32-bit", "12 + data 1", "2", "26", "PASS"],
    ["64-bit", "6 + data 1", "2", "14", "PASS"],
    ["128-bit", "3 + data 1", "2", "8", "PASS"],
  ], 0.85, 4.25, [1.55, 2.45, 1.4, 1.75, 1.6], 0.52, 7.4);
  footer(s, "Marker: header_inserter: face_start pending-latched (not IDLE, state=st_drain_last)");
}

{
  const s = pptx.addSlide();
  title(s, "Timing / II 의미", "C04는 bounded stall을 기다리며 보존하고, pending 1-depth로 다음 frame 시작을 연결합니다.");
  table(s, [
    ["Metric", "결론"],
    ["Latency", "stall 길이만큼 first output/TLAST가 늦어지지만 row/frame boundary는 유지"],
    ["Throughput", "ready=1일 때 1 beat/clk, width 증가 시 drain beat 감소"],
    ["Pipeline", "cell input -> FIFO -> face output -> header prefix/data/drain -> final AXIS"],
    ["II", "ST_DRAIN_LAST에서 다음 face_start 1-depth pending, TLAST 수락 직후 다음 frame 시작"],
    ["제약", "다중 pending과 외부 장기 stall worst case는 system release gate"],
  ], 0.72, 1.35, [2.0, 9.6], 0.56, 7.2);
  box(s, "다음 진행: CHAIN-P1-03 Hit[16] SW/range 계약을 문서화해 final VDMA 16-bit 정책과 거리/wrap 의미를 닫습니다.", 1.0, 5.8, 11.3, 0.62, { fill: C.blueFill, line: C.blue, bold: true, size: 11.2 });
  footer(s, "문서: C07_System_Integration_260515194251_C04_Direct_Matrix_Result_v001.md");
}

pptx.writeFile({ fileName: out });
