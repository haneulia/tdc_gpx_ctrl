const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C02_Chip_Acquisition/C02_Chip_Acquisition_260501014728_Width_Timing_Verification_v001.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C02 width timing verification";
pptx.title = "C02 Width Timing Verification v001";
pptx.lang = "ko-KR";
pptx.theme = { headFontFace: "Malgun Gothic", bodyFontFace: "Malgun Gothic", lang: "ko-KR" };

const C = {
  bg: "FAFBFD",
  ink: "172033",
  muted: "64748B",
  gray: "CBD5E1",
  header: "E2E8F0",
  card: "FFFFFF",
  blue: "2563EB",
  blueFill: "EFF6FF",
  green: "16A34A",
  greenFill: "ECFDF5",
  orange: "EA580C",
  orangeFill: "FFF7ED",
  red: "DC2626",
  redFill: "FEF2F2",
  violet: "7C3AED",
  violetFill: "F5F3FF",
  cyan: "0891B2",
  cyanFill: "ECFEFF"
};
const font = "Malgun Gothic";

function bg(slide) {
  slide.background = { color: C.bg };
}

function txt(slide, text, x, y, w, h, size = 14, bold = false, color = C.ink, align = "left") {
  slide.addText(text, {
    x, y, w, h,
    fontFace: font,
    fontSize: size,
    bold,
    color,
    align,
    valign: "mid",
    fit: "shrink",
    margin: 0.04,
    breakLine: false
  });
}

function box(slide, text, x, y, w, h, fill = C.card, line = C.gray, size = 12, bold = false, color = C.ink) {
  slide.addShape(pptx.ShapeType.roundRect, {
    x, y, w, h,
    rectRadius: 0.04,
    fill: { color: fill },
    line: { color: line, width: 1 }
  });
  txt(slide, text, x + 0.06, y + 0.05, w - 0.12, h - 0.1, size, bold, color, "center");
}

function arrow(slide, x1, y1, x2, y2, color = C.muted) {
  slide.addShape(pptx.ShapeType.line, {
    x: x1, y: y1, w: x2 - x1, h: y2 - y1,
    line: { color, width: 1.4, beginArrowType: "none", endArrowType: "triangle" }
  });
}

function title(slide, main, sub) {
  bg(slide);
  txt(slide, main, 0.55, 0.28, 12.2, 0.45, 22, true, C.ink);
  txt(slide, sub, 0.58, 0.75, 12.0, 0.28, 9.5, false, C.muted);
  slide.addShape(pptx.ShapeType.line, { x: 0.55, y: 1.10, w: 12.2, h: 0, line: { color: C.gray, width: 0.8 } });
}

function table(slide, rows, x, y, widths, rowH = 0.45, size = 10.5) {
  rows.forEach((row, r) => {
    let cx = x;
    row.forEach((cell, c) => {
      const fill = r === 0 ? C.header : C.card;
      const bold = r === 0;
      box(slide, String(cell), cx, y + r * rowH, widths[c], rowH - 0.035, fill, C.gray, r === 0 ? size : size - 0.6, bold);
      cx += widths[c] + 0.04;
    });
  });
}

function foot(slide, text) {
  txt(slide, text, 0.65, 6.95, 12.0, 0.24, 8.2, false, C.muted, "center");
}

let s = pptx.addSlide();
bg(s);
txt(s, "C02 Width / Timing Verification", 0.75, 0.65, 12.0, 0.55, 29, true, C.blue);
txt(s, "Data Flow와 Timing Pipeline의 미검증 항목 보완 결과", 0.75, 1.28, 12.0, 0.48, 20, true, C.ink);
box(s, "32-bit top\nPASS\n72 beats", 0.9, 2.25, 2.55, 1.12, C.greenFill, C.green, 15, true, C.green);
box(s, "64-bit top\nPASS\n44 beats", 3.75, 2.25, 2.55, 1.12, C.greenFill, C.green, 15, true, C.green);
box(s, "128-bit top\nPASS\n38 beats", 6.6, 2.25, 2.55, 1.12, C.greenFill, C.green, 15, true, C.green);
box(s, "max_hits matrix\nPASS\n1~7 전체", 9.45, 2.25, 2.75, 1.12, C.cyanFill, C.cyan, 15, true, C.cyan);
txt(s, "핵심: 넓은 bus는 output serialize/final AXIS 구간의 beat 수를 줄인다. 단, runtime max_hits_cfg가 실제로 설정되어야 한다.", 0.95, 4.10, 11.4, 0.55, 14.5, true, C.orange, "center");
txt(s, "작성: 2026-05-01 01:47:28 KST / 수정: 2026-05-01 01:51:19 KST", 0.95, 6.25, 9.5, 0.28, 11.5, false, C.muted);
foot(s, "근거: xsim_top_int_width32/64/128.log, xsim_width_timing_matrix.log");

s = pptx.addSlide();
title(s, "Runtime Max Hits 계약", "기존 공백: 거리 기준 max_hits 계산값이 CSR CTL21에 쓰이지 않으면 기본값 7로 동작한다.");
box(s, "Range 기반 계산\nC_MAX_HITS=3", 0.8, 1.65, 2.35, 0.86, C.blueFill, C.blue, 13.5, true, C.blue);
arrow(s, 3.15, 2.08, 4.0, 2.08);
box(s, "Chip CSR write\nCTL21 = 0x0003_0000", 4.0, 1.65, 2.95, 0.86, C.orangeFill, C.orange, 13.5, true, C.orange);
arrow(s, 6.95, 2.08, 7.8, 2.08);
box(s, "cell_builder / assembler\nBcell(3,W)", 7.8, 1.65, 2.7, 0.86, C.greenFill, C.green, 13.5, true, C.green);
arrow(s, 10.5, 2.08, 11.15, 2.08);
box(s, "Final AXIS\nbeat 감소", 11.15, 1.65, 1.6, 0.86, C.cyanFill, C.cyan, 12.5, true, C.cyan);
table(s, [
  ["구분", "보완 전 TB", "보완 후 TB"],
  ["max_hits_cfg", "CSR 미설정 -> 기본 7", "CTL21[18:16]=3"],
  ["32-bit 결과", "104 beats", "72 beats"],
  ["64-bit 결과", "60 beats", "44 beats"],
  ["128-bit 결과", "38 beats", "38 beats"],
], 0.9, 3.15, [2.4, 4.5, 4.5], 0.52, 11.0);
box(s, "운영 계약\n상위 제어/SW는 face 시작 전 CTL21.max_hits_cfg를 설정해야 throughput 이득을 얻는다.\n000은 안전 기본값 7 alias이다.", 1.0, 6.0, 11.3, 0.62, C.redFill, C.red, 12.5, true, C.red);
foot(s, "근거: tb_tdc_gpx_top_int.vhd CTL21 write, xsim_top_int_width*.log line 55");

s = pptx.addSlide();
title(s, "Top 통합 시뮬레이션", "공통 조건: active chips=4, stops/chip=2, cols=2, max_hits_cfg=3");
table(s, [
  ["폭", "기대 beat", "Rising", "Falling", "TLAST", "결과"],
  ["32", "72", "72", "72", "2 / 2", "PASS"],
  ["64", "44", "44", "44", "2 / 2", "PASS"],
  ["128", "38", "38", "38", "2 / 2", "PASS"],
], 0.8, 1.45, [1.1, 1.7, 1.7, 1.7, 1.5, 1.6], 0.56, 11.5);
box(s, "ExpectedBeats = cols * (HeaderBeats(W) + active_chips * stops_per_chip * Bcell(max_hits,W))", 0.95, 4.65, 11.4, 0.58, C.blueFill, C.blue, 13.5, true, C.blue);
box(s, "검증 강화\nTB가 nonzero만 보던 상태에서 expected beat와 rising/falling tlast까지 assert하도록 변경했다.", 1.0, 5.65, 11.3, 0.62, C.greenFill, C.green, 13.2, true, C.green);
foot(s, "근거: xsim_top_int_width32.log:88-92, width64.log:88-92, width128.log:88-92");

s = pptx.addSlide();
title(s, "Matrix / Pipeline Timing", "C=4, N=8 기준 line beat 산식과 output 구간 시간");
table(s, [
  ["max_hits", "32-bit", "64-bit", "128-bit"],
  ["1", "76", "70", "67"],
  ["3", "108", "70", "67"],
  ["5", "140", "102", "67"],
  ["7", "172", "102", "67"],
], 0.75, 1.35, [2.0, 2.2, 2.2, 2.2], 0.5, 11.2);
table(s, [
  ["Top 조건", "32-bit", "64-bit", "128-bit"],
  ["beats/slope", "72", "44", "38"],
  ["150MHz 시간", "0.480us", "0.293us", "0.253us"],
  ["TB 200MHz 시간", "0.360us", "0.220us", "0.190us"],
], 0.75, 4.1, [2.0, 2.2, 2.2, 2.2], 0.5, 11.0);
box(s, "II 해석: final AXIS는 ready 유지 시 beat 단위 II=1이다. 폭 증가는 II를 1보다 작게 만들지 않고, 필요한 beat 수를 줄인다.", 8.3, 2.0, 4.25, 1.45, C.orangeFill, C.orange, 12.8, true, C.orange);
box(s, "GPX READ와 raw/event 내부 payload는 output 폭 변경으로 빨라지지 않는다.", 8.3, 4.8, 4.25, 0.82, C.violetFill, C.violet, 12.8, true, C.violet);
foot(s, "근거: tb_tdc_gpx_width_timing_matrix.vhd, xsim_width_timing_matrix.log:30-110");

s = pptx.addSlide();
title(s, "17-bit Raw Hit 계약", "이번 보완은 full 17-bit 저장 PASS가 아니라 16-bit cell 계약 명확화이다.");
box(s, "raw_event\n17-bit raw hit", 0.9, 1.7, 2.4, 0.9, C.blueFill, C.blue, 14, true, C.blue);
arrow(s, 3.3, 2.15, 4.25, 2.15);
box(s, "cell_builder\nlower 16-bit 저장", 4.25, 1.7, 2.65, 0.9, C.orangeFill, C.orange, 14, true, C.orange);
arrow(s, 6.9, 2.15, 7.85, 2.15);
box(s, "cell slot\n16-bit contract", 7.85, 1.7, 2.45, 0.9, C.greenFill, C.green, 14, true, C.green);
box(s, "후속 필요 시\nformat/header/parser 동시 변경", 10.55, 1.7, 1.95, 0.9, C.redFill, C.red, 11.8, true, C.red);
table(s, [
  ["항목", "결론"],
  ["DF-C02-W-04", "현재 C02는 16-bit cell 저장 계약으로 닫음"],
  ["full 17-bit 보존", "C02 이후 별도 format 변경 필요"],
  ["근거", "tdc_gpx_pkg.vhd, tdc_gpx_cell_builder.vhd, matrix TB"],
], 1.0, 3.45, [2.8, 8.2], 0.55, 11.4);
box(s, "C02 다음 판단\n32/64/128 폭과 runtime max_hits 기반 output timing은 검증 완료. 17-bit full preservation은 후속 범위로 분리한다.", 1.0, 6.1, 11.3, 0.62, C.greenFill, C.green, 12.8, true, C.green);
foot(s, "근거: tdc_gpx_pkg.vhd:48-50, tdc_gpx_cell_builder.vhd:644, xsim_width_timing_matrix.log:30");

pptx.writeFile({ fileName: out });
