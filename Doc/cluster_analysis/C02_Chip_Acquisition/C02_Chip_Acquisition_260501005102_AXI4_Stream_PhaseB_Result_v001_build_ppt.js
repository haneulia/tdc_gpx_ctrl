const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/pptxgenjs");

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C02 AXI4-Stream Phase B Result";
pptx.title = "C02 AXI4-Stream Phase B";
pptx.lang = "ko-KR";
pptx.theme = {
  headFontFace: "Malgun Gothic",
  bodyFontFace: "Malgun Gothic",
  lang: "ko-KR",
};

const C = {
  bg: "FAFBFD",
  ink: "1F2937",
  muted: "64748B",
  blue: "2563EB",
  blueFill: "EFF6FF",
  green: "16A34A",
  greenFill: "ECFDF5",
  orange: "EA580C",
  orangeFill: "FFF7ED",
  red: "DC2626",
  redFill: "FEF2F2",
  gray: "CBD5E1",
  card: "FFFFFF",
  header: "F1F5F9",
};
const font = "Malgun Gothic";

function bg(slide) {
  slide.background = { color: C.bg };
}

function txt(slide, text, x, y, w, h, size = 18, bold = false, color = C.ink, align = "left") {
  slide.addText(text, {
    x, y, w, h,
    fontFace: font,
    fontSize: size,
    bold,
    color,
    align,
    valign: "mid",
    fit: "shrink",
    margin: 0.02,
    breakLine: false,
  });
}

function box(slide, text, x, y, w, h, fill = C.card, line = C.gray, size = 15, bold = false, color = C.ink) {
  slide.addShape(pptx.ShapeType.roundRect, {
    x, y, w, h,
    rectRadius: 0.05,
    fill: { color: fill },
    line: { color: line, width: 1 },
  });
  slide.addText(text, {
    x: x + 0.06,
    y: y + 0.04,
    w: w - 0.12,
    h: h - 0.08,
    fontFace: font,
    fontSize: size,
    bold,
    color,
    align: "center",
    valign: "mid",
    fit: "shrink",
    margin: 0.02,
  });
}

function arrow(slide, x1, y1, x2, y2, color = C.blue) {
  slide.addShape(pptx.ShapeType.line, {
    x: x1,
    y: y1,
    w: x2 - x1,
    h: y2 - y1,
    line: { color, width: 2, beginArrowType: "none", endArrowType: "triangle" },
  });
}

function bullets(slide, items, x, y, w, h, size = 17) {
  slide.addText(items.map((text) => ({ text, options: { bullet: { type: "bullet" } } })), {
    x, y, w, h,
    fontFace: font,
    fontSize: size,
    color: C.ink,
    fit: "shrink",
    valign: "top",
    margin: 0.03,
    paraSpaceAfterPt: 7,
  });
}

function table(slide, rows, x, y, widths, rowH = 0.5) {
  rows.forEach((row, r) => {
    let cx = x;
    row.forEach((cell, c) => {
      const isHeader = r === 0;
      box(
        slide,
        cell,
        cx,
        y + r * rowH,
        widths[c],
        rowH - 0.04,
        isHeader ? C.header : C.card,
        C.gray,
        isHeader ? 13 : 12,
        isHeader,
        C.ink,
      );
      cx += widths[c] + 0.04;
    });
  });
}

let slide = pptx.addSlide();
bg(slide);
txt(slide, "C02 AXI4-Stream Phase B", 0.7, 0.62, 11.8, 0.5, 28, true, C.blue);
txt(slide, "raw/event/stop/fire 폭 계약 상수화 결과", 0.7, 1.15, 11.8, 0.75, 34, true, C.ink);
box(slide, "기능 변경 없음\n폭 계약 명시화", 0.9, 2.35, 3.2, 1.1, C.blueFill, C.blue, 21, true, C.blue);
box(slide, "64-bit 통합 시뮬레이션\nPASS 유지", 5.05, 2.35, 3.2, 1.1, C.greenFill, C.green, 21, true, C.green);
box(slide, "Latency / II\n변화 없음", 9.2, 2.35, 2.7, 1.1, C.orangeFill, C.orange, 21, true, C.orange);
txt(slide, "문서 시각: 2026-05-01 00:51:02 KST", 0.9, 6.35, 7.5, 0.32, 14, false, C.muted);
txt(slide, "기준: Doc/TDC-GPX-Datasheet.pdf + C01/C02 구현 계약", 0.9, 6.73, 10.5, 0.32, 14, false, C.muted);

slide = pptx.addSlide();
bg(slide);
txt(slide, "폭 계약 테이블", 0.6, 0.45, 12.0, 0.45, 27, true, C.ink);
table(slide, [
  ["경계", "tdata", "tuser", "tkeep", "pack", "근거"],
  ["bus response", "32", "8", "4", "40", "pkg + bus_phy/chip_ctrl"],
  ["raw IFIFO", "32", "8", "-", "40", "pkg + config/decode"],
  ["event", "32", "16", "-", "48", "pkg + decode/cell"],
  ["stop event", "32", "32", "4", "-", "top/config/stop decode"],
  ["final output", "32/64/128", "1", "full", "-", "Phase A 유지"],
], 0.55, 1.25, [2.05, 1.35, 1.35, 1.35, 1.35, 4.4], 0.62);
txt(slide, "핵심: 출력 폭 확장 계약과 GPX raw/event 내부 계약을 분리했다.", 0.75, 5.55, 11.5, 0.45, 20, true, C.blue);
txt(slide, "따라서 64-bit 출력 성공과 내부 40/48-bit pack 경계가 서로 독립적으로 추적된다.", 0.75, 6.05, 11.5, 0.4, 16, false, C.muted);

slide = pptx.addSlide();
bg(slide);
txt(slide, "데이터 흐름", 0.6, 0.45, 12.0, 0.45, 27, true, C.ink);
const nodes = [
  ["GPX IC\n28-bit raw", 0.55, 2.25, 1.55],
  ["bus_phy\n32+8", 2.35, 2.25, 1.45],
  ["chip_ctrl\nraw 32+8", 4.05, 2.25, 1.65],
  ["raw CDC/skid\npack 40", 5.95, 2.25, 1.7],
  ["decode/event\npack 48", 7.95, 2.25, 1.7],
  ["cell/output\n32/64/128", 9.95, 2.25, 1.85],
];
nodes.forEach(([label, x, y, w], i) => {
  const fill = i === 0 ? C.orangeFill : i === nodes.length - 1 ? C.greenFill : C.card;
  const line = i === 0 ? C.orange : i === nodes.length - 1 ? C.green : C.gray;
  box(slide, label, x, y, w, 1.05, fill, line, 14, true);
});
for (let i = 0; i < nodes.length - 1; i += 1) {
  arrow(slide, nodes[i][1] + nodes[i][3], 2.78, nodes[i + 1][1], 2.78);
}
box(slide, "stop_evt_tuser는 tdata와 generic 분리\n현재 기본값은 32-bit로 기존 동작 유지", 2.1, 4.25, 4.0, 0.95, C.blueFill, C.blue, 16, true, C.blue);
box(slide, "Datasheet 기반 GPX raw word 해석은 그대로 유지\n코드 추적성과 확장성만 개선", 7.0, 4.25, 4.2, 0.95, C.greenFill, C.green, 16, true, C.green);

slide = pptx.addSlide();
bg(slide);
txt(slide, "Timing / Latency / Throughput / II", 0.6, 0.45, 12.0, 0.45, 27, true, C.ink);
box(slide, "C01 READ", 0.8, 1.55, 1.5, 0.62, C.card, C.gray, 15, true);
arrow(slide, 2.3, 1.86, 3.0, 1.86);
box(slide, "raw CDC/skid", 3.0, 1.55, 1.8, 0.62, C.card, C.gray, 15, true);
arrow(slide, 4.8, 1.86, 5.55, 1.86);
box(slide, "decode", 5.55, 1.55, 1.45, 0.62, C.card, C.gray, 15, true);
arrow(slide, 7.0, 1.86, 7.75, 1.86);
box(slide, "event", 7.75, 1.55, 1.45, 0.62, C.card, C.gray, 15, true);
arrow(slide, 9.2, 1.86, 9.95, 1.86);
box(slide, "cell/output", 9.95, 1.55, 1.8, 0.62, C.card, C.gray, 15, true);
bullets(slide, [
  "Latency: 신규 FF/FIFO/state wait 없음, 변화 없음",
  "Throughput: 기존 raw 40-bit/event 48-bit pack 유지, 변화 없음",
  "Pipeline: 기존 C01 -> C02 -> C03 구조 유지",
  "II: handshake 조건 미변경, beat 기준 II=1 조건 유지",
  "효과: 타이밍 분석 시 경계 폭을 상수명으로 추적 가능",
], 0.95, 3.0, 11.4, 1.9, 18);

slide = pptx.addSlide();
bg(slide);
txt(slide, "검증 결과", 0.6, 0.45, 12.0, 0.45, 27, true, C.ink);
table(slide, [
  ["Testbench", "결과", "핵심 확인"],
  ["tb_tdc_gpx_decode_pipe_phaseb", "PASS", "ALL TESTS PASSED (5 checks)"],
  ["tb_tdc_gpx_stop_cfg_phaseb", "PASS", "stop_cfg_decode ALL TESTS PASSED"],
  ["tb_tdc_gpx_config_ctrl_phaseb", "PASS", "expected-count CDC/top integration PASS"],
  ["tb_tdc_gpx_cell_pipe_phaseb", "PASS", "cell output valid/tlast/tuser PASS"],
  ["tb_tdc_gpx_top_int_phaseb64", "PASS", "64-bit rising/falling beats=60, tlast=2"],
], 0.55, 1.25, [4.2, 1.3, 6.55], 0.62);
box(slide, "판단: Phase C 진입 가능\n단, 256-bit 이상 또는 partial TKEEP는 별도 설계 판단 필요", 1.1, 6.0, 11.0, 0.75, C.greenFill, C.green, 19, true, C.green);

pptx.writeFile({
  fileName: "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C02_Chip_Acquisition/C02_Chip_Acquisition_260501005102_AXI4_Stream_PhaseB_Result_v001.pptx",
});
