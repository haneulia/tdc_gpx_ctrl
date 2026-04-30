const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C02_Chip_Acquisition/C02_Chip_Acquisition_260501011313_Data_Flow_Review_v002.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C02 data flow review for 32/64/128-bit AXI4-Stream";
pptx.title = "C02 Data Flow Review v002";
pptx.lang = "ko-KR";
pptx.theme = {
  headFontFace: "Malgun Gothic",
  bodyFontFace: "Malgun Gothic",
  lang: "ko-KR",
};

const C = {
  bg: "FAFBFD",
  ink: "172033",
  muted: "64748B",
  blue: "2563EB",
  blueFill: "EFF6FF",
  cyan: "0891B2",
  cyanFill: "ECFEFF",
  green: "16A34A",
  greenFill: "ECFDF5",
  orange: "EA580C",
  orangeFill: "FFF7ED",
  red: "DC2626",
  redFill: "FEF2F2",
  violet: "7C3AED",
  violetFill: "F5F3FF",
  gray: "CBD5E1",
  header: "E2E8F0",
  card: "FFFFFF",
};
const font = "Malgun Gothic";

function bg(slide) {
  slide.background = { color: C.bg };
}

function txt(slide, text, x, y, w, h, size = 16, bold = false, color = C.ink, align = "left") {
  slide.addText(text, {
    x, y, w, h,
    fontFace: font,
    fontSize: size,
    bold,
    color,
    align,
    valign: "mid",
    fit: "shrink",
    margin: 0.03,
    breakLine: false,
  });
}

function box(slide, text, x, y, w, h, fill = C.card, line = C.gray, size = 14, bold = false, color = C.ink) {
  slide.addShape(pptx.ShapeType.roundRect, {
    x, y, w, h,
    rectRadius: 0.04,
    fill: { color: fill },
    line: { color: line, width: 1 },
  });
  txt(slide, text, x + 0.07, y + 0.05, w - 0.14, h - 0.1, size, bold, color, "center");
}

function arrow(slide, x1, y1, x2, y2, color = C.muted, width = 1.5) {
  slide.addShape(pptx.ShapeType.line, {
    x: x1,
    y: y1,
    w: x2 - x1,
    h: y2 - y1,
    line: { color, width, beginArrowType: "none", endArrowType: "triangle" },
  });
}

function title(slide, main, sub) {
  bg(slide);
  txt(slide, main, 0.55, 0.32, 12.3, 0.46, 22, true, C.ink);
  txt(slide, sub, 0.58, 0.82, 12.2, 0.28, 9.5, false, C.muted);
  slide.addShape(pptx.ShapeType.line, {
    x: 0.55, y: 1.18, w: 12.2, h: 0,
    line: { color: C.gray, width: 0.8 },
  });
}

function table(slide, rows, x, y, widths, rowH = 0.45, size = 10.5) {
  rows.forEach((row, r) => {
    let cx = x;
    row.forEach((cell, c) => {
      box(
        slide,
        cell,
        cx,
        y + r * rowH,
        widths[c],
        rowH - 0.035,
        r === 0 ? C.header : C.card,
        C.gray,
        r === 0 ? size : size - 0.8,
        r === 0,
        C.ink,
      );
      cx += widths[c] + 0.04;
    });
  });
}

function foot(slide, text) {
  txt(slide, text, 0.65, 6.95, 12.0, 0.24, 8.2, false, C.muted, "center");
}

let s = pptx.addSlide();
bg(s);
txt(s, "C02 Data Flow Review v002", 0.75, 0.72, 12.0, 0.55, 30, true, C.blue);
txt(s, "32 / 64 / 128-bit AXI4-Stream 폭 기준 재검토", 0.75, 1.35, 12.0, 0.55, 24, true, C.ink);
box(s, "Raw/Event\n고정 32-bit", 0.95, 2.35, 2.6, 1.05, C.blueFill, C.blue, 17, true, C.blue);
box(s, "Cell/Header\n폭 증가 = 더 빠른 출력", 4.0, 2.35, 2.6, 1.05, C.greenFill, C.green, 16, true, C.green);
box(s, "TKEEP/TSTRB\nFull-one", 7.05, 2.35, 2.6, 1.05, C.violetFill, C.violet, 17, true, C.violet);
box(s, "256-bit\n미반영", 10.1, 2.35, 2.15, 1.05, C.redFill, C.red, 17, true, C.red);
txt(s, "핵심: 넓은 bus는 output serialize 구간을 빠르게 한다. 단, GPX READ/raw/event 수집 속도는 그대로다.", 0.9, 4.25, 11.5, 0.48, 15, true, C.orange, "center");
txt(s, "작성 시간: 2026-05-01 01:13:13 KST", 0.9, 6.28, 7.0, 0.3, 12, false, C.muted);
txt(s, "기준: Doc/TDC-GPX-Datasheet.pdf + 현재 RTL", 0.9, 6.62, 9.0, 0.3, 12, false, C.muted);

s = pptx.addSlide();
title(s, "데이터 플로우 기준점", "폭 변경은 cell_builder 이후 output 구간을 빠르게 한다. GPX raw/event 의미와 수집 속도는 고정이다.");
const nodes = [
  ["GPX IFIFO\n28b raw", 0.55, C.blueFill, C.blue],
  ["chip_ctrl\n32b+8b", 2.1, C.cyanFill, C.cyan],
  ["raw CDC\npack 40", 3.75, C.card, C.gray],
  ["decode/event\n32b+16b", 5.3, C.violetFill, C.violet],
  ["cell_builder\n32/64/128", 7.05, C.greenFill, C.green],
  ["face/header\nfull keep", 8.95, C.orangeFill, C.orange],
  ["final AXIS\nrise/fall", 10.85, C.redFill, C.red],
];
nodes.forEach(([label, x, fill, line], i) => {
  box(s, label, x, 2.0, 1.25, 0.78, fill, line, 10.5, true, line);
  if (i < nodes.length - 1) arrow(s, x + 1.25, 2.39, nodes[i + 1][1], 2.39);
});
box(s, "고정 폭 구간\nGPX 의미 보존", 1.0, 3.8, 5.2, 0.75, C.blueFill, C.blue, 15, true, C.blue);
box(s, "가변 폭 구간\nbeat 수 감소 + byte throughput 증가", 7.0, 3.8, 4.7, 0.75, C.greenFill, C.green, 14, true, C.green);
table(s, [
  ["구간", "폭 계약", "폭 변경 영향"],
  ["raw stream", "tdata=32, tuser=8", "없음"],
  ["event stream", "tdata=32, tuser=16", "없음"],
  ["final output", "tdata=32/64/128", "header/cell beat 수 변경"],
], 1.0, 5.0, [2.4, 3.5, 5.2], 0.43, 10.2);
foot(s, "근거: tdc_gpx_pkg.vhd:81-101, tdc_gpx_top.vhd:36/147-158");

s = pptx.addSlide();
title(s, "32 / 64 / 128 폭별 비교", "지원 폭은 모두 full-keep이고 48-byte header가 정수 beat로 정렬된다.");
table(s, [
  ["Width", "Byte/beat", "TKEEP", "Header beats", "Static cell beats", "max_hits=7 emit"],
  ["32-bit", "4", "4b", "12", "8", "5"],
  ["64-bit", "8", "8b", "6", "4", "3"],
  ["128-bit", "16", "16b", "3", "2", "2"],
  ["256-bit", "32", "32b", "1.5", "1", "미지원"],
], 0.8, 1.45, [1.45, 1.55, 1.35, 2.0, 2.15, 2.25], 0.53, 11);
box(s, "Static cell envelope = 32 byte\nfn_beats_per_cell()", 1.0, 4.75, 3.5, 0.75, C.blueFill, C.blue, 13.5, true, C.blue);
box(s, "Runtime emit beats = max_hits_cfg 기반\nfn_beats_per_cell_rt()", 4.95, 4.75, 3.6, 0.75, C.greenFill, C.green, 13.5, true, C.green);
box(s, "문서/검증에서 두 수치를 분리해야 함", 9.0, 4.75, 3.35, 0.75, C.orangeFill, C.orange, 13.5, true, C.orange);
foot(s, "근거: tdc_gpx_pkg.vhd:183-184/744-795/803-821, tdc_gpx_cell_builder.vhd:935-944");

s = pptx.addSlide();
title(s, "TUSER와 제어 의미", "final tuser(0)는 SOF이다. fault/degraded는 별도 metadata/status 경로를 함께 봐야 한다.");
box(s, "raw/event tuser\nslope, stop, chip, shot, control", 0.8, 1.65, 2.65, 0.9, C.blueFill, C.blue, 12.5, true, C.blue);
arrow(s, 3.45, 2.1, 4.1, 2.1);
box(s, "cell tuser(0)\nchip slice faulted", 4.1, 1.65, 2.35, 0.9, C.orangeFill, C.orange, 12.5, true, C.orange);
arrow(s, 6.45, 2.1, 7.1, 2.1);
box(s, "row/frame status\nfault pulse + CSR", 7.1, 1.65, 2.45, 0.9, C.violetFill, C.violet, 12.5, true, C.violet);
arrow(s, 9.55, 2.1, 10.2, 2.1);
box(s, "final tuser(0)\nSOF only", 10.2, 1.65, 2.2, 0.9, C.greenFill, C.green, 12.5, true, C.green);
table(s, [
  ["downstream 판단", "계약"],
  ["slope", "rising/falling stream 자체로 구분"],
  ["frame start", "final tuser(0)=1"],
  ["line end", "tlast=1"],
  ["fault/degraded", "cell metadata + row/frame pulse + CSR/status"],
], 1.0, 3.65, [3.0, 8.7], 0.48, 10.5);
foot(s, "근거: tdc_gpx_cell_builder.vhd:948-961, tdc_gpx_header_inserter.vhd:493-498");

s = pptx.addSlide();
title(s, "Throughput / Latency / II", "폭이 커지면 output serialize 구간은 빨라진다. GPX READ 병목은 변하지 않는다.");
table(s, [
  ["Width", "Byte/beat", "Beat II", "150MHz 이론 throughput"],
  ["32-bit", "4", "1 clk", "600 MB/s"],
  ["64-bit", "8", "1 clk", "1.2 GB/s"],
  ["128-bit", "16", "1 clk", "2.4 GB/s"],
], 0.9, 1.4, [1.8, 2.0, 2.0, 3.2], 0.55, 11.5);
box(s, "64-bit\n32-bit 대비 2x byte/beat", 0.95, 4.45, 3.1, 0.75, C.blueFill, C.blue, 13.5, true, C.blue);
box(s, "128-bit\n32-bit 대비 4x byte/beat", 4.35, 4.45, 3.1, 0.75, C.greenFill, C.green, 13.5, true, C.green);
box(s, "II\nAXIS beat II=1 유지", 7.75, 4.45, 3.1, 0.75, C.violetFill, C.violet, 13.5, true, C.violet);
txt(s, "주의: 빨라지는 지점은 output 구간이다. 전체 시스템은 GPX READ timing, expected-count drain, downstream ready가 함께 제한한다.", 1.0, 5.85, 11.3, 0.45, 14, true, C.orange, "center");
foot(s, "근거: tdc_gpx_top.vhd:20/42/51-57, tdc_gpx_cell_builder.vhd:948-1011");

s = pptx.addSlide();
title(s, "검증 상태와 남은 경계", "64-bit top 통합은 PASS 근거가 있고, 32/128 top 통합은 별도 close 항목으로 남긴다.");
table(s, [
  ["항목", "현재 근거", "판단"],
  ["Header widths", "TB 32/64/128 instance", "확인"],
  ["Full keep/strb", "header TB + RTL", "확인"],
  ["64-bit top", "xsim rising/falling beats=60", "PASS"],
  ["32-bit top", "최신 full top 로그 별도 필요", "남음"],
  ["128-bit top", "최신 full top 로그 별도 필요", "남음"],
  ["max_hits matrix", "runtime emit beat 별도 계측 필요", "남음"],
], 0.65, 1.35, [2.3, 5.0, 4.25], 0.45, 10);
box(s, "v002 판단\n32/64/128 데이터 의미는 보존된다.\n남은 일은 full top width matrix와 runtime beat 계측이다.", 1.0, 5.65, 11.25, 0.8, C.greenFill, C.green, 16, true, C.green);
foot(s, "근거: tb_tdc_gpx_header_inserter_widths.vhd:92-169/282-315/344-347, xsim.log");

pptx.writeFile({ fileName: out });
