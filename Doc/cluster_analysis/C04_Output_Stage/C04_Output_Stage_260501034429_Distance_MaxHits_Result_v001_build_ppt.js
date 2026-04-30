const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C04_Output_Stage/C04_Output_Stage_260501034429_Distance_MaxHits_Result_v001.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C04 Distance MaxHits Result v001";
pptx.title = "C04 Distance MaxHits Result v001";
pptx.lang = "ko-KR";
pptx.theme = { headFontFace: "Malgun Gothic", bodyFontFace: "Malgun Gothic", lang: "ko-KR" };

const font = "Malgun Gothic";
const C = {
  bg: "FAFBFD", ink: "172033", muted: "64748B", line: "CBD5E1", card: "FFFFFF",
  blue: "2563EB", blueFill: "EFF6FF", green: "16A34A", greenFill: "ECFDF5",
  orange: "EA580C", orangeFill: "FFF7ED", red: "DC2626", redFill: "FEF2F2",
  cyan: "0891B2", cyanFill: "ECFEFF", slate: "F8FAFC"
};

function bg(slide) { slide.background = { color: C.bg }; }
function text(slide, value, x, y, w, h, opt = {}) {
  slide.addText(value, {
    x, y, w, h, fontFace: font, fontSize: opt.size || 11, bold: opt.bold || false,
    color: opt.color || C.ink, align: opt.align || "left", valign: "mid",
    fit: "shrink", margin: opt.margin === undefined ? 0.04 : opt.margin
  });
}
function title(slide, main, sub) {
  bg(slide);
  text(slide, main, 0.55, 0.28, 12.2, 0.45, { size: 21, bold: true });
  text(slide, sub, 0.58, 0.74, 12.0, 0.3, { size: 9.2, color: C.muted });
  slide.addShape(pptx.ShapeType.line, { x: 0.55, y: 1.08, w: 12.2, h: 0, line: { color: C.line, width: 0.8 } });
}
function footer(slide, value) { text(slide, value, 0.65, 6.96, 12, 0.25, { size: 8, color: C.muted, align: "center" }); }
function box(slide, value, x, y, w, h, opt = {}) {
  slide.addShape(pptx.ShapeType.rect, { x, y, w, h, fill: { color: opt.fill || C.card }, line: { color: opt.line || C.line, width: 1 } });
  text(slide, value, x + 0.08, y + 0.06, w - 0.16, h - 0.12, { size: opt.size || 11, bold: opt.bold || false, color: opt.color || C.ink, align: opt.align || "center" });
}
function arrow(slide, x1, y1, x2, y2, color = C.muted) {
  slide.addShape(pptx.ShapeType.line, { x: x1, y: y1, w: x2 - x1, h: y2 - y1, line: { color, width: 1.5, endArrowType: "triangle" } });
}
function table(slide, rows, x, y, widths, rowH, size = 9.3) {
  rows.forEach((row, r) => {
    let curX = x;
    row.forEach((cell, c) => {
      slide.addShape(pptx.ShapeType.rect, {
        x: curX, y: y + r * rowH, w: widths[c], h: rowH,
        fill: { color: r === 0 ? "E2E8F0" : C.card },
        line: { color: C.line, width: 0.7 }
      });
      text(slide, cell, curX + 0.05, y + r * rowH + 0.03, widths[c] - 0.1, rowH - 0.06, {
        size, bold: r === 0, align: c === 0 ? "center" : "left"
      });
      curX += widths[c];
    });
  });
}

{
  const s = pptx.addSlide();
  title(s, "거리 기반 max_hits_cfg 결과", "150m 시작, FAIL 시 50m 증가, 같은 cfg 재시도");
  box(s, "Distance\n150m", 0.8, 1.55, 1.45, 0.75, { fill: C.orangeFill, line: C.orange, bold: true });
  arrow(s, 2.3, 1.92, 2.95, 1.92, C.orange);
  box(s, "cfg\n1..7", 3.0, 1.55, 1.25, 0.75, { fill: C.blueFill, line: C.blue, bold: true });
  arrow(s, 4.3, 1.92, 4.95, 1.92, C.blue);
  box(s, "C04 line drain\n150MHz", 5.0, 1.45, 2.05, 0.95, { fill: C.cyanFill, line: C.cyan, bold: true });
  arrow(s, 7.1, 1.92, 7.75, 1.92, C.cyan);
  box(s, "PASS\ncfg++", 7.8, 1.55, 1.45, 0.75, { fill: C.greenFill, line: C.green, bold: true });
  arrow(s, 9.3, 1.92, 9.95, 1.92, C.green);
  box(s, "FAIL\n+50m retry", 10.0, 1.55, 2.1, 0.75, { fill: C.redFill, line: C.red, bold: true });
  table(s, [
    ["기준", "값"],
    ["거리", "150m 시작, FAIL 때만 50m 증가"],
    ["부하", "4 chips x 8 stops full line"],
    ["clock", "final output drain 150MHz"],
    ["판정", "drain_ps <= allowed_ps"]
  ], 1.0, 4.0, [2.0, 9.2], 0.48, 10);
  footer(s, "TB: tb_tdc_gpx_distance_maxhits_matrix.vhd");
}

{
  const s = pptx.addSlide();
  title(s, "Timing 공식", "거리 왕복 시간과 output drain 시간을 직접 비교");
  table(s, [
    ["항목", "공식"],
    ["allowed_ps", "distance_m * 6671"],
    ["line_beats", "header_beats + 4 * 8 * beats_per_cell"],
    ["drain_ps", "line_beats * 6667"],
    ["PASS", "drain_ps <= allowed_ps"]
  ], 1.05, 1.4, [2.2, 8.9], 0.62, 11);
  box(s, "Datasheet의 hit 원천 구조는 기준으로 유지하고,\n이번 실험은 C04 final output throughput 경계를 보는 계산형 xsim 검증입니다.", 1.05, 4.8, 11.1, 0.9, { fill: C.blueFill, line: C.blue, size: 10.5, bold: true });
  footer(s, "round_trip_ps_per_m=6671, output_period_ps=6667");
}

{
  const s = pptx.addSlide();
  title(s, "32bit 결과", "150m에서는 cfg=1..6 PASS, cfg=7은 200m에서 PASS");
  table(s, [
    ["거리", "cfg", "beats", "drain ps", "allowed ps", "결과"],
    ["150", "1", "76", "506692", "1000650", "PASS"],
    ["150", "2", "76", "506692", "1000650", "PASS"],
    ["150", "3", "108", "720036", "1000650", "PASS"],
    ["150", "4", "108", "720036", "1000650", "PASS"],
    ["150", "5", "140", "933380", "1000650", "PASS"],
    ["150", "6", "140", "933380", "1000650", "PASS"],
    ["150", "7", "172", "1146724", "1000650", "FAIL"],
    ["200", "7", "172", "1146724", "1334200", "PASS"]
  ], 0.55, 1.25, [1.0, 0.8, 1.15, 1.85, 1.9, 1.35], 0.43, 8.6);
  box(s, "결론: 32bit 150m 운용은 cfg<=6 권장, cfg=7은 200m 이상에서 timing PASS", 1.0, 5.55, 11.2, 0.58, { fill: C.orangeFill, line: C.orange, bold: true });
  footer(s, "근거: xsim_distance_maxhits_matrix.log:34-48");
}

{
  const s = pptx.addSlide();
  title(s, "64bit / 128bit 결과", "두 폭 모두 150m에서 cfg=1..7 PASS");
  table(s, [
    ["Width", "cfg 범위", "line beats", "drain ps", "allowed ps", "결과"],
    ["64", "1..4", "70", "466690", "1000650", "PASS"],
    ["64", "5..7", "102", "680034", "1000650", "PASS"],
    ["128", "1..7", "67", "446689", "1000650", "PASS"]
  ], 0.75, 1.55, [1.2, 1.8, 1.8, 1.9, 1.9, 1.35], 0.58, 10);
  box(s, "결론: 64bit와 128bit는 150m부터 max_hits_cfg=7까지 timing 여유가 있습니다.", 1.0, 4.35, 11.2, 0.65, { fill: C.greenFill, line: C.green, bold: true });
  footer(s, "근거: xsim_distance_maxhits_matrix.log:54-86");
}

{
  const s = pptx.addSlide();
  title(s, "종합 경계", "운용 선택 관점의 핵심 표");
  table(s, [
    ["Width", "150m PASS", "최초 FAIL", "재시도", "판단"],
    ["32", "cfg 1..6", "150m cfg=7", "200m cfg=7 PASS", "cfg=7은 200m 이상"],
    ["64", "cfg 1..7", "없음", "없음", "150m부터 full cfg 가능"],
    ["128", "cfg 1..7", "없음", "없음", "150m부터 full cfg 가능"]
  ], 0.7, 1.45, [1.1, 2.2, 2.2, 2.4, 2.7], 0.62, 9.8);
  box(s, "xsim PASS: tb_tdc_gpx_distance_maxhits_matrix", 1.0, 5.1, 11.2, 0.55, { fill: C.greenFill, line: C.green, bold: true });
  footer(s, "Commit 대상에는 계획/결과 문서, PPT, TB를 포함");
}

pptx.writeFile({ fileName: out });
