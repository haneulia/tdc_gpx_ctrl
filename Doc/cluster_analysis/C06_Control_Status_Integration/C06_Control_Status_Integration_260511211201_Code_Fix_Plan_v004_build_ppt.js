const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C06_Control_Status_Integration/C06_Control_Status_Integration_260511211201_Code_Fix_Plan_v004.pptx";
const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C06 Code Fix Plan v004";
pptx.title = "C06 Code Fix Plan v004";
pptx.lang = "ko-KR";
pptx.theme = { headFontFace: "Malgun Gothic", bodyFontFace: "Malgun Gothic", lang: "ko-KR" };

const font = "Malgun Gothic";
const C = {
  bg: "F8FAFC", ink: "111827", muted: "64748B", line: "CBD5E1",
  white: "FFFFFF", dark: "0F172A", blue: "2563EB", blueFill: "DBEAFE",
  green: "16A34A", greenFill: "DCFCE7", orange: "EA580C", orangeFill: "FFEDD5",
  red: "DC2626", redFill: "FEE2E2", purple: "7C3AED", purpleFill: "F3E8FF", slate: "E2E8F0",
};

function tx(slide, value, x, y, w, h, opt = {}) {
  slide.addText(value, { x, y, w, h, fontFace: font, fontSize: opt.size || 10, bold: !!opt.bold,
    color: opt.color || C.ink, align: opt.align || "left", valign: "mid", fit: "shrink", margin: 0.04 });
}
function title(slide, main, sub) {
  slide.background = { color: C.bg };
  tx(slide, main, 0.55, 0.28, 12.1, 0.46, { size: 21, bold: true });
  tx(slide, sub, 0.58, 0.78, 12.0, 0.3, { size: 9.2, color: C.muted });
  slide.addShape(pptx.ShapeType.line, { x: 0.58, y: 1.13, w: 12.1, h: 0, line: { color: C.line, width: 0.8 } });
}
function box(slide, value, x, y, w, h, opt = {}) {
  slide.addShape(pptx.ShapeType.roundRect, { x, y, w, h, rectRadius: 0.04,
    fill: { color: opt.fill || C.white }, line: { color: opt.line || C.line, width: 1 } });
  tx(slide, value, x + 0.08, y + 0.06, w - 0.16, h - 0.12,
    { size: opt.size || 10, bold: opt.bold, color: opt.color || C.ink, align: opt.align || "center" });
}
function arrow(slide, x1, y1, x2, y2, color = C.muted) {
  slide.addShape(pptx.ShapeType.line, { x: x1, y: y1, w: x2 - x1, h: y2 - y1,
    line: { color, width: 1.3, endArrowType: "triangle" } });
}
function footer(slide, value) { tx(slide, value, 0.6, 7.04, 12.1, 0.24, { size: 7.4, color: C.muted, align: "center" }); }

{
  const s = pptx.addSlide();
  s.background = { color: C.dark };
  tx(s, "C06 Code Fix Plan v004", 0.72, 0.8, 8.8, 0.58, { size: 28, bold: true, color: C.white });
  tx(s, "soft_reset recovery open을 RTL 보완 또는 운용 계약으로 닫는 단계", 0.76, 1.55, 9.9, 0.42, { size: 15, color: "CBD5E1" });
  box(s, "P1\n원인 분해", 0.9, 3.2, 2.2, 0.95, { fill: "1E3A8A", line: "93C5FD", color: C.white, bold: true });
  box(s, "P1\n정책 결정", 3.65, 3.2, 2.2, 0.95, { fill: "7C2D12", line: "FDBA74", color: C.white, bold: true });
  box(s, "P1\n구현/검증", 6.4, 3.2, 2.2, 0.95, { fill: "14532D", line: "86EFAC", color: C.white, bold: true });
  box(s, "P2\nhandoff 갱신", 9.15, 3.2, 2.2, 0.95, { fill: "4C1D95", line: "C4B5FD", color: C.white, bold: true });
  footer(s, "생성 2026-05-11 21:12:01 KST | 기준: Result v003 / Datasheet I-Mode single");
}

{
  const s = pptx.addSlide();
  title(s, "분석 대상 경로", "soft_reset이 PH_RESP_DRAIN에서 chip_busy를 해제하지 못하는 지점을 분리한다.");
  const nodes = [
    ["MAIN_CTRL[30]\nsoft_reset", 0.8, 2.1, C.orangeFill, C.orange],
    ["CDC pulse\nconfig_ctrl", 2.75, 2.1, C.blueFill, C.blue],
    ["chip_ctrl\nPH_RESP_DRAIN", 4.75, 2.1, C.redFill, C.red],
    ["bus idle?\nbusy/rsp", 6.95, 2.1, C.purpleFill, C.purple],
    ["PH_INIT\nor quarantine", 9.15, 2.1, C.greenFill, C.green],
  ];
  nodes.forEach(([label, x, y, fill, line], i) => {
    box(s, label, x, y, 1.55, 1.0, { fill, line, bold: true, size: 9.2 });
    if (i < nodes.length - 1) arrow(s, x + 1.58, y + 0.5, nodes[i + 1][1] - 0.06, y + 0.5);
  });
  tx(s, "계측 대상: i_bus_busy, i_bus_rsp_pending, s_drain_to_init_r, s_err_drain_cap_r, s_err_bus_fatal_r, s_chip_busy", 0.9, 4.25, 11.3, 0.45, { size: 12, bold: true });
  tx(s, "완료 조건: soft_reset PASS를 보장하거나, force_reinit 전용 recovery 계약으로 handoff 문서와 TB를 정렬한다.", 0.9, 4.95, 11.3, 0.5, { size: 11.5 });
  footer(s, "C06 Plan v004 | 다음 실행: focused probe -> RTL/계약 선택 -> 회귀");
}

pptx.writeFile({ fileName: out });
