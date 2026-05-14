const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/.pnpm/pptxgenjs@4.0.1/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C06_Control_Status_Integration/C06_Control_Status_Integration_260514134116_Review_v001.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C06 Review v001";
pptx.title = "C06 Review v001";
pptx.lang = "ko-KR";
pptx.theme = { headFontFace: "Malgun Gothic", bodyFontFace: "Malgun Gothic", lang: "ko-KR" };

const font = "Malgun Gothic";
const C = {
  bg: "F8FAFC", ink: "111827", muted: "64748B", line: "CBD5E1",
  white: "FFFFFF", dark: "111827", blue: "2563EB", blueFill: "DBEAFE",
  green: "15803D", greenFill: "DCFCE7", orange: "C2410C", orangeFill: "FFEDD5",
  red: "B91C1C", redFill: "FEE2E2", purple: "6D28D9", purpleFill: "EDE9FE",
  teal: "0F766E", tealFill: "CCFBF1", slate: "E2E8F0",
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
  tx(slide, main, 0.55, 0.28, 12.1, 0.46, { size: 21, bold: true });
  tx(slide, sub, 0.58, 0.78, 12.0, 0.3, { size: 9.2, color: C.muted });
  slide.addShape(pptx.ShapeType.line, { x: 0.58, y: 1.13, w: 12.1, h: 0, line: { color: C.line, width: 0.8 } });
}

function footer(slide, value) {
  tx(slide, value, 0.58, 7.05, 12.15, 0.24, { size: 7.2, color: C.muted, align: "center" });
}

function box(slide, label, x, y, w, h, opt = {}) {
  slide.addShape(pptx.ShapeType.roundRect, {
    x, y, w, h, rectRadius: 0.04,
    fill: { color: opt.fill || C.white },
    line: { color: opt.line || C.line, width: 0.9 },
  });
  tx(slide, label, x + 0.08, y + 0.06, w - 0.16, h - 0.12, {
    size: opt.size || 9.0,
    bold: opt.bold,
    color: opt.color || C.ink,
    align: opt.align || "center",
  });
}

function arrow(slide, x1, y1, x2, y2, color = C.muted) {
  slide.addShape(pptx.ShapeType.line, {
    x: x1, y: y1, w: x2 - x1, h: y2 - y1,
    line: { color, width: 1.2, endArrowType: "triangle" },
  });
}

function table(slide, rows, x, y, widths, rowH, size = 7.4) {
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
  tx(s, "C06 Review v001", 0.72, 0.82, 8.2, 0.55, { size: 28, bold: true, color: C.white });
  tx(s, "사용자 종합 평가 재판정과 v006 hardening 방향", 0.76, 1.58, 10.4, 0.42, { size: 15, color: "CBD5E1" });
  box(s, "VALID\nfalse positive lesson", 0.92, 3.05, 2.55, 1.0, { fill: "7F1D1D", line: "FCA5A5", color: C.white, bold: true, size: 12 });
  box(s, "PARTIAL\nPASS wording", 3.92, 3.05, 2.55, 1.0, { fill: "713F12", line: "FDE68A", color: C.white, bold: true, size: 12 });
  box(s, "VALID\nwidth/reserve risk", 6.92, 3.05, 2.55, 1.0, { fill: "1E3A8A", line: "93C5FD", color: C.white, bold: true, size: 12 });
  box(s, "PLAN\nv006 hardening", 9.92, 3.05, 2.55, 1.0, { fill: "14532D", line: "86EFAC", color: C.white, bold: true, size: 12 });
  tx(s, "판정: v005는 Absolute PASS가 아니라 GO_WITH_CONTRACT입니다.", 0.78, 5.35, 11.8, 0.36, { size: 12.8, bold: true, color: C.white });
  footer(s, "생성 2026-05-14 13:41:16 KST | 기준: C06 Result v005, C07 Handoff v001");
}

{
  const s = pptx.addSlide();
  title(s, "재판정 Matrix", "사용자 리뷰 입력을 최신 v005 문서 기준으로 분류했습니다.");
  table(s, [
    ["ID", "지적", "판단", "v006 조치"],
    ["RV-01", "v003 false positive", "Valid", "C01~C04 marker audit"],
    ["RV-02", "절대 PASS 표현 위험", "Partial", "GO_WITH_CONTRACT 재강조"],
    ["RV-03", "64-bit recovery만 직접 검증", "Valid", "32/64/128 force/soft sweep"],
    ["RV-04", "8 us reserve 가정", "Valid", "reserve sweep / 실측 계약"],
    ["RV-05", "Hit[16] 폐기 거리 위험", "Valid", "SW/range contract"],
    ["RV-06", "lane imbalance", "Partial", "top-level stall 결합 stress"],
    ["RV-07", "sticky clear 차이", "Partial", "SW-visible clear map"],
  ], 0.58, 1.38, [0.9, 3.55, 1.25, 5.75], 0.43, 6.7);
  footer(s, "근거: C06_Control_Status_Integration_260514134116_Review_v001.md section 2");
}

{
  const s = pptx.addSlide();
  title(s, "PASS Marker 원칙", "PASS는 source, destination, effect를 모두 관측해야 닫힙니다.");
  const nodes = [
    ["Source marker\nAXI command / input event", 1.0, C.orangeFill, C.orange],
    ["Destination marker\nTDC domain / registered boundary", 4.3, C.blueFill, C.blue],
    ["Effect marker\nFSM phase / stream / sticky", 7.6, C.purpleFill, C.purple],
    ["PASS\nall observed", 10.6, C.greenFill, C.green],
  ];
  nodes.forEach(([label, x, fill, line], i) => {
    box(s, label, x, 2.15, 2.0, 0.95, { fill, line, bold: true, size: 8.2 });
    if (i < nodes.length - 1) arrow(s, x + 2.02, 2.62, nodes[i + 1][1] - 0.08, 2.62);
  });
  table(s, [
    ["v004/v005 근거", "의미"],
    ["force/soft log line 142", "source pulse"],
    ["force/soft log line 146", "TDC domain pulse"],
    ["force line 148/150/152/154, soft line 148/156", "chip_ctrl effect"],
    ["force line 231/232/236/238, soft line 239/240/244/246", "output preservation"],
  ], 1.0, 4.02, [4.9, 6.2], 0.46, 7.1);
  footer(s, "v003 false positive 교훈을 이후 Cluster 검증 기준으로 승격");
}

{
  const s = pptx.addSlide();
  title(s, "Hit[16] 폐기의 정량 의미", "현재 generation 정책은 유지하되, SW가 거리 한계를 명시적으로 수락해야 합니다.");
  box(s, "16-bit direct\n5.308416 us", 0.95, 1.72, 2.6, 0.95, { fill: C.blueFill, line: C.blue, bold: true, size: 12 });
  box(s, "round-trip distance\n약 796 m", 3.95, 1.72, 2.6, 0.95, { fill: C.greenFill, line: C.green, bold: true, size: 12 });
  box(s, "810 m round-trip\n약 5.404 us", 6.95, 1.72, 2.6, 0.95, { fill: C.orangeFill, line: C.orange, bold: true, size: 12 });
  box(s, "excess\n약 95 ns", 9.95, 1.72, 2.6, 0.95, { fill: C.redFill, line: C.red, bold: true, size: 12 });
  table(s, [
    ["계약", "내용"],
    ["Datasheet raw", "IFIFO data는 Hit[16:0]"],
    ["final stream", "이번 generation에서는 Hit[16] 폐기"],
    ["SW parser", "16-bit hit slot과 wrap/overflow 정책 수락 필요"],
  ], 1.0, 3.65, [2.4, 8.9], 0.55, 8.2);
  footer(s, "계산 기준: 사용자 운용 분석의 81 ps bin assumption");
}

{
  const s = pptx.addSlide();
  title(s, "v006 우선순위", "GO_WITH_CONTRACT를 release/system integration 전 hardening 계획으로 연결합니다.");
  table(s, [
    ["우선순위", "항목", "이유"],
    ["P0", "32/64/128 force/soft recovery sweep", "64-bit 단일 직접 검증 의존 제거"],
    ["P0", "8 us reserve measurement/sweep", "polygon budget 핵심 가정"],
    ["P0", "output backpressure + polygon stress", "ready high 가정 해제"],
    ["P1", "C01~C04 PASS marker audit", "false positive 패턴 전파 여부 확인"],
    ["P1", "Hit[16] SW/range contract", "거리 정책과 parser 해석 연결"],
    ["P2", "sticky / lane imbalance 보강", "release 관찰성 강화"],
  ], 0.68, 1.45, [1.25, 5.05, 5.05], 0.52, 7.5);
  footer(s, "후속 계획: C06_Control_Status_Integration_260514134116_Code_Fix_Plan_v006.md");
}

pptx.writeFile({ fileName: out });
