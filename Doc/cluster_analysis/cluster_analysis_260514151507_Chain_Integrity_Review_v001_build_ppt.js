const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/.pnpm/pptxgenjs@4.0.1/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/cluster_analysis_260514151507_Chain_Integrity_Review_v001.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C01-C06 Cluster Chain Integrity Review";
pptx.title = "C01-C06 Chain Integrity Review v001";
pptx.lang = "ko-KR";
pptx.theme = { headFontFace: "Malgun Gothic", bodyFontFace: "Malgun Gothic", lang: "ko-KR" };

const font = "Malgun Gothic";
const C = {
  bg: "F8FAFC", ink: "111827", muted: "64748B", line: "CBD5E1",
  dark: "0F172A", white: "FFFFFF", blue: "2563EB", blueFill: "DBEAFE",
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
    breakLine: false,
    margin: opt.margin === undefined ? 0.04 : opt.margin,
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
    line: { color: opt.line || C.line, width: 0.9 },
  });
  tx(slide, label, x + 0.08, y + 0.06, w - 0.16, h - 0.12, {
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
  tx(s, "C01-C06", 0.72, 0.72, 5.5, 0.42, { size: 18, bold: true, color: "93C5FD" });
  tx(s, "Cluster Chain\nIntegrity Review", 0.72, 1.25, 8.8, 1.15, { size: 31, bold: true, color: C.white });
  tx(s, "사용자 탑 쌓기 리뷰를 최신 C06 v006 hardening 기준으로 보정하고, C07에서 닫아야 할 P0/P1 chain 작업으로 전개합니다.", 0.78, 2.72, 11.5, 0.6, { size: 13.5, color: "E5E7EB" });
  box(s, "C06 내부 blocker\n닫힘", 0.95, 4.12, 2.45, 0.9, { fill: "14532D", line: "86EFAC", color: C.white, bold: true, size: 12 });
  box(s, "전체 release chain\n조건부", 3.95, 4.12, 2.45, 0.9, { fill: "713F12", line: "FDE68A", color: C.white, bold: true, size: 12 });
  box(s, "C07 system closure\n필수", 6.95, 4.12, 2.45, 0.9, { fill: "1E3A8A", line: "93C5FD", color: C.white, bold: true, size: 12 });
  box(s, "Datasheet\n절대 기준", 9.95, 4.12, 2.45, 0.9, { fill: "4C1D95", line: "C4B5FD", color: C.white, bold: true, size: 12 });
  footer(s, "생성 2026-05-14 15:15:07 KST | 기준: Doc/TDC-GPX-Datasheet.pdf");
}

{
  const s = pptx.addSlide();
  title(s, "최신 v006 기준 재판정", "C06 관련 stale 위험은 제거하고, 구조적으로 남은 chain 위험만 C07로 승격합니다.");
  table(s, [
    ["항목", "v006 기준 판단", "조치"],
    ["Recovery 폭", "32/64/128 force/soft PASS", "닫힘"],
    ["Bounded stall", "32/64/128 beat/tlast PASS", "장기 stall은 C07"],
    ["Lane stall", "rise-only/fall-only PASS", "C03 내부 matrix는 P1"],
    ["False positive", "source/dest/effect/output marker 도입", "C01-C04 audit"],
    ["Output CDC 전체 재설계", "bounded stall만으로는 전체 closure 아님", "C07 P0"],
    ["8 us reserve", "RTL/xsim 측정값 아님", "C07 P0"],
  ], 0.62, 1.35, [2.2, 6.2, 3.0], 0.47, 7.0);
  footer(s, "근거: C06 Result v006, C06 Handoff v002, C02 Readiness Review v001");
}

{
  const s = pptx.addSlide();
  title(s, "Chain Map", "형식적 handoff는 연결되어 있으나, output CDC/ready risk는 C07에서 명시적으로 닫아야 합니다.");
  const nodes = [
    ["C01\nBus Read", 0.75, C.blueFill, C.blue],
    ["C02\nAcquisition", 2.75, C.tealFill, C.teal],
    ["C03\nCell Pipe", 4.75, C.orangeFill, C.orange],
    ["C04\nOutput", 6.75, C.purpleFill, C.purple],
    ["C06\nControl", 8.75, C.greenFill, C.green],
    ["C07\nSystem", 10.75, C.redFill, C.red],
  ];
  nodes.forEach(([label, x, fill, line], i) => {
    box(s, label, x, 2.0, 1.45, 0.92, { fill, line, bold: true, size: 10.5 });
    if (i < nodes.length - 1) arrow(s, x + 1.48, 2.46, nodes[i + 1][1] - 0.08, 2.46);
  });
  tx(s, "9 contracts", 1.75, 1.62, 1.0, 0.24, { size: 7.5, color: C.muted, align: "center" });
  tx(s, "10 contracts\n+ CDC handoff", 3.50, 1.50, 1.4, 0.42, { size: 7.1, color: C.muted, align: "center" });
  tx(s, "Hit[16]\npolicy", 5.80, 1.50, 1.0, 0.42, { size: 7.1, color: C.muted, align: "center" });
  tx(s, "ready-high\nassumption", 7.72, 1.50, 1.1, 0.42, { size: 7.1, color: C.muted, align: "center" });
  tx(s, "hardened\ncontract", 9.70, 1.50, 1.0, 0.42, { size: 7.1, color: C.muted, align: "center" });
  box(s, "C02: output stream CDC 전체 재설계\nC03: direct matrix 부족\nC04: ready/header pending 직접 증거 부족\nC06: reserve/system stall은 C07 조건", 1.25, 4.5, 10.8, 1.0, { fill: C.redFill, line: C.red, bold: true, size: 12 });
  footer(s, "의미: C06 v006은 강화되었지만 C02 handoff의 architecture closure까지 대체하지는 않습니다.");
}

{
  const s = pptx.addSlide();
  title(s, "Layer별 강도", "기초는 단단하지만 위쪽 layer에서 직접 검증 깊이가 얕아진 지점이 있습니다.");
  table(s, [
    ["Layer", "최신 판단", "남은 위험"],
    ["C01", "Strong", "PASS marker audit 권고"],
    ["C02", "Strong with handoff", "output CDC 전체 재설계 후속"],
    ["C03", "Thin direct evidence", "dual-buffer, IFIFO2, drop/quarantine matrix"],
    ["C04", "Partial direct evidence", "ready path, header pending timing"],
    ["C06", "Hardened conditional", "reserve, 장기 stall, architecture closure"],
  ], 0.75, 1.45, [1.4, 3.5, 6.4], 0.56, 7.8);
  footer(s, "판정 기준: Datasheet, RTL/file evidence, xsim log, handoff lineage");
}

{
  const s = pptx.addSlide();
  title(s, "Output CDC / Ready Risk", "이 위험은 C02 handoff에서 시작되어 C06 ready-high finding으로 다시 노출된 같은 계열입니다.");
  const xs = [1.0, 3.2, 5.4, 7.6, 9.8];
  const labels = [
    "C02\nraw CDC closed",
    "Handoff\n전체 stream CDC",
    "C03/C04\nwidth/sanitize",
    "C06\nbounded stall",
    "C07\narchitecture closure",
  ];
  labels.forEach((label, i) => {
    const fill = i === 1 || i === 4 ? C.redFill : C.blueFill;
    const line = i === 1 || i === 4 ? C.red : C.blue;
    box(s, label, xs[i], 2.0, 1.55, 0.92, { fill, line, bold: true, size: 9.3 });
    if (i < labels.length - 1) arrow(s, xs[i] + 1.58, 2.46, xs[i + 1] - 0.08, 2.46);
  });
  box(s, "v006 bounded stall PASS는 중요하지만, '전체 CDC 재설계가 완료됐다'는 증거는 아닙니다.\nC07에서 현재 구조 수락 / patch 필요 / contract exception 중 하나로 결론을 내려야 합니다.", 1.0, 4.35, 11.3, 1.0, { fill: C.orangeFill, line: C.orange, bold: true, size: 12.5 });
  footer(s, "근거: C02 Readiness Review v001 section 319/337/379, C06 Analysis F-C06-A02");
}

{
  const s = pptx.addSlide();
  title(s, "False Positive 방지 기준", "PASS는 source 한쪽 marker가 아니라 네 계층을 모두 봐야 합니다.");
  const xs = [1.15, 3.9, 6.65, 9.4];
  const labels = [
    ["Source", "명령/입력 발생"],
    ["Destination", "대상 clock domain 도착"],
    ["Effect", "FSM/data path 반응"],
    ["Output", "외부 결과 보존"],
  ];
  labels.forEach(([a, b], i) => {
    box(s, `${a}\n${b}`, xs[i], 2.2, 1.8, 0.95, { fill: [C.orangeFill, C.blueFill, C.purpleFill, C.greenFill][i], line: [C.orange, C.blue, C.purple, C.green][i], bold: true, size: 9.5 });
    if (i < labels.length - 1) arrow(s, xs[i] + 1.83, 2.68, xs[i + 1] - 0.08, 2.68);
  });
  table(s, [
    ["소급 audit 대상", "확인"],
    ["C02 OP-C02-04", "downstream tuser source/dest/effect/output"],
    ["C04 Hit[16] sanitize", "입력 Hit[16] -> sanitize effect -> final stream"],
    ["C04 width beat/tlast", "header/face source -> final AXIS output"],
  ], 1.15, 4.35, [3.2, 7.6], 0.45, 8.0);
  footer(s, "C06 v006 recovery 검증은 이 marker 기준을 이미 적용했습니다.");
}

{
  const s = pptx.addSlide();
  title(s, "C07 Action Plan", "남은 위험은 C07 System Integration / Chain Hardening 계획으로 넘깁니다.");
  table(s, [
    ["Priority", "ID", "작업", "완료 기준"],
    ["P0", "CHAIN-01", "output stream CDC closure", "현재 구조 수락 또는 patch/exception 결정"],
    ["P0", "CHAIN-02", "C02->C06 stall stress TB", "source/dest/effect/output PASS"],
    ["P0", "CHAIN-03", "C02/C04 marker audit", "false positive 방지 표 작성"],
    ["P0", "CHAIN-04", "8 us reserve 실측", "거리/margin 재산출"],
    ["P1", "CHAIN-05", "C03 direct matrix", "dual-buffer/IFIFO2/drop 검증"],
    ["P1", "CHAIN-06", "C04 ready/header TB", "pending/tlast/frame_done 검증"],
  ], 0.55, 1.32, [0.9, 1.3, 4.1, 5.4], 0.43, 6.6);
  footer(s, "상세 계획: C07_System_Integration_260514151507_Plan_v001.md");
}

pptx.writeFile({ fileName: out });
