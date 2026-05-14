const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/.pnpm/pptxgenjs@4.0.1/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C07_System_Integration/C07_System_Integration_260514151507_Plan_v001.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C07 System Integration Plan";
pptx.title = "C07 System Integration Plan v001";
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
    line: { color: opt.line || C.line, width: 0.9 },
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
  tx(s, "C07", 0.72, 0.72, 3.0, 0.42, { size: 18, bold: true, color: "93C5FD" });
  tx(s, "System Integration\nChain Hardening Plan", 0.72, 1.25, 9.5, 1.15, { size: 29, bold: true, color: C.white });
  tx(s, "C06는 GO_WITH_HARDENED_CONTRACT입니다. C07은 release 판단 전에 output CDC, chain stall, C03/C04 직접 검증, reserve 실측을 닫습니다.", 0.78, 2.72, 11.5, 0.6, { size: 13.2, color: "E5E7EB" });
  box(s, "P0\nCDC closure", 1.0, 4.15, 2.2, 0.9, { fill: "1E3A8A", line: "93C5FD", color: C.white, bold: true, size: 12 });
  box(s, "P0\nChain stress", 3.65, 4.15, 2.2, 0.9, { fill: "14532D", line: "86EFAC", color: C.white, bold: true, size: 12 });
  box(s, "P0\nReserve", 6.3, 4.15, 2.2, 0.9, { fill: "713F12", line: "FDE68A", color: C.white, bold: true, size: 12 });
  box(s, "P1\nC03/C04 matrix", 8.95, 4.15, 2.2, 0.9, { fill: "4C1D95", line: "C4B5FD", color: C.white, bold: true, size: 12 });
  footer(s, "생성 2026-05-14 15:15:07 KST | Vivado: C:\\AMDDesignTools\\2025.2.1\\Vivado");
}

{
  const s = pptx.addSlide();
  title(s, "C07 Scope", "RTL 내부 C06 수정이 아니라 system/release chain closure가 목적입니다.");
  table(s, [
    ["구분", "포함", "설명"],
    ["I-Mode single", "YES", "프로젝트 운용 기준"],
    ["Quiet/M/continuous", "NO", "이번 generation 제외"],
    ["32/64/128 width", "YES", "256-bit 제외"],
    ["Hit[16] final 보존", "NO", "이번 generation 폐기 정책"],
    ["Hit[16] SW 계약", "YES", "16-bit slot 한계 명시"],
    ["VDMA/PS/Ethernet reserve", "YES", "polygon budget 최종 근거"],
  ], 0.75, 1.38, [2.6, 1.2, 7.4], 0.48, 7.2);
  footer(s, "기준: C06 Handoff v002 + Chain Integrity Review v001");
}

{
  const s = pptx.addSlide();
  title(s, "P0 Closure", "release 판단 전에 반드시 닫아야 하는 네 가지입니다.");
  table(s, [
    ["ID", "작업", "완료 기준"],
    ["P0-01", "Output stream CDC 전체 재설계 closure", "현재 구조 수락 또는 patch/exception 결정"],
    ["P0-02", "C02->C06 output ready/stall stress TB", "source/dest/effect/output PASS"],
    ["P0-03", "C02/C04 marker audit", "false positive 방지 marker 표"],
    ["P0-04", "8 us reserve 실측/보수치 갱신", "거리/margin table 재산출"],
  ], 0.65, 1.5, [1.4, 5.1, 5.2], 0.58, 7.3);
  footer(s, "P0는 C07 release readiness 이전 blocker입니다.");
}

{
  const s = pptx.addSlide();
  title(s, "End-to-End Stress TB", "C06 v006 bounded stall을 C02->C03->C04->C06 전체 data chain으로 확장합니다.");
  const xs = [0.9, 3.2, 5.5, 7.8, 10.1];
  const labels = ["C02\nraw/event", "C03\ncell_builder", "C04\nface/header", "C06\ntop/status", "Final AXIS\nstall monitor"];
  labels.forEach((label, i) => {
    box(s, label, xs[i], 2.15, 1.65, 0.92, { fill: [C.tealFill, C.orangeFill, C.purpleFill, C.greenFill, C.redFill][i], line: [C.teal, C.orange, C.purple, C.green, C.red][i], bold: true, size: 9.3 });
    if (i < labels.length - 1) arrow(s, xs[i] + 1.68, 2.61, xs[i + 1] - 0.08, 2.61);
  });
  table(s, [
    ["조건", "값"],
    ["width", "32 / 64 / 128"],
    ["max_hits_cfg", "1 / 3 / 5 / 7 또는 TB capability 단계 sweep"],
    ["stall", "bounded + lane-only"],
    ["marker", "source / destination / effect / output"],
  ], 1.2, 4.25, [2.6, 8.6], 0.42, 7.5);
  footer(s, "archive: sim_results/vivado_xsim/sessions/<stamp>_c07_chain_stress/");
}

{
  const s = pptx.addSlide();
  title(s, "P1 Matrix", "C03/C04에서 통합 PASS로 우회된 직접 검증을 보강합니다.");
  table(s, [
    ["영역", "검증"],
    ["C03", "dual-buffer next-shot II"],
    ["C03", "IFIFO2 wait / timeout"],
    ["C03", "shot drop / quarantine"],
    ["C03", "width / max_hits matrix"],
    ["C04", "face_assembler ready path"],
    ["C04", "header_inserter face_start pending"],
    ["C04", "header/data backpressure"],
  ], 0.8, 1.35, [2.0, 9.3], 0.47, 7.6);
  footer(s, "목표: C03/C04 layer를 C01/C02 수준에 가깝게 보강");
}

{
  const s = pptx.addSlide();
  title(s, "Timing / Pipeline / II", "C07은 shot deadline까지의 전체 시간 예산을 stage별로 분해해야 합니다.");
  const xs = [0.85, 2.8, 4.75, 6.7, 8.65, 10.6];
  const labels = ["T0\nstart", "T1\nstop/final", "T2\nC02 drain", "T3\nC03 cell", "T4\nC04 output", "T5\nTLAST"];
  labels.forEach((label, i) => {
    box(s, label, xs[i], 2.0, 1.35, 0.8, { fill: C.blueFill, line: C.blue, bold: true, size: 8.7 });
    if (i < labels.length - 1) arrow(s, xs[i] + 1.38, 2.4, xs[i + 1] - 0.08, 2.4);
  });
  box(s, "Reserve: VDMA / PS / Ethernet", 4.25, 3.7, 4.7, 0.65, { fill: C.orangeFill, line: C.orange, bold: true, size: 12 });
  box(s, "Deadline: next start_tdc <= 13.888889 us", 3.55, 4.75, 6.1, 0.65, { fill: C.redFill, line: C.red, bold: true, size: 12 });
  table(s, [
    ["Metric", "측정"],
    ["Latency", "stage별 T0->T5"],
    ["Throughput", "width별 beat count"],
    ["Pipeline", "register/FIFO/CDC 경계"],
    ["II", "next shot accept interval"],
  ], 0.9, 5.75, [2.0, 4.3], 0.32, 6.6);
  footer(s, "운용 기준: 5각 mirror 60도 FOV, point interval 0.05도 -> 13.888889 us");
}

{
  const s = pptx.addSlide();
  title(s, "C07 산출물", "각 결과는 Markdown + PPT + Vivado/xsim archive로 추적합니다.");
  table(s, [
    ["산출물", "파일명 규칙"],
    ["Chain stress result", "C07_System_Integration_<YYMMDDHHMMSS>_Chain_Stress_Result_v001"],
    ["Marker audit result", "C07_System_Integration_<YYMMDDHHMMSS>_Marker_Audit_Result_v001"],
    ["Reserve measurement", "C07_System_Integration_<YYMMDDHHMMSS>_Reserve_Measurement_Result_v001"],
    ["Release readiness", "C07_System_Integration_<YYMMDDHHMMSS>_Release_Readiness_v001"],
  ], 0.6, 1.45, [3.0, 8.4], 0.55, 7.2);
  box(s, "사용자 판단: 현재 구조 수락 여부, reserve 실측/보수값, 810 m 허용 여부, Hit[16] SW 계약", 1.05, 5.45, 11.1, 0.7, { fill: C.redFill, line: C.red, bold: true, size: 11.5 });
  footer(s, "C07 Plan v001은 다음 code/test 작업의 기준 문서입니다.");
}

pptx.writeFile({ fileName: out });
