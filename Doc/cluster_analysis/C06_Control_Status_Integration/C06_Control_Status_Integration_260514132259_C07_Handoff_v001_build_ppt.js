const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/.pnpm/pptxgenjs@4.0.1/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C06_Control_Status_Integration/C06_Control_Status_Integration_260514132259_C07_Handoff_v001.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C06 to C07 Handoff v001";
pptx.title = "C06 to C07 Handoff v001";
pptx.lang = "ko-KR";
pptx.theme = { headFontFace: "Malgun Gothic", bodyFontFace: "Malgun Gothic", lang: "ko-KR" };

const font = "Malgun Gothic";
const C = {
  bg: "F8FAFC", ink: "111827", muted: "64748B", line: "CBD5E1",
  white: "FFFFFF", dark: "0F172A", blue: "2563EB", blueFill: "DBEAFE",
  green: "15803D", greenFill: "DCFCE7", orange: "C2410C", orangeFill: "FFEDD5",
  red: "B91C1C", redFill: "FEE2E2", purple: "6D28D9", purpleFill: "EDE9FE",
  teal: "0F766E", tealFill: "CCFBF1", slate: "E2E8F0",
};

function tx(slide, value, x, y, w, h, opt = {}) {
  slide.addText(value, {
    x, y, w, h, fontFace: font, fontSize: opt.size || 10, bold: !!opt.bold,
    color: opt.color || C.ink, align: opt.align || "left", valign: "mid",
    fit: "shrink", margin: opt.margin === undefined ? 0.04 : opt.margin,
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
    fill: { color: opt.fill || C.white }, line: { color: opt.line || C.line, width: 0.9 },
  });
  tx(slide, label, x + 0.08, y + 0.06, w - 0.16, h - 0.12, {
    size: opt.size || 9.0, bold: opt.bold, color: opt.color || C.ink, align: opt.align || "center",
  });
}
function arrow(slide, x1, y1, x2, y2, color = C.muted) {
  slide.addShape(pptx.ShapeType.line, { x: x1, y: y1, w: x2 - x1, h: y2 - y1, line: { color, width: 1.25, endArrowType: "triangle" } });
}
function table(slide, rows, x, y, widths, rowH, size = 7.5) {
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
  tx(s, "C06 -> 다음 단계", 0.72, 0.88, 7.8, 0.56, { size: 27, bold: true, color: C.white });
  tx(s, "Control/Status Integration Handoff v001", 0.76, 1.62, 8.7, 0.42, { size: 15, color: "CBD5E1" });
  tx(s, "C06은 GO_WITH_CONTRACT입니다. 다음 단계는 C06 내부 수정이 아니라 시스템 소비자 계약 수락입니다.", 0.78, 2.55, 11.4, 0.55, { size: 13, color: C.white });
  box(s, "I-Mode single\nclose", 0.95, 4.05, 2.35, 0.88, { fill: "14532D", line: "86EFAC", color: C.white, bold: true });
  box(s, "32/64/128\nbaseline PASS", 3.75, 4.05, 2.35, 0.88, { fill: "1E3A8A", line: "93C5FD", color: C.white, bold: true });
  box(s, "force/soft\nrecovery PASS", 6.55, 4.05, 2.35, 0.88, { fill: "14532D", line: "86EFAC", color: C.white, bold: true });
  box(s, "system\nreserve open", 9.35, 4.05, 2.35, 0.88, { fill: "713F12", line: "FDE68A", color: C.white, bold: true });
  footer(s, "archive: sim_results/vivado_xsim/sessions/260514132259_c06_v004_recovery/");
}

{
  const s = pptx.addSlide();
  title(s, "인계 계약", "다음 단계는 아래 계약을 수락한 뒤 시스템 관점 검증으로 확장합니다.");
  table(s, [
    ["ID", "계약", "상태"],
    ["H-01", "운용 범위는 I-Mode single", "Accepted"],
    ["H-02", "GPX timing은 C01/C02 Datasheet 계약 상속", "Accepted"],
    ["H-03", "AXI recovery pulse는 TDC pulse로 보존", "Closed"],
    ["H-04", "force_reinit -> PH_INIT -> next run", "Closed"],
    ["H-05", "soft_reset -> PH_RESP_DRAIN -> PH_INIT", "Closed"],
    ["H-06", "face/status recovery boundary reset", "Closed"],
    ["H-07", "Hit[16] final output 제거 정책 유지", "Accepted"],
  ], 0.74, 1.45, [0.95, 7.45, 2.9], 0.48, 7.0);
  footer(s, "근거: C06_Control_Status_Integration_260514132259_C07_Handoff_v001.md section 3");
}

{
  const s = pptx.addSlide();
  title(s, "Data / Control / Status 경계", "C01~C06 내부를 다시 여는 것이 아니라, final stream과 status 소비 계약을 닫습니다.");
  const nodes = [
    ["C01\nGPX bus", 0.75, C.blueFill, C.blue],
    ["C02\nacquire", 2.45, C.blueFill, C.blue],
    ["C03\ncell pipe", 4.15, C.tealFill, C.teal],
    ["C04\noutput", 5.85, C.greenFill, C.green],
    ["C06\ncontrol/status", 7.75, C.purpleFill, C.purple],
    ["C07/system\nVDMA/PS", 10.0, C.orangeFill, C.orange],
  ];
  nodes.forEach(([label, x, fill, line], i) => {
    box(s, label, x, 2.15, 1.25, 0.92, { fill, line, bold: true, size: 8.2 });
    if (i < nodes.length - 1) arrow(s, x + 1.28, 2.61, nodes[i + 1][1] - 0.05, 2.61);
  });
  table(s, [
    ["경계", "다음 단계 수락 내용"],
    ["final stream", "rise/fall lane, SOF/tlast, 32/64/128 width parser"],
    ["status", "STAT5/6/7 polling, o_irq_pipe reserved"],
    ["timing", "C06 II는 TB scenario interval과 RTL latency를 분리해서 해석"],
    ["system budget", "VDMA/PS/Ethernet 8 us reserve는 별도 실측/검증"],
  ], 0.95, 4.05, [2.4, 8.8], 0.50, 7.7);
  footer(s, "C06 handoff는 system consumer contract로 확장");
}

{
  const s = pptx.addSlide();
  title(s, "Latency / Throughput / II", "다음 단계에서 그대로 받아야 하는 C06 관측값입니다.");
  table(s, [
    ["Metric", "값", "주의"],
    ["T0 -> T1", "11 clk / 55 ns", "fire_count final"],
    ["T0 -> T2", "92 clk / 460 ns", "IrFlag TB timing"],
    ["T0 -> TLAST", "189~192 clk", "64-bit small TB"],
    ["shot-to-shot T0", "2107 clk / 10.535 us", "TB interval, 최소 II 아님"],
    ["source -> TDC", "20 ns / 4 clk", "recovery CDC"],
    ["recovery throughput", "88 beats / tlast 4", "64-bit force/soft"],
  ], 0.75, 1.55, [2.7, 3.0, 5.75], 0.54, 8.0);
  box(s, "핵심: C06 recovery는 steady-state shot pipeline을 악화시키지 않고, run-to-run recovery boundary만 닫습니다.", 0.95, 5.65, 11.25, 0.65, { fill: C.greenFill, line: C.green, bold: true, size: 11 });
  footer(s, "근거: C06 Result v005 section 5, official xsim logs");
}

{
  const s = pptx.addSlide();
  title(s, "다음 Cluster 선택지", "초기 계획은 C06까지 정의되어 있으므로 C07은 목적을 먼저 정해야 합니다.");
  table(s, [
    ["후보", "목적", "권고 상황"],
    ["C07_System_Integration", "VDMA/PS/Ethernet, parser, board reserve", "시스템 연결 검증을 계속할 때"],
    ["C07_Release_Readiness", "synthesis/STA, sim report, release checklist", "release 준비 단계일 때"],
    ["Final Report", "C01~C06 운용 개념과 검증 패키지 통합", "추가 RTL 분석이 없을 때"],
  ], 0.72, 1.55, [2.8, 5.25, 3.75], 0.62, 8.0);
  box(s, "인계 결론: C06은 GO_WITH_CONTRACT. 다음 단계는 내부 재수정이 아니라 계약 수락과 시스템 검증입니다.", 0.95, 5.65, 11.2, 0.65, { fill: C.blueFill, line: C.blue, bold: true, size: 11 });
  footer(s, "C06 -> C07 Handoff v001");
}

pptx.writeFile({ fileName: out });
