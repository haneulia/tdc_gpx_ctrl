const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C06_Control_Status_Integration/C06_Control_Status_Integration_260511203322_Code_Fix_Result_v002.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C06 Code Fix Result v002";
pptx.title = "C06 Code Fix Result v002";
pptx.lang = "ko-KR";
pptx.theme = {
  headFontFace: "Malgun Gothic",
  bodyFontFace: "Malgun Gothic",
  lang: "ko-KR",
};

const font = "Malgun Gothic";
const C = {
  bg: "F8FAFC",
  ink: "111827",
  muted: "64748B",
  line: "CBD5E1",
  white: "FFFFFF",
  dark: "0F172A",
  blue: "2563EB",
  blueFill: "DBEAFE",
  green: "16A34A",
  greenFill: "DCFCE7",
  orange: "EA580C",
  orangeFill: "FFEDD5",
  red: "DC2626",
  redFill: "FEE2E2",
  purple: "7C3AED",
  purpleFill: "F3E8FF",
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
    valign: opt.valign || "mid",
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

function box(slide, value, x, y, w, h, opt = {}) {
  slide.addShape(pptx.ShapeType.roundRect, {
    x, y, w, h,
    rectRadius: 0.04,
    fill: { color: opt.fill || C.white },
    line: { color: opt.line || C.line, width: opt.width || 1 },
  });
  tx(slide, value, x + 0.08, y + 0.06, w - 0.16, h - 0.12, {
    size: opt.size || 10,
    bold: opt.bold,
    color: opt.color || C.ink,
    align: opt.align || "center",
  });
}

function footer(slide, value) {
  tx(slide, value, 0.6, 7.04, 12.1, 0.24, { size: 7.4, color: C.muted, align: "center" });
}

function arrow(slide, x1, y1, x2, y2, color = C.muted) {
  slide.addShape(pptx.ShapeType.line, {
    x: x1, y: y1, w: x2 - x1, h: y2 - y1,
    line: { color, width: 1.3, endArrowType: "triangle" },
  });
}

function table(slide, rows, x, y, widths, rowH, size = 8.1) {
  rows.forEach((row, r) => {
    let curX = x;
    row.forEach((cell, c) => {
      slide.addShape(pptx.ShapeType.rect, {
        x: curX, y: y + r * rowH, w: widths[c], h: rowH,
        fill: { color: r === 0 ? C.slate : C.white },
        line: { color: C.line, width: 0.45 },
      });
      tx(slide, cell, curX + 0.04, y + r * rowH + 0.02, widths[c] - 0.08, rowH - 0.04, {
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
  tx(s, "C06", 0.72, 0.58, 1.4, 0.42, { size: 15, bold: true, color: "93C5FD" });
  tx(s, "Control / Status Integration", 0.72, 1.08, 9.2, 0.58, { size: 27, bold: true, color: C.white });
  tx(s, "Code Fix Result v002", 0.76, 1.72, 5.0, 0.38, { size: 15, color: "CBD5E1" });
  tx(s, "직접 xsim 회귀 기준: 32/64/128-bit width, T0~T6 marker, bounded tready stall, rise/fall imbalance 검증 PASS", 0.78, 2.48, 10.8, 0.62, { size: 13.2, color: C.white });
  box(s, "PASS\n32/64/128", 0.86, 3.7, 2.2, 0.95, { fill: "14532D", line: "86EFAC", color: C.white, bold: true });
  box(s, "PASS\nT0~T6 marker", 3.45, 3.7, 2.35, 0.95, { fill: "1E3A8A", line: "93C5FD", color: C.white, bold: true });
  box(s, "PASS\nBackpressure", 6.2, 3.7, 2.35, 0.95, { fill: "7C2D12", line: "FDBA74", color: C.white, bold: true });
  box(s, "OPEN\nRecovery v003", 8.95, 3.7, 2.45, 0.95, { fill: "7F1D1D", line: "FCA5A5", color: C.white, bold: true });
  footer(s, "C06 Code Fix Result v002 | 생성 2026-05-11 20:33:22 KST | 실행 stamp 260511203055");
}

{
  const s = pptx.addSlide();
  title(s, "검증 결과 Matrix", "FP2-C06-01~07은 닫혔고, FP2-C06-08만 v003으로 승계");
  table(s, [
    ["항목", "결과", "근거"],
    ["compile / elab", "PASS + 환경 예외", "sync_fifo 포함 direct compile/elab PASS"],
    ["32-bit output", "PASS", "expected_beats=72, tlast PASS"],
    ["64-bit output", "PASS", "expected_beats=44, tlast PASS"],
    ["128-bit output", "PASS", "expected_beats=38, tlast PASS"],
    ["backpressure", "PASS", "bp_gap=17, stall=906clk, beat/tlast 보존"],
    ["fall-only abort", "PASS", "Scenario F 포함 A~F PASS"],
    ["IRQ/status", "PASS + accepted", "o_irq=0, o_irq_pipe=0, STAT5/6/7 사용"],
    ["recovery", "OPEN", "soft_reset/force_reinit CSR 회귀 v003 승계"],
  ], 0.66, 1.4, [2.35, 2.9, 6.4], 0.5, 8.0);
  footer(s, "근거: xsim_c06_v002_*_260511203055.log");
}

{
  const s = pptx.addSlide();
  title(s, "Timing Block", "T3는 실제 output 완료가 아니라 TB drain wait 종료 marker로 해석해야 한다.");
  const nodes = [
    ["T0\nSTART_TDC", 0.78, 2.0, C.blueFill, C.blue],
    ["T4\nFIRST_BEAT\n+7clk", 2.55, 2.0, C.greenFill, C.green],
    ["T1\nFIRE_FINAL\n+11clk", 4.45, 2.0, C.purpleFill, C.purple],
    ["T2\nIRFLAG\n+92clk", 6.35, 2.0, C.orangeFill, C.orange],
    ["T5\nTLAST\n+189~200clk", 8.25, 2.0, C.greenFill, C.green],
    ["T3\nDRAIN_WAIT_END\n+1092clk", 10.35, 2.0, C.slate, C.muted],
  ];
  nodes.forEach(([label, x, y, fill, line], i) => {
    box(s, label, x, y, 1.35, 1.0, { fill, line, bold: true, size: 9.3 });
    if (i < nodes.length - 1) arrow(s, x + 1.38, y + 0.5, nodes[i + 1][1] - 0.05, y + 0.5);
  });
  tx(s, "관측 결과: output은 T0+7clk에 시작되고, TLAST는 32-bit에서 더 늦다. TB의 T3 drain wait는 보수적 대기 종료 시점이므로 output 완료와 동일시하면 안 된다.", 0.9, 4.25, 11.1, 0.7, { size: 12 });
  footer(s, "근거: top_int width별 C06_MARKER T0~T6 로그");
}

{
  const s = pptx.addSlide();
  title(s, "Latency / Throughput / II", "200 MHz 기준 1 clock = 5 ns");
  table(s, [
    ["폭", "T0->T4", "T0->T2", "T0->T5", "beats", "판정"],
    ["32", "7clk / 35ns", "92clk / 460ns", "197~200clk", "72", "기준 PASS"],
    ["64", "7clk / 35ns", "92clk / 460ns", "189~192clk", "44", "beat 감소 PASS"],
    ["128", "7clk / 35ns", "92clk / 460ns", "189~192clk", "38", "beat 감소, TLAST plateau"],
    ["64 BP", "7clk / 35ns", "92clk / 460ns", "191~192clk", "44", "stall 906clk에도 보존"],
  ], 0.74, 1.5, [1.0, 2.0, 2.0, 2.1, 1.1, 4.0], 0.62, 8.0);
  tx(s, "II 해석: TB의 shot-to-shot T0 간격은 2107clk(10.535us)로, 설계 최소 II가 아니라 시나리오 간격이다. C06 v002는 first beat, TLAST, stall 보존을 검증한다.", 0.78, 5.18, 11.6, 0.55, { size: 11.2 });
  footer(s, "근거: Result v002 section 6");
}

{
  const s = pptx.addSlide();
  title(s, "Status / IRQ 계약", "pipeline fault observability는 STAT5/6/7 polling 기준, o_irq_pipe는 reserved");
  box(s, "STAT5\nbusy / overrun / fatal\nchip / drain / seq mask", 0.9, 1.7, 3.2, 1.1, { fill: C.blueFill, line: C.blue, bold: true });
  box(s, "STAT6\nread timeout / flush / overrun\ncmd collision / drain complete", 5.0, 1.7, 3.2, 1.1, { fill: C.greenFill, line: C.green, bold: true });
  box(s, "STAT7\nreg timeout / stop ID\nrun timeout / quarantine", 9.1, 1.7, 3.2, 1.1, { fill: C.orangeFill, line: C.orange, bold: true });
  box(s, "o_irq_pipe = reserved / tie-off\n정상 run o_irq=0, o_irq_pipe=0", 2.2, 4.05, 3.6, 0.88, { fill: C.purpleFill, line: C.purple, bold: true });
  arrow(s, 5.95, 4.49, 7.5, 4.49);
  box(s, "dedicated IRQ 필요 시\nv003에서 별도 요구사항 정의", 7.6, 4.05, 3.6, 0.88, { fill: C.redFill, line: C.red, bold: true });
  footer(s, "근거: tdc_gpx_csr_pipeline.vhd:345-349, Result v002 section 7~8");
}

{
  const s = pptx.addSlide();
  title(s, "v003 승계", "C06 handoff 전 마지막 확인은 CSR 기반 recovery closure");
  table(s, [
    ["v003 ID", "내용", "권고"],
    ["FP3-C06-01", "soft_reset/force_reinit top recovery", "px_utility_pkg AXI4-Lite helper로 run->recovery->run 검증"],
    ["FP3-C06-02", "project syntax policy", "direct compile/elab를 표준 증거로 둘지 .xpr를 갱신할지 결정"],
    ["FP3-C06-03", "pipeline IRQ policy", "기본은 reserved 유지, 필요 시 dedicated IRQ 요구사항 분리"],
  ], 0.8, 1.62, [1.55, 4.4, 5.8], 0.72, 8.4);
  tx(s, "판정: v002 실행은 완료. v003 recovery만 닫히면 C06 handoff 판단 가능.", 0.85, 4.9, 10.8, 0.42, { size: 13, bold: true });
  footer(s, "다음 산출물: Code_Fix_Result_v003 + C06 handoff 판단");
}

pptx.writeFile({ fileName: out }).then(() => {
  console.log(out);
});
