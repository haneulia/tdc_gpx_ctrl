const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C06_Control_Status_Integration/C06_Control_Status_Integration_260511200655_Code_Fix_Plan_v002.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C06 Code Fix Plan v002";
pptx.title = "C06 Code Fix Plan v002";
pptx.company = "OpenAI";
pptx.lang = "ko-KR";
pptx.theme = {
  headFontFace: "Malgun Gothic",
  bodyFontFace: "Malgun Gothic",
  lang: "ko-KR",
};
pptx.layout = "CUSTOM_WIDE";
pptx.margin = 0;

const font = "Malgun Gothic";
const C = {
  bg: "F8FAFC",
  ink: "111827",
  muted: "64748B",
  line: "CBD5E1",
  blue: "2563EB",
  blueFill: "DBEAFE",
  green: "16A34A",
  greenFill: "DCFCE7",
  orange: "EA580C",
  orangeFill: "FFEDD5",
  red: "DC2626",
  redFill: "FEE2E2",
  white: "FFFFFF",
  slate: "E2E8F0",
  dark: "0F172A",
  purple: "7C3AED",
  purpleFill: "F3E8FF",
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
    breakLine: opt.breakLine || false,
  });
}

function title(slide, main, sub) {
  slide.background = { color: C.bg };
  tx(slide, main, 0.55, 0.28, 12.2, 0.46, { size: 21, bold: true });
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
    align: opt.align || "center",
    color: opt.color || C.ink,
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

function table(slide, rows, x, y, widths, rowH, size = 8.2) {
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
  tx(s, "Code Fix Plan v002", 0.76, 1.72, 5.0, 0.38, { size: 15, color: "CBD5E1" });
  tx(s, "목표: v001 이후 남은 검증 계약을 실제 xsim 회귀로 닫고, 남은 항목은 v003으로 추적 승계한다.", 0.78, 2.5, 9.8, 0.55, { size: 14, color: C.white });
  box(s, "Datasheet 기준\nI-Mode Single", 0.85, 3.62, 2.35, 0.95, { fill: "1E3A8A", line: "60A5FA", color: C.white, bold: true });
  box(s, "T0~T6 marker\nLatency/II 계측", 3.55, 3.62, 2.5, 0.95, { fill: "14532D", line: "86EFAC", color: C.white, bold: true });
  box(s, "32/64/128 width\nfresh xsim", 6.4, 3.62, 2.5, 0.95, { fill: "7C2D12", line: "FDBA74", color: C.white, bold: true });
  box(s, "Result v002\nOpen은 v003 승계", 9.25, 3.62, 2.6, 0.95, { fill: "581C87", line: "C084FC", color: C.white, bold: true });
  footer(s, "C06 Code Fix Plan v002 | 생성 2026-05-11 20:06:55 KST | 수정 2026-05-11 20:33:22 KST");
}

{
  const s = pptx.addSlide();
  title(s, "v002 작업 항목", "P1은 직접 xsim closure, P2는 운영 계약과 status/IRQ 추적성 closure");
  table(s, [
    ["ID", "핵심 질문", "완료 기준"],
    ["FP2-01", "sync FIFO compile-order가 안전한가?", "direct compile/elab PASS 또는 환경 예외 문서화"],
    ["FP2-02", "32/64/128 width가 모두 동작하는가?", "expected beats/tlast PASS"],
    ["FP2-03", "T0~T6로 latency/II를 계측하는가?", "cycle/time marker 로그"],
    ["FP2-04", "downstream tready stall에서 보존되는가?", "bounded stall PASS"],
    ["FP2-05", "fall-only abort가 rise를 죽이지 않는가?", "Scenario F PASS"],
    ["FP2-06", "o_irq/o_irq_pipe 계약이 명확한가?", "no-spurious + reserved 정책"],
    ["FP2-07", "STAT5/6/7 source/clear 추적 가능한가?", "status map 작성"],
    ["FP2-08", "soft_reset/force_reinit recovery가 닫혔는가?", "deadlock 없음 또는 v003 승계"],
  ], 0.65, 1.46, [1.05, 5.35, 5.65], 0.48, 8.0);
  footer(s, "근거: Code_Fix_Plan_v002.md section 5, 8, 9");
}

{
  const s = pptx.addSlide();
  title(s, "실행 흐름", "계획은 Result v002로 닫고, 남은 Open/Partial만 Plan v003으로 승계한다.");
  const xs = [0.75, 2.7, 4.65, 6.6, 8.55, 10.5];
  const labels = ["compile\n입력 정리", "width\n32/64/128", "T0~T6\nmarker", "tready\nstall", "IRQ/status\n계약", "v003\n승계"];
  labels.forEach((label, i) => {
    box(s, label, xs[i], 2.35, 1.45, 0.82, { fill: i < 5 ? C.blueFill : C.orangeFill, line: i < 5 ? C.blue : C.orange, bold: true, size: 10 });
    if (i < labels.length - 1) arrow(s, xs[i] + 1.48, 2.76, xs[i + 1] - 0.05, 2.76, C.muted);
  });
  tx(s, "운영 프로토콜", 0.8, 4.08, 2.5, 0.32, { size: 13, bold: true });
  tx(s, "Fix Plan vN -> Result vN -> Plan vN에 실행 결과 역기록 -> Open/Partial/New 항목을 Fix Plan vN+1로 승계", 0.82, 4.55, 10.9, 0.5, { size: 12 });
  footer(s, "근거: cluster_analysis operating protocol + Plan v002 section 2");
}

{
  const s = pptx.addSlide();
  title(s, "Datasheet 기준", "C06는 payload 의미를 바꾸지 않고 I-Mode single의 control/status 관측을 검증한다.");
  table(s, [
    ["근거", "위치", "v002 적용"],
    ["I-Mode single", "PDF p25 section 2.3", "T0 start와 stop/drain/output marker만 관측"],
    ["Single measurement", "PDF p29 section 2.11.1", "IrFlag, EF/IFIFO read, reset 흐름을 marker로 분해"],
    ["IFIFO data structure", "PDF p27 section 2.4", "payload 의미는 C02/C03/C04 계약 유지"],
    ["empty FIFO read 금지", "PDF p27", "backpressure에서도 beat 보존과 status 추적 분리"],
  ], 0.78, 1.7, [2.5, 2.8, 6.5], 0.62, 8.3);
  footer(s, "절대 기준: Doc/TDC-GPX-Datasheet.pdf");
}

{
  const s = pptx.addSlide();
  title(s, "v002 실행 후 판정", "대부분 닫혔고, recovery closure만 v003으로 넘긴다.");
  table(s, [
    ["범주", "판정", "다음"],
    ["compile/direct xsim", "Verified", "표준 증거로 유지"],
    ["32/64/128 width", "Verified", "추가 수정 없음"],
    ["T0~T6/Backpressure", "Verified", "Result v002에 수치 기록"],
    ["rise/fall imbalance", "Verified", "Scenario F 유지"],
    ["IRQ/status", "Verified + Accepted Exception", "reserved 유지 여부 v003에서 명시"],
    ["soft_reset/force_reinit", "Partial / Open", "FP3-C06-01로 승계"],
  ], 0.8, 1.55, [2.8, 3.4, 5.2], 0.64, 8.4);
  footer(s, "근거: Plan v002 section 9, Result v002");
}

pptx.writeFile({ fileName: out }).then(() => {
  console.log(out);
});
