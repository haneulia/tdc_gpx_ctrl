const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C06_Control_Status_Integration/C06_Control_Status_Integration_260511192515_Code_Fix_Plan_v001.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C06 Code Fix Plan v001";
pptx.title = "C06 Code Fix Plan v001";
pptx.lang = "ko-KR";
pptx.theme = { headFontFace: "Malgun Gothic", bodyFontFace: "Malgun Gothic", lang: "ko-KR" };

const font = "Malgun Gothic";
const C = {
  bg: "F8FAFC", ink: "111827", muted: "64748B", line: "CBD5E1", white: "FFFFFF",
  blue: "2563EB", blueFill: "EFF6FF", green: "16A34A", greenFill: "ECFDF5",
  red: "DC2626", redFill: "FEF2F2", orange: "EA580C", orangeFill: "FFF7ED",
  cyan: "0891B2", cyanFill: "ECFEFF", purple: "7C3AED", purpleFill: "F5F3FF",
  slate: "E2E8F0", dark: "0F172A"
};

function bg(slide) { slide.background = { color: C.bg }; }
function text(slide, value, x, y, w, h, opt = {}) {
  slide.addText(value, {
    x, y, w, h, fontFace: font, fontSize: opt.size || 11, bold: opt.bold || false,
    color: opt.color || C.ink, align: opt.align || "left", valign: opt.valign || "mid",
    fit: "shrink", margin: opt.margin === undefined ? 0.04 : opt.margin
  });
}
function title(slide, main, sub) {
  bg(slide);
  text(slide, main, 0.55, 0.24, 12.2, 0.48, { size: 21, bold: true });
  text(slide, sub, 0.58, 0.74, 12.15, 0.34, { size: 9.4, color: C.muted });
  slide.addShape(pptx.ShapeType.line, { x: 0.55, y: 1.1, w: 12.2, h: 0, line: { color: C.line, width: 0.8 } });
}
function footer(slide, value) {
  text(slide, value, 0.65, 6.98, 12, 0.24, { size: 7.8, color: C.muted, align: "center" });
}
function box(slide, value, x, y, w, h, opt = {}) {
  slide.addShape(pptx.ShapeType.roundRect, {
    x, y, w, h, rectRadius: 0.06,
    fill: { color: opt.fill || C.white },
    line: { color: opt.line || C.line, width: opt.lineWidth || 1 }
  });
  text(slide, value, x + 0.07, y + 0.05, w - 0.14, h - 0.1, {
    size: opt.size || 11, bold: opt.bold || false, color: opt.color || C.ink,
    align: opt.align || "center", valign: opt.valign || "mid"
  });
}
function arrow(slide, x1, y1, x2, y2, color = C.muted, width = 1.4) {
  slide.addShape(pptx.ShapeType.line, { x: x1, y: y1, w: x2 - x1, h: y2 - y1, line: { color, width, endArrowType: "triangle" } });
}
function table(slide, rows, x, y, widths, rowH, size = 8.2) {
  rows.forEach((row, r) => {
    let curX = x;
    row.forEach((cell, c) => {
      const head = r === 0;
      slide.addShape(pptx.ShapeType.rect, {
        x: curX, y: y + r * rowH, w: widths[c], h: rowH,
        fill: { color: head ? C.slate : C.white },
        line: { color: C.line, width: 0.5 }
      });
      text(slide, cell, curX + 0.04, y + r * rowH + 0.02, widths[c] - 0.08, rowH - 0.04, {
        size, bold: head, align: c === 0 ? "center" : "left"
      });
      curX += widths[c];
    });
  });
}

{
  const s = pptx.addSlide();
  title(s, "C06 Code Fix Plan", "Baseline PASS 이후 남은 Open 항목을 작은 RTL 수정과 필수 검증으로 닫는다.");
  box(s, "수정 가능 판정\n정상 data/output baseline 확보", 0.75, 1.5, 2.7, 0.88, { fill: C.greenFill, line: C.green, bold: true });
  box(s, "수정 범위\ncontrol/status/IRQ/backpressure 계약", 3.75, 1.5, 3.0, 0.88, { fill: C.blueFill, line: C.blue, bold: true });
  box(s, "범위 제외\ndata payload 재설계, pipeline IRQ 신규 기능", 7.05, 1.5, 4.3, 0.88, { fill: C.orangeFill, line: C.orange, bold: true });

  text(s, "이번 계획의 기본 결정", 0.85, 3.05, 3.4, 0.32, { size: 15, bold: true });
  table(s, [
    ["항목", "결정"],
    ["status_agg busy/overrun", "register boundary 추가"],
    ["face_seq start outputs", "register boundary 추가"],
    ["fault sticky clear", "soft_clear 대상 통일"],
    ["o_irq_pipe", "reserved/tied-off 유지"],
    ["quarantine_escape_mask", "reset-only 예외 유지"],
  ], 0.85, 3.55, [3.1, 7.4], 0.44, 8.6);
  footer(s, "C06_Control_Status_Integration_260511192515_Code_Fix_Plan_v001");
}

{
  const s = pptx.addSlide();
  title(s, "Phase 구조", "수정은 timing boundary부터 status recovery, IRQ 계약, 검증 순서로 진행한다.");
  const phases = [
    ["A", "status_agg", "busy/overrun register"],
    ["B", "face_seq", "start output register"],
    ["C", "top sticky", "soft_clear policy"],
    ["D", "IRQ", "o_irq_pipe reserved"],
    ["E", "TB", "T0~T6 / stall / sticky"],
  ];
  phases.forEach((p, i) => {
    const x = 0.75 + i * 2.45;
    const fill = [C.blueFill, C.purpleFill, C.greenFill, C.orangeFill, C.cyanFill][i];
    const line = [C.blue, C.purple, C.green, C.orange, C.cyan][i];
    box(s, `${p[0]}\n${p[1]}\n${p[2]}`, x, 1.8, 1.75, 1.0, { fill, line, bold: true, size: 9 });
    if (i < phases.length - 1) arrow(s, x + 1.78, 2.3, x + 2.35, 2.3, C.muted);
  });
  table(s, [
    ["Phase", "수정 파일", "검증"],
    ["A", "tdc_gpx_status_agg.vhd", "STAT5 busy/overrun latency 1clk"],
    ["B", "tdc_gpx_face_seq.vhd", "face_seq A~D, top width sweep"],
    ["C", "tdc_gpx_top.vhd", "err_soft_clear 전/후 status readback"],
    ["D", "tdc_gpx_top/csr_pipeline", "o_irq_pipe=0 reserved 계약"],
    ["E", "TB files", "VB-C06-01/03/04/05/09/10"],
  ], 0.9, 3.55, [1.3, 3.45, 6.3], 0.43, 8.2);
  footer(s, "Phase A~E");
}

{
  const s = pptx.addSlide();
  title(s, "Timing 영향", "수정은 대부분 data throughput을 바꾸지 않고, boundary latency만 명확히 한다.");
  table(s, [
    ["수정", "Latency", "Throughput", "II"],
    ["status register", "status +1clk", "data 영향 없음", "직접 영향 없음"],
    ["face_seq start register", "T2 +1clk 가능", "beat rate 영향 없음", "최소 II +1clk 가능"],
    ["sticky clear", "clear 관측 +0~1clk", "data 영향 없음", "recovery 판단 안정"],
    ["o_irq_pipe reserved", "IRQ 기능 추가 없음", "data 영향 없음", "직접 영향 없음"],
    ["tready stall 검증", "stall만큼 T3~T6 증가", "bounded stall만큼 감소", "T6 기준 증가"],
  ], 0.65, 1.35, [2.9, 2.55, 3.25, 3.1], 0.56, 8.2);

  box(s, "핵심\nregister boundary는 timing closure를 좋게 만들지만, T0~T6 marker와 기존 baseline을 기준으로 +1 clock 영향까지 문서화해야 한다.", 0.9, 5.65, 11.2, 0.65, { fill: C.orangeFill, line: C.orange, bold: true, size: 10 });
  footer(s, "Latency / Throughput / Pipeline / II");
}

{
  const s = pptx.addSlide();
  title(s, "IRQ 계약", "이번 C06에서는 pipeline IRQ를 새로 만들지 않고 reserved/tied-off로 명확히 한다.");
  box(s, "config_ctrl IRQ\no_irq\n기존 유지", 1.0, 1.75, 2.5, 0.9, { fill: C.blueFill, line: C.blue, bold: true });
  arrow(s, 3.55, 2.2, 4.35, 2.2, C.blue);
  box(s, "SW config/control\ninterrupt path", 4.35, 1.75, 2.6, 0.9, { fill: C.blueFill, line: C.blue, bold: true });

  box(s, "csr_pipeline IRQ\no_irq_pipe\nsource=0", 1.0, 3.55, 2.5, 0.9, { fill: C.redFill, line: C.red, bold: true });
  arrow(s, 3.55, 4.0, 4.35, 4.0, C.red);
  box(s, "reserved/tied-off\n이번 세대 계약", 4.35, 3.55, 2.6, 0.9, { fill: C.redFill, line: C.red, bold: true });

  box(s, "이유\nSTAT5/6/7 sticky OR를 IRQ로 만들려면 mask/clear/pulse-level 정책이 새로 필요하다. C06 안정화 범위를 넘으므로 다음 generation 후보로 남긴다.", 7.45, 2.1, 4.25, 1.65, { fill: C.white, line: C.line, size: 10 });
  footer(s, "IRQ source/clear contract");
}

{
  const s = pptx.addSlide();
  title(s, "검증 Matrix", "수정 후 C06 완료 판정에 필요한 항목만 필수로 묶는다.");
  table(s, [
    ["ID", "상태 목표", "검증 내용"],
    ["VB-C06-01", "Verified", "I-Mode single T0~T6 marker"],
    ["VB-C06-02", "Verified", "drain 중 next start defer/drop"],
    ["VB-C06-03", "Verified", "rise/fall imbalance"],
    ["VB-C06-04", "Verified", "output tready stall"],
    ["VB-C06-05", "Verified", "sticky clear/readback"],
    ["VB-C06-06", "Verified", "max_hits_cfg early/late snapshot"],
    ["VB-C06-07", "Verified", "32/64/128 width sweep"],
    ["VB-C06-08", "Verified", "reset/soft_reset/force_reinit"],
    ["VB-C06-09", "Verified", "o_irq_pipe reserved"],
    ["VB-C06-10", "Verified", "polygon budget + stall penalty"],
  ], 0.55, 1.25, [1.55, 1.85, 8.9], 0.43, 7.6);
  footer(s, "Fresh xsim required after code changes");
}

{
  const s = pptx.addSlide();
  title(s, "다음 실행", "계획 승인 후 바로 Phase A부터 코드 수정에 들어간다.");
  box(s, "1\nPhase A\nstatus_agg register화", 0.85, 1.7, 2.15, 0.95, { fill: C.blueFill, line: C.blue, bold: true });
  arrow(s, 3.05, 2.18, 3.75, 2.18, C.muted);
  box(s, "2\nPhase B\nface_seq start FF", 3.8, 1.7, 2.15, 0.95, { fill: C.purpleFill, line: C.purple, bold: true });
  arrow(s, 6.0, 2.18, 6.7, 2.18, C.muted);
  box(s, "3\nPhase C/D\nsticky + IRQ contract", 6.75, 1.7, 2.15, 0.95, { fill: C.greenFill, line: C.green, bold: true });
  arrow(s, 8.95, 2.18, 9.65, 2.18, C.muted);
  box(s, "4\nPhase E\nfresh xsim closure", 9.7, 1.7, 2.15, 0.95, { fill: C.orangeFill, line: C.orange, bold: true });

  box(s, "작업 원칙\n합성 가능한 VHDL, module boundary register, 조합 2-depth 규칙, AXI4-Lite TB helper는 px_utility_pkg 사용", 1.1, 4.05, 10.6, 0.8, { fill: C.white, line: C.line, bold: true, size: 10 });
  box(s, "다음 산출물\nC06_Code_Fix_v001 또는 C06_Verify_v001", 1.1, 5.25, 10.6, 0.62, { fill: C.cyanFill, line: C.cyan, bold: true, size: 10 });
  footer(s, "Next: code modification");
}

pptx.writeFile({ fileName: out });

