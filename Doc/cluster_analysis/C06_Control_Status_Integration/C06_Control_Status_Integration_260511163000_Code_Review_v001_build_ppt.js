const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C06_Control_Status_Integration/C06_Control_Status_Integration_260511163000_Code_Review_v001.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C06 Control Status Integration Code Review v001";
pptx.title = "C06 Control Status Integration Code Review v001";
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
  blue: "2563EB",
  blueFill: "EFF6FF",
  green: "16A34A",
  greenFill: "ECFDF5",
  red: "DC2626",
  redFill: "FEF2F2",
  orange: "EA580C",
  orangeFill: "FFF7ED",
  cyan: "0891B2",
  cyanFill: "ECFEFF",
  purple: "7C3AED",
  purpleFill: "F5F3FF",
  slate: "E2E8F0",
  dark: "0F172A",
};

function bg(slide) {
  slide.background = { color: C.bg };
}

function text(slide, value, x, y, w, h, opt = {}) {
  slide.addText(value, {
    x, y, w, h,
    fontFace: font,
    fontSize: opt.size || 11,
    bold: opt.bold || false,
    color: opt.color || C.ink,
    align: opt.align || "left",
    valign: opt.valign || "mid",
    fit: "shrink",
    margin: opt.margin === undefined ? 0.04 : opt.margin,
    breakLine: opt.breakLine || false,
  });
}

function title(slide, main, sub) {
  bg(slide);
  text(slide, main, 0.55, 0.24, 12.25, 0.48, { size: 21, bold: true });
  text(slide, sub, 0.58, 0.74, 12.15, 0.34, { size: 9.4, color: C.muted });
  slide.addShape(pptx.ShapeType.line, {
    x: 0.55, y: 1.1, w: 12.2, h: 0,
    line: { color: C.line, width: 0.8 },
  });
}

function footer(slide, value) {
  text(slide, value, 0.65, 6.98, 12.0, 0.25, { size: 7.8, color: C.muted, align: "center" });
}

function box(slide, value, x, y, w, h, opt = {}) {
  slide.addShape(pptx.ShapeType.roundRect, {
    x, y, w, h,
    rectRadius: 0.06,
    fill: { color: opt.fill || C.white },
    line: { color: opt.line || C.line, width: opt.lineWidth || 1.0 },
  });
  text(slide, value, x + 0.07, y + 0.06, w - 0.14, h - 0.12, {
    size: opt.size || 11,
    bold: opt.bold || false,
    color: opt.color || C.ink,
    align: opt.align || "center",
    valign: opt.valign || "mid",
  });
}

function pill(slide, value, x, y, w, color, fill) {
  slide.addShape(pptx.ShapeType.roundRect, {
    x, y, w, h: 0.34,
    rectRadius: 0.16,
    fill: { color: fill },
    line: { color, width: 0.8 },
  });
  text(slide, value, x + 0.08, y + 0.05, w - 0.16, 0.22, { size: 8.7, bold: true, color, align: "center" });
}

function arrow(slide, x1, y1, x2, y2, color = C.muted, width = 1.4) {
  slide.addShape(pptx.ShapeType.line, {
    x: x1, y: y1, w: x2 - x1, h: y2 - y1,
    line: { color, width, endArrowType: "triangle" },
  });
}

function line(slide, x1, y1, x2, y2, color = C.line, width = 1) {
  slide.addShape(pptx.ShapeType.line, {
    x: x1, y: y1, w: x2 - x1, h: y2 - y1,
    line: { color, width },
  });
}

function table(slide, rows, x, y, widths, rowH, size = 8.2) {
  rows.forEach((row, r) => {
    let curX = x;
    row.forEach((cell, c) => {
      const head = r === 0;
      slide.addShape(pptx.ShapeType.rect, {
        x: curX, y: y + r * rowH, w: widths[c], h: rowH,
        fill: { color: head ? C.slate : C.white },
        line: { color: C.line, width: 0.5 },
      });
      text(slide, cell, curX + 0.04, y + r * rowH + 0.02, widths[c] - 0.08, rowH - 0.04, {
        size,
        bold: head,
        align: c === 0 ? "center" : "left",
      });
      curX += widths[c];
    });
  });
}

{
  const s = pptx.addSlide();
  title(s, "C06 Code Review v001", "Control/Status/IRQ 통합은 구현이 존재하지만, end-to-end II와 SW 계약은 아직 닫히지 않았다.");

  box(s, "C06 목적\nC01~C04 data path를 top-level 운용 계약으로 묶기", 0.75, 1.45, 3.1, 1.05, { fill: C.blueFill, line: C.blue, bold: true });
  box(s, "현재 판정\n다음 Cluster 진입 전 C06 검증 필요", 4.05, 1.45, 3.1, 1.05, { fill: C.orangeFill, line: C.orange, bold: true });
  box(s, "핵심 위험\nready stall, sticky clear, IRQ 의미, start 경계", 7.35, 1.45, 4.95, 1.05, { fill: C.redFill, line: C.red, bold: true });

  text(s, "코드 상태 요약", 0.85, 2.95, 2.2, 0.35, { size: 15, bold: true });
  pill(s, "양호: packet_start register", 0.85, 3.45, 2.45, C.green, C.greenFill);
  pill(s, "양호: frame_done_both register", 3.5, 3.45, 2.65, C.green, C.greenFill);
  pill(s, "양호: face_closing register", 6.35, 3.45, 2.45, C.green, C.greenFill);
  pill(s, "Open: C06 II/backpressure", 9.0, 3.45, 2.7, C.orange, C.orangeFill);

  text(s, "리뷰 기준", 0.85, 4.35, 1.8, 0.3, { size: 14, bold: true });
  box(s, "Datasheet page 25/27/29\nI-Mode single, IFIFO read, IrFlag 이후 read sequence", 0.85, 4.78, 3.6, 0.78, { fill: C.white, line: C.line, size: 9.2 });
  box(s, "프로젝트 규칙\n모듈 경계 register, 조합 최대 2-depth, status 추적성", 4.65, 4.78, 3.6, 0.78, { fill: C.white, line: C.line, size: 9.2 });
  box(s, "C06 Plan/Analysis\nVB-C06-01..10, T0..T6, latency/throughput/II", 8.45, 4.78, 3.6, 0.78, { fill: C.white, line: C.line, size: 9.2 });

  footer(s, "C06_Control_Status_Integration_260511163000_Code_Review_v001");
}

{
  const s = pptx.addSlide();
  title(s, "Findings 지도", "결함, 검증 공백, 문서 계약 미완료를 분리해 다음 수정 순서를 결정한다.");
  table(s, [
    ["ID", "Priority", "핵심 판단", "다음 조치"],
    ["CR-01", "P1", "End-to-end II와 ready stall이 아직 검증되지 않음", "VB-C06-01/04/10 우선 실행"],
    ["CR-02", "P2", "shot/face start 출력 경계에 조합 qualifier 존재", "baseline 후 register화 검토"],
    ["CR-03", "P2", "status_agg busy/overrun이 넓은 조합 OR", "status register boundary 추가"],
    ["CR-04", "P2", "sticky clear 정책이 field별로 다름", "soft_clear 정책 통일 또는 예외화"],
    ["CR-05", "P2", "o_irq와 o_irq_pipe 의미가 분리되지 않음", "IRQ source/clear 계약 작성"],
    ["CR-06", "P3", "status source/clear가 여러 module에 분산", "status map 작성"],
    ["CR-07", "P3", "fall-only abort는 의도 같지만 검증 필요", "VB-C06-03 수행"],
  ], 0.55, 1.35, [0.82, 1.05, 6.15, 4.55], 0.52, 8.1);

  box(s, "리뷰 결론\nC06은 수정 전 baseline 측정이 먼저 필요하다. 특히 start gating register화는 T2를 +1 clock 이동시킬 수 있어 변경 전/후 비교 근거가 필요하다.", 0.85, 5.95, 11.65, 0.78, { fill: C.orangeFill, line: C.orange, size: 10.2, bold: true });
  footer(s, "Findings: F-C06-CR-01..07");
}

{
  const s = pptx.addSlide();
  title(s, "T0~T6 Timing 관점", "C06 완료 판정은 output tlast가 아니라 다음 start 수락 가능 시점까지 봐야 한다.");

  const y = 2.25;
  const xs = [0.75, 2.25, 3.9, 5.55, 7.2, 8.85, 10.45];
  const labels = [
    "T0\nstart_tdc",
    "T1\npacket_start",
    "T2\nchip start",
    "T3\nfirst beat",
    "T4\ntlast",
    "T5\nframe_done",
    "T6\nnext start",
  ];
  for (let i = 0; i < xs.length; i++) {
    box(s, labels[i], xs[i], y, 1.15, 0.72, { fill: i < 3 ? C.blueFill : i < 5 ? C.purpleFill : C.greenFill, line: i < 3 ? C.blue : i < 5 ? C.purple : C.green, bold: true, size: 9.4 });
    if (i < xs.length - 1) arrow(s, xs[i] + 1.17, y + 0.36, xs[i + 1] - 0.08, y + 0.36, C.muted);
  }

  line(s, 0.78, 3.45, 12.05, 3.45, C.line, 1.1);
  text(s, "control", 0.8, 3.55, 1.0, 0.25, { size: 8.3, color: C.blue, bold: true });
  text(s, "GPX read / C02 / C03", 3.55, 3.55, 2.3, 0.25, { size: 8.3, color: C.purple, bold: true });
  text(s, "C04 final AXIS + ready", 6.8, 3.55, 2.7, 0.25, { size: 8.3, color: C.orange, bold: true });
  text(s, "status / rearm", 9.95, 3.55, 1.7, 0.25, { size: 8.3, color: C.green, bold: true });

  box(s, "운용 budget\n13.888889 us - 8 us reserve - 왕복거리시간", 0.9, 4.35, 3.65, 0.82, { fill: C.white, line: C.line, size: 10.1 });
  box(s, "PASS 조건\nT0→T6 measured latency + ready stall penalty ≤ usable processing time", 4.85, 4.35, 4.2, 0.82, { fill: C.orangeFill, line: C.orange, size: 10.1, bold: true });
  box(s, "검증 필요\n32/64/128 width, max_hits_cfg, 거리, tready pattern 동시 sweep", 9.35, 4.35, 3.15, 0.82, { fill: C.redFill, line: C.red, size: 9.2 });

  footer(s, "Timing / Latency / Throughput / Pipeline / II");
}

{
  const s = pptx.addSlide();
  title(s, "Start/Face 제어 경계", "packet_start는 register로 닫혔지만, final start qualifier는 아직 조합 경계다.");

  box(s, "raw shot edge\ns_shot_raw_pulse", 0.8, 2.0, 1.75, 0.68, { fill: C.white, line: C.line, size: 9.5 });
  arrow(s, 2.55, 2.34, 3.05, 2.34, C.blue);
  box(s, "multi-condition gate\ns_packet_start_comb", 3.05, 1.85, 2.05, 0.98, { fill: C.orangeFill, line: C.orange, size: 9.0, bold: true });
  arrow(s, 5.1, 2.34, 5.65, 2.34, C.blue);
  box(s, "register\ns_packet_start_r", 5.65, 1.95, 1.75, 0.78, { fill: C.greenFill, line: C.green, size: 9.5, bold: true });
  arrow(s, 7.4, 2.34, 7.95, 2.34, C.blue);
  box(s, "pending/closing/abort qualifier\ns_shot_start_gated", 7.95, 1.77, 2.25, 1.14, { fill: C.redFill, line: C.red, size: 8.6, bold: true });
  arrow(s, 10.2, 2.34, 10.75, 2.34, C.red);
  box(s, "module outputs\no_shot_start_*", 10.75, 1.95, 1.75, 0.78, { fill: C.redFill, line: C.red, size: 9.2, bold: true });

  box(s, "권장 판단\nshot_start_gated / face_start_gated / per-chip start를 register boundary로 닫는다.", 1.0, 4.15, 3.95, 0.84, { fill: C.blueFill, line: C.blue, size: 10.0, bold: true });
  box(s, "주의 영향\nstart가 1 clock 늦어질 수 있으므로 baseline T0~T6와 수정 후 T0~T6를 비교해야 한다.", 5.15, 4.15, 3.95, 0.84, { fill: C.orangeFill, line: C.orange, size: 10.0, bold: true });
  box(s, "검증 연결\nVB-C06-01 normal, VB-C06-02 drain 중 next start, VB-C06-03 rise/fall imbalance", 9.3, 4.15, 3.0, 0.84, { fill: C.greenFill, line: C.green, size: 8.8 });

  footer(s, "Code refs: tdc_gpx_face_seq.vhd:515-535, 674-693");
}

{
  const s = pptx.addSlide();
  title(s, "Status / IRQ 계약", "SW가 읽는 status와 interrupt source를 같은 의미로 착각하지 않게 분리해야 한다.");

  box(s, "status_agg\nbusy/overrun/error count", 0.85, 1.65, 2.25, 0.88, { fill: C.blueFill, line: C.blue, bold: true, size: 9.5 });
  arrow(s, 3.1, 2.09, 3.75, 2.09, C.blue);
  box(s, "tdc_gpx_top\nextra sticky fields", 3.75, 1.65, 2.25, 0.88, { fill: C.purpleFill, line: C.purple, bold: true, size: 9.5 });
  arrow(s, 6.0, 2.09, 6.65, 2.09, C.purple);
  box(s, "csr_pipeline\nSTAT5/6/7 packing", 6.65, 1.65, 2.25, 0.88, { fill: C.greenFill, line: C.green, bold: true, size: 9.5 });
  arrow(s, 8.9, 2.09, 9.55, 2.09, C.green);
  box(s, "SW visible\nAXI4-Lite status", 9.55, 1.65, 2.25, 0.88, { fill: C.white, line: C.line, bold: true, size: 9.5 });

  box(s, "csr_pipeline IRQ\nintrpt_src_in = 0", 1.15, 3.45, 2.55, 0.78, { fill: C.redFill, line: C.red, bold: true, size: 9.5 });
  arrow(s, 3.7, 3.84, 4.6, 3.84, C.red);
  box(s, "o_irq_pipe\n현재 사실상 reserved 후보", 4.6, 3.45, 2.45, 0.78, { fill: C.redFill, line: C.red, bold: true, size: 9.2 });

  box(s, "config_ctrl IRQ\ncontrol/config CSR source", 7.15, 3.45, 2.55, 0.78, { fill: C.orangeFill, line: C.orange, bold: true, size: 9.3 });
  arrow(s, 9.7, 3.84, 10.45, 3.84, C.orange);
  box(s, "o_irq\nmain IRQ", 10.45, 3.45, 1.6, 0.78, { fill: C.orangeFill, line: C.orange, bold: true, size: 9.5 });

  box(s, "결정 필요\n1) o_irq_pipe reserved/tied-off로 문서화\n2) 또는 pipeline fault/done sticky를 OR한 registered IRQ로 재정의", 1.0, 5.25, 11.25, 0.72, { fill: C.white, line: C.line, size: 10.0, bold: true });
  footer(s, "Code refs: tdc_gpx_top.vhd:169-170, 465, 630 / csr_pipeline.vhd:345-346");
}

{
  const s = pptx.addSlide();
  title(s, "권장 진행 순서", "바로 수정하기보다 baseline 측정 후 낮은 위험 수정부터 닫는 순서가 안전하다.");

  const phases = [
    ["1", "Baseline 측정", "T0~T6, ready stall, sticky/IRQ 현재값"],
    ["2", "Status/IRQ 계약", "STAT5/6/7, o_irq, o_irq_pipe, clear source"],
    ["3", "낮은 위험 수정", "status_agg busy/overrun register화"],
    ["4", "Start 경계 수정", "shot/face start register화 여부 결정"],
    ["5", "Sticky clear 정리", "soft_clear 통일 또는 reset-only 예외 문서화"],
  ];
  phases.forEach((p, i) => {
    const x = 0.85 + i * 2.45;
    const y = 1.85 + (i % 2) * 0.42;
    box(s, p[0], x, y, 0.48, 0.48, { fill: C.dark, line: C.dark, color: C.white, bold: true, size: 12 });
    text(s, p[1], x + 0.62, y - 0.02, 1.55, 0.25, { size: 10.5, bold: true });
    text(s, p[2], x + 0.62, y + 0.26, 1.55, 0.52, { size: 7.7, color: C.muted, valign: "top" });
    if (i < phases.length - 1) arrow(s, x + 2.07, y + 0.24, x + 2.42, 1.85 + ((i + 1) % 2) * 0.42 + 0.24, C.muted, 1.2);
  });

  table(s, [
    ["Exit 기준", "필요 근거"],
    ["C06 수정 착수 가능", "Baseline v001에서 정상/ready-stall/sticky/IRQ 관측 완료"],
    ["C06 완료 가능", "수정 후 VB-C06-01/03/04/05/09/10 PASS"],
    ["다음 Cluster 진입 가능", "T0→T6 II와 polygon budget이 32/64/128 폭별로 닫힘"],
  ], 1.1, 4.25, [3.0, 8.2], 0.48, 9.0);

  footer(s, "Recommended next artifact: C06_Verify_Baseline_v001");
}

pptx.writeFile({ fileName: out });

