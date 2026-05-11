const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C06_Control_Status_Integration/C06_Control_Status_Integration_260511195318_Code_Fix_Result_v001.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C06 Code Fix Result v001";
pptx.title = "C06 Code Fix Result v001";
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
function tx(slide, value, x, y, w, h, opt = {}) {
  slide.addText(value, {
    x, y, w, h,
    fontFace: font,
    fontSize: opt.size || 10,
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
  tx(slide, main, 0.58, 0.26, 12.2, 0.48, { size: 20.5, bold: true });
  tx(slide, sub, 0.6, 0.77, 12.0, 0.34, { size: 9.4, color: C.muted });
  slide.addShape(pptx.ShapeType.line, {
    x: 0.58, y: 1.12, w: 12.2, h: 0,
    line: { color: C.line, width: 0.8 },
  });
}
function footer(slide, value) {
  tx(slide, value, 0.7, 7.0, 11.9, 0.24, { size: 7.6, color: C.muted, align: "center" });
}
function box(slide, value, x, y, w, h, opt = {}) {
  slide.addShape(pptx.ShapeType.roundRect, {
    x, y, w, h,
    rectRadius: 0.06,
    fill: { color: opt.fill || C.white },
    line: { color: opt.line || C.line, width: opt.lineWidth || 1 },
  });
  tx(slide, value, x + 0.08, y + 0.05, w - 0.16, h - 0.1, {
    size: opt.size || 10,
    bold: opt.bold || false,
    color: opt.color || C.ink,
    align: opt.align || "center",
    valign: opt.valign || "mid",
  });
}
function arrow(slide, x1, y1, x2, y2, color = C.muted) {
  slide.addShape(pptx.ShapeType.line, {
    x: x1, y: y1, w: x2 - x1, h: y2 - y1,
    line: { color, width: 1.4, endArrowType: "triangle" },
  });
}
function table(slide, rows, x, y, widths, rowH, size = 8.0) {
  rows.forEach((row, r) => {
    let curX = x;
    row.forEach((cell, c) => {
      const head = r === 0;
      slide.addShape(pptx.ShapeType.rect, {
        x: curX, y: y + r * rowH, w: widths[c], h: rowH,
        fill: { color: head ? C.slate : C.white },
        line: { color: C.line, width: 0.5 },
      });
      tx(slide, cell, curX + 0.04, y + r * rowH + 0.02, widths[c] - 0.08, rowH - 0.04, {
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
  bg(s);
  tx(s, "C06", 0.8, 0.7, 1.4, 0.45, { size: 15, bold: true, color: C.blue });
  tx(s, "Control / Status Integration", 0.8, 1.15, 8.8, 0.58, { size: 27, bold: true });
  tx(s, "Code Fix Result v001", 0.82, 1.78, 5.0, 0.42, { size: 15, color: C.muted });
  tx(s, "핵심 판정", 0.92, 3.05, 1.8, 0.3, { size: 13, bold: true });
  box(s, "RTL 보완\nPhase A~D 반영", 0.92, 3.48, 2.35, 0.9, { fill: C.blueFill, line: C.blue, bold: true });
  box(s, "xsim 검증\nfocused + top64 PASS", 3.62, 3.48, 2.75, 0.9, { fill: C.greenFill, line: C.green, bold: true });
  box(s, "잔여\nproject compile-order warning", 6.72, 3.48, 3.55, 0.9, { fill: C.orangeFill, line: C.orange, bold: true });
  tx(s, "Datasheet 기준은 I-Mode single 흐름과 IFIFO data 구조를 유지하는 것이다. 이번 수정은 bus timing/payload 의미를 바꾸지 않고 control/status 경계를 닫는다.", 0.92, 5.2, 11.4, 0.5, { size: 11.3 });
  footer(s, "C06_Control_Status_Integration_260511195318_Code_Fix_Result_v001");
}

{
  const s = pptx.addSlide();
  title(s, "Phase별 반영 구조", "조합 boundary를 줄이고 status recovery 계약을 명시했다.");
  const phases = [
    ["A", "status_agg\nbusy/overrun FF", C.blueFill, C.blue],
    ["B", "face_seq\nstart output FF", C.purpleFill, C.purple],
    ["C", "top sticky\nsoft_clear", C.greenFill, C.green],
    ["D", "IRQ\nreserved 계약", C.orangeFill, C.orange],
    ["E", "TB/xsim\n증거 기록", C.cyanFill, C.cyan],
  ];
  phases.forEach((p, i) => {
    const x = 0.72 + i * 2.45;
    box(s, `${p[0]}\n${p[1]}`, x, 1.68, 1.78, 1.05, { fill: p[2], line: p[3], bold: true, size: 9.2 });
    if (i < phases.length - 1) arrow(s, x + 1.82, 2.2, x + 2.35, 2.2, C.muted);
  });
  table(s, [
    ["Phase", "파일", "결과"],
    ["A", "tdc_gpx_status_agg.vhd", "wide fan-in status를 register boundary로 닫음"],
    ["B", "tdc_gpx_face_seq.vhd", "start_comb와 output_start_r 분리"],
    ["C", "tdc_gpx_top.vhd", "frame/row/run timeout sticky를 soft_clear 대상화"],
    ["D", "tdc_gpx_top / csr_pipeline", "o_irq_pipe는 reserved/tied-off로 명시"],
    ["E", "TB + TCL probe", "status, face_seq, mask sweep, top64, sticky probe PASS"],
  ], 0.85, 3.45, [1.1, 3.35, 6.85], 0.43, 8.2);
  footer(s, "근거: RTL line trace 및 xsim_c06_fix_*_260511194330.log");
}

{
  const s = pptx.addSlide();
  title(s, "Timing: face_seq start boundary", "외부 start marker는 +1 clock 이동하지만 pulse width와 nominal II는 유지된다.");
  const y = 2.0;
  box(s, "raw edge", 0.85, y, 1.4, 0.64, { fill: C.white, line: C.line, bold: true });
  arrow(s, 2.28, y + 0.32, 3.2, y + 0.32);
  box(s, "packet_start_r\nshot_pending_r", 3.2, y - 0.05, 2.0, 0.74, { fill: C.blueFill, line: C.blue, bold: true, size: 9.2 });
  arrow(s, 5.25, y + 0.32, 6.05, y + 0.32);
  box(s, "start_comb\ninternal decision", 6.05, y - 0.05, 2.0, 0.74, { fill: C.purpleFill, line: C.purple, bold: true, size: 9.2 });
  arrow(s, 8.1, y + 0.32, 8.9, y + 0.32);
  box(s, "start_output_reg\n+1 clk", 8.9, y - 0.05, 2.0, 0.74, { fill: C.greenFill, line: C.green, bold: true, size: 9.2 });
  arrow(s, 10.95, y + 0.32, 11.75, y + 0.32);
  box(s, "downstream\nchip/C04", 11.1, y + 0.9, 1.6, 0.58, { fill: C.white, line: C.line, size: 8.5 });
  slideLine(s, 0.95, 4.05, 12.1, 4.05);
  tx(s, "T0", 1.0, 4.12, 0.5, 0.25, { size: 8, bold: true, color: C.muted });
  tx(s, "T1", 3.45, 4.12, 0.5, 0.25, { size: 8, bold: true, color: C.muted });
  tx(s, "T2 comb", 6.25, 4.12, 0.8, 0.25, { size: 8, bold: true, color: C.muted });
  tx(s, "T2 out", 9.1, 4.12, 0.8, 0.25, { size: 8, bold: true, color: C.green });
  tx(s, "+1 axis clock", 8.58, 4.52, 1.6, 0.26, { size: 8.5, bold: true, color: C.green, align: "center" });
  table(s, [
    ["항목", "판정"],
    ["Latency", "downstream start 관측 기준 +1 clock"],
    ["Throughput", "raw edge별 1 shot 유지"],
    ["Pipeline", "accept decision -> output FF -> downstream"],
    ["II", "정상 간격에서는 증가 없음. marker만 1 clock 이동"],
  ], 0.92, 5.05, [2.0, 9.1], 0.36, 8.2);
  footer(s, "근거: tdc_gpx_face_seq.vhd:678-719, xsim_c06_fix_face_seq_260511194330.log");
}

function slideLine(slide, x, y, w, h) {
  slide.addShape(pptx.ShapeType.line, { x, y, w, h, line: { color: C.line, width: 1.0 } });
}

{
  const s = pptx.addSlide();
  title(s, "Status / Sticky Pipeline", "운용 fault는 soft_clear 대상, hard quarantine은 reset-only 예외로 분리했다.");
  box(s, "live status fan-in", 0.95, 1.65, 2.0, 0.7, { fill: C.blueFill, line: C.blue, bold: true });
  arrow(s, 3.0, 2.0, 3.95, 2.0);
  box(s, "p_live_status FF", 3.95, 1.65, 2.0, 0.7, { fill: C.greenFill, line: C.green, bold: true });
  arrow(s, 6.0, 2.0, 6.95, 2.0);
  box(s, "STAT5/6/7 packing", 6.95, 1.65, 2.0, 0.7, { fill: C.white, line: C.line, bold: true });
  box(s, "+1 clk\nstatus sampling", 10.0, 1.52, 2.2, 0.96, { fill: C.orangeFill, line: C.orange, bold: true });
  box(s, "frame/row/run fault pulse", 1.1, 3.35, 2.45, 0.72, { fill: C.redFill, line: C.red, bold: true, size: 9.2 });
  arrow(s, 3.62, 3.72, 4.55, 3.72, C.red);
  box(s, "sticky FF", 4.55, 3.35, 1.55, 0.72, { fill: C.greenFill, line: C.green, bold: true });
  arrow(s, 6.15, 3.72, 7.15, 3.72);
  box(s, "STAT read", 7.15, 3.35, 1.45, 0.72, { fill: C.white, line: C.line, bold: true });
  box(s, "err_soft_clear\nclears", 9.55, 3.35, 1.95, 0.72, { fill: C.greenFill, line: C.green, bold: true });
  box(s, "quarantine_escape\nreset-only 유지", 4.1, 5.1, 3.1, 0.72, { fill: C.orangeFill, line: C.orange, bold: true });
  tx(s, "top sticky probe 결과: frame=0, row=0, run=0000, quarantine=0010 after soft_clear", 1.05, 6.15, 11.2, 0.36, { size: 10.5, bold: true });
  footer(s, "근거: tdc_gpx_top.vhd:1071-1127, xsim_c06_fix_top_sticky_probe_260511194330.log:95-117");
}

{
  const s = pptx.addSlide();
  title(s, "Verification Matrix", "새 source compile 기반으로 C06 핵심 경계를 확인했다.");
  table(s, [
    ["검증", "판정", "근거"],
    ["Vivado syntax", "PASS", "vivado.log:83-84"],
    ["face_seq focused", "PASS", "xsim_c06_fix_face_seq...:30-50"],
    ["mask sweep", "PASS", "xsim_c06_fix_mask_sweep...:30-54"],
    ["status_agg focused", "PASS", "xsim_c06_fix_status_agg...:28-35"],
    ["top 64-bit integration", "PASS", "xsim_c06_fix_top_int64...:80-93"],
    ["top sticky probe", "PASS", "xsim_c06_fix_top_sticky_probe...:95-117"],
  ], 0.8, 1.55, [3.0, 1.4, 7.4], 0.55, 8.4);
  tx(s, "주의: project syntax에는 tdc_gpx_sync_fifo compile-order critical warning이 남아 있다. 직접 compile/elab에서는 해당 파일을 포함해 mask sweep/top elaboration이 성공했다.", 0.92, 5.85, 11.4, 0.62, { size: 10.2, color: C.orange, bold: true });
  footer(s, "검증 로그: xsim_c06_fix_*_260511194330.log");
}

{
  const s = pptx.addSlide();
  title(s, "C06 Close / Next", "기능 blocker는 없고, project compile-order warning 정리가 후속 권장 항목이다.");
  box(s, "닫힘\ncontrol/status boundary", 0.95, 1.75, 2.75, 0.95, { fill: C.greenFill, line: C.green, bold: true });
  box(s, "닫힘\nsoft_clear policy", 4.05, 1.75, 2.45, 0.95, { fill: C.greenFill, line: C.green, bold: true });
  box(s, "닫힘\nIRQ reserved contract", 6.85, 1.75, 2.65, 0.95, { fill: C.greenFill, line: C.green, bold: true });
  box(s, "후속\ncompile-order warning", 9.85, 1.75, 2.65, 0.95, { fill: C.orangeFill, line: C.orange, bold: true });
  table(s, [
    ["항목", "판단"],
    ["C06 완료도", "필수 RTL 보완과 핵심 검증 PASS"],
    ["Latency 영향", "start output +1 clock, status +1 clock"],
    ["Throughput / II", "정상 운용 기준 변화 없음"],
    ["다음 단계", "project source list 정리 후 다음 Cluster 진입 가능"],
  ], 1.05, 3.55, [2.4, 8.8], 0.48, 9.0);
  footer(s, "C06 Code Fix Result v001");
}

pptx.writeFile({ fileName: out });
