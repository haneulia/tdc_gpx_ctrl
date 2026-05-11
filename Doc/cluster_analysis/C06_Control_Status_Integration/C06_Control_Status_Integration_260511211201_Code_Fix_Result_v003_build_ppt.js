const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C06_Control_Status_Integration/C06_Control_Status_Integration_260511211201_Code_Fix_Result_v003.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C06 Code Fix Result v003";
pptx.title = "C06 Code Fix Result v003";
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
    breakLine: false,
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

function table(slide, rows, x, y, widths, rowH, size = 8.4) {
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
  tx(s, "Code Fix Result v003", 0.76, 1.72, 5.0, 0.38, { size: 15, color: "CBD5E1" });
  tx(s, "force_reinit recovery는 PASS. soft_reset recovery는 chip_busy/PH_RESP_DRAIN 경계 Open으로 분리.", 0.78, 2.48, 10.8, 0.62, { size: 13.2, color: C.white });
  box(s, "PASS\nforce_reinit", 0.86, 3.7, 2.35, 0.95, { fill: "14532D", line: "86EFAC", color: C.white, bold: true });
  box(s, "OPEN\nsoft_reset", 3.7, 3.7, 2.35, 0.95, { fill: "7F1D1D", line: "FCA5A5", color: C.white, bold: true });
  box(s, "ACCEPT\nSTAT polling", 6.55, 3.7, 2.35, 0.95, { fill: "1E3A8A", line: "93C5FD", color: C.white, bold: true });
  box(s, "NEXT\nPlan v004", 9.4, 3.7, 2.35, 0.95, { fill: "713F12", line: "FDE68A", color: C.white, bold: true });
  footer(s, "C06 Code Fix Result v003 | 생성 2026-05-11 21:12:01 KST | 실행 stamp 260511212000");
}

{
  const s = pptx.addSlide();
  title(s, "Recovery 검증 Matrix", "v003은 recovery 경로를 force_reinit PASS와 soft_reset Open으로 분리했다.");
  table(s, [
    ["ID", "시나리오", "결과", "근거"],
    ["V3-01", "v002 baseline", "PASS", "v003 script 선행 실행"],
    ["V3-02", "run -> force_reinit -> run", "PASS", "2회 run, total 88 beats"],
    ["V3-03", "post-force STAT", "PASS", "T7 post-force-reinit"],
    ["V3-04", "output/IRQ", "PASS", "output PASS, IRQ 0"],
    ["V3-05", "run -> soft_reset -> run", "OPEN", "pipeline busy pending 후 IFIFO read=0"],
    ["V3-06", "o_irq_pipe policy", "ACCEPT", "reserved/tie-off 유지"],
  ], 0.66, 1.38, [1.0, 3.25, 1.7, 6.0], 0.58, 8.3);
  footer(s, "근거 로그: xsim_c06_v003_top_int_force_260511212000.log / xsim_c06_v003_top_int_soft_260511212000.log");
}

{
  const s = pptx.addSlide();
  title(s, "force_reinit PASS 흐름", "PH_RESP_DRAIN을 우회해 PH_INIT으로 재진입하고 다음 run이 정상 출력된다.");
  const nodes = [
    ["Run #1\nPASS", 0.7, 2.1, C.greenFill, C.green],
    ["T8\nFORCE_REINIT", 2.55, 2.1, C.purpleFill, C.purple],
    ["PH_INIT\nre-entry", 4.55, 2.1, C.blueFill, C.blue],
    ["Run #2\nSTART", 6.55, 2.1, C.orangeFill, C.orange],
    ["Run #2\nPASS", 8.45, 2.1, C.greenFill, C.green],
    ["STAT\nclean", 10.35, 2.1, C.slate, C.muted],
  ];
  nodes.forEach(([label, x, y, fill, line], i) => {
    box(s, label, x, y, 1.35, 1.0, { fill, line, bold: true, size: 9.0 });
    if (i < nodes.length - 1) arrow(s, x + 1.38, y + 0.5, nodes[i + 1][1] - 0.06, y + 0.5);
  });
  table(s, [
    ["구간", "run1", "run2", "판단"],
    ["T0->T1", "11clk", "11clk", "동일"],
    ["T0->T2", "92clk", "92clk", "동일"],
    ["T0->T5", "192/189clk", "190/189clk", "보존"],
  ], 1.1, 4.28, [2.1, 2.3, 2.3, 4.4], 0.48, 8.4);
  footer(s, "근거: force log T0/T1/T2/T5 markers, cycle 2891~17465");
}

{
  const s = pptx.addSlide();
  title(s, "soft_reset Open 흐름", "status read가 가능해도 다음 START accept가 닫히지 않는다.");
  const nodes = [
    ["Run #1\nPASS", 0.8, 2.0, C.greenFill, C.green],
    ["T8\nSOFT_RESET", 2.65, 2.0, C.orangeFill, C.orange],
    ["PH_RESP\nDRAIN", 4.6, 2.0, C.redFill, C.red],
    ["Run #2\nSTART", 6.55, 2.0, C.blueFill, C.blue],
    ["pending\nbusy", 8.5, 2.0, C.redFill, C.red],
    ["IFIFO\nread=0", 10.45, 2.0, C.redFill, C.red],
  ];
  nodes.forEach(([label, x, y, fill, line], i) => {
    box(s, label, x, y, 1.35, 1.0, { fill, line, bold: true, size: 9.0 });
    if (i < nodes.length - 1) arrow(s, x + 1.38, y + 0.5, nodes[i + 1][1] - 0.06, y + 0.5, i >= 2 ? C.red : C.muted);
  });
  tx(s, "12,000 clk 추가 대기 후에도 face_seq는 pipeline busy pending을 반복했다. 따라서 단순 대기값 부족이 아니라 chip_busy/PH_RESP_DRAIN 경계 Open으로 판단한다.", 0.9, 4.05, 11.3, 0.7, { size: 12 });
  table(s, [
    ["지점", "cycle", "판단"],
    ["soft_reset", "8692", "MAIN_CTRL[30]"],
    ["post-soft status", "11570", "CSR read 가능"],
    ["shot stimulus", "23605", "accept 불가 상태"],
    ["failure", "24697", "IFIFO read=0"],
  ], 1.0, 5.0, [2.5, 2.0, 6.3], 0.42, 8.0);
  footer(s, "근거: soft log T8/T7/pending/Failure markers");
}

{
  const s = pptx.addSlide();
  title(s, "v004 수정 방향", "soft_reset을 회복 명령으로 보장할지, force_reinit 전용 recovery로 계약화할지 결정한다.");
  table(s, [
    ["후보", "방향", "장점", "위험"],
    ["A", "soft_reset도 PH_INIT 회복 보장", "SW 단순", "stale response pollution"],
    ["B", "force_reinit만 recovery", "현재 구조와 주석에 부합", "SW 계약 복잡"],
    ["C", "bus_phy/response soft flush", "soft_reset 안전 보장 가능", "RTL 영향 큼"],
  ], 0.7, 1.45, [1.0, 3.4, 3.1, 4.6], 0.62, 8.2);
  tx(s, "권고: 먼저 focused TB로 i_bus_busy / i_bus_rsp_pending / s_drain_to_init_r를 분리 계측한 뒤 A/B/C 중 하나를 선택한다.", 0.8, 4.8, 11.6, 0.55, { size: 12.2, bold: true });
  footer(s, "다음 산출물: Code Fix Plan v004 -> Result v004 -> C06 handoff 갱신");
}

pptx.writeFile({ fileName: out });
