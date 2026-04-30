const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C02_Chip_Acquisition/C02_Chip_Acquisition_260501020359_CTL21_Max_Hits_Timing_Contract_v001.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C02 CTL21 max_hits timing contract";
pptx.title = "C02 CTL21 Max Hits Timing Contract v001";
pptx.company = "tdc_gpx_ctrl";
pptx.lang = "ko-KR";
pptx.theme = {
  headFontFace: "Malgun Gothic",
  bodyFontFace: "Malgun Gothic",
  lang: "ko-KR"
};

const font = "Malgun Gothic";
const C = {
  bg: "FAFBFD",
  ink: "172033",
  muted: "64748B",
  line: "CBD5E1",
  table: "E2E8F0",
  card: "FFFFFF",
  blue: "2563EB",
  blueFill: "EFF6FF",
  green: "16A34A",
  greenFill: "ECFDF5",
  orange: "EA580C",
  orangeFill: "FFF7ED",
  red: "DC2626",
  redFill: "FEF2F2",
  cyan: "0891B2",
  cyanFill: "ECFEFF",
  violet: "7C3AED",
  violetFill: "F5F3FF"
};

function bg(slide) {
  slide.background = { color: C.bg };
}

function text(slide, value, x, y, w, h, opt = {}) {
  slide.addText(value, {
    x, y, w, h,
    fontFace: font,
    fontSize: opt.size || 12,
    bold: opt.bold || false,
    color: opt.color || C.ink,
    align: opt.align || "left",
    valign: opt.valign || "mid",
    fit: "shrink",
    margin: opt.margin === undefined ? 0.04 : opt.margin,
    breakLine: false
  });
}

function box(slide, value, x, y, w, h, opt = {}) {
  slide.addShape(pptx.ShapeType.roundRect, {
    x, y, w, h,
    rectRadius: 0.05,
    fill: { color: opt.fill || C.card },
    line: { color: opt.line || C.line, width: opt.lineWidth || 1 }
  });
  text(slide, value, x + 0.07, y + 0.05, w - 0.14, h - 0.1, {
    size: opt.size || 11.5,
    bold: opt.bold || false,
    color: opt.color || C.ink,
    align: opt.align || "center"
  });
}

function arrow(slide, x1, y1, x2, y2, color = C.muted) {
  slide.addShape(pptx.ShapeType.line, {
    x: x1,
    y: y1,
    w: x2 - x1,
    h: y2 - y1,
    line: { color, width: 1.4, beginArrowType: "none", endArrowType: "triangle" }
  });
}

function title(slide, main, sub) {
  bg(slide);
  text(slide, main, 0.55, 0.27, 12.2, 0.46, { size: 21.5, bold: true });
  text(slide, sub, 0.58, 0.74, 12.0, 0.28, { size: 9.4, color: C.muted });
  slide.addShape(pptx.ShapeType.line, {
    x: 0.55, y: 1.08, w: 12.2, h: 0,
    line: { color: C.line, width: 0.8 }
  });
}

function foot(slide, value) {
  text(slide, value, 0.62, 6.96, 12.1, 0.25, {
    size: 8.1,
    color: C.muted,
    align: "center"
  });
}

function table(slide, rows, x, y, widths, rowH = 0.43, size = 10.4) {
  rows.forEach((row, r) => {
    let cx = x;
    row.forEach((cell, c) => {
      const header = r === 0;
      box(slide, String(cell), cx, y + r * rowH, widths[c], rowH - 0.03, {
        fill: header ? C.table : C.card,
        line: C.line,
        size: header ? size : size - 0.4,
        bold: header,
        align: "center"
      });
      cx += widths[c] + 0.04;
    });
  });
}

function lane(slide, label, x, y, w, color) {
  slide.addShape(pptx.ShapeType.line, {
    x, y, w, h: 0,
    line: { color, width: 1.2, beginArrowType: "none", endArrowType: "triangle" }
  });
  text(slide, label, x, y - 0.25, w, 0.22, { size: 9.5, color, bold: true, align: "center" });
}

let s = pptx.addSlide();
bg(s);
text(s, "C02 CTL21 Max Hits Timing Contract", 0.72, 0.58, 12.0, 0.55, { size: 27, bold: true, color: C.blue });
text(s, "거리/운용 모드별 max_hits_cfg는 face 시작 전에 설정되어야 output width 이득이 반영된다", 0.74, 1.20, 11.7, 0.42, { size: 16.5, bold: true });
box(s, "early\nmax_hits=3\n44 beats", 0.85, 2.05, 2.35, 1.08, { fill: C.greenFill, line: C.green, color: C.green, size: 15, bold: true });
box(s, "unset\nalias 7\n60 beats", 3.55, 2.05, 2.35, 1.08, { fill: C.orangeFill, line: C.orange, color: C.orange, size: 15, bold: true });
box(s, "zero\nalias 7\n60 beats", 6.25, 2.05, 2.35, 1.08, { fill: C.orangeFill, line: C.orange, color: C.orange, size: 15, bold: true });
box(s, "late\nface0 60 + face1 44\n104 beats", 8.95, 2.05, 3.1, 1.08, { fill: C.cyanFill, line: C.cyan, color: C.cyan, size: 14, bold: true });
box(s, "판단: 폭 generic만으로 throughput 이득이 확정되지 않는다.\n`CTL21[18:16]`이 packet_start 전에 유효해야 현재 face cell beat가 줄어든다.", 1.0, 4.15, 11.3, 0.85, { fill: C.blueFill, line: C.blue, color: C.blue, size: 14, bold: true });
text(s, "작성: 2026-05-01 02:03:59 KST / 수정: 2026-05-01 02:07:23 KST", 0.95, 6.22, 9.5, 0.3, { size: 10.7, color: C.muted });
foot(s, "근거: xsim_top_ctl21_early64/unset64/zero64/late64.log, tb_tdc_gpx_top_int.vhd");

s = pptx.addSlide();
title(s, "설정 경로와 스냅샷 경계", "CTL21은 CDC를 통과한 뒤 face_seq의 packet_start에서 face 설정으로 닫힌다.");
box(s, "AXI4-Lite write\nCTL21[18:16]", 0.75, 1.62, 1.9, 0.78, { fill: C.blueFill, line: C.blue, color: C.blue, size: 12.8, bold: true });
arrow(s, 2.65, 2.01, 3.25, 2.01, C.blue);
box(s, "csr_chip\nxpm_cdc_handshake", 3.25, 1.62, 2.0, 0.78, { fill: C.cyanFill, line: C.cyan, color: C.cyan, size: 12.2, bold: true });
arrow(s, 5.25, 2.01, 5.85, 2.01, C.cyan);
box(s, "config_ctrl\nmerged cfg", 5.85, 1.62, 1.7, 0.78, { fill: C.violetFill, line: C.violet, color: C.violet, size: 12.2, bold: true });
arrow(s, 7.55, 2.01, 8.15, 2.01, C.violet);
box(s, "face_seq\npacket_start snapshot", 8.15, 1.62, 2.05, 0.78, { fill: C.orangeFill, line: C.orange, color: C.orange, size: 12.2, bold: true });
arrow(s, 10.2, 2.01, 10.85, 2.01, C.orange);
box(s, "cell/output\nbeat 결정", 10.85, 1.62, 1.75, 0.78, { fill: C.greenFill, line: C.green, color: C.green, size: 12.2, bold: true });
box(s, "핵심 경계\npacket_start 이후에는 현재 face의 max_hits_cfg가 고정된다.", 1.0, 3.15, 5.25, 0.72, { fill: C.redFill, line: C.red, color: C.red, size: 13, bold: true });
box(s, "START 보호\ncsr_pipeline은 chip CSR CDC idle을 start gate에 반영한다.", 7.0, 3.15, 5.25, 0.72, { fill: C.greenFill, line: C.green, color: C.green, size: 13, bold: true });
table(s, [
  ["근거", "코드 위치"],
  ["CTL21 bit / CDC", "tdc_gpx_csr_chip.vhd:19, 596-615"],
  ["0 -> 7 alias", "tdc_gpx_csr_chip.vhd:873-876"],
  ["START CDC idle gate", "tdc_gpx_csr_pipeline.vhd:607-610, 670"],
  ["face cfg snapshot", "tdc_gpx_face_seq.vhd:378-384, 490"],
  ["top 연결", "tdc_gpx_top.vhd:683, 765, 913"],
], 1.0, 4.35, [3.15, 8.15], 0.40, 10.1);
foot(s, "해석: CTL21 write가 START보다 먼저 issued 되면 START acceptance는 CDC idle 이후로 밀린다.");

s = pptx.addSlide();
title(s, "Early vs Late Timing", "같은 max_hits_cfg=3이라도 packet_start 전후에 따라 적용 face가 달라진다.");
lane(s, "early: 현재 face 반영", 0.95, 2.0, 5.3, C.green);
box(s, "CTL21=3", 1.0, 2.25, 1.2, 0.48, { fill: C.greenFill, line: C.green, color: C.green, size: 10.5, bold: true });
box(s, "CDC done", 2.5, 2.25, 1.25, 0.48, { fill: C.greenFill, line: C.green, color: C.green, size: 10.5, bold: true });
box(s, "START", 4.0, 2.25, 1.1, 0.48, { fill: C.greenFill, line: C.green, color: C.green, size: 10.5, bold: true });
box(s, "packet_start\nsnapshot=3", 5.4, 2.14, 1.7, 0.7, { fill: C.greenFill, line: C.green, color: C.green, size: 10.5, bold: true });
box(s, "44 beats", 7.45, 2.20, 1.35, 0.58, { fill: C.greenFill, line: C.green, color: C.green, size: 12, bold: true });

lane(s, "late: 다음 face부터 반영", 0.95, 4.25, 5.3, C.orange);
box(s, "face0\npacket_start", 1.0, 4.38, 1.55, 0.64, { fill: C.orangeFill, line: C.orange, color: C.orange, size: 10.5, bold: true });
box(s, "CTL21=3", 3.0, 4.46, 1.2, 0.48, { fill: C.orangeFill, line: C.orange, color: C.orange, size: 10.5, bold: true });
box(s, "CDC done", 4.55, 4.46, 1.25, 0.48, { fill: C.orangeFill, line: C.orange, color: C.orange, size: 10.5, bold: true });
box(s, "face1\npacket_start", 6.25, 4.38, 1.55, 0.64, { fill: C.cyanFill, line: C.cyan, color: C.cyan, size: 10.5, bold: true });
box(s, "60 + 44\n= 104 beats", 8.3, 4.34, 1.65, 0.72, { fill: C.cyanFill, line: C.cyan, color: C.cyan, size: 12, bold: true });
box(s, "금지 오해\nface 중간 CTL21 write가 현재 face output 형식을 바꾸는 구조가 아니다.", 1.05, 5.95, 11.25, 0.55, { fill: C.redFill, line: C.red, color: C.red, size: 12.5, bold: true });
foot(s, "근거: xsim_top_ctl21_early64.log:55/89/93, xsim_top_ctl21_late64.log:78/98/102");

s = pptx.addSlide();
title(s, "xsim 검증 Matrix", "64-bit top 통합 TB에서 CTL21 설정 시점별 output beat와 TLAST를 assert했다.");
table(s, [
  ["Mode", "조건", "기대", "R/F beats", "TLAST", "결과"],
  ["early", "packet_start 전 3", "44", "44 / 44", "2 / 2", "PASS"],
  ["unset", "CTL21 미설정", "60", "60 / 60", "2 / 2", "PASS"],
  ["zero", "CTL21=0 alias", "60", "60 / 60", "2 / 2", "PASS"],
  ["late", "face0 후 3", "104", "104 / 104", "4 / 4", "PASS"],
], 0.75, 1.35, [1.25, 2.5, 1.2, 1.55, 1.25, 1.25], 0.50, 10.8);
box(s, "TB 보완\nG_MAX_HITS_WRITE_MODE로 unset/early/zero/late를 분리하고, expected beat와 tlast를 mode별로 assert했다.", 0.9, 4.55, 11.55, 0.68, { fill: C.blueFill, line: C.blue, color: C.blue, size: 13, bold: true });
box(s, "추가 발견\nfire_count / expected-count matching은 run global이 아니라 face-local shot count 기준으로 맞아야 한다.", 0.9, 5.65, 11.55, 0.68, { fill: C.violetFill, line: C.violet, color: C.violet, size: 13, bold: true });
foot(s, "근거: tb_tdc_gpx_top_int.vhd:74-80, 907-918, 954-964, 993-1022");

s = pptx.addSlide();
title(s, "Latency / Throughput / Pipeline / II", "CTL21은 GPX READ II를 바꾸지 않고 final output serialize beat 수를 줄인다.");
table(s, [
  ["상태", "max_hits", "beats/slope", "150MHz time", "판단"],
  ["unset/zero", "7", "60", "0.400 us", "안전 기본값"],
  ["early", "3", "44", "0.293 us", "약 26.7% 감소"],
  ["late 2-face", "7 -> 3", "104", "0.693 us", "다음 face 반영"],
], 0.75, 1.35, [2.0, 1.35, 1.65, 1.65, 3.1], 0.52, 10.4);
box(s, "Pipeline 경계: CTL21 write -> CSR CDC -> config_ctrl -> packet_start snapshot -> cell/output beat 결정", 0.95, 4.18, 11.35, 0.58, { fill: C.cyanFill, line: C.cyan, color: C.cyan, size: 12.5, bold: true });
box(s, "II 해석\nfinal AXIS ready가 유지되면 beat 단위 II=1이다. Width와 max_hits는 II를 1보다 낮추는 것이 아니라 필요한 beat 수를 줄인다.", 0.95, 5.12, 11.35, 0.78, { fill: C.greenFill, line: C.green, color: C.green, size: 12.6, bold: true });
box(s, "운용 규칙\n거리/모드별 CTL21.max_hits_cfg를 START 또는 face packet_start 이전에 설정한다. 000은 7-hit alias로 관리한다.", 0.95, 6.18, 11.35, 0.54, { fill: C.redFill, line: C.red, color: C.red, size: 12.2, bold: true });
foot(s, "C02 계약: OP-C02-CTL21-01 ~ OP-C02-CTL21-05");

pptx.writeFile({ fileName: out });
