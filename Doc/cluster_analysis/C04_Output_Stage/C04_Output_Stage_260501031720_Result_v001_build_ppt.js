const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C04_Output_Stage/C04_Output_Stage_260501031720_Result_v001.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C04 Output Stage Result v001";
pptx.title = "C04 Output Stage Result v001";
pptx.lang = "ko-KR";
pptx.theme = { headFontFace: "Malgun Gothic", bodyFontFace: "Malgun Gothic", lang: "ko-KR" };
pptx.margin = 0;

const font = "Malgun Gothic";
const C = {
  bg: "FAFBFD",
  ink: "172033",
  muted: "64748B",
  line: "CBD5E1",
  card: "FFFFFF",
  blue: "2563EB",
  blueFill: "EFF6FF",
  green: "16A34A",
  greenFill: "ECFDF5",
  orange: "EA580C",
  orangeFill: "FFF7ED",
  red: "DC2626",
  redFill: "FEF2F2",
  slate: "F8FAFC",
  cyan: "0891B2",
  cyanFill: "ECFEFF"
};

function setBg(slide) {
  slide.background = { color: C.bg };
}

function addText(slide, value, x, y, w, h, opt = {}) {
  slide.addText(value, {
    x, y, w, h,
    fontFace: font,
    fontSize: opt.size || 11,
    bold: opt.bold || false,
    color: opt.color || C.ink,
    align: opt.align || "left",
    valign: "mid",
    fit: "shrink",
    margin: opt.margin === undefined ? 0.04 : opt.margin,
    breakLine: false
  });
}

function title(slide, main, sub) {
  setBg(slide);
  addText(slide, main, 0.55, 0.28, 12.2, 0.45, { size: 21, bold: true });
  addText(slide, sub, 0.58, 0.74, 12.0, 0.3, { size: 9.2, color: C.muted });
  slide.addShape(pptx.ShapeType.line, { x: 0.55, y: 1.08, w: 12.2, h: 0, line: { color: C.line, width: 0.8 } });
}

function footer(slide, value) {
  addText(slide, value, 0.65, 6.96, 12.0, 0.25, { size: 8.0, color: C.muted, align: "center" });
}

function box(slide, value, x, y, w, h, opt = {}) {
  slide.addShape(pptx.ShapeType.rect, {
    x, y, w, h,
    fill: { color: opt.fill || C.card },
    line: { color: opt.line || C.line, width: opt.width || 1 }
  });
  addText(slide, value, x + 0.08, y + 0.06, w - 0.16, h - 0.12, {
    size: opt.size || 11,
    bold: opt.bold || false,
    color: opt.color || C.ink,
    align: opt.align || "center"
  });
}

function arrow(slide, x1, y1, x2, y2, color = C.muted) {
  slide.addShape(pptx.ShapeType.line, {
    x: x1, y: y1, w: x2 - x1, h: y2 - y1,
    line: { color, width: 1.5, beginArrowType: "none", endArrowType: "triangle" }
  });
}

function table(slide, rows, x, y, widths, rowH, size = 9.5) {
  rows.forEach((row, r) => {
    let curX = x;
    row.forEach((cell, c) => {
      slide.addShape(pptx.ShapeType.rect, {
        x: curX,
        y: y + r * rowH,
        w: widths[c],
        h: rowH,
        fill: { color: r === 0 ? "E2E8F0" : C.card },
        line: { color: C.line, width: 0.7 }
      });
      addText(slide, cell, curX + 0.05, y + r * rowH + 0.03, widths[c] - 0.1, rowH - 0.06, {
        size,
        bold: r === 0,
        align: c === 0 ? "center" : "left"
      });
      curX += widths[c];
    });
  });
}

{
  const s = pptx.addSlide();
  title(s, "C04 수정 결과 v001", "최종 VDMA stream에서는 Hit[16]을 버리고 metadata[6:0]을 0으로 정리");
  box(s, "C03 cell stream\nmetadata[6:0]\n= Hit[16] vector", 0.75, 1.65, 2.25, 1.05, { fill: C.orangeFill, line: C.orange, bold: true });
  arrow(s, 3.05, 2.18, 3.75, 2.18, C.orange);
  box(s, "C04-C\nface_assembler\nmetadata clear", 3.8, 1.65, 2.15, 1.05, { fill: C.blueFill, line: C.blue, bold: true });
  arrow(s, 6.0, 2.18, 6.7, 2.18, C.blue);
  box(s, "Output FIFO\n+ header prefix", 6.75, 1.65, 2.0, 1.05, { fill: C.cyanFill, line: C.cyan, bold: true });
  arrow(s, 8.8, 2.18, 9.5, 2.18, C.cyan);
  box(s, "Final VDMA\nmetadata[6:0]=0\nHit[15:0] only", 9.55, 1.65, 2.45, 1.05, { fill: C.greenFill, line: C.green, bold: true });
  table(s, [
    ["항목", "반영"],
    ["RTL", "real-chip metadata beat에서 tdata[6:0] clear"],
    ["TB", "metadata[6:0]=0x55 주입 후 final VDMA에서 0 확인"],
    ["정책", "full 17-bit output 복원은 다음 generation에서 검토"]
  ], 1.0, 4.15, [2.0, 9.3], 0.5, 10);
  footer(s, "기준: Doc/TDC-GPX-Datasheet.pdf, 계획: C04_Output_Stage_260501030046_Plan_v002");
}

{
  const s = pptx.addSlide();
  title(s, "수정 지점", "C03 내부 보존과 C04 final output 계약을 분리");
  table(s, [
    ["파일", "핵심 변경", "추적 근거"],
    ["tdc_gpx_face_assembler.vhd", "metadata beat forwarding 시 [6:0]=0", "827-831"],
    ["tb_tdc_gpx_output_stage.vhd", "runtime beats/cell 기준으로 TB 정렬", "45-46"],
    ["tb_tdc_gpx_output_stage.vhd", "final metadata sanitize monitor 추가", "176-177, 327-329"],
    ["tb_tdc_gpx_output_stage_w32/w128.vhd", "폭별 wrapper TB 추가", "전체 파일"]
  ], 0.7, 1.35, [3.05, 5.35, 2.65], 0.56, 9.3);
  box(s, "중요 판단\nstatic fn_beats_per_cell이 아니라 runtime fn_beats_per_cell_rt(max_hits_cfg, width)를 검증 기준으로 사용", 1.0, 5.25, 11.2, 0.82, { fill: C.blueFill, line: C.blue, size: 10.5, bold: true });
  footer(s, "metadata clear는 등록 출력 직전 bit mask라 pipeline stage와 II를 증가시키지 않음");
}

{
  const s = pptx.addSlide();
  title(s, "Timing / Pipeline / II", "width가 넓어질수록 runtime line beat 수가 감소");
  box(s, "C03 beat\naccepted", 0.75, 1.55, 1.7, 0.7, { fill: C.card, bold: true });
  arrow(s, 2.48, 1.9, 3.05, 1.9);
  box(s, "face_assembler\nsanitize", 3.08, 1.55, 1.95, 0.7, { fill: C.blueFill, line: C.blue, bold: true });
  arrow(s, 5.06, 1.9, 5.62, 1.9);
  box(s, "output FIFO", 5.65, 1.55, 1.65, 0.7, { fill: C.cyanFill, line: C.cyan, bold: true });
  arrow(s, 7.33, 1.9, 7.9, 1.9);
  box(s, "header\nprefix/data", 7.93, 1.55, 1.85, 0.7, { fill: C.orangeFill, line: C.orange, bold: true });
  arrow(s, 9.82, 1.9, 10.38, 1.9);
  box(s, "VDMA\nII=1 beat/clk", 10.42, 1.55, 1.9, 0.7, { fill: C.greenFill, line: C.green, bold: true });
  table(s, [
    ["Width", "Header", "Runtime cell", "Scenario1 beats", "완료 시각"],
    ["32", "12", "5", "22", "317.5 ns"],
    ["64", "6", "3", "12", "297.5 ns"],
    ["128", "3", "2", "7", "287.5 ns"]
  ], 1.15, 3.25, [1.25, 1.5, 2.0, 2.1, 2.0], 0.48, 9.5);
  footer(s, "조건: active_chip=1, stops_per_chip=2, max_hits_cfg=7, 200 MHz TB clock");
}

{
  const s = pptx.addSlide();
  title(s, "검증 결과", "Vivado 2025.2.1 xsim 폭별 PASS");
  table(s, [
    ["Width", "Snapshot", "metadata", "Scenario"],
    ["32", "tb_tdc_gpx_output_stage_c04_w32", "sanitize ok / seen 2", "S1 PASS, S2 PASS"],
    ["64", "tb_tdc_gpx_output_stage_c04_w64", "sanitize ok / seen 2", "S1 PASS, S2 PASS"],
    ["128", "tb_tdc_gpx_output_stage_c04_w128", "sanitize ok / seen 2", "S1 PASS, S2 PASS"]
  ], 0.75, 1.45, [1.0, 4.6, 3.0, 3.0], 0.58, 9.4);
  box(s, "공통 확인\nmetadata[6:0]에 0x55를 넣어도 final VDMA metadata beat에서는 0으로 관측됨", 1.0, 4.75, 11.2, 0.78, { fill: C.greenFill, line: C.green, size: 10.5, bold: true });
  footer(s, "로그: xsim_output_stage_c04_w32/w64/w128.log");
}

{
  const s = pptx.addSlide();
  title(s, "닫힌 항목과 후속", "이번 generation final stream 계약은 닫고, 17-bit 복원은 다음 generation으로 유지");
  table(s, [
    ["구분", "판단"],
    ["닫힘", "Q-C04-01: final VDMA metadata[6:0]=0 구현 및 검증 완료"],
    ["유지", "C03 내부 metadata[6:0] 보존은 유지 가능"],
    ["제외", "최종 VDMA Hit[16] 출력은 이번 generation에서 제외"],
    ["후속", "full 17-bit output format, SW parser, VDMA layout 재검토"]
  ], 1.05, 1.55, [1.45, 9.75], 0.58, 10);
  footer(s, "C04는 다음 Cluster 진입 전 final output 계약 관점의 핵심 위험을 하나 닫음");
}

pptx.writeFile({ fileName: out });
