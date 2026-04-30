const pptxgen = require("C:/Users/victe/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/pptxgenjs");

const out = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL/Doc/cluster_analysis/C03_Cell_Pipe/C03_Cell_Pipe_260501024504_Fix_Result_v001.pptx";

const pptx = new pptxgen();
pptx.defineLayout({ name: "CUSTOM_WIDE", width: 13.333, height: 7.5 });
pptx.layout = "CUSTOM_WIDE";
pptx.author = "Codex";
pptx.subject = "C03 Cell Pipe Fix Result";
pptx.title = "C03 Cell Pipe Fix Result v001";
pptx.lang = "ko-KR";
pptx.theme = { headFontFace: "Malgun Gothic", bodyFontFace: "Malgun Gothic", lang: "ko-KR" };

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
  cyan: "0891B2",
  cyanFill: "ECFEFF",
  slateFill: "F8FAFC"
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

function title(slide, main, sub) {
  bg(slide);
  text(slide, main, 0.55, 0.28, 12.2, 0.45, { size: 21.5, bold: true });
  text(slide, sub, 0.58, 0.74, 12.0, 0.3, { size: 9.2, color: C.muted });
  slide.addShape(pptx.ShapeType.line, { x: 0.55, y: 1.08, w: 12.2, h: 0, line: { color: C.line, width: 0.8 } });
}

function foot(slide, value) {
  text(slide, value, 0.65, 6.96, 12.0, 0.25, { size: 8.2, color: C.muted, align: "center" });
}

function box(slide, value, x, y, w, h, opt = {}) {
  slide.addShape(pptx.ShapeType.roundRect, {
    x, y, w, h,
    rectRadius: 0.05,
    fill: { color: opt.fill || C.card },
    line: { color: opt.line || C.line, width: opt.width || 1 }
  });
  text(slide, value, x + 0.08, y + 0.06, w - 0.16, h - 0.12, {
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

function pill(slide, value, x, y, w, color, fill) {
  box(slide, value, x, y, w, 0.38, { fill, line: color, color, size: 9.5, bold: true });
}

function table(slide, rows, x, y, widths, rowH, size) {
  rows.forEach((row, r) => {
    let curX = x;
    row.forEach((cell, c) => {
      slide.addShape(pptx.ShapeType.rect, {
        x: curX, y: y + r * rowH, w: widths[c], h: rowH,
        fill: { color: r === 0 ? "E2E8F0" : C.card },
        line: { color: C.line, width: 0.7 }
      });
      text(slide, cell, curX + 0.05, y + r * rowH + 0.04, widths[c] - 0.1, rowH - 0.08, {
        size,
        bold: r === 0,
        align: c === 0 ? "center" : "left"
      });
      curX += widths[c];
    });
  });
}

// Slide 1
{
  const s = pptx.addSlide();
  title(s, "C03 Cell Pipe 보완 결과", "2026-05-01 02:45 KST | 기준: TDC-GPX Datasheet | 대상: cell_pipe / cell_builder");
  pill(s, "F-C03-01 닫힘", 0.75, 1.35, 2.25, C.green, C.greenFill);
  pill(s, "F-C03-02 닫힘", 3.25, 1.35, 2.25, C.green, C.greenFill);
  pill(s, "F-C03-03 닫힘", 5.75, 1.35, 2.25, C.green, C.greenFill);
  pill(s, "Regression PASS", 8.25, 1.35, 2.65, C.blue, C.blueFill);

  box(s, "Hit[16]\nmetadata 보존", 0.85, 2.1, 2.55, 1.0, { fill: C.greenFill, line: C.green, bold: true });
  arrow(s, 3.45, 2.6, 4.15, 2.6, C.green);
  box(s, "입력 skid\nready 경계", 4.2, 2.1, 2.55, 1.0, { fill: C.blueFill, line: C.blue, bold: true });
  arrow(s, 6.8, 2.6, 7.5, 2.6, C.blue);
  box(s, "per-slope abort\nstale 제거", 7.55, 2.1, 2.55, 1.0, { fill: C.orangeFill, line: C.orange, bold: true });
  arrow(s, 10.15, 2.6, 10.85, 2.6, C.orange);
  box(s, "C04 계약\nmetadata 수락", 10.9, 2.1, 1.65, 1.0, { fill: C.slateFill, line: C.line, bold: true, size: 10 });

  table(s, [
    ["항목", "핵심 판단"],
    ["Datasheet", "I-Mode data는 ChaCode[27:26], Start#[25:18], Slope[17], Hit[16:0] 구조"],
    ["RTL", "16-bit hit slot 유지, Hit[16]은 metadata [6:0]에 slot별 보존"],
    ["검증", "기존 smoke PASS + C03 전용 Hit[16]/skid/abort regression PASS"]
  ], 0.85, 3.65, [1.55, 10.1], 0.5, 10.2);
  foot(s, "근거: Doc/TDC-GPX-Datasheet.pdf page 20, 27 / xsim_c03_cell_pipe_fix.log:36");
}

// Slide 2
{
  const s = pptx.addSlide();
  title(s, "Hit[16] 보존 구조", "slot 폭을 키우지 않고 metadata 예약 bit로 17-bit raw hit를 복원 가능하게 만든다");
  box(s, "Datasheet\nHit[16:0]", 0.8, 1.55, 1.85, 0.9, { fill: C.slateFill, bold: true });
  arrow(s, 2.7, 2.0, 3.45, 2.0);
  box(s, "cell_builder\ncollect", 3.5, 1.55, 2.05, 0.9, { fill: C.blueFill, line: C.blue, bold: true });
  arrow(s, 5.6, 1.78, 6.35, 1.45);
  arrow(s, 5.6, 2.23, 6.35, 2.65);
  box(s, "hit_slot[n]\nHit[15:0]", 6.4, 1.1, 2.05, 0.8, { fill: C.greenFill, line: C.green, bold: true });
  box(s, "hit_msb_vec[n]\nHit[16]", 6.4, 2.45, 2.05, 0.8, { fill: C.orangeFill, line: C.orange, bold: true });
  arrow(s, 8.5, 1.5, 9.25, 1.5, C.green);
  arrow(s, 8.5, 2.85, 9.25, 2.85, C.orange);
  box(s, "cell stream\nC04 입력", 9.3, 1.78, 2.35, 0.9, { fill: C.card, line: C.line, bold: true });

  table(s, [
    ["Metadata bit", "의미"],
    ["[31:25]", "hit_valid[6:0]"],
    ["[24:18]", "slope_vec[6:0]"],
    ["[15:12]", "hit_count_actual"],
    ["[11]/[10]", "hit_dropped / error_fill"],
    ["[9:8]", "chip_id"],
    ["[6:0]", "hit_msb_vec[6:0] = Hit[16] per slot"]
  ], 1.05, 3.65, [2.2, 8.85], 0.38, 9.4);
  foot(s, "근거: tdc_gpx_pkg.vhd:610-619, tdc_gpx_cell_builder.vhd:399-428, 653");
}

// Slide 3
{
  const s = pptx.addSlide();
  title(s, "Ready 경계와 Abort 보강", "Cluster 2/3 경계 ready는 skid가 닫고, demux는 skid output만 pop한다");
  box(s, "C2 event\nstream", 0.75, 2.05, 1.65, 0.85, { fill: C.slateFill, bold: true });
  arrow(s, 2.45, 2.48, 3.15, 2.48);
  box(s, "C03 input skid\n2-deep", 3.2, 1.95, 2.1, 1.05, { fill: C.blueFill, line: C.blue, bold: true });
  arrow(s, 5.35, 2.48, 6.05, 2.48);
  box(s, "slope demux\nready decision", 6.1, 1.95, 2.25, 1.05, { fill: C.card, line: C.line, bold: true });
  arrow(s, 8.4, 2.22, 9.15, 1.65, C.green);
  arrow(s, 8.4, 2.72, 9.15, 3.25, C.orange);
  box(s, "rising hold\nregister", 9.2, 1.15, 2.0, 0.85, { fill: C.greenFill, line: C.green, bold: true });
  box(s, "falling hold\nregister", 9.2, 2.9, 2.0, 0.85, { fill: C.orangeFill, line: C.orange, bold: true });

  pill(s, "i_abort_rise: rising clear/drop", 1.0, 4.25, 3.15, C.green, C.greenFill);
  pill(s, "i_abort_fall: falling clear/drop", 4.35, 4.25, 3.15, C.orange, C.orangeFill);
  pill(s, "i_abort: skid flush + both clear", 7.7, 4.25, 3.1, C.red, C.redFill);
  table(s, [
    ["문제", "해결"],
    ["stale-ready", "외부 ready를 skid registered ready로 전환"],
    ["holding stale", "slope별 abort로 valid clear"],
    ["abort 중 beat", "target slope가 abort 중이면 pop 후 drop"]
  ], 1.0, 5.0, [2.0, 8.8], 0.42, 9.5);
  foot(s, "근거: tdc_gpx_cell_pipe.vhd:171-203, 214-250");
}

// Slide 4
{
  const s = pptx.addSlide();
  title(s, "Timing / Pipeline / II 영향", "안정성은 증가하고 steady-state throughput은 유지된다");
  table(s, [
    ["Metric", "결과", "판단"],
    ["Latency", "C2/C3 boundary +1 clk", "input skid 추가 영향"],
    ["Throughput", "32/64/128 beat 수 동일", "Hit[16]은 metadata bit 사용"],
    ["Pipeline", "skid -> demux -> builder", "경계 register화"],
    ["II", "ready 유지 시 II=1", "skid/demux/builder가 모두 받을 때"]
  ], 0.85, 1.45, [1.7, 3.8, 6.0], 0.55, 10.0);

  box(s, "32-bit\n5 beats/cell", 1.1, 4.45, 2.2, 0.75, { fill: C.slateFill, bold: true });
  box(s, "64-bit\n3 beats/cell", 3.65, 4.45, 2.2, 0.75, { fill: C.blueFill, line: C.blue, bold: true });
  box(s, "128-bit\n2 beats/cell", 6.2, 4.45, 2.2, 0.75, { fill: C.greenFill, line: C.green, bold: true });
  box(s, "metadata\nlast beat 유지", 8.75, 4.45, 2.2, 0.75, { fill: C.orangeFill, line: C.orange, bold: true });
  foot(s, "근거: fn_beats_per_cell_rt 유지 / tb_tdc_gpx_cell_pipe_c03_fix PASS");
}

// Slide 5
{
  const s = pptx.addSlide();
  title(s, "검증 결과와 C04 계약", "C03 보완 regression PASS, 다음 Cluster는 metadata 계약을 수락해야 한다");
  box(s, "xvhdl\nPASS", 0.9, 1.4, 1.7, 0.7, { fill: C.greenFill, line: C.green, bold: true });
  arrow(s, 2.65, 1.75, 3.15, 1.75, C.green);
  box(s, "smoke TB\nPASS", 3.2, 1.4, 1.9, 0.7, { fill: C.greenFill, line: C.green, bold: true });
  arrow(s, 5.15, 1.75, 5.65, 1.75, C.green);
  box(s, "C03 fix TB\nPASS", 5.7, 1.4, 2.0, 0.7, { fill: C.greenFill, line: C.green, bold: true });
  arrow(s, 7.75, 1.75, 8.25, 1.75, C.green);
  box(s, "C04\n계약 수락", 8.3, 1.4, 2.1, 0.7, { fill: C.blueFill, line: C.blue, bold: true });

  table(s, [
    ["C04 계약", "내용"],
    ["C03-C04-01", "metadata [6:0] = hit_msb_vec[6:0]"],
    ["C03-C04-02", "Hit[15:0]은 기존 hit slot data beat"],
    ["C03-C04-03", "full hit = Hit[16] & Hit[15:0]"],
    ["C03-C04-04", "32/64/128 output width별 beat count 변경 없음"],
    ["C03-C04-05", "per-slope abort 후 stale hit metadata 잔류 없음"]
  ], 0.9, 2.75, [2.25, 8.85], 0.45, 9.4);
  foot(s, "근거: xsim_c03_cell_pipe_smoke.log:28 / xsim_c03_cell_pipe_fix.log:36");
}

pptx.writeFile({ fileName: out });
