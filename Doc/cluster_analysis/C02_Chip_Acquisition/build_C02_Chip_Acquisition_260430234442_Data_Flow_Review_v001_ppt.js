const pptxgen = require("pptxgenjs");
const path = require("path");

const pptx = new pptxgen();
pptx.layout = "LAYOUT_WIDE";
pptx.author = "Codex";
pptx.subject = "C02 Chip Acquisition data flow review";
pptx.title = "C02 Data Flow Review v001";
pptx.company = "OpenAI";
pptx.lang = "ko-KR";
pptx.theme = {
  headFontFace: "Malgun Gothic",
  bodyFontFace: "Malgun Gothic",
  lang: "ko-KR",
};
pptx.defineLayout({ name: "LAYOUT_WIDE", width: 13.333, height: 7.5 });

const C = {
  bg: "F8FAFC",
  ink: "172033",
  muted: "64748B",
  line: "CBD5E1",
  white: "FFFFFF",
  blue: "2563EB",
  blueSoft: "DBEAFE",
  teal: "0F766E",
  tealSoft: "CCFBF1",
  green: "059669",
  greenSoft: "D1FAE5",
  amber: "D97706",
  amberSoft: "FEF3C7",
  red: "DC2626",
  redSoft: "FEE2E2",
  violet: "7C3AED",
  violetSoft: "EDE9FE",
  slateSoft: "E2E8F0",
  cyan: "0891B2",
  cyanSoft: "CFFAFE",
};

function header(slide, title, sub) {
  slide.background = { color: C.bg };
  slide.addText(title, {
    x: 0.42, y: 0.22, w: 12.5, h: 0.42,
    fontFace: "Malgun Gothic", fontSize: 18, bold: true,
    color: C.ink, margin: 0, fit: "shrink",
  });
  slide.addText(sub, {
    x: 0.44, y: 0.72, w: 12.2, h: 0.27,
    fontFace: "Malgun Gothic", fontSize: 8.5,
    color: C.muted, margin: 0, fit: "shrink",
  });
  slide.addShape(pptx.ShapeType.line, {
    x: 0.42, y: 1.05, w: 12.5, h: 0,
    line: { color: C.line, width: 0.8 },
  });
}

function text(slide, x, y, w, h, value, opts = {}) {
  slide.addText(value, {
    x, y, w, h,
    fontFace: "Malgun Gothic",
    fontSize: opts.fontSize || 10,
    bold: !!opts.bold,
    color: opts.color || C.ink,
    align: opts.align || "left",
    valign: opts.valign || "top",
    margin: opts.margin === undefined ? 0.04 : opts.margin,
    fit: "shrink",
    breakLine: !!opts.breakLine,
  });
}

function box(slide, x, y, w, h, value, fill, opts = {}) {
  slide.addShape(pptx.ShapeType.roundRect, {
    x, y, w, h,
    rectRadius: 0.04,
    fill: { color: fill },
    line: { color: opts.line || C.line, width: opts.lineWidth || 1 },
  });
  text(slide, x + 0.08, y + 0.06, w - 0.16, h - 0.12, value, {
    fontSize: opts.fontSize || 10,
    bold: opts.bold,
    color: opts.color || C.ink,
    align: opts.align || "center",
    valign: "mid",
    margin: 0.02,
  });
}

function arrow(slide, x1, y1, x2, y2, color = C.muted, width = 1.2) {
  slide.addShape(pptx.ShapeType.line, {
    x: x1, y: y1, w: x2 - x1, h: y2 - y1,
    line: { color, width, beginArrowType: "none", endArrowType: "triangle" },
  });
}

function tableRow(slide, y, cells, widths, fills = []) {
  let x = 0.65;
  cells.forEach((cell, i) => {
    box(slide, x, y, widths[i], 0.43, cell, fills[i] || C.white, {
      fontSize: 8.6,
      bold: y < 1.8,
      align: i === 0 ? "center" : "left",
      line: C.line,
    });
    x += widths[i] + 0.04;
  });
}

function tag(slide, x, y, w, value, fill, line) {
  box(slide, x, y, w, 0.38, value, fill, { line, fontSize: 9.5, bold: true });
}

{
  const s = pptx.addSlide();
  header(s, "C02 데이터 플로우 결론", "측정 데이터 경로와 제어/소유권 경로를 분리해서 다음 Cluster 계약을 판단");

  box(s, 0.75, 1.4, 3.55, 1.0, "측정 데이터\nGPX 28-bit IFIFO word", C.blueSoft, { line: C.blue, fontSize: 14, bold: true });
  box(s, 4.85, 1.4, 3.55, 1.0, "제어/소유권\nfire count + expected count", C.greenSoft, { line: C.green, fontSize: 14, bold: true });
  box(s, 8.95, 1.4, 3.55, 1.0, "품질 정보\nmetadata + status pulse", C.amberSoft, { line: C.amber, fontSize: 14, bold: true });

  text(s, 0.85, 2.95, 11.75, 0.52,
    "핵심 판정: 최종 AXI-stream의 tuser(0)는 SOF이며 fault가 아니다. fault/degraded 판단은 cell metadata, row/frame fault pulse, CSR/status를 함께 봐야 한다.",
    { fontSize: 14, bold: true, align: "center", color: C.ink });

  tableRow(s, 4.05, ["구분", "다음 Cluster가 받아야 할 의미", "주의점"], [2.1, 5.35, 4.65], [C.slateSoft, C.slateSoft, C.slateSoft]);
  tableRow(s, 4.55, ["Stream", "rising / falling 2개 AXIS", "stream 자체가 slope 의미"], [2.1, 5.35, 4.65]);
  tableRow(s, 5.05, ["Boundary", "tuser(0)=SOF, tlast=EOL", "중간 tuser와 의미 다름"], [2.1, 5.35, 4.65]);
  tableRow(s, 5.55, ["Payload", "header + dense cell", "hit slot은 현재 16-bit"], [2.1, 5.35, 4.65]);
  tableRow(s, 6.05, ["Quality", "error_fill, hit_dropped, fault pulse", "final tuser로 해석 금지"], [2.1, 5.35, 4.65]);
}

{
  const s = pptx.addSlide();
  header(s, "측정 데이터 경로", "GPX IC에서 다음 Cluster까지 데이터가 변환되는 순서");

  const xs = [0.55, 2.05, 3.65, 5.2, 6.85, 8.55, 10.1, 11.65];
  const labels = [
    "GPX\nIFIFO",
    "bus_phy\nREAD",
    "chip_ctrl\nraw AXIS",
    "raw_cdc\nSYNC/ASYNC",
    "decode\nI-Mode",
    "raw_event\nTag",
    "cell_pipe\nR/F split",
    "output\nHeader"
  ];
  labels.forEach((label, i) => {
    const fill = [C.blueSoft, C.tealSoft, C.cyanSoft, C.slateSoft, C.violetSoft, C.greenSoft, C.amberSoft, C.redSoft][i];
    const line = [C.blue, C.teal, C.cyan, C.muted, C.violet, C.green, C.amber, C.red][i];
    box(s, xs[i], 1.75, 1.18, 0.82, label, fill, { line, fontSize: 9.6, bold: true });
    if (i < labels.length - 1) arrow(s, xs[i] + 1.18, 2.16, xs[i + 1], 2.16);
  });

  tag(s, 0.55, 3.15, 1.35, "28-bit raw", C.blueSoft, C.blue);
  tag(s, 3.47, 3.15, 1.65, "tdata[27:0]", C.cyanSoft, C.cyan);
  tag(s, 5.05, 3.15, 1.55, "hit/slope/stop", C.violetSoft, C.violet);
  tag(s, 6.75, 3.15, 1.7, "chip/shot/hit_seq", C.greenSoft, C.green);
  tag(s, 8.5, 3.15, 1.45, "dense cell", C.amberSoft, C.amber);
  tag(s, 10.65, 3.15, 1.55, "VDMA stream", C.redSoft, C.red);

  box(s, 0.75, 4.35, 3.55, 0.8, "데이터의 정체성\nraw_hit + stop_id + slope", C.white, { fontSize: 12, bold: true });
  box(s, 4.9, 4.35, 3.55, 0.8, "소유권 추가\nchip_id + shot_seq + hit_seq", C.white, { fontSize: 12, bold: true });
  box(s, 9.05, 4.35, 3.55, 0.8, "소비 형식 변환\ncell/row/frame", C.white, { fontSize: 12, bold: true });

  text(s, 0.85, 6.1, 11.65, 0.35,
    "근거: tdc_gpx_chip_ctrl.vhd, tdc_gpx_decoder_i_mode.vhd, tdc_gpx_raw_event_builder.vhd, tdc_gpx_cell_pipe.vhd, tdc_gpx_output_stage.vhd",
    { fontSize: 9.5, color: C.muted, align: "center" });
}

{
  const s = pptx.addSlide();
  header(s, "제어/소유권 경로", "어느 shot의 데이터를 몇 개 읽을지 결정하는 흐름");

  box(s, 0.75, 1.55, 2.35, 0.9, "laser_ctrl\nfire count", C.greenSoft, { line: C.green, fontSize: 13, bold: true });
  box(s, 0.75, 3.45, 2.35, 0.9, "echo_receiver\nexpected count", C.amberSoft, { line: C.amber, fontSize: 13, bold: true });
  box(s, 4.0, 2.45, 2.45, 0.95, "config_ctrl\nfire-count match", C.blueSoft, { line: C.blue, fontSize: 13, bold: true });
  box(s, 7.25, 2.45, 2.45, 0.95, "chip_run\nIFIFO read count", C.violetSoft, { line: C.violet, fontSize: 13, bold: true });
  box(s, 10.35, 2.45, 2.45, 0.95, "chip_ctrl\nREAD drain", C.cyanSoft, { line: C.cyan, fontSize: 13, bold: true });

  arrow(s, 3.1, 2.0, 4.0, 2.75, C.green);
  arrow(s, 3.1, 3.9, 4.0, 3.1, C.amber);
  arrow(s, 6.45, 2.92, 7.25, 2.92, C.blue);
  arrow(s, 9.7, 2.92, 10.35, 2.92, C.violet);

  text(s, 0.85, 5.0, 11.7, 0.52,
    "이 경로가 맞아야 raw data가 '맞는 shot'의 데이터로 해석된다. zero-stop shot도 final beat로 expected=0을 확정해야 drain 판단이 닫힌다.",
    { fontSize: 13, bold: true, align: "center" });

  tableRow(s, 5.9, ["검토 포인트", "판정"], [3.0, 9.1], [C.slateSoft, C.slateSoft]);
  tableRow(s, 6.4, ["wait counter", "임의 settle 대기보다 fire-count match 기반 expected tuple이 운영상 명확"], [3.0, 9.1]);
}

{
  const s = pptx.addSlide();
  header(s, "Bit / tuser 의미 변화", "중간 stream과 최종 stream의 tuser 의미를 혼동하지 않도록 분리");

  tableRow(s, 1.35, ["단계", "payload", "tuser 의미", "판정"], [2.2, 2.85, 4.1, 3.0], [C.slateSoft, C.slateSoft, C.slateSoft, C.slateSoft]);
  tableRow(s, 1.85, ["GPX IFIFO", "28-bit word", "없음", "Datasheet raw 기준"], [2.2, 2.85, 4.1, 3.0]);
  tableRow(s, 2.35, ["chip_ctrl", "tdata[27:0]", "IFIFO, drain_done, faulted control", "data/control 혼합"], [2.2, 2.85, 4.1, 3.0]);
  tableRow(s, 2.85, ["decoder", "raw_hit[16:0]", "slope, channel, stop, IFIFO", "I-Mode 해석"], [2.2, 2.85, 4.1, 3.0]);
  tableRow(s, 3.35, ["raw_event", "raw_hit[16:0]", "chip, stop, shot_seq, hit_seq", "소유권 tag"], [2.2, 2.85, 4.1, 3.0]);
  tableRow(s, 3.85, ["cell_builder", "dense cell", "tlast에서 faulted", "cell 품질"], [2.2, 2.85, 4.1, 3.0]);
  tableRow(s, 4.35, ["final stream", "header + cell", "tuser(0)=SOF", "fault 아님"], [2.2, 2.85, 4.1, 3.0]);

  box(s, 1.1, 5.55, 3.2, 0.7, "raw hit 17-bit", C.blueSoft, { line: C.blue, fontSize: 13, bold: true });
  arrow(s, 4.3, 5.9, 5.1, 5.9);
  box(s, 5.1, 5.55, 3.2, 0.7, "cell slot 16-bit", C.amberSoft, { line: C.amber, fontSize: 13, bold: true });
  arrow(s, 8.3, 5.9, 9.1, 5.9);
  box(s, 9.1, 5.55, 3.2, 0.7, "17-bit 필요 시\nformat 확장", C.redSoft, { line: C.red, fontSize: 12.5, bold: true });
}

{
  const s = pptx.addSlide();
  header(s, "Cell / Row / Frame 구조", "다음 Cluster가 payload를 파싱하는 단위");

  box(s, 0.75, 1.35, 11.9, 0.55, "Frame, face 단위", C.slateSoft, { fontSize: 14, bold: true });
  arrow(s, 6.7, 1.9, 6.7, 2.35);
  box(s, 1.25, 2.35, 10.9, 0.55, "Row, 한 shot 또는 한 line", C.blueSoft, { line: C.blue, fontSize: 13, bold: true });
  arrow(s, 6.7, 2.9, 6.7, 3.35);
  ["chip0", "chip1", "chip2", "chip3"].forEach((chip, i) => {
    box(s, 1.25 + i * 2.75, 3.35, 2.35, 0.55, chip, C.greenSoft, { line: C.green, fontSize: 12.5, bold: true });
    arrow(s, 2.42 + i * 2.75, 3.9, 2.42 + i * 2.75, 4.35, C.green);
    box(s, 1.25 + i * 2.75, 4.35, 2.35, 0.78, "stop0..7\ncell slice", C.white, { fontSize: 11, bold: true });
  });

  box(s, 0.9, 5.85, 3.65, 0.62, "hit_slot + hit_valid", C.cyanSoft, { line: C.cyan, fontSize: 12, bold: true });
  box(s, 4.85, 5.85, 3.65, 0.62, "slope_vec + hit_count", C.violetSoft, { line: C.violet, fontSize: 12, bold: true });
  box(s, 8.8, 5.85, 3.65, 0.62, "hit_dropped + error_fill", C.amberSoft, { line: C.amber, fontSize: 12, bold: true });
}

{
  const s = pptx.addSlide();
  header(s, "데이터 변형과 위험 지점", "정상 변환과 검출 가능한 손실을 구분");

  tableRow(s, 1.35, ["지점", "현상", "다음 Cluster 판단"], [2.45, 4.4, 5.25], [C.slateSoft, C.slateSoft, C.slateSoft]);
  tableRow(s, 1.85, ["hit width", "17-bit raw -> 16-bit slot", "17-bit 전체 필요 시 format 변경"], [2.45, 4.4, 5.25]);
  tableRow(s, 2.35, ["hit overflow", "max hit 초과 drop", "hit_dropped metadata 확인"], [2.45, 4.4, 5.25]);
  tableRow(s, 2.85, ["IFIFO miss", "synthetic EOS / faulted", "cell fault + row fault 확인"], [2.45, 4.4, 5.25]);
  tableRow(s, 3.35, ["face timeout", "blank cell 생성", "error_fill=1 확인"], [2.45, 4.4, 5.25]);
  tableRow(s, 3.85, ["header", "face_start snapshot", "post-drain error는 status/metadata"], [2.45, 4.4, 5.25]);

  box(s, 0.95, 5.25, 3.45, 0.82, "데이터 값\nhit_slot", C.blueSoft, { line: C.blue, fontSize: 13, bold: true });
  box(s, 4.95, 5.25, 3.45, 0.82, "데이터 품질\nmetadata/status", C.amberSoft, { line: C.amber, fontSize: 13, bold: true });
  box(s, 8.95, 5.25, 3.45, 0.82, "프레임 경계\nSOF/EOL", C.greenSoft, { line: C.green, fontSize: 13, bold: true });
}

{
  const s = pptx.addSlide();
  header(s, "Timing / Latency / Throughput / II", "데이터 플로우 문서에서는 기존 C02 측정값을 의미 기준으로 재배치");

  tableRow(s, 1.35, ["항목", "현재 기준", "데이터 플로우 의미"], [2.6, 3.4, 6.1], [C.slateSoft, C.slateSoft, C.slateSoft]);
  tableRow(s, 1.85, ["GPX READ II", "25 ns 이상", "Datasheet 40 MHz 이하 read legality"], [2.6, 3.4, 6.1]);
  tableRow(s, 2.35, ["raw stream II", "1 beat/clk 가능", "상위 병목은 bus read"], [2.6, 3.4, 6.1]);
  tableRow(s, 2.85, ["first raw", "16 또는 40 clk", "valid와 accepted를 분리해서 해석"], [2.6, 3.4, 6.1]);
  tableRow(s, 3.35, ["output done", "428 clk", "header + cell row 조립 포함"], [2.6, 3.4, 6.1]);
  tableRow(s, 3.85, ["final stream", "rising/falling 독립", "각 stream은 slope별 frame"], [2.6, 3.4, 6.1]);

  box(s, 0.95, 5.25, 3.4, 0.75, "Bus legality\nREAD 간격이 최상위 제약", C.redSoft, { line: C.red, fontSize: 12.5, bold: true });
  box(s, 4.95, 5.25, 3.4, 0.75, "Pipeline\nraw -> cell -> row -> frame", C.blueSoft, { line: C.blue, fontSize: 12.5, bold: true });
  box(s, 8.95, 5.25, 3.4, 0.75, "II 판단\n데이터와 control beat 분리", C.greenSoft, { line: C.green, fontSize: 12.5, bold: true });
}

{
  const s = pptx.addSlide();
  header(s, "다음 Cluster 인계 계약", "데이터가 잘 넘어갔는지 판단하기 위한 수락 조건");

  const items = [
    ["1", "rising/falling 두 stream을 별도 slope frame으로 수락"],
    ["2", "최종 tuser(0)는 SOF로만 해석"],
    ["3", "tlast는 line end, EOL로 해석"],
    ["4", "fault는 cell metadata/status pulse/CSR로 확인"],
    ["5", "header는 face_start snapshot으로 해석"],
    ["6", "17-bit hit 전체 보존 필요 여부를 다음 Cluster에서 결정"],
  ];

  items.forEach((it, idx) => {
    const y = 1.35 + idx * 0.72;
    box(s, 1.0, y, 0.55, 0.46, it[0], C.blueSoft, { line: C.blue, fontSize: 11, bold: true });
    box(s, 1.75, y, 10.55, 0.46, it[1], C.white, { fontSize: 11, bold: true, align: "left" });
  });

  box(s, 1.05, 6.25, 11.25, 0.52, "판정: 데이터 플로우 계약은 조건부 GO. 다음 Cluster는 stream 경계 + cell metadata + status를 함께 검증해야 한다.", C.greenSoft, { line: C.green, fontSize: 12.5, bold: true });
}

async function main() {
  await pptx.writeFile({
    fileName: path.join(__dirname, "C02_Chip_Acquisition_260430234442_Data_Flow_Review_v001.pptx"),
  });
}

main().catch((err) => {
  console.error(err);
  process.exit(1);
});
