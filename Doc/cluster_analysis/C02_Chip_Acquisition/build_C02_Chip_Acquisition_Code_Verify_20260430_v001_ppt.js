const pptxgen = require("pptxgenjs");
const path = require("path");

const pptx = new pptxgen();
pptx.layout = "LAYOUT_WIDE";
pptx.author = "Codex";
pptx.company = "OpenAI";
pptx.subject = "C02 Chip Acquisition additional verification";
pptx.title = "C02 Chip Acquisition Code Verify v001";
pptx.lang = "ko-KR";
pptx.theme = {
  headFontFace: "Malgun Gothic",
  bodyFontFace: "Malgun Gothic",
  lang: "ko-KR",
};
pptx.defineLayout({ name: "LAYOUT_WIDE", width: 13.333, height: 7.5 });
pptx.margin = 0;

const C = {
  ink: "172033",
  muted: "596579",
  line: "CBD5E1",
  bg: "F8FAFC",
  white: "FFFFFF",
  blue: "2563EB",
  blueSoft: "DBEAFE",
  green: "059669",
  greenSoft: "D1FAE5",
  amber: "D97706",
  amberSoft: "FEF3C7",
  red: "DC2626",
  redSoft: "FEE2E2",
  violet: "7C3AED",
  violetSoft: "EDE9FE",
};

function addHeader(slide, title, sub) {
  slide.background = { color: C.bg };
  slide.addText(title, {
    x: 0.45, y: 0.22, w: 12.45, h: 0.42,
    fontFace: "Malgun Gothic", fontSize: 18, bold: true, color: C.ink,
    margin: 0,
  });
  if (sub) {
    slide.addText(sub, {
      x: 0.46, y: 0.68, w: 12.2, h: 0.28,
      fontFace: "Malgun Gothic", fontSize: 8.5, color: C.muted,
      margin: 0,
    });
  }
  slide.addShape(pptx.ShapeType.line, {
    x: 0.45, y: 1.03, w: 12.42, h: 0,
    line: { color: C.line, width: 0.8 },
  });
}

function box(slide, x, y, w, h, text, fill, line = C.line, opts = {}) {
  slide.addShape(pptx.ShapeType.roundRect, {
    x, y, w, h,
    rectRadius: 0.06,
    fill: { color: fill },
    line: { color: line, width: opts.lineWidth || 1 },
  });
  slide.addText(text, {
    x: x + 0.12, y: y + 0.08, w: w - 0.24, h: h - 0.16,
    fontFace: "Malgun Gothic",
    fontSize: opts.fontSize || 11,
    bold: !!opts.bold,
    color: opts.color || C.ink,
    valign: "mid",
    align: opts.align || "center",
    fit: "shrink",
    margin: 0.02,
    breakLine: false,
  });
}

function label(slide, x, y, w, h, text, opts = {}) {
  slide.addText(text, {
    x, y, w, h,
    fontFace: "Malgun Gothic",
    fontSize: opts.fontSize || 10,
    bold: !!opts.bold,
    color: opts.color || C.ink,
    valign: opts.valign || "top",
    align: opts.align || "left",
    fit: "shrink",
    margin: 0,
    breakLine: false,
  });
}

function arrow(slide, x1, y1, x2, y2, color = C.muted) {
  slide.addShape(pptx.ShapeType.line, {
    x: x1, y: y1, w: x2 - x1, h: y2 - y1,
    line: { color, width: 1.4, beginArrowType: "none", endArrowType: "triangle" },
  });
}

function pill(slide, x, y, w, text, fill, color = C.ink) {
  slide.addShape(pptx.ShapeType.roundRect, {
    x, y, w, h: 0.36,
    rectRadius: 0.16,
    fill: { color: fill },
    line: { color: fill, transparency: 100 },
  });
  slide.addText(text, {
    x: x + 0.08, y: y + 0.075, w: w - 0.16, h: 0.16,
    fontFace: "Malgun Gothic", fontSize: 8.5, bold: true,
    color, align: "center", margin: 0, fit: "shrink",
  });
}

// Slide 1
{
  const s = pptx.addSlide();
  s.background = { color: "EEF2F7" };
  label(s, 0.65, 0.55, 11.8, 0.45, "C02 Chip Acquisition", { fontSize: 18, bold: true, color: C.blue });
  label(s, 0.65, 1.15, 11.7, 0.75, "추가 검증 결과 v001", { fontSize: 32, bold: true, color: C.ink });
  label(s, 0.67, 2.05, 11.2, 0.35, "raw backpressure, stale expected-count, latency / pipeline / II", { fontSize: 13, color: C.muted });
  pill(s, 0.67, 2.72, 1.3, "xsim PASS", C.greenSoft, C.green);
  pill(s, 2.12, 2.72, 2.15, "total_raw_words=256", C.blueSoft, C.blue);
  pill(s, 4.42, 2.72, 2.25, "fault propagation fixed", C.amberSoft, C.amber);
  box(s, 0.72, 4.05, 3.65, 1.1, "검증 중 결함 발견\nEF 안전 종료 시 fault 표시 누락", C.redSoft, "FCA5A5", { fontSize: 14, bold: true, color: C.red });
  box(s, 4.85, 4.05, 3.65, 1.1, "RTL 보완\ncompletion 경로 mismatch 반영", C.greenSoft, "86EFAC", { fontSize: 14, bold: true, color: C.green });
  box(s, 8.98, 4.05, 3.65, 1.1, "재검증 PASS\nempty read 0, tuser clean", C.blueSoft, "93C5FD", { fontSize: 14, bold: true, color: C.blue });
  arrow(s, 4.42, 4.6, 4.78, 4.6, C.muted);
  arrow(s, 8.55, 4.6, 8.91, 4.6, C.muted);
  label(s, 0.7, 6.78, 12.0, 0.26, "기준: Doc/TDC-GPX-Datasheet.pdf + C01 bus timing 계약 / 최종 수정: 2026-04-30 15:58:25 +09:00", { fontSize: 8.5, color: C.muted });
}

// Slide 2
{
  const s = pptx.addSlide();
  addHeader(s, "검증 범위", "C02 내부 drain pipeline에서 직접 확인 가능한 경계를 우선 닫음");
  box(s, 0.7, 1.45, 2.25, 0.78, "GPX IC\nIFIFO / EF", C.white, C.line, { bold: true });
  box(s, 3.35, 1.45, 2.25, 0.78, "C01 bus_phy\nread response", C.white, C.line, { bold: true });
  box(s, 6.0, 1.45, 2.25, 0.78, "C02 chip_run\ndrain decision", C.white, C.line, { bold: true });
  box(s, 8.65, 1.45, 2.25, 0.78, "C02 raw FIFO\nAXI output", C.white, C.line, { bold: true });
  box(s, 11.1, 1.45, 1.5, 0.78, "downstream\nready", C.white, C.line, { bold: true });
  arrow(s, 2.95, 1.84, 3.3, 1.84);
  arrow(s, 5.6, 1.84, 5.95, 1.84);
  arrow(s, 8.25, 1.84, 8.6, 1.84);
  arrow(s, 10.9, 1.84, 11.06, 1.84);

  box(s, 0.7, 3.05, 2.65, 1.0, "VB-C02-03\ncount-known / stale count", C.blueSoft, "93C5FD", { bold: true, fontSize: 11 });
  box(s, 3.65, 3.05, 2.65, 1.0, "VB-C02-05\nlatency / pipeline / II", C.violetSoft, "C4B5FD", { bold: true, fontSize: 11 });
  box(s, 6.6, 3.05, 2.65, 1.0, "VB-C02-06\nbounded backpressure", C.greenSoft, "86EFAC", { bold: true, fontSize: 11 });
  box(s, 9.55, 3.05, 2.65, 1.0, "VB-C02-08\nfault propagation", C.amberSoft, "FCD34D", { bold: true, fontSize: 11 });

  label(s, 0.75, 4.85, 11.9, 0.85, "이번 검증은 C02 내부 positive/bounded 경계를 닫는다. forced negative exit-code, PH_RESP_DRAIN stuck/fatal, config_ctrl/top CDC, downstream 전체 tuser boundary는 별도 보류 항목이다.", { fontSize: 13, color: C.ink });
}

// Slide 3
{
  const s = pptx.addSlide();
  addHeader(s, "발견 결함과 보완", "stale expected-count가 EF로 종료될 때 faulted 표시가 누락되던 경로를 수정");
  box(s, 0.75, 1.45, 2.65, 0.9, "입력\nexpected=4/1\nactual=2/0", C.redSoft, "FCA5A5", { bold: true, color: C.red });
  box(s, 4.0, 1.45, 2.65, 0.9, "EF 우선\nempty read 방지", C.greenSoft, "86EFAC", { bold: true, color: C.green });
  box(s, 7.25, 1.45, 2.65, 0.9, "기존 RTL\nfaulted=0", C.redSoft, "FCA5A5", { bold: true, color: C.red });
  box(s, 10.45, 1.45, 2.1, 0.9, "보완 RTL\nfaulted=1", C.greenSoft, "86EFAC", { bold: true, color: C.green });
  arrow(s, 3.45, 1.9, 3.95, 1.9);
  arrow(s, 6.7, 1.9, 7.2, 1.9);
  arrow(s, 9.95, 1.9, 10.4, 1.9);

  label(s, 0.82, 3.0, 5.9, 0.5, "수정 위치", { fontSize: 14, bold: true });
  box(s, 0.85, 3.55, 5.75, 0.72, "tdc_gpx_chip_run.vhd:524-554\ncompletion 경로에서도 expected-vs-actual mismatch 계산", C.white, C.line, { fontSize: 10.5 });
  box(s, 0.85, 4.45, 5.75, 0.72, "tdc_gpx_chip_run.vhd:633-643\nfallback 경로와 공통 변수 사용", C.white, C.line, { fontSize: 10.5 });

  label(s, 7.1, 3.0, 5.2, 0.5, "검증 증거", { fontSize: 14, bold: true });
  box(s, 7.1, 3.55, 5.15, 0.72, "[17] stale expected count stopped at EF without empty read", C.white, C.line, { fontSize: 10.5 });
  box(s, 7.1, 4.45, 5.15, 0.72, "[17] mismatch propagated via raw tuser fault flag and sticky", C.white, C.line, { fontSize: 10.5 });
}

// Slide 4
{
  const s = pptx.addSlide();
  addHeader(s, "Latency / Pipeline / II", "200 MHz 기준, bounded raw output backpressure 조건 실측");
  const items = [
    ["T0", "IrFlag pin assert", "0 clk"],
    ["T1", "first raw data accepted", "40 clk / 200 ns"],
    ["T2", "chip_run internal complete", "167 clk / 835 ns"],
    ["T3", "output drain_done accepted", "168 clk / 840 ns"],
  ];
  items.forEach((it, idx) => {
    const x = 0.75 + idx * 3.05;
    box(s, x, 1.55, 2.45, 0.9, `${it[0]}\n${it[1]}`, idx === 0 ? C.white : C.blueSoft, idx === 0 ? C.line : "93C5FD", { fontSize: 10.2, bold: true });
    label(s, x + 0.22, 2.62, 2.0, 0.22, it[2], { fontSize: 9.5, align: "center", color: C.muted });
    if (idx < items.length - 1) arrow(s, x + 2.48, 2.0, x + 2.95, 2.0);
  });
  box(s, 0.9, 4.0, 2.55, 0.78, "output hold\n1 clk / 5 ns", C.greenSoft, "86EFAC", { bold: true });
  box(s, 3.85, 4.0, 2.55, 0.78, "II min\n1 clk", C.violetSoft, "C4B5FD", { bold: true });
  box(s, 6.8, 4.0, 2.55, 0.78, "II max\n14 clk", C.violetSoft, "C4B5FD", { bold: true });
  box(s, 9.75, 4.0, 2.55, 0.78, "data words\n20 exact", C.blueSoft, "93C5FD", { bold: true });
  label(s, 0.9, 5.65, 11.6, 0.58, "II_min은 GPX read II가 아니라 raw output FIFO가 tready 해제 후 보유 beat를 연속 handshaking한 출력 측 II다. GPX IC read timing은 Datasheet와 C01 bus_phy 계약이 절대 기준이다.", { fontSize: 11.5, color: C.ink });
}

// Slide 5
{
  const s = pptx.addSlide();
  addHeader(s, "VB Matrix 업데이트", "Plan v004 기준 추가 close 상태");
  const rows = [
    ["VB-C02-03", "강화", "stale expected-count: EF 우선 stop + faulted drain_done"],
    ["VB-C02-05", "강화", "latency / pipeline split / output II cycle 계측"],
    ["VB-C02-06", "부분 close", "bounded raw tready backpressure PASS"],
    ["VB-C02-08", "부분 close", "stale mismatch fault propagation PASS"],
    ["VB-C02-10", "강화", "verify 문서 + xsim 로그 증거 추가"],
  ];
  rows.forEach((r, idx) => {
    const y = 1.35 + idx * 0.82;
    box(s, 0.75, y, 1.45, 0.52, r[0], C.white, C.line, { fontSize: 9.2, bold: true });
    box(s, 2.35, y, 1.25, 0.52, r[1], r[1] === "부분 close" ? C.amberSoft : C.greenSoft, r[1] === "부분 close" ? "FCD34D" : "86EFAC", { fontSize: 9.2, bold: true });
    box(s, 3.75, y, 8.55, 0.52, r[2], C.white, C.line, { fontSize: 9.2, align: "left" });
  });
  label(s, 0.8, 6.05, 11.7, 0.55, "전체 close는 아직 아니다. negative exit-code, PH_RESP_DRAIN fatal, config_ctrl/top CDC, downstream 전체 tuser boundary, illegal timing matrix가 남아 있다.", { fontSize: 11.5, color: C.ink });
}

// Slide 6
{
  const s = pptx.addSlide();
  addHeader(s, "다음 검증 포커스", "C02를 완전히 닫기 위해 남은 경계");
  box(s, 0.8, 1.55, 3.65, 1.0, "Negative TB\nforced empty read / tuser violation\nexit-code evidence", C.redSoft, "FCA5A5", { fontSize: 12, bold: true, color: C.red });
  box(s, 4.85, 1.55, 3.65, 1.0, "PH_RESP_DRAIN\nstuck / fatal / auto-recover\nlong-run evidence", C.amberSoft, "FCD34D", { fontSize: 12, bold: true, color: C.amber });
  box(s, 8.9, 1.55, 3.65, 1.0, "Integration\nconfig_ctrl/top expected CDC\ndownstream tuser boundary", C.blueSoft, "93C5FD", { fontSize: 12, bold: true, color: C.blue });
  box(s, 0.8, 3.35, 5.55, 1.0, "Timing legality matrix\nDatasheet + C01 bus timing 기준으로 illegal 조합 clamp/assertion", C.white, C.line, { fontSize: 12, bold: true });
  box(s, 6.95, 3.35, 5.55, 1.0, "Evidence package\nVB별 PASS / HOLD / FAIL 로그 경로와 문서 추적성 정리", C.white, C.line, { fontSize: 12, bold: true });
  label(s, 0.85, 5.75, 11.6, 0.38, "현재 상태: C02 내부 보강 검증 PASS. 다음 단계는 negative/integration/fatal path를 분리해 닫는 것이 합리적이다.", { fontSize: 12.5, color: C.ink });
}

async function main() {
  const out = path.join(__dirname, "C02_Chip_Acquisition_Code_Verify_20260430_v001.pptx");
  await pptx.writeFile({ fileName: out });
  console.log(out);
}

main().catch((err) => {
  console.error(err);
  process.exit(1);
});
