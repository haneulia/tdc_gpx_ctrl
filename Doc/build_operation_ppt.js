// =============================================================================
// tdc_gpx_top 운용 개념 + 코드 결함 검토 보강 PPT 생성기
// =============================================================================
// 기존 PPT 두 개(structure / timing-dataflow)가 다루지 않은
// "운용 개념(operation lifecycle)"과 "코드 결함 review"를 채우는 보강판.

const pptxgen = require("pptxgenjs");
const pres = new pptxgen();

pres.layout = "LAYOUT_WIDE"; // 13.3 x 7.5
pres.author = "tdc_gpx_top operational analysis";
pres.title = "TDC-GPX Controller — Operation & Code Review";

// -----------------------------------------------------------------------------
// Color palette (Ocean / Midnight Executive blend)
// -----------------------------------------------------------------------------
const C = {
  navy:     "0F1B3D",
  deep:     "1E2761",
  primary:  "065A82",
  teal:     "1C7293",
  accent:   "F0A500", // amber accent for highlights
  warn:     "C75146", // for code-deficiency callouts
  ok:       "2E8B57", // green
  bg:       "F7F9FC",
  panel:    "FFFFFF",
  light:    "E2E8F0",
  muted:    "64748B",
  text:     "1E293B",
  dark:     "0B1220",
};

const FONT_HEADER = "Malgun Gothic";  // Korean header
const FONT_BODY   = "Malgun Gothic";  // Korean body
const FONT_MONO   = "Consolas";       // for code/registers

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------
function pageHeader(slide, title, kicker) {
  // Top dark bar
  slide.addShape(pres.shapes.RECTANGLE, {
    x: 0, y: 0, w: 13.3, h: 0.85,
    fill: { color: C.navy }, line: { color: C.navy, width: 0 },
  });
  // Vertical accent
  slide.addShape(pres.shapes.RECTANGLE, {
    x: 0, y: 0, w: 0.18, h: 7.5,
    fill: { color: C.accent }, line: { color: C.accent, width: 0 },
  });
  // Kicker
  slide.addText(kicker || "TDC-GPX 운용 분석", {
    x: 0.45, y: 0.10, w: 8, h: 0.30,
    fontSize: 11, fontFace: FONT_BODY, color: "F0A500",
    bold: true, charSpacing: 4, margin: 0,
  });
  // Title
  slide.addText(title, {
    x: 0.45, y: 0.34, w: 12.5, h: 0.55,
    fontSize: 22, fontFace: FONT_HEADER, color: "FFFFFF",
    bold: true, margin: 0,
  });
}

function pageFooter(slide, num, total, note) {
  slide.addShape(pres.shapes.LINE, {
    x: 0.45, y: 7.20, w: 12.45, h: 0,
    line: { color: C.light, width: 0.75 },
  });
  slide.addText(`tdc_gpx_top — Operation & Code Review · ${note || ""}`.trim(), {
    x: 0.45, y: 7.22, w: 9, h: 0.25,
    fontSize: 9, fontFace: FONT_BODY, color: C.muted, margin: 0,
  });
  slide.addText(`${num} / ${total}`, {
    x: 12.0, y: 7.22, w: 0.9, h: 0.25,
    fontSize: 9, fontFace: FONT_BODY, color: C.muted,
    align: "right", margin: 0,
  });
}

function panel(slide, x, y, w, h, fill = C.panel) {
  slide.addShape(pres.shapes.RECTANGLE, {
    x, y, w, h,
    fill: { color: fill }, line: { color: C.light, width: 0.5 },
  });
}

function tagPanel(slide, x, y, w, h, label, color) {
  // Colored vertical bar + light background
  slide.addShape(pres.shapes.RECTANGLE, {
    x, y, w, h,
    fill: { color: C.panel }, line: { color: C.light, width: 0.5 },
  });
  slide.addShape(pres.shapes.RECTANGLE, {
    x, y, w: 0.10, h,
    fill: { color }, line: { color, width: 0 },
  });
  if (label) {
    slide.addText(label, {
      x: x + 0.20, y: y + 0.06, w: w - 0.30, h: 0.30,
      fontSize: 11, bold: true, fontFace: FONT_HEADER,
      color, margin: 0,
    });
  }
}

const TOTAL = 18;
let slideNum = 0;

function bg(slide) {
  slide.background = { color: C.bg };
}

// =============================================================================
// SLIDE 1 — Cover
// =============================================================================
{
  slideNum = 1;
  const s = pres.addSlide();
  s.background = { color: C.navy };

  // Vertical accent stripe
  s.addShape(pres.shapes.RECTANGLE, {
    x: 0, y: 0, w: 0.4, h: 7.5,
    fill: { color: C.accent }, line: { color: C.accent, width: 0 },
  });
  s.addShape(pres.shapes.RECTANGLE, {
    x: 0.4, y: 0, w: 0.05, h: 7.5,
    fill: { color: C.deep }, line: { color: C.deep, width: 0 },
  });

  s.addText("OPERATION & CODE REVIEW", {
    x: 0.9, y: 1.2, w: 11, h: 0.4,
    fontSize: 13, fontFace: FONT_BODY, color: C.accent,
    bold: true, charSpacing: 8, margin: 0,
  });

  s.addText("tdc_gpx_top 운용 개념 보강\n+ 코드 결함 검토 보고서", {
    x: 0.9, y: 1.65, w: 11.5, h: 1.8,
    fontSize: 38, fontFace: FONT_HEADER, color: "FFFFFF",
    bold: true, margin: 0,
  });

  s.addText("기존 PPT(구조 / 타이밍 / 메모리 / 데이터 흐름) 두 편이 다루지 않은\n" +
            "운용 lifecycle · face_seq 시퀀스 · 오류·복구 · 거리 use case · CSR 운용 모델 · SIM 모드\n" +
            "그리고 코드 내부의 부실 항목(죽은 코드, deprecated 포트, 임시 패치, 하드코드)을 정리한다.",
  {
    x: 0.9, y: 3.85, w: 11.5, h: 1.6,
    fontSize: 14, fontFace: FONT_BODY, color: "CADCFC",
    margin: 0, paraSpaceAfter: 6,
  });

  // Sub-area cards on cover
  const cards = [
    { x: 0.9,  t: "Lifecycle",    d: "boot → cfg → start →\nshot → frame → clear" },
    { x: 4.0,  t: "Coordination", d: "face_seq · cmd_arb\n· chip_run · err_handler" },
    { x: 7.1,  t: "Operation",    d: "rise/fall, abort,\nbackpressure, SIM mode" },
    { x: 10.2, t: "Code Review",  d: "dead code, hardcodes,\nR4a hack, known issues" },
  ];
  for (const c of cards) {
    s.addShape(pres.shapes.RECTANGLE, {
      x: c.x, y: 5.7, w: 2.9, h: 1.3,
      fill: { color: C.deep }, line: { color: C.accent, width: 1 },
    });
    s.addText(c.t, {
      x: c.x + 0.15, y: 5.78, w: 2.7, h: 0.4,
      fontSize: 14, bold: true, fontFace: FONT_HEADER,
      color: C.accent, margin: 0,
    });
    s.addText(c.d, {
      x: c.x + 0.15, y: 6.20, w: 2.7, h: 0.85,
      fontSize: 10, fontFace: FONT_BODY, color: "CADCFC",
      margin: 0, paraSpaceAfter: 2,
    });
  }

  s.addText("Source code basis · 2026-04-28 · branch main · grounded by file:line", {
    x: 0.9, y: 7.18, w: 11, h: 0.25,
    fontSize: 9.5, fontFace: FONT_BODY, color: C.muted, margin: 0,
  });
}

// =============================================================================
// SLIDE 2 — Coverage gap (existing vs this PPT)
// =============================================================================
{
  slideNum = 2;
  const s = pres.addSlide();
  bg(s);
  pageHeader(s, "기존 PPT 두 편이 다루는 영역과 본 PPT가 보강하는 영역",
             "Coverage map · 무엇이 비어 있었는가");

  // Two-column comparison header
  s.addText("기존 두 편 (이미 다룸)", {
    x: 0.45, y: 1.05, w: 6.0, h: 0.4,
    fontSize: 14, bold: true, fontFace: FONT_HEADER,
    color: C.muted, margin: 0,
  });
  s.addText("본 보강 PPT (이번에 추가)", {
    x: 6.85, y: 1.05, w: 6.0, h: 0.4,
    fontSize: 14, bold: true, fontFace: FONT_HEADER,
    color: C.primary, margin: 0,
  });

  const coveredRows = [
    "• 4 cluster 구조 + face_seq + status_agg 골격",
    "• Clock domain (i_axis_aclk · i_tdc_clk · s_axi_aclk)",
    "• datasheet bus timing (tS-AD / tPW-RL / tV-DR)",
    "• decode_pipe 정형 (raw word → event → hit_seq)",
    "• cell_builder dual-buffer + max_hits cell layout",
    "• face_assembler row build / blank-fill / strict in-order",
    "• header_inserter 48B prefix · line / frame metadata",
    "• 메모리 인벤토리 (Hit FIFO · IFIFO · cell_builder · FIFO들)",
    "• Latency / throughput 견적 (32b vs 64b)",
    "• Ownership 변환 단계 (chip → AXIS → row → line)",
  ];

  const newRows = [
    "① 시스템 lifecycle (boot → cmd_start → shot → clear)",
    "② chip_init 9-state FSM 운용 흐름",
    "③ face_seq FSM + cmd_start acceptance · cfg snapshot",
    "④ shot 트리거 chain (raw → edge → packet_start)",
    "⑤ rise / fall slope 비대칭 운용 (primary / secondary)",
    "⑥ 거리 use case 별 max_range_clks 윈도우 표",
    "⑦ err_handler 분류·복구 (FSM / debounce / retry)",
    "⑧ STATUS 레지스터 (STAT5 / 6 / 7) clear semantic",
    "⑨ backpressure & abort 전파 경로",
    "⑩ SIM 모드 운용 (motor / echo / laser 협조)",
    "⑪ Code review : dead code / deprecated 포트 / R4a 패치",
    "⑫ Code review : 하드코드 watchdog / known_issues 잔여",
  ];

  // Left covered panel
  panel(s, 0.45, 1.5, 6.0, 5.4, "FFFFFF");
  s.addShape(pres.shapes.RECTANGLE, {
    x: 0.45, y: 1.5, w: 0.10, h: 5.4,
    fill: { color: C.muted }, line: { color: C.muted, width: 0 },
  });
  s.addText(coveredRows.map((t, i) => ({
    text: t, options: { breakLine: i < coveredRows.length - 1, color: C.text }
  })), {
    x: 0.7, y: 1.65, w: 5.7, h: 5.1,
    fontSize: 11, fontFace: FONT_BODY, color: C.text,
    margin: 0, paraSpaceAfter: 6,
  });

  // Right new panel
  panel(s, 6.85, 1.5, 6.0, 5.4, "FFFFFF");
  s.addShape(pres.shapes.RECTANGLE, {
    x: 6.85, y: 1.5, w: 0.10, h: 5.4,
    fill: { color: C.primary }, line: { color: C.primary, width: 0 },
  });
  s.addText(newRows.map((t, i) => ({
    text: t, options: { breakLine: i < newRows.length - 1, color: C.text, bold: i >= 10 }
  })), {
    x: 7.10, y: 1.65, w: 5.7, h: 5.1,
    fontSize: 11, fontFace: FONT_BODY, color: C.text,
    margin: 0, paraSpaceAfter: 5,
  });

  pageFooter(s, slideNum, TOTAL, "Doc/tdc_gpx_module_structure_*.pptx + tdc_gpx_timing_dataflow_*.pptx 와 비교");
}

// =============================================================================
// SLIDE 3 — End-to-end lifecycle map
// =============================================================================
{
  slideNum = 3;
  const s = pres.addSlide();
  bg(s);
  pageHeader(s, "전체 운용 lifecycle 한장 요약",
             "Boot → Configuration → Measurement loop → Frame emit → Clear");

  // Timeline lifecycle
  const stages = [
    { x: 0.45, t: "0. POWER-ON",   d: "i_axis_aresetn 동기화\nxpm_cdc_async_rst → s_tdc_aresetn",
      c: C.muted },
    { x: 2.55, t: "1. CFG WRITE",  d: "CFG_IMAGE[0..15]\nBUS_TIMING\nSTART_OFF1 / CFG_REG7\nSCAN_TIMEOUT",
      c: C.primary },
    { x: 4.65, t: "2. cmd_cfg_write", d: "MAIN_CTRL[31] pulse\n→ cmd_arb dispatch\n→ chip_init x4 (cfg-only)\nbusy 폴링",
      c: C.primary },
    { x: 6.75, t: "3. cmd_start",  d: "MAIN_CTRL[28] pulse\nface_seq geometry validate\nST_IDLE → ST_WAIT_SHOT",
      c: C.teal },
    { x: 8.85, t: "4. SHOT LOOP",  d: "i_shot_start_raw\n→ packet_start (face)\n→ ARMED→CAPTURE→DRAIN→ALU\n→ row → header → VDMA",
      c: C.teal },
    { x: 10.95, t: "5. CLEAR",     d: "frame_done_both\nface_id++ / frame_id++\nerr_soft_clear (sticky)\ncmd_stop / soft_reset",
      c: C.accent },
  ];
  for (const st of stages) {
    s.addShape(pres.shapes.RECTANGLE, {
      x: st.x, y: 1.2, w: 2.0, h: 1.85,
      fill: { color: C.panel }, line: { color: st.c, width: 1.2 },
    });
    s.addShape(pres.shapes.RECTANGLE, {
      x: st.x, y: 1.2, w: 2.0, h: 0.32,
      fill: { color: st.c }, line: { color: st.c, width: 0 },
    });
    s.addText(st.t, {
      x: st.x + 0.08, y: 1.23, w: 1.84, h: 0.28,
      fontSize: 10, bold: true, fontFace: FONT_HEADER,
      color: "FFFFFF", margin: 0,
    });
    s.addText(st.d, {
      x: st.x + 0.10, y: 1.55, w: 1.84, h: 1.45,
      fontSize: 9, fontFace: FONT_BODY, color: C.text,
      margin: 0, paraSpaceAfter: 1,
    });
  }
  // Arrows between stages
  for (let i = 0; i < stages.length - 1; i++) {
    s.addShape(pres.shapes.LINE, {
      x: stages[i].x + 2.0, y: 2.12, w: 0.10, h: 0,
      line: { color: C.muted, width: 1.5, endArrowType: "triangle" },
    });
  }

  // Lifecycle "loop" box for measurement
  s.addShape(pres.shapes.RECTANGLE, {
    x: 0.45, y: 3.4, w: 12.45, h: 1.7,
    fill: { color: "EEF2F7" }, line: { color: C.teal, width: 1 },
  });
  s.addText("MEASUREMENT LOOP — shot 단위 (4단계 반복)", {
    x: 0.65, y: 3.5, w: 12, h: 0.35,
    fontSize: 13, bold: true, fontFace: FONT_HEADER, color: C.deep, margin: 0,
  });
  const loopSteps = [
    { x: 0.65,  t: "(a) shot_start",        d: "edge-detected i_shot_start_raw → s_shot_raw_pulse → packet_start_r" },
    { x: 3.85,  t: "(b) per-chip ARMED",    d: "shot_start_per_chip[i] = shot_start_gated AND face_active_mask(i)" },
    { x: 7.05,  t: "(c) CAPTURE → DRAIN",   d: "range_active 동안 stop hit accept → IrFlag → IFIFO drain → ALU pulse" },
    { x: 10.25, t: "(d) row build / emit",  d: "cell_builder ping-pong → face_assembler in-order → header → VDMA" },
  ];
  for (const ls of loopSteps) {
    s.addText(ls.t, {
      x: ls.x, y: 3.95, w: 3.1, h: 0.3,
      fontSize: 11, bold: true, fontFace: FONT_HEADER, color: C.primary, margin: 0,
    });
    s.addText(ls.d, {
      x: ls.x, y: 4.25, w: 3.1, h: 0.78,
      fontSize: 9.5, fontFace: FONT_BODY, color: C.text, margin: 0,
    });
  }

  // Bottom note: error/recovery branch
  s.addShape(pres.shapes.RECTANGLE, {
    x: 0.45, y: 5.4, w: 6.1, h: 1.65,
    fill: { color: C.panel }, line: { color: C.warn, width: 1 },
  });
  s.addText("오류 분기 — err_handler 자동 복구", {
    x: 0.6, y: 5.48, w: 5.9, h: 0.32,
    fontSize: 12, bold: true, fontFace: FONT_HEADER, color: C.warn, margin: 0,
  });
  s.addText("ErrFlag(debounce) → Reg12 read → 분류 → 칩 soft_reset →\n" +
            "frame 경계 동기 → 정상 복귀.\n" +
            "g_MAX_RETRIES(3) 초과 시 err_fatal sticky → SW가 err_soft_clear.",
  {
    x: 0.6, y: 5.82, w: 5.9, h: 1.18,
    fontSize: 10, fontFace: FONT_BODY, color: C.text, margin: 0, paraSpaceAfter: 3,
  });

  s.addShape(pres.shapes.RECTANGLE, {
    x: 6.80, y: 5.4, w: 6.1, h: 1.65,
    fill: { color: C.panel }, line: { color: C.accent, width: 1 },
  });
  s.addText("정지 / 재구성 분기 — SW 명령", {
    x: 6.95, y: 5.48, w: 5.9, h: 0.32,
    fontSize: 12, bold: true, fontFace: FONT_HEADER, color: C.accent, margin: 0,
  });
  s.addText("cmd_stop : ARMED·IN_FACE에서 즉시 ST_IDLE, pending start 폐기.\n" +
            "cmd_soft_reset : 모든 FSM IDLE 복귀, cfg는 유지.\n" +
            "force_reinit (CTL2[0]) : PH_RESP_DRAIN quarantine 탈출 (단발).",
  {
    x: 6.95, y: 5.82, w: 5.9, h: 1.18,
    fontSize: 10, fontFace: FONT_BODY, color: C.text, margin: 0, paraSpaceAfter: 3,
  });

  pageFooter(s, slideNum, TOTAL, "register_map.md · face_seq.vhd · err_handler.vhd · chip_init.vhd");
}

// =============================================================================
// SLIDE 4 — SW boot / configuration sequence (CSR-level operation)
// =============================================================================
{
  slideNum = 4;
  const s = pres.addSlide();
  bg(s);
  pageHeader(s, "SW 부팅·구성 운용 시퀀스 — register_map 기준",
             "단계별 CSR write · 폴링 조건 · 검증 절차");

  // Step diagram
  const x0 = 0.45, top = 1.1, w = 6.4;
  panel(s, x0, top, w, 5.95);
  s.addText("정상 부팅 시퀀스 (csr_pipeline + csr_chip)", {
    x: x0 + 0.18, y: top + 0.10, w: w - 0.3, h: 0.32,
    fontSize: 13, bold: true, fontFace: FONT_HEADER, color: C.deep, margin: 0,
  });
  const seqRows = [
    ["1", "CFG_IMAGE[0..15] write", "csr_chip 0x14 ~ 0x50, 16 × 32b TDC 칩 레지스터 미러"],
    ["2", "BUS_TIMING write",        "csr_chip 0x04 — bus_clk_div / bus_ticks / reg_target"],
    ["3", "START_OFF1 / CFG_REG7",   "0x0C, 0x10 — 칩-side 상수 보정"],
    ["4", "SCAN_TIMEOUT write",      "0x54 — max_scan_clks · max_hits_cfg"],
    ["5", "MAIN_CTRL[31]=cmd_cfg_write", "csr_pipeline 0x00, edge-detected 1-cycle pulse"],
    ["6", "STATUS.busy 폴링",        "0x54[0] = 0 까지 대기 (chip_init 9-state 완료)"],
    ["7", "RANGE_COLS write",        "csr_pipeline 0x04 — max_range_clks / cols_per_face"],
    ["8", "MAIN_CTRL[28]=cmd_start", "csr_pipeline 0x00 — face_seq 시작"],
  ];
  let rowY = top + 0.55;
  for (const [n, name, desc] of seqRows) {
    s.addShape(pres.shapes.OVAL, {
      x: x0 + 0.20, y: rowY, w: 0.45, h: 0.45,
      fill: { color: C.primary }, line: { color: C.primary, width: 0 },
    });
    s.addText(n, {
      x: x0 + 0.20, y: rowY + 0.06, w: 0.45, h: 0.32,
      fontSize: 13, bold: true, fontFace: FONT_HEADER,
      color: "FFFFFF", align: "center", margin: 0,
    });
    s.addText(name, {
      x: x0 + 0.78, y: rowY - 0.02, w: w - 1.0, h: 0.28,
      fontSize: 11, bold: true, fontFace: FONT_HEADER,
      color: C.text, margin: 0,
    });
    s.addText(desc, {
      x: x0 + 0.78, y: rowY + 0.24, w: w - 1.0, h: 0.32,
      fontSize: 9.5, fontFace: FONT_BODY, color: C.muted, margin: 0,
    });
    rowY += 0.62;
  }

  // Right side : critical caveats
  const xR = 7.05, wR = 5.85;
  panel(s, xR, top, wR, 2.85);
  s.addText("⚠ SW 운용 contract — 이 약속이 깨지면 silent fail", {
    x: xR + 0.15, y: top + 0.10, w: wR - 0.30, h: 0.32,
    fontSize: 12.5, bold: true, fontFace: FONT_HEADER, color: C.warn, margin: 0,
  });
  s.addText([
    { text: "(1) CFG / CFG_IMAGE 는 2-FF 동기 + ASYNC_REG 만 사용 — handshake 아님.\n", options: {} },
    { text: "    bundle 원자성은 SW가 ‘값 안정 → ≥4 cycle 대기 → 명령 pulse’ 순서로 보장.\n", options: { color: C.muted } },
    { text: "(2) reg-access bundle (cmd_reg_addr+wdata) 만 xpm_cdc_handshake.\n", options: {} },
    { text: "(3) 명령 pulse는 edge-detect — level 유지하면 1 회만 trigger.\n", options: {} },
    { text: "(4) cmd_cfg_write는 cmd_arb queue 1-depth — busy 중 추가요청은\n", options: {} },
    { text: "    o_cfg_write_coalesced sticky로 묻힘 (chip_init L72).", options: { color: C.muted } },
  ], {
    x: xR + 0.15, y: top + 0.45, w: wR - 0.30, h: 2.30,
    fontSize: 10, fontFace: FONT_BODY, color: C.text, margin: 0, paraSpaceAfter: 1,
  });

  // Right bottom: power-up state changes
  panel(s, xR, top + 3.0, wR, 2.95);
  s.addText("chip_init 가 칩에 강제하는 상태 (9-state per chip)", {
    x: xR + 0.15, y: top + 3.10, w: wR - 0.30, h: 0.32,
    fontSize: 12.5, bold: true, fontFace: FONT_HEADER, color: C.deep, margin: 0,
  });
  const fsmStates = [
    "ST_POWERUP        — puresn = '0', stopdis = '0', g_POWERUP_CLKS(48) 대기",
    "ST_PU_RELEASE     — puresn = '1' 출력 (전력 안정 표시)",
    "ST_STOPDIS_HIGH   — stopdis = '1' (master reset 동안 stop 차단)",
    "ST_CFG_WRITE      — Reg0..Reg15 11회 burst write (Reg14[4]=0 강제)",
    "ST_CFG_WR_WAIT    — 16-bit watchdog (응답 timeout시 stopdis = '0' 안전)",
    "ST_MASTER_RESET   — Reg4[22]=1 master reset pulse",
    "ST_MR_WAIT        — 16-bit watchdog",
    "ST_RECOVERY       — g_RECOVERY_CLKS(8) 대기",
    "ST_STOPDIS_LOW    — stopdis = '0', o_done 1-cycle pulse",
  ];
  s.addText(fsmStates.map((t, i) => ({
    text: t, options: { breakLine: i < fsmStates.length - 1 }
  })), {
    x: xR + 0.18, y: top + 3.45, w: wR - 0.36, h: 2.45,
    fontSize: 9, fontFace: FONT_MONO, color: C.text, margin: 0, paraSpaceAfter: 1,
  });

  pageFooter(s, slideNum, TOTAL, "register_map.md L156–164 · tdc_gpx_chip_init.vhd L1–94");
}

// =============================================================================
// SLIDE 5 — chip_init / cmd_arb dispatch operation
// =============================================================================
{
  slideNum = 5;
  const s = pres.addSlide();
  bg(s);
  pageHeader(s, "다중 칩 명령 조정 — cmd_arb · chip_init · 동시성",
             "1 SW pulse → 4 chip parallel dispatch · 충돌 회피 · queue depth=1");

  // cmd_arb diagram
  panel(s, 0.45, 1.1, 12.45, 2.85);
  s.addText("cmd_arb : SW 한 번 누르면 4 칩 모두에게 동시에 보낸다", {
    x: 0.6, y: 1.2, w: 12, h: 0.32,
    fontSize: 13, bold: true, fontFace: FONT_HEADER, color: C.deep, margin: 0,
  });

  // Source: csr_pipeline / csr_chip
  s.addShape(pres.shapes.RECTANGLE, {
    x: 0.65, y: 1.7, w: 2.4, h: 1.95,
    fill: { color: "EEF2F7" }, line: { color: C.muted, width: 0.8 },
  });
  s.addText("csr_pipeline\ncsr_chip", {
    x: 0.65, y: 1.78, w: 2.4, h: 0.5,
    fontSize: 12, bold: true, fontFace: FONT_HEADER, color: C.deep, align: "center", margin: 0,
  });
  s.addText([
    { text: "cmd_start, cmd_stop\n", options: {} },
    { text: "cmd_soft_reset\n", options: {} },
    { text: "cmd_cfg_write\n", options: {} },
    { text: "reg_read / write\n", options: {} },
    { text: "force_reinit\nerr_soft_clear", options: {} },
  ], {
    x: 0.7, y: 2.30, w: 2.3, h: 1.30,
    fontSize: 9.5, fontFace: FONT_MONO, color: C.text, align: "center", margin: 0, paraSpaceAfter: 0,
  });

  // Arrow
  s.addShape(pres.shapes.LINE, {
    x: 3.10, y: 2.65, w: 0.5, h: 0,
    line: { color: C.primary, width: 2.5, endArrowType: "triangle" },
  });

  // cmd_arb central
  s.addShape(pres.shapes.RECTANGLE, {
    x: 3.65, y: 1.7, w: 3.2, h: 1.95,
    fill: { color: C.primary }, line: { color: C.primary, width: 0 },
  });
  s.addText("cmd_arb", {
    x: 3.65, y: 1.78, w: 3.2, h: 0.4,
    fontSize: 14, bold: true, fontFace: FONT_HEADER, color: "FFFFFF", align: "center", margin: 0,
  });
  s.addText([
    "• cmd_start → 4-chip parallel dispatch",
    "• cmd_cfg_write : queue depth=1 / chip",
    "• reg-access overlap → reg_rejected",
    "• mask=0000 시 reg_zero_mask sticky",
    "• reg_timeout_mask (STAT7[3:0])",
    "• done_pulse → IRQ (1-clk)",
  ].map((t, i, a) => ({ text: t, options: { breakLine: i < a.length - 1 } })), {
    x: 3.75, y: 2.20, w: 3.0, h: 1.40,
    fontSize: 9, fontFace: FONT_BODY, color: "FFFFFF", margin: 0, paraSpaceAfter: 0,
  });

  // Arrow
  s.addShape(pres.shapes.LINE, {
    x: 6.90, y: 2.65, w: 0.5, h: 0,
    line: { color: C.primary, width: 2.5, endArrowType: "triangle" },
  });

  // 4 chip box
  s.addShape(pres.shapes.RECTANGLE, {
    x: 7.45, y: 1.7, w: 5.40, h: 1.95,
    fill: { color: C.panel }, line: { color: C.teal, width: 0.8 },
  });
  s.addText("chip_init / chip_run / chip_reg / err_handler  ×4", {
    x: 7.45, y: 1.78, w: 5.40, h: 0.32,
    fontSize: 11, bold: true, fontFace: FONT_HEADER, color: C.teal, align: "center", margin: 0,
  });
  // 4 chip cells
  const chipColors = [C.teal, C.teal, C.teal, C.teal];
  for (let i = 0; i < 4; i++) {
    const cx = 7.55 + i * 1.32;
    s.addShape(pres.shapes.RECTANGLE, {
      x: cx, y: 2.2, w: 1.20, h: 1.35,
      fill: { color: "EEF2F7" }, line: { color: C.teal, width: 0.6 },
    });
    s.addText(`chip ${i}`, {
      x: cx, y: 2.24, w: 1.20, h: 0.30,
      fontSize: 10, bold: true, fontFace: FONT_HEADER,
      color: C.deep, align: "center", margin: 0,
    });
    s.addText("init\nrun\nreg\nerr", {
      x: cx, y: 2.55, w: 1.20, h: 1.0,
      fontSize: 9.5, fontFace: FONT_MONO, color: C.muted,
      align: "center", margin: 0, paraSpaceAfter: 0,
    });
  }

  // Bottom : details (parallelism / safety)
  panel(s, 0.45, 4.10, 6.10, 2.95);
  s.addText("동시 실행이 보장되는 항목 / 안 되는 항목", {
    x: 0.60, y: 4.18, w: 5.85, h: 0.32,
    fontSize: 12.5, bold: true, fontFace: FONT_HEADER, color: C.deep, margin: 0,
  });
  s.addText([
    { text: "● cmd_start / cmd_stop / cmd_soft_reset → 4 칩 동시 dispatch.\n", options: { color: C.ok } },
    { text: "● cmd_cfg_write → mask 비트로 동시 dispatch (queue depth=1/칩).\n", options: { color: C.ok } },
    { text: "● init / err / cfg_write 충돌 → busy 상태 우선, 늦게 온 cfg_write는\n  새 snapshot으로 coalesce, sticky o_cfg_write_coalesced 통보.\n", options: { color: C.text } },
    { text: "✗ reg-access (read/write) overlap → cmd_arb 거부, reg_rejected sticky.\n", options: { color: C.warn } },
    { text: "✗ active_chip_mask = 0000 reg request → reg_zero_mask sticky.\n", options: { color: C.warn } },
    { text: "✗ chip_init 진행 중 cmd_start → face_seq 가 reject + cfg_rejected.", options: { color: C.warn } },
  ], {
    x: 0.60, y: 4.55, w: 5.85, h: 2.45,
    fontSize: 10, fontFace: FONT_BODY, color: C.text, margin: 0, paraSpaceAfter: 2,
  });

  panel(s, 6.80, 4.10, 6.10, 2.95);
  s.addText("관측 — STAT5 / STAT6 / STAT7 으로 보는 dispatch 결과", {
    x: 6.95, y: 4.18, w: 5.85, h: 0.32,
    fontSize: 12.5, bold: true, fontFace: FONT_HEADER, color: C.deep, margin: 0,
  });
  s.addTable([
    [
      { text: "신호", options: { bold: true, fill: { color: C.deep }, color: "FFFFFF" } },
      { text: "STAT 비트", options: { bold: true, fill: { color: C.deep }, color: "FFFFFF" } },
      { text: "의미", options: { bold: true, fill: { color: C.deep }, color: "FFFFFF" } },
    ],
    ["STATUS.busy", "STAT5[0]", "어떤 FSM이라도 비-IDLE"],
    ["chip_error_mask", "STAT5[7:4]", "라이브 칩 에러 (level)"],
    ["err_fatal", "STAT5[2]", "retry exhausted (sticky)"],
    ["reg_rejected / reg_zero_mask", "STAT6[1] / [2]", "cmd_arb 거부 (sticky)"],
    ["err_reg_overflow_mask", "STAT6[27:24]", "chip_reg 3rd-pulse queue overflow"],
    ["reg_timeout_mask", "STAT7[3:0]", "per-chip reg transaction timeout"],
    ["run_timeout_cause_last", "STAT7[10:8]", "last drain timeout 원인 (3-bit)"],
  ], {
    x: 6.92, y: 4.58, w: 5.92, h: 2.40,
    colW: [1.95, 1.45, 2.52],
    fontSize: 9, fontFace: FONT_BODY, color: C.text,
    border: { pt: 0.5, color: C.light },
  });

  pageFooter(s, slideNum, TOTAL, "tdc_gpx_cmd_arb.vhd · register_map.md STAT5/6/7 · known_issues.md Cat C");
}

// =============================================================================
// SLIDE 6 — face_seq FSM operation
// =============================================================================
{
  slideNum = 6;
  const s = pres.addSlide();
  bg(s);
  pageHeader(s, "face_seq 운용 — cmd_start acceptance · face/frame ID · cfg snapshot",
             "ST_IDLE → ST_WAIT_SHOT → ST_IN_FACE 의 의미와 전이 조건");

  // FSM diagram
  panel(s, 0.45, 1.1, 7.0, 3.55);
  s.addText("face_seq FSM (3-state) + 핵심 entry / exit 조건", {
    x: 0.60, y: 1.18, w: 6.85, h: 0.32,
    fontSize: 12.5, bold: true, fontFace: FONT_HEADER, color: C.deep, margin: 0,
  });

  // 3-state nodes
  const states = [
    { x: 0.85, y: 2.35, t: "ST_IDLE", c: C.muted },
    { x: 3.15, y: 2.35, t: "ST_WAIT_SHOT", c: C.primary },
    { x: 5.45, y: 2.35, t: "ST_IN_FACE", c: C.teal },
  ];
  for (const st of states) {
    s.addShape(pres.shapes.ROUNDED_RECTANGLE, {
      x: st.x, y: st.y, w: 1.85, h: 0.95,
      fill: { color: st.c }, line: { color: st.c, width: 0 },
      rectRadius: 0.12,
    });
    s.addText(st.t, {
      x: st.x, y: st.y + 0.32, w: 1.85, h: 0.35,
      fontSize: 12, bold: true, fontFace: FONT_HEADER,
      color: "FFFFFF", align: "center", margin: 0,
    });
  }
  // Arrows
  s.addShape(pres.shapes.LINE, {
    x: 2.70, y: 2.83, w: 0.45, h: 0,
    line: { color: C.text, width: 2, endArrowType: "triangle" },
  });
  s.addShape(pres.shapes.LINE, {
    x: 5.00, y: 2.83, w: 0.45, h: 0,
    line: { color: C.text, width: 2, endArrowType: "triangle" },
  });
  // Loop arrows (down)
  s.addText("cmd_start ∧ valid_cfg ∧ pipeline_idle\n→ cmd_start_accepted", {
    x: 2.45, y: 1.85, w: 1.6, h: 0.45,
    fontSize: 8.5, fontFace: FONT_MONO, color: C.text, align: "center", margin: 0,
  });
  s.addText("packet_start_r\n(shot edge captured)", {
    x: 4.75, y: 1.85, w: 1.6, h: 0.45,
    fontSize: 8.5, fontFace: FONT_MONO, color: C.text, align: "center", margin: 0,
  });
  // Down loop labels
  s.addText("frame_done_both → face_id++ / frame_id++\ncmd_stop / abort → ST_IDLE", {
    x: 2.5, y: 3.45, w: 4.6, h: 0.55,
    fontSize: 9, fontFace: FONT_MONO, color: C.warn, align: "center", margin: 0,
  });
  s.addShape(pres.shapes.LINE, {
    x: 6.30, y: 3.40, w: 0, h: 0.55,
    line: { color: C.warn, width: 1.5 },
  });
  s.addShape(pres.shapes.LINE, {
    x: 1.85, y: 3.95, w: 4.45, h: 0,
    line: { color: C.warn, width: 1.5, endArrowType: "triangle" },
  });
  s.addShape(pres.shapes.LINE, {
    x: 1.85, y: 3.40, w: 0, h: 0.55,
    line: { color: C.warn, width: 1.5 },
  });

  // Validation rules box
  panel(s, 7.65, 1.1, 5.25, 3.55);
  s.addText("cmd_start acceptance 검증 규칙 (face_seq.vhd:222–245)", {
    x: 7.78, y: 1.18, w: 5.05, h: 0.32,
    fontSize: 12, bold: true, fontFace: FONT_HEADER, color: C.deep, margin: 0,
  });
  s.addText([
    { text: "Geometry validation (config 거부 → cfg_rejected pulse)\n", options: { bold: true, color: C.warn } },
    { text: "  • active_chip_mask = 0000        → reject\n", options: { color: C.text } },
    { text: "  • stops_per_chip < 2  or  > 8    → reject\n", options: { color: C.text } },
    { text: "  • cols_per_face < 1               → reject\n", options: { color: C.text } },
    { text: "  • n_faces = 0                      → reject\n", options: { color: C.text } },
    { text: "Pipeline idle 검사 (10조건 AND)\n", options: { bold: true, color: C.primary } },
    { text: "  chip_busy=0  reg_outstanding=0\n", options: { color: C.muted } },
    { text: "  face_asm_idle (rise+fall)\n", options: { color: C.muted } },
    { text: "  hdr_idle (rise+fall)\n", options: { color: C.muted } },
    { text: "  face_*tvalid = 0  m_axis_*tvalid = 0\n", options: { color: C.muted } },
    { text: "Pending latch (R5)\n", options: { bold: true, color: C.ok } },
    { text: "  busy 시 pulse를 latch → 매 IDLE 사이클 재시도.\n  cmd_stop / abort 가 오면 폐기 (silent loss 방지).", options: { color: C.text } },
  ], {
    x: 7.78, y: 1.55, w: 5.05, h: 3.05,
    fontSize: 9.2, fontFace: FONT_BODY, color: C.text, margin: 0, paraSpaceAfter: 0,
  });

  // Bottom : config snapshot mechanism
  panel(s, 0.45, 4.85, 12.45, 2.20);
  s.addText("config snapshot — 'face 경계에서만 구성 freeze'", {
    x: 0.60, y: 4.93, w: 12.2, h: 0.32,
    fontSize: 13, bold: true, fontFace: FONT_HEADER, color: C.deep, margin: 0,
  });
  // 3 columns about snapshot
  const cols = [
    {
      x: 0.60, t: "i_cfg (live)",
      d: "csr_pipeline + csr_chip 가 매 cycle 반영하는 live 구성. SW가 측정 중에 임의로 바꿔도 진행 중인 frame은 영향을 안 받는다."
    },
    {
      x: 4.80, t: "s_cfg_face_r (snapshot)",
      d: "packet_start_r 1-clk pulse 시점에 i_cfg 전체를 한번 latch. ST_IN_FACE 동안 header_inserter / output_stage 가 이걸 사용."
    },
    {
      x: 9.00, t: "t_cfg_image (per-chip mirror)",
      d: "TDC 칩 Reg0..15 의 SW shadow. cmd_cfg_write 시 chip_init 가 이 값을 11회 burst write 로 칩에 기록."
    },
  ];
  for (const c of cols) {
    s.addShape(pres.shapes.RECTANGLE, {
      x: c.x, y: 5.30, w: 4.0, h: 1.65,
      fill: { color: "EEF2F7" }, line: { color: C.primary, width: 0.6 },
    });
    s.addText(c.t, {
      x: c.x + 0.10, y: 5.36, w: 3.85, h: 0.32,
      fontSize: 11, bold: true, fontFace: FONT_HEADER, color: C.primary, margin: 0,
    });
    s.addText(c.d, {
      x: c.x + 0.10, y: 5.70, w: 3.85, h: 1.20,
      fontSize: 9.5, fontFace: FONT_BODY, color: C.text, margin: 0,
    });
  }

  pageFooter(s, slideNum, TOTAL, "tdc_gpx_face_seq.vhd:185–377 · known_issues.md (clamp vs reject 정책)");
}

// =============================================================================
// SLIDE 7 — Shot trigger chain & rise/fall slope asymmetry
// =============================================================================
{
  slideNum = 7;
  const s = pres.addSlide();
  bg(s);
  pageHeader(s, "Shot 트리거 chain · rise / fall 비대칭 운용",
             "raw level → edge pulse → packet_start → per-chip gate → primary/secondary slope");

  // Top : shot trigger chain
  panel(s, 0.45, 1.1, 12.45, 2.40);
  s.addText("Shot 트리거 — i_shot_start_raw 가 4 칩에 도달하는 경로", {
    x: 0.60, y: 1.18, w: 12.2, h: 0.32,
    fontSize: 13, bold: true, fontFace: FONT_HEADER, color: C.deep, margin: 0,
  });

  // 5-stage chain with details
  const chain = [
    { x: 0.60, t: "laser_ctrl",   sub: "i_shot_start (1-clk pulse)",                c: C.muted },
    { x: 3.10, t: "edge detect",  sub: "p_shot_raw_edge → s_shot_raw_pulse",        c: C.primary },
    { x: 5.60, t: "packet_start", sub: "ST_WAIT_SHOT + idle conditions\n→ s_packet_start_r (registered, 1-cycle, glitch-free)", c: C.primary },
    { x: 8.40, t: "shot_pending → gated", sub: "s_shot_pending_r → s_shot_start_gated\n(closing/abort/stop kill)",              c: C.teal },
    { x: 11.10, t: "per-chip gate", sub: "shot_start_per_chip[i] = gated AND\nface_active_mask[i]",                              c: C.teal },
  ];
  for (const cn of chain) {
    s.addShape(pres.shapes.RECTANGLE, {
      x: cn.x, y: 1.65, w: 2.30, h: 1.65,
      fill: { color: C.panel }, line: { color: cn.c, width: 1 },
    });
    s.addShape(pres.shapes.RECTANGLE, {
      x: cn.x, y: 1.65, w: 2.30, h: 0.32,
      fill: { color: cn.c }, line: { color: cn.c, width: 0 },
    });
    s.addText(cn.t, {
      x: cn.x + 0.05, y: 1.68, w: 2.20, h: 0.28,
      fontSize: 10.5, bold: true, fontFace: FONT_HEADER,
      color: "FFFFFF", margin: 0, align: "center",
    });
    s.addText(cn.sub, {
      x: cn.x + 0.10, y: 2.05, w: 2.10, h: 1.20,
      fontSize: 9, fontFace: FONT_MONO, color: C.text, margin: 0,
    });
  }
  // Arrows
  for (let i = 0; i < chain.length - 1; i++) {
    s.addShape(pres.shapes.LINE, {
      x: chain[i].x + 2.30, y: 2.45, w: 0.10, h: 0,
      line: { color: C.muted, width: 1.5, endArrowType: "triangle" },
    });
  }

  // Bottom : rise/fall slope asymmetry
  panel(s, 0.45, 3.7, 6.10, 3.40);
  s.addText("Rise vs Fall slope — 운용상 비대칭 정책", {
    x: 0.60, y: 3.78, w: 5.95, h: 0.32,
    fontSize: 13, bold: true, fontFace: FONT_HEADER, color: C.deep, margin: 0,
  });
  s.addText([
    { text: "Rise (PRIMARY)\n", options: { bold: true, color: C.primary } },
    { text: "  • Stop pulse count + timing 의 source-of-truth\n  • 자기 자신만 valid해도 의미 있음 (단독 채널)\n  • Rise abort 는 fall 까지 같이 죽임\n\n", options: { color: C.text } },
    { text: "Fall (SECONDARY)\n", options: { bold: true, color: C.teal } },
    { text: "  • Pulse-width verification 보조 채널 (선택적)\n  • Fall-only abort 는 Rise를 죽이지 않음\n  • frame_done_both 만 fall 완료를 기다림\n\n", options: { color: C.text } },
    { text: "frame_abort_count\n", options: { bold: true, color: C.warn } },
    { text: "  → 어느 쪽 abort든 +1 (R9 #14)\n  → Rise/Fall 둘 다의 stream loss 가시화", options: { color: C.text } },
  ], {
    x: 0.60, y: 4.15, w: 5.85, h: 2.85,
    fontSize: 10.5, fontFace: FONT_BODY, color: C.text, margin: 0, paraSpaceAfter: 1,
  });

  // Right : abort propagation
  panel(s, 6.80, 3.7, 6.10, 3.40);
  s.addText("Abort 전파 정책 (face_seq.vhd:523–530)", {
    x: 6.95, y: 3.78, w: 5.95, h: 0.32,
    fontSize: 13, bold: true, fontFace: FONT_HEADER, color: C.deep, margin: 0,
  });

  s.addTable([
    [
      { text: "trigger", options: { bold: true, fill: { color: C.deep }, color: "FFFFFF" } },
      { text: "abort_rise", options: { bold: true, fill: { color: C.deep }, color: "FFFFFF" } },
      { text: "abort_fall", options: { bold: true, fill: { color: C.deep }, color: "FFFFFF" } },
    ],
    ["i_face_abort (rise overrun)",      "ON",  "ON"],
    ["i_face_fall_abort (fall overrun)", "—",   "ON"],
    ["i_cmd_stop",                       "ON",  "ON"],
    ["i_cmd_soft_reset",                 "ON",  "ON"],
    ["VDMA backpressure (tready=0)",     "—",   "—"],
  ], {
    x: 6.92, y: 4.18, w: 5.92, h: 1.85,
    colW: [3.05, 1.43, 1.44],
    fontSize: 9.5, fontFace: FONT_BODY, color: C.text,
    border: { pt: 0.5, color: C.light },
  });

  s.addText([
    { text: "shot_start_gated / face_start_gated → rise-only 로 gating.\n", options: { color: C.text } },
    { text: "이 비대칭은 #22 Sprint 3 의 의도된 정책 (known_issues.md Cat B-4).\n", options: { color: C.muted } },
    { text: "Reviewer 가 한쪽만 보면 비대칭처럼 보이지만, fall-only abort 는\nrise VDMA stream 을 그대로 살려둔다.", options: { color: C.muted } },
  ], {
    x: 6.95, y: 6.10, w: 5.92, h: 0.95,
    fontSize: 9.5, fontFace: FONT_BODY, color: C.text, margin: 0, paraSpaceAfter: 1,
  });

  pageFooter(s, slideNum, TOTAL, "face_seq.vhd:469–650 · known_issues.md Cat B-4");
}

// =============================================================================
// SLIDE 8 — distance use cases (operational table)
// =============================================================================
{
  slideNum = 8;
  const s = pres.addSlide();
  bg(s);
  pageHeader(s, "거리 use case 별 운용 — max_range_clks 윈도우 표",
             "100 / 250 / 500 / 750 / 1000 m 측정 거리 ↔ chip_run drain 윈도우");

  // Theory box (top)
  panel(s, 0.45, 1.1, 12.45, 1.20);
  s.addText("운용 식 — shot 윈도우는 거리에서 유도된다", {
    x: 0.60, y: 1.18, w: 12.2, h: 0.32,
    fontSize: 13, bold: true, fontFace: FONT_HEADER, color: C.deep, margin: 0,
  });
  s.addText([
    { text: "T_window  =  max_range_clks × T_axis    ", options: { bold: true, color: C.primary } },
    { text: "(거리 왕복 시간, capture_active 동안 stop hit 수신)\n", options: { color: C.muted } },
    { text: "T_drain_cap  =  max_range_clks + g_DRAIN_MARGIN_CLKS(256)    ", options: { bold: true, color: C.warn } },
    { text: "(chip_run 7개 timeout cap, x\"FFFF\" saturation)", options: { color: C.muted } },
  ], {
    x: 0.60, y: 1.55, w: 12.2, h: 0.65,
    fontSize: 11, fontFace: FONT_MONO, color: C.text, margin: 0,
  });

  // Big distance table
  panel(s, 0.45, 2.45, 12.45, 4.50);
  s.addText("CSR : RANGE_COLS[15:0] = max_range_clks  ·  AXIS = 200 MHz (T = 5 ns) 가정", {
    x: 0.60, y: 2.55, w: 12.2, h: 0.32,
    fontSize: 11, bold: true, fontFace: FONT_HEADER, color: C.deep, margin: 0,
  });

  const tHead = [
    { text: "거리", options: { bold: true, fill: { color: C.deep }, color: "FFFFFF" } },
    { text: "왕복 (m)", options: { bold: true, fill: { color: C.deep }, color: "FFFFFF" } },
    { text: "왕복 시간\n(@c=3×10⁸)", options: { bold: true, fill: { color: C.deep }, color: "FFFFFF" } },
    { text: "max_range_clks\n(200MHz)", options: { bold: true, fill: { color: C.deep }, color: "FFFFFF" } },
    { text: "drain cap\n(+256)", options: { bold: true, fill: { color: C.deep }, color: "FFFFFF" } },
    { text: "1 shot wall-clock\n(approx)", options: { bold: true, fill: { color: C.deep }, color: "FFFFFF" } },
    { text: "운용 메모", options: { bold: true, fill: { color: C.deep }, color: "FFFFFF" } },
  ];
  // Compute roundtrip = 2*d/c, max_range_clks = roundtrip / T_axis(5ns)
  // d=100m → 666.7ns / 5 = 134 clks ; ... etc
  // d=250m → 1666.7 / 5 = 334 ; d=500m → 3333.3/5 = 667 ; d=750m → 5000/5 = 1000 ; d=1000 → 6666.7/5=1334
  const rows = [
    [
      { text: "100 m",  options: { bold: true } }, "200",
      "0.667 µs", "134", "390", "≈ 1 µs (capture+drain)", "고해상도 / 짧은 ToF · 빠른 frame"
    ],
    [
      { text: "250 m",  options: { bold: true } }, "500",
      "1.667 µs", "334", "590", "≈ 2 µs",          "중거리 표준 — drain margin 충분"
    ],
    [
      { text: "500 m",  options: { bold: true } }, "1000",
      "3.333 µs", "667", "923", "≈ 4 µs",          "기본 시뮬 설정 (G_MAX_RANGE_M=500)"
    ],
    [
      { text: "750 m",  options: { bold: true } }, "1500",
      "5.000 µs", "1000", "1256", "≈ 6 µs",        "긴 ToF — face FIFO 깊이 검토 필요"
    ],
    [
      { text: "1000 m", options: { bold: true } }, "2000",
      "6.667 µs", "1334", "1590", "≈ 7-8 µs",       "최대 거리 — shot rate 한계 결정 요인"
    ],
  ];
  s.addTable([tHead, ...rows], {
    x: 0.60, y: 2.95, w: 12.2, h: 2.95,
    colW: [0.9, 1.1, 1.6, 1.7, 1.4, 2.6, 2.9],
    fontSize: 10, fontFace: FONT_BODY, color: C.text,
    border: { pt: 0.5, color: C.light },
    valign: "middle",
  });

  // Operational notes
  s.addText([
    { text: "운용 함의 — ", options: { bold: true, color: C.warn } },
    { text: "(1) max_range_clks = 0 은 \"watchdog disable\" 로 해석되어 chip_run 이 legacy x\"FFFF\" cap (≈327µs) 으로 fallback. \n", options: {} },
    { text: "(2) Drain margin 256 = 1.28 µs 는 bus roundtrip + ALU pulse + downstream backpressure jitter 흡수용. SW가 더 좁히려면 g_DRAIN_MARGIN_CLKS 줄여야 함. \n", options: {} },
    { text: "(3) 1 shot wall-clock 은 chip_run 4개가 병렬이라 \"4× drain\" 이 아니라 1× drain ≈ output serialization 으로 결정. \n", options: {} },
    { text: "(4) shot rate 상한 ≈ 1 / (T_window + T_serialize). 1000m + 64-bit + max_hits=7 stops_full 에서 ≈ 8µs → 125 kHz 한계.", options: {} },
  ], {
    x: 0.60, y: 5.95, w: 12.2, h: 1.0,
    fontSize: 9.5, fontFace: FONT_BODY, color: C.text, margin: 0,
  });

  pageFooter(s, slideNum, TOTAL, "tdc_gpx_chip_run.vhd:68–100 · memory project_tdc_window_timing.md");
}

// =============================================================================
// SLIDE 9 — chip_run measurement cycle (ARMED → CAPTURE → DRAIN → ALU)
// =============================================================================
{
  slideNum = 9;
  const s = pres.addSlide();
  bg(s);
  pageHeader(s, "측정 단위 cycle — chip_run FSM 운용",
             "ARMED → CAPTURE → DRAIN_LATCH → DRAIN_EF1/2 → ALU pulse → ARMED");

  // FSM banner with state boxes
  panel(s, 0.45, 1.1, 12.45, 2.45);
  s.addText("chip_run 9-state — shot 1회 처리 흐름", {
    x: 0.60, y: 1.18, w: 12.2, h: 0.32,
    fontSize: 13, bold: true, fontFace: FONT_HEADER, color: C.deep, margin: 0,
  });

  const fsm = [
    { t: "ST_OFF",          d: "i_start 대기 (cmd_start_accepted 후 ARMED)" },
    { t: "ST_ARMED",        d: "shot_start 대기, range_active='0', stopdis 풀림" },
    { t: "ST_CAPTURE",      d: "range_active='1', max_range_clks 동안\nstop hit 칩 내부 IFIFO 적재" },
    { t: "ST_DRAIN_LATCH",  d: "IrFlag 도달 시 expected_ififo1/2 latch (1 cycle)" },
    { t: "ST_DRAIN_EF1",    d: "IFIFO1 비울 때까지 28-bit burst read" },
    { t: "ST_DRAIN_EF2",    d: "IFIFO2 비울 때까지 28-bit burst read" },
    { t: "ST_DRAIN_CHECK",  d: "drain_cnt vs expected 검증 → mismatch sticky" },
    { t: "ST_ALU",          d: "ALU trigger pulse (g_ALU_PULSE_CLKS=4)" },
    { t: "ST_RECOVERY",     d: "recovery clks (post-ALU) → ARMED 복귀" },
  ];
  for (let i = 0; i < fsm.length; i++) {
    const x = 0.60 + (i % 9) * 1.36;
    const y = 1.55;
    s.addShape(pres.shapes.RECTANGLE, {
      x, y, w: 1.32, h: 0.45,
      fill: { color: i === 0 ? C.muted : (i === 8 ? C.muted : C.primary) },
      line: { color: C.deep, width: 0 },
    });
    s.addText(fsm[i].t, {
      x: x + 0.02, y: y + 0.07, w: 1.28, h: 0.30,
      fontSize: 9.2, bold: true, fontFace: FONT_HEADER,
      color: "FFFFFF", align: "center", margin: 0,
    });
    // arrow
    if (i < fsm.length - 1) {
      s.addShape(pres.shapes.LINE, {
        x: x + 1.32, y: y + 0.225, w: 0.04, h: 0,
        line: { color: C.muted, width: 1.5, endArrowType: "triangle" },
      });
    }
    s.addText(fsm[i].d, {
      x: x - 0.05, y: y + 0.50, w: 1.42, h: 1.0,
      fontSize: 7.8, fontFace: FONT_BODY, color: C.text,
      align: "center", margin: 0,
    });
  }

  // Bottom split : timeout cause table + range_active timing
  panel(s, 0.45, 3.7, 6.40, 3.4);
  s.addText("timeout_cause — STAT7[10:8] 에 latch 되는 마지막 원인", {
    x: 0.60, y: 3.78, w: 6.25, h: 0.32,
    fontSize: 12, bold: true, fontFace: FONT_HEADER, color: C.warn, margin: 0,
  });
  s.addTable([
    [
      { text: "code", options: { bold: true, fill: { color: C.warn }, color: "FFFFFF" } },
      { text: "원인", options: { bold: true, fill: { color: C.warn }, color: "FFFFFF" } },
      { text: "관측", options: { bold: true, fill: { color: C.warn }, color: "FFFFFF" } },
    ],
    ["001", "raw_busy",            "raw hold register full → stall"],
    ["010", "ef1_rsp",             "IFIFO1 read 응답 missing"],
    ["011", "ef2_rsp",             "IFIFO2 read 응답 missing"],
    ["100", "burst_rsp",           "burst read 도중 응답 미완"],
    ["101", "flush_rsp",           "DRAIN_CHECK fallback"],
    ["110", "overrun_flush",       "예상치 초과 hit (IFIFO 넘침 가능)"],
    ["111", "capture_stop_fallback", "cmd_stop 후 IrFlag 못 본 경우"],
  ], {
    x: 0.60, y: 4.17, w: 6.25, h: 2.85,
    colW: [0.85, 2.05, 3.35],
    fontSize: 9, fontFace: FONT_BODY, color: C.text,
    border: { pt: 0.5, color: C.light },
  });

  // range_active explanation
  panel(s, 7.05, 3.7, 5.85, 3.4);
  s.addText("range_active 윈도우 — 의미와 활용", {
    x: 7.20, y: 3.78, w: 5.65, h: 0.32,
    fontSize: 12, bold: true, fontFace: FONT_HEADER, color: C.deep, margin: 0,
  });
  s.addText([
    { text: "● ST_CAPTURE 시작 ~ ST_RECOVERY 끝까지 '1' 유지\n", options: { color: C.text } },
    { text: "● cell_builder p_collect 가 'shot 활성 윈도우' 로 사용 → \n  이 동안 들어온 raw event 만 dense cell 에 적재\n", options: {} },
    { text: "● status_agg 의 error_cycle_count 도 이 윈도우 활성 시간을\n  활용한 'duration-based' 카운터 (event count 가 아님!)\n", options: {} },
    { text: "● shot_start mid-window arrival → mid-shot stopdis 발생 시\n  err_stopdis_mid_shot_mask sticky (R12 B8)\n\n", options: {} },
    { text: "운용 권장 : ", options: { bold: true, color: C.warn } },
    { text: "shot rate 가 너무 빠르면 range_active 가 채\n끝나기 전에 다음 shot 이 와서 overrun_flush 발생.\nshot 간격 ≥ T_window + T_serialize 유지.", options: {} },
  ], {
    x: 7.20, y: 4.15, w: 5.65, h: 2.85,
    fontSize: 9.5, fontFace: FONT_BODY, color: C.text, margin: 0, paraSpaceAfter: 1,
  });

  pageFooter(s, slideNum, TOTAL, "chip_run.vhd:60–177 · register_map.md STAT7 layout");
}

// =============================================================================
// SLIDE 10 — Error classification & recovery
// =============================================================================
{
  slideNum = 10;
  const s = pres.addSlide();
  bg(s);
  pageHeader(s, "오류 분류·복구 운용 — err_handler FSM 6-state",
             "ErrFlag debounce → Reg12 read → recovery → frame 경계 동기화 → 정상");

  // FSM diagram
  panel(s, 0.45, 1.1, 7.55, 3.10);
  s.addText("err_handler 자동 복구 FSM", {
    x: 0.60, y: 1.18, w: 7.40, h: 0.32,
    fontSize: 12, bold: true, fontFace: FONT_HEADER, color: C.deep, margin: 0,
  });
  const efsm = [
    { x: 0.65, t: "ST_IDLE",            sub: "ErrFlag debounce\n(g_DEBOUNCE_CLKS=4)" },
    { x: 1.85, t: "ST_READ_REG12",      sub: "reg_outstanding=0 대기\n(R10 16-bit cap)" },
    { x: 3.05, t: "ST_WAIT_READ",       sub: "reg done pulse 대기\n(0xFFFF watchdog)" },
    { x: 4.25, t: "ST_RECOVERY",        sub: "per-chip\nsoft_reset pulse" },
    { x: 5.45, t: "ST_WAIT_RECOVERY",   sub: "recov_timeout 14b\n+ stable_low 검사" },
    { x: 6.65, t: "ST_WAIT_FRAME_DONE", sub: "frame 경계 동기\n(R10 17-bit watchdog)" },
  ];
  for (let i = 0; i < efsm.length; i++) {
    const e = efsm[i];
    s.addShape(pres.shapes.RECTANGLE, {
      x: e.x, y: 1.65, w: 1.15, h: 1.10,
      fill: { color: i === 0 ? C.muted : C.warn }, line: { color: C.warn, width: 0 },
    });
    s.addText(e.t, {
      x: e.x, y: 1.70, w: 1.15, h: 0.30,
      fontSize: 8.4, bold: true, fontFace: FONT_HEADER,
      color: "FFFFFF", align: "center", margin: 0,
    });
    s.addText(e.sub, {
      x: e.x, y: 2.02, w: 1.15, h: 0.70,
      fontSize: 7.5, fontFace: FONT_BODY,
      color: "FFFFFF", align: "center", margin: 0,
    });
    if (i < efsm.length - 1) {
      s.addShape(pres.shapes.LINE, {
        x: e.x + 1.15, y: 2.20, w: 0.05, h: 0,
        line: { color: C.muted, width: 1.5, endArrowType: "triangle" },
      });
    }
  }
  // Loop arrow back to IDLE
  s.addShape(pres.shapes.LINE, {
    x: 7.80, y: 2.20, w: 0, h: 0.85,
    line: { color: C.warn, width: 1.5 },
  });
  s.addShape(pres.shapes.LINE, {
    x: 1.20, y: 3.05, w: 6.60, h: 0,
    line: { color: C.warn, width: 1.5 },
  });
  s.addShape(pres.shapes.LINE, {
    x: 1.20, y: 2.75, w: 0, h: 0.30,
    line: { color: C.warn, width: 1.5, endArrowType: "triangle" },
  });
  s.addText("frame_done + shot_start ⇒ retry++, max=g_MAX_RETRIES(3)\n초과 시 err_fatal sticky 'on', SW 가 i_err_soft_clear 로 해제",
  {
    x: 1.5, y: 3.20, w: 6.4, h: 0.85,
    fontSize: 9.5, fontFace: FONT_BODY, color: C.warn, italic: true, margin: 0,
  });

  // Right : sticky vs level vs counter (clear semantics)
  panel(s, 8.10, 1.1, 4.80, 3.10);
  s.addText("Clear semantic 운용", {
    x: 8.25, y: 1.18, w: 4.60, h: 0.32,
    fontSize: 12, bold: true, fontFace: FONT_HEADER, color: C.deep, margin: 0,
  });
  s.addText([
    { text: "level\n", options: { bold: true, color: C.primary } },
    { text: "  현재 상태 그대로 (chip_error_mask). Live 관측용.\n\n", options: {} },
    { text: "sticky\n", options: { bold: true, color: C.warn } },
    { text: "  발생하면 i_rst_n 또는 i_err_soft_clear pulse 까지 '1'.\n", options: {} },
    { text: "  err_fatal · drain_timeout_mask · sequence_error_mask\n  err_read_timeout · err_reg_overflow_mask · stop_id_error\n\n", options: { color: C.muted } },
    { text: "wrap counter (8-bit)\n", options: { bold: true, color: C.teal } },
    { text: "  shot_overrun_count · face_start_collapsed.\n", options: {} },
    { text: "  CDC handshake 중 변경은 collapse → '≥ N' 으로만 해석.", options: { color: C.muted } },
  ], {
    x: 8.25, y: 1.55, w: 4.60, h: 2.55,
    fontSize: 9.2, fontFace: FONT_BODY, color: C.text, margin: 0, paraSpaceAfter: 0,
  });

  // Bottom : per-chip error class table
  panel(s, 0.45, 4.35, 12.45, 2.70);
  s.addText("Per-chip 오류 클래스 — 발생 위치 · 의미 · 해소 경로", {
    x: 0.60, y: 4.43, w: 12.2, h: 0.32,
    fontSize: 12, bold: true, fontFace: FONT_HEADER, color: C.deep, margin: 0,
  });
  s.addTable([
    [
      { text: "오류", options: { bold: true, fill: { color: C.deep }, color: "FFFFFF" } },
      { text: "발생 위치", options: { bold: true, fill: { color: C.deep }, color: "FFFFFF" } },
      { text: "의미 / 운용 영향", options: { bold: true, fill: { color: C.deep }, color: "FFFFFF" } },
      { text: "해소", options: { bold: true, fill: { color: C.deep }, color: "FFFFFF" } },
    ],
    ["err_drain_timeout",   "chip_run watchdog",        "expected count 만큼 drain 못함 → frame 손상", "soft_clear / hard reset"],
    ["err_drain_mismatch",  "chip_run ST_DRAIN_CHECK",  "drain_cnt ≠ expected, fallback 종료",         "drain_done_faulted 비트 carry"],
    ["err_sequence",        "stop_tdc 시퀀스 검증",     "stop_tdc out-of-order arrival",               "soft_clear"],
    ["err_raw_drop",        "raw FIFO + drain_cap",     "6-deep raw FIFO 가 받아내지 못함",           "n_drain_cap 조정"],
    ["err_stopdis_mid_shot","range_active 도중 stopdis","측정 윈도우 깨짐",                            "shot rate 하향 / cmd_stop"],
    ["err_rw_ambiguous",    "cmd_arb",                  "동시 read+write 모호",                        "SW 시퀀스 재검토"],
    ["err_cmd_collision",   "cmd_arb",                  "init/run/reg 명령 충돌",                       "queue 1-depth 정책 인지"],
    ["err_bus_fatal",       "bus_phy",                  "복구 불가 bus error (R13 axis 2)",           "force_reinit / reset"],
  ], {
    x: 0.60, y: 4.80, w: 12.2, h: 2.18,
    colW: [2.0, 2.1, 5.3, 2.8],
    fontSize: 8.7, fontFace: FONT_BODY, color: C.text,
    border: { pt: 0.5, color: C.light },
  });

  pageFooter(s, slideNum, TOTAL, "err_handler.vhd:1–200 · status_observability.md · top L177–187");
}

// =============================================================================
// SLIDE 11 — STATUS register operational view
// =============================================================================
{
  slideNum = 11;
  const s = pres.addSlide();
  bg(s);
  pageHeader(s, "STATUS 레지스터 운용 모델 — STAT5 / STAT6 / STAT7",
             "어떤 비트를 언제 읽고, 무엇으로 지우는가 (CDC handshake 기준)");

  // Three vertical panels for STAT5, STAT6, STAT7
  const panels = [
    {
      x: 0.45, t: "STAT5  @ 0x54  (pipeline 기본)",
      color: C.primary,
      rows: [
        ["[0] busy",                "level",   "어떤 FSM이라도 비-IDLE"],
        ["[1] pipeline_overrun",    "level",   "rise OR fall face_assembler"],
        ["[2] err_fatal",           "sticky",  "retry exhausted"],
        ["[7:4] chip_error_mask",   "level",   "live per-chip merge"],
        ["[11:8] drain_timeout",    "sticky",  "chip_run watchdog"],
        ["[15:12] sequence_error",  "sticky",  "stop_tdc 시퀀스"],
      ],
      clear: "i_rst_n 또는 i_err_soft_clear pulse",
    },
    {
      x: 4.62, t: "STAT6  @ 0x58  (확장 stickies + counters)",
      color: C.teal,
      rows: [
        ["[0] err_read_timeout",    "sticky",  "err_handler ST_WAIT_READ"],
        ["[1] reg_rejected",        "sticky",  "cmd_arb overlap"],
        ["[2] reg_zero_mask",       "sticky",  "mask=0000 reg request"],
        ["[3] shot_flush_drop_rise","sticky",  "shot_start 시 FIFO 비어있지 않음"],
        ["[4] shot_flush_drop_fall","sticky",  "fall 동일"],
        ["[15:8] shot_overrun_count_rise", "wrap8", "blank-fill count"],
        ["[23:16] shot_overrun_count_fall","wrap8", "blank-fill count"],
        ["[27:24] err_reg_overflow_mask",  "sticky", "chip_reg 3rd-pulse"],
        ["[31:28] run_drain_complete_mask","sticky", "다음 shot_start로 clear"],
      ],
      clear: "다수 sticky : soft_clear ; flush_drop : reset only",
    },
    {
      x: 8.79, t: "STAT7  @ 0x5C  (R11 신규 운용 가시성)",
      color: C.accent,
      rows: [
        ["[3:0] reg_timeout_mask",        "sticky",  "per-chip reg transaction"],
        ["[7:4] stop_id_error_mask",      "sticky",  "cell_builder out-of-range"],
        ["[10:8] run_timeout_cause_last", "latched", "마지막 timeout 3-bit code"],
        ["[23:16] rise_face_start_collapsed", "wrap8", "header_inserter coalesce"],
        ["[31:24] fall_face_start_collapsed", "wrap8", "header_inserter coalesce"],
      ],
      clear: "reg_timeout : 정상 완료 OR soft_clear ;\nstop_id : soft_clear ; cause_last : 자동 clear 없음",
    },
  ];

  for (const p of panels) {
    panel(s, p.x, 1.1, 4.10, 5.65);
    // Header
    s.addShape(pres.shapes.RECTANGLE, {
      x: p.x, y: 1.1, w: 4.10, h: 0.45,
      fill: { color: p.color }, line: { color: p.color, width: 0 },
    });
    s.addText(p.t, {
      x: p.x + 0.10, y: 1.16, w: 3.95, h: 0.34,
      fontSize: 11, bold: true, fontFace: FONT_HEADER,
      color: "FFFFFF", margin: 0,
    });
    // Mini table
    const tHead = [
      { text: "비트", options: { bold: true, fill: { color: "EEF2F7" }, color: C.deep, fontSize: 9 } },
      { text: "유형", options: { bold: true, fill: { color: "EEF2F7" }, color: C.deep, fontSize: 9 } },
      { text: "의미", options: { bold: true, fill: { color: "EEF2F7" }, color: C.deep, fontSize: 9 } },
    ];
    s.addTable([tHead, ...p.rows], {
      x: p.x + 0.05, y: 1.62, w: 4.0, h: 4.4,
      colW: [1.55, 0.85, 1.60],
      fontSize: 8.2, fontFace: FONT_BODY, color: C.text,
      border: { pt: 0.4, color: C.light },
    });
    // Clear semantic
    s.addShape(pres.shapes.RECTANGLE, {
      x: p.x + 0.05, y: 6.10, w: 4.0, h: 0.60,
      fill: { color: "EEF2F7" }, line: { color: p.color, width: 0.5 },
    });
    s.addText([
      { text: "Clear: ", options: { bold: true, color: p.color } },
      { text: p.clear, options: { color: C.text } },
    ], {
      x: p.x + 0.10, y: 6.13, w: 3.95, h: 0.55,
      fontSize: 8.5, fontFace: FONT_BODY, margin: 0, paraSpaceAfter: 0,
    });
  }

  // Bottom note: CDC caveat
  s.addShape(pres.shapes.RECTANGLE, {
    x: 0.45, y: 6.85, w: 12.45, h: 0.32,
    fill: { color: C.warn }, line: { color: C.warn, width: 0 },
  });
  s.addText([
    { text: "⚠ CDC 운용 주의 — ", options: { bold: true, color: "FFFFFF" } },
    { text: "STAT5 / 6 / 7 은 32-bit xpm_cdc_handshake 로 s_axi_aclk 도메인 가시화. wrap counter 는 in-flight handshake 동안 변경은 collapse → SW 는 '≥ N' 으로만 해석 (R7 B-3).", options: { color: "FFFFFF" } },
  ], {
    x: 0.55, y: 6.88, w: 12.25, h: 0.28,
    fontSize: 9.5, fontFace: FONT_BODY, margin: 0,
  });

  pageFooter(s, slideNum, TOTAL, "register_map.md L50–119 · status_agg.vhd · csr_pipeline.vhd");
}

// =============================================================================
// SLIDE 12 — Backpressure & Abort propagation
// =============================================================================
{
  slideNum = 12;
  const s = pres.addSlide();
  bg(s);
  pageHeader(s, "Backpressure / Abort 전파 — VDMA tready 가 chip_ctrl 까지 닿는 길",
             "downstream stall 이 어떻게 흡수되고, 어떻게 abort 로 격상되는가");

  // Top : back-pressure chain (right to left)
  panel(s, 0.45, 1.1, 12.45, 3.20);
  s.addText("downstream stall 이 흡수되는 경로 (오른쪽 → 왼쪽)", {
    x: 0.60, y: 1.18, w: 12.2, h: 0.32,
    fontSize: 13, bold: true, fontFace: FONT_HEADER, color: C.deep, margin: 0,
  });
  // 7 nodes
  const bpChain = [
    { x: 0.60,  t: "TDC chip\nIFIFO1/2",        d: "datasheet\n28×256 reservoir",         c: C.muted },
    { x: 2.30,  t: "chip_ctrl\nraw FIFO",       d: "6-deep absorb\n(R9 #7)",                c: C.primary },
    { x: 4.00,  t: "raw_cdc\nxpm_fifo_async",   d: "16×40b CDC",                            c: C.primary },
    { x: 5.70,  t: "decode_pipe\n+ skid",       d: "+1 cycle / stage",                      c: C.teal },
    { x: 7.40,  t: "cell_builder\n2-buffer",    d: "shot-local dense\n(register file)",     c: C.teal },
    { x: 9.10,  t: "face_assembler\n+ FIFO 16", d: "row build\n+ blank-fill",                c: C.teal },
    { x: 10.80, t: "header_inserter\n+ FIFO",   d: "48B header\nVDMA gate",                 c: C.accent },
  ];
  // VDMA target
  for (const bn of bpChain) {
    s.addShape(pres.shapes.RECTANGLE, {
      x: bn.x, y: 1.65, w: 1.55, h: 1.65,
      fill: { color: C.panel }, line: { color: bn.c, width: 1 },
    });
    s.addShape(pres.shapes.RECTANGLE, {
      x: bn.x, y: 1.65, w: 1.55, h: 0.32,
      fill: { color: bn.c }, line: { color: bn.c, width: 0 },
    });
    s.addText(bn.t, {
      x: bn.x + 0.05, y: 1.68, w: 1.45, h: 0.28,
      fontSize: 9.5, bold: true, fontFace: FONT_HEADER,
      color: "FFFFFF", align: "center", margin: 0,
    });
    s.addText(bn.d, {
      x: bn.x + 0.05, y: 2.05, w: 1.45, h: 1.20,
      fontSize: 8.5, fontFace: FONT_BODY, color: C.text,
      align: "center", margin: 0,
    });
  }
  // tready=0 arrow back-prop (above)
  s.addShape(pres.shapes.LINE, {
    x: 1.40, y: 1.55, w: 11, h: 0,
    line: { color: C.warn, width: 2, beginArrowType: "triangle" },
  });
  s.addText("tready = 0  back-pressure", {
    x: 5.0, y: 1.30, w: 5.0, h: 0.22,
    fontSize: 9.5, bold: true, fontFace: FONT_BODY,
    color: C.warn, align: "center", margin: 0,
  });

  // dataflow forward arrow (bottom)
  s.addShape(pres.shapes.LINE, {
    x: 1.40, y: 3.50, w: 11, h: 0,
    line: { color: C.primary, width: 2, endArrowType: "triangle" },
  });
  s.addText("data flow →", {
    x: 5.5, y: 3.55, w: 4, h: 0.22,
    fontSize: 9.5, bold: true, fontFace: FONT_BODY,
    color: C.primary, align: "center", margin: 0,
  });

  // VDMA box
  s.addShape(pres.shapes.RECTANGLE, {
    x: 12.40, y: 1.65, w: 0.50, h: 1.65,
    fill: { color: C.dark }, line: { color: C.dark, width: 0 },
  });
  s.addText("VDMA", {
    x: 12.30, y: 2.30, w: 0.7, h: 0.4,
    fontSize: 8, bold: true, fontFace: FONT_HEADER,
    color: "FFFFFF", align: "center", margin: 0,
  });

  // Bottom split : abort triggers + reset patterns
  panel(s, 0.45, 4.45, 6.10, 2.65);
  s.addText("abort 로 격상되는 조건 — 흡수 한계 초과 시", {
    x: 0.60, y: 4.53, w: 5.95, h: 0.32,
    fontSize: 12, bold: true, fontFace: FONT_HEADER, color: C.warn, margin: 0,
  });
  s.addText([
    { text: "(1) face_assembler scan timeout (chip 0..3 미도달, blank-fill)\n", options: {} },
    { text: "(2) face_assembler shot_start 시 FIFO non-empty (shot_flush_drop)\n", options: {} },
    { text: "(3) header_inserter drain watchdog\n", options: {} },
    { text: "(4) cell_builder QUARANTINE 진입 (drain_done lost) — 자동탈출 X (B-1)\n", options: {} },
    { text: "(5) chip_run timeout_cause 발생 (1-clk pulse)\n", options: {} },
    { text: "→ 어느 쪽이든 ", options: { color: C.muted } },
    { text: "abort_rise / abort_fall 로 face_seq 가 변환,\n  frame_id 는 abort frame 으로 보존, frame_abort_count++.\n", options: {} },
  ], {
    x: 0.60, y: 4.90, w: 5.95, h: 2.10,
    fontSize: 9.5, fontFace: FONT_BODY, color: C.text, margin: 0, paraSpaceAfter: 1,
  });

  panel(s, 6.80, 4.45, 6.10, 2.65);
  s.addText("FIFO reset 운용 — R4a 계열 패치 영향", {
    x: 6.95, y: 4.53, w: 5.95, h: 0.32,
    fontSize: 12, bold: true, fontFace: FONT_HEADER, color: C.warn, margin: 0,
  });
  s.addText([
    { text: "face_assembler 입력 FIFO ×4 : reset = i_rst_n AND NOT (shot_start OR abort)\n", options: { fontFace: FONT_MONO, color: C.text } },
    { text: "face_assembler 출력 FIFO    : 동일 (R4a 추가)\n", options: { fontFace: FONT_MONO, color: C.text } },
    { text: "output_stage rise/fall FIFO  : 동일 (R4a 추가)\n\n", options: { fontFace: FONT_MONO, color: C.text } },
    { text: "왜 shot_start 도 reset 으로 쓰는가?\n", options: { bold: true, color: C.deep } },
    { text: "xsim 2025.2.1 의 xpm_fifo_axis 는 elaboration 후 추가 reset toggle 없이는\ns_axis_tready='0' 에서 영구 lock. shot_start 가 그 toggle 역할.\n", options: {} },
    { text: "Trade-off : ", options: { bold: true, color: C.warn } },
    { text: "이전 row 의 tail beat 가 in-flight 인 경우 drop 가능성. 현재는\nface_seq packet_start 가 drain 후에만 발사하므로 손실 미관측.", options: {} },
  ], {
    x: 6.95, y: 4.90, w: 5.95, h: 2.15,
    fontSize: 9, fontFace: FONT_BODY, color: C.text, margin: 0, paraSpaceAfter: 1,
  });

  pageFooter(s, slideNum, TOTAL, "face_assembler.vhd:435–470 · output_stage.vhd · memory project_tb_integration_status.md");
}

// =============================================================================
// SLIDE 13 — SIM mode operation
// =============================================================================
{
  slideNum = 13;
  const s = pres.addSlide();
  bg(s);
  pageHeader(s, "SIM 모드 운용 — TB 환경에서의 협조 동작",
             "motor_decoder · echo_receiver · laser_ctrl · TDC behavioral model");

  // Top diagram
  panel(s, 0.45, 1.1, 12.45, 2.85);
  s.addText("4-IP 협조 — laser_ctrl 이 실제로 firing 되려면", {
    x: 0.60, y: 1.18, w: 12.2, h: 0.32,
    fontSize: 13, bold: true, fontFace: FONT_HEADER, color: C.deep, margin: 0,
  });

  const ipBoxes = [
    {
      x: 0.65, t: "motor_decoder",
      d: "CTL0 := C_MD_CTL0_SIM (sim_en=1)\n+ SWRST_SIM\n→ face_center 가상 발생",
      c: C.primary,
    },
    {
      x: 3.65, t: "echo_receiver",
      d: "CTL0[0] = SIM_EN\nCTL1 := C_SIM_TARGET_CLKS\n→ stop_evt 가상 packet",
      c: C.primary,
    },
    {
      x: 6.65, t: "laser_ctrl",
      d: "CTL4 virt_offset = C_SIM_TARGET_CLKS\n(이미 sim mode 기본)\n→ shot_start pulse 발사",
      c: C.teal,
    },
    {
      x: 9.65, t: "tdc_gpx_top",
      d: "별도 SIM CSR 없음.\nTB가 behavioral chip model 인스턴스\n(4 IFIFO emulator)",
      c: C.teal,
    },
  ];
  for (const ip of ipBoxes) {
    s.addShape(pres.shapes.RECTANGLE, {
      x: ip.x, y: 1.65, w: 2.85, h: 2.10,
      fill: { color: C.panel }, line: { color: ip.c, width: 1 },
    });
    s.addShape(pres.shapes.RECTANGLE, {
      x: ip.x, y: 1.65, w: 2.85, h: 0.35,
      fill: { color: ip.c }, line: { color: ip.c, width: 0 },
    });
    s.addText(ip.t, {
      x: ip.x + 0.10, y: 1.70, w: 2.65, h: 0.30,
      fontSize: 11.5, bold: true, fontFace: FONT_HEADER,
      color: "FFFFFF", margin: 0,
    });
    s.addText(ip.d, {
      x: ip.x + 0.10, y: 2.10, w: 2.65, h: 1.60,
      fontSize: 9.5, fontFace: FONT_BODY, color: C.text, margin: 0, paraSpaceAfter: 1,
    });
  }

  // Bottom : warnings + TB pattern
  panel(s, 0.45, 4.10, 6.10, 2.95);
  s.addText("PHYS 모드가 TB에서 실패하는 이유 (반드시 SIM 사용)", {
    x: 0.60, y: 4.18, w: 5.95, h: 0.32,
    fontSize: 12, bold: true, fontFace: FONT_HEADER, color: C.warn, margin: 0,
  });
  s.addText([
    { text: "tb_laser_ctrl_pkg 의 motor_decoder face center 식은\nAXIS=150 MHz 가정인데 tb 는 200 MHz 로 돌림.\n", options: { color: C.text } },
    { text: "→ 물리 encoder 가 정해진 face boundary 에 안 맞음\n→ laser_ctrl 이 영구히 firing 안 됨 → shot 이 안 발생.\n\n", options: { color: C.warn } },
    { text: "교훈 : 통합 TB 는 4-IP 모두 SIM 켜야 함.\n", options: { bold: true, color: C.text } },
    { text: "tb_tdc_gpx_full_int / tb_tdc_gpx_top_int 에 generic 으로\nG_AXIS_CLK_MHZ / G_MAX_RANGE_M / G_TDC_STIM_MODE 등\n파라미터화 되어 있다 — 거리/도메인 변경 시 generic 만 바꾸면 됨.", options: { color: C.text } },
  ], {
    x: 0.60, y: 4.55, w: 5.95, h: 2.42,
    fontSize: 9.5, fontFace: FONT_BODY, color: C.text, margin: 0, paraSpaceAfter: 1,
  });

  panel(s, 6.80, 4.10, 6.10, 2.95);
  s.addText("실측에서 발견된 TB-side 함정 (memory R4a 후속)", {
    x: 6.95, y: 4.18, w: 5.95, h: 0.32,
    fontSize: 12, bold: true, fontFace: FONT_HEADER, color: C.warn, margin: 0,
  });
  s.addText([
    { text: "(a) ", options: { bold: true, color: C.warn } },
    { text: "VHDL-2008 external name (<< signal ... >>) 가 xsim 2025.2.1\n에서 'only elaborated instance can be referenced' 로 실패.\n", options: { color: C.text } },
    { text: "    → tb 의 dbg_* monitor 신호는 stub. 카운터 값 = 0 정상.\n\n", options: { color: C.muted } },
    { text: "(b) ", options: { bold: true, color: C.warn } },
    { text: "add_condition \"sig(N) == 1\" / \"sig[N] == 1\" 로 4b slv 비트별\n카운트 시 under-count 심각 (probe artifact).\n", options: { color: C.text } },
    { text: "    → 'run N ns ; get_value' settled-time 샘플링 방식 사용.\n\n", options: { color: C.muted } },
    { text: "(c) echo_receiver IP 는 num_ctl_regs=17 → 4-chip × 8 stop full\nsim 커버리지 위해 ≥ 33 으로 재생성 필요.", options: { color: C.text } },
  ], {
    x: 6.95, y: 4.55, w: 5.95, h: 2.42,
    fontSize: 9, fontFace: FONT_BODY, color: C.text, margin: 0, paraSpaceAfter: 1,
  });

  pageFooter(s, slideNum, TOTAL, "memory project_tb_integration_status.md · tb_tdc_gpx_full_int.vhd:1–80");
}

// =============================================================================
// SLIDE 14 — Code review #1 : dead code, deprecated, naming debt
// =============================================================================
{
  slideNum = 14;
  const s = pres.addSlide();
  bg(s);
  pageHeader(s, "코드 결함 review ① — 제거되었지만 주석으로만 남은 자취",
             "Dead signals · deprecated 포트 · naming debt — 청소 항목");

  // Issue table (severity-based)
  panel(s, 0.45, 1.1, 12.45, 5.95);
  s.addText("9개 항목 — 모두 file:line 으로 검증됨", {
    x: 0.60, y: 1.18, w: 12.2, h: 0.32,
    fontSize: 12, bold: true, fontFace: FONT_HEADER, color: C.deep, margin: 0,
  });

  s.addTable([
    [
      { text: "#", options: { bold: true, fill: { color: C.deep }, color: "FFFFFF" } },
      { text: "유형", options: { bold: true, fill: { color: C.deep }, color: "FFFFFF" } },
      { text: "위치", options: { bold: true, fill: { color: C.deep }, color: "FFFFFF" } },
      { text: "설명 / 영향", options: { bold: true, fill: { color: C.deep }, color: "FFFFFF" } },
      { text: "권장", options: { bold: true, fill: { color: C.deep }, color: "FFFFFF" } },
    ],
    [
      { text: "1", options: { bold: true } },
      "DEPRECATED port",
      "face_assembler.vhd:131–151",
      "o_face_abort 가 'Round 4 이후 영구 0'. tdc_gpx_top.vhd:772 에서 'open' 으로 묶음. output_stage 의 forwarding 코드도 같이 dead.",
      "TB 4종 동시 정리 후 port 제거 (release 1회 묶어서)",
    ],
    [
      { text: "2", options: { bold: true } },
      "naming debt",
      "err_handler.vhd:52–62",
      "i_reg11_data_* 4개 포트가 실제로는 Reg12 (chip status) 데이터를 운반. 이름이 의미를 호도.",
      "config_ctrl + csr_chip 까지 한 번에 reg12 로 rename",
    ],
    [
      { text: "3", options: { bold: true } },
      "dead signal 주석",
      "face_seq.vhd:114–116",
      "s_shot_overrun_r 가 'Round 5 #20 에서 제거됨' 주석만 남음. 실신호 없음 (clean).",
      "주석 자체를 제거 (의미가 다 사라진 흔적)",
    ],
    [
      { text: "4", options: { bold: true } },
      "removed-overrun trail",
      "chip_ctrl.vhd:30, 308, 1014\nchip_run.vhd:47, 167",
      "Round 5 #5 / Round 6 B2 에서 overrun-drop path 가 5곳에 걸쳐 'removed' 주석으로 단편 잔존.",
      "주석 통합 정리 — 한 곳에 history 노트 + 나머지 제거",
    ],
    [
      { text: "5", options: { bold: true } },
      "tied-to-zero 잔존",
      "tdc_gpx_top.vhd:261–266, 845",
      "s_face_abort / s_face_fall_abort 명시적으로 ground 됨. R7 C-1 의도된 동작이지만 'reconnected' 위험 주석으로 보호 중.",
      "현 상태 유지 (의도된 안전장치)",
    ],
    [
      { text: "6", options: { bold: true } },
      "STAT4..31 미사용",
      "csr_chip.vhd:31, 446",
      "STAT4..31 에 driver 없이 0. 미래 확장용 슬롯이지만 driver-less 상태로 합성됨.",
      "사용 시점까지 유지 / 명시적 'reserved' 코멘트",
    ],
    [
      { text: "7", options: { bold: true } },
      "stub (legacy compat)",
      "csr_pipeline.vhd : open ports",
      "tdc_gpx_top.vhd:542, 553 의 o_cmd_start, o_reg_loop_resume 가 'open' — face_seq 가 미래에 사용 예정.",
      "예약 포트로 명시 / 사용 시점에 제거 결정",
    ],
    [
      { text: "8", options: { bold: true } },
      "alias",
      "face_seq.vhd:148, 530",
      "s_pipeline_abort = s_pipeline_abort_rise (legacy alias). #22 Sprint 3 후속 작업의 잔여물.",
      "Sprint 3 완성 시 alias 제거 + 해당 caller 직접 사용",
    ],
    [
      { text: "9", options: { bold: true } },
      "shot_overrun 주석 분산",
      "chip_run.vhd 295\nchip_ctrl.vhd 309",
      "s_overrun_deferred_r / 비슷한 'removed' 자취가 여러 곳에 흩어져 있음.",
      "Round 6 정리 commit 의 history 한 줄로 통합 메모",
    ],
  ], {
    x: 0.60, y: 1.55, w: 12.2, h: 5.40,
    colW: [0.4, 1.85, 2.20, 5.30, 2.45],
    fontSize: 8.5, fontFace: FONT_BODY, color: C.text,
    border: { pt: 0.5, color: C.light },
  });

  pageFooter(s, slideNum, TOTAL, "검증된 file:line 인용 — grep 'removed|DEPRECATED|legacy' 결과");
}

// =============================================================================
// SLIDE 15 — Code review #2 : hardcodes, R4a hack, known_issues residual
// =============================================================================
{
  slideNum = 15;
  const s = pres.addSlide();
  bg(s);
  pageHeader(s, "코드 결함 review ② — Hardcoded watchdog · R4a 임시 패치 · known_issues 잔여",
             "기능적 영향이 더 큰 항목들 — 향후 cleanup 우선순위 후보");

  // Top : hardcodes
  panel(s, 0.45, 1.1, 6.20, 2.80);
  s.addText("Hardcoded watchdog / margin (generic 화 후보)", {
    x: 0.60, y: 1.18, w: 6.05, h: 0.32,
    fontSize: 12, bold: true, fontFace: FONT_HEADER, color: C.warn, margin: 0,
  });
  s.addTable([
    [
      { text: "위치", options: { bold: true, fill: { color: C.warn }, color: "FFFFFF" } },
      { text: "값", options: { bold: true, fill: { color: C.warn }, color: "FFFFFF" } },
      { text: "의미", options: { bold: true, fill: { color: C.warn }, color: "FFFFFF" } },
    ],
    ["err_handler.vhd:156",   "0xFFFF",      "ST_WAIT_READ TO (generic 없음)"],
    ["err_handler.vhd:165",   "0xFFFF",      "R10 #4 ST_READ_REG12 entry TO"],
    ["err_handler.vhd:172",   "17-bit",      "R10 #5 ST_WAIT_FRAME_DONE (~6.5ms)"],
    ["err_handler.vhd:141",   "14-bit",      "recov_timeout (~82µs @ 200MHz)"],
    ["chip_ctrl.vhd:73",      "4096 clk",    "g_BUS_IDLE_STABLE (~20µs)"],
    ["face_assembler",        "depth=16",    "input/output FIFO (config 없음)"],
    ["chip_run g_DRAIN_MARGIN_CLKS", "256", "거리당 윈도우 + 1.28µs (generic 있음 ✓)"],
  ], {
    x: 0.60, y: 1.55, w: 6.05, h: 2.30,
    colW: [2.45, 1.05, 2.55],
    fontSize: 8.5, fontFace: FONT_BODY, color: C.text,
    border: { pt: 0.5, color: C.light },
  });

  // R4a hack
  panel(s, 6.85, 1.1, 6.05, 2.80);
  s.addText("R4a — xpm_fifo_axis xsim 2025.2.1 quirk 패치", {
    x: 7.0, y: 1.18, w: 5.90, h: 0.32,
    fontSize: 12, bold: true, fontFace: FONT_HEADER, color: C.warn, margin: 0,
  });
  s.addText([
    { text: "증상 : ", options: { bold: true } },
    { text: "elaboration 후 reset toggle 없으면 s_axis_tready 영구 '0'.\n", options: {} },
    { text: "패치 : ", options: { bold: true, color: C.warn } },
    { text: "shot_start 를 reset trigger 로 추가 (face_assembler.vhd:442–462,\noutput_stage.vhd 동일).\n", options: {} },
    { text: "원래 의도 : ", options: { bold: true, color: C.muted } },
    { text: "L445–446 'shot_start 는 output FIFO 를 flush 하면 안 된다 — 이전\nrow tail beat 가 transit 중일 수 있음'. → 현재는 face_seq 가 drain 후에만\npacket_start 발사하므로 손실 미관측.\n\n", options: { color: C.muted } },
    { text: "리스크 : ", options: { bold: true, color: C.warn } },
    { text: "tail-beat-in-flight 인 corner case 에서 drop 가능. ", options: {} },
    { text: "후속 audit 필요.\n", options: { italic: true, color: C.warn } },
    { text: "USE_ADV_FEATURES → \"1000\", m_aclk → i_clk 변경도 hygiene 으로 적용.", options: { color: C.muted } },
  ], {
    x: 7.0, y: 1.55, w: 5.90, h: 2.35,
    fontSize: 9, fontFace: FONT_BODY, color: C.text, margin: 0, paraSpaceAfter: 1,
  });

  // known_issues residual
  panel(s, 0.45, 4.05, 12.45, 3.0);
  s.addText("known_issues.md 에 명시된 'intentional' 항목 — 운용에서 알아야 할 trade-off", {
    x: 0.60, y: 4.13, w: 12.2, h: 0.32,
    fontSize: 12, bold: true, fontFace: FONT_HEADER, color: C.deep, margin: 0,
  });
  s.addTable([
    [
      { text: "Cat", options: { bold: true, fill: { color: C.deep }, color: "FFFFFF" } },
      { text: "항목", options: { bold: true, fill: { color: C.deep }, color: "FFFFFF" } },
      { text: "선택된 정책", options: { bold: true, fill: { color: C.deep }, color: "FFFFFF" } },
      { text: "운용에서 주의할 점", options: { bold: true, fill: { color: C.deep }, color: "FFFFFF" } },
    ],
    [
      "A", "out-of-spec config (stops_per_chip < 2 등)",
      "Clamp-to-default (face_assembler / cell_builder / header_inserter)",
      "단, face_seq 는 hard-reject + cfg_rejected pulse → SW 가 cmd_start 거부 감지해야 함"
    ],
    [
      "B-1", "cell_builder.ST_C_QUARANTINE",
      "drain_done 또는 abort 까지 'stay forever'",
      "shot_dropped 모니터 + stuck QUARANTINE 시 i_abort 로만 회복 가능"
    ],
    [
      "B-2", "raw FIFO control + data 단일 path",
      "depth=6 으로 흡수 (분리하지 않음)",
      "shot rate 가 비정상으로 빠르면 drain_cap mask sticky 발생"
    ],
    [
      "B-3", "cfg / cfg_image CDC",
      "2-FF sync + ASYNC_REG (handshake 아님)",
      "SW 가 '값 안정 → 4 cycle 대기 → 명령 pulse' 순서 책임"
    ],
    [
      "B-4", "rise/fall slope abort 비대칭",
      "rise-only gating, fall 은 frame_done 만 기다림",
      "fall-only abort 는 rise stream 살림 — primary/secondary 정책 인지"
    ],
    [
      "D", "face_assembler.o_face_abort port",
      "DEPRECATED, '0' 영구 stub",
      "TB 3 종이 아직 참조 → port 제거 시 동시 cleanup 필요"
    ],
  ], {
    x: 0.60, y: 4.50, w: 12.2, h: 2.50,
    colW: [0.55, 2.95, 3.40, 5.30],
    fontSize: 8.5, fontFace: FONT_BODY, color: C.text,
    border: { pt: 0.5, color: C.light },
  });

  pageFooter(s, slideNum, TOTAL, "Doc/known_issues.md · 검증된 grep 결과 (TODO/FIXME/HACK 마커는 0건)");
}

// =============================================================================
// SLIDE 16 — Observability gaps & recommendations
// =============================================================================
{
  slideNum = 16;
  const s = pres.addSlide();
  bg(s);
  pageHeader(s, "관측성 보강 권장 — status_observability.md 잔여 항목",
             "현재 STAT 에 노출 안 된 신호 + 향후 추가 후보");

  // Two columns
  panel(s, 0.45, 1.1, 6.10, 5.95);
  s.addText("이미 노출 — t_tdc_status / STAT5/6/7", {
    x: 0.60, y: 1.18, w: 5.95, h: 0.32,
    fontSize: 12, bold: true, fontFace: FONT_HEADER, color: C.ok, margin: 0,
  });
  s.addText([
    { text: "STATUS / IRQ\n", options: { bold: true, color: C.deep } },
    { text: "  busy, pipeline_overrun, err_fatal\n", options: { color: C.text } },
    { text: "  rise_overrun / fall_overrun\n  err_drain_sticky / err_seq_sticky\n  chip_error_mask / err_chip_mask\n", options: { color: C.text } },
    { text: "  err_cause (HFifoFull / IFifoFull / NotLocked)\n", options: { color: C.text } },
    { text: "  rsp_mismatch_mask\n\n", options: { color: C.text } },
    { text: "확장 stickies (R5 / R6 / R7)\n", options: { bold: true, color: C.deep } },
    { text: "  err_read_timeout / reg_rejected / reg_zero_mask\n", options: { color: C.text } },
    { text: "  shot_flush_drop_rise/fall\n  shot_overrun_count_rise/fall (8b wrap)\n", options: { color: C.text } },
    { text: "  err_reg_overflow_mask\n  run_drain_complete_mask (per-shot clear)\n\n", options: { color: C.text } },
    { text: "R11 신규 (STAT7)\n", options: { bold: true, color: C.deep } },
    { text: "  reg_timeout_mask, stop_id_error_mask\n  run_timeout_cause_last (3-bit code)\n", options: { color: C.text } },
    { text: "  rise/fall_face_start_collapsed (8b wrap)\n", options: { color: C.text } },
    { text: "\n카운터 / 통계\n", options: { bold: true, color: C.deep } },
    { text: "  global_shot_seq · frame_id · face_id\n", options: { color: C.text } },
    { text: "  shot_drop_count · frame_abort_count\n  error_cycle_count (32b duration count)\n  timestamp (64b free-running)", options: { color: C.text } },
  ], {
    x: 0.60, y: 1.55, w: 5.95, h: 5.40,
    fontSize: 9.5, fontFace: FONT_BODY, color: C.text, margin: 0, paraSpaceAfter: 0,
  });

  panel(s, 6.80, 1.1, 6.10, 5.95);
  s.addText("미노출 — 추가 권장 (t_tdc_status 미포함)", {
    x: 6.95, y: 1.18, w: 5.95, h: 0.32,
    fontSize: 12, bold: true, fontFace: FONT_HEADER, color: C.warn, margin: 0,
  });
  s.addText([
    { text: "신호별 (status_observability.md 'NOT in t_tdc_status')\n", options: { bold: true, color: C.deep } },
    { text: "  o_shot_dropped (cell_builder×8) — shot 단위 drop\n", options: { color: C.text } },
    { text: "  o_hit_dropped_any — cell hit overflow per slope\n", options: { color: C.text } },
    { text: "  o_slice_timeout — IFIFO2 timeout truncation\n", options: { color: C.text } },
    { text: "  o_cfg_rejected (face_seq) — 1-clk pulse (현재 sticky 없음)\n", options: { color: C.text } },
    { text: "  s_raw_hold_busy — chip_ctrl raw hold stall (counter 추천)\n", options: { color: C.text } },
    { text: "  o_timeout (chip_init/chip_reg) — 통합 sticky 없음\n\n", options: { color: C.text } },
    { text: "추천 신규 sticky / counter\n", options: { bold: true, color: C.deep } },
    { text: "  ① init_timeout_mask (per-chip)\n", options: { color: C.text } },
    { text: "  ② reg_timeout_mask (현 STAT7 적용 — 부분 해결)\n", options: { color: C.text } },
    { text: "  ③ raw_stall_count (8b wrap)\n", options: { color: C.text } },
    { text: "  ④ cfg_rejected_count (16b wrap)\n", options: { color: C.text } },
    { text: "  ⑤ unexpected_restart_count (header_inserter 비-IDLE face_start)\n", options: { color: C.text } },
    { text: "  ⑥ late_beat_flushed_count (face_assembler shot_flush)\n\n", options: { color: C.text } },
    { text: "정책 권장\n", options: { bold: true, color: C.deep } },
    { text: "  Pulse vs Counter vs Sticky 구분 명확화 (이미 doc 명시).\n", options: { color: C.text } },
    { text: "  CDC handshake 의 wrap counter collapse 한계 SW 매뉴얼화.", options: { color: C.text } },
  ], {
    x: 6.95, y: 1.55, w: 5.95, h: 5.40,
    fontSize: 9, fontFace: FONT_BODY, color: C.text, margin: 0, paraSpaceAfter: 0,
  });

  pageFooter(s, slideNum, TOTAL, "Doc/status_observability.md · top L177–187");
}

// =============================================================================
// SLIDE 17 — Cleanup priority recommendation
// =============================================================================
{
  slideNum = 17;
  const s = pres.addSlide();
  bg(s);
  pageHeader(s, "정리 / cleanup 우선순위 — 결함을 등급별로 분류",
             "위험도 × 정리 비용 매트릭스 — 어디부터 손볼 것인가");

  // Quadrant matrix
  const mx = 0.45, my = 1.1, mw = 8.45, mh = 5.5;
  panel(s, mx, my, mw, mh);
  s.addText("결함 우선순위 매트릭스 (위험도 ↑ × 비용 →)", {
    x: mx + 0.20, y: my + 0.10, w: mw - 0.4, h: 0.32,
    fontSize: 12, bold: true, fontFace: FONT_HEADER, color: C.deep, margin: 0,
  });

  // Axis lines
  const ax0 = mx + 0.65, ay0 = my + 0.65, axW = mw - 1.0, ayH = mh - 1.05;
  s.addShape(pres.shapes.LINE, {
    x: ax0, y: ay0, w: 0, h: ayH,
    line: { color: C.muted, width: 1.3, endArrowType: "triangle" },
  });
  s.addShape(pres.shapes.LINE, {
    x: ax0, y: ay0 + ayH, w: axW, h: 0,
    line: { color: C.muted, width: 1.3, endArrowType: "triangle" },
  });
  // Quadrant separator
  s.addShape(pres.shapes.LINE, {
    x: ax0, y: ay0 + ayH / 2, w: axW, h: 0,
    line: { color: C.light, width: 0.7, dashType: "dash" },
  });
  s.addShape(pres.shapes.LINE, {
    x: ax0 + axW / 2, y: ay0, w: 0, h: ayH,
    line: { color: C.light, width: 0.7, dashType: "dash" },
  });
  // Axis labels
  s.addText("위험도 ↑", {
    x: ax0 - 0.6, y: ay0, w: 0.7, h: 0.4,
    fontSize: 9, bold: true, color: C.muted, margin: 0,
  });
  s.addText("정리 비용 →", {
    x: ax0 + axW - 0.95, y: ay0 + ayH + 0.05, w: 1.0, h: 0.32,
    fontSize: 9, bold: true, color: C.muted, margin: 0,
  });
  // Quadrant labels
  s.addText("HIGH 위험 / LOW 비용\n— 즉시", {
    x: ax0 + 0.05, y: ay0 + 0.10, w: axW / 2 - 0.1, h: 0.5,
    fontSize: 9.5, bold: true, color: C.warn, margin: 0,
  });
  s.addText("HIGH 위험 / HIGH 비용\n— 다음 sprint", {
    x: ax0 + axW / 2 + 0.05, y: ay0 + 0.10, w: axW / 2 - 0.1, h: 0.5,
    fontSize: 9.5, bold: true, color: C.warn, margin: 0,
  });
  s.addText("LOW 위험 / LOW 비용\n— 자투리 시간", {
    x: ax0 + 0.05, y: ay0 + ayH / 2 + 0.10, w: axW / 2 - 0.1, h: 0.5,
    fontSize: 9.5, bold: true, color: C.ok, margin: 0,
  });
  s.addText("LOW 위험 / HIGH 비용\n— 보류", {
    x: ax0 + axW / 2 + 0.05, y: ay0 + ayH / 2 + 0.10, w: axW / 2 - 0.1, h: 0.5,
    fontSize: 9.5, bold: true, color: C.muted, margin: 0,
  });

  // Place items (x = cost 0..1, y = risk 0..1 with origin top-left risk-high)
  const items = [
    // HL — high risk low cost
    { x: 0.10, y: 0.20, t: "R4a tail-beat\naudit",          c: C.warn },
    { x: 0.18, y: 0.35, t: "B-1 QUARANTINE\nliveness alarm", c: C.warn },
    { x: 0.30, y: 0.10, t: "err_handler\nhardcode→generic",  c: C.warn },
    // HH — high risk high cost
    { x: 0.65, y: 0.30, t: "echo_receiver IP\nregen (33 regs)", c: C.warn },
    { x: 0.80, y: 0.20, t: "Sprint 3 fall-only\nabort 완성",    c: C.warn },
    // LL — low risk low cost
    { x: 0.12, y: 0.70, t: "removed-comments\n청소",          c: C.ok },
    { x: 0.25, y: 0.78, t: "naming debt\nreg11→reg12",          c: C.ok },
    { x: 0.35, y: 0.65, t: "alias 제거\n(legacy abort)",        c: C.ok },
    { x: 0.18, y: 0.90, t: "dead signal\n주석 정리",            c: C.ok },
    // LH — low risk high cost
    { x: 0.85, y: 0.85, t: "o_face_abort\nport 제거 (TB 3종)",  c: C.muted },
    { x: 0.65, y: 0.90, t: "관측성 6개\n신규 sticky",           c: C.muted },
  ];
  for (const it of items) {
    const px = ax0 + it.x * axW;
    const py = ay0 + it.y * ayH;
    s.addShape(pres.shapes.OVAL, {
      x: px - 0.55, y: py - 0.20, w: 1.10, h: 0.42,
      fill: { color: it.c }, line: { color: it.c, width: 0 },
    });
    s.addText(it.t, {
      x: px - 0.55, y: py - 0.20, w: 1.10, h: 0.42,
      fontSize: 6.8, bold: true, fontFace: FONT_HEADER,
      color: "FFFFFF", align: "center", valign: "middle", margin: 0,
    });
  }

  // Right panel : suggested 3-step roadmap
  panel(s, 9.05, 1.1, 3.85, 5.5);
  s.addText("3-step 실행 roadmap", {
    x: 9.20, y: 1.18, w: 3.65, h: 0.32,
    fontSize: 12, bold: true, fontFace: FONT_HEADER, color: C.deep, margin: 0,
  });
  s.addShape(pres.shapes.RECTANGLE, {
    x: 9.20, y: 1.55, w: 3.65, h: 1.55,
    fill: { color: "FFEDED" }, line: { color: C.warn, width: 0.8 },
  });
  s.addText("Phase 1 — 기능 위험\n(2 ~ 3일)", {
    x: 9.30, y: 1.62, w: 3.55, h: 0.4,
    fontSize: 11, bold: true, color: C.warn, margin: 0,
  });
  s.addText("• R4a tail-beat audit\n• err_handler 하드코드 generic 화\n• cell_builder QUARANTINE 알람", {
    x: 9.30, y: 2.00, w: 3.55, h: 1.15,
    fontSize: 9.5, color: C.text, margin: 0, paraSpaceAfter: 1,
  });

  s.addShape(pres.shapes.RECTANGLE, {
    x: 9.20, y: 3.20, w: 3.65, h: 1.55,
    fill: { color: "F0F8FF" }, line: { color: C.primary, width: 0.8 },
  });
  s.addText("Phase 2 — 정리\n(0.5일)", {
    x: 9.30, y: 3.27, w: 3.55, h: 0.4,
    fontSize: 11, bold: true, color: C.primary, margin: 0,
  });
  s.addText("• removed-* 주석 통합\n• reg11 → reg12 rename\n• alias / dead signal 제거", {
    x: 9.30, y: 3.65, w: 3.55, h: 1.15,
    fontSize: 9.5, color: C.text, margin: 0, paraSpaceAfter: 1,
  });

  s.addShape(pres.shapes.RECTANGLE, {
    x: 9.20, y: 4.85, w: 3.65, h: 1.65,
    fill: { color: "F5F5F5" }, line: { color: C.muted, width: 0.8 },
  });
  s.addText("Phase 3 — 인터페이스\n(미정)", {
    x: 9.30, y: 4.92, w: 3.55, h: 0.4,
    fontSize: 11, bold: true, color: C.muted, margin: 0,
  });
  s.addText("• Sprint 3 fall-only abort\n• o_face_abort port 제거\n• echo_receiver IP regen\n• 관측성 ⑥ 항목 추가", {
    x: 9.30, y: 5.30, w: 3.55, h: 1.20,
    fontSize: 9.5, color: C.text, margin: 0, paraSpaceAfter: 1,
  });

  pageFooter(s, slideNum, TOTAL, "후속 sprint 권장 — Phase 1 부터 손대면 운용 안정성에 가장 큰 효과");
}

// =============================================================================
// SLIDE 18 — Conclusion
// =============================================================================
{
  slideNum = 18;
  const s = pres.addSlide();
  bg(s);
  pageHeader(s, "결론 — 운용 개념 보강 및 코드 결함 검토 정리",
             "추가된 12개 운용 개념 + 검증된 9 + 다중 결함 항목");

  // Three takeaway columns
  const cols2 = [
    {
      x: 0.45, t: "운용 개념 — 새로 채워진 12 영역",
      d: [
        "① Boot lifecycle (CSR write 순서 / 폴링)",
        "② chip_init 9-state 부팅",
        "③ face_seq FSM 3-state + cfg snapshot",
        "④ shot 트리거 chain (raw → packet)",
        "⑤ rise/fall slope 비대칭 정책",
        "⑥ 거리 use case 별 max_range_clks",
        "⑦ chip_run 9-state 측정 cycle",
        "⑧ err_handler 6-state 자동 복구",
        "⑨ STAT5/6/7 운용 모델 (clear semantic)",
        "⑩ Backpressure / abort 전파",
        "⑪ SIM mode 4-IP 협조",
        "⑫ 관측성 보강 후보 6 신호",
      ],
      c: C.primary,
    },
    {
      x: 4.62, t: "코드 결함 — 검증된 항목",
      d: [
        "① DEPRECATED port (face_assembler.o_face_abort)",
        "② naming debt (reg11→reg12)",
        "③ dead signal 주석 잔존 (face_seq)",
        "④ removed-overrun 5곳 분산 잔재",
        "⑤ tied-to-zero 의도된 ground (top)",
        "⑥ STAT4..31 driver-less 슬롯",
        "⑦ open ports (예약)",
        "⑧ legacy alias (rise abort)",
        "⑨ 하드코드 watchdog 6 곳",
        "⑩ R4a xpm_fifo reset hack (tail-beat 위험)",
        "⑪ B-1 cell_builder QUARANTINE 영구화",
        "⑫ B-3 CFG 2-FF (handshake 아님)",
      ],
      c: C.warn,
    },
    {
      x: 8.79, t: "다음 액션 — 우선순위",
      d: [
        "Phase 1 (즉시)",
        "  · R4a tail-beat audit",
        "  · err_handler hardcode → generic",
        "  · QUARANTINE liveness alarm",
        "",
        "Phase 2 (자투리)",
        "  · removed-* 주석 통합",
        "  · reg11 → reg12 rename",
        "  · alias / dead signal 제거",
        "",
        "Phase 3 (인터페이스 sprint)",
        "  · Sprint 3 fall-only abort 완성",
        "  · o_face_abort port 제거 + TB 동시 작업",
        "  · echo_receiver IP 33-reg regen",
      ],
      c: C.accent,
    },
  ];

  for (const c of cols2) {
    panel(s, c.x, 1.1, 4.10, 5.95);
    s.addShape(pres.shapes.RECTANGLE, {
      x: c.x, y: 1.1, w: 4.10, h: 0.45,
      fill: { color: c.c }, line: { color: c.c, width: 0 },
    });
    s.addText(c.t, {
      x: c.x + 0.10, y: 1.16, w: 3.95, h: 0.34,
      fontSize: 11.5, bold: true, fontFace: FONT_HEADER,
      color: "FFFFFF", margin: 0,
    });
    s.addText(c.d.map((line, i, a) => ({
      text: line, options: {
        breakLine: i < a.length - 1,
        bold: line.startsWith("Phase"),
        color: line.startsWith("Phase") ? c.c : C.text,
      }
    })), {
      x: c.x + 0.18, y: 1.65, w: 3.85, h: 5.30,
      fontSize: 10, fontFace: FONT_BODY, color: C.text,
      margin: 0, paraSpaceAfter: 1,
    });
  }

  pageFooter(s, slideNum, TOTAL, "최종 — 본 보강 PPT 와 기존 두 PPT를 함께 보면 tdc_gpx_top 전체 운용·구조 이해 완성");
}

// -----------------------------------------------------------------------------
// Save
// -----------------------------------------------------------------------------
pres.writeFile({ fileName: "Doc/tdc_gpx_top_operation_and_review.pptx" })
    .then(name => console.log("Saved:", name));
