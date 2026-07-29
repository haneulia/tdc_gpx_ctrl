import path from "node:path";
import { fileURLToPath } from "node:url";

const scriptDir = path.dirname(fileURLToPath(import.meta.url));
const hdlRoot = path.resolve(scriptDir, "..", "..");
const sessionRoot = path.join(hdlRoot, "sim_results", "vivado_xsim", "sessions");

process.env.C08_HTML_PATH = path.join(
  hdlRoot,
  "Doc",
  "cluster_analysis",
  "C08_HDL_HTML_Alignment",
  "C08_HDL_HTML_Alignment_260729_Unified_CSR_Timing_Contract_Simulator_v025.html"
);
process.env.C08_DEFAULT_RESULT_PATHS = JSON.stringify([
  path.join(sessionRoot, "260729_local_t0_contract_r6_system_integration_smoke", "rtl_result.json"),
  path.join(sessionRoot, "260729_unified_t0_contract_r6_system_integration_smoke", "rtl_result.json"),
  path.join(sessionRoot, "260729_local_axis200_tdc200_r1_system_integration_smoke", "rtl_result.json"),
  path.join(sessionRoot, "260729_unified_axis200_tdc200_r1_system_integration_smoke", "rtl_result.json"),
  path.join(sessionRoot, "260729_unified_wmatrix_axis150_w32_system_integration_smoke", "rtl_result.json"),
  path.join(sessionRoot, "260729_unified_wmatrix_axis150_w64_system_integration_smoke", "rtl_result.json"),
  path.join(sessionRoot, "260729_unified_wmatrix_axis150_w128_system_integration_smoke", "rtl_result.json"),
  path.join(sessionRoot, "260729_unified_wmatrix_axis200_w32_system_integration_smoke", "rtl_result.json"),
  path.join(sessionRoot, "260729_unified_wmatrix_axis200_w64_system_integration_smoke", "rtl_result.json"),
  path.join(sessionRoot, "260729_unified_wmatrix_axis200_w128_system_integration_smoke", "rtl_result.json")
]);
process.env.C08_PASS_MARKER = "C08_V025_HTML_SELF_TEST_PASS";

await import("./check_c08_v023_html.mjs");
