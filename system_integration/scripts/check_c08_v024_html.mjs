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
  "C08_HDL_HTML_Alignment_260727_Operating_Point_Budget_Simulator_v024.html"
);
process.env.C08_DEFAULT_RESULT_PATHS = JSON.stringify([
  path.join(sessionRoot, "260727_operating_point_matrix1_r1_w32_system_integration_smoke", "rtl_result.json"),
  path.join(sessionRoot, "260727_operating_point_final1_system_integration_smoke", "rtl_result.json"),
  path.join(sessionRoot, "260727_operating_point_matrix1_r7_w64_system_integration_smoke", "rtl_result.json"),
  path.join(sessionRoot, "260727_operating_point_matrix1_r7_w128_system_integration_smoke", "rtl_result.json"),
  path.join(sessionRoot, "260727_point_budget_boundary1_system_integration_smoke", "rtl_result.json")
]);
process.env.C08_PASS_MARKER = "C08_V024_HTML_SELF_TEST_PASS";

await import("./check_c08_v023_html.mjs");
