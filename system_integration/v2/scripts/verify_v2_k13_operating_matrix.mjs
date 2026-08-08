import assert from "node:assert/strict";
import fs from "node:fs";
import vm from "node:vm";

function executableGolden(htmlPath) {
  const html = fs.readFileSync(htmlPath, "utf8");
  const scriptMatch = html.match(/<script>([\s\S]*?)<\/script>/);
  if (!scriptMatch) throw new Error(`No inline model script in ${htmlPath}`);

  const marker = "    const K13_UI_START = true;";
  const markerIndex = scriptMatch[1].indexOf(marker);
  if (markerIndex < 0) throw new Error("K1-3 HTML initialization marker is missing");

  const context = Object.create(null);
  vm.runInNewContext(
    `${scriptMatch[1].slice(0, markerIndex)}\n` +
      "globalThis.__golden = buildK13Golden();",
    context,
    {filename:htmlPath}
  );
  return JSON.parse(JSON.stringify(context.__golden));
}

function readJson(path) {
  return JSON.parse(fs.readFileSync(path, "utf8").replace(/^\uFEFF/, ""));
}

if (process.argv[2] === "--materialize") {
  if (process.argv.length !== 5) {
    throw new Error(
      "usage: node verify_v2_k13_operating_matrix.mjs --materialize <html> <golden-json>"
    );
  }
  const golden = executableGolden(process.argv[3]);
  fs.writeFileSync(process.argv[4], `${JSON.stringify(golden,null,2)}\n`, "utf8");
  console.log("LIDAR_V2_K13_GOLDEN_MATERIALIZED");
  process.exit(0);
}

if (process.argv.length !== 6) {
  throw new Error(
    "usage: node verify_v2_k13_operating_matrix.mjs " +
    "<html> <golden-json> <rtl-measurements-json> <comparison-json>"
  );
}

const [htmlPath,goldenPath,measurementPath,comparisonPath] = process.argv.slice(2);
const htmlGolden = executableGolden(htmlPath);
const checkedGolden = readJson(goldenPath);
const measurements = readJson(measurementPath);

assert.deepStrictEqual(
  htmlGolden,
  checkedGolden,
  "Checked K1-3 Golden JSON differs from the executable HTML model"
);
assert.equal(
  measurements.schema,
  "tdc-gpx-lidar-v2-k13-rtl-measurements-v1",
  "Unexpected RTL measurement schema"
);

const acquisitionKeys = [
  "proc_mhz","tdc_mhz","width","returns","target_5ns"
];
const exactAcquisitionFields = [
  "shot_to_start_clks","start_to_stop_clks","hsize_bytes",
  "vsize_lines","stride_bytes","frame_beats"
];
const profileKey = (value,keys) => keys.map(key => value[key]).join("/");

assert.equal(
  measurements.acquisition_profiles.length,
  checkedGolden.acquisition_expectations.length,
  "Acquisition profile count mismatch"
);
const expectedAcquisition = new Map(
  checkedGolden.acquisition_expectations.map(value => [
    profileKey(value,acquisitionKeys), value
  ])
);
for (const actual of measurements.acquisition_profiles) {
  const key = profileKey(actual,acquisitionKeys);
  const expected = expectedAcquisition.get(key);
  assert.ok(expected, `Unexpected acquisition profile ${key}`);
  for (const field of exactAcquisitionFields) {
    assert.equal(actual[field],expected[field],`${key}: ${field} mismatch`);
  }
  assert.ok(
    actual.stop_to_line_clks >= expected.stop_to_line_clks_min &&
      actual.stop_to_line_clks <= expected.stop_to_line_clks_max,
    `${key}: STOP-to-Line latency escaped the one-clock async phase window`
  );
  assert.ok(
    actual.shot_to_line_clks >= expected.shot_to_line_clks_min &&
      actual.shot_to_line_clks <= expected.shot_to_line_clks_max,
    `${key}: Shot-to-Line latency escaped the one-clock async phase window`
  );
  assert.equal(
    actual.shot_to_line_clks,
    actual.start_to_stop_clks + actual.stop_to_line_clks,
    `${key}: latency decomposition mismatch`
  );
  expectedAcquisition.delete(key);
}
assert.equal(expectedAcquisition.size,0,"One or more acquisition profiles were not run");

const topologyKeys = ["proc_mhz","width","topology","stops","returns"];
const exactTopologyFields = [
  ...topologyKeys,"slots","hsize_bytes","vsize_lines",
  "stride_bytes","frame_beats"
];
assert.equal(
  measurements.topology_profiles.length,
  checkedGolden.topology_expectations.length,
  "Topology profile count mismatch"
);
const expectedTopology = new Map(
  checkedGolden.topology_expectations.map(value => [
    profileKey(value,topologyKeys), value
  ])
);
for (const actual of measurements.topology_profiles) {
  const key = profileKey(actual,topologyKeys);
  const expected = expectedTopology.get(key);
  assert.ok(expected,`Unexpected topology profile ${key}`);
  for (const field of exactTopologyFields) {
    assert.equal(actual[field],expected[field],`${key}: ${field} mismatch`);
  }
  expectedTopology.delete(key);
}
assert.equal(expectedTopology.size,0,"One or more topology profiles were not run");

const passCases = checkedGolden.operating_cases.filter(
  value => value.result.status === "PASS"
);
const checkCases = checkedGolden.operating_cases.filter(
  value => value.result.status === "CHECK"
);
assert.ok(passCases.length > 0,"Golden matrix has no feasible operating point");
assert.ok(checkCases.length > 0,"Golden matrix has no rejected operating point");

const minimumMarginCase = checkedGolden.operating_cases.reduce((left,right) =>
  right.result.margin_clks < left.result.margin_clks ? right : left
);
const comparison = {
  schema:"tdc-gpx-lidar-v2-k13-comparison-v1",
  result:"PASS",
  acquisition_profiles:measurements.acquisition_profiles.length,
  topology_profiles:measurements.topology_profiles.length,
  operating_pass_cases:passCases.length,
  operating_check_cases:checkCases.length,
  minimum_margin_case:{
    name:minimumMarginCase.name,
    margin_clks:minimumMarginCase.result.margin_clks,
    status:minimumMarginCase.result.status
  },
  checks:[
    "HTML model equals checked Golden JSON",
    "RTL target-window conversion and VDMA geometry exact-match",
    "RTL STOP-to-Line latency remains inside measured async phase window",
    "32/64/128-bit maximum topology geometry exact-match",
    "Golden set contains both feasible and infeasible operating points"
  ]
};
fs.writeFileSync(comparisonPath,`${JSON.stringify(comparison,null,2)}\n`,"utf8");
console.log("LIDAR_V2_K13_RTL_HTML_OPERATING_MATRIX_PASS");
