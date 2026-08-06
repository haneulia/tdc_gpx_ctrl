import assert from "node:assert/strict";
import fs from "node:fs";
import vm from "node:vm";

if (process.argv.length !== 4) {
  throw new Error(
    "usage: node verify_packed17_html_golden.mjs <html> <golden-json>"
  );
}

const htmlPath = process.argv[2];
const goldenPath = process.argv[3];
const html = fs.readFileSync(htmlPath, "utf8");
const scriptMatch = html.match(/<script>([\s\S]*?)<\/script>/);

if (!scriptMatch) {
  throw new Error(`No inline model script in ${htmlPath}`);
}

const initializationMarker = "    buildMaskGrid();";
const markerIndex = scriptMatch[1].indexOf(initializationMarker);
if (markerIndex < 0) {
  throw new Error("HTML model initialization marker is missing");
}

const modelSource = scriptMatch[1].slice(0, markerIndex);
const context = Object.create(null);
vm.runInNewContext(
  `${modelSource}\n` +
    "globalThis.__golden = {" +
    "schema:'tdc-gpx-packed17-ddr-golden-v1'," +
    "generated_by:'C08-v026 buildJ9Golden'," +
    "scenario:'one real Shot + one Hole Shot + ordered Face Footer'," +
    "profiles:[32,64,128].map(buildJ9Golden)};",
  context,
  { filename: htmlPath }
);

const htmlGolden = JSON.parse(JSON.stringify(context.__golden));
const checkedGolden = JSON.parse(fs.readFileSync(goldenPath, "utf8"));
assert.deepStrictEqual(
  htmlGolden,
  checkedGolden,
  "Checked Golden JSON differs from the executable HTML model"
);

for (const profile of checkedGolden.profiles) {
  assert.equal(
    profile.memory_words.length * 4,
    profile.geometry.frame_allocation_bytes,
    `width ${profile.output_width_bits}: memory image size mismatch`
  );
  assert.ok(
    profile.memory_words.includes(profile.geometry.reserve_fill_word),
    `width ${profile.output_width_bits}: fixed-STRIDE reserve was not modeled`
  );
}

console.log("LIDAR_V2_PACKED17_HTML_GOLDEN_PASS");
