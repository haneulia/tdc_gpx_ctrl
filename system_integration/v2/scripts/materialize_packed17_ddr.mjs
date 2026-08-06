import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";

if (process.argv.length < 4 || process.argv.length > 5) {
  throw new Error(
    "usage: node materialize_packed17_ddr.mjs <ddr-golden.json> " +
      "<output-dir> [j9-capture-dir]"
  );
}

const goldenPath = process.argv[2];
const outputDir = process.argv[3];
const captureDir = process.argv[4];
const golden = JSON.parse(fs.readFileSync(goldenPath, "utf8"));
const clocks = [150, 200];
const manifest = [];

function parseCapture(capturePath) {
  return fs.readFileSync(capturePath, "utf8")
    .split(/\r?\n/)
    .filter((line) => line.trim() !== "")
    .map((line, index) => {
      const match = line.match(/^0x([0-9a-f]{8})\s+0x([0-9a-f]{8})$/i);
      assert.ok(match, `${capturePath}:${index + 1}: malformed capture line`);
      assert.equal(
        Number.parseInt(match[1], 16),
        index * 4,
        `${capturePath}:${index + 1}: non-contiguous address`
      );
      return `0x${match[2].toUpperCase()}`;
    });
}

function wordsToBuffer(words) {
  const buffer = Buffer.alloc(words.length * 4);
  words.forEach((word, index) => {
    buffer.writeUInt32LE(Number.parseInt(word.slice(2), 16), index * 4);
  });
  return buffer;
}

fs.mkdirSync(outputDir, { recursive: true });
for (const clockMhz of clocks) {
  for (const profile of golden.profiles) {
    const width = profile.output_width_bits;
    const expected = profile.memory_words.map(
      (word) => `0x${word.slice(2).toUpperCase()}`
    );
    let words = expected;
    let source = "checked DDR Golden";

    if (captureDir) {
      const capturePath = path.join(
        captureDir,
        `ddr_capture_${clockMhz}_${width}.hex`
      );
      words = parseCapture(capturePath);
      assert.deepStrictEqual(
        words,
        expected,
        `${clockMhz} MHz / ${width}-bit J9 capture differs from checked Golden`
      );
      source = capturePath;
    }

    const outputPath = path.join(outputDir, `ddr_${clockMhz}_${width}.bin`);
    const buffer = wordsToBuffer(words);
    assert.equal(
      buffer.length,
      profile.geometry.frame_allocation_bytes,
      `${clockMhz} MHz / ${width}-bit allocation size mismatch`
    );
    fs.writeFileSync(outputPath, buffer);
    manifest.push({
      processing_clock_mhz: clockMhz,
      output_width_bits: width,
      input_file: outputPath.replaceAll("\\", "/"),
      input_source: source.replaceAll("\\", "/"),
      bytes: buffer.length,
      geometry: profile.geometry,
    });
  }
}

fs.writeFileSync(
  path.join(outputDir, "ddr_fixture_manifest.json"),
  `${JSON.stringify(manifest, null, 2)}\n`,
  "ascii"
);
console.log("LIDAR_V2_PACKED17_DDR_MATERIALIZE_PASS");
