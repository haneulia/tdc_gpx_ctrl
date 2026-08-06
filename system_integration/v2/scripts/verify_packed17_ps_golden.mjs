import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";
import vm from "node:vm";

if (process.argv.length < 4 || process.argv.length > 5) {
  throw new Error(
    "usage: node verify_packed17_ps_golden.mjs <html> <golden-json> " +
      "[--update|capture-dir]"
  );
}

const htmlPath = process.argv[2];
const goldenPath = process.argv[3];
const mode = process.argv[4];
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
    "globalThis.__psGolden = {" +
    "schema:'tdc-gpx-packed17-ps-ethernet-golden-v1'," +
    "generated_by:'C08-v026 buildJ10PsGolden'," +
    "scenario:'J9 DDR image normalized to Face Header and one H-Line'," +
    "profiles:[150,200].map(buildJ10PsGolden)};",
  context,
  { filename: htmlPath }
);

const htmlGolden = JSON.parse(JSON.stringify(context.__psGolden));
if (mode === "--update") {
  fs.writeFileSync(goldenPath, `${JSON.stringify(htmlGolden, null, 2)}\n`, "ascii");
  console.log("LIDAR_V2_PS_HTML_GOLDEN_UPDATED");
  process.exit(0);
}

const checkedGolden = JSON.parse(fs.readFileSync(goldenPath, "utf8"));
assert.deepStrictEqual(
  htmlGolden,
  checkedGolden,
  "Checked PS/Ethernet Golden differs from the executable HTML model"
);

function packetBytes(packet) {
  assert.equal(
    packet.hex.length,
    packet.length_bytes * 2,
    `${packet.kind}: hex length mismatch`
  );
  return Buffer.from(packet.hex, "hex");
}

function parseCapture(capturePath) {
  const bytes = fs.readFileSync(capturePath);
  const packets = [];
  let offset = 0;

  while (offset < bytes.length) {
    assert.ok(offset + 4 <= bytes.length, `${capturePath}: truncated packet length`);
    const length = bytes.readUInt32LE(offset);
    offset += 4;
    assert.ok(length <= 1440, `${capturePath}: packet exceeds 1440 bytes`);
    assert.ok(offset + length <= bytes.length, `${capturePath}: truncated packet payload`);
    packets.push(bytes.subarray(offset, offset + length));
    offset += length;
  }
  return packets;
}

for (const profile of checkedGolden.profiles) {
  const packets = profile.packets.map(packetBytes);
  assert.equal(packets[0].length, 1440, "Face Header must occupy one 1440-byte payload");
  assert.equal(packets[0].subarray(0, 4).toString("ascii"), "LFH1");
  assert.equal(packets[1].subarray(0, 4).toString("ascii"), "LHL1");
  assert.equal(packets[1].readUInt8(28), 3, "H-Line sample width must be 3 bytes");
  assert.equal(packets[1].subarray(32, 35).toString("hex"), "010003");
  assert.equal(packets[1].subarray(35, 38).toString("hex"), "000004");
}
console.log("LIDAR_V2_PS_HTML_GOLDEN_PASS");

if (mode) {
  const captureDir = mode;
  for (const profile of checkedGolden.profiles) {
    const clockMhz = profile.processing_clock_mhz;
    const expectedPackets = profile.packets.map(packetBytes);
    const widthCaptures = [];

    for (const width of [32, 64, 128]) {
      const capturePath = path.join(
        captureDir,
        `ps_capture_${clockMhz}_${width}.pkt`
      );
      const actualPackets = parseCapture(capturePath);
      assert.equal(
        actualPackets.length,
        expectedPackets.length,
        `${clockMhz} MHz / ${width}-bit packet-count mismatch`
      );
      actualPackets.forEach((actual, packetIndex) => {
        assert.deepStrictEqual(
          actual,
          expectedPackets[packetIndex],
          `${clockMhz} MHz / ${width}-bit packet ${packetIndex} byte mismatch`
        );
      });
      widthCaptures.push(fs.readFileSync(capturePath));
    }

    assert.deepStrictEqual(
      widthCaptures[0],
      widthCaptures[1],
      `${clockMhz} MHz: 32/64-bit PS payload differs`
    );
    assert.deepStrictEqual(
      widthCaptures[0],
      widthCaptures[2],
      `${clockMhz} MHz: 32/128-bit PS payload differs`
    );
  }
  console.log("LIDAR_V2_PS_HLINE_ETHERNET_COMPARE_PASS");
}
