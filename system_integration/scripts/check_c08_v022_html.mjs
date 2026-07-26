import fs from "node:fs";
import path from "node:path";
import vm from "node:vm";
import { fileURLToPath } from "node:url";

const scriptDir = path.dirname(fileURLToPath(import.meta.url));
const hdlRoot = path.resolve(scriptDir, "..", "..");
const htmlPath = path.join(
  hdlRoot,
  "Doc",
  "cluster_analysis",
  "C08_HDL_HTML_Alignment",
  "C08_HDL_HTML_Alignment_260727_External_GPX_I_Mode_Integration_Simulator_v022.html"
);
const defaultResultPaths = [
  path.join(
    hdlRoot,
    "sim_results",
    "vivado_xsim",
    "sessions",
    "260727_i_mode_final_internal_system_integration_smoke",
    "rtl_result.json"
  ),
  path.join(
    hdlRoot,
    "sim_results",
    "vivado_xsim",
    "sessions",
    "260727_i_mode_final_external_system_integration_smoke",
    "rtl_result.json"
  )
];
const requestedResultPaths = process.argv.slice(2).map(value => path.resolve(value));
const resultPaths = requestedResultPaths.length > 0
  ? requestedResultPaths
  : defaultResultPaths.filter(resultPath => fs.existsSync(resultPath));

const html = fs.readFileSync(htmlPath, "utf8");
const scriptMatch = html.match(/<script>\s*([\s\S]*?)\s*<\/script>/);
if (!scriptMatch) throw new Error("C08 HTML script block is missing");

function attribute(text, name) {
  return text.match(new RegExp(`\\b${name}="([^"]*)"`, "i"))?.[1] ?? "";
}

function makeContext2d() {
  return new Proxy({}, {
    get(_target, property) {
      if (property === "measureText") {
        return text => ({ width: String(text).length * 6 });
      }
      return () => undefined;
    },
    set() {
      return true;
    }
  });
}

function makeElement(id, attrs = "") {
  const classes = new Set(attribute(attrs, "class").split(/\s+/).filter(Boolean));
  const width = Number(attribute(attrs, "width")) || 720;
  const height = Number(attribute(attrs, "height")) || 320;
  const element = {
    id,
    value: attribute(attrs, "value"),
    checked: /\bchecked\b/i.test(attrs),
    disabled: /\bdisabled\b/i.test(attrs),
    className: [...classes].join(" "),
    dataset: {},
    files: [],
    innerHTML: "",
    textContent: "",
    title: attribute(attrs, "title"),
    style: {},
    width,
    height,
    clientWidth: width,
    clientHeight: height,
    parentElement: null,
    classList: {
      add: (...names) => names.forEach(name => classes.add(name)),
      remove: (...names) => names.forEach(name => classes.delete(name)),
      toggle: (name, force) => {
        const enabled = force === undefined ? !classes.has(name) : !!force;
        if (enabled) classes.add(name);
        else classes.delete(name);
        return enabled;
      }
    },
    addEventListener: () => undefined,
    setCustomValidity: message => { element.validationMessage = message; },
    getContext: () => makeContext2d(),
    getBoundingClientRect: () => ({ left: 0, top: 0, width, height }),
    closest: () => ({ querySelector: () => makeElement(`${id}-header`) }),
    querySelector: () => makeElement(`${id}-child`),
    querySelectorAll: () => []
  };
  element.parentElement = {
    clientWidth: width,
    clientHeight: height,
    getBoundingClientRect: element.getBoundingClientRect
  };
  return element;
}

const elements = new Map();
for (const match of html.matchAll(/<([a-z0-9]+)\b([^>]*\bid="([^"]+)"[^>]*)>/gi)) {
  elements.set(match[3], makeElement(match[3], match[2]));
}
for (const match of html.matchAll(/<select\b([^>]*\bid="([^"]+)"[^>]*)>([\s\S]*?)<\/select>/gi)) {
  const options = [...match[3].matchAll(/<option\b([^>]*)>([\s\S]*?)<\/option>/gi)];
  const selected = options.find(option => /\bselected\b/i.test(option[1])) ?? options[0];
  const value = selected ? (attribute(selected[1], "value") || selected[2].trim()) : "";
  (elements.get(match[2]) ?? makeElement(match[2], match[1])).value = value;
}

const facetButtons = [1, 2, 3, 4, 5].map(facet => {
  const button = makeElement(`facet-${facet}`);
  button.dataset.facets = String(facet);
  return button;
});
const handoffTables = [0, 1].map(index => ({
  tBodies: [{ rows: [makeElement(`handoff-row-${index}`)] }]
}));
const documentElement = { dataset: {} };
const document = {
  documentElement,
  getElementById(id) {
    if (!elements.has(id)) elements.set(id, makeElement(id));
    return elements.get(id);
  },
  querySelector(selector) {
    const id = selector.match(/^#([A-Za-z0-9_-]+)/)?.[1];
    return id ? this.getElementById(id) : null;
  },
  querySelectorAll(selector) {
    if (selector === "#facetSeg button") return facetButtons;
    if (selector === ".handoff-contract-table") return handoffTables;
    if (selector === ".table-lang-toggle") {
      return [...elements.values()].filter(element => element.className.includes("table-lang-toggle"));
    }
    if (selector === ".table-freeze-toggle") {
      return [...elements.values()].filter(element => element.className.includes("table-freeze-toggle"));
    }
    return [];
  }
};

const localStorage = {
  getItem: () => null,
  setItem: () => undefined
};
const sandbox = {
  console,
  document,
  localStorage,
  location: { search: "" },
  performance: { now: () => 0 },
  requestAnimationFrame: () => 0,
  URLSearchParams,
  setTimeout,
  clearTimeout
};
sandbox.window = sandbox;
sandbox.globalThis = sandbox;
sandbox.addEventListener = () => undefined;
vm.createContext(sandbox);
vm.runInContext(scriptMatch[1], sandbox, { filename: htmlPath });

const selfTests = {
  slope: sandbox.__C08_SLOPE_CONTRACT_SELF_TEST__,
  timing: sandbox.__C08_CLOCK_TIMING_SELF_TEST__,
  pins: sandbox.__C08_PHYSICAL_PIN_CONTRACT_SELF_TEST__,
  fifo: sandbox.__C08_GPX_FIFO_OWNERSHIP_SELF_TEST__,
  handoff: sandbox.__C08_IP_HANDOFF_CONTRACT_SELF_TEST__,
  iMode: sandbox.__C08_I_MODE_OWNERSHIP_SELF_TEST__,
  rtlComparison: sandbox.__C08_RTL_CONTRACT_COMPARISON_SELF_TEST__
};
for (const [name, result] of Object.entries(selfTests)) {
  if (!result?.pass) {
    throw new Error(`${name} self-test failed: ${(result?.errors ?? []).join(", ")}`);
  }
}

for (const resultPath of resultPaths) {
  if (!fs.existsSync(resultPath)) {
    throw new Error(`RTL result does not exist: ${resultPath}`);
  }
  const contract = JSON.parse(fs.readFileSync(resultPath, "utf8").replace(/^\uFEFF/, ""));
  sandbox.__C08_APPLY_RTL_CONTRACT_SCENARIO__(contract);
  const model = sandbox.__C08_RENDER__();
  const comparison = sandbox.__C08_BUILD_RTL_CONTRACT_COMPARISON__(model, contract);
  if (!comparison.pass) {
    throw new Error(`${path.basename(path.dirname(resultPath))}: ${comparison.firstFailure || comparison.error}`);
  }
  const iModeRows = comparison.rows.filter(row => row.label.includes("I-Mode"));
  if (iModeRows.length !== 2 || iModeRows.some(row => !row.pass)) {
    throw new Error(`${path.basename(path.dirname(resultPath))}: I-Mode rows did not pass`);
  }
  console.log(`PASS ${path.basename(path.dirname(resultPath))}: ${iModeRows.map(row => row.label).join("; ")}`);
}

if (resultPaths.length === 0) {
  console.log("PASS HTML self-tests; no local RTL result archives were found");
}
console.log("C08_V022_HTML_SELF_TEST_PASS");
