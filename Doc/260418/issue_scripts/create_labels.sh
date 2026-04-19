#!/bin/bash
set -e
GH="/c/Program Files/GitHub CLI/gh.exe"
R="haneulia/tdc_gpx_ctrl"

create_label() {
  local name="$1"; local color="$2"; local desc="$3"
  "$GH" -R "$R" label create "$name" --color "$color" --description "$desc" --force
}

# Priority
create_label "P1-critical"      "b60205" "High-confidence issue, fix first"
create_label "P2-important"     "d93f0b" "Risky assumption or observability gap"
create_label "P3-maintenance"   "fbca04" "Maintenance / interface drift"

# Category
create_label "fsm-bug"          "d73a4a" "FSM logic / sequencing problem"
create_label "cdc"              "5319e7" "Clock-domain crossing / reset"
create_label "observability"    "0e8a16" "Missing status / trace / port"
create_label "maintenance"      "c5def5" "Dead code / stale comments / naming"

# Area (per module)
for a in chip-run chip-ctrl config-ctrl cell-builder cmd-arb err-handler \
         face-assembler chip-init chip-reg header-inserter cell-pipe face-seq; do
  create_label "area-${a}" "ededed" "Module: tdc_gpx_${a//-/_}"
done

echo "--- labels done ---"
