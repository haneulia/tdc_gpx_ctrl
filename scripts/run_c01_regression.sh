#!/usr/bin/env bash
# =============================================================================
# run_c01_regression.sh
#   C01_GPX_Bus_Read 통합 회귀 entrypoint.
#
#   실행 항목:
#     1. tb_tdc_gpx_bus_phy_c01_contract  (C01 contract)
#     2. tb_tdc_gpx_bus_phy               (unit)
#     3. tb_tdc_gpx_chip_ctrl             (integration)
#     4. tb_tdc_gpx_config_ctrl  - g_DUT_STREAM_CLK_MODE=SYNC
#     5. tb_tdc_gpx_config_ctrl  - g_DUT_STREAM_CLK_MODE=ASYNC  (raw_cdc FIFO 활성)
#     6. tb_tdc_gpx_csr_chip_clamp        (CSR clamp via px_axi_lite_writer/reader)
#
#   허용 warning (R-C01-V002 기준):
#     - "bus_phy: bus timing clamped (div="
#     - "bus_phy: write request ignored (oen_permanent='1')"
#     - "VRFC 10-3532] module 'glbl' does not have a parameter named 'g_dut_stream_clk_mode'"
#       -> xelab의 -generic_top이 glbl에도 적용되어 발생. DUT generic 전달은 정상.
# =============================================================================
set -u

HERE="$(cd "$(dirname "$0")" && pwd)"
HDL_DIR="$(cd "${HERE}/.." && pwd)"

LOG_DIR="${HDL_DIR}/tmp/c01_verify"

mkdir -p "${LOG_DIR}"

# R-C01-V003-03: persist integrated console transcript for traceability.
# R-C01-V004-01 / V003-01 (Plan v006 #2): --negative branches transcript filename
# only. Failure injection is a separate concern handled by the env hook
# (C01_FORCE_NEGATIVE_STAGE1) inside tmp/c01_verify/run_regression.sh.
NEGATIVE_MODE=0
if [ "${1:-}" = "--negative" ]; then
    NEGATIVE_MODE=1
fi

if [ ${NEGATIVE_MODE} -eq 1 ]; then
    # Windows-safe ISO 8601 basic timestamp (no colons), Plan v006 P-C01-V003-01.
    TS="$(date -u '+%Y%m%dT%H%M%SZ')"
    TRANSCRIPT="${LOG_DIR}/run_c01_regression_negative_${TS}.log"
else
    TRANSCRIPT="${LOG_DIR}/run_c01_regression.log"
fi
exec > >(tee "${TRANSCRIPT}") 2>&1

echo "[run_c01_regression] start: $(date '+%Y-%m-%d %H:%M:%S %z')"
echo "[run_c01_regression] mode: $([ ${NEGATIVE_MODE} -eq 1 ] && echo negative || echo positive)"
echo "[run_c01_regression] transcript: ${TRANSCRIPT}"
echo ""
echo "############################################################"
echo "# Stage 1: 4 TB direct regression (bus_phy_c01, bus_phy,    "
echo "#          chip_ctrl, config_ctrl SYNC default)             "
echo "############################################################"
bash "${LOG_DIR}/run_regression.sh"
RC1=$?

echo ""
echo "############################################################"
echo "# Stage 2: config_ctrl SYNC + ASYNC two-mode regression     "
echo "############################################################"
bash "${LOG_DIR}/run_config_ctrl_two_modes.sh"
RC2=$?

echo ""
echo "############################################################"
echo "# Stage 3: CSR clamp regression (tb_tdc_gpx_csr_chip_clamp) "
echo "############################################################"
SIM_DIR="C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/tdc_gpx_ctrl.sim/sim_1/behav/xsim"
export PATH="/c/AMDDesignTools/2025.2.1/Vivado/bin:$PATH"
cd "${SIM_DIR}"

xelab --incr --debug typical --relax --mt 2 \
    -L xil_defaultlib -L unisims_ver -L unimacro_ver -L secureip -L xpm -L work \
    --snapshot tb_tdc_gpx_csr_chip_clamp_snap \
    xil_defaultlib.tb_tdc_gpx_csr_chip_clamp work.glbl \
    -log "${LOG_DIR}/elab_tb_tdc_gpx_csr_chip_clamp.log" >/dev/null 2>&1
RC_E=$?

cat > tb_tdc_gpx_csr_chip_clamp_run.tcl <<EOF
run 100us
quit
EOF

xsim tb_tdc_gpx_csr_chip_clamp_snap \
    -tclbatch tb_tdc_gpx_csr_chip_clamp_run.tcl \
    -log "${LOG_DIR}/sim_tb_tdc_gpx_csr_chip_clamp.log" >/dev/null 2>&1
RC_S=$?

if [ ${RC_E} -ne 0 ] || [ ${RC_S} -ne 0 ]; then
    echo "ELAB or SIM exit failure for csr_chip_clamp (elab=${RC_E}, sim=${RC_S})"
    RC3=1
elif grep -qE "severity failure|RUNTIME ERROR" "${LOG_DIR}/sim_tb_tdc_gpx_csr_chip_clamp.log"; then
    echo "SIM ASSERT FAIL for csr_chip_clamp"
    grep -E "severity failure|RUNTIME ERROR" "${LOG_DIR}/sim_tb_tdc_gpx_csr_chip_clamp.log" | head -3
    RC3=1
elif grep -qE "ALL TESTS PASSED" "${LOG_DIR}/sim_tb_tdc_gpx_csr_chip_clamp.log"; then
    marker=$(grep -E "ALL TESTS PASSED" "${LOG_DIR}/sim_tb_tdc_gpx_csr_chip_clamp.log")
    echo "SIM PASS: tb_tdc_gpx_csr_chip_clamp -- ${marker}"
    RC3=0
else
    echo "SIM RAN (no PASS marker) for csr_chip_clamp"
    RC3=1
fi

echo ""
echo "[run_c01_regression] end: $(date '+%Y-%m-%d %H:%M:%S %z')"
echo "############################################################"
if [ ${RC1} -eq 0 ] && [ ${RC2} -eq 0 ] && [ ${RC3} -eq 0 ]; then
    echo "# C01 regression: ALL PASS                                 "
    echo "############################################################"
    RC_FINAL=0
else
    echo "# C01 regression: FAIL (stage1=${RC1}, stage2=${RC2}, stage3=${RC3})"
    echo "############################################################"
    RC_FINAL=1
fi
# Plan v006 #1 / R-C01-V004-01: persist final integrated exit code in transcript.
echo "INTEGRATED EXIT CODE = ${RC_FINAL}"
exit ${RC_FINAL}
