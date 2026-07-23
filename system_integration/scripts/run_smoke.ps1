param(
    [string]$Scenario = "",
    [ValidateSet("", "internal", "external")]
    [string]$EncoderSource = "",
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss")
)

$ErrorActionPreference = "Stop"

$Hdl = (Resolve-Path (Join-Path $PSScriptRoot "../..")).Path
$IpRoot = "C:/Projects/my_sp/lib/IP"
$TdcRoot = "$IpRoot/tdc_gpx_ctrl"
$Vivado = "C:/AMDDesignTools/2025.2.1/Vivado/bin"

if ([string]::IsNullOrWhiteSpace($Scenario)) {
    $Scenario = Join-Path $Hdl "system_integration/scenarios/smoke_internal_200m_v001.json"
}
$Scenario = (Resolve-Path $Scenario).Path
$Cfg = Get-Content -Raw -LiteralPath $Scenario | ConvertFrom-Json

# Schema-v1 compatibility: an older scenario with one clock remains a
# same-clock run. New scenarios state both domains explicitly.
if ($null -eq $Cfg.tdc_clock_mhz) {
    $Cfg | Add-Member -NotePropertyName tdc_clock_mhz `
        -NotePropertyValue $Cfg.axis_clock_mhz
}

$SupportedClocksMhz = @(50, 100, 125, 150, 200)
if ([double]$Cfg.axis_clock_mhz -notin $SupportedClocksMhz) {
    throw "Unsupported axis_clock_mhz '$($Cfg.axis_clock_mhz)'"
}
if ([double]$Cfg.tdc_clock_mhz -notin $SupportedClocksMhz) {
    throw "Unsupported tdc_clock_mhz '$($Cfg.tdc_clock_mhz)'"
}
if ([double]$Cfg.axis_clock_mhz -gt [double]$Cfg.tdc_clock_mhz) {
    throw "axis_clock_mhz must not exceed tdc_clock_mhz"
}

if (-not [string]::IsNullOrWhiteSpace($EncoderSource)) {
    $Cfg.encoder_source = $EncoderSource
}

$Work = Join-Path $Hdl "tmp/system_integration/$Stamp"
$Archive = Join-Path $Hdl "sim_results/vivado_xsim/sessions/${Stamp}_system_integration_smoke"
New-Item -ItemType Directory -Force -Path $Work, $Archive | Out-Null

function Invoke-Checked {
    param(
        [string]$Exe,
        [string[]]$ArgList
    )
    & $Exe @ArgList
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed ($LASTEXITCODE): $Exe $($ArgList -join ' ')"
    }
}

function Assert-FileExists {
    param([string[]]$Files)
    foreach ($File in $Files) {
        if (-not (Test-Path -LiteralPath $File -PathType Leaf)) {
            throw "Missing integration dependency: $File"
        }
    }
}

$Csr7 = "$TdcRoot/tdc_gpx_ctrl.gen/sources_1/ip/my_axil_csr_0"
$Csr32 = "$TdcRoot/tdc_gpx_ctrl.gen/sources_1/ip/echo_receiver_axil_csr32"
$GeneratedFiles = @(
    "$Csr7/src/axil_ctrl_regs.vhd",
    "$Csr7/src/axil_fsm.vhd",
    "$Csr7/src/axil_intr.vhd",
    "$Csr7/src/axil_stat_regs.vhd",
    "$Csr7/src/my_axil_csr_top.vhd",
    "$Csr7/sim/my_axil_csr_0.vhd",
    "$TdcRoot/tdc_gpx_ctrl.gen/sources_1/ip/laser_ctl_axil_csr/sim/laser_ctl_axil_csr.vhd",
    "$TdcRoot/tdc_gpx_ctrl.gen/sources_1/ip/tdc_gpx_axil_csr_pipeline/sim/tdc_gpx_axil_csr_pipeline.vhd",
    "$Csr32/src/axil_ctrl_regs_32.vhd",
    "$Csr32/src/axil_fsm_32.vhd",
    "$Csr32/src/axil_intr_32.vhd",
    "$Csr32/src/axil_stat_regs_32.vhd",
    "$Csr32/src/my_axil_csr32_top.vhd",
    "$Csr32/sim/echo_receiver_axil_csr32.vhd",
    "$TdcRoot/tdc_gpx_ctrl.gen/sources_1/ip/tdc_gpx_axil_csr32_chip/sim/tdc_gpx_axil_csr32_chip.vhd"
)

$MotorFiles = @(
    "$IpRoot/virtual_encoder/HDL/enc_pkg.vhd",
    "$IpRoot/virtual_encoder/HDL/enc_param_apply_ctrl.vhd",
    "$IpRoot/virtual_encoder/HDL/enc_phase_counter.vhd",
    "$IpRoot/virtual_encoder/HDL/enc_position_tracker.vhd",
    "$IpRoot/virtual_encoder/HDL/enc_index_pulse.vhd",
    "$IpRoot/virtual_encoder/HDL/enc_timing_generator.vhd",
    "$IpRoot/virtual_encoder/HDL/enc_top.vhd",
    "$IpRoot/motor_decoder/HDL/quad_decoder.vhd",
    "$IpRoot/motor_decoder/HDL/mirror_active_detect.vhd",
    "$IpRoot/motor_decoder/HDL/motor_irq_bridge.vhd",
    "$IpRoot/motor_decoder/HDL/motor_decoder_cfg_pkg.vhd",
    "$IpRoot/motor_decoder/HDL/motor_cfg_commit_ctrl.vhd",
    "$IpRoot/motor_decoder/HDL/motor_axis_stream_out.vhd",
    "$IpRoot/motor_decoder/HDL/motor_decoder_csr.vhd",
    "$IpRoot/motor_decoder/HDL/motor_decoder_top.vhd"
)

$LaserFiles = @(
    "$IpRoot/laser_ctrl/HDL/laser_ctrl_types_pkg.vhd",
    "$IpRoot/laser_ctrl/HDL/laser_ctrl_cfg_pkg.vhd",
    "$IpRoot/laser_ctrl/HDL/laser_ctrl_math_pkg.vhd",
    "$IpRoot/laser_ctrl/HDL/laser_ctrl_pkg.vhd",
    "$IpRoot/laser_ctrl/HDL/laser_ctrl_status.vhd",
    "$IpRoot/laser_ctrl/HDL/laser_ctrl_tdc.vhd",
    "$IpRoot/laser_ctrl/HDL/laser_ctrl_result.vhd",
    "$IpRoot/laser_ctrl/HDL/laser_ctrl_fire_done_bridge.vhd",
    "$IpRoot/laser_ctrl/HDL/laser_ctrl_scheduler.vhd",
    "$IpRoot/laser_ctrl/HDL/laser_ctrl_csr.vhd",
    "$IpRoot/laser_ctrl/HDL/laser_ctrl_metrics.vhd",
    "$IpRoot/laser_ctrl/HDL/laser_ctrl_axis_in.vhd",
    "$IpRoot/laser_ctrl/HDL/laser_ctrl_executor.vhd",
    "$IpRoot/laser_ctrl/HDL/laser_ctrl_top.vhd",
    "$IpRoot/laser_ctrl/HDL/tb_laser_ctrl_pkg.vhd",
    "$IpRoot/laser_ctrl/HDL/tb_laser_ctrl_tests_pkg.vhd"
)

$EchoFiles = @(
    "$IpRoot/echo_receiver/HDL/echo_receiver_pkg.vhd",
    "$IpRoot/echo_receiver/HDL/echo_receiver_core.vhd",
    "$IpRoot/echo_receiver/HDL/echo_receiver_csr.vhd",
    "$IpRoot/echo_receiver/HDL/echo_receiver_top.vhd"
)

$TdcFiles = @(
    "$Hdl/px_utility_pkg.vhd",
    "$Hdl/tdc_gpx_pkg.vhd",
    "$Hdl/tdc_gpx_cfg_pkg.vhd",
    "$Hdl/tdc_gpx_atomic_snapshot_cdc.vhd",
    "$Hdl/tdc_gpx_bus_phy.vhd",
    "$Hdl/tdc_gpx_skid_buffer.vhd",
    "$Hdl/tdc_gpx_sync_fifo.vhd",
    "$Hdl/tdc_gpx_cell_builder.vhd",
    "$Hdl/tdc_gpx_cell_pipe.vhd",
    "$Hdl/tdc_gpx_chip_init.vhd",
    "$Hdl/tdc_gpx_chip_run.vhd",
    "$Hdl/tdc_gpx_chip_reg.vhd",
    "$Hdl/tdc_gpx_chip_ctrl.vhd",
    "$Hdl/tdc_gpx_csr_chip.vhd",
    "$Hdl/tdc_gpx_cmd_arb.vhd",
    "$Hdl/tdc_gpx_err_handler.vhd",
    "$Hdl/tdc_gpx_cfg_image_override.vhd",
    "$Hdl/tdc_gpx_reg_rsp_cdc.vhd",
    "$Hdl/tdc_gpx_config_ctrl.vhd",
    "$Hdl/tdc_gpx_decoder_i_mode.vhd",
    "$Hdl/tdc_gpx_raw_event_builder.vhd",
    "$Hdl/tdc_gpx_decode_pipe.vhd",
    "$Hdl/tdc_gpx_face_assembler.vhd",
    "$Hdl/tdc_gpx_line_packer.vhd",
    "$Hdl/tdc_gpx_header_inserter.vhd",
    "$Hdl/tdc_gpx_face_seq.vhd",
    "$Hdl/tdc_gpx_output_stage.vhd",
    "$Hdl/tdc_gpx_csr_pipeline.vhd",
    "$Hdl/tdc_gpx_status_agg.vhd",
    "$Hdl/tdc_gpx_top.vhd",
    "$Hdl/tb_tdc_gpx_full_int.vhd"
)

$CanonicalFiles = @($MotorFiles + $LaserFiles + $EchoFiles + $TdcFiles)
Assert-FileExists @($GeneratedFiles + $CanonicalFiles)

$VhdlProject = Join-Path $Work "system_smoke_vhdl.prj"
$VlogProject = Join-Path $Work "system_smoke_vlog.prj"
$VhdlLines = @()
foreach ($File in $GeneratedFiles) {
    $VhdlLines += "vhdl xil_defaultlib `"$File`""
}
foreach ($File in $MotorFiles + $LaserFiles + $EchoFiles + $TdcFiles) {
    $VhdlLines += "vhdl2008 xil_defaultlib `"$File`""
}
$VhdlLines += "nosort"
$VhdlLines | Set-Content -Encoding ASCII -LiteralPath $VhdlProject

$Glbl = "$TdcRoot/tdc_gpx_ctrl.sim/sim_1/behav/xsim/glbl.v"
Assert-FileExists @($Glbl)
@(
    "verilog xil_defaultlib `"$Glbl`"",
    "nosort"
) | Set-Content -Encoding ASCII -LiteralPath $VlogProject

$CompileLog = Join-Path $Work "xvhdl.log"
$ElabLog = Join-Path $Work "xelab.log"
$SimLog = Join-Path $Work "xsim.log"
$Snapshot = "system_smoke_${Stamp}_snap"

Push-Location $Work
try {
    Invoke-Checked "$Vivado/xvlog.bat" @(
        "--relax", "-prj", $VlogProject, "-log", (Join-Path $Work "xvlog.log")
    )
    Invoke-Checked "$Vivado/xvhdl.bat" @(
        "--relax", "-prj", $VhdlProject, "-log", $CompileLog
    )

    $ElabArgFile = Join-Path $Work "xelab.f"
    @(
        "--debug typical",
        "--relax",
        "--mt 2",
        "-L xil_defaultlib",
        "-L unisims_ver",
        "-L unimacro_ver",
        "-L secureip",
        "-L xpm",
        "--snapshot $Snapshot",
        "--generic_top `"G_AXIS_CLK_MHZ=$($Cfg.axis_clock_mhz)`"",
        "--generic_top `"G_TDC_CLK_MHZ=$($Cfg.tdc_clock_mhz)`"",
        "--generic_top `"G_MAX_RANGE_M=$($Cfg.max_range_m)`"",
        "--generic_top `"G_SIM_TARGET_M=$($Cfg.target_distance_m)`"",
        "--generic_top `"G_TDATA_WIDTH=$($Cfg.output_width_bits)`"",
        "--generic_top `"G_STOPS_PER_CHIP=$($Cfg.stops_per_chip)`"",
        "--generic_top `"G_COLS_PER_FACE=$($Cfg.columns_per_face)`"",
        "--generic_top `"G_N_FACES=$($Cfg.faces_per_frame)`"",
        "--generic_top `"G_ACTIVE_CHIP_MASK=4'b$($Cfg.active_chip_mask)`"",
        "--generic_top `"G_CHIP_SLOPE_MASK=4'b$($Cfg.chip_slope_mask)`"",
        "--generic_top `"G_ENCODER_SOURCE=$($Cfg.encoder_source)`"",
        "--generic_top `"G_TDC_STIM_MODE=$($Cfg.tdc_stimulus_mode)`"",
        "--generic_top `"G_BP_TREADY_GAP=$($Cfg.backpressure_gap_clocks)`"",
        "--generic_top `"G_ENC_RUN_US=$($Cfg.encoder_observation_us)`"",
        "xil_defaultlib.tb_tdc_gpx_full_int",
        "xil_defaultlib.glbl",
        "-log `"$ElabLog`""
    ) | Set-Content -Encoding ASCII -LiteralPath $ElabArgFile
    Invoke-Checked "$Vivado/xelab.bat" @("-f", $ElabArgFile)
    Invoke-Checked "$Vivado/xsim.bat" @(
        $Snapshot, "-runall", "-log", $SimLog
    )
}
finally {
    Pop-Location
}

if (Select-String -LiteralPath $SimLog -Pattern "Failure:|Fatal:|assertion error|ERROR:" -CaseSensitive:$false -Quiet) {
    throw "System integration smoke log contains a failure marker: $SimLog"
}
if (-not (Select-String -LiteralPath $SimLog -Pattern "SYSTEM_INTEGRATION_SMOKE_PASS" -Quiet)) {
    throw "System integration smoke PASS marker is missing: $SimLog"
}

$MetricLine = (Select-String -LiteralPath $SimLog -Pattern "RTL_RESULT " | Select-Object -Last 1).Line
if ([string]::IsNullOrWhiteSpace($MetricLine)) {
    throw "RTL_RESULT marker is missing: $SimLog"
}

$Metrics = [ordered]@{}
foreach ($Match in [regex]::Matches($MetricLine, '([a-z][a-z0-9_]*)=([^\s]+)')) {
    $Key = $Match.Groups[1].Value
    $Value = $Match.Groups[2].Value
    if ($Key -in @("stat5", "stat6", "stat7")) {
        # CSR status words are a fixed-width bit contract. Keep all three as
        # eight-digit hex strings so JSON consumers never see value-dependent
        # number/string type changes or lost leading zeroes.
        if ($Value -notmatch '^[0-9A-Fa-f]{8}$') {
            throw "Malformed $Key status word in RTL_RESULT: $Value"
        }
        $Metrics[$Key] = $Value.ToUpperInvariant()
    }
    elseif ($Value -match '^-?\d+$') {
        $Metrics[$Key] = [int64]$Value
    }
    else {
        $Metrics[$Key] = $Value
    }
}

$SourceEntries = foreach ($File in @($CanonicalFiles + $GeneratedFiles)) {
    $Hash = Get-FileHash -Algorithm SHA256 -LiteralPath $File
    [ordered]@{
        path = $File.Replace('\', '/')
        sha256 = $Hash.Hash.ToLowerInvariant()
    }
}

$TdcCommit = (& git -C $Hdl rev-parse HEAD).Trim()
$TdcWorktreeDirty = -not [string]::IsNullOrWhiteSpace(
    ((& git -C $Hdl status --porcelain) -join "`n")
)
$Result = [ordered]@{
    schema_version = 2
    scenario_source = $Scenario.Replace('\', '/')
    scenario = $Cfg
    verdict = "PASS"
    pass_marker = "SYSTEM_INTEGRATION_SMOKE_PASS"
    tdc_git_commit = $TdcCommit
    tdc_git_worktree_dirty = $TdcWorktreeDirty
    metrics = $Metrics
    source_manifest = $SourceEntries
}

$ResultPath = Join-Path $Archive "rtl_result.json"
$Result | ConvertTo-Json -Depth 8 | Set-Content -Encoding UTF8 -LiteralPath $ResultPath

# Compact machine contract for the C08 HTML simulator. Reproducibility hashes
# and logs remain in rtl_result.json; the UI does not need to load that payload.
$ContractResult = [ordered]@{
    schema_version = 2
    scenario_id = $Cfg.scenario_id
    verdict = "PASS"
    scenario = $Cfg
    metrics = $Metrics
}
$ContractResult | ConvertTo-Json -Depth 8 | Set-Content -Encoding UTF8 `
    -LiteralPath (Join-Path $Archive "rtl_contract.json")
$Cfg | ConvertTo-Json -Depth 8 | Set-Content -Encoding UTF8 `
    -LiteralPath (Join-Path $Archive "scenario.json")
Copy-Item -LiteralPath (Join-Path $Work "xvlog.log"), $CompileLog, $ElabLog, $SimLog -Destination $Archive

Write-Host "SYSTEM_INTEGRATION_SMOKE_PASS"
Write-Host "Result: $ResultPath"
