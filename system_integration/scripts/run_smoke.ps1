param(
    [string]$Scenario = "",
    [ValidateSet("", "internal", "external")]
    [string]$EncoderSource = "",
    [ValidateSet(0, 32, 64, 128)]
    [int]$OutputWidth = 0,
    [ValidateRange(0, 7)]
    [int]$ReturnsPerStop = 0,
    [ValidateRange(0, 7)]
    [int]$MaxHits = 0,
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss")
)

$ErrorActionPreference = "Stop"

$Hdl = (Resolve-Path (Join-Path $PSScriptRoot "../..")).Path
$IpRoot = "C:/Projects/my_sp/lib/IP"
$TdcRoot = "$IpRoot/tdc_gpx_ctrl"
$Vivado = "C:/AMDDesignTools/2025.2.1/Vivado/bin"

if ([string]::IsNullOrWhiteSpace($Scenario)) {
    $Scenario = Join-Path $Hdl "system_integration/scenarios/smoke_internal_axis150_tdc200_v001.json"
}
$Scenario = (Resolve-Path $Scenario).Path
$Cfg = Get-Content -Raw -LiteralPath $Scenario | ConvertFrom-Json

# Schema-v1 compatibility: an older scenario with one clock remains a
# same-clock run. New scenarios state both domains explicitly.
if ($null -eq $Cfg.tdc_clock_mhz) {
    $Cfg | Add-Member -NotePropertyName tdc_clock_mhz `
        -NotePropertyValue $Cfg.axis_clock_mhz
}

$ScenarioDefaults = [ordered]@{
    revolution_period_us = 100.0
    optical_shot_interval_deg = 36.5
    returns_per_stop = 1
    max_hits_cfg = 3
    tdc_drain_margin_time_ns = 6000
    echo_stimulus_mode = "synthetic_single"
    rearm_guard_5ns_ticks = 0
}
foreach ($Entry in $ScenarioDefaults.GetEnumerator()) {
    if ($null -eq $Cfg.($Entry.Key)) {
        $Cfg | Add-Member -NotePropertyName $Entry.Key `
            -NotePropertyValue $Entry.Value
    }
}

# Keep product operating requirements separate from the accelerated encoder
# profile used to shorten xsim runtime. Legacy scenarios represent themselves.
if ($null -eq $Cfg.operating_motor_rpm) {
    $Cfg | Add-Member -NotePropertyName operating_motor_rpm `
        -NotePropertyValue (60000000.0 / [double]$Cfg.revolution_period_us)
}
if ($null -eq $Cfg.horizontal_resolution_deg) {
    $Cfg | Add-Member -NotePropertyName horizontal_resolution_deg `
        -NotePropertyValue ([double]$Cfg.optical_shot_interval_deg)
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

if ($OutputWidth -ne 0) {
    $Cfg.output_width_bits = $OutputWidth
    $Cfg.scenario_id = "$($Cfg.scenario_id)_w${OutputWidth}"
}
if ($ReturnsPerStop -ne 0) {
    $Cfg.returns_per_stop = $ReturnsPerStop
    $Cfg.scenario_id = "$($Cfg.scenario_id)_r${ReturnsPerStop}"
}
if ($MaxHits -ne 0) {
    $Cfg.max_hits_cfg = $MaxHits
    $Cfg.scenario_id = "$($Cfg.scenario_id)_mh${MaxHits}"
}
$SupportedOutputWidths = @(32, 64, 128)
if ([int]$Cfg.output_width_bits -notin $SupportedOutputWidths) {
    throw "Unsupported output_width_bits '$($Cfg.output_width_bits)'; use 32, 64, or 128"
}
if ([double]$Cfg.revolution_period_us -le 0.0) {
    throw "revolution_period_us must be positive"
}
if ([double]$Cfg.optical_shot_interval_deg -le 0.0 -or
    [double]$Cfg.optical_shot_interval_deg -gt 360.0) {
    throw "optical_shot_interval_deg must be in (0, 360]"
}
if ([double]$Cfg.operating_motor_rpm -le 0.0) {
    throw "operating_motor_rpm must be positive"
}
if ([double]$Cfg.horizontal_resolution_deg -le 0.0 -or
    [double]$Cfg.horizontal_resolution_deg -gt 360.0) {
    throw "horizontal_resolution_deg must be in (0, 360]"
}
if ([int]$Cfg.returns_per_stop -lt 1 -or [int]$Cfg.returns_per_stop -gt 7) {
    throw "returns_per_stop must be in 1..7"
}
if ([int]$Cfg.max_hits_cfg -lt 1 -or [int]$Cfg.max_hits_cfg -gt 7) {
    throw "max_hits_cfg must be in 1..7"
}
if ([int]$Cfg.tdc_drain_margin_time_ns -le 0) {
    throw "tdc_drain_margin_time_ns must be positive"
}
if ([int]$Cfg.returns_per_stop -gt [int]$Cfg.max_hits_cfg) {
    throw "returns_per_stop must not exceed max_hits_cfg"
}
if ([string]$Cfg.echo_stimulus_mode -notin @("synthetic_single", "physical_multi")) {
    throw "echo_stimulus_mode must be synthetic_single or physical_multi"
}
if ([string]$Cfg.echo_stimulus_mode -eq "physical_multi" -and
    [int]$Cfg.stops_per_chip -ne 8) {
    throw "physical_multi requires stops_per_chip=8"
}
if ([int]$Cfg.rearm_guard_5ns_ticks -lt 0 -or
    [int]$Cfg.rearm_guard_5ns_ticks -gt 65535) {
    throw "rearm_guard_5ns_ticks must be in 0..65535"
}

$OperatingRevUs = 60000000.0 / [double]$Cfg.operating_motor_rpm
$OperatingPointIntervalUs = $OperatingRevUs * `
    [double]$Cfg.horizontal_resolution_deg / 720.0
$SimulationPointIntervalUs = [double]$Cfg.revolution_period_us * `
    [double]$Cfg.optical_shot_interval_deg / 720.0
$PointIntervalToleranceUs = 1.0 / [double]$Cfg.axis_clock_mhz
if ([math]::Abs($OperatingPointIntervalUs - $SimulationPointIntervalUs) -gt
    ($PointIntervalToleranceUs + 1.0e-9)) {
    throw "Accelerated simulation point interval does not match operating RPM/resolution: operating=$OperatingPointIntervalUs us simulation=$SimulationPointIntervalUs us"
}
$Cfg | Add-Member -Force -NotePropertyName operating_point_interval_us `
    -NotePropertyValue $OperatingPointIntervalUs
$Cfg | Add-Member -Force -NotePropertyName simulation_point_interval_us `
    -NotePropertyValue $SimulationPointIntervalUs

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

$CsrFiles = @(
    "$IpRoot/my_axil_csr/HDL/axil_fsm.vhd",
    "$IpRoot/my_axil_csr/HDL/axil_ctrl_regs.vhd",
    "$IpRoot/my_axil_csr/HDL/axil_stat_regs.vhd",
    "$IpRoot/my_axil_csr/HDL/axil_intr.vhd",
    "$IpRoot/my_axil_csr/HDL/my_axil_csr_top.vhd",
    "$IpRoot/my_axil_csr32/HDL/my_axil_csr32_pkg.vhd",
    "$IpRoot/my_axil_csr32/HDL/axil_fsm_32.vhd",
    "$IpRoot/my_axil_csr32/HDL/axil_ctrl_regs_32.vhd",
    "$IpRoot/my_axil_csr32/HDL/axil_stat_regs_32.vhd",
    "$IpRoot/my_axil_csr32/HDL/axil_intr_32.vhd",
    "$IpRoot/my_axil_csr32/HDL/my_axil_csr32_top.vhd"
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
    "$IpRoot/laser_ctrl/HDL/laser_ctrl_cfg_pkg.vhd",
    "$IpRoot/laser_ctrl/HDL/laser_ctrl_types_pkg.vhd",
    "$IpRoot/laser_ctrl/HDL/laser_ctrl_timebase.vhd",
    "$IpRoot/laser_ctrl/HDL/laser_ctrl_cdc_snapshot.vhd",
    "$IpRoot/laser_ctrl/HDL/laser_ctrl_event_counters.vhd",
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
    "$IpRoot/motor_laser_ctrl/HDL/motor_laser_ctrl_top.vhd",
    "$IpRoot/laser_ctrl/HDL/tb_laser_ctrl_pkg.vhd",
    "$IpRoot/laser_ctrl/HDL/tb_laser_ctrl_tests_pkg.vhd"
)

$EchoFiles = @(
    "$IpRoot/echo_receiver/HDL/echo_receiver_pkg.vhd",
    "$IpRoot/echo_receiver/HDL/echo_receiver_timebase.vhd",
    "$IpRoot/echo_receiver/HDL/echo_receiver_stop_frontend.vhd",
    "$IpRoot/echo_receiver/HDL/echo_receiver_core.vhd",
    "$IpRoot/echo_receiver/HDL/echo_receiver_cdc_snapshot.vhd",
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
    "$Hdl/system_integration/tb/tdc_gpx_external_chip_model.vhd",
    "$Hdl/tb_tdc_gpx_full_int.vhd"
)

$CanonicalFiles = @($CsrFiles + $MotorFiles + $LaserFiles + $EchoFiles + $TdcFiles)
Assert-FileExists $CanonicalFiles

$VhdlProject = Join-Path $Work "system_smoke_vhdl.prj"
$VlogProject = Join-Path $Work "system_smoke_vlog.prj"
$VhdlLines = @()
foreach ($File in $CanonicalFiles) {
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
        "--generic_top `"G_REV_TIME_US=$($Cfg.revolution_period_us)`"",
        "--generic_top `"G_OPTICAL_SHOT_INTERVAL_DEG=$($Cfg.optical_shot_interval_deg)`"",
        "--generic_top `"G_OPERATING_MOTOR_RPM=$($Cfg.operating_motor_rpm)`"",
        "--generic_top `"G_HORIZONTAL_RESOLUTION_DEG=$($Cfg.horizontal_resolution_deg)`"",
        "--generic_top `"G_TDATA_WIDTH=$($Cfg.output_width_bits)`"",
        "--generic_top `"G_STOPS_PER_CHIP=$($Cfg.stops_per_chip)`"",
        "--generic_top `"G_RETURNS_PER_STOP=$($Cfg.returns_per_stop)`"",
        "--generic_top `"G_MAX_HITS_CFG=$($Cfg.max_hits_cfg)`"",
        "--generic_top `"G_TDC_DRAIN_MARGIN_TIME_NS=$($Cfg.tdc_drain_margin_time_ns)`"",
        "--generic_top `"G_COLS_PER_FACE=$($Cfg.columns_per_face)`"",
        "--generic_top `"G_N_FACES=$($Cfg.faces_per_frame)`"",
        "--generic_top `"G_ACTIVE_CHIP_MASK=4'b$($Cfg.active_chip_mask)`"",
        "--generic_top `"G_CHIP_SLOPE_MASK=4'b$($Cfg.chip_slope_mask)`"",
        "--generic_top `"G_ENCODER_SOURCE=$($Cfg.encoder_source)`"",
        "--generic_top `"G_ECHO_STIM_MODE=$($Cfg.echo_stimulus_mode)`"",
        "--generic_top `"G_REARM_GUARD_5NS_TICKS=$($Cfg.rearm_guard_5ns_ticks)`"",
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

$SourceEntries = foreach ($File in $CanonicalFiles) {
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
