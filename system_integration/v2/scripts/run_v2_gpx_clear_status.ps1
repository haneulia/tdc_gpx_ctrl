param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss"),
    [switch]$KeepWork
)

$ErrorActionPreference = "Stop"

$Hdl = (Resolve-Path (Join-Path $PSScriptRoot "../../..")).Path
$Vivado = "C:/AMDDesignTools/2025.2.1/Vivado/bin"
$Glbl = (Resolve-Path "C:/AMDDesignTools/2025.2.1/Vivado/data/verilog/src/glbl.v").Path
$WorkRoot = Join-Path $Hdl "tmp/v2_gpx_clear_status"
$Work = Join-Path $WorkRoot $Stamp
$Result = Join-Path $Hdl "signoff_results/sessions/${Stamp}_v2_gpx_clear_status"

New-Item -ItemType Directory -Force -Path $Work | Out-Null
New-Item -ItemType Directory -Force -Path $Result | Out-Null

function Invoke-Checked {
    param([string]$Exe, [string[]]$ArgList)
    & $Exe @ArgList
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed ($LASTEXITCODE): $Exe $($ArgList -join ' ')"
    }
}

function Assert-SimLog {
    param([string]$Log, [string]$PassPattern)
    if (Select-String -LiteralPath $Log `
            -Pattern "Failure:|Fatal:|assertion error|^ERROR:" `
            -CaseSensitive:$false -Quiet) {
        throw "Simulation log contains a failure marker: $Log"
    }
    if (-not (Select-String -LiteralPath $Log -Pattern $PassPattern -Quiet)) {
        throw "Missing PASS marker '$PassPattern': $Log"
    }
}

$Files = @(
    "$Hdl/px_utility_pkg.vhd",
    "$Hdl/tdc_gpx_pkg.vhd",
    "$Hdl/tdc_gpx_cfg_pkg.vhd",
    "$Hdl/tb_tdc_gpx_pkg.vhd",
    "$Hdl/tdc_gpx_bus_phy.vhd",
    "$Hdl/tdc_gpx_skid_buffer.vhd",
    "$Hdl/tdc_gpx_chip_init.vhd",
    "$Hdl/tdc_gpx_chip_run.vhd",
    "$Hdl/tdc_gpx_chip_reg.vhd",
    "$Hdl/tdc_gpx_chip_ctrl.vhd",
    "$Hdl/tdc_gpx_cmd_arb.vhd",
    "$Hdl/tb_tdc_gpx_chip_init_cfg_owner.vhd",
    "$Hdl/tb_tdc_gpx_request_loss.vhd",
    "$Hdl/tb_tdc_gpx_chip_ctrl.vhd"
)

$Project = Join-Path $Work "v2_gpx_clear_status_vhdl.prj"
$VerilogProject = Join-Path $Work "v2_gpx_clear_status_verilog.prj"
$ProjectLines = foreach ($File in $Files) {
    "vhdl2008 xil_defaultlib `"$($File.Replace('\', '/'))`""
}
$ProjectLines += "nosort"
$ProjectLines | Set-Content -Encoding ASCII -LiteralPath $Project
@(
    "verilog xil_defaultlib `"$($Glbl.Replace('\', '/'))`"",
    "nosort"
) | Set-Content -Encoding ASCII -LiteralPath $VerilogProject

$RunAllTcl = Join-Path $Work "run_all.tcl"
@("run all", "quit") | Set-Content -Encoding ASCII -LiteralPath $RunAllTcl
$FaultTcl = (Resolve-Path (Join-Path $PSScriptRoot `
    "v2_gpx_clear_status_fault_injection.tcl")).Path

Push-Location $Work
try {
    Invoke-Checked "$Vivado/xvlog.bat" @(
        "--relax", "-prj", $VerilogProject,
        "-log", (Join-Path $Work "xvlog.log"))
    Invoke-Checked "$Vivado/xvhdl.bat" @(
        "--relax", "-prj", $Project,
        "-log", (Join-Path $Work "xvhdl.log"))

    $Tests = @(
        @("tb_tdc_gpx_chip_init_cfg_owner", $RunAllTcl,
          "LIDAR_V2_K11_INIT_STICKY_CLEAR_PASS", "init_owner"),
        @("tb_tdc_gpx_request_loss", $RunAllTcl,
          "LIDAR_V2_K11_REGISTER_STICKY_CLEAR_PASS", "register_owner"),
        @("tb_tdc_gpx_chip_ctrl", $FaultTcl,
          "LIDAR_V2_K11_TDC_STICKY_CLEAR_PASS", "fault_injection"),
        @("tb_tdc_gpx_chip_ctrl", $RunAllTcl,
          "ALL TESTS PASSED", "full_regression")
    )

    foreach ($Test in $Tests) {
        $Top = $Test[0]
        $Tcl = $Test[1]
        $TclForXsim = $Tcl.Replace('\', '/')
        $Pass = $Test[2]
        $Scenario = $Test[3]
        $Snapshot = "${Top}_${Scenario}_${Stamp}_snap"
        $ElabLog = Join-Path $Work "xelab_${Top}_${Scenario}.log"
        $SimLog = Join-Path $Work "xsim_${Top}_${Scenario}.log"

        Invoke-Checked "$Vivado/xelab.bat" @(
            "--debug", "typical", "--relax", "--mt", "2",
            "--snapshot", $Snapshot,
            "xil_defaultlib.$Top", "xil_defaultlib.glbl",
            "-log", $ElabLog)
        Invoke-Checked "$Vivado/xsim.bat" @(
            $Snapshot, "-tclbatch", $TclForXsim, "-log", $SimLog)
        Assert-SimLog $SimLog $Pass

        Copy-Item -LiteralPath $ElabLog -Destination $Result
        Copy-Item -LiteralPath $SimLog -Destination $Result
    }
}
finally {
    Pop-Location
}

Copy-Item -LiteralPath (Join-Path $Work "xvhdl.log") -Destination $Result
Copy-Item -LiteralPath $FaultTcl -Destination $Result

if (-not $KeepWork) {
    $ResolvedWorkRoot = [IO.Path]::GetFullPath($WorkRoot)
    $ResolvedWork = [IO.Path]::GetFullPath($Work)
    if (-not $ResolvedWork.StartsWith($ResolvedWorkRoot + `
            [IO.Path]::DirectorySeparatorChar,
            [StringComparison]::OrdinalIgnoreCase)) {
        throw "Refusing to remove work path outside expected root: $ResolvedWork"
    }
    Remove-Item -LiteralPath $ResolvedWork -Recurse -Force
}

Write-Host "LIDAR_V2_K11_GPX_CLEAR_STATUS_REGRESSION_PASS"
Write-Host "Result: $Result"
