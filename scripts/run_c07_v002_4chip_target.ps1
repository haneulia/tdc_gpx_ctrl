param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss")
)

$ErrorActionPreference = "Stop"

$Root = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl"
$Hdl = "$Root/HDL"
$Vivado = "C:/AMDDesignTools/2025.2.1/Vivado/bin"
$Tmp = "$Hdl/tmp/c07_v002_4chip_target"

New-Item -ItemType Directory -Force -Path $Tmp | Out-Null

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

function Assert-SimLog {
    param(
        [string]$Log,
        [string]$PassPattern
    )
    if (Select-String -Path $Log -Pattern "Failure:|ERROR:|\bfailed\b" -CaseSensitive:$false -Quiet) {
        throw "Simulation log contains failure/error marker: $Log"
    }
    if (-not (Select-String -Path $Log -Pattern $PassPattern -SimpleMatch -Quiet)) {
        throw "Simulation log does not contain expected PASS marker '$PassPattern': $Log"
    }
}

function Invoke-Xelab {
    param(
        [string]$Snapshot,
        [string]$Log,
        [string]$Top,
        [string[]]$Generics = @()
    )
    $argFile = Join-Path $Tmp "$Snapshot.f"
    $lines = @(
        "--debug typical",
        "--relax",
        "--mt 2",
        "-L xil_defaultlib",
        "-L unisims_ver",
        "-L unimacro_ver",
        "-L secureip",
        "-L xpm",
        "--snapshot $Snapshot"
    )
    foreach ($g in $Generics) {
        $lines += "--generic_top `"$g`""
    }
    $lines += $Top
    $lines += "xil_defaultlib.glbl"
    $lines += "-log $Log"
    $lines | Set-Content -Encoding ASCII $argFile
    Invoke-Checked "$Vivado/xelab.bat" @("-f", $argFile)
}

Push-Location $Hdl
try {
    & "$Hdl/scripts/run_c06_v002_regression.ps1" -Stamp $Stamp -NoArchiveOnExit
    if ($LASTEXITCODE -ne 0) {
        throw "C07 4-chip target prerequisite compile/regression failed"
    }

    $wrapperPrj = Join-Path $Tmp "c07_v002_4chip_target_wrapper.prj"
    @(
        "vhdl2008 xil_defaultlib `"$Hdl/tb_tdc_gpx_top_int_c07_4chip_target.vhd`"",
        "nosort"
    ) | Set-Content -Encoding ASCII $wrapperPrj
    Invoke-Checked "$Vivado/xvhdl.bat" @("--relax", "-prj", $wrapperPrj, "-log", "xvhdl_c07_v002_4chip_target_wrapper_$Stamp.log")

    foreach ($w in @(64, 128)) {
        $snap = "tb_c07_v002_4chip_target_w${w}_snap"
        $log = "xsim_c07_v002_4chip_target_w${w}_$Stamp.log"
        Invoke-Xelab $snap "xelab_c07_v002_4chip_target_w${w}_$Stamp.log" `
            "xil_defaultlib.tb_tdc_gpx_top_int_c07_4chip_target" `
            @("G_TDATA_WIDTH=$w")
        Invoke-Checked "$Vivado/xsim.bat" @($snap, "-runall", "-log", $log)
        Assert-SimLog $log "output streams emitted beats/tlast as expected - PASS"
        Assert-SimLog $log "Hit[16] final metadata preservation - PASS"
    }
}
finally {
    Pop-Location
    & "$Hdl/scripts/archive_vivado_xsim_outputs.ps1" `
        -Root $Hdl `
        -Stamp $Stamp `
        -Label "c07_v002_4chip_target"
}
