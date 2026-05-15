param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss"),
    [int[]]$ReservePs = @(8000000, 9000000, 10000000, 11000000, 12000000)
)

$ErrorActionPreference = "Stop"

$Hdl = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL"
$Vivado = "C:/AMDDesignTools/2025.2.1/Vivado/bin"
$Tmp = "$Hdl/tmp/c07_v001_reserve_budget"

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
        throw "Simulation log does not contain expected marker '$PassPattern': $Log"
    }
}

function Invoke-Xelab {
    param(
        [string]$Snapshot,
        [string]$Log,
        [string[]]$Generics = @()
    )
    $argFile = Join-Path $Tmp "$Snapshot.f"
    $lines = @(
        "--debug typical",
        "--relax",
        "--mt 2",
        "-L xil_defaultlib",
        "--snapshot $Snapshot"
    )
    foreach ($g in $Generics) {
        $lines += "--generic_top `"$g`""
    }
    $lines += "tb_tdc_gpx_polygon_budget_matrix"
    $lines += "-log $Log"
    $lines | Set-Content -Encoding ASCII $argFile
    Invoke-Checked "$Vivado/xelab.bat" @("-f", $argFile)
}

Push-Location $Hdl
try {
    Invoke-Checked "$Vivado/xvhdl.bat" @(
        "--2008",
        "tdc_gpx_pkg.vhd",
        "tb_tdc_gpx_polygon_budget_matrix.vhd",
        "-log",
        "xvhdl_c07_v001_reserve_budget_$Stamp.log"
    )

    foreach ($reserve in $ReservePs) {
        $reserveUs = [int]($reserve / 1000000)
        $snap = "tb_c07_v001_reserve_${reserveUs}us_snap"
        $log = "xsim_c07_v001_reserve_${reserveUs}us_$Stamp.log"
        Invoke-Xelab $snap "xelab_c07_v001_reserve_${reserveUs}us_$Stamp.log" @(
            "G_SYSTEM_RESERVED_PS=$reserve"
        )
        Invoke-Checked "$Vivado/xsim.bat" @($snap, "-runall", "-log", $log)
        Assert-SimLog $log "SYSTEM_RESERVED_PS=$reserve"
        Assert-SimLog $log "*** tb_tdc_gpx_polygon_budget_matrix PASS ***"
    }
}
finally {
    Pop-Location
    & "$Hdl/scripts/archive_vivado_xsim_outputs.ps1" `
        -Root $Hdl `
        -Stamp $Stamp `
        -Label "c07_v001_reserve_budget"
}
