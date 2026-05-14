param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss"),
    [switch]$RunProjectSyntax
)

$ErrorActionPreference = "Stop"

$Hdl = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL"
$Vivado = "C:/AMDDesignTools/2025.2.1/Vivado/bin"
$Tmp = "$Hdl/tmp/c06_v004"

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
    if (-not (Select-String -Path $Log -Pattern $PassPattern -Quiet)) {
        throw "Simulation log does not contain expected PASS marker '$PassPattern': $Log"
    }
}

function Assert-RecoveryEvidence {
    param(
        [string]$Log,
        [string]$Mode
    )
    Assert-SimLog $Log "recovery mode $Mode PASS"
    Assert-SimLog $Log "output streams emitted beats/tlast as expected - PASS"
    Assert-SimLog $Log "config_ctrl: $Mode"
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
    if ($RunProjectSyntax) {
        & "$Hdl/scripts/run_c06_v002_regression.ps1" -Stamp $Stamp -RunProjectSyntax -NoArchiveOnExit
    }
    else {
        & "$Hdl/scripts/run_c06_v002_regression.ps1" -Stamp $Stamp -NoArchiveOnExit
    }
    if ($LASTEXITCODE -ne 0) {
        throw "C06 v002 baseline regression failed"
    }

    $forceSnap = "tb_c06_v004_top_int_force_snap"
    Invoke-Xelab $forceSnap "xelab_c06_v004_top_int_force_$Stamp.log" `
        "xil_defaultlib.tb_tdc_gpx_top_int" @("G_TDATA_WIDTH=64", "G_RECOVERY_MODE=2")
    Invoke-Checked "$Vivado/xsim.bat" @($forceSnap, "-runall", "-log", "xsim_c06_v004_top_int_force_$Stamp.log")
    Assert-RecoveryEvidence "xsim_c06_v004_top_int_force_$Stamp.log" "force_reinit"

    $softSnap = "tb_c06_v004_top_int_soft_snap"
    Invoke-Xelab $softSnap "xelab_c06_v004_top_int_soft_$Stamp.log" `
        "xil_defaultlib.tb_tdc_gpx_top_int" @("G_TDATA_WIDTH=64", "G_RECOVERY_MODE=1")
    Invoke-Checked "$Vivado/xsim.bat" @($softSnap, "-runall", "-log", "xsim_c06_v004_top_int_soft_$Stamp.log")
    Assert-RecoveryEvidence "xsim_c06_v004_top_int_soft_$Stamp.log" "soft_reset"
}
finally {
    Pop-Location
    & "$Hdl/scripts/archive_vivado_xsim_outputs.ps1" `
        -Root $Hdl `
        -Stamp $Stamp `
        -Label "c06_v004_recovery"
}
