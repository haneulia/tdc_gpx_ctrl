param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss"),
    [switch]$RunProjectSyntax,
    [switch]$RequireSoftPass
)

$ErrorActionPreference = "Stop"

$Hdl = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL"
$Vivado = "C:/AMDDesignTools/2025.2.1/Vivado/bin"
$Tmp = "$Hdl/tmp/c06_v003"

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

function Assert-SoftResetProbe {
    param([string]$Log)
    if (Select-String -Path $Log -Pattern "recovery mode soft_reset PASS" -Quiet) {
        Write-Host "soft_reset recovery PASS: $Log"
        return
    }
    if (Select-String -Path $Log -Pattern "IFIFO1 expected-count drain mismatch" -Quiet) {
        if (Select-String -Path $Log -Pattern "face_seq: cmd_start pending-latched \(pipeline busy\)" -Quiet) {
            Write-Host "soft_reset recovery remains OPEN as expected for v003 probe: $Log"
            return
        }
    }
    throw "soft_reset probe ended in an unexpected state: $Log"
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
    $baseArgs = @("-Stamp", $Stamp)
    if ($RunProjectSyntax) {
        $baseArgs += "-RunProjectSyntax"
    }

    & "$Hdl/scripts/run_c06_v002_regression.ps1" @baseArgs
    if ($LASTEXITCODE -ne 0) {
        throw "C06 v002 baseline regression failed"
    }

    $forceSnap = "tb_c06_v003_top_int_force_snap"
    Invoke-Xelab $forceSnap "xelab_c06_v003_top_int_force_$Stamp.log" `
        "xil_defaultlib.tb_tdc_gpx_top_int" @("G_TDATA_WIDTH=64", "G_RECOVERY_MODE=2")
    Invoke-Checked "$Vivado/xsim.bat" @($forceSnap, "-runall", "-log", "xsim_c06_v003_top_int_force_$Stamp.log")
    Assert-SimLog "xsim_c06_v003_top_int_force_$Stamp.log" "recovery mode force_reinit PASS"
    Assert-SimLog "xsim_c06_v003_top_int_force_$Stamp.log" "output streams emitted beats/tlast as expected - PASS"

    $softSnap = "tb_c06_v003_top_int_soft_snap"
    Invoke-Xelab $softSnap "xelab_c06_v003_top_int_soft_$Stamp.log" `
        "xil_defaultlib.tb_tdc_gpx_top_int" @("G_TDATA_WIDTH=64", "G_RECOVERY_MODE=1")
    Invoke-Checked "$Vivado/xsim.bat" @($softSnap, "-runall", "-log", "xsim_c06_v003_top_int_soft_$Stamp.log")
    if ($RequireSoftPass) {
        Assert-SimLog "xsim_c06_v003_top_int_soft_$Stamp.log" "recovery mode soft_reset PASS"
        Assert-SimLog "xsim_c06_v003_top_int_soft_$Stamp.log" "output streams emitted beats/tlast as expected - PASS"
    }
    else {
        Assert-SoftResetProbe "xsim_c06_v003_top_int_soft_$Stamp.log"
    }
}
finally {
    Pop-Location
}
