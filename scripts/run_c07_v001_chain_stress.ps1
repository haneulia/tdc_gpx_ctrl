param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss"),
    [switch]$RunProjectSyntax
)

$ErrorActionPreference = "Stop"

$Hdl = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl/HDL"
$Vivado = "C:/AMDDesignTools/2025.2.1/Vivado/bin"
$Tmp = "$Hdl/tmp/c07_v001"

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
        throw "Simulation log does not contain expected marker '$PassPattern': $Log"
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
    if ($RunProjectSyntax) {
        & "$Hdl/scripts/run_c06_v002_regression.ps1" -Stamp $Stamp -RunProjectSyntax -NoArchiveOnExit
    }
    else {
        & "$Hdl/scripts/run_c06_v002_regression.ps1" -Stamp $Stamp -NoArchiveOnExit
    }
    if ($LASTEXITCODE -ne 0) {
        throw "C07 prerequisite C06 baseline regression failed"
    }

    foreach ($w in @(32, 64, 128)) {
        foreach ($mh in @(1, 3, 5, 7)) {
            $snap = "tb_c07_v001_top_int_w${w}_mh${mh}_bp_snap"
            $log = "xsim_c07_v001_top_int_w${w}_mh${mh}_bp_$Stamp.log"
            Invoke-Xelab $snap "xelab_c07_v001_top_int_w${w}_mh${mh}_bp_$Stamp.log" `
                "xil_defaultlib.tb_tdc_gpx_top_int" `
                @(
                    "G_TDATA_WIDTH=$w",
                    "G_MAX_HITS_OVERRIDE=$mh",
                    "G_N_FACES=2",
                    "G_BP_TREADY_GAP=17"
                )
            Invoke-Checked "$Vivado/xsim.bat" @($snap, "-runall", "-log", $log)
            Assert-SimLog $log "max_hits=$mh"
            Assert-SimLog $log "faces=2"
            Assert-SimLog $log "bounded output backpressure preserved beats/tlast - PASS"
            Assert-SimLog $log "output streams emitted beats/tlast as expected - PASS"
        }
    }

    $lateSnap = "tb_c07_v001_top_int_w64_late_mh3_snap"
    $lateLog = "xsim_c07_v001_top_int_w64_late_mh3_$Stamp.log"
    Invoke-Xelab $lateSnap "xelab_c07_v001_top_int_w64_late_mh3_$Stamp.log" `
        "xil_defaultlib.tb_tdc_gpx_top_int" `
        @(
            "G_TDATA_WIDTH=64",
            "G_MAX_HITS_OVERRIDE=3",
            "G_N_FACES=2",
            "G_MAX_HITS_WRITE_MODE=3",
            "G_BP_TREADY_GAP=17"
        )
    Invoke-Checked "$Vivado/xsim.bat" @($lateSnap, "-runall", "-log", $lateLog)
    Assert-SimLog $lateLog "max_hits_mode=late-after-packet-start"
    Assert-SimLog $lateLog "bounded output backpressure preserved beats/tlast - PASS"
    Assert-SimLog $lateLog "output streams emitted beats/tlast as expected - PASS"
}
finally {
    Pop-Location
    & "$Hdl/scripts/archive_vivado_xsim_outputs.ps1" `
        -Root $Hdl `
        -Stamp $Stamp `
        -Label "c07_v001_chain_stress"
}
