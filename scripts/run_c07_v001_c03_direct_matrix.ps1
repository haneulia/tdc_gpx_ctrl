param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss")
)

$ErrorActionPreference = "Stop"

$Root = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl"
$Hdl = "$Root/HDL"
$Vivado = "C:/AMDDesignTools/2025.2.1/Vivado/bin"
$Tmp = "$Hdl/tmp/c07_v001_c03_direct"

New-Item -ItemType Directory -Force -Path $Tmp | Out-Null

$vhdlPrj = Join-Path $Tmp "c07_v001_c03_direct_vhdl.prj"
$vlogPrj = Join-Path $Tmp "c07_v001_c03_direct_vlog.prj"

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

$vhdl2008Files = @(
    "$Hdl/tdc_gpx_pkg.vhd",
    "$Hdl/tdc_gpx_skid_buffer.vhd",
    "$Hdl/tdc_gpx_cell_builder.vhd",
    "$Hdl/tdc_gpx_cell_pipe.vhd",
    "$Hdl/tb_tdc_gpx_cell_pipe_c03_fix.vhd",
    "$Hdl/tb_tdc_gpx_cell_builder_c07_direct.vhd"
)

$vhdlLines = @()
foreach ($f in $vhdl2008Files) { $vhdlLines += "vhdl2008 xil_defaultlib `"$f`"" }
$vhdlLines += "nosort"
$vhdlLines | Set-Content -Encoding ASCII $vhdlPrj

@(
    "verilog xil_defaultlib `"$Root/tdc_gpx_ctrl.sim/sim_1/behav/xsim/glbl.v`"",
    "nosort"
) | Set-Content -Encoding ASCII $vlogPrj

Push-Location $Hdl
try {
    Invoke-Checked "$Vivado/xvlog.bat" @("--relax", "-prj", $vlogPrj, "-log", "xvlog_c07_v001_c03_direct_$Stamp.log")
    Invoke-Checked "$Vivado/xvhdl.bat" @("--relax", "-prj", $vhdlPrj, "-log", "xvhdl_c07_v001_c03_direct_$Stamp.log")

    Invoke-Xelab "tb_c07_v001_c03_pipe_fix_snap" "xelab_c07_v001_c03_pipe_fix_$Stamp.log" `
        "xil_defaultlib.tb_tdc_gpx_cell_pipe_c03_fix"
    Invoke-Checked "$Vivado/xsim.bat" @("tb_c07_v001_c03_pipe_fix_snap", "-runall", "-log", "xsim_c07_v001_c03_pipe_fix_$Stamp.log")
    Assert-SimLog "xsim_c07_v001_c03_pipe_fix_$Stamp.log" "PASS: C03 cell_pipe Hit[16], input skid, and per-slope abort regression passed."

    foreach ($w in @(32, 64, 128)) {
        foreach ($mh in @(1, 3, 5, 7)) {
            $snap = "tb_c07_v001_c03_matrix_w${w}_mh${mh}_snap"
            $log = "xsim_c07_v001_c03_matrix_w${w}_mh${mh}_$Stamp.log"
            Invoke-Xelab $snap "xelab_c07_v001_c03_matrix_w${w}_mh${mh}_$Stamp.log" `
                "xil_defaultlib.tb_tdc_gpx_cell_builder_c07_direct" `
                @(
                    "G_TDATA_WIDTH=$w",
                    "G_MAX_HITS=$mh",
                    "G_SCENARIO=0"
                )
            Invoke-Checked "$Vivado/xsim.bat" @($snap, "-runall", "-log", $log)
            Assert-SimLog $log "PASS: C07 C03 direct matrix width=$w max_hits=$mh"
        }
    }

    foreach ($w in @(32, 64, 128)) {
        $snap = "tb_c07_v001_c03_ififo2_w${w}_snap"
        $log = "xsim_c07_v001_c03_ififo2_w${w}_$Stamp.log"
        Invoke-Xelab $snap "xelab_c07_v001_c03_ififo2_w${w}_$Stamp.log" `
            "xil_defaultlib.tb_tdc_gpx_cell_builder_c07_direct" `
            @(
                "G_TDATA_WIDTH=$w",
                "G_MAX_HITS=7",
                "G_SCENARIO=1"
            )
        Invoke-Checked "$Vivado/xsim.bat" @($snap, "-runall", "-log", $log)
        Assert-SimLog $log "PASS: C07 C03 IFIFO2 timeout width=$w max_hits=7"
    }

    $dualSnap = "tb_c07_v001_c03_dual_w64_mh3_snap"
    $dualLog = "xsim_c07_v001_c03_dual_w64_mh3_$Stamp.log"
    Invoke-Xelab $dualSnap "xelab_c07_v001_c03_dual_w64_mh3_$Stamp.log" `
        "xil_defaultlib.tb_tdc_gpx_cell_builder_c07_direct" `
        @(
            "G_TDATA_WIDTH=64",
            "G_MAX_HITS=3",
            "G_SCENARIO=2"
        )
    Invoke-Checked "$Vivado/xsim.bat" @($dualSnap, "-runall", "-log", $dualLog)
    Assert-SimLog $dualLog "PASS: C07 C03 dual-buffer next-shot II width=64 max_hits=3"

    $dropSnap = "tb_c07_v001_c03_drop_w64_mh3_snap"
    $dropLog = "xsim_c07_v001_c03_drop_w64_mh3_$Stamp.log"
    Invoke-Xelab $dropSnap "xelab_c07_v001_c03_drop_w64_mh3_$Stamp.log" `
        "xil_defaultlib.tb_tdc_gpx_cell_builder_c07_direct" `
        @(
            "G_TDATA_WIDTH=64",
            "G_MAX_HITS=3",
            "G_SCENARIO=3"
        )
    Invoke-Checked "$Vivado/xsim.bat" @($dropSnap, "-runall", "-log", $dropLog)
    Assert-SimLog $dropLog "PASS: C07 C03 drop/quarantine width=64 max_hits=3"

    $stopsSnap = "tb_c07_v001_c03_stops_snapshot_w64_mh3_snap"
    $stopsLog = "xsim_c07_v001_c03_stops_snapshot_w64_mh3_$Stamp.log"
    Invoke-Xelab $stopsSnap "xelab_c07_v001_c03_stops_snapshot_w64_mh3_$Stamp.log" `
        "xil_defaultlib.tb_tdc_gpx_cell_builder_c07_direct" `
        @(
            "G_TDATA_WIDTH=64",
            "G_MAX_HITS=3",
            "G_SCENARIO=4"
        )
    Invoke-Checked "$Vivado/xsim.bat" @($stopsSnap, "-runall", "-log", $stopsLog)
    Assert-SimLog $stopsLog "PASS: C07 C03 stops_per_chip buffer snapshot width=64 max_hits=3"

    $reuseSnap = "tb_c07_v001_c03_reuse_stale_w64_mh3_snap"
    $reuseLog = "xsim_c07_v001_c03_reuse_stale_w64_mh3_$Stamp.log"
    Invoke-Xelab $reuseSnap "xelab_c07_v001_c03_reuse_stale_w64_mh3_$Stamp.log" `
        "xil_defaultlib.tb_tdc_gpx_cell_builder_c07_direct" `
        @(
            "G_TDATA_WIDTH=64",
            "G_MAX_HITS=3",
            "G_SCENARIO=5"
        )
    Invoke-Checked "$Vivado/xsim.bat" @($reuseSnap, "-runall", "-log", $reuseLog)
    Assert-SimLog $reuseLog "PASS: C07 C03 reused buffer invalidated stale payload width=64 max_hits=3"
}
finally {
    Pop-Location
    & "$Hdl/scripts/archive_vivado_xsim_outputs.ps1" `
        -Root $Hdl `
        -Stamp $Stamp `
        -Label "c07_v001_c03_direct_matrix"
}
