param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss")
)

$ErrorActionPreference = "Stop"

$Root = "C:/Projects/my_sp/lib/IP/tdc_gpx_ctrl"
$Hdl = "$Root/HDL"
$Vivado = "C:/AMDDesignTools/2025.2.1/Vivado/bin"
$Tmp = "$Hdl/tmp/c07_v001_c04_direct"

New-Item -ItemType Directory -Force -Path $Tmp | Out-Null

$vhdlPrj = Join-Path $Tmp "c07_v001_c04_direct_vhdl.prj"
$vlogPrj = Join-Path $Tmp "c07_v001_c04_direct_vlog.prj"

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
    "$Hdl/tdc_gpx_sync_fifo.vhd",
    "$Hdl/tdc_gpx_face_assembler.vhd",
    "$Hdl/tdc_gpx_header_inserter.vhd",
    "$Hdl/tb_tdc_gpx_face_assembler_c07_direct.vhd",
    "$Hdl/tb_tdc_gpx_header_inserter_c07_direct.vhd"
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
    Invoke-Checked "$Vivado/xvlog.bat" @("--relax", "-prj", $vlogPrj, "-log", "xvlog_c07_v001_c04_direct_$Stamp.log")
    Invoke-Checked "$Vivado/xvhdl.bat" @("--relax", "-prj", $vhdlPrj, "-log", "xvhdl_c07_v001_c04_direct_$Stamp.log")

    foreach ($w in @(32, 64, 128)) {
        $snap = "tb_c07_v001_c04_face_w${w}_snap"
        $log = "xsim_c07_v001_c04_face_w${w}_$Stamp.log"
        Invoke-Xelab $snap "xelab_c07_v001_c04_face_w${w}_$Stamp.log" `
            "xil_defaultlib.tb_tdc_gpx_face_assembler_c07_direct" `
            @("G_TDATA_WIDTH=$w")
        Invoke-Checked "$Vivado/xsim.bat" @($snap, "-runall", "-log", $log)
        Assert-SimLog $log "PASS: C07 C04 face_assembler ready boundary width=$w"
    }

    foreach ($w in @(32, 64, 128)) {
        $snap = "tb_c07_v001_c04_header_w${w}_snap"
        $log = "xsim_c07_v001_c04_header_w${w}_$Stamp.log"
        Invoke-Xelab $snap "xelab_c07_v001_c04_header_w${w}_$Stamp.log" `
            "xil_defaultlib.tb_tdc_gpx_header_inserter_c07_direct" `
            @("G_TDATA_WIDTH=$w")
        Invoke-Checked "$Vivado/xsim.bat" @($snap, "-runall", "-log", $log)
        Assert-SimLog $log "PASS: C07 C04 header pending/stall width=$w"
    }
}
finally {
    Pop-Location
    & "$Hdl/scripts/archive_vivado_xsim_outputs.ps1" `
        -Root $Hdl `
        -Stamp $Stamp `
        -Label "c07_v001_c04_direct_matrix"
}
