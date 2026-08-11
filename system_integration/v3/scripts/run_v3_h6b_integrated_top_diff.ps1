param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss"),
    [switch]$SkipHlsSynthesis,
    [string]$VivadoRoot = "C:\AMDDesignTools\2025.2.1\Vivado"
)

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$V3Root = Split-Path -Parent $ScriptDir
$RepoRoot = (Resolve-Path (Join-Path $V3Root "..\..")).Path
$V2Root = Join-Path $RepoRoot "system_integration\v2"
$OrderFile = Join-Path $V2Root "scripts\v2_rtl_compile_order.txt"
$Work = Join-Path $RepoRoot ".work\v3_h6b_integrated_top_diff\$Stamp"
$ToolHome = Join-Path $RepoRoot ".work\v3_h6b_integrated_tool_home"
$VivadoBin = Join-Path $VivadoRoot "bin"
$Glbl = Join-Path $VivadoRoot "data\verilog\src\glbl.v"

function Invoke-Checked {
    param(
        [Parameter(Mandatory = $true)][string]$Executable,
        [Parameter(Mandatory = $true)][string[]]$ToolArguments
    )
    & $Executable @ToolArguments
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed ($LASTEXITCODE): $Executable"
    }
}

# xelab의 name=value Generic 인자는 PowerShell native argument binder에서
# 등호가 분리될 수 있으므로 검증된 cmd.exe batch quoting으로 전달한다.
function Invoke-BatchChecked {
    param(
        [Parameter(Mandatory = $true)][string]$Executable,
        [Parameter(Mandatory = $true)][string[]]$ToolArguments
    )
    $QuotedArguments = foreach ($Argument in $ToolArguments) {
        if ($Argument -match '[\s=]') {
            '"' + $Argument.Replace('"', '\"') + '"'
        }
        else {
            $Argument
        }
    }
    $CommandLine = '"' + $Executable + '" ' +
        ($QuotedArguments -join ' ')
    & $env:ComSpec /d /s /c $CommandLine
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed ($LASTEXITCODE): $CommandLine"
    }
}

$HlsComponents = @(
    [ordered]@{
        Runner = "run_v3_hls_hit_decoder.ps1"
        Rtl = ".work\v3_hls_hit_decoder_component\hls\syn\verilog"
    },
    [ordered]@{
        Runner = "run_v3_hls_cell_collector.ps1"
        Rtl = ".work\v3_hls_cell_collector_component\hls\syn\verilog"
    },
    [ordered]@{
        Runner = "run_v3_hls_frame_assembler.ps1"
        Rtl = ".work\v3_hls_frame_assembler_component\hls\syn\verilog"
    },
    [ordered]@{
        Runner = "run_v3_hls_lane_word_formatter.ps1"
        Rtl = ".work\v3_hls_lane_word_formatter_component\hls\syn\verilog"
    }
)

if (-not $SkipHlsSynthesis) {
    foreach ($Component in $HlsComponents) {
        Invoke-Checked -Executable "powershell.exe" -ToolArguments @(
            "-NoProfile", "-ExecutionPolicy", "Bypass",
            "-File", (Join-Path $ScriptDir $Component.Runner),
            "-Step", "csynth"
        )
    }
}

$HlsFiles = @()
$HlsDataFiles = @()
foreach ($Component in $HlsComponents) {
    $Directory = Join-Path $RepoRoot $Component.Rtl
    if (-not (Test-Path -LiteralPath $Directory)) {
        throw "Generated HLS RTL directory is missing: $Directory"
    }
    $Files = @(Get-ChildItem -File -LiteralPath $Directory -Filter "*.v" |
        Sort-Object -Property Name | ForEach-Object { $_.FullName })
    if ($Files.Count -eq 0) {
        throw "Generated HLS RTL files are missing: $Directory"
    }
    $HlsFiles += $Files
    $HlsDataFiles += @(Get-ChildItem -File -LiteralPath $Directory `
        -Filter "*.dat" | ForEach-Object { $_.FullName })
}
$HlsFiles = @($HlsFiles | Sort-Object -Unique)

$V2Files = foreach ($Line in Get-Content -LiteralPath $OrderFile) {
    $Entry = $Line.Trim()
    if ($Entry.Length -gt 0 -and -not $Entry.StartsWith("#")) {
        (Resolve-Path -LiteralPath (Join-Path $RepoRoot $Entry)).Path
    }
}

$V3Files = @(
    (Join-Path $RepoRoot "px_utility_pkg.vhd"),
    (Join-Path $RepoRoot "tdc_gpx_sync_fifo.vhd"),
    (Join-Path $V2Root "pkg\lidar_config_reference_pkg.vhd"),
    (Join-Path $V3Root "pkg\lidar_v3_hls_contract_pkg.vhd"),
    (Join-Path $V3Root "pkg\lidar_v3_status_pkg.vhd"),
    (Join-Path $V3Root "rtl\status\lidar_v3_processing_status_source.vhd"),
    (Join-Path $V3Root "rtl\status\lidar_v3_status_snapshot_subsystem.vhd"),
    (Join-Path $V3Root "rtl\bridges\lidar_gpx_hit_decoder_hls_adapter.vhd"),
    (Join-Path $V3Root "rtl\bridges\lidar_gpx_cell_collector_hls_adapter.vhd"),
    (Join-Path $V3Root "rtl\bridges\lidar_gpx_frame_lane_assembler_hls_adapter.vhd"),
    (Join-Path $V3Root "rtl\bridges\lidar_gpx_lane_word_formatter_hls_adapter.vhd"),
    (Join-Path $V3Root "rtl\top\lidar_gpx_hls_hit_cell_frame_pipeline.vhd"),
    (Join-Path $V3Root "rtl\top\lidar_gpx_hls_axis_output_subsystem.vhd"),
    (Join-Path $V3Root "rtl\top\lidar_gpx_hls_mixed_data_top.vhd"),
    (Join-Path $V3Root "rtl\top\lidar_gpx_hls_parent_data_subsystem.vhd"),
    (Join-Path $V3Root "rtl\top\tdc_gpx_lidar_ctrl_v3_top.vhd"),
    (Join-Path $V3Root "tb\tb_tdc_gpx_lidar_ctrl_v3_h6b.vhd")
)

$VhdlFiles = @($V2Files + $V3Files)
foreach ($File in $VhdlFiles + $HlsFiles + @($Glbl)) {
    if (-not (Test-Path -LiteralPath $File)) {
        throw "Required source is missing: $File"
    }
}

$Profiles = @(
    [ordered]@{
        Name = "proc150_tdc200_w32"
        ProcMhz = 150
        TdcMhz = 200
        Width = 32
    },
    [ordered]@{
        Name = "proc200_tdc150_w64"
        ProcMhz = 200
        TdcMhz = 150
        Width = 64
    }
)

New-Item -ItemType Directory -Force -Path $Work | Out-Null
New-Item -ItemType Directory -Force -Path $ToolHome | Out-Null
foreach ($DataFile in $HlsDataFiles) {
    Copy-Item -LiteralPath $DataFile -Destination $Work -Force
}

$VerilogProject = Join-Path $Work "hls_verilog.prj"
$VhdlProject = Join-Path $Work "h6b_vhdl.prj"
$RunTcl = Join-Path $Work "run.tcl"

$VerilogLines = foreach ($File in $HlsFiles + @($Glbl)) {
    "verilog xil_defaultlib `"$($File.Replace('\', '/'))`""
}
$VerilogLines += "nosort"
$VerilogLines | Set-Content -Encoding ASCII -LiteralPath $VerilogProject

$VhdlLines = foreach ($File in $VhdlFiles) {
    "vhdl2008 xil_defaultlib `"$($File.Replace('\', '/'))`""
}
$VhdlLines += "nosort"
$VhdlLines | Set-Content -Encoding ASCII -LiteralPath $VhdlProject
@("run all", "quit") | Set-Content -Encoding ASCII -LiteralPath $RunTcl

$SavedEnvironment = @{
    HOME = $env:HOME
    USERPROFILE = $env:USERPROFILE
    APPDATA = $env:APPDATA
    LOCALAPPDATA = $env:LOCALAPPDATA
}

try {
    $env:HOME = $ToolHome
    $env:USERPROFILE = $ToolHome
    $env:APPDATA = $ToolHome
    $env:LOCALAPPDATA = $ToolHome
    Push-Location $Work
    try {
        Invoke-Checked -Executable (Join-Path $VivadoBin "xvlog.bat") `
            -ToolArguments @(
                "--relax", "-prj", $VerilogProject,
                "-log", (Join-Path $Work "xvlog.log"))
        Invoke-Checked -Executable (Join-Path $VivadoBin "xvhdl.bat") `
            -ToolArguments @(
                "--2008", "--relax", "-prj", $VhdlProject,
                "-log", (Join-Path $Work "xvhdl.log"))

        foreach ($Profile in $Profiles) {
            $Top = "tb_tdc_gpx_lidar_ctrl_v3_h6b_axis"
            $Snapshot = "$($Profile.Name)_${Stamp}_snap"
            $ElabLog = Join-Path $Work "xelab_$($Profile.Name).log"
            $SimLog = Join-Path $Work "xsim_$($Profile.Name).log"
            Invoke-BatchChecked -Executable (Join-Path $VivadoBin "xelab.bat") `
                -ToolArguments @(
                    "--debug", "off", "--relax", "--mt", "2",
                    "--snapshot", $Snapshot,
                    "--generic_top", "G_PROC_CLK_MHZ=$($Profile.ProcMhz)",
                    "--generic_top", "G_TDC_CLK_MHZ=$($Profile.TdcMhz)",
                    "--generic_top", "G_OUTPUT_WIDTH=$($Profile.Width)",
                    "--generic_top", "G_AXIS_STALL_CLKS=13",
                    "xil_defaultlib.$Top", "xil_defaultlib.glbl",
                    "-log", $ElabLog)
            Invoke-Checked -Executable (Join-Path $VivadoBin "xsim.bat") `
                -ToolArguments @(
                    $Snapshot, "-tclbatch", $RunTcl.Replace('\', '/'),
                    "-log", $SimLog.Replace('\', '/'))

            $Text = Get-Content -Raw -LiteralPath $SimLog
            $RequiredMarkers = @(
                (("LIDAR_V3_H6B_INTEGRATED_TOP_PASS proc_mhz={0} " +
                    "tdc_mhz={1}") -f
                    $Profile.ProcMhz, $Profile.TdcMhz),
                (("LIDAR_V3_H6B_REG7_SHADOW_ACTIVE_PHYSICAL_PASS " +
                    "proc_mhz={0} tdc_mhz={1}") -f
                    $Profile.ProcMhz, $Profile.TdcMhz),
                (("LIDAR_V3_H6B_AXIS_PASS proc_mhz={0} tdc_mhz={1} " +
                    "output_width={2} stall_clks=13") -f
                    $Profile.ProcMhz, $Profile.TdcMhz, $Profile.Width)
            )
            foreach ($Marker in $RequiredMarkers) {
                if (-not $Text.Contains($Marker)) {
                    throw "$($Profile.Name) missing PASS marker: $Marker"
                }
            }
            if ($Text -match "Failure:|Fatal:|ERROR:") {
                throw "$($Profile.Name) reported a simulation failure"
            }
            Write-Host "LIDAR_V3_H6B_INTEGRATED_PROFILE_PASS $($Profile.Name)"
        }
    }
    finally {
        Pop-Location
    }
}
finally {
    foreach ($Key in $SavedEnvironment.Keys) {
        Set-Item -Path "Env:$Key" -Value $SavedEnvironment[$Key]
    }
}

Write-Host "LIDAR_V3_H6B_INTEGRATED_TOP_DIFF_PASS"
Write-Host "Result: $Work"
