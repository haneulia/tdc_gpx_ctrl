param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss"),
    [switch]$SkipHlsSynthesis,
    [string]$VivadoRoot = "C:\AMDDesignTools\2025.2.1\Vivado"
)

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$V3Root = Split-Path -Parent $ScriptDir
$Hdl = (Resolve-Path (Join-Path $V3Root "..\..")).Path
$V2Root = Join-Path $Hdl "system_integration\v2"
$Vivado = Join-Path $VivadoRoot "bin"
$Work = Join-Path $Hdl ".work\v3_h6b2_ddr_golden\$Stamp"
$ToolHome = Join-Path $Hdl ".work\v3_h6b2_ddr_tool_home"
$HlsRunner = Join-Path $ScriptDir "run_v3_hls_lane_word_formatter.ps1"
$HlsRtl = Join-Path $Hdl (
    ".work\v3_hls_lane_word_formatter_component\hls\syn\verilog")
$Glbl = Join-Path $VivadoRoot "data\verilog\src\glbl.v"
$HtmlModel = Join-Path $Hdl (
    "Doc/cluster_analysis/C08_HDL_HTML_Alignment/" +
    "C08_HDL_HTML_Alignment_260806_PACKED17_VDMA_ABI_Simulator_v026.html")
$GoldenFile = Join-Path $Hdl (
    "system_integration/v2/golden/packed17_j9_ddr_golden.json")
$GoldenVerifier = Join-Path $Hdl (
    "system_integration/v2/scripts/verify_packed17_html_golden.mjs")

New-Item -ItemType Directory -Force -Path $Work | Out-Null
New-Item -ItemType Directory -Force -Path $ToolHome | Out-Null

function Invoke-Checked {
    param([string]$Exe, [string[]]$ArgList)
    & $Exe @ArgList
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed ($LASTEXITCODE): $Exe $($ArgList -join ' ')"
    }
}

function Compare-DdrCapture {
    param(
        [string]$CapturePath,
        [int]$Width,
        [object]$Golden
    )

    $Profile = @($Golden.profiles | Where-Object {
        [int]$_.output_width_bits -eq $Width
    })
    if ($Profile.Count -ne 1) {
        throw "Golden profile count for width $Width is $($Profile.Count)"
    }
    $Profile = $Profile[0]
    $Capture = @(Get-Content -LiteralPath $CapturePath | Where-Object {
        -not [string]::IsNullOrWhiteSpace($_)
    })
    $Expected = @($Profile.memory_words)
    if ($Capture.Count -ne $Expected.Count) {
        throw "Width $Width capture has $($Capture.Count) Words; expected $($Expected.Count)"
    }

    for ($Index = 0; $Index -lt $Expected.Count; $Index++) {
        if ($Capture[$Index] -notmatch
            '^0x([0-9A-Fa-f]{8})\s+0x([0-9A-Fa-f]{8})$') {
            throw "Width $Width malformed capture line ${Index}: $($Capture[$Index])"
        }
        $ActualAddress = "0x$($Matches[1])"
        $ActualWord = "0x$($Matches[2])"
        $ExpectedAddress = "0x{0:X8}" -f ($Index * 4)
        $ExpectedWord = [string]$Expected[$Index]
        if (-not [string]::Equals($ActualAddress, $ExpectedAddress,
                [System.StringComparison]::OrdinalIgnoreCase) -or
            -not [string]::Equals($ActualWord, $ExpectedWord,
                [System.StringComparison]::OrdinalIgnoreCase)) {
            throw "Width $Width mismatch at Word ${Index}: " +
                "actual $ActualAddress $ActualWord, " +
                "expected $ExpectedAddress $ExpectedWord"
        }
    }

    $ReserveCount = @($Expected | Where-Object {
        [string]::Equals([string]$_,
            [string]$Profile.geometry.reserve_fill_word,
            [System.StringComparison]::OrdinalIgnoreCase)
    }).Count
    if ($ReserveCount -eq 0) {
        throw "Width $Width did not exercise fixed-STRIDE reserve Words"
    }

    [ordered]@{
        width_bits = $Width
        words_compared = $Expected.Count
        bytes_compared = $Expected.Count * 4
        reserve_words_preserved = $ReserveCount
        hsize_bytes = [int]$Profile.geometry.hsize_bytes
        vsize_lines = [int]$Profile.geometry.vsize_lines
        stride_bytes = [int]$Profile.geometry.stride_bytes
        capture_sha256 = (Get-FileHash -LiteralPath $CapturePath -Algorithm SHA256).Hash
    }
}

if (-not $SkipHlsSynthesis) {
    Invoke-Checked "powershell.exe" @(
        "-NoProfile", "-ExecutionPolicy", "Bypass",
        "-File", $HlsRunner, "-Step", "csynth")
}
if (-not (Test-Path -LiteralPath $HlsRtl)) {
    throw "Generated H4 HLS RTL directory is missing: $HlsRtl"
}
$HlsFiles = @(Get-ChildItem -File -LiteralPath $HlsRtl -Filter "*.v" |
    Sort-Object -Property Name | ForEach-Object { $_.FullName })
if ($HlsFiles.Count -eq 0 -or -not (Test-Path -LiteralPath $Glbl)) {
    throw "Generated H4 HLS RTL or Vivado glbl.v is missing"
}
Get-ChildItem -File -LiteralPath $HlsRtl -Filter "*.dat" |
    Copy-Item -Destination $Work -Force

Invoke-Checked "node" @($GoldenVerifier, $HtmlModel, $GoldenFile)
$Golden = Get-Content -Raw -LiteralPath $GoldenFile | ConvertFrom-Json

$CommonFiles = @(
    (Join-Path $V2Root "pkg\lidar_build_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_config_types_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_event_types_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_gpx_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_gpx_event_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_gpx_data_pkg.vhd"),
    (Join-Path $V2Root "pkg\lidar_gpx_vdma_pkg.vhd"),
    (Join-Path $V3Root "pkg\lidar_v3_hls_contract_pkg.vhd"),
    (Join-Path $Hdl "tdc_gpx_skid_buffer.vhd"),
    (Join-Path $V2Root "rtl\proc\lidar_gpx_vdma_profile_manager.vhd"),
    (Join-Path $V2Root "rtl\proc\lidar_gpx_frame_close_fork.vhd"),
    (Join-Path $V2Root "rtl\proc\lidar_gpx_axis_word_packer.vhd"),
    (Join-Path $V3Root (
        "rtl\bridges\lidar_gpx_lane_word_formatter_hls_adapter.vhd")),
    (Join-Path $V3Root "rtl\top\lidar_gpx_hls_axis_output_subsystem.vhd")
)
$SimFiles = $CommonFiles + @(
    (Join-Path $V3Root "tb\tb_lidar_v3_h6b2_ddr_golden.vhd")
)
foreach ($File in $SimFiles) {
    if (-not (Test-Path -LiteralPath $File)) {
        throw "Required VHDL source is missing: $File"
    }
}

$VerilogProject = Join-Path $Work "h4_hls_verilog.prj"
$VerilogLines = foreach ($File in $HlsFiles + @($Glbl)) {
    "verilog xil_defaultlib `"$($File.Replace('\', '/'))`""
}
$VerilogLines += "nosort"
$VerilogLines | Set-Content -Encoding ASCII -LiteralPath $VerilogProject

$Project = Join-Path $Work "v3_h6b2_ddr_golden_vhdl.prj"
$ProjectLines = foreach ($File in $SimFiles) {
    "vhdl2008 xil_defaultlib `"$($File.Replace('\', '/'))`""
}
$ProjectLines += "nosort"
$ProjectLines | Set-Content -Encoding ASCII -LiteralPath $Project
$RunTcl = Join-Path $Work "run.tcl"
@("run all", "quit") | Set-Content -Encoding ASCII -LiteralPath $RunTcl

$Profiles = @(
    [ordered]@{ Mhz = 150; Width = 32 },
    [ordered]@{ Mhz = 200; Width = 64 }
)
$Comparison = @()

$SavedEnvironment = @{
    HOME = $env:HOME
    USERPROFILE = $env:USERPROFILE
    APPDATA = $env:APPDATA
    LOCALAPPDATA = $env:LOCALAPPDATA
}
$env:HOME = $ToolHome
$env:USERPROFILE = $ToolHome
$env:APPDATA = Join-Path $ToolHome "AppData\Roaming"
$env:LOCALAPPDATA = Join-Path $ToolHome "AppData\Local"
New-Item -ItemType Directory -Force -Path $env:APPDATA | Out-Null
New-Item -ItemType Directory -Force -Path $env:LOCALAPPDATA | Out-Null

Push-Location $Work
try {
    Invoke-Checked "$Vivado/xvlog.bat" @(
        "--relax", "-prj", $VerilogProject,
        "-log", (Join-Path $Work "xvlog.log")
    )
    Invoke-Checked "$Vivado/xvhdl.bat" @(
        "--relax", "-prj", $Project,
        "-log", (Join-Path $Work "xvhdl.log")
    )

    foreach ($Profile in $Profiles) {
        $Mhz = $Profile.Mhz
        $Width = $Profile.Width
        $Top = "tb_lidar_v3_h6b2_ddr_golden_${Mhz}_${Width}"
        $Snapshot = "${Top}_${Stamp}_snap"
        $SimLog = Join-Path $Work "xsim_${Top}.log"
        Invoke-Checked "$Vivado/xelab.bat" @(
            "--debug", "off", "--relax", "--mt", "2",
            "--snapshot", $Snapshot, "xil_defaultlib.$Top",
            "xil_defaultlib.glbl",
            "-log", (Join-Path $Work "xelab_${Top}.log")
        )
        Invoke-Checked "$Vivado/xsim.bat" @(
            $Snapshot, "-tclbatch", $RunTcl.Replace('\', '/'),
            "-log", $SimLog.Replace('\', '/')
        )
        if ((Get-Content -Raw -LiteralPath $SimLog) -notmatch
            "LIDAR_V3_H6B2_DDR_CAPTURE_PASS") {
            throw "$Top did not report the final PASS marker"
        }

        $Capture = Join-Path $Work "ddr_capture_${Mhz}_${Width}.hex"
        if (-not (Test-Path -LiteralPath $Capture)) {
            throw "$Top did not create $Capture"
        }
        $Row = Compare-DdrCapture -CapturePath $Capture `
            -Width $Width -Golden $Golden
        $Row.processing_clock_mhz = $Mhz
        $Comparison += [pscustomobject]$Row
    }
}
finally {
    Pop-Location
    foreach ($Name in $SavedEnvironment.Keys) {
        $Value = $SavedEnvironment[$Name]
        if ($null -eq $Value) {
            Remove-Item "Env:$Name" -ErrorAction SilentlyContinue
        }
        else {
            Set-Item "Env:$Name" $Value
        }
    }
}

$Comparison | Select-Object processing_clock_mhz, width_bits,
    words_compared, bytes_compared, reserve_words_preserved,
    hsize_bytes, vsize_lines, stride_bytes, capture_sha256 |
    Export-Csv -NoTypeInformation -Encoding ASCII -LiteralPath (
        Join-Path $Work "comparison_summary.csv")

$Scenario = [ordered]@{
    checkpoint = "V3 H6-B2 HLS output DDR image versus V2/HTML Golden Vector"
    profiles = @("150 MHz / 32-bit", "200 MHz / 64-bit")
    scenario = $Golden.scenario
    compare_unit = "32-bit Word at every allocated DDR address"
    reserve_initial_value = "0xA5A5A5A5"
    stride_policy = "only HSIZE Words overwritten; reserve must remain initialized"
    html_golden_sha256 = (Get-FileHash -LiteralPath $GoldenFile -Algorithm SHA256).Hash
    board_exclusion = "Real AXI VDMA and DDR hardware remain board evidence"
}
$Scenario | ConvertTo-Json -Depth 5 | Set-Content -Encoding ASCII -LiteralPath (
    Join-Path $Work "scenario.json")

$ManifestFiles = @($SimFiles + $HlsFiles + $PSCommandPath + $GoldenVerifier +
    $GoldenFile + $HtmlModel) | Sort-Object -Unique
$Manifest = foreach ($File in $ManifestFiles) {
    $Item = Get-Item -LiteralPath $File
    $Hash = Get-FileHash -LiteralPath $File -Algorithm SHA256
    [ordered]@{
        path = $Item.FullName.Replace('\', '/')
        bytes = $Item.Length
        sha256 = $Hash.Hash
    }
}
$Manifest | ConvertTo-Json -Depth 3 | Set-Content -Encoding ASCII -LiteralPath (
    Join-Path $Work "source_manifest.json")

Write-Output "LIDAR_V3_H6B2_DDR_GOLDEN_PASS"
Write-Output "Result: $Work"
