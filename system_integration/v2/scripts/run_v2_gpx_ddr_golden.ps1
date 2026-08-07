param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss"),
    [ValidateSet(0, 150, 200)]
    [int]$OnlyMhz = 0,
    [ValidateSet(0, 32, 64, 128)]
    [int]$OnlyWidth = 0
)

$ErrorActionPreference = "Stop"

$Hdl = (Resolve-Path (Join-Path $PSScriptRoot "../../..")).Path
$Vivado = "C:/AMDDesignTools/2025.2.1/Vivado/bin"
$WorkRoot = Join-Path $Hdl "tmp/v2_gpx_ddr_golden"
$Work = Join-Path $WorkRoot $Stamp
$Archive = Join-Path $Hdl (
    "signoff_results/sessions/${Stamp}_v2_gpx_ddr_golden")
$HtmlModel = Join-Path $Hdl (
    "Doc/cluster_analysis/C08_HDL_HTML_Alignment/" +
    "C08_HDL_HTML_Alignment_260806_PACKED17_VDMA_ABI_Simulator_v026.html")
$GoldenFile = Join-Path $Hdl (
    "system_integration/v2/golden/packed17_j9_ddr_golden.json")
$GoldenVerifier = Join-Path $Hdl (
    "system_integration/v2/scripts/verify_packed17_html_golden.mjs")

New-Item -ItemType Directory -Force -Path $Work | Out-Null

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

Invoke-Checked "node" @($GoldenVerifier, $HtmlModel, $GoldenFile)
$Golden = Get-Content -Raw -LiteralPath $GoldenFile | ConvertFrom-Json

$CommonFiles = @(
    "$Hdl/system_integration/v2/pkg/lidar_build_pkg.vhd",
    "$Hdl/system_integration/v2/pkg/lidar_config_types_pkg.vhd",
    "$Hdl/system_integration/v2/pkg/lidar_event_types_pkg.vhd",
    "$Hdl/system_integration/v2/pkg/lidar_gpx_pkg.vhd",
    "$Hdl/system_integration/v2/pkg/lidar_gpx_event_pkg.vhd",
    "$Hdl/system_integration/v2/pkg/lidar_gpx_data_pkg.vhd",
    "$Hdl/system_integration/v2/pkg/lidar_gpx_vdma_pkg.vhd",
    "$Hdl/system_integration/v2/rtl/proc/lidar_gpx_vdma_profile_manager.vhd",
    "$Hdl/system_integration/v2/rtl/proc/lidar_gpx_cell_word_serializer.vhd",
    "$Hdl/system_integration/v2/rtl/proc/lidar_gpx_shot_line_builder.vhd",
    "$Hdl/system_integration/v2/rtl/proc/lidar_gpx_hole_line_expander.vhd",
    "$Hdl/system_integration/v2/rtl/proc/lidar_gpx_face_footer_builder.vhd",
    "$Hdl/system_integration/v2/rtl/proc/lidar_gpx_axis_word_packer.vhd",
    "$Hdl/system_integration/v2/rtl/proc/lidar_gpx_axis_lane_pipeline.vhd"
)
$SimFiles = $CommonFiles + @(
    "$Hdl/system_integration/v2/tb/tb_lidar_gpx_ddr_golden.vhd"
)

$Project = Join-Path $Work "v2_gpx_ddr_golden_vhdl.prj"
$ProjectLines = foreach ($File in $SimFiles) {
    "vhdl2008 xil_defaultlib `"$($File.Replace('\', '/'))`""
}
$ProjectLines += "nosort"
$ProjectLines | Set-Content -Encoding ASCII -LiteralPath $Project
$RunTcl = Join-Path $Work "run.tcl"
@("run all", "quit") | Set-Content -Encoding ASCII -LiteralPath $RunTcl

$ClockValues = if ($OnlyMhz -eq 0) { @(150, 200) } else { @($OnlyMhz) }
$WidthValues = if ($OnlyWidth -eq 0) { @(32, 64, 128) } else { @($OnlyWidth) }
$Comparison = @()

Push-Location $Work
try {
    Invoke-Checked "$Vivado/xvhdl.bat" @(
        "--relax", "-prj", $Project,
        "-log", (Join-Path $Work "xvhdl.log")
    )

    foreach ($Mhz in $ClockValues) {
        foreach ($Width in $WidthValues) {
            $Top = "tb_lidar_gpx_ddr_golden_${Mhz}_${Width}"
            $Snapshot = "${Top}_${Stamp}_snap"
            $SimLog = Join-Path $Work "xsim_${Top}.log"
            Invoke-Checked "$Vivado/xelab.bat" @(
                "--debug", "off", "--relax", "--mt", "2",
                "--snapshot", $Snapshot, "xil_defaultlib.$Top",
                "-log", (Join-Path $Work "xelab_${Top}.log")
            )
            Invoke-Checked "$Vivado/xsim.bat" @(
                $Snapshot, "-tclbatch", $RunTcl.Replace('\', '/'),
                "-log", $SimLog.Replace('\', '/')
            )
            if ((Get-Content -Raw -LiteralPath $SimLog) -notmatch
                "LIDAR_V2_GPX_DDR_CAPTURE_PASS") {
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
}
finally {
    Pop-Location
}

New-Item -ItemType Directory -Force -Path $Archive | Out-Null
$Comparison | Select-Object processing_clock_mhz, width_bits,
    words_compared, bytes_compared, reserve_words_preserved,
    hsize_bytes, vsize_lines, stride_bytes, capture_sha256 |
    Export-Csv -NoTypeInformation -Encoding ASCII -LiteralPath (
        Join-Path $Archive "comparison_summary.csv")

$Scenario = [ordered]@{
    checkpoint = "J9 XSIM DDR memory image versus HTML Golden Vector"
    processing_clocks_mhz = @($ClockValues)
    output_widths = @($WidthValues)
    scenario = $Golden.scenario
    compare_unit = "32-bit Word at every allocated DDR address"
    reserve_initial_value = "0xA5A5A5A5"
    stride_policy = "only HSIZE Words overwritten; reserve must remain initialized"
    html_golden_sha256 = (Get-FileHash -LiteralPath $GoldenFile -Algorithm SHA256).Hash
}
$Scenario | ConvertTo-Json -Depth 5 | Set-Content -Encoding ASCII -LiteralPath (
    Join-Path $Archive "scenario.json")

$ManifestFiles = @($SimFiles + $PSCommandPath + $GoldenVerifier +
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
    Join-Path $Archive "source_manifest.json")

Copy-Item -Force -LiteralPath $GoldenFile -Destination $Archive
$Artifacts = Get-ChildItem -File -LiteralPath $Work | Where-Object {
    $_.Name -match '^(xvhdl|xelab|xsim).*\.log$|^ddr_capture_.*\.hex$|\.prj$|^run\.tcl$'
} | ForEach-Object { $_.FullName }
if ($Artifacts) {
    Copy-Item -Force -LiteralPath $Artifacts -Destination $Archive
}

$ResolvedWork = (Resolve-Path -LiteralPath $Work).Path
$ResolvedRoot = (Resolve-Path -LiteralPath $WorkRoot).Path
if (-not $ResolvedWork.StartsWith($ResolvedRoot,
        [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "Refusing to remove work directory outside $ResolvedRoot"
}
Remove-Item -LiteralPath $ResolvedWork -Recurse -Force

Write-Output "LIDAR_V2_GPX_DDR_GOLDEN_PASS"
Write-Output "Result: $Archive"
