param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss"),
    [switch]$SkipDdrRegression,
    [string]$DdrSession = ""
)

$ErrorActionPreference = "Stop"

$Hdl = (Resolve-Path (Join-Path $PSScriptRoot "../../..")).Path
$WorkRoot = Join-Path $Hdl "tmp/v2_gpx_ps_hline"
$Work = Join-Path $WorkRoot $Stamp
$Archive = Join-Path $Hdl (
    "signoff_results/sessions/${Stamp}_v2_gpx_ps_hline")
$DdrGolden = Join-Path $Hdl (
    "system_integration/v2/golden/packed17_j9_ddr_golden.json")
$PsGolden = Join-Path $Hdl (
    "system_integration/v2/golden/packed17_j10_ps_ethernet_golden.json")
$HtmlModel = Join-Path $Hdl (
    "Doc/cluster_analysis/C08_HDL_HTML_Alignment/" +
    "C08_HDL_HTML_Alignment_260806_PACKED17_VDMA_ABI_Simulator_v026.html")
$Materializer = Join-Path $Hdl (
    "system_integration/v2/scripts/materialize_packed17_ddr.mjs")
$Verifier = Join-Path $Hdl (
    "system_integration/v2/scripts/verify_packed17_ps_golden.mjs")
$DdrRegression = Join-Path $Hdl (
    "system_integration/v2/scripts/run_v2_gpx_ddr_golden.ps1")
$DecoderSource = Join-Path $Hdl (
    "system_integration/v2/sw_reference/lidar_packed17_ps_decoder.c")
$DecoderHeader = Join-Path $Hdl (
    "system_integration/v2/sw_reference/lidar_packed17_ps_decoder.h")
$DecoderCli = Join-Path $Hdl (
    "system_integration/v2/sw_reference/lidar_packed17_ps_decode_cli.c")
$HostGcc = "C:/AMDDesignTools/2025.2.1/tps/mingw/10.0.0/win64.o/nt/bin/gcc.exe"
$ArmGcc = "C:/AMDDesignTools/2025.2.1/Vitis/gnu/aarch32/nt/" +
    "gcc-arm-none-eabi/bin/arm-none-eabi-gcc.exe"

function Invoke-Checked {
    param([string]$Exe, [string[]]$ArgList)
    & $Exe @ArgList
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed ($LASTEXITCODE): $Exe $($ArgList -join ' ')"
    }
}

function Get-PacketLengths {
    param([string]$CapturePath)

    $Bytes = [System.IO.File]::ReadAllBytes($CapturePath)
    $Offset = 0
    $Lengths = @()
    while ($Offset -lt $Bytes.Length) {
        if (($Offset + 4) -gt $Bytes.Length) {
            throw "Truncated packet length in $CapturePath"
        }
        $Length = [BitConverter]::ToUInt32($Bytes, $Offset)
        $Offset += 4
        if ($Length -gt 1440 -or ($Offset + $Length) -gt $Bytes.Length) {
            throw "Invalid packet length $Length in $CapturePath"
        }
        $Lengths += $Length
        $Offset += $Length
    }
    return $Lengths
}

New-Item -ItemType Directory -Force -Path $Work | Out-Null

Invoke-Checked "node" @($Verifier, $HtmlModel, $PsGolden)

$DdrCaptureDirectory = ""
if (-not $SkipDdrRegression) {
    $J9Stamp = "${Stamp}_j9"
    Invoke-Checked "powershell.exe" @(
        "-NoProfile", "-ExecutionPolicy", "Bypass",
        "-File", $DdrRegression, "-Stamp", $J9Stamp)
    $DdrCaptureDirectory = Join-Path $Hdl (
        "signoff_results/sessions/${J9Stamp}_v2_gpx_ddr_golden")
} elseif (-not [string]::IsNullOrWhiteSpace($DdrSession)) {
    $DdrCaptureDirectory = (Resolve-Path -LiteralPath $DdrSession).Path
}

$MaterializeArgs = @($Materializer, $DdrGolden, $Work)
if (-not [string]::IsNullOrWhiteSpace($DdrCaptureDirectory)) {
    $MaterializeArgs += $DdrCaptureDirectory
}
Invoke-Checked "node" $MaterializeArgs

$HostDecoder = Join-Path $Work "lidar_packed17_ps_decode.exe"
Invoke-Checked $HostGcc @(
    "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
    $DecoderSource, $DecoderCli, "-o", $HostDecoder)

$ArmObject = Join-Path $Work "lidar_packed17_ps_decoder_cortex_a9.o"
Invoke-Checked $ArmGcc @(
    "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
    "-mcpu=cortex-a9", "-marm", "-ffreestanding", "-c",
    $DecoderSource, "-o", $ArmObject)

$Fixtures = Get-Content -Raw -LiteralPath (
    Join-Path $Work "ddr_fixture_manifest.json") | ConvertFrom-Json
$Results = @()
foreach ($Fixture in $Fixtures) {
    $Mhz = [int]$Fixture.processing_clock_mhz
    $Width = [int]$Fixture.output_width_bits
    $Capture = Join-Path $Work "ps_capture_${Mhz}_${Width}.pkt"
    $DecoderLog = Join-Path $Work "ps_decode_${Mhz}_${Width}.log"
    $Arguments = @(
        [string]$Fixture.input_file,
        $Capture,
        [string]$Width,
        [string]$Fixture.geometry.hsize_bytes,
        [string]$Fixture.geometry.stride_bytes,
        [string]$Fixture.geometry.vsize_lines,
        [string]$Fixture.geometry.planned_shots,
        [string]($Mhz * 1000000),
        "81", "5", "14400", "6000", "7200", "8400", "1", "1")

    $Output = & $HostDecoder @Arguments 2>&1
    $Output | Set-Content -Encoding ASCII -LiteralPath $DecoderLog
    if ($LASTEXITCODE -ne 0 -or
        ($Output -join "`n") -notmatch "LIDAR_V2_PS_DECODE_PASS") {
        throw "PS decoder failed for ${Mhz} MHz / ${Width}-bit"
    }

    $Lengths = @(Get-PacketLengths -CapturePath $Capture)
    $Results += [pscustomobject][ordered]@{
        processing_clock_mhz = $Mhz
        output_width_bits = $Width
        packet_count = $Lengths.Count
        packet_lengths_bytes = ($Lengths -join "+")
        application_payload_bytes = ($Lengths | Measure-Object -Sum).Sum
        capture_sha256 = (Get-FileHash -LiteralPath $Capture `
            -Algorithm SHA256).Hash
        ddr_source = [string]$Fixture.input_source
    }
}

Invoke-Checked "node" @($Verifier, $HtmlModel, $PsGolden, $Work)

foreach ($Mhz in @(150, 200)) {
    $Rows = @($Results | Where-Object {
        $_.processing_clock_mhz -eq $Mhz
    })
    if ($Rows.Count -ne 3 -or
        @($Rows.capture_sha256 | Select-Object -Unique).Count -ne 1) {
        throw "Transport-width normalization failed at ${Mhz} MHz"
    }
}

New-Item -ItemType Directory -Force -Path $Archive | Out-Null
$Results | Sort-Object processing_clock_mhz, output_width_bits |
    Export-Csv -NoTypeInformation -Encoding ASCII -LiteralPath (
        Join-Path $Archive "comparison_summary.csv")

$Scenario = [ordered]@{
    checkpoint = "J10 PS H-Line and Ethernet byte comparison"
    source_ddr_session = $DdrCaptureDirectory
    processing_clocks_mhz = @(150, 200)
    output_widths_bits = @(32, 64, 128)
    cache_contract = @(
        "DMA-owned decode rejected",
        "cache-sync-complete required before CPU ownership",
        "released buffer decode rejected")
    wire_contract = @(
        "packet 0: 1440-byte Face Header",
        "packet 1+: 32-byte H-Line Header plus 3-byte samples")
    board_exclusion = "Physical DMA cache invalidation remains J11 board evidence"
    cortex_a9_object_sha256 = (Get-FileHash -LiteralPath $ArmObject `
        -Algorithm SHA256).Hash
}
$Scenario | ConvertTo-Json -Depth 5 |
    Set-Content -Encoding ASCII -LiteralPath (
        Join-Path $Archive "scenario.json")

$ManifestFiles = @(
    $PSCommandPath, $Materializer, $Verifier, $DdrGolden, $PsGolden,
    $HtmlModel, $DecoderSource, $DecoderHeader, $DecoderCli)
$Manifest = foreach ($File in $ManifestFiles) {
    $Item = Get-Item -LiteralPath $File
    $Hash = Get-FileHash -LiteralPath $File -Algorithm SHA256
    [ordered]@{
        path = $Item.FullName.Replace('\', '/')
        bytes = $Item.Length
        sha256 = $Hash.Hash
    }
}
$Manifest | ConvertTo-Json -Depth 3 |
    Set-Content -Encoding ASCII -LiteralPath (
        Join-Path $Archive "source_manifest.json")

$Artifacts = Get-ChildItem -File -LiteralPath $Work | Where-Object {
    $_.Name -match '^ps_capture_.*\.pkt$|^ps_decode_.*\.log$|' +
        '^ddr_fixture_manifest\.json$|cortex_a9\.o$'
} | ForEach-Object { $_.FullName }
if ($Artifacts) {
    Copy-Item -Force -LiteralPath $Artifacts -Destination $Archive
}
Copy-Item -Force -LiteralPath $PsGolden -Destination $Archive

$ResolvedWork = (Resolve-Path -LiteralPath $Work).Path
$ResolvedRoot = (Resolve-Path -LiteralPath $WorkRoot).Path
if (-not $ResolvedWork.StartsWith($ResolvedRoot,
        [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "Refusing to remove work directory outside $ResolvedRoot"
}
Remove-Item -LiteralPath $ResolvedWork -Recurse -Force

Write-Output "LIDAR_V2_GPX_PS_HLINE_SIGNOFF_PASS"
Write-Output "Result: $Archive"
