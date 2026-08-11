param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss"),
    [switch]$SkipDdrRegression,
    [string]$DdrSession = "",
    [switch]$SkipHlsSynthesis
)

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$V3Root = Split-Path -Parent $ScriptDir
$Hdl = (Resolve-Path (Join-Path $V3Root "..\..")).Path
$V2Root = Join-Path $Hdl "system_integration\v2"
$Work = Join-Path $Hdl ".work\v3_h6b2_ps_hline\$Stamp"
$DdrRegression = Join-Path $ScriptDir "run_v3_h6b2_ddr_golden.ps1"
$DdrGoldenFile = Join-Path $V2Root "golden\packed17_j9_ddr_golden.json"
$PsGoldenFile = Join-Path $V2Root (
    "golden\packed17_j10_ps_ethernet_golden.json")
$HtmlModel = Join-Path $Hdl (
    "Doc\cluster_analysis\C08_HDL_HTML_Alignment\" +
    "C08_HDL_HTML_Alignment_260806_PACKED17_VDMA_ABI_Simulator_v026.html")
$PsGoldenVerifier = Join-Path $V2Root (
    "scripts\verify_packed17_ps_golden.mjs")
$DecoderSource = Join-Path $V2Root (
    "sw_reference\lidar_packed17_ps_decoder.c")
$DecoderHeader = Join-Path $V2Root (
    "sw_reference\lidar_packed17_ps_decoder.h")
$DecoderCli = Join-Path $V2Root (
    "sw_reference\lidar_packed17_ps_decode_cli.c")
$HostGcc = "C:\AMDDesignTools\2025.2.1\tps\mingw\10.0.0\" +
    "win64.o\nt\bin\gcc.exe"
$ArmGcc = "C:\AMDDesignTools\2025.2.1\Vitis\gnu\aarch32\nt\" +
    "gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe"

function Invoke-Checked {
    param([string]$Executable, [string[]]$ToolArguments)

    & $Executable @ToolArguments
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed ($LASTEXITCODE): $Executable " +
            ($ToolArguments -join " ")
    }
}

function Convert-HexToBytes {
    param([string]$Hex)

    if (($Hex.Length % 2) -ne 0) {
        throw "Odd-length hexadecimal payload"
    }
    $Bytes = [byte[]]::new($Hex.Length / 2)
    for ($Index = 0; $Index -lt $Bytes.Length; $Index++) {
        $Bytes[$Index] = [Convert]::ToByte($Hex.Substring($Index * 2, 2), 16)
    }
    return $Bytes
}

function Assert-ByteArrayEqual {
    param(
        [byte[]]$Actual,
        [byte[]]$Expected,
        [string]$Context
    )

    if ($Actual.Length -ne $Expected.Length) {
        throw "$Context length mismatch: actual $($Actual.Length), " +
            "expected $($Expected.Length)"
    }
    for ($Index = 0; $Index -lt $Expected.Length; $Index++) {
        if ($Actual[$Index] -ne $Expected[$Index]) {
            throw "$Context byte mismatch at $Index`: actual " +
                ("0x{0:X2}" -f $Actual[$Index]) + ", expected " +
                ("0x{0:X2}" -f $Expected[$Index])
        }
    }
}

function Convert-DdrCapture {
    param(
        [string]$CapturePath,
        [string]$BinaryPath,
        [int]$Width,
        [object]$Golden
    )

    $Profiles = @($Golden.profiles | Where-Object {
        [int]$_.output_width_bits -eq $Width
    })
    if ($Profiles.Count -ne 1) {
        throw "DDR Golden profile count for width $Width is $($Profiles.Count)"
    }
    $Profile = $Profiles[0]
    $Lines = @(Get-Content -LiteralPath $CapturePath | Where-Object {
        -not [string]::IsNullOrWhiteSpace($_)
    })
    $ExpectedWords = @($Profile.memory_words)
    if ($Lines.Count -ne $ExpectedWords.Count) {
        throw "Width $Width DDR capture Word count mismatch"
    }

    $Bytes = [byte[]]::new($Lines.Count * 4)
    for ($Index = 0; $Index -lt $Lines.Count; $Index++) {
        if ($Lines[$Index] -notmatch
            '^0x([0-9A-Fa-f]{8})\s+0x([0-9A-Fa-f]{8})$') {
            throw "Malformed DDR capture at width $Width, line $($Index + 1)"
        }
        $Address = [Convert]::ToUInt32($Matches[1], 16)
        $Word = [Convert]::ToUInt32($Matches[2], 16)
        $ExpectedWord = [Convert]::ToUInt32(
            ([string]$ExpectedWords[$Index]).Substring(2), 16)
        if ($Address -ne ($Index * 4) -or $Word -ne $ExpectedWord) {
            throw "Width $Width DDR capture differs from Golden at Word $Index"
        }
        $Bytes[$Index * 4] = [byte]($Word -band 0xFF)
        $Bytes[$Index * 4 + 1] = [byte](($Word -shr 8) -band 0xFF)
        $Bytes[$Index * 4 + 2] = [byte](($Word -shr 16) -band 0xFF)
        $Bytes[$Index * 4 + 3] = [byte](($Word -shr 24) -band 0xFF)
    }
    [System.IO.File]::WriteAllBytes($BinaryPath, $Bytes)
    return $Profile
}

function Read-PacketCapture {
    param([string]$CapturePath)

    $Bytes = [System.IO.File]::ReadAllBytes($CapturePath)
    $Packets = [System.Collections.Generic.List[object]]::new()
    $Offset = 0
    while ($Offset -lt $Bytes.Length) {
        if (($Offset + 4) -gt $Bytes.Length) {
            throw "Truncated packet length in $CapturePath"
        }
        $Length = [BitConverter]::ToUInt32($Bytes, $Offset)
        $Offset += 4
        if ($Length -gt 1440 -or ($Offset + $Length) -gt $Bytes.Length) {
            throw "Invalid packet length $Length in $CapturePath"
        }
        $Payload = [byte[]]::new($Length)
        [Array]::Copy($Bytes, $Offset, $Payload, 0, $Length)
        $Packets.Add([pscustomobject]@{ Payload = $Payload })
        $Offset += $Length
    }
    return $Packets
}

function Compare-PacketGolden {
    param(
        [string]$CapturePath,
        [int]$Mhz,
        [object]$Golden
    )

    $Profiles = @($Golden.profiles | Where-Object {
        [int]$_.processing_clock_mhz -eq $Mhz
    })
    if ($Profiles.Count -ne 1) {
        throw "PS Golden profile count for $Mhz MHz is $($Profiles.Count)"
    }
    $ActualPackets = @(Read-PacketCapture -CapturePath $CapturePath)
    $ExpectedPackets = @($Profiles[0].packets)
    if ($ActualPackets.Count -ne $ExpectedPackets.Count) {
        throw "$Mhz MHz packet-count mismatch"
    }
    for ($Index = 0; $Index -lt $ExpectedPackets.Count; $Index++) {
        $Expected = Convert-HexToBytes -Hex ([string]$ExpectedPackets[$Index].hex)
        Assert-ByteArrayEqual -Actual $ActualPackets[$Index].Payload `
            -Expected $Expected -Context "$Mhz MHz packet $Index"
    }
    return [ordered]@{
        packet_count = $ActualPackets.Count
        face_header_bytes = $ActualPackets[0].Payload.Length
        hline_bytes = $ActualPackets[1].Payload.Length
        capture_sha256 = (Get-FileHash -LiteralPath $CapturePath `
            -Algorithm SHA256).Hash
    }
}

$RequiredFiles = @(
    $DdrRegression, $DdrGoldenFile, $PsGoldenFile, $HtmlModel,
    $PsGoldenVerifier, $DecoderSource, $DecoderHeader, $DecoderCli,
    $HostGcc, $ArmGcc
)
foreach ($File in $RequiredFiles) {
    if (-not (Test-Path -LiteralPath $File)) {
        throw "Required file is missing: $File"
    }
}

New-Item -ItemType Directory -Force -Path $Work | Out-Null
Invoke-Checked "node" @($PsGoldenVerifier, $HtmlModel, $PsGoldenFile)

if (-not $SkipDdrRegression) {
    $DdrStamp = "${Stamp}_ddr"
    $DdrArguments = @(
        "-NoProfile", "-ExecutionPolicy", "Bypass",
        "-File", $DdrRegression, "-Stamp", $DdrStamp
    )
    if ($SkipHlsSynthesis) {
        $DdrArguments += "-SkipHlsSynthesis"
    }
    Invoke-Checked "powershell.exe" $DdrArguments
    $DdrSession = Join-Path $Hdl ".work\v3_h6b2_ddr_golden\$DdrStamp"
}
if ([string]::IsNullOrWhiteSpace($DdrSession)) {
    throw "DdrSession is required when SkipDdrRegression is selected"
}
$DdrSession = (Resolve-Path -LiteralPath $DdrSession).Path

$DdrGolden = Get-Content -Raw -LiteralPath $DdrGoldenFile | ConvertFrom-Json
$PsGolden = Get-Content -Raw -LiteralPath $PsGoldenFile | ConvertFrom-Json
$HostDecoder = Join-Path $Work "lidar_packed17_ps_decode.exe"
$ArmObject = Join-Path $Work "lidar_packed17_ps_decoder_cortex_a9.o"
Invoke-Checked $HostGcc @(
    "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
    $DecoderSource, $DecoderCli, "-o", $HostDecoder
)
Invoke-Checked $ArmGcc @(
    "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
    "-mcpu=cortex-a9", "-marm", "-ffreestanding", "-c",
    $DecoderSource, "-o", $ArmObject
)

$Fixtures = @(
    [ordered]@{ Mhz = 150; Width = 32 },
    [ordered]@{ Mhz = 200; Width = 64 }
)
$Results = @()
foreach ($Fixture in $Fixtures) {
    $Mhz = $Fixture.Mhz
    $Width = $Fixture.Width
    $DdrCapture = Join-Path $DdrSession "ddr_capture_${Mhz}_${Width}.hex"
    if (-not (Test-Path -LiteralPath $DdrCapture)) {
        throw "V3 DDR capture is missing: $DdrCapture"
    }
    $DdrBinary = Join-Path $Work "ddr_${Mhz}_${Width}.bin"
    $DdrProfile = Convert-DdrCapture -CapturePath $DdrCapture `
        -BinaryPath $DdrBinary -Width $Width -Golden $DdrGolden

    $PacketCapture = Join-Path $Work "ps_capture_${Mhz}_${Width}.pkt"
    $DecoderLog = Join-Path $Work "ps_decode_${Mhz}_${Width}.log"
    $Arguments = @(
        $DdrBinary, $PacketCapture, [string]$Width,
        [string]$DdrProfile.geometry.hsize_bytes,
        [string]$DdrProfile.geometry.stride_bytes,
        [string]$DdrProfile.geometry.vsize_lines,
        [string]$DdrProfile.geometry.planned_shots,
        [string]($Mhz * 1000000),
        "81", "5", "14400", "6000", "7200", "8400", "1", "1"
    )
    $DecoderOutput = & $HostDecoder @Arguments 2>&1
    $DecoderOutput | Set-Content -Encoding ASCII -LiteralPath $DecoderLog
    if ($LASTEXITCODE -ne 0 -or
        ($DecoderOutput -join "`n") -notmatch "LIDAR_V2_PS_DECODE_PASS") {
        throw "PS decoder failed for $Mhz MHz / $Width-bit"
    }

    $PacketResult = Compare-PacketGolden -CapturePath $PacketCapture `
        -Mhz $Mhz -Golden $PsGolden
    $Results += [pscustomobject][ordered]@{
        processing_clock_mhz = $Mhz
        output_width_bits = $Width
        hsize_bytes = [int]$DdrProfile.geometry.hsize_bytes
        vsize_lines = [int]$DdrProfile.geometry.vsize_lines
        stride_bytes = [int]$DdrProfile.geometry.stride_bytes
        packet_count = $PacketResult.packet_count
        face_header_bytes = $PacketResult.face_header_bytes
        hline_bytes = $PacketResult.hline_bytes
        packet_capture_sha256 = $PacketResult.capture_sha256
        ddr_capture_sha256 = (Get-FileHash -LiteralPath $DdrCapture `
            -Algorithm SHA256).Hash
    }
}

$Results | Export-Csv -NoTypeInformation -Encoding ASCII -LiteralPath (
    Join-Path $Work "comparison_summary.csv")
$Scenario = [ordered]@{
    checkpoint = "V3 H6-B2 PS H-Line and Ethernet Golden comparison"
    ddr_session = $DdrSession.Replace('\', '/')
    profiles = @("150 MHz / 32-bit", "200 MHz / 64-bit")
    cache_ownership_contract = @(
        "DMA-owned decode is rejected",
        "CPU ownership requires completed cache synchronization",
        "decode after release to DMA is rejected"
    )
    packet_contract = @(
        "packet 0 is the 1440-byte Face Header payload",
        "packet 1 is one H-Line header plus PACKED17 samples repacked to 3 bytes"
    )
    board_exclusion =
        "Physical Zynq DMA cache invalidate/flush remains board evidence"
    cortex_a9_object_sha256 = (Get-FileHash -LiteralPath $ArmObject `
        -Algorithm SHA256).Hash
}
$Scenario | ConvertTo-Json -Depth 5 | Set-Content -Encoding ASCII `
    -LiteralPath (Join-Path $Work "scenario.json")

$ManifestFiles = @(
    $PSCommandPath, $DdrRegression, $DdrGoldenFile, $PsGoldenFile,
    $HtmlModel, $PsGoldenVerifier, $DecoderSource, $DecoderHeader, $DecoderCli
)
$Manifest = foreach ($File in $ManifestFiles) {
    $Item = Get-Item -LiteralPath $File
    [ordered]@{
        path = $Item.FullName.Replace('\', '/')
        bytes = $Item.Length
        sha256 = (Get-FileHash -LiteralPath $File -Algorithm SHA256).Hash
    }
}
$Manifest | ConvertTo-Json -Depth 3 | Set-Content -Encoding ASCII `
    -LiteralPath (Join-Path $Work "source_manifest.json")

Write-Output "LIDAR_V3_H6B2_PS_HLINE_ETHERNET_PASS"
Write-Output "Result: $Work"
