param()

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$V2Dir = Split-Path -Parent $ScriptDir
$TbDir = Join-Path $V2Dir "tb"
$Guide = Join-Path (Split-Path -Parent $V2Dir) `
    "v2_architecture/V2_TESTBENCH_COVERAGE_GUIDE_KO.md"

if (-not (Test-Path -LiteralPath $Guide)) {
    throw "Testbench coverage guide is missing: $Guide"
}

$Assets = @(Get-ChildItem -LiteralPath $TbDir -Filter "*.vhd" -File |
    Sort-Object Name)
$RunScripts = @(Get-ChildItem -LiteralPath $ScriptDir -Filter "run_v2_*.ps1" -File)
$RunText = ($RunScripts | ForEach-Object {
    Get-Content -Raw -Encoding utf8 -LiteralPath $_.FullName
}) -join "`n"
$GuideText = Get-Content -Raw -Encoding utf8 -LiteralPath $Guide

function ConvertFrom-UnicodeCodePoint {
    param([string[]]$CodePoint)

    return -join ($CodePoint | ForEach-Object {
        [char][Convert]::ToInt32($_, 16)
    })
}

# Windows PowerShell 5.1은 UTF-8 BOM이 없는 script의 한글 literal을 ANSI로
# 해석할 수 있다. 검사 대상 VHDL은 UTF-8로 읽고, marker는 code point로 만든다.
$RequiredMarkers = @(
    (ConvertFrom-UnicodeCodePoint @("D14C", "C2A4", "D2B8", "0020", "C790", "C0B0", "0020", "BAA9", "C801", "003A")),
    (ConvertFrom-UnicodeCodePoint @("D575", "C2EC", "0020", "AC80", "C99D", "0020", "ACC4", "C57D", "003A")),
    (ConvertFrom-UnicodeCodePoint @("C2E4", "D589", "0020", "D68C", "ADC0", "003A")),
    (ConvertFrom-UnicodeCodePoint @("C720", "C9C0", "BCF4", "C218", "0020", "C8FC", "C758", "003A"))
)
$RelatedRtlMarker = ConvertFrom-UnicodeCodePoint @(
    "AD00", "B828", "0020", "0052", "0054", "004C", "003A")
$RelatedRtlTbMarker = ConvertFrom-UnicodeCodePoint @(
    "AD00", "B828", "0020", "0052", "0054", "004C", "002F", "0054", "0042", "003A")

$Failures = [System.Collections.Generic.List[string]]::new()
foreach ($Asset in $Assets) {
    $Text = Get-Content -Raw -Encoding utf8 -LiteralPath $Asset.FullName
    foreach ($Marker in $RequiredMarkers) {
        if (-not $Text.Contains($Marker)) {
            $Failures.Add("$($Asset.Name): missing header marker '$Marker'")
        }
    }
    if (-not $Text.Contains($RelatedRtlMarker) -and
            -not $Text.Contains($RelatedRtlTbMarker)) {
        $Failures.Add("$($Asset.Name): missing related-RTL header marker")
    }
    if (-not $GuideText.Contains("``$($Asset.Name)``")) {
        $Failures.Add("$($Asset.Name): missing from coverage guide")
    }
    if (-not $RunText.Contains($Asset.Name)) {
        $Failures.Add("$($Asset.Name): not referenced by a run_v2_*.ps1 regression")
    }
}

if ($Failures.Count -ne 0) {
    $Failures | ForEach-Object { Write-Error $_ }
    throw "V2 testbench documentation coverage failed: $($Failures.Count) issue(s)"
}

$PrimaryCount = @($Assets | Where-Object { $_.Name -like "tb_*.vhd" -and
    $_.Name -notlike "*_profiles.vhd" }).Count
$ProfileCount = @($Assets | Where-Object { $_.Name -like "*_profiles.vhd" }).Count
$HarnessCount = @($Assets | Where-Object { $_.Name -like "*_impl.vhd" }).Count

$PassMessage = "LIDAR_V2_TESTBENCH_DOC_COVERAGE_PASS files={0} primary={1} " +
    "profiles={2} harnesses={3}"
Write-Host ($PassMessage -f $Assets.Count, $PrimaryCount, $ProfileCount,
    $HarnessCount)
