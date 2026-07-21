param(
    [Parameter(Mandatory = $true)]
    [string]$HdlDir,
    [Parameter(Mandatory = $true)]
    [string]$ReportPath
)

$ErrorActionPreference = 'Stop'

function Get-EntityGenericNames {
    param(
        [Parameter(Mandatory = $true)][string]$Text,
        [Parameter(Mandatory = $true)][string]$EntityName
    )

    $entityPattern = '(?is)entity\s+' + [regex]::Escape($EntityName) +
        '\s+is\s*generic\s*\((?<body>.*?)\)\s*;\s*port\s*\('
    $entityMatch = [regex]::Match($Text, $entityPattern)
    if (-not $entityMatch.Success) {
        throw "Cannot find generic block for entity $EntityName"
    }

    return @(
        [regex]::Matches(
            $entityMatch.Groups['body'].Value,
            '(?m)^\s*(g_[A-Za-z0-9_]+)\s*:'
        ) | ForEach-Object { $_.Groups[1].Value }
    )
}

function Add-DuplicateFailures {
    param(
        [Parameter(Mandatory = $true)][string]$Scope,
        [Parameter(Mandatory = $true)][string[]]$Names,
        [Parameter(Mandatory = $true)]$Failures
    )

    foreach ($group in @($Names | Group-Object | Where-Object Count -gt 1)) {
        $Failures.Add("DUPLICATE $Scope $($group.Name) count=$($group.Count)")
    }
}

$topPath = Join-Path $HdlDir 'tdc_gpx_top.vhd'
$parentPath = Join-Path $HdlDir 'parent_ref/rtl/tdc_gpx_parent_core.vhd'
if (-not (Test-Path -LiteralPath $topPath)) {
    throw "Top RTL was not found: $topPath"
}
if (-not (Test-Path -LiteralPath $parentPath)) {
    throw "Parent wrapper RTL was not found: $parentPath"
}

$topText = Get-Content -Raw -LiteralPath $topPath
$parentText = Get-Content -Raw -LiteralPath $parentPath
$topNames = @(Get-EntityGenericNames -Text $topText -EntityName 'tdc_gpx_top')
$parentNames = @(Get-EntityGenericNames -Text $parentText -EntityName 'tdc_gpx_parent_core')

$mapMatch = [regex]::Match(
    $parentText,
    '(?is)entity\s+work\.tdc_gpx_top\s*generic\s+map\s*\((?<body>.*?)\)\s*port\s+map'
)
if (-not $mapMatch.Success) {
    throw 'Cannot find the tdc_gpx_top generic map in tdc_gpx_parent_core'
}

$mapPairs = @(
    [regex]::Matches(
        $mapMatch.Groups['body'].Value,
        '(?m)^\s*(g_[A-Za-z0-9_]+)\s*=>\s*(g_[A-Za-z0-9_]+)'
    ) | ForEach-Object {
        [pscustomobject]@{
            Formal = $_.Groups[1].Value
            Actual = $_.Groups[2].Value
        }
    }
)

$failures = [System.Collections.Generic.List[string]]::new()
$report = [System.Collections.Generic.List[string]]::new()
Add-DuplicateFailures -Scope 'top generic' -Names $topNames -Failures $failures
Add-DuplicateFailures -Scope 'parent generic' -Names $parentNames -Failures $failures
Add-DuplicateFailures -Scope 'generic map formal' -Names @($mapPairs.Formal) -Failures $failures

$topSet = [System.Collections.Generic.HashSet[string]]::new(
    [string[]]$topNames,
    [System.StringComparer]::Ordinal
)
$parentSet = [System.Collections.Generic.HashSet[string]]::new(
    [string[]]$parentNames,
    [System.StringComparer]::Ordinal
)
$mapByFormal = @{}
foreach ($pair in $mapPairs) {
    $mapByFormal[$pair.Formal] = $pair.Actual
}

foreach ($name in $topNames) {
    if (-not $parentSet.Contains($name)) {
        $failures.Add("MISSING parent generic $name")
    }
    if (-not $mapByFormal.ContainsKey($name)) {
        $failures.Add("MISSING tdc_gpx_top generic map $name")
    } elseif ($mapByFormal[$name] -cne $name) {
        $failures.Add("MISMATCH generic map $name=>$($mapByFormal[$name])")
    }
}

foreach ($name in $parentNames) {
    if (-not $topSet.Contains($name)) {
        $failures.Add("EXTRA parent generic $name")
    }
}
foreach ($formal in $mapByFormal.Keys) {
    if (-not $topSet.Contains($formal)) {
        $failures.Add("EXTRA tdc_gpx_top generic map $formal")
    }
}

$report.Add("top_generic_count=$($topNames.Count)")
$report.Add("parent_generic_count=$($parentNames.Count)")
$report.Add("mapped_generic_count=$($mapPairs.Count)")
foreach ($name in $topNames) {
    if ($parentSet.Contains($name) -and
        $mapByFormal.ContainsKey($name) -and
        $mapByFormal[$name] -ceq $name) {
        $report.Add("PASS $name=>$name")
    }
}

if ($failures.Count -eq 0) {
    $report.Add('TOP_PARENT_GENERIC_PARITY_PASS')
} else {
    foreach ($failure in $failures) {
        $report.Add("FAIL $failure")
    }
}

$reportDir = Split-Path -Parent $ReportPath
New-Item -ItemType Directory -Force -Path $reportDir | Out-Null
$report | Set-Content -LiteralPath $ReportPath -Encoding ascii

if ($failures.Count -ne 0) {
    throw "Top/parent generic parity failed ($($failures.Count) issue(s)); see $ReportPath"
}

Write-Host "Top/parent generic parity PASS ($($topNames.Count) generics)"
Write-Host "Generic parity report: $ReportPath"
