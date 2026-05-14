param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path,
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss"),
    [string]$Label = "manual",
    [switch]$IncludeLegacySimDirs,
    [switch]$WhatIf
)

$ErrorActionPreference = "Stop"

$RootPath = (Resolve-Path $Root).Path
$SafeLabel = ($Label -replace '[^0-9A-Za-z_.-]', '_')
$SessionName = "${Stamp}_${SafeLabel}"
$SessionRoot = Join-Path $RootPath "sim_results\vivado_xsim\sessions\$SessionName"

$Categories = @(
    "logs\compile",
    "logs\elaborate",
    "logs\simulate",
    "logs\vivado",
    "logs\journal",
    "waves",
    "work",
    "tmp",
    "crash"
)

foreach ($cat in $Categories) {
    if (-not $WhatIf) {
        New-Item -ItemType Directory -Force -Path (Join-Path $SessionRoot $cat) | Out-Null
    }
}

$Moved = New-Object System.Collections.Generic.List[object]

function Get-RelativePath {
    param([string]$Path)
    $full = [System.IO.Path]::GetFullPath($Path)
    $root = [System.IO.Path]::GetFullPath($RootPath)
    if (-not $root.EndsWith([System.IO.Path]::DirectorySeparatorChar)) {
        $root += [System.IO.Path]::DirectorySeparatorChar
    }
    return $full.Substring($root.Length)
}

function Test-PathInsideRoot {
    param([string]$Path)
    $full = [System.IO.Path]::GetFullPath($Path)
    $root = [System.IO.Path]::GetFullPath($RootPath)
    if (-not $root.EndsWith([System.IO.Path]::DirectorySeparatorChar)) {
        $root += [System.IO.Path]::DirectorySeparatorChar
    }
    return $full.StartsWith($root, [System.StringComparison]::OrdinalIgnoreCase)
}

function Test-GitTracked {
    param([string]$Path)
    $rel = (Get-RelativePath $Path).Replace('\', '/')
    $matches = & git -C $RootPath ls-files -- $rel
    return ($matches -contains $rel)
}

function Test-GitTrackedDescendant {
    param([string]$Path)
    $rel = (Get-RelativePath $Path).TrimEnd('\', '/').Replace('\', '/')
    $matches = & git -C $RootPath ls-files -- "$rel/*"
    return ($matches.Count -gt 0)
}

function Move-Artifact {
    param(
        [System.IO.FileSystemInfo]$Item,
        [string]$Category
    )

    if (-not (Test-PathInsideRoot $Item.FullName)) {
        throw "Refusing to move path outside workspace: $($Item.FullName)"
    }

    if ($Item.PSIsContainer) {
        if (Test-GitTrackedDescendant $Item.FullName) {
            Write-Host "skip tracked directory: $($Item.FullName)"
            return
        }
    }
    elseif (Test-GitTracked $Item.FullName) {
        Write-Host "skip tracked file: $($Item.FullName)"
        return
    }

    $destDir = Join-Path $SessionRoot $Category
    $dest = Join-Path $destDir $Item.Name
    if (-not $WhatIf) {
        New-Item -ItemType Directory -Force -Path $destDir | Out-Null
        if (Test-Path -LiteralPath $dest) {
            $suffix = Get-Date -Format "HHmmssfff"
            $dest = Join-Path $destDir "$($Item.BaseName)_$suffix$($Item.Extension)"
        }
        Move-Item -LiteralPath $Item.FullName -Destination $dest
    }

    $Moved.Add([pscustomobject]@{
        Source = Get-RelativePath $Item.FullName
        Destination = if ($WhatIf) { Join-Path $Category $Item.Name } else { Get-RelativePath $dest }
        Category = $Category
        Bytes = if ($Item.PSIsContainer) { 0 } else { $Item.Length }
        LastWriteTime = $Item.LastWriteTime.ToString("yyyy-MM-dd HH:mm:ss")
    }) | Out-Null
}

$FileRules = @(
    @{ Filter = "xvlog*.log"; Category = "logs\compile" },
    @{ Filter = "xvhdl*.log"; Category = "logs\compile" },
    @{ Filter = "xelab*.log"; Category = "logs\elaborate" },
    @{ Filter = "xsim*.log"; Category = "logs\simulate" },
    @{ Filter = "vivado*.log"; Category = "logs\vivado" },
    @{ Filter = "*.jou"; Category = "logs\journal" },
    @{ Filter = "*.pb"; Category = "work" },
    @{ Filter = "*.wdb"; Category = "waves" },
    @{ Filter = "dfx_runtime.txt"; Category = "work" },
    @{ Filter = "hs_err_pid*.log"; Category = "crash" },
    @{ Filter = "hs_err_pid*.dmp"; Category = "crash" }
)

foreach ($rule in $FileRules) {
    Get-ChildItem -LiteralPath $RootPath -File -Filter $rule.Filter -ErrorAction SilentlyContinue |
        ForEach-Object { Move-Artifact $_ $rule.Category }
}

$DirRules = @(
    @{ Name = "xsim.dir"; Category = "work" },
    @{ Name = ".Xil"; Category = "work" },
    @{ Name = "tmp"; Category = "tmp" },
    @{ Name = "-p"; Category = "work" }
)

foreach ($rule in $DirRules) {
    $path = Join-Path $RootPath $rule.Name
    if (Test-Path -LiteralPath $path) {
        Move-Artifact (Get-Item -LiteralPath $path) $rule.Category
    }
}

if ($IncludeLegacySimDirs) {
    Get-ChildItem -LiteralPath $RootPath -Directory -Filter "sim_*" -ErrorAction SilentlyContinue |
        Where-Object { $_.Name -ne "sim_results" -and $_.Name -ne "sim_work" } |
        ForEach-Object { Move-Artifact $_ "work" }
}

if (-not $WhatIf) {
    $manifestCsv = Join-Path $SessionRoot "manifest.csv"
    $Moved | Export-Csv -NoTypeInformation -Encoding UTF8 $manifestCsv

    $readme = Join-Path $SessionRoot "README.md"
    $created = Get-Date -Format "yyyy-MM-dd HH:mm:ss K"
    @(
        "# Vivado xsim Session Archive",
        "",
        "| Field | Value |",
        "|---|---|",
        "| Stamp | $Stamp |",
        "| Label | $SafeLabel |",
        "| Root | $RootPath |",
        "| Created | $created |",
        "| Artifact count | $($Moved.Count) |",
        "",
        "## Folder Structure",
        "",
        "| Folder | Contents |",
        "|---|---|",
        '| `logs/compile` | `xvlog`, `xvhdl` compile log |',
        '| `logs/elaborate` | `xelab` elaboration log |',
        '| `logs/simulate` | `xsim` simulation log |',
        '| `logs/vivado` | Vivado batch/project log |',
        '| `logs/journal` | `.jou` journal |',
        '| `waves` | `.wdb` waveform database |',
        '| `work` | `.pb`, `xsim.dir`, `.Xil`, legacy sim work dirs |',
        '| `tmp` | script-generated temporary project/argument files |',
        '| `crash` | JVM crash dump/log |',
        "",
        "## Manifest",
        "",
        'Detailed move records are tracked in `manifest.csv`.'
    ) | Set-Content -Encoding UTF8 $readme
}

Write-Host "Vivado/xsim archive session: $SessionRoot"
Write-Host "Artifacts moved: $($Moved.Count)"
