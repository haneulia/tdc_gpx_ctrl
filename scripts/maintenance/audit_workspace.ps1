[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [ValidateSet("V2", "V3")]
    [string]$Profile,

    [switch]$RequireCopyIsolation,
    [switch]$DeepGitCheck
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$failures = New-Object System.Collections.Generic.List[string]
$warnings = New-Object System.Collections.Generic.List[string]

function Add-CheckFailure {
    param([string]$Message)
    $script:failures.Add($Message)
    Write-Host "[FAIL] $Message" -ForegroundColor Red
}

function Add-CheckWarning {
    param([string]$Message)
    $script:warnings.Add($Message)
    Write-Host "[WARN] $Message" -ForegroundColor Yellow
}

function Add-CheckPass {
    param([string]$Message)
    Write-Host "[PASS] $Message" -ForegroundColor Green
}

function Require-Path {
    param(
        [string]$Root,
        [string]$RelativePath
    )

    $path = Join-Path $Root $RelativePath
    if (Test-Path -LiteralPath $path) {
        Add-CheckPass "required path: $RelativePath"
    } else {
        Add-CheckFailure "missing required path: $RelativePath"
    }
}

function Get-NormalizedPath {
    param([string]$Path)
    return [System.IO.Path]::GetFullPath($Path).TrimEnd('\')
}

$repoRootText = & git rev-parse --show-toplevel 2>&1
if ($LASTEXITCODE -ne 0) {
    throw "This script must run inside a Git worktree: $repoRootText"
}
$repoRoot = Get-NormalizedPath (($repoRootText | Select-Object -First 1).Trim())

if ($Profile -eq "V2") {
    $baselineCommit = "8c97dd251b70ee602305bfc2eb7ca51ddebc5c6e"
    $completeBranch = "codex/tdc-gpx-lidar-v2-complete"
    $maintenanceBranch = "codex/tdc-gpx-lidar-v2-maintenance"
    $requiredPaths = @(
        "system_integration/v2/V2_COMPLETE_BRANCH_KO.md",
        "system_integration/v2/pkg",
        "system_integration/v2/rtl",
        "system_integration/v2/tb",
        "system_integration/v2/ip_package/v2_ip_package_manifest.tcl",
        "system_integration/v2/ip_package/tdc_gpx_lidar_ctrl_v2_xgui.tcl",
        "system_integration/v2/ip_repo/tdc_gpx_lidar_ctrl_v2_2_0/component.xml"
    )
    $packageRoots = @(
        "system_integration/v2/ip_package",
        "system_integration/v2/ip_repo"
    )
    $passMarker = "LIDAR_V2_MAINTENANCE_WORKSPACE_AUDIT_PASS"
} else {
    $baselineCommit = "bd4041474d16c2fe856de410b346ab26ffc73885"
    $completeBranch = "codex/tdc-gpx-lidar-v3-complete"
    $maintenanceBranch = "codex/tdc-gpx-lidar-v3-maintenance"
    $requiredPaths = @(
        "system_integration/v3/V3_COMPLETE_BRANCH_KO.md",
        "system_integration/v2/pkg",
        "system_integration/v2/rtl",
        "system_integration/v3/hls",
        "system_integration/v3/rtl",
        "system_integration/v3/tb",
        "system_integration/v3/ip_package/v3_ip_package_manifest.tcl",
        "system_integration/v3/ip_package/tdc_gpx_lidar_ctrl_v3_xgui.tcl",
        "system_integration/v3/ip_repo/tdc_gpx_lidar_ctrl_v3_3_0/component.xml",
        "system_integration/v3/ip_repo/tdc_gpx_lidar_ctrl_v3_hls_ip_3_0/component.xml"
    )
    $packageRoots = @(
        "system_integration/v3/ip_package",
        "system_integration/v3/ip_repo"
    )
    $passMarker = "LIDAR_V3_MAINTENANCE_WORKSPACE_AUDIT_PASS"
}

Write-Host "Maintenance workspace audit"
Write-Host "  Profile : $Profile"
Write-Host "  Root    : $repoRoot"

$branchText = & git -C $repoRoot branch --show-current 2>&1
if ($LASTEXITCODE -ne 0) {
    Add-CheckFailure "cannot read current branch: $branchText"
    $currentBranch = ""
} else {
    $currentBranch = ($branchText | Select-Object -First 1).Trim()
    if ($currentBranch -eq $completeBranch) {
        Add-CheckFailure "the frozen complete branch is checked out; switch to $maintenanceBranch"
    } elseif ($currentBranch -eq $maintenanceBranch) {
        Add-CheckPass "maintenance branch: $currentBranch"
    } else {
        Add-CheckWarning "feature branch detected: $currentBranch (expected parent: $maintenanceBranch)"
    }
}

& git -C $repoRoot cat-file -e "$baselineCommit^{commit}" 2>$null
if ($LASTEXITCODE -ne 0) {
    Add-CheckFailure "baseline commit is missing: $baselineCommit"
} else {
    & git -C $repoRoot merge-base --is-ancestor $baselineCommit HEAD
    if ($LASTEXITCODE -eq 0) {
        Add-CheckPass "baseline commit is an ancestor of HEAD"
    } else {
        Add-CheckFailure "HEAD does not descend from baseline $baselineCommit"
    }
}

Require-Path $repoRoot "system_integration/common/rtl/csr/axil_fsm_32.vhd"
Require-Path $repoRoot "system_integration/common/rtl/csr/axil_intr_32.vhd"
Require-Path $repoRoot "tdc_gpx_pkg.vhd"
Require-Path $repoRoot "tdc_gpx_bus_phy.vhd"
foreach ($relativePath in $requiredPaths) {
    Require-Path $repoRoot $relativePath
}

if ($Profile -eq "V2" -and (Test-Path -LiteralPath (Join-Path $repoRoot "system_integration/v3"))) {
    Add-CheckFailure "V2 workspace unexpectedly contains system_integration/v3"
} elseif ($Profile -eq "V2") {
    Add-CheckPass "V2 workspace excludes the V3 implementation tree"
}

$metadataFiles = New-Object System.Collections.Generic.List[System.IO.FileInfo]
foreach ($relativeRoot in $packageRoots) {
    $fullRoot = Join-Path $repoRoot $relativeRoot
    if (Test-Path -LiteralPath $fullRoot) {
        Get-ChildItem -LiteralPath $fullRoot -Recurse -File | Where-Object {
            $_.Name -eq "component.xml" -or $_.Extension -eq ".tcl"
        } | ForEach-Object { $metadataFiles.Add($_) }
    }
}

$absolutePathMatches = @($metadataFiles | Select-String -Pattern 'C:/Projects|C:\\Projects|\.work[\\/]' -ErrorAction Stop)
if ($absolutePathMatches.Count -eq 0) {
    Add-CheckPass "IP-XACT and XGUI metadata contain no source-worktree absolute paths"
} else {
    Add-CheckFailure "IP-XACT/XGUI metadata contain $($absolutePathMatches.Count) absolute or .work path reference(s)"
    $absolutePathMatches | Select-Object -First 10 | ForEach-Object {
        Write-Host "       $($_.Path):$($_.LineNumber): $($_.Line.Trim())"
    }
}

$ownershipSearchRoots = @(
    (Join-Path $repoRoot "system_integration/common"),
    (Join-Path $repoRoot "system_integration/$($Profile.ToLower())")
)
$ownershipFiles = @($ownershipSearchRoots | Where-Object { Test-Path -LiteralPath $_ } |
    ForEach-Object { Get-ChildItem -LiteralPath $_ -Recurse -File } |
    Where-Object { $_.Extension -in @(".tcl", ".ps1", ".xml") })
$externalCsrMatches = @($ownershipFiles | Select-String -SimpleMatch "../ip_repo/src/csr" -ErrorAction Stop)
if ($externalCsrMatches.Count -eq 0) {
    Add-CheckPass "no synthesis manifest depends on the legacy external CSR source path"
} else {
    Add-CheckFailure "legacy external CSR source path is still referenced"
}

$trackedFiles = @(& git -C $repoRoot ls-files)
$trackedGenerated = @($trackedFiles | Where-Object {
    $_ -match '(^|/)(\.work|\.Xil|tmp|\.gen|\.srcs|xsim\.dir|2025\.2\.1|tclapp)(/|$)'
})
if ($trackedGenerated.Count -eq 0) {
    Add-CheckPass "generated work directories are not tracked by Git"
} else {
    Add-CheckFailure "Git tracks generated work paths: $($trackedGenerated -join ', ')"
}

$generatedRoots = @(".work", ".Xil", "tmp", ".gen", ".srcs", "xsim.dir", "2025.2.1", "tclapp")
$presentGenerated = @($generatedRoots | Where-Object { Test-Path -LiteralPath (Join-Path $repoRoot $_) })
if ($presentGenerated.Count -eq 0) {
    Add-CheckPass "workspace currently has no top-level generated work directory"
} else {
    Add-CheckWarning "disposable generated directories are present: $($presentGenerated -join ', ')"
}

$statusLines = @(& git -C $repoRoot status --short)
if ($statusLines.Count -eq 0) {
    Add-CheckPass "Git worktree is clean"
} else {
    Add-CheckWarning "Git worktree has $($statusLines.Count) uncommitted path(s)"
}

if ($RequireCopyIsolation) {
    $protectedRoot = Get-NormalizedPath "C:\Projects\my_sp\lib\IP\tdc_gpx_ctrl\HDL"
    if ($repoRoot -ieq $protectedRoot) {
        Add-CheckFailure "audit is running in the protected original workspace"
    } else {
        Add-CheckPass "workspace is separate from the protected original"
    }

    $originUrl = & git -C $repoRoot remote get-url origin 2>&1
    if ($LASTEXITCODE -eq 0 -and (($originUrl | Select-Object -First 1).Trim()) -match 'github\.com[/:]haneulia/tdc_gpx_ctrl(?:\.git)?$') {
        Add-CheckPass "origin points to the TDC-GPX GitHub repository"
    } else {
        Add-CheckFailure "origin is missing or unexpected: $originUrl"
    }

    $sourceFetch = & git -C $repoRoot remote get-url source-local 2>&1
    if ($LASTEXITCODE -ne 0) {
        Add-CheckFailure "source-local remote is missing"
    } else {
        $sourcePath = Get-NormalizedPath (($sourceFetch | Select-Object -First 1).Trim())
        if ($sourcePath -ieq $protectedRoot) {
            Add-CheckPass "source-local fetch points to the protected original"
        } else {
            Add-CheckFailure "source-local fetch path is unexpected: $sourcePath"
        }
    }

    $sourcePush = & git -C $repoRoot remote get-url --push source-local 2>&1
    if ($LASTEXITCODE -eq 0 -and (($sourcePush | Select-Object -First 1).Trim()) -eq "disabled://read-only") {
        Add-CheckPass "source-local push is disabled"
    } else {
        Add-CheckFailure "source-local push must be disabled://read-only"
    }

    $alternatesText = & git -C $repoRoot rev-parse --git-path objects/info/alternates 2>&1
    if ($LASTEXITCODE -ne 0) {
        Add-CheckFailure "cannot resolve the Git object alternates path"
        $alternatesPath = ""
    } else {
        $alternatesPath = (($alternatesText | Select-Object -First 1).Trim())
        if (-not [System.IO.Path]::IsPathRooted($alternatesPath)) {
            $alternatesPath = Join-Path $repoRoot $alternatesPath
        }
    }
    if ($alternatesPath -ne "" -and (Test-Path -LiteralPath $alternatesPath)) {
        Add-CheckFailure "Git object alternates are present; this is not a fully independent clone"
    } elseif ($alternatesPath -ne "") {
        Add-CheckPass "Git object database is independent"
    }
}

if ($DeepGitCheck) {
    & git -C $repoRoot fsck --full --no-dangling
    if ($LASTEXITCODE -eq 0) {
        Add-CheckPass "deep Git object check"
    } else {
        Add-CheckFailure "git fsck reported an error"
    }
}

Write-Host ""
Write-Host "Audit summary: failures=$($failures.Count), warnings=$($warnings.Count)"
if ($failures.Count -ne 0) {
    exit 1
}

Write-Host $passMarker -ForegroundColor Green
