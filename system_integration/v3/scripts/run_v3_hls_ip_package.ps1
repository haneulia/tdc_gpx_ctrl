[CmdletBinding()]
param(
    [switch]$SkipHlsSynthesis,
    [switch]$RefreshHlsChildIp,
    [string]$VitisRoot = "",
    [string]$VivadoRoot = ""
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$V3Root = Split-Path -Parent $ScriptDir
$HdlRoot = (Resolve-Path (Join-Path $V3Root "..\..")).Path
$ControllerRoot = Split-Path -Parent $HdlRoot
$IpRepo = Join-Path $V3Root "ip_repo"
$StageRoot = Join-Path $HdlRoot ".work\v3hip"
$ToolHome = Join-Path ([System.IO.Path]::GetTempPath()) `
    "tdc_gpx_v3_hls_ip_package_tool_home"
$VivadoToolHome = Join-Path ([System.IO.Path]::GetTempPath()) "v3vip"
$PackageTcl = Join-Path $ScriptDir "package_v3_ip.tcl"
$CheckTcl = Join-Path $ScriptDir "check_v3_hls_ip_package.tcl"
$ParentPackage = Join-Path $IpRepo "tdc_gpx_lidar_ctrl_v3_hls_ip_3_0"

function Resolve-ToolRoot {
    param(
        [string]$Requested,
        [string]$EnvironmentValue,
        [string[]]$Defaults,
        [string]$RelativeExecutable,
        [string]$ToolName
    )
    $Candidates = @()
    if (-not [string]::IsNullOrWhiteSpace($Requested)) {
        $Candidates += $Requested
    }
    if (-not [string]::IsNullOrWhiteSpace($EnvironmentValue)) {
        $Candidates += $EnvironmentValue
    }
    $Candidates += $Defaults
    foreach ($Candidate in $Candidates | Select-Object -Unique) {
        if (Test-Path -LiteralPath (Join-Path $Candidate $RelativeExecutable)) {
            return (Resolve-Path -LiteralPath $Candidate).Path
        }
    }
    throw "$ToolName installation was not found."
}

function Invoke-Checked {
    param(
        [string]$Executable,
        [string[]]$ToolArguments,
        [string]$PassMarker = ""
    )
    $Output = & $Executable @ToolArguments 2>&1
    $Output | ForEach-Object { Write-Output $_ }
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed ($LASTEXITCODE): $Executable"
    }
    if (-not [string]::IsNullOrWhiteSpace($PassMarker) -and
            -not ($Output -match [regex]::Escape($PassMarker))) {
        throw "Required PASS marker is missing: $PassMarker"
    }
}

$ResolvedVivadoRoot = Resolve-ToolRoot -Requested $VivadoRoot `
    -EnvironmentValue $env:XILINX_VIVADO `
    -Defaults @("C:\AMDDesignTools\2025.2.1\Vivado",
        "C:\AMDDesignTools\2025.2\Vivado") `
    -RelativeExecutable "bin\vivado.bat" -ToolName "Vivado"
$Vivado = Join-Path $ResolvedVivadoRoot "bin\vivado.bat"
$VivadoVersion = Split-Path -Leaf (Split-Path -Parent $ResolvedVivadoRoot)
$VivadoAppData = Join-Path $VivadoToolHome "R"
$VivadoLocalAppData = Join-Path $VivadoToolHome "L"
$VivadoTclStore = Join-Path $VivadoAppData `
    "Xilinx\Vivado\$VivadoVersion\XilinxTclStore"
$InstalledTclStore = Join-Path $ResolvedVivadoRoot "data\XilinxTclStore"

# HLS-generated LUTRAM filenames are long enough to exceed the legacy Windows
# 260-character path when expanded below the repository. Use one short drive
# for export, extraction, installation, and Vivado parent packaging. Map the
# controller root, not HDL itself: the canonical compile order intentionally
# reaches the sibling ../ip_repo CSR source tree.
$SubstExe = "C:\Windows\System32\subst.exe"
$ExistingMaps = @{}
foreach ($Line in @(& $SubstExe)) {
    if ($Line -match '^([A-Z]:)\\: => (.+)$') {
        $ExistingMaps[$Matches[1]] = [System.IO.Path]::GetFullPath(
            $Matches[2].Trim())
    }
}
$ShortDrive = @($ExistingMaps.Keys) |
    Where-Object { $ExistingMaps[$_] -eq $ControllerRoot } |
    Select-Object -First 1
$MapCreated = $false
if ($null -eq $ShortDrive) {
    $ShortDrive = @("P:", "Q:", "R:", "S:", "T:", "U:") |
        Where-Object { -not $ExistingMaps.ContainsKey($_) -and
            -not (Test-Path -LiteralPath "$_\") } |
        Select-Object -First 1
    if ($null -eq $ShortDrive) {
        throw "No free P: through U: drive is available for HLS IP export."
    }
    & $SubstExe $ShortDrive $ControllerRoot
    if ($LASTEXITCODE -ne 0) {
        throw "Failed to map $ShortDrive to $HdlRoot"
    }
    $MapCreated = $true
}
$MappedHdlRoot = "$ShortDrive\HDL"
$MappedV3Root = "$MappedHdlRoot\system_integration\v3"
$MappedIpRepo = "$MappedV3Root\ip_repo"
$MappedStageRoot = "$MappedHdlRoot\.work\v3hip"
$MappedParentPackage = "$MappedIpRepo\tdc_gpx_lidar_ctrl_v3_hls_ip_3_0"

$Components = @(
    [pscustomobject]@{
        Top = "gpx_hit_decoder_hls"
        Source = "gpx_hit_decoder"
        Work = "v3_hls_hit_decoder_component"
        Runner = "run_v3_hls_hit_decoder.ps1"
        Display = "GPX Hit Decoder HLS"
    },
    [pscustomobject]@{
        Top = "gpx_cell_collector_hls"
        Source = "gpx_cell_collector"
        Work = "v3_hls_cell_collector_component"
        Runner = "run_v3_hls_cell_collector.ps1"
        Display = "GPX Cell Collector HLS"
    },
    [pscustomobject]@{
        Top = "gpx_frame_assembler_hls"
        Source = "gpx_frame_assembler"
        Work = "v3_hls_frame_assembler_component"
        Runner = "run_v3_hls_frame_assembler.ps1"
        Display = "GPX Frame Assembler HLS"
    },
    [pscustomobject]@{
        Top = "gpx_lane_word_formatter_hls"
        Source = "gpx_lane_word_formatter"
        Work = "v3_hls_lane_word_formatter_component"
        Runner = "run_v3_hls_lane_word_formatter.ps1"
        Display = "GPX Lane Word Formatter HLS"
    })

foreach ($Required in @($PackageTcl, $CheckTcl)) {
    if (-not (Test-Path -LiteralPath $Required -PathType Leaf)) {
        throw "Required V3 HLS-IP package input is missing: $Required"
    }
}

function Test-HlsChildIpPackage {
    param([pscustomobject]$Component)

    $PackageRoot = Join-Path $IpRepo "$($Component.Top)_3_0"
    $ComponentXml = Join-Path $PackageRoot "component.xml"
    if (-not (Test-Path -LiteralPath $ComponentXml -PathType Leaf)) {
        return $false
    }
    $Text = Get-Content -Raw -LiteralPath $ComponentXml
    foreach ($Token in @("<spirit:vendor>victek.co.kr</spirit:vendor>",
            "<spirit:library>hls_ip</spirit:library>",
            "<spirit:name>$($Component.Top)</spirit:name>",
            "<spirit:version>3.0</spirit:version>")) {
        if (-not $Text.Contains($Token)) {
            return $false
        }
    }
    return $true
}

$NeedChildExport = [bool]$RefreshHlsChildIp
foreach ($Component in $Components) {
    if (-not (Test-HlsChildIpPackage -Component $Component)) {
        $NeedChildExport = $true
    }
}

$SavedEnvironment = @{
    HOME = $env:HOME
    USERPROFILE = $env:USERPROFILE
    APPDATA = $env:APPDATA
    LOCALAPPDATA = $env:LOCALAPPDATA
}
$EnvironmentOverridden = $false
function Restore-ToolEnvironment {
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

function Set-VivadoToolEnvironment {
    $StoreMarker = Join-Path $VivadoTclStore ".done"
    if (-not (Test-Path -LiteralPath $StoreMarker -PathType Leaf)) {
        New-Item -ItemType Directory -Force -Path `
            (Split-Path -Parent $VivadoTclStore), `
            $VivadoLocalAppData | Out-Null
        Copy-Item -LiteralPath $InstalledTclStore `
            -Destination $VivadoTclStore -Recurse -Force
    }
    Get-ChildItem -LiteralPath $VivadoTclStore -Recurse -File -Force |
        ForEach-Object { $_.IsReadOnly = $false }
    $env:HOME = $VivadoToolHome
    $env:USERPROFILE = $VivadoToolHome
    $env:APPDATA = $VivadoAppData
    $env:LOCALAPPDATA = $VivadoLocalAppData
    $script:EnvironmentOverridden = $true
}
try {
    if ($NeedChildExport) {
        # Use a real local path, not the substituted drive, for Vitis export.
        # Detached helper processes can outlive one command and must not depend
        # on a drive mapping that is later removed.
        New-Item -ItemType Directory -Force -Path $ToolHome | Out-Null
        $env:HOME = $ToolHome
        $env:USERPROFILE = $ToolHome
        $env:APPDATA = Join-Path $ToolHome "AppData\Roaming"
        $env:LOCALAPPDATA = Join-Path $ToolHome "AppData\Local"
        New-Item -ItemType Directory -Force -Path `
            $env:APPDATA, $env:LOCALAPPDATA | Out-Null
        $EnvironmentOverridden = $true

        $ResolvedVitisRoot = Resolve-ToolRoot -Requested $VitisRoot `
            -EnvironmentValue $env:XILINX_VITIS `
            -Defaults @("C:\AMDDesignTools\2025.2.1\Vitis",
                "C:\AMDDesignTools\2025.2\Vitis") `
            -RelativeExecutable "bin\vitis-run.bat" -ToolName "Vitis"
        $VitisRun = Join-Path $ResolvedVitisRoot "bin\vitis-run.bat"

        if (-not $SkipHlsSynthesis) {
            foreach ($Component in $Components) {
                Invoke-Checked -Executable "powershell.exe" -ToolArguments @(
                    "-NoProfile", "-ExecutionPolicy", "Bypass", "-File",
                    (Join-Path $ScriptDir $Component.Runner), "-Step", "csynth")
            }
        }

        $AllowedStageParent = [System.IO.Path]::GetFullPath(
            (Join-Path $HdlRoot ".work"))
        $ResolvedStage = [System.IO.Path]::GetFullPath($StageRoot)
        if (-not $ResolvedStage.StartsWith(
                "$AllowedStageParent\",
                [System.StringComparison]::OrdinalIgnoreCase)) {
            throw "Refusing to replace staging directory outside .work: $StageRoot"
        }
        if (Test-Path -LiteralPath $MappedStageRoot) {
            Remove-Item -LiteralPath $MappedStageRoot -Recurse -Force
        }
        New-Item -ItemType Directory -Force -Path $MappedStageRoot | Out-Null

        foreach ($Component in $Components) {
            $SourceRoot = Join-Path $MappedV3Root "hls\$($Component.Source)"
            $Config = Join-Path $SourceRoot "hls_config.cfg"
            $WorkDir = Join-Path $MappedHdlRoot ".work\$($Component.Work)"
            $GeneratedTop = Join-Path $WorkDir `
                "hls\syn\verilog\$($Component.Top).v"
            if (-not (Test-Path -LiteralPath $GeneratedTop -PathType Leaf)) {
                throw "HLS synthesis output is missing: $GeneratedTop"
            }

            $Zip = Join-Path $MappedStageRoot "$($Component.Top).zip"
            $Expanded = Join-Path $MappedStageRoot "$($Component.Top)_3_0"
            Push-Location $SourceRoot
            try {
                Invoke-Checked -Executable $VitisRun -ToolArguments @(
                    "--package", "--mode", "hls", "--config", $Config,
                    "--work_dir", $WorkDir,
                    "--hls.package.output.format=ip_catalog",
                    "--hls.package.output.file=$Zip",
                    "--hls.package.ip.vendor=victek.co.kr",
                    "--hls.package.ip.library=hls_ip",
                    "--hls.package.ip.name=$($Component.Top)",
                    "--hls.package.ip.version=3.0",
                    "--hls.package.ip.display_name=$($Component.Display)")
            }
            finally {
                Pop-Location
            }
            if (-not (Test-Path -LiteralPath $Zip -PathType Leaf)) {
                throw "HLS IP archive was not generated: $Zip"
            }
            Expand-Archive -LiteralPath $Zip -DestinationPath $Expanded -Force
            $ComponentXml = Join-Path $Expanded "component.xml"
            if (-not (Test-Path -LiteralPath $ComponentXml -PathType Leaf)) {
                throw "Exported HLS IP has no component.xml: $Expanded"
            }
            $Text = Get-Content -Raw -LiteralPath $ComponentXml
            $ExpectedVlnv = "victek.co.kr:hls_ip:$($Component.Top):3.0"
            foreach ($Token in @("<spirit:vendor>victek.co.kr</spirit:vendor>",
                    "<spirit:library>hls_ip</spirit:library>",
                    "<spirit:name>$($Component.Top)</spirit:name>",
                    "<spirit:version>3.0</spirit:version>")) {
                if (-not $Text.Contains($Token)) {
                    throw "Exported HLS IP metadata mismatch ($ExpectedVlnv): $Token"
                }
            }
            Write-Output "LIDAR_V3_HLS_CHILD_IP_EXPORT_PASS vlnv=$ExpectedVlnv"
        }

        $ResolvedRepo = [System.IO.Path]::GetFullPath($MappedIpRepo)
        foreach ($Component in $Components) {
            $Destination = Join-Path $MappedIpRepo "$($Component.Top)_3_0"
            $ResolvedDestination = [System.IO.Path]::GetFullPath($Destination)
            if (-not $ResolvedDestination.StartsWith(
                    "$ResolvedRepo\",
                    [System.StringComparison]::OrdinalIgnoreCase)) {
                throw "Refusing to replace HLS IP outside V3 ip_repo: $Destination"
            }
            if (Test-Path -LiteralPath $Destination) {
                Remove-Item -LiteralPath $Destination -Recurse -Force
            }
            Copy-Item -LiteralPath (Join-Path $MappedStageRoot `
                "$($Component.Top)_3_0") -Destination $Destination -Recurse
        }
        Restore-ToolEnvironment
        $EnvironmentOverridden = $false
    }
    else {
        foreach ($Component in $Components) {
            Write-Output ("LIDAR_V3_HLS_CHILD_IP_REUSE_PASS vlnv=" +
                "victek.co.kr:hls_ip:$($Component.Top):3.0")
        }
    }

    # Package and check against a writable copy of the installed Tcl Store.
    # This keeps batch validation independent from a stale per-user catalog and
    # avoids writing tool state into the source tree.
    Set-VivadoToolEnvironment

    Invoke-Checked -Executable $Vivado -ToolArguments @(
        "-mode", "batch", "-notrace", "-nolog", "-nojournal",
        "-source", $PackageTcl, "-tclargs", $MappedParentPackage, "HLS_IP") `
        -PassMarker "TDC_GPX_LIDAR_CTRL_V3_IP_PACKAGE_PASS"

    Invoke-Checked -Executable $Vivado -ToolArguments @(
        "-mode", "batch", "-notrace", "-nolog", "-nojournal",
        "-source", $CheckTcl, "-tclargs", $MappedIpRepo) `
        -PassMarker "TDC_GPX_LIDAR_CTRL_V3_HLS_IP_CHECK_PASS"
}
finally {
    if ($EnvironmentOverridden) {
        Restore-ToolEnvironment
    }
    if ($MapCreated) {
        & $SubstExe $ShortDrive /D | Out-Null
    }
}

Write-Output "LIDAR_V3_HLS_IP_PACKAGE_RUNNER_PASS repo=$IpRepo"
