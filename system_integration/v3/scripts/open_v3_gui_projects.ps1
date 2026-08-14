[CmdletBinding()]
param(
    [ValidateSet("All", "Vivado32", "Vivado64", "Hls")]
    [string]$Target = "All",
    [switch]$ValidateOnly,
    [switch]$RebuildMissingHls,
    [string]$VivadoRoot = "",
    [string]$VitisRoot = ""
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$V3Root = Split-Path -Parent $ScriptDir
$RepoRoot = (Resolve-Path (Join-Path $V3Root "..\..")).Path
$WorkRoot = Join-Path $RepoRoot ".work"
$GuiSessionRoot = Join-Path $WorkRoot "v3_gui_sessions"
$VivadoLauncher = Join-Path $ScriptDir "open_v3_vivado_signoff_gui.tcl"

function Assert-File {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path,
        [Parameter(Mandatory = $true)]
        [string]$Purpose
    )

    if (-not (Test-Path -LiteralPath $Path -PathType Leaf)) {
        throw "$Purpose is missing: $Path"
    }
}

function Resolve-ToolRoot {
    param(
        [string]$RequestedRoot,
        [string]$EnvironmentRoot,
        [string[]]$Candidates,
        [string]$ToolRelativePath,
        [string]$ToolName
    )

    $Roots = @()
    if (-not [string]::IsNullOrWhiteSpace($RequestedRoot)) {
        $Roots += $RequestedRoot
    }
    if (-not [string]::IsNullOrWhiteSpace($EnvironmentRoot)) {
        $Roots += $EnvironmentRoot
    }
    $Roots += $Candidates

    foreach ($Root in $Roots | Select-Object -Unique) {
        if (Test-Path -LiteralPath (Join-Path $Root $ToolRelativePath) -PathType Leaf) {
            return (Resolve-Path -LiteralPath $Root).Path
        }
    }

    throw "$ToolName installation was not found. Supply its install root explicitly."
}

$VivadoProfiles = @(
    [pscustomobject]@{
        Target = "Vivado32"
        Label = "W32"
        Xpr = Join-Path $WorkRoot "v3_parent_l0\w32\project_4_lidar_v3_l0.xpr"
        Dcp = Join-Path $WorkRoot "v3_parent_signoff\260813_200mhz_adaptive_reset_w32_impl1\post_route.dcp"
        Timing = Join-Path $WorkRoot "v3_parent_signoff\260813_200mhz_adaptive_reset_w32_impl1\post_route_timing_summary.rpt"
        Bit = Join-Path $WorkRoot "v3_parent_signoff\260813_200mhz_adaptive_reset_w32_impl1\design_1_lidar_ctrl_v3_wrapper.bit"
        Xsa = Join-Path $WorkRoot "v3_parent_signoff\260813_200mhz_adaptive_reset_w32_impl1\tdc_gpx_lidar_ctrl_v3_4chip.xsa"
    },
    [pscustomobject]@{
        Target = "Vivado64"
        Label = "W64"
        Xpr = Join-Path $WorkRoot "v3_parent_l0\w64\project_4_lidar_v3_l0.xpr"
        Dcp = Join-Path $WorkRoot "v3_parent_signoff\260813_200mhz_adaptive_reset_w64_impl1\post_route.dcp"
        Timing = Join-Path $WorkRoot "v3_parent_signoff\260813_200mhz_adaptive_reset_w64_impl1\post_route_timing_summary.rpt"
        Bit = Join-Path $WorkRoot "v3_parent_signoff\260813_200mhz_adaptive_reset_w64_impl1\design_1_lidar_ctrl_v3_wrapper.bit"
        Xsa = Join-Path $WorkRoot "v3_parent_signoff\260813_200mhz_adaptive_reset_w64_impl1\tdc_gpx_lidar_ctrl_v3_4chip.xsa"
    }
)

$HlsComponents = @(
    [pscustomobject]@{
        Name = "v3_hls_hit_decoder_component"
        Runner = "run_v3_hls_hit_decoder.ps1"
        Config = "hls\gpx_hit_decoder\hls_config.cfg"
    },
    [pscustomobject]@{
        Name = "v3_hls_cell_collector_component"
        Runner = "run_v3_hls_cell_collector.ps1"
        Config = "hls\gpx_cell_collector\hls_config.cfg"
    },
    [pscustomobject]@{
        Name = "v3_hls_frame_assembler_component"
        Runner = "run_v3_hls_frame_assembler.ps1"
        Config = "hls\gpx_frame_assembler\hls_config.cfg"
    },
    [pscustomobject]@{
        Name = "v3_hls_lane_word_formatter_component"
        Runner = "run_v3_hls_lane_word_formatter.ps1"
        Config = "hls\gpx_lane_word_formatter\hls_config.cfg"
    }
)

function Test-HlsComponentComplete {
    param([Parameter(Mandatory = $true)]$Component)

    $ComponentRoot = Join-Path $WorkRoot $Component.Name
    $Manifest = Join-Path $ComponentRoot "vitis-comp.json"
    $CompileSummary = Join-Path $ComponentRoot ($Component.Name + ".hlscompile_summary")
    $CsimSummary = Join-Path $ComponentRoot ($Component.Name + ".hlsrun_csim_summary")

    if (-not (Test-Path -LiteralPath $Manifest -PathType Leaf)) { return $false }
    if (-not (Test-Path -LiteralPath $CompileSummary -PathType Leaf)) { return $false }
    if (-not (Test-Path -LiteralPath $CsimSummary -PathType Leaf)) { return $false }

    $CosimReports = Get-ChildItem -LiteralPath $ComponentRoot -Recurse -File `
        -Filter "*_cosim.rpt" -ErrorAction SilentlyContinue
    return ($null -ne ($CosimReports | Select-Object -First 1))
}

function Assert-HlsComponentContract {
    param([Parameter(Mandatory = $true)]$Component)

    $ComponentRoot = Join-Path $WorkRoot $Component.Name
    $ManifestPath = Join-Path $ComponentRoot "vitis-comp.json"
    $ExpectedConfig = (Resolve-Path -LiteralPath (Join-Path $V3Root $Component.Config)).Path

    Assert-File -Path $ManifestPath -Purpose "$($Component.Name) Vitis component manifest"
    $Manifest = Get-Content -Raw -LiteralPath $ManifestPath | ConvertFrom-Json
    if ($Manifest.type -ne "HLS") {
        throw "$($Component.Name) is not an HLS component."
    }
    if ($Manifest.configuration.configFiles.Count -ne 1) {
        throw "$($Component.Name) must reference exactly one canonical HLS config."
    }

    $ActualConfig = [System.IO.Path]::GetFullPath(
        (Join-Path $ComponentRoot $Manifest.configuration.configFiles[0]))
    if ($ActualConfig -ne $ExpectedConfig) {
        throw "$($Component.Name) points to a non-canonical config: $ActualConfig"
    }

    Assert-File `
        -Path (Join-Path $ComponentRoot ($Component.Name + ".hlscompile_summary")) `
        -Purpose "$($Component.Name) C synthesis summary"
    Assert-File `
        -Path (Join-Path $ComponentRoot ($Component.Name + ".hlsrun_csim_summary")) `
        -Purpose "$($Component.Name) C simulation summary"

    $CosimReport = Get-ChildItem -LiteralPath $ComponentRoot -Recurse -File `
        -Filter "*_cosim.rpt" -ErrorAction SilentlyContinue |
        Sort-Object LastWriteTime -Descending |
        Select-Object -First 1
    if ($null -eq $CosimReport) {
        throw "$($Component.Name) C/RTL co-simulation report is missing."
    }
    if ((Get-Content -Raw -LiteralPath $CosimReport.FullName) -notmatch '\|\s*Verilog\s*\|\s*Pass\s*\|') {
        throw "$($Component.Name) latest C/RTL co-simulation report is not PASS."
    }

    Write-Host "LIDAR_V3_HLS_GUI_COMPONENT_PASS component=$($Component.Name)"
}

$NeedsVivado = $Target -in @("All", "Vivado32", "Vivado64")
$NeedsHls = $Target -in @("All", "Hls")

if ($NeedsVivado) {
    Assert-File -Path $VivadoLauncher -Purpose "Vivado sign-off GUI Tcl launcher"
    foreach ($Profile in $VivadoProfiles) {
        if ($Target -ne "All" -and $Target -ne $Profile.Target) { continue }
        Assert-File -Path $Profile.Xpr -Purpose "$($Profile.Label) editable Vivado project"
        Assert-File -Path $Profile.Dcp -Purpose "$($Profile.Label) routed sign-off checkpoint"
        Assert-File -Path $Profile.Timing -Purpose "$($Profile.Label) routed timing report"
        Assert-File -Path $Profile.Bit -Purpose "$($Profile.Label) bitstream"
        Assert-File -Path $Profile.Xsa -Purpose "$($Profile.Label) hardware platform"
        Write-Host "LIDAR_V3_VIVADO_GUI_PROFILE_PASS profile=$($Profile.Label)"
    }
}

if ($NeedsHls) {
    $MissingComponents = @($HlsComponents | Where-Object { -not (Test-HlsComponentComplete $_) })
    if ($MissingComponents.Count -gt 0 -and $RebuildMissingHls) {
        $ResolvedVitisRoot = Resolve-ToolRoot `
            -RequestedRoot $VitisRoot `
            -EnvironmentRoot $env:XILINX_VITIS `
            -Candidates @("C:\AMDDesignTools\2025.2.1\Vitis", "C:\AMDDesignTools\2025.2\Vitis") `
            -ToolRelativePath "bin\vitis.bat" `
            -ToolName "Vitis"

        foreach ($Component in $MissingComponents) {
            $Runner = Join-Path $ScriptDir $Component.Runner
            Assert-File -Path $Runner -Purpose "$($Component.Name) reproducible runner"
            & powershell.exe -NoProfile -ExecutionPolicy Bypass -File $Runner `
                -Step all -VitisRoot $ResolvedVitisRoot
            if ($LASTEXITCODE -ne 0) {
                throw "$($Component.Name) rebuild failed with exit code $LASTEXITCODE."
            }
        }
    } elseif ($MissingComponents.Count -gt 0) {
        $Names = ($MissingComponents.Name -join ", ")
        throw "Verified HLS components are missing: $Names. Re-run with -RebuildMissingHls."
    }

    foreach ($Component in $HlsComponents) {
        Assert-HlsComponentContract -Component $Component
    }
}

if ($ValidateOnly) {
    Write-Host "LIDAR_V3_GUI_PROJECTS_VALIDATE_PASS target=$Target"
    exit 0
}

if ($NeedsVivado) {
    $ResolvedVivadoRoot = Resolve-ToolRoot `
        -RequestedRoot $VivadoRoot `
        -EnvironmentRoot $env:XILINX_VIVADO `
        -Candidates @("C:\AMDDesignTools\2025.2.1\Vivado", "C:\AMDDesignTools\2025.2\Vivado") `
        -ToolRelativePath "bin\vivado.bat" `
        -ToolName "Vivado"
    $VivadoBat = Join-Path $ResolvedVivadoRoot "bin\vivado.bat"

    foreach ($Profile in $VivadoProfiles) {
        if ($Target -ne "All" -and $Target -ne $Profile.Target) { continue }
        $ProfileSessionRoot = Join-Path $GuiSessionRoot $Profile.Label
        New-Item -ItemType Directory -Force -Path $ProfileSessionRoot | Out-Null
        $Process = Start-Process `
            -FilePath $VivadoBat `
            -ArgumentList @(
                "-mode", "gui", "-nolog", "-nojournal",
                "-source", $VivadoLauncher,
                "-tclargs", $Profile.Label, $Profile.Xpr, $Profile.Dcp
            ) `
            -WorkingDirectory $ProfileSessionRoot `
            -PassThru
        Write-Host "LIDAR_V3_VIVADO_GUI_STARTED profile=$($Profile.Label) pid=$($Process.Id)"
    }
}

if ($NeedsHls) {
    $ResolvedVitisRoot = Resolve-ToolRoot `
        -RequestedRoot $VitisRoot `
        -EnvironmentRoot $env:XILINX_VITIS `
        -Candidates @("C:\AMDDesignTools\2025.2.1\Vitis", "C:\AMDDesignTools\2025.2\Vitis") `
        -ToolRelativePath "bin\vitis.bat" `
        -ToolName "Vitis"
    $VitisBat = Join-Path $ResolvedVitisRoot "bin\vitis.bat"
    $Process = Start-Process `
        -FilePath $VitisBat `
        -ArgumentList @("-w", $WorkRoot) `
        -WorkingDirectory $RepoRoot `
        -PassThru
    Write-Host "LIDAR_V3_VITIS_HLS_GUI_STARTED workspace=$WorkRoot pid=$($Process.Id)"
}

Write-Host "LIDAR_V3_GUI_PROJECTS_LAUNCH_PASS target=$Target"
