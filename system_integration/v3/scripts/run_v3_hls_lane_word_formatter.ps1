param(
    [ValidateSet("csim", "csynth", "cosim", "all")]
    [string]$Step = "all",
    [string]$VitisRoot = "C:\AMDDesignTools\2025.2\Vitis"
)

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$V3Root = Split-Path -Parent $ScriptDir
$RepoRoot = (Resolve-Path (Join-Path $V3Root "..\..")).Path
$Runner = Join-Path $VitisRoot "bin\vitis-run.bat"
$Compiler = Join-Path $VitisRoot "bin\v++.bat"
$ComponentRoot = Join-Path $V3Root "hls\gpx_lane_word_formatter"
$Config = Join-Path $ComponentRoot "hls_config.cfg"
$WorkDir = Join-Path $RepoRoot ".work\v3_hls_lane_word_formatter_component"
$ToolHome = Join-Path $RepoRoot ".work\v3_hls_lane_word_formatter_tool_home"
$Profiles = @(
    "return_sweep",
    "multi_cell",
    "holes_footer_32",
    "all_hole_footer_64",
    "reset_faults"
)

foreach ($File in @($Runner, $Compiler, $Config)) {
    if (-not (Test-Path -LiteralPath $File)) {
        throw "Required HLS file is missing: $File"
    }
}

function Invoke-HlsTool {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Executable,
        [Parameter(Mandatory = $true)]
        [string[]]$ToolArguments,
        [Parameter(Mandatory = $true)]
        [string]$Description
    )

    & $Executable @ToolArguments
    if ($LASTEXITCODE -ne 0) {
        throw "$Description failed with exit code $LASTEXITCODE"
    }
}

$CommonArguments = @(
    "--mode", "hls",
    "--config", $Config,
    "--work_dir", $WorkDir
)

$SavedEnvironment = @{
    HOME = $env:HOME
    USERPROFILE = $env:USERPROFILE
    APPDATA = $env:APPDATA
    LOCALAPPDATA = $env:LOCALAPPDATA
    V3_HLS_PROFILE = $env:V3_HLS_PROFILE
}

New-Item -ItemType Directory -Force -Path $ToolHome | Out-Null
$env:HOME = $ToolHome
$env:USERPROFILE = $ToolHome
$env:APPDATA = Join-Path $ToolHome "AppData\Roaming"
$env:LOCALAPPDATA = Join-Path $ToolHome "AppData\Local"
New-Item -ItemType Directory -Force -Path $env:APPDATA | Out-Null
New-Item -ItemType Directory -Force -Path $env:LOCALAPPDATA | Out-Null

Push-Location $ComponentRoot
try {
    if ($Step -eq "csim" -or $Step -eq "all") {
        foreach ($Profile in $Profiles) {
            $env:V3_HLS_PROFILE = $Profile
            Invoke-HlsTool `
                -Executable $Runner `
                -ToolArguments (@("--csim") + $CommonArguments) `
                -Description "V3 H4 C simulation ($Profile)"
            Write-Host (
                "LIDAR_V3_HLS_GPX_LANE_WORD_FORMATTER_CSIM_PROFILE_PASS " +
                "profile=$Profile")
        }
    }

    if ($Step -eq "csynth" -or $Step -eq "cosim" -or $Step -eq "all") {
        Invoke-HlsTool `
            -Executable $Compiler `
            -ToolArguments (@("-c") + $CommonArguments) `
            -Description "V3 H4 C synthesis"
    }

    if ($Step -eq "cosim" -or $Step -eq "all") {
        $ProfileReportDir = Join-Path $WorkDir "reports\profiles"
        New-Item -ItemType Directory -Force -Path $ProfileReportDir | Out-Null
        foreach ($Profile in $Profiles) {
            $env:V3_HLS_PROFILE = $Profile
            Invoke-HlsTool `
                -Executable $Runner `
                -ToolArguments (@("--cosim") + $CommonArguments) `
                -Description "V3 H4 C/RTL co-simulation ($Profile)"

            $CosimReport = Join-Path $WorkDir (
                "hls\sim\report\gpx_lane_word_formatter_hls_cosim.rpt")
            $ReportText = Get-Content -Raw -LiteralPath $CosimReport
            if ($ReportText -notmatch '\|\s*Verilog\|\s*Pass\|') {
                throw "V3 H4 co-simulation failed ($Profile)"
            }
            Copy-Item -Force -LiteralPath $CosimReport -Destination (
                Join-Path $ProfileReportDir (
                    "gpx_lane_word_formatter_hls_${Profile}_cosim.rpt"))
            Write-Host (
                "LIDAR_V3_HLS_GPX_LANE_WORD_FORMATTER_COSIM_PROFILE_PASS " +
                "profile=$Profile")
        }
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

$StepName = $Step.ToUpperInvariant()
Write-Host "LIDAR_V3_HLS_GPX_LANE_WORD_FORMATTER_${StepName}_PASS"
