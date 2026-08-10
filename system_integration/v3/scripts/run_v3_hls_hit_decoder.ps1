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
$ComponentRoot = Join-Path $V3Root "hls\gpx_hit_decoder"
$Config = Join-Path $ComponentRoot "hls_config.cfg"
$WorkDir = Join-Path $RepoRoot ".work\v3_hls_hit_decoder_component"
$ToolHome = Join-Path $RepoRoot ".work\v3_hls_tool_home"

if (-not (Test-Path -LiteralPath $Runner)) {
    throw "Vitis HLS runner not found: $Runner"
}
if (-not (Test-Path -LiteralPath $Compiler)) {
    throw "Vitis HLS compiler not found: $Compiler"
}
if (-not (Test-Path -LiteralPath $Config)) {
    throw "HLS configuration not found: $Config"
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
        Remove-Item Env:V3_HLS_PROFILE -ErrorAction SilentlyContinue
        Invoke-HlsTool -Executable $Runner -ToolArguments (@("--csim") + $CommonArguments) -Description "V3 HLS Hit Decoder C simulation"
    }

    if ($Step -eq "csynth" -or $Step -eq "cosim" -or $Step -eq "all") {
        Invoke-HlsTool -Executable $Compiler -ToolArguments (@("-c") + $CommonArguments) -Description "V3 HLS Hit Decoder C synthesis"
    }

    if ($Step -eq "cosim" -or $Step -eq "all") {
        $ProfileReportDir = Join-Path $WorkDir "reports\profiles"
        New-Item -ItemType Directory -Force -Path $ProfileReportDir | Out-Null
        foreach ($Profile in @("dedicated", "one_chip_dual", "reduced", "all_dual")) {
            $env:V3_HLS_PROFILE = $Profile
            Invoke-HlsTool -Executable $Runner -ToolArguments (@("--cosim") + $CommonArguments) -Description "V3 HLS Hit Decoder C/RTL co-simulation ($Profile)"

            $CosimReport = Join-Path $WorkDir "hls\sim\report\gpx_hit_decoder_hls_cosim.rpt"
            $ReportText = Get-Content -Raw -LiteralPath $CosimReport
            if ($ReportText -notmatch '\|\s*Verilog\|\s*Pass\|') {
                throw "V3 HLS Hit Decoder co-simulation report did not pass ($Profile)"
            }
            Copy-Item -Force -LiteralPath $CosimReport -Destination (
                Join-Path $ProfileReportDir "gpx_hit_decoder_hls_${Profile}_cosim.rpt")
            Write-Host "LIDAR_V3_HLS_GPX_HIT_DECODER_COSIM_PROFILE_PASS profile=$Profile"
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
Write-Host "LIDAR_V3_HLS_GPX_HIT_DECODER_${StepName}_PASS"
