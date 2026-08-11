param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss"),
    [string]$VivadoRoot = "C:\AMDDesignTools\2025.2.1\Vivado"
)

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$V3Root = Split-Path -Parent $ScriptDir
$RepoRoot = (Resolve-Path (Join-Path $V3Root "..\..")).Path
$Work = Join-Path $RepoRoot ".work\v3_shared_stream_boundary_diff\$Stamp"
$ToolHome = Join-Path $RepoRoot ".work\v3_shared_boundary_tool_home"
$VivadoBin = Join-Path $VivadoRoot "bin"

function Invoke-Checked {
    param(
        [Parameter(Mandatory = $true)][string]$Executable,
        [Parameter(Mandatory = $true)][string[]]$ToolArguments
    )
    & $Executable @ToolArguments
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed ($LASTEXITCODE): $Executable"
    }
}

$VhdlFiles = @(
    (Join-Path $RepoRoot "tdc_gpx_skid_buffer.vhd"),
    (Join-Path $RepoRoot "tdc_gpx_sync_fifo.vhd"),
    (Join-Path $RepoRoot "tb_tdc_gpx_stale_ready.vhd")
)
foreach ($File in $VhdlFiles) {
    if (-not (Test-Path -LiteralPath $File)) {
        throw "Required VHDL source is missing: $File"
    }
}

New-Item -ItemType Directory -Force -Path $Work | Out-Null
New-Item -ItemType Directory -Force -Path $ToolHome | Out-Null
$RunTcl = Join-Path $Work "run.tcl"
@("run all", "quit") | Set-Content -Encoding ASCII -LiteralPath $RunTcl

$SavedEnvironment = @{
    HOME = $env:HOME
    USERPROFILE = $env:USERPROFILE
    APPDATA = $env:APPDATA
    LOCALAPPDATA = $env:LOCALAPPDATA
}
$env:HOME = $ToolHome
$env:USERPROFILE = $ToolHome
$env:APPDATA = Join-Path $ToolHome "AppData\Roaming"
$env:LOCALAPPDATA = Join-Path $ToolHome "AppData\Local"
New-Item -ItemType Directory -Force -Path $env:APPDATA | Out-Null
New-Item -ItemType Directory -Force -Path $env:LOCALAPPDATA | Out-Null

Push-Location $Work
try {
    Invoke-Checked -Executable (Join-Path $VivadoBin "xvhdl.bat") `
        -ToolArguments (@("--2008", "--relax") + $VhdlFiles + @(
            "-log", (Join-Path $Work "xvhdl.log")))
    Invoke-Checked -Executable (Join-Path $VivadoBin "xelab.bat") `
        -ToolArguments @(
            "--debug", "off", "--relax", "--snapshot",
            "v3_shared_boundary_${Stamp}_snap",
            "work.tb_tdc_gpx_stale_ready",
            "-log", (Join-Path $Work "xelab.log")
        )
    Invoke-Checked -Executable (Join-Path $VivadoBin "xsim.bat") `
        -ToolArguments @(
            "v3_shared_boundary_${Stamp}_snap",
            "-tclbatch", $RunTcl.Replace('\', '/'),
            "-log", (Join-Path $Work "xsim.log")
        )
    $SimulationText = Get-Content -Raw -LiteralPath (Join-Path $Work "xsim.log")
    if ($SimulationText -notmatch
            "tb_tdc_gpx_stale_ready: ALL TESTS PASSED") {
        throw "Shared stream boundary PASS marker is missing"
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

Write-Host "LIDAR_V3_SHARED_STREAM_BOUNDARY_DIFF_PASS"
Write-Host "Result: $Work"
