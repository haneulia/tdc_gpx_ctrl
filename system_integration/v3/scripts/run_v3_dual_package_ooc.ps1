[CmdletBinding()]
param(
    [ValidateSet("ALL", "EMBEDDED32", "EMBEDDED64", "HLS32", "HLS64")]
    [string]$Selector = "ALL",
    [string]$VivadoRoot = ""
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$V3Root = Split-Path -Parent $ScriptDir
$HdlRoot = (Resolve-Path (Join-Path $V3Root "..\..")).Path
$ControllerRoot = Split-Path -Parent $HdlRoot
$IpRepo = Join-Path $V3Root "ip_repo"
$CheckTcl = Join-Path $ScriptDir "check_v3_dual_package_ooc.tcl"

$Candidates = @()
if (-not [string]::IsNullOrWhiteSpace($VivadoRoot)) {
    $Candidates += $VivadoRoot
}
if (-not [string]::IsNullOrWhiteSpace($env:XILINX_VIVADO)) {
    $Candidates += $env:XILINX_VIVADO
}
$Candidates += @("C:\AMDDesignTools\2025.2.1\Vivado",
    "C:\AMDDesignTools\2025.2\Vivado")
$ResolvedVivadoRoot = $null
foreach ($Candidate in $Candidates | Select-Object -Unique) {
    if (Test-Path -LiteralPath (Join-Path $Candidate "bin\vivado.bat")) {
        $ResolvedVivadoRoot = (Resolve-Path -LiteralPath $Candidate).Path
        break
    }
}
if ($null -eq $ResolvedVivadoRoot) {
    throw "Vivado installation was not found."
}
if (-not (Test-Path -LiteralPath $CheckTcl -PathType Leaf)) {
    throw "V3 dual-package OOC checker is missing: $CheckTcl"
}

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
        throw "No free P: through U: drive is available for V3 OOC synthesis."
    }
    & $SubstExe $ShortDrive $ControllerRoot
    if ($LASTEXITCODE -ne 0) {
        throw "Failed to map $ShortDrive to $ControllerRoot"
    }
    $MapCreated = $true
}

$MappedHdlRoot = "$ShortDrive\HDL"
$MappedIpRepo = "$MappedHdlRoot\system_integration\v3\ip_repo"
$MappedCheckTcl = "$MappedHdlRoot\system_integration\v3\scripts\check_v3_dual_package_ooc.tcl"
$RunSelectors = if ($Selector -eq "ALL") {
    @("EMBEDDED32", "EMBEDDED64", "HLS32", "HLS64")
}
else {
    @($Selector)
}
$Vivado = Join-Path $ResolvedVivadoRoot "bin\vivado.bat"
$ToolHome = Join-Path ([System.IO.Path]::GetTempPath()) "v3vt"
$ToolAppData = Join-Path $ToolHome "R"
$ToolLocalAppData = Join-Path $ToolHome "L"
$ToolTclStore = Join-Path $ToolAppData `
    "Xilinx\Vivado\2025.2.1\XilinxTclStore"
$InstalledTclStore = Join-Path $ResolvedVivadoRoot "data\XilinxTclStore"
if (-not (Test-Path -LiteralPath (Join-Path $ToolTclStore ".done") `
        -PathType Leaf)) {
    New-Item -ItemType Directory -Force -Path `
        (Split-Path -Parent $ToolTclStore), $ToolLocalAppData | Out-Null
    Copy-Item -LiteralPath $InstalledTclStore -Destination $ToolTclStore `
        -Recurse -Force
}
Get-ChildItem -LiteralPath $ToolTclStore -Recurse -File -Force |
    ForEach-Object { $_.IsReadOnly = $false }

$SavedEnvironment = @{
    HOME = $env:HOME
    USERPROFILE = $env:USERPROFILE
    APPDATA = $env:APPDATA
    LOCALAPPDATA = $env:LOCALAPPDATA
}

try {
    $env:HOME = $ToolHome
    $env:USERPROFILE = $ToolHome
    $env:APPDATA = $ToolAppData
    $env:LOCALAPPDATA = $ToolLocalAppData
    $LogDir = Join-Path $HdlRoot ".work\v3_dual_package_ooc_logs"
    New-Item -ItemType Directory -Force -Path $LogDir | Out-Null
    foreach ($RunSelector in $RunSelectors) {
        $SelectorLower = $RunSelector.ToLowerInvariant()
        $MappedWorkDir = "$MappedHdlRoot\.work\v3_dual_package_ooc_$SelectorLower"
        $LogPath = Join-Path $LogDir "$SelectorLower.log"
        $Writer = [System.IO.StreamWriter]::new($LogPath, $false)
        try {
            & $Vivado -mode batch -notrace -nolog -nojournal `
                -source $MappedCheckTcl -tclargs `
                $MappedIpRepo $MappedWorkDir $RunSelector 2>&1 |
                ForEach-Object {
                    $Line = $_.ToString()
                    $Writer.WriteLine($Line)
                    if ($Line -match '^LIDAR_V3_|^TDC_GPX_LIDAR_CTRL_V3_|^synth_design completed successfully|^ERROR:|^CRITICAL WARNING:') {
                        Write-Output $Line
                    }
                }
            $ExitCode = $LASTEXITCODE
        }
        finally {
            $Writer.Dispose()
        }
        if ($ExitCode -ne 0) {
            Get-Content -LiteralPath $LogPath -Tail 80
            throw "V3 dual-package OOC synthesis failed ($ExitCode): $RunSelector. Log: $LogPath"
        }
        if (-not (Select-String -LiteralPath $LogPath -Quiet -SimpleMatch `
                "TDC_GPX_LIDAR_CTRL_V3_DUAL_PACKAGE_OOC_CHECK_PASS")) {
            throw "V3 dual-package OOC PASS marker is missing: $RunSelector. Log: $LogPath"
        }
    }
}
finally {
    foreach ($Name in $SavedEnvironment.Keys) {
        $Value = $SavedEnvironment[$Name]
        if ($null -eq $Value) {
            Remove-Item "Env:$Name" -ErrorAction SilentlyContinue
        }
        else {
            Set-Item "Env:$Name" $Value
        }
    }
    if ($MapCreated) {
        & $SubstExe $ShortDrive /D | Out-Null
    }
}

Write-Output "LIDAR_V3_DUAL_PACKAGE_OOC_RUNNER_PASS selector=$Selector repo=$IpRepo"
