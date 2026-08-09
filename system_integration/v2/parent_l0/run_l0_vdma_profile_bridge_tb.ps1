param(
    [string]$SessionTag = ''
)

$ErrorActionPreference = 'Stop'
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$HdlRoot = (Resolve-Path (Join-Path $ScriptDir '..\..\..')).Path
$VivadoBin = if ($env:XILINX_VIVADO) {
    Join-Path $env:XILINX_VIVADO 'bin'
} else {
    'C:\AMDDesignTools\2025.2.1\Vivado\bin'
}
if ([string]::IsNullOrWhiteSpace($SessionTag)) {
    $SessionTag = '{0}_l0_vdma_profile_bridge' -f (
        Get-Date -Format 'yyMMdd_HHmmss')
}
if ($SessionTag -notmatch '^[A-Za-z0-9_.-]+$') {
    throw "SessionTag contains unsupported characters: $SessionTag"
}

$WorkRoot = Join-Path $HdlRoot 'tmp\l0_vdma_profile_bridge'
$Work = Join-Path $WorkRoot $SessionTag
$Result = Join-Path $HdlRoot "signoff_results\sessions\$SessionTag"
if ((Test-Path -LiteralPath $Work) -or (Test-Path -LiteralPath $Result)) {
    throw "Session already exists: $SessionTag"
}
New-Item -ItemType Directory -Path $Work, $Result | Out-Null

function Invoke-Checked {
    param([string]$Exe, [string[]]$Arguments)
    & $Exe @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed ($LASTEXITCODE): $Exe"
    }
}

$Sources = @(
    (Join-Path $ScriptDir 'l0_vdma_profile_bridge.vhd'),
    (Join-Path $ScriptDir 'tb_l0_vdma_profile_bridge.vhd'))
$Glbl = Join-Path (Split-Path -Parent $VivadoBin) 'data\verilog\src\glbl.v'
if (-not (Test-Path -LiteralPath $Glbl)) {
    throw "Vivado simulation glbl.v is missing: $Glbl"
}
$Project = Join-Path $Work 'bridge.prj'
$Sources | ForEach-Object {
    "vhdl2008 xil_defaultlib `"$($_.Replace('\', '/'))`""
} | Set-Content -Encoding ASCII -LiteralPath $Project
$RunTcl = Join-Path $Work 'run.tcl'
@('run all', 'quit') | Set-Content -Encoding ASCII -LiteralPath $RunTcl
$CompileLog = Join-Path $Work 'xvhdl.log'
$ElabLog = Join-Path $Work 'xelab.log'
$SimLog = Join-Path $Work 'xsim.log'
$Snapshot = 'tb_l0_vdma_profile_bridge_snap'

Push-Location $Work
try {
    Invoke-Checked (Join-Path $VivadoBin 'xvlog.bat') @(
        '--relax', '--work', 'xil_defaultlib', $Glbl)
    Invoke-Checked (Join-Path $VivadoBin 'xvhdl.bat') @(
        '--relax', '-prj', $Project, '-log', $CompileLog)
    Invoke-Checked (Join-Path $VivadoBin 'xelab.bat') @(
        '--debug', 'off', '--relax', '--mt', '2',
        '--snapshot', $Snapshot,
        'xil_defaultlib.tb_l0_vdma_profile_bridge',
        'xil_defaultlib.glbl',
        '-log', $ElabLog)
    Invoke-Checked (Join-Path $VivadoBin 'xsim.bat') @(
        $Snapshot, '-tclbatch', $RunTcl.Replace('\', '/'),
        '-log', $SimLog.Replace('\', '/'))
} finally {
    Pop-Location
}

$SimText = Get-Content -Raw -LiteralPath $SimLog
if ($SimText -notmatch 'L0_VDMA_PROFILE_BRIDGE_150_TO_100_PASS') {
    throw "VDMA profile bridge did not report PASS: $SimLog"
}
Copy-Item -LiteralPath $CompileLog, $ElabLog, $SimLog, $Project, $RunTcl `
    -Destination $Result

$Commit = (& git -C $HdlRoot rev-parse HEAD).Trim()
@(
    "session=$SessionTag",
    'clocks=processing:150MHz,csr:100MHz',
    "hdl_commit=$Commit",
    'coverage=atomic payload,delayed ACK,stale-high ACK,back-to-back profile') |
    Set-Content -Encoding UTF8 -LiteralPath (
        Join-Path $Result 'session_manifest.txt')

$ResolvedWork = (Resolve-Path -LiteralPath $Work).Path
$ResolvedRoot = (Resolve-Path -LiteralPath $WorkRoot).Path
if (-not $ResolvedWork.StartsWith(
        $ResolvedRoot, [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "Refusing to remove work directory outside $ResolvedRoot"
}
Remove-Item -LiteralPath $ResolvedWork -Recurse -Force

Write-Output "L0_VDMA_PROFILE_BRIDGE_TEST_PASS results=$Result"
