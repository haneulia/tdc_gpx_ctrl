param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss")
)

$ErrorActionPreference = "Stop"

$Hdl = (Resolve-Path (Join-Path $PSScriptRoot "../../..")).Path
$Vivado = "C:/AMDDesignTools/2025.2.1/Vivado/bin"
$Glbl = (Resolve-Path `
    "C:/AMDDesignTools/2025.2.1/Vivado/data/verilog/src/glbl.v").Path
$OrderFile = Join-Path $PSScriptRoot "v2_rtl_compile_order.txt"
$WorkRoot = Join-Path $Hdl "tmp/v2_top_shell"
$Work = Join-Path $WorkRoot $Stamp
$Archive = Join-Path $Hdl (
    "signoff_results/sessions/${Stamp}_v2_top_shell")

function Invoke-Checked {
    param([string]$Exe, [string[]]$ArgList)
    & $Exe @ArgList
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed ($LASTEXITCODE): $Exe $($ArgList -join ' ')"
    }
}

function Invoke-BatchChecked {
    param([string]$Exe, [string[]]$ArgList)

    # Vivado's Windows batch loader splits NAME=VALUE generic arguments unless
    # they remain quoted through cmd.exe. Preserve each argument as one token.
    $QuotedArgs = foreach ($Arg in $ArgList) {
        if ($Arg -match '[\s=]') {
            '"' + $Arg.Replace('"', '\"') + '"'
        }
        else {
            $Arg
        }
    }
    $CommandLine = '"' + $Exe + '" ' + ($QuotedArgs -join ' ')
    & $env:ComSpec /d /s /c $CommandLine
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed ($LASTEXITCODE): $CommandLine"
    }
}

New-Item -ItemType Directory -Force -Path $Work | Out-Null

$SourceFiles = foreach ($Line in Get-Content -LiteralPath $OrderFile) {
    $Entry = $Line.Trim()
    if ($Entry.Length -gt 0 -and -not $Entry.StartsWith("#")) {
        $Resolved = Resolve-Path -LiteralPath (Join-Path $Hdl $Entry)
        $Resolved.Path
    }
}

$Project = Join-Path $Work "v2_top_shell.prj"
$GlblProject = Join-Path $Work "glbl.prj"
$ProjectLines = foreach ($File in $SourceFiles) {
    "vhdl2008 xil_defaultlib `"$($File.Replace('\', '/'))`""
}
$ProjectLines += "nosort"
$ProjectLines | Set-Content -Encoding ASCII -LiteralPath $Project
@(
    "verilog xil_defaultlib `"$($Glbl.Replace('\', '/'))`"",
    "nosort"
) | Set-Content -Encoding ASCII -LiteralPath $GlblProject

$RunTcl = Join-Path $Work "run.tcl"
@("run 1 ns", "quit") | Set-Content -Encoding ASCII -LiteralPath $RunTcl

Push-Location $Work
try {
    Invoke-Checked "$Vivado/xvlog.bat" @(
        "--relax", "-prj", $GlblProject,
        "-log", (Join-Path $Work "xvlog_glbl.log"))
    Invoke-Checked "$Vivado/xvhdl.bat" @(
        "--2008", "--relax", "-prj", $Project,
        "-log", (Join-Path $Work "xvhdl.log"))

    $Profiles = @(
        [ordered]@{ name = "proc150_tdc200"; proc = 150; tdc = 200 },
        [ordered]@{ name = "proc200_tdc150"; proc = 200; tdc = 150 }
    )

    foreach ($Profile in $Profiles) {
        $Snapshot = "v2_top_shell_$($Profile.name)_${Stamp}_snap"
        $ElabLog = Join-Path $Work "xelab_$($Profile.name).log"
        $SimLog = Join-Path $Work "xsim_$($Profile.name).log"
        Invoke-BatchChecked "$Vivado/xelab.bat" @(
            "--debug", "off", "--relax", "--mt", "2",
            "--snapshot", $Snapshot,
            "--generic_top", "G_PROC_CLK_MHZ=$($Profile.proc)",
            "--generic_top", "G_TDC_CLK_MHZ=$($Profile.tdc)",
            "xil_defaultlib.tdc_gpx_lidar_ctrl_v2_top",
            "xil_defaultlib.glbl",
            "-log", $ElabLog)
        Invoke-Checked "$Vivado/xsim.bat" @(
            $Snapshot, "-tclbatch", $RunTcl.Replace('\', '/'),
            "-log", $SimLog.Replace('\', '/'))

        $Text = Get-Content -Raw -LiteralPath $SimLog
        $Marker = "LIDAR_V2_TOP_SHELL_PASS proc_mhz=$($Profile.proc) " +
            "tdc_mhz=$($Profile.tdc)"
        if ($Text -notmatch [regex]::Escape($Marker)) {
            throw "$($Profile.name) did not report the shell PASS marker"
        }
        if ($Text -match "Failure:|Fatal:") {
            throw "$($Profile.name) reported a failure"
        }
    }
}
finally {
    Pop-Location
}

New-Item -ItemType Directory -Force -Path $Archive | Out-Null
$Scenario = [ordered]@{
    checkpoint = "K0-2"
    purpose = "full production source compile order and fail-safe top shell"
    profiles = @("Processing 150 / TDC 200 MHz", "Processing 200 / TDC 150 MHz")
    release_ip = $false
    expected_output = "all physical commands and AXIS valids inactive"
}
$Scenario | ConvertTo-Json -Depth 3 | Set-Content -Encoding ASCII `
    -LiteralPath (Join-Path $Archive "scenario.json")

$Manifest = foreach ($File in $SourceFiles) {
    $Item = Get-Item -LiteralPath $File
    $Hash = Get-FileHash -LiteralPath $File -Algorithm SHA256
    [ordered]@{
        path = $Item.FullName.Replace('\', '/')
        bytes = $Item.Length
        sha256 = $Hash.Hash
    }
}
$Manifest | ConvertTo-Json -Depth 3 | Set-Content -Encoding ASCII `
    -LiteralPath (Join-Path $Archive "source_manifest.json")
Copy-Item -Force -LiteralPath $OrderFile -Destination $Archive

$Artifacts = Get-ChildItem -File -LiteralPath $Work | Where-Object {
    $_.Name -match '^(xvhdl|xelab|xsim).*\.log$|\.prj$|^run\.tcl$'
} | ForEach-Object { $_.FullName }
Copy-Item -Force -LiteralPath $Artifacts -Destination $Archive

$ResolvedWork = (Resolve-Path -LiteralPath $Work).Path
$ResolvedRoot = (Resolve-Path -LiteralPath $WorkRoot).Path
if (-not $ResolvedWork.StartsWith($ResolvedRoot,
        [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "Refusing to remove work directory outside $ResolvedRoot"
}
Remove-Item -LiteralPath $ResolvedWork -Recurse -Force

Write-Output "LIDAR_V2_TOP_SHELL_REGRESSION_PASS"
Write-Output "Result: $Archive"
