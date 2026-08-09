param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss")
)

$ErrorActionPreference = "Stop"

$Hdl = (Resolve-Path (Join-Path $PSScriptRoot "../../..")).Path
$Vivado = "C:/AMDDesignTools/2025.2.1/Vivado/bin"
$Glbl = (Resolve-Path -LiteralPath (
    "C:/AMDDesignTools/2025.2.1/Vivado/data/verilog/src/glbl.v")).Path
$OrderFile = Join-Path $PSScriptRoot "v2_rtl_compile_order.txt"
$WorkRoot = Join-Path $Hdl "tmp/v2_k03_integration"
$Work = Join-Path $WorkRoot $Stamp
$Archive = Join-Path $Hdl (
    "signoff_results/sessions/${Stamp}_v2_k03_integration")

function Invoke-Checked {
    param([string]$Exe, [string[]]$ArgList)
    & $Exe @ArgList
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed ($LASTEXITCODE): $Exe $($ArgList -join ' ')"
    }
}

function Invoke-BatchChecked {
    param([string]$Exe, [string[]]$ArgList)
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
        (Resolve-Path -LiteralPath (Join-Path $Hdl $Entry)).Path
    }
}
$SourceFiles += @(
    (Resolve-Path -LiteralPath (Join-Path $Hdl `
        "system_integration/v2/pkg/lidar_config_reference_pkg.vhd")).Path,
    (Resolve-Path -LiteralPath (Join-Path $Hdl `
        "system_integration/v2/tb/tb_lidar_system_command_cdc.vhd")).Path,
    (Resolve-Path -LiteralPath (Join-Path $Hdl `
        "system_integration/v2/tb/tb_lidar_vdma_profile_cdc.vhd")).Path,
    (Resolve-Path -LiteralPath (Join-Path $Hdl `
        "system_integration/v2/tb/tb_tdc_gpx_lidar_ctrl_v2_k03.vhd")).Path
)

$Project = Join-Path $Work "v2_k03_integration.prj"
$VerilogProject = Join-Path $Work "v2_k03_integration_verilog.prj"
$ProjectLines = foreach ($File in $SourceFiles) {
    "vhdl2008 xil_defaultlib `"$($File.Replace('\', '/'))`""
}
$ProjectLines += "nosort"
$ProjectLines | Set-Content -Encoding ASCII -LiteralPath $Project
@(
    "verilog xil_defaultlib `"$($Glbl.Replace('\', '/'))`"",
    "nosort"
) | Set-Content -Encoding ASCII -LiteralPath $VerilogProject

$RunTcl = Join-Path $Work "run.tcl"
@("run all", "quit") | Set-Content -Encoding ASCII -LiteralPath $RunTcl

$Profiles = @(
    [ordered]@{ name = "proc150_tdc200"; proc = 150; tdc = 200 },
    [ordered]@{ name = "proc200_tdc150"; proc = 200; tdc = 150 }
)
$Tests = @(
    [ordered]@{
        name = "system_command_cdc"
        unit = "tb_lidar_system_command_cdc"
        marker = "LIDAR_V2_SYSTEM_COMMAND_CDC_PASS"
    },
    [ordered]@{
        name = "vdma_profile_cdc"
        unit = "tb_lidar_vdma_profile_cdc"
        marker = "LIDAR_V2_VDMA_PROFILE_CDC_PASS"
    },
    [ordered]@{
        name = "top_k03"
        unit = "tb_tdc_gpx_lidar_ctrl_v2_k03"
        marker = "LIDAR_V2_TOP_K03_INTEGRATION_PASS"
    }
)

Push-Location $Work
try {
    Invoke-Checked "$Vivado/xvlog.bat" @(
        "--relax", "-prj", $VerilogProject,
        "-log", (Join-Path $Work "xvlog.log"))
    Invoke-Checked "$Vivado/xvhdl.bat" @(
        "--2008", "--relax", "-prj", $Project,
        "-log", (Join-Path $Work "xvhdl.log"))

    foreach ($Profile in $Profiles) {
        foreach ($Test in $Tests) {
            $Stem = "$($Test.name)_$($Profile.name)"
            $Snapshot = "${Stem}_${Stamp}_snap"
            $ElabLog = Join-Path $Work "xelab_${Stem}.log"
            $SimLog = Join-Path $Work "xsim_${Stem}.log"
            Invoke-BatchChecked "$Vivado/xelab.bat" @(
                "--debug", "off", "--relax", "--mt", "2",
                "--snapshot", $Snapshot,
                "--generic_top", "G_PROC_CLK_MHZ=$($Profile.proc)",
                "--generic_top", "G_TDC_CLK_MHZ=$($Profile.tdc)",
                "xil_defaultlib.$($Test.unit)",
                "xil_defaultlib.glbl",
                "-log", $ElabLog)
            Invoke-Checked "$Vivado/xsim.bat" @(
                $Snapshot, "-tclbatch", $RunTcl.Replace('\', '/'),
                "-log", $SimLog.Replace('\', '/'))

            $Text = Get-Content -Raw -LiteralPath $SimLog
            $Marker = "$($Test.marker) proc_mhz=$($Profile.proc) " +
                "tdc_mhz=$($Profile.tdc)"
            if ($Text -notmatch [regex]::Escape($Marker)) {
                throw "$Stem did not report its PASS marker"
            }
            if ($Text -match "Failure:|Fatal:") {
                throw "$Stem reported a failure"
            }
        }
    }
}
finally {
    Pop-Location
}

New-Item -ItemType Directory -Force -Path $Archive | Out-Null
$Scenario = [ordered]@{
    checkpoint = "K0-3"
    purpose = "atomic Rise/Fall VDMA activation and acknowledged system commands"
    profiles = @("Processing 150 / TDC 200 MHz", "Processing 200 / TDC 150 MHz")
    checks = @(
        "CLEAR_STATUS and SOFT_RESET exactly once per destination",
        "busy and ambiguous command rejection",
        "Rise/Fall VDMA ACK barrier before Active version release",
        "Return 7 to 1 changes HSIZE while fixed maximum STRIDE is preserved",
        "commit-success IRQ exact flag and W1C clear"
    )
}
$Scenario | ConvertTo-Json -Depth 4 | Set-Content -Encoding ASCII `
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
    $_.Name -match '^(xvlog|xvhdl|xelab|xsim).*\.log$|\.prj$|^run\.tcl$'
} | ForEach-Object { $_.FullName }
Copy-Item -Force -LiteralPath $Artifacts -Destination $Archive

$ResolvedWork = (Resolve-Path -LiteralPath $Work).Path
$ResolvedRoot = (Resolve-Path -LiteralPath $WorkRoot).Path
if (-not $ResolvedWork.StartsWith($ResolvedRoot,
        [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "Refusing to remove work directory outside $ResolvedRoot"
}
Remove-Item -LiteralPath $ResolvedWork -Recurse -Force

Write-Output "LIDAR_V2_K03_INTEGRATION_REGRESSION_PASS"
Write-Output "Result: $Archive"
