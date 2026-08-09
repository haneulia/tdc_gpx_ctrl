param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss"),
    [switch]$IncludeReleaseProfiles
)

$ErrorActionPreference = "Stop"

$Hdl = (Resolve-Path (Join-Path $PSScriptRoot "../../..")).Path
$Vivado = "C:/AMDDesignTools/2025.2.1/Vivado/bin"
$Glbl = (Resolve-Path -LiteralPath (
    "C:/AMDDesignTools/2025.2.1/Vivado/data/verilog/src/glbl.v")).Path
$OrderFile = Join-Path $PSScriptRoot "v2_rtl_compile_order.txt"
$WorkRoot = Join-Path $Hdl "tmp/v2_k05_integration"
$Work = Join-Path $WorkRoot $Stamp
$Archive = Join-Path $Hdl (
    "signoff_results/sessions/${Stamp}_v2_k05_integration")

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

$ProductionSourceFiles = foreach ($Line in Get-Content -LiteralPath $OrderFile) {
    $Entry = $Line.Trim()
    if ($Entry.Length -gt 0 -and -not $Entry.StartsWith("#")) {
        (Resolve-Path -LiteralPath (Join-Path $Hdl $Entry)).Path
    }
}
$SourceFiles = @($ProductionSourceFiles)
$SourceFiles += @(
    (Resolve-Path -LiteralPath (Join-Path $Hdl `
        "system_integration/v2/pkg/lidar_config_reference_pkg.vhd")).Path,
    (Resolve-Path -LiteralPath (Join-Path $Hdl `
        "system_integration/v2/tb/tb_tdc_gpx_lidar_ctrl_v2_k05.vhd")).Path
)

$VhdlProject = Join-Path $Work "v2_k05_integration_vhdl.prj"
$VerilogProject = Join-Path $Work "v2_k05_integration_verilog.prj"
$ProjectLines = foreach ($File in $SourceFiles) {
    "vhdl2008 xil_defaultlib `"$($File.Replace('\', '/'))`""
}
$ProjectLines += "nosort"
$ProjectLines | Set-Content -Encoding ASCII -LiteralPath $VhdlProject
@(
    "verilog xil_defaultlib `"$($Glbl.Replace('\', '/'))`"",
    "nosort"
) | Set-Content -Encoding ASCII -LiteralPath $VerilogProject

$RunTcl = Join-Path $Work "run.tcl"
@("run all", "quit") | Set-Content -Encoding ASCII -LiteralPath $RunTcl

$Profiles = @(
    [ordered]@{
        name = "proc150_tdc200_async"; proc = 150; tdc = 200; mode = "ASYNC"
    },
    [ordered]@{
        name = "proc200_tdc150_async"; proc = 200; tdc = 150; mode = "ASYNC"
    }
)
if ($IncludeReleaseProfiles) {
    $Profiles += @(
        [ordered]@{
            name = "proc150_tdc150_sync"; proc = 150; tdc = 150; mode = "SYNC"
        },
        [ordered]@{
            name = "proc200_tdc50_async"; proc = 200; tdc = 50; mode = "ASYNC"
        },
        [ordered]@{
            name = "proc50_tdc200_async"; proc = 50; tdc = 200; mode = "ASYNC"
        },
        [ordered]@{
            name = "proc150_tdc100_async"; proc = 150; tdc = 100; mode = "ASYNC"
        }
    )
}

Push-Location $Work
try {
    Invoke-Checked "$Vivado/xvlog.bat" @(
        "--relax", "-prj", $VerilogProject,
        "-log", (Join-Path $Work "xvlog.log"))
    Invoke-Checked "$Vivado/xvhdl.bat" @(
        "--2008", "--relax", "-prj", $VhdlProject,
        "-log", (Join-Path $Work "xvhdl.log"))

    foreach ($Profile in $Profiles) {
        $Stem = "top_k05_$($Profile.name)"
        $Snapshot = "${Stem}_${Stamp}_snap"
        $ElabLog = Join-Path $Work "xelab_${Stem}.log"
        $SimLog = Join-Path $Work "xsim_${Stem}.log"
        Invoke-BatchChecked "$Vivado/xelab.bat" @(
            "--debug", "off", "--relax", "--mt", "2",
            "--snapshot", $Snapshot,
            "--generic_top", "G_PROC_CLK_MHZ=$($Profile.proc)",
            "--generic_top", "G_TDC_CLK_MHZ=$($Profile.tdc)",
            "--generic_top", "G_STREAM_CLK_MODE=$($Profile.mode)",
            "xil_defaultlib.tb_tdc_gpx_lidar_ctrl_v2_k05",
            "xil_defaultlib.glbl",
            "-log", $ElabLog)
        Invoke-Checked "$Vivado/xsim.bat" @(
            $Snapshot, "-tclbatch", $RunTcl.Replace('\', '/'),
            "-log", $SimLog.Replace('\', '/'))

        $Text = Get-Content -Raw -LiteralPath $SimLog
        $Marker = "LIDAR_V2_TOP_K05_GPX_B5_B8_PASS " +
            "proc_mhz=$($Profile.proc) tdc_mhz=$($Profile.tdc)"
        if ($Text -notmatch [regex]::Escape($Marker)) {
            throw "$Stem did not report its PASS marker"
        }
        $Reg7Marker = "LIDAR_V2_K12_REG7_SHADOW_ACTIVE_PHYSICAL_PASS " +
            "proc_mhz=$($Profile.proc) tdc_mhz=$($Profile.tdc)"
        if ($Text -notmatch [regex]::Escape($Reg7Marker)) {
            throw "$Stem did not close the K1-2 Reg7 contract"
        }
        if ($Text -match "Failure:|Fatal:") {
            throw "$Stem reported a failure"
        }
    }
}
finally {
    Pop-Location
}

New-Item -ItemType Directory -Force -Path $Archive | Out-Null
$Scenario = [ordered]@{
    checkpoint = "K0-5 / K1-4 release clock gate"
    purpose = "Top-level GPX configuration, IFIFO drain and B5-B8 closure"
    release_profiles_included = [bool]$IncludeReleaseProfiles
    profiles = @($Profiles | ForEach-Object {
        "Processing $($_.proc) / TDC $($_.tdc) MHz / $($_.mode)"
    })
    checks = @(
        "deferred GPX register-image activation",
        "Reg7 staging MTimer override, in-flight Shadow snapshot and active view",
        "failed MTimer-range COMMIT rollback and physical two-Chip Reg7 readback",
        "physical write/data/address pin mapping",
        "two Chips by eight STOPs by seven Returns IFIFO drain",
        "Processing RUN level CDC into TDC domain",
        "B5-B8 completion and Face-close reopening"
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

Write-Output "LIDAR_V2_K05_INTEGRATION_REGRESSION_PASS"
Write-Output "Result: $Archive"
