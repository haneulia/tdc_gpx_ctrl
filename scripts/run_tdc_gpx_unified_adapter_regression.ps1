param(
    [string]$VivadoBin = 'C:/AMDDesignTools/2025.2.1/Vivado/bin',
    [string]$Work = (Join-Path $env:TEMP (
        'tdc_gpx_unified_adapter_' + (Get-Date -Format 'yyMMddHHmmss')))
)

$ErrorActionPreference = 'Stop'
$Hdl = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$Glbl = (Resolve-Path (Join-Path $VivadoBin '../data/verilog/src/glbl.v')).Path
$files = @(
    "$Hdl/tdc_gpx_pkg.vhd",
    "$Hdl/tdc_gpx_cfg_pkg.vhd",
    "$Hdl/tdc_gpx_unified_cdc_snapshot.vhd",
    "$Hdl/tdc_gpx_unified_csr_adapter.vhd",
    "$Hdl/tb_tdc_gpx_unified_csr_adapter.vhd"
)

foreach ($file in $files) {
    if (-not (Test-Path -LiteralPath $file -PathType Leaf)) {
        throw "Missing source: $file"
    }
}

New-Item -ItemType Directory -Force -Path $Work | Out-Null
$Work = (Resolve-Path -LiteralPath $Work).Path
$prj = Join-Path $Work 'tdc_gpx_unified_adapter.prj'
@($files | ForEach-Object { "vhdl2008 xil_defaultlib `"$_`"" }) + 'nosort' |
    Set-Content -LiteralPath $prj -Encoding ASCII

$runTcl = Join-Path $Work 'run_all.tcl'
@('run -all', 'quit') | Set-Content -LiteralPath $runTcl -Encoding ASCII
$runTclXsim = $runTcl.Replace('\', '/')

Push-Location $Work
try {
    & "$VivadoBin/xvlog.bat" --work xil_defaultlib --relax $Glbl `
        -log (Join-Path $Work 'xvlog.log')
    if ($LASTEXITCODE -ne 0) { throw 'xvlog failed' }

    & "$VivadoBin/xvhdl.bat" --relax -prj $prj `
        -log (Join-Path $Work 'xvhdl.log')
    if ($LASTEXITCODE -ne 0) { throw 'xvhdl failed' }

    $top = 'tb_tdc_gpx_unified_csr_adapter'
    $snap = "${top}_snap"
    & "$VivadoBin/xelab.bat" --debug off --relax --mt 2 `
        -L xil_defaultlib -L unisims_ver -L unimacro_ver -L secureip -L xpm `
        --snapshot $snap "xil_defaultlib.$top" xil_defaultlib.glbl `
        -log (Join-Path $Work 'xelab.log')
    if ($LASTEXITCODE -ne 0) { throw 'xelab failed' }

    $simLog = Join-Path $Work 'xsim.log'
    & "$VivadoBin/xsim.bat" $snap -tclbatch $runTclXsim -log $simLog
    if ($LASTEXITCODE -ne 0) { throw 'xsim failed' }

    $body = Get-Content -Raw -LiteralPath $simLog
    if ($body -notmatch 'TDC_GPX_UNIFIED_CSR_ADAPTER_PASS') {
        throw 'TDC-GPX unified adapter PASS marker is missing'
    }
    if ($body -match '(?i)severity failure|assertion failed') {
        throw 'TDC-GPX unified adapter emitted an assertion failure'
    }

    Write-Host 'TDC_GPX_UNIFIED_ADAPTER_REGRESSION_PASS'
    Write-Host "Logs: $Work"
}
finally {
    Pop-Location
}
