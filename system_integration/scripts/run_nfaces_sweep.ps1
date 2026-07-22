param(
    [string] $Scenario = "",
    [string] $Stamp = (Get-Date -Format "yyMMddHHmmss")
)

$ErrorActionPreference = "Stop"

$Hdl = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$Smoke = Join-Path $PSScriptRoot "run_smoke.ps1"
if ([string]::IsNullOrWhiteSpace($Scenario)) {
    $Scenario = Join-Path $PSScriptRoot "..\scenarios\smoke_internal_200m_v001.json"
}
$Scenario = (Resolve-Path $Scenario).Path
$Base = Get-Content -Raw -LiteralPath $Scenario | ConvertFrom-Json
$InputDir = Join-Path $Hdl "tmp\system_integration\${Stamp}_nfaces_sweep_inputs"
New-Item -ItemType Directory -Force -Path $InputDir | Out-Null

$Results = @()
foreach ($Faces in 1..5) {
    $Cfg = $Base.PSObject.Copy()
    $Cfg.scenario_id = "nfaces_${Faces}_contract_v001"
    $Cfg.description = "Motor-owned static n_faces contract sweep: ${Faces} face(s)"
    $Cfg.faces_per_frame = $Faces

    $ScenarioPath = Join-Path $InputDir "nfaces_${Faces}.json"
    $Cfg | ConvertTo-Json -Depth 8 | Set-Content -Encoding UTF8 -LiteralPath $ScenarioPath

    $RunStamp = "${Stamp}_nfaces_${Faces}"
    Write-Host "[n_faces=$Faces] running canonical IP integration smoke"
    & $Smoke -Scenario $ScenarioPath -Stamp $RunStamp
    if ($LASTEXITCODE -ne 0) {
        throw "n_faces=$Faces integration smoke failed"
    }

    $ResultPath = Join-Path $Hdl `
        "sim_results\vivado_xsim\sessions\${RunStamp}_system_integration_smoke\rtl_result.json"
    $Result = Get-Content -Raw -LiteralPath $ResultPath | ConvertFrom-Json
    if ($Result.verdict -ne "PASS" -or
        [int]$Result.metrics.faces_per_frame -ne $Faces) {
        throw "n_faces=$Faces result contract mismatch: $ResultPath"
    }
    $Results += $ResultPath
}

Write-Host "N_FACES_1_TO_5_SWEEP_PASS"
$Results | ForEach-Object { Write-Host "Result: $_" }
