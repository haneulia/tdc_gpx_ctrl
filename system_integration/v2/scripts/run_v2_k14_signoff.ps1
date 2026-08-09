param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss"),
    [switch]$Resume
)

$ErrorActionPreference = "Stop"

if ($Stamp -notmatch '^[A-Za-z0-9][A-Za-z0-9_.-]*$') {
    throw "Stamp may contain only letters, digits, dot, underscore and hyphen"
}

$Hdl = (Resolve-Path (Join-Path $PSScriptRoot "../../..")).Path
$WorkRoot = Join-Path $Hdl "tmp/v2_k14_signoff"
$Work = Join-Path $WorkRoot $Stamp
$Archive = Join-Path $Hdl (
    "signoff_results/sessions/${Stamp}_v2_k14_signoff")
$OrderFile = Join-Path $PSScriptRoot "v2_rtl_compile_order.txt"

function Get-FixedArchive {
    param([string]$ChildStamp, [string]$Suffix)
    return Join-Path $Hdl (
        "signoff_results/sessions/${ChildStamp}_${Suffix}")
}

function Get-PackageArchiveFromLog {
    param([string]$Text)

    $Match = [regex]::Match($Text,
        'LIDAR_V2_K010_IP_PACKAGE_SIGNOFF_PASS session=([^\r\n]+)')
    if (-not $Match.Success) {
        return ""
    }
    return $Match.Groups[1].Value.Trim()
}

function Get-EvidenceHash {
    param([string]$EvidenceDirectory)

    if ([string]::IsNullOrWhiteSpace($EvidenceDirectory)) {
        return ""
    }
    foreach ($Name in @("scenario.json", "SUMMARY.txt")) {
        $Candidate = Join-Path $EvidenceDirectory $Name
        if (Test-Path -LiteralPath $Candidate) {
            return (Get-FileHash -LiteralPath $Candidate `
                -Algorithm SHA256).Hash
        }
    }
    return ""
}

function Get-InputFiles {
    $Files = [System.Collections.Generic.List[string]]::new()
    foreach ($Line in Get-Content -LiteralPath $OrderFile) {
        $Entry = $Line.Trim()
        if ($Entry.Length -gt 0 -and -not $Entry.StartsWith("#")) {
            $Files.Add((Resolve-Path -LiteralPath (
                Join-Path $Hdl $Entry)).Path)
        }
    }
    $Files.Add($PSCommandPath)
    $Files.Add($OrderFile)
    # Resume is valid only when every human-maintained input consumed by the
    # child gates is unchanged. Generated ip_repo/component.xml is excluded:
    # package_v2_ip.tcl recreates it and changes timestamps/checksums each run.
    foreach ($Directory in @(
        (Join-Path $Hdl "system_integration/v2/scripts"),
        (Join-Path $Hdl "system_integration/v2/tb"),
        (Join-Path $Hdl "system_integration/v2/golden"),
        (Join-Path $Hdl "system_integration/v2/sw_reference"),
        (Join-Path $Hdl "system_integration/v2/ip_package"))) {
        foreach ($File in Get-ChildItem -File -Recurse -LiteralPath $Directory) {
            $Files.Add($File.FullName)
        }
    }
    foreach ($Relative in @(
        "tb_tdc_gpx_request_loss.vhd",
        "tb_tdc_gpx_chip_ctrl.vhd",
        "system_integration/v2_architecture/V2_RTL_MAINTENANCE_GUIDE_KO.md",
        "system_integration/v2_architecture/V2_TESTBENCH_COVERAGE_GUIDE_KO.md",
        "Doc/cluster_analysis/C08_HDL_HTML_Alignment/C08_HDL_HTML_Alignment_260806_PACKED17_VDMA_ABI_Simulator_v026.html",
        "Doc/cluster_analysis/C08_HDL_HTML_Alignment/C08_HDL_HTML_Alignment_260808_V2_Operating_Matrix_Simulator_v027.html")) {
        $Files.Add((Resolve-Path -LiteralPath (
            Join-Path $Hdl $Relative)).Path)
    }

    return @($Files | Sort-Object -Unique)
}

function Get-InputSnapshotHash {
    $Lines = foreach ($File in Get-InputFiles) {
        $Normalized = $File.Replace('\', '/')
        $Hash = (Get-FileHash -LiteralPath $File -Algorithm SHA256).Hash
        "$Normalized=$Hash"
    }
    $Bytes = [System.Text.Encoding]::UTF8.GetBytes($Lines -join "`n")
    $Hasher = [System.Security.Cryptography.SHA256]::Create()
    try {
        return ([System.BitConverter]::ToString(
            $Hasher.ComputeHash($Bytes))).Replace("-", "")
    }
    finally {
        $Hasher.Dispose()
    }
}

New-Item -ItemType Directory -Force -Path $Work | Out-Null

$InputSnapshot = Get-InputSnapshotHash
$InputSnapshotPath = Join-Path $Work "input_snapshot.sha256"
if ([bool]$Resume) {
    if (-not (Test-Path -LiteralPath $InputSnapshotPath)) {
        throw "Cannot safely resume: input snapshot is missing for $Stamp"
    }
    $RecordedSnapshot = (Get-Content -Raw -LiteralPath $InputSnapshotPath).Trim()
    if ($RecordedSnapshot -ne $InputSnapshot) {
        throw "Cannot safely resume: a Sign-off input changed; use a new Stamp"
    }
}
$InputSnapshot | Set-Content -Encoding ASCII -LiteralPath $InputSnapshotPath

$MaintenanceStamp = "${Stamp}_01_gpx_maintenance"
$TopStamp = "${Stamp}_02_top_release"
$ImplStamp = "${Stamp}_03_top_implementation"
$CoordinatorStamp = "${Stamp}_04_gpx_coordinator"
$CdcStamp = "${Stamp}_05_cdc"
$AxisStamp = "${Stamp}_06_axis"
$StatusStamp = "${Stamp}_07_status_irq"
$MatrixStamp = "${Stamp}_08_operating_matrix"
$PsStamp = "${Stamp}_09_ps_hline"

$Steps = @(
    [pscustomobject][ordered]@{
        name = "testbench_docs"
        script = Join-Path $PSScriptRoot "check_v2_testbench_docs.ps1"
        arguments = @()
        marker = "LIDAR_V2_TESTBENCH_DOC_COVERAGE_PASS"
        archive = ""
        package_archive = $false
    },
    [pscustomobject][ordered]@{
        name = "gpx_register_maintenance"
        script = Join-Path $PSScriptRoot "run_v2_gpx_clear_status.ps1"
        arguments = @("-Stamp", $MaintenanceStamp)
        marker = "LIDAR_V2_K11_GPX_CLEAR_STATUS_REGRESSION_PASS"
        archive = Get-FixedArchive $MaintenanceStamp "v2_gpx_clear_status"
        package_archive = $false
    },
    [pscustomobject][ordered]@{
        name = "release_top_functional"
        script = Join-Path $PSScriptRoot "run_v2_k05_integration.ps1"
        arguments = @("-Stamp", $TopStamp, "-IncludeReleaseProfiles")
        marker = "LIDAR_V2_K05_INTEGRATION_REGRESSION_PASS"
        archive = Get-FixedArchive $TopStamp "v2_k05_integration"
        package_archive = $false
    },
    [pscustomobject][ordered]@{
        name = "routine_top_implementation"
        script = Join-Path $PSScriptRoot "run_v2_k06_top_implementation.ps1"
        arguments = @("-Stamp", $ImplStamp)
        marker = "LIDAR_V2_K06_TOP_IMPLEMENTATION_PASS"
        archive = Get-FixedArchive $ImplStamp "v2_k06_top_implementation"
        package_archive = $false
    },
    [pscustomobject][ordered]@{
        name = "gpx_acquisition_coordinator"
        script = Join-Path $PSScriptRoot "run_v2_gpx_acquisition_coordinator.ps1"
        arguments = @("-Stamp", $CoordinatorStamp)
        marker = "LIDAR_V2_GPX_ACQUISITION_COORDINATOR_PASS"
        archive = Get-FixedArchive $CoordinatorStamp "v2_gpx_acquisition_coordinator"
        package_archive = $false
    },
    [pscustomobject][ordered]@{
        name = "release_cdc"
        script = Join-Path $PSScriptRoot "run_v2_gpx_event_gateway.ps1"
        arguments = @("-Stamp", $CdcStamp)
        marker = "LIDAR_V2_GPX_EVENT_GATEWAY_PASS"
        archive = Get-FixedArchive $CdcStamp "v2_gpx_event_gateway"
        package_archive = $false
    },
    [pscustomobject][ordered]@{
        name = "routine_axis_widths"
        script = Join-Path $PSScriptRoot "run_v2_k06_axis_integration.ps1"
        arguments = @("-Stamp", $AxisStamp)
        marker = "LIDAR_V2_K06_AXIS_INTEGRATION_REGRESSION_PASS"
        archive = Get-FixedArchive $AxisStamp "v2_k06_axis_integration"
        package_archive = $false
    },
    [pscustomobject][ordered]@{
        name = "routine_status_irq"
        script = Join-Path $PSScriptRoot "run_v2_k08_status_irq.ps1"
        arguments = @("-Stamp", $StatusStamp)
        marker = "LIDAR_V2_K08_STATUS_IRQ_REGRESSION_PASS"
        archive = Get-FixedArchive $StatusStamp "v2_k08_status_irq"
        package_archive = $false
    },
    [pscustomobject][ordered]@{
        name = "rtl_html_operating_matrix"
        script = Join-Path $PSScriptRoot "run_v2_k13_operating_matrix.ps1"
        arguments = @("-Stamp", $MatrixStamp)
        marker = "LIDAR_V2_K13_OPERATING_MATRIX_PASS"
        archive = Get-FixedArchive $MatrixStamp "v2_k13_operating_matrix"
        package_archive = $false
    },
    [pscustomobject][ordered]@{
        name = "ddr_html_ps_ethernet"
        script = Join-Path $PSScriptRoot "run_v2_gpx_ps_hline.ps1"
        arguments = @("-Stamp", $PsStamp)
        marker = "LIDAR_V2_GPX_PS_HLINE_SIGNOFF_PASS"
        archive = Get-FixedArchive $PsStamp "v2_gpx_ps_hline"
        package_archive = $false
    },
    [pscustomobject][ordered]@{
        name = "ip_package_xgui_ooc"
        script = Join-Path $PSScriptRoot "run_v2_k010_ip_package.ps1"
        arguments = @()
        marker = "LIDAR_V2_K010_IP_PACKAGE_SIGNOFF_PASS"
        archive = ""
        package_archive = $true
    }
)

$Results = [System.Collections.Generic.List[object]]::new()
$RunnerLines = [System.Collections.Generic.List[string]]::new()
$RunnerLines.Add("K1-4 Sign-off stamp=$Stamp resume=$([bool]$Resume)")
$RunnerLines.Add("input_snapshot_sha256=$InputSnapshot")

foreach ($Step in $Steps) {
    $Log = Join-Path $Work "$($Step.name).log"
    $Text = if (Test-Path -LiteralPath $Log) {
        Get-Content -Raw -LiteralPath $Log
    }
    else {
        ""
    }
    $EvidenceDirectory = [string]$Step.archive
    if ($Step.package_archive -and -not [string]::IsNullOrWhiteSpace($Text)) {
        $EvidenceDirectory = Get-PackageArchiveFromLog $Text
    }

    $CanResume = [bool]$Resume -and
        $Text.Contains([string]$Step.marker) -and
        ([string]::IsNullOrWhiteSpace($EvidenceDirectory) -or
            (Test-Path -LiteralPath $EvidenceDirectory))

    $Started = Get-Date
    $Disposition = "executed"
    if ($CanResume) {
        $Disposition = "resumed"
        Write-Host "[K1-4] reuse $($Step.name)"
    }
    else {
        Write-Host "[K1-4] run $($Step.name)"
        $InvokeArguments = @(
            "-NoProfile", "-ExecutionPolicy", "Bypass",
            "-File", [string]$Step.script) + @($Step.arguments)
        # Native stderr is part of the child evidence. Do not let PowerShell
        # convert it into a terminating parent error before the exit code and
        # PASS marker can be checked and the complete log can be preserved.
        $SavedErrorActionPreference = $ErrorActionPreference
        $ErrorActionPreference = "Continue"
        & powershell.exe @InvokeArguments 2>&1 |
            Tee-Object -FilePath $Log
        $ExitCode = $LASTEXITCODE
        $ErrorActionPreference = $SavedErrorActionPreference
        if ($ExitCode -ne 0) {
            throw "$($Step.name) failed with exit code $ExitCode; see $Log"
        }
        $Text = Get-Content -Raw -LiteralPath $Log
        if (-not $Text.Contains([string]$Step.marker)) {
            throw "$($Step.name) did not report marker '$($Step.marker)'"
        }
        if ($Step.package_archive) {
            $EvidenceDirectory = Get-PackageArchiveFromLog $Text
        }
    }

    if (-not [string]::IsNullOrWhiteSpace($EvidenceDirectory) -and
            -not (Test-Path -LiteralPath $EvidenceDirectory)) {
        throw "$($Step.name) evidence directory is missing: $EvidenceDirectory"
    }

    $Duration = [math]::Round(((Get-Date) - $Started).TotalSeconds, 3)
    $LogHash = (Get-FileHash -LiteralPath $Log -Algorithm SHA256).Hash
    $EvidenceHash = Get-EvidenceHash $EvidenceDirectory
    $Results.Add([pscustomobject][ordered]@{
        order = $Results.Count + 1
        step = $Step.name
        disposition = $Disposition
        marker = $Step.marker
        duration_seconds = $Duration
        evidence_directory = $EvidenceDirectory
        evidence_anchor_sha256 = $EvidenceHash
        invocation_log_sha256 = $LogHash
    })
    $RunnerLines.Add(
        "$($Step.name) PASS disposition=$Disposition duration_s=$Duration " +
        "evidence=$EvidenceDirectory log_sha256=$LogHash")
}

New-Item -ItemType Directory -Force -Path $Archive | Out-Null
$Results | ConvertTo-Json -Depth 4 |
    Set-Content -Encoding ASCII -LiteralPath (
        Join-Path $Archive "step_summary.json")
$Results | Export-Csv -NoTypeInformation -Encoding ASCII -LiteralPath (
    Join-Path $Archive "step_summary.csv")
$RunnerLines | Set-Content -Encoding ASCII -LiteralPath (
    Join-Path $Archive "runner.log")
Copy-Item -Force -LiteralPath $InputSnapshotPath -Destination $Archive

$Scenario = [ordered]@{
    checkpoint = "K1-4 final integrated release sign-off"
    routine_clock_profiles = @(
        "Processing 150 / TDC 200 MHz ASYNC",
        "Processing 200 / TDC 150 MHz ASYNC")
    release_clock_profiles = @(
        "shared physical 150 MHz SYNC",
        "Processing 200 / TDC 50 MHz ASYNC",
        "Processing 50 / TDC 200 MHz ASYNC",
        "Processing 150 / TDC 100 MHz ASYNC")
    output_widths_bits = @(32, 64, 128)
    closed_layers = @(
        "GPX register maintenance, timeout and CLEAR_STATUS recovery",
        "four-Chip acquisition arbitration and buffered register service",
        "functional Top and status/IRQ integration",
        "CDC structure and post-route timing",
        "RTL versus executable HTML operating matrix",
        "DDR Word and PS/Ethernet Byte Golden comparison",
        "IP-XACT source, XGUI and packaged OOC profiles")
    excluded_board_layers = @(
        "physical AXI VDMA and HP-port sustained traffic",
        "FreeRTOS or PetaLinux DMA cache maintenance",
        "PCB GPX reference clock, bus timing and LVDS integrity",
        "physical laser safety and Ethernet sustained throughput")
}
$Scenario | ConvertTo-Json -Depth 5 |
    Set-Content -Encoding ASCII -LiteralPath (
        Join-Path $Archive "scenario.json")

$Manifest = foreach ($File in Get-InputFiles) {
    $Item = Get-Item -LiteralPath $File
    [ordered]@{
        path = $Item.FullName.Replace('\', '/')
        bytes = $Item.Length
        sha256 = (Get-FileHash -LiteralPath $File `
            -Algorithm SHA256).Hash
    }
}
$Manifest | ConvertTo-Json -Depth 3 |
    Set-Content -Encoding ASCII -LiteralPath (
        Join-Path $Archive "source_manifest.json")

$ResolvedWork = (Resolve-Path -LiteralPath $Work).Path
$ResolvedRoot = (Resolve-Path -LiteralPath $WorkRoot).Path
if (-not $ResolvedWork.StartsWith($ResolvedRoot,
        [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "Refusing to remove work directory outside $ResolvedRoot"
}
Remove-Item -LiteralPath $ResolvedWork -Recurse -Force

Write-Output "LIDAR_V2_K14_SIGNOFF_PASS"
Write-Output "Result: $Archive"
