param(
    [Parameter(Mandatory = $true)]
    [string]$ResultDir,
    [Parameter(Mandatory = $true)]
    [ValidateSet('SYNTH', 'IMPL')]
    [string]$Stage
)

$ErrorActionPreference = 'Stop'
$Stage = $Stage.ToUpperInvariant()
$Prefix = if ($Stage -eq 'IMPL') { 'post_route' } else { 'post_synth' }
$Report = [System.Collections.Generic.List[string]]::new()
$MinimumSetupSlackNs = 0.100

function Assert-True {
    param(
        [Parameter(Mandatory = $true)][bool]$Condition,
        [Parameter(Mandatory = $true)][string]$Message
    )

    if (-not $Condition) {
        throw "Parent reference sign-off failed: $Message"
    }
}

function Read-Report {
    param([Parameter(Mandatory = $true)][string]$Name)

    $path = Join-Path $ResultDir $Name
    Assert-True (Test-Path -LiteralPath $path) "missing report $path"
    return Get-Content -Raw -LiteralPath $path
}

function Get-RuleSummary {
    param([Parameter(Mandatory = $true)][string]$Text)

    $pattern = '(?m)^\|\s*(?<id>[A-Z]+-\d+)\s*\|\s*' +
        '(?<severity>Critical Warning|Warning|Advisory|Info)\s*\|' +
        '.*?\|\s*(?<count>\d+)\s*\|\s*$'
    return @([regex]::Matches($Text, $pattern) | ForEach-Object {
        [pscustomobject]@{
            Id = $_.Groups['id'].Value
            Severity = $_.Groups['severity'].Value
            Count = [int]$_.Groups['count'].Value
        }
    })
}

function Assert-AllowedRules {
    param(
        [Parameter(Mandatory = $true)]$Rows,
        [Parameter(Mandatory = $true)][string[]]$Allowed,
        [Parameter(Mandatory = $true)][string]$Name
    )

    $unexpected = @($Rows | Where-Object { $_.Id -notin $Allowed })
    Assert-True ($unexpected.Count -eq 0) (
        "$Name has unreviewed rules: " +
        (($unexpected | ForEach-Object { "$($_.Id)[$($_.Severity)]=$($_.Count)" }) -join ', ')
    )
    $Report.Add("PASS $Name.rules=" + (($Rows | ForEach-Object {
        "$($_.Id)[$($_.Severity)]=$($_.Count)"
    }) -join ','))
}

Assert-True (Test-Path -LiteralPath $ResultDir) "missing result directory $ResultDir"

$contractPath = Join-Path $ResultDir 'parent_ref_contract_verified.txt'
Assert-True (Test-Path -LiteralPath $contractPath) "missing parent contract report"
$contractLines = @(Get-Content -LiteralPath $contractPath | Where-Object { $_.Trim() })
Assert-True ($contractLines.Count -eq 38) "expected 38 contract checks, got $($contractLines.Count)"
Assert-True ((@($contractLines | Where-Object { $_ -notmatch '^PASS ' })).Count -eq 0) `
    'parent contract report contains a non-PASS line'
$Report.Add('PASS contract_checks=38')

$timing = Read-Report "${Prefix}_timing_summary.rpt"
Assert-True ($timing -match 'All user specified timing constraints are met\.') `
    "$Prefix timing constraints are not met"
Assert-True ($timing -match 'unconstrained_internal_endpoints \(0\)') `
    "$Prefix has unconstrained internal endpoints"
Assert-True ($timing -match 'There are 0 pins that are not constrained for maximum delay\.') `
    "$Prefix has pins without a maximum-delay constraint"

$timingLine = @($timing -split "`r?`n" | Where-Object {
    $_ -match '^\s*-?\d+\.\d{3}\s+-?\d+\.\d{3}\s+\d+\s+\d+\s+-?\d+\.\d{3}'
} | Select-Object -First 1)
Assert-True ($timingLine.Count -eq 1) "could not parse $Prefix design timing summary"
$timingFields = @($timingLine[0].Trim() -split '\s+')
Assert-True ($timingFields.Count -eq 12) `
    "$Prefix design timing summary has $($timingFields.Count) fields, expected 12"
$wns = [double]$timingFields[0]
$tns = [double]$timingFields[1]
$setupFailing = [int]$timingFields[2]
$whs = [double]$timingFields[4]
$ths = [double]$timingFields[5]
$holdFailing = [int]$timingFields[6]
$wpws = [double]$timingFields[8]
$tpws = [double]$timingFields[9]
$pulseFailing = [int]$timingFields[10]
Assert-True ($wns -ge $MinimumSetupSlackNs -and $tns -eq 0.0 -and $setupFailing -eq 0) `
    "$Prefix setup timing failed: WNS=$wns TNS=$tns endpoints=$setupFailing"
Assert-True ($whs -ge 0.0 -and $ths -eq 0.0 -and $holdFailing -eq 0) `
    "$Prefix hold timing failed: WHS=$whs THS=$ths endpoints=$holdFailing"
Assert-True ($wpws -ge 0.0 -and $tpws -eq 0.0 -and $pulseFailing -eq 0) `
    "$Prefix pulse-width timing failed: WPWS=$wpws TPWS=$tpws endpoints=$pulseFailing"
$Report.Add("PASS timing.design_wns_ns=$wns")
$Report.Add("PASS timing.minimum_setup_slack_ns=$MinimumSetupSlackNs")
$Report.Add("PASS timing.design_whs_ns=$whs")
$Report.Add("PASS timing.design_wpws_ns=$wpws")
$Report.Add('PASS timing.unconstrained_internal_endpoints=0')

$clockRows = @{}
foreach ($line in ($timing -split "`r?`n")) {
    if ($line -match '^\s*(clk_fpga_[012])\s+(-?\d+\.\d{3})\s+(-?\d+\.\d{3})\s+(\d+)') {
        $clockRows[$Matches[1]] = [pscustomobject]@{
            Wns = [double]$Matches[2]
            Tns = [double]$Matches[3]
            Failing = [int]$Matches[4]
        }
    }
}
foreach ($clock in @('clk_fpga_0', 'clk_fpga_1', 'clk_fpga_2')) {
    Assert-True $clockRows.ContainsKey($clock) "missing timing row for $clock"
    $row = $clockRows[$clock]
    Assert-True ($row.Wns -ge $MinimumSetupSlackNs -and $row.Tns -eq 0.0 -and $row.Failing -eq 0) `
        "$clock timing failed: WNS=$($row.Wns) TNS=$($row.Tns) endpoints=$($row.Failing)"
    $Report.Add("PASS timing.$clock.wns_ns=$($row.Wns)")
}

$interaction = Read-Report "${Prefix}_clock_interaction.rpt"
Assert-True ($interaction -notmatch '(?i)unsafe') "$Prefix clock interaction contains an unsafe path"
foreach ($pair in @(
    @('clk_fpga_0', 'clk_fpga_1'),
    @('clk_fpga_1', 'clk_fpga_0'),
    @('clk_fpga_1', 'clk_fpga_2'),
    @('clk_fpga_2', 'clk_fpga_1')
)) {
    $pattern = '(?m)^' + $pair[0] + '\s+' + $pair[1] +
        '\s+.*Ignored\s+Max Delay Datapath Only\s*$'
    Assert-True ($interaction -match $pattern) `
        "$Prefix clock pair $($pair[0])->$($pair[1]) is not owned by XPM max-delay"
}
Assert-True ($interaction -match '(?m)^clk_fpga_2\s+clk_fpga_2\s+.*Clean\s+Timed\s*$') `
    "$Prefix 200 MHz TDC domain is not fully timed"
$Report.Add('PASS clock_interaction=xpm_max_delay_no_unsafe_paths')

$manualCdc = Read-Report "${Prefix}_manual_cdc.rpt"
$manualCdcLines = @($manualCdc -split "`r?`n" | Where-Object { $_.Trim() })
$expectedDiagnosticPins = if ($Stage -eq 'IMPL') { 12 } else { 40 }
$expectedManualTotal = if ($Stage -eq 'IMPL') { 40 } else { 68 }
$expectedManualCdc = [ordered]@{
    'tdc_pin_status' = 24
    'diagnostic_status' = $expectedDiagnosticPins
    'csr_idle' = 2
    'command_toggle' = 2
}
Assert-True ($manualCdcLines.Count -eq 5) `
    "$Prefix manual CDC report must contain four groups and one total"
foreach ($entry in $expectedManualCdc.GetEnumerator()) {
    $expected = 'PASS stage=' + $Prefix + ' group=' + $entry.Key +
        ' first_stage_pins=' + $entry.Value + ' async_reg_pins=' + $entry.Value
    Assert-True ($manualCdcLines -contains $expected) `
        "$Prefix manual CDC report is missing '$expected'"
}
Assert-True ($manualCdcLines -contains "PASS stage=$Prefix total_first_stage_pins=$expectedManualTotal") `
    "$Prefix manual CDC report does not prove the $expectedManualTotal-pin total"
$Report.Add("PASS manual_cdc.first_stage_pins=$expectedManualTotal")

if ($Stage -eq 'IMPL') {
    $manualDetail = Read-Report "${Prefix}_manual_cdc_detail.rpt"
    $detailLines = @($manualDetail -split "`r?`n" | Where-Object { $_.Trim() })
    Assert-True ($detailLines.Count -eq 40) `
        "$Prefix manual CDC detail must contain 40 surviving pins"
    foreach ($entry in $expectedManualCdc.GetEnumerator()) {
        $groupLines = @($detailLines | Where-Object { $_ -match " group=$($entry.Key) " })
        Assert-True ($groupLines.Count -eq $entry.Value) `
            "$Prefix manual CDC detail group $($entry.Key) expected $($entry.Value) pins"
    }

    $diagnosticLines = @($detailLines | Where-Object { $_ -match ' group=diagnostic_status ' })
    foreach ($signal in @('cmd_collision', 'err_bus_fatal', 'init_cfg_coalesced')) {
        foreach ($bit in 0..3) {
            $suffix = "/s_${signal}_meta_r_reg[$bit]/D"
            Assert-True ((@($diagnosticLines | Where-Object { $_.EndsWith($suffix) })).Count -eq 1) `
                "$Prefix manual CDC detail is missing the surviving diagnostic $suffix"
        }
    }
    $Report.Add('PASS manual_cdc.route_diagnostic_subset=cmd_collision,err_bus_fatal,init_cfg_coalesced')
}

$busSkew = Read-Report "${Prefix}_bus_skew.rpt"
$busSummary = [regex]::Match(
    $busSkew,
    '(?s)1\. Bus Skew Report Summary\r?\n-+\r?\n(?<body>.*?)' +
    '\r?\n2\. Bus Skew Report Per Constraint\r?\n-+'
)
Assert-True $busSummary.Success "$Prefix bus-skew summary could not be parsed"
$busRows = [regex]::Matches(
    $busSummary.Groups['body'].Value,
    '(?m)^\s+(?:Slow|Fast)\s+(-?\d+\.\d+)\s+(-?\d+\.\d+)\s+(-?\d+\.\d+)\s*$'
)
$minimumBusRows = if ($Stage -eq 'IMPL') { 35 } else { 40 }
Assert-True ($busRows.Count -ge $minimumBusRows) `
    "$Prefix bus-skew constraints dropped below $minimumBusRows (got $($busRows.Count))"
$busSlacks = @($busRows | ForEach-Object { [double]$_.Groups[3].Value })
$minimumBusSlack = [double](($busSlacks | Measure-Object -Minimum).Minimum)
Assert-True ($minimumBusSlack -ge 0.0) "$Prefix bus-skew failed: min slack=$minimumBusSlack ns"
$Report.Add("PASS bus_skew.constraints=$($busRows.Count)")
$Report.Add("PASS bus_skew.min_slack_ns=$minimumBusSlack")

$cdc = Read-Report "${Prefix}_cdc.rpt"
$cdcRows = @([regex]::Matches(
    $cdc,
    '(?m)^(?<id>CDC-\d+)\s+(?<severity>Critical|Warning|Info)\s+(?<count>\d+)\s+'
) | ForEach-Object {
    [pscustomobject]@{
        Id = $_.Groups['id'].Value
        Severity = $_.Groups['severity'].Value
        Count = [int]$_.Groups['count'].Value
    }
})
Assert-True ($cdcRows.Count -gt 0) "$Prefix CDC summary is empty"
$cdcCritical = @($cdcRows | Where-Object { $_.Severity -eq 'Critical' -and $_.Count -gt 0 })
Assert-True ($cdcCritical.Count -eq 0) (
    "$Prefix has critical CDC findings: " +
    (($cdcCritical | ForEach-Object { "$($_.Id)=$($_.Count)" }) -join ', ')
)
$cdcWarnings = @($cdcRows | Where-Object { $_.Severity -eq 'Warning' -and $_.Count -gt 0 })
Assert-AllowedRules $cdcWarnings @('CDC-6', 'CDC-15') 'cdc_warning'
$cdc6 = @($cdcWarnings | Where-Object { $_.Id -eq 'CDC-6' })
$cdc15 = @($cdcWarnings | Where-Object { $_.Id -eq 'CDC-15' })
$cdc6Count = 0
if ($cdc6.Count -gt 0) {
    Assert-True ($cdc6.Count -eq 1 -and $cdc6[0].Count -le 2) `
        "$Prefix CDC-6 count must be 0..2"
    $cdc6Count = $cdc6[0].Count
}
Assert-True ($cdc15.Count -eq 1 -and $cdc15[0].Count -eq 160) `
    "$Prefix CDC-15 count must be 160"

$cdc6Details = @($cdc -split "`r?`n" | Where-Object {
    $_ -match '^\s*\d+\s+CDC-6\s+Warning'
})
Assert-True ($cdc6Details.Count -eq $cdc6Count) `
    "$Prefix CDC-6 detail count $($cdc6Details.Count) differs from summary $cdc6Count"
$allowedCdc6Destinations = @(
    's_cmd_collision_vec_meta_r_reg',
    's_drain_faulted_mask_meta_r_reg'
)
$unexpectedCdc6 = @($cdc6Details | Where-Object {
    $line = $_
    -not ($allowedCdc6Destinations | Where-Object { $line.Contains($_) })
})
Assert-True ($unexpectedCdc6.Count -eq 0) `
    "$Prefix CDC-6 contains a non-diagnostic vector crossing"

$cdc15Details = @($cdc -split "`r?`n" | Where-Object {
    $_ -match '^\s*\d+\s+CDC-15\s+Warning'
})
Assert-True ($cdc15Details.Count -eq 160) `
    "$Prefix CDC-15 detail count must be 160, got $($cdc15Details.Count)"
Assert-True ((@($cdc15Details | Where-Object {
    $_ -notmatch 'xpm_fifo_base_inst'
})).Count -eq 0) "$Prefix CDC-15 contains a non-XPM-FIFO path"
$Report.Add("PASS cdc.cdc6_independent_masks=$cdc6Count")
$Report.Add('PASS cdc.cdc15_xpm_fifo_paths=160')
$Report.Add('PASS cdc.critical=0')

$methodology = Read-Report "${Prefix}_methodology.rpt"
$methodologyRows = @(Get-RuleSummary $methodology)
Assert-True ($methodologyRows.Count -gt 0) "$Prefix methodology summary is empty"
Assert-AllowedRules $methodologyRows @('TIMING-9', 'TIMING-18', 'TIMING-37', 'ULMTCS-1') `
    'methodology'
$expectedMethodology = @{
    'TIMING-9' = 1
    'TIMING-18' = 292
    'TIMING-37' = 1
    'ULMTCS-1' = 1
}
foreach ($rule in $expectedMethodology.GetEnumerator()) {
    $actual = @($methodologyRows | Where-Object { $_.Id -eq $rule.Key })
    Assert-True ($actual.Count -eq 1 -and $actual[0].Count -eq $rule.Value) `
        "$Prefix methodology $($rule.Key) expected $($rule.Value) checks"
}
Assert-True ((@($methodologyRows | Where-Object { $_.Severity -eq 'Critical Warning' })).Count -eq 0) `
    "$Prefix methodology has a critical warning"
Assert-True ((@($methodologyRows | Where-Object { $_.Id -eq 'TIMING-24' })).Count -eq 0) `
    "$Prefix XPM max-delay constraints are overridden (TIMING-24)"
$Report.Add('PASS methodology.timing24=0')

$drc = Read-Report "${Prefix}_drc.rpt"
$drcRows = @(Get-RuleSummary $drc)
Assert-True ($drcRows.Count -gt 0) "$Prefix DRC summary is empty"
$allowedDrc = @('NSTD-1', 'UCIO-1', 'REQP-181')
$expectedDrc = @{
    'NSTD-1' = 1
    'UCIO-1' = 1
    'REQP-181' = 2
}
if ($Stage -eq 'IMPL') {
    $allowedDrc += 'RTSTAT-10'
    $expectedDrc['RTSTAT-10'] = 1
}
Assert-AllowedRules $drcRows $allowedDrc 'drc'
foreach ($rule in $expectedDrc.GetEnumerator()) {
    $actual = @($drcRows | Where-Object { $_.Id -eq $rule.Key })
    Assert-True ($actual.Count -eq 1 -and $actual[0].Count -eq $rule.Value) `
        "$Prefix DRC $($rule.Key) expected $($rule.Value) checks"
}
$boardOpen = @($drcRows | Where-Object {
    $_.Id -in @('NSTD-1', 'UCIO-1') -and $_.Severity -eq 'Critical Warning'
})
Assert-True ($boardOpen.Count -eq 2) `
    "$Prefix must expose exactly the NSTD-1 and UCIO-1 board-open waivers"
$Report.Add('WAIVER board_io=NSTD-1,UCIO-1')

if ($Stage -eq 'IMPL') {
    $routeStatus = Read-Report 'post_route_status.rpt'
    Assert-True ($routeStatus -match '# of routable nets\.+\s*:\s*(\d+)\s*:') `
        'could not parse routable-net count'
    $routable = [int]$Matches[1]
    Assert-True ($routeStatus -match '# of fully routed nets\.+\s*:\s*(\d+)\s*:') `
        'could not parse fully-routed-net count'
    $fullyRouted = [int]$Matches[1]
    Assert-True ($routeStatus -match '# of nets with routing errors\.+\s*:\s*(\d+)\s*:') `
        'could not parse routing-error count'
    $routingErrors = [int]$Matches[1]
    Assert-True ($routable -gt 0 -and $fullyRouted -eq $routable -and $routingErrors -eq 0) `
        "route is incomplete: routable=$routable fully_routed=$fullyRouted errors=$routingErrors"
    $dcpPath = Join-Path $ResultDir 'post_route.dcp'
    Assert-True (Test-Path -LiteralPath $dcpPath) 'missing post-route checkpoint'
    Assert-True ((Get-Item -LiteralPath $dcpPath).Length -gt 0) 'post-route checkpoint is empty'
    $Report.Add("PASS route.fully_routed_nets=$fullyRouted")
    $Report.Add('PASS route.routing_errors=0')
    $Report.Add('PASS route.checkpoint=post_route.dcp')
}

$outputPath = Join-Path $ResultDir 'parent_ref_signoff_verified.txt'
$Report.Add("STATUS REFERENCE_${Stage}_PASS_WITH_BOARD_IO_OPEN")
$Report | Set-Content -LiteralPath $outputPath -Encoding ascii
Write-Host "Parent reference $Stage sign-off PASS (board I/O remains open)"
Write-Host "Sign-off report: $outputPath"
