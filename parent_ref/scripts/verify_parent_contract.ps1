param(
    [Parameter(Mandatory = $true)]
    [string]$BdPath,
    [Parameter(Mandatory = $true)]
    [string]$ReportPath
)

$ErrorActionPreference = 'Stop'

function Assert-ContractValue {
    param(
        [Parameter(Mandatory = $true)][string]$Name,
        [AllowNull()]$Actual,
        [AllowNull()]$Expected
    )

    if ([string]$Actual -cne [string]$Expected) {
        throw "Parent contract mismatch: $Name expected '$Expected', got '$Actual'"
    }

    $script:Report.Add("PASS $Name=$Actual")
}

function Join-InterfacePorts {
    param([Parameter(Mandatory = $true)]$InterfaceNet)
    return (@($InterfaceNet.interface_ports) -join '|')
}

if (-not (Test-Path -LiteralPath $BdPath)) {
    throw "Parent block design was not generated: $BdPath"
}

$bd = Get-Content -Raw -LiteralPath $BdPath | ConvertFrom-Json
$design = $bd.design
$ps = $design.components.processing_system7_0
$tdc = $design.components.tdc_gpx_0
$vdmaRise = $design.components.vdma_rise
$vdmaFall = $design.components.vdma_fall
$psData = $design.addressing.'/processing_system7_0'.address_spaces.Data.segments
$riseData = $design.addressing.'/vdma_rise'.address_spaces.Data_S2MM.segments
$fallData = $design.addressing.'/vdma_fall'.address_spaces.Data_S2MM.segments
$Report = [System.Collections.Generic.List[string]]::new()

Assert-ContractValue 'device' $design.design_info.device 'xc7z020clg484-2'
Assert-ContractValue 'validated' $design.design_info.validated 'true'

Assert-ContractValue 'fclk0_actual_mhz' $ps.parameters.PCW_ACT_FPGA0_PERIPHERAL_FREQMHZ.value '100.000000'
Assert-ContractValue 'fclk1_actual_mhz' $ps.parameters.PCW_ACT_FPGA1_PERIPHERAL_FREQMHZ.value '150.000000'
Assert-ContractValue 'fclk2_actual_mhz' $ps.parameters.PCW_ACT_FPGA2_PERIPHERAL_FREQMHZ.value '200.000000'
Assert-ContractValue 'hp0_enabled' $ps.parameters.PCW_USE_S_AXI_HP0.value '1'
Assert-ContractValue 'hp1_enabled' $ps.parameters.PCW_USE_S_AXI_HP1.value '1'
Assert-ContractValue 'fabric_irq_enabled' $ps.parameters.PCW_USE_FABRIC_INTERRUPT.value '1'
Assert-ContractValue 'irq_f2p_enabled' $ps.parameters.PCW_IRQ_F2P_INTR.value '1'

Assert-ContractValue 'output_width_bits' $tdc.parameters.g_OUTPUT_WIDTH.value '32'
Assert-ContractValue 'slope_mode' $tdc.parameters.g_SLOPE_CHIP_MODE.value 'DEDICATED_2X2'
Assert-ContractValue 'stream_clock_mode' $tdc.parameters.g_STREAM_CLK_MODE.value 'ASYNC'
Assert-ContractValue 'ctrl_clock_busifs' $tdc.ports.i_ctrl_aclk.parameters.ASSOCIATED_BUSIF.value 's_axi:s_axi_pipe'
Assert-ContractValue 'ctrl_clock_hz' $tdc.ports.i_ctrl_aclk.parameters.FREQ_HZ.value '100000000'
Assert-ContractValue 'axis_clock_busifs' $tdc.ports.i_axis_aclk.parameters.ASSOCIATED_BUSIF.value 'm_axis:m_axis_fall'
Assert-ContractValue 'axis_clock_hz' $tdc.ports.i_axis_aclk.parameters.FREQ_HZ.value '150000000'
Assert-ContractValue 'tdc_clock_hz' $tdc.ports.i_tdc_clk.parameters.FREQ_HZ.value '200000000'

foreach ($lane in @('m_axis', 'm_axis_fall')) {
    Assert-ContractValue "$lane.bytes_per_beat" $tdc.interface_ports.$lane.parameters.TDATA_NUM_BYTES.value '4'
    Assert-ContractValue "$lane.freq_hz" $tdc.interface_ports.$lane.parameters.FREQ_HZ.value '150000000'
    Assert-ContractValue "$lane.clock_domain" $tdc.interface_ports.$lane.parameters.CLK_DOMAIN.value `
        'tdc_gpx_parent_processing_system7_0_0_FCLK_CLK1'
}

Assert-ContractValue 'vdma_rise.mm_width_bits' $vdmaRise.parameters.c_m_axi_s2mm_data_width.value '64'
Assert-ContractValue 'vdma_fall.mm_width_bits' $vdmaFall.parameters.c_m_axi_s2mm_data_width.value '64'
Assert-ContractValue 'vdma_rise.frame_stores' $vdmaRise.parameters.c_num_fstores.value '3'
Assert-ContractValue 'vdma_fall.frame_stores' $vdmaFall.parameters.c_num_fstores.value '3'

Assert-ContractValue 'rise_axis_path' `
    (Join-InterfacePorts $design.interface_nets.tdc_gpx_0_m_axis) `
    'tdc_gpx_0/m_axis|vdma_rise/S_AXIS_S2MM'
Assert-ContractValue 'fall_axis_path' `
    (Join-InterfacePorts $design.interface_nets.tdc_gpx_0_m_axis_fall) `
    'tdc_gpx_0/m_axis_fall|vdma_fall/S_AXIS_S2MM'
Assert-ContractValue 'rise_hp_path' `
    (Join-InterfacePorts $design.interface_nets.data_rise_M00_AXI) `
    'data_rise/M00_AXI|processing_system7_0/S_AXI_HP0'
Assert-ContractValue 'fall_hp_path' `
    (Join-InterfacePorts $design.interface_nets.data_fall_M00_AXI) `
    'data_fall/M00_AXI|processing_system7_0/S_AXI_HP1'

Assert-ContractValue 'addr.tdc_chip' $psData.SEG_tdc_gpx_0_reg0.offset '0x40000000'
Assert-ContractValue 'addr.tdc_pipeline' $psData.SEG_tdc_gpx_0_reg0_1.offset '0x40000800'
Assert-ContractValue 'addr.geometry' $psData.SEG_geometry_gpio_Reg.offset '0x41200000'
Assert-ContractValue 'addr.vdma_fall' $psData.SEG_vdma_fall_Reg.offset '0x43000000'
Assert-ContractValue 'addr.vdma_rise' $psData.SEG_vdma_rise_Reg.offset '0x43010000'
Assert-ContractValue 'rise_ddr_segment' `
    $riseData.SEG_processing_system7_0_HP0_DDR_LOWOCM.address_block `
    '/processing_system7_0/S_AXI_HP0/HP0_DDR_LOWOCM'
Assert-ContractValue 'fall_ddr_segment' `
    $fallData.SEG_processing_system7_0_HP1_DDR_LOWOCM.address_block `
    '/processing_system7_0/S_AXI_HP1/HP1_DDR_LOWOCM'

$reportDir = Split-Path -Parent $ReportPath
New-Item -ItemType Directory -Force -Path $reportDir | Out-Null
$Report | Set-Content -LiteralPath $ReportPath -Encoding ascii
Write-Host "Parent contract verification PASS ($($Report.Count) checks)"
Write-Host "Contract report: $ReportPath"
