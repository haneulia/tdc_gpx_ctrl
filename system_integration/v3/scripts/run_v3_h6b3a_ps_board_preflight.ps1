param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss"),
    [string]$BspInclude = "",
    [switch]$CheckVivadoVdma
)

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$V3Root = Split-Path -Parent $ScriptDir
$Hdl = (Resolve-Path (Join-Path $V3Root "..\..")).Path
$V3Ps = Join-Path $V3Root "sw_reference\zynq_ps_example"
$V2Ps = Join-Path $Hdl "system_integration\v2\sw_reference\zynq_ps_example"
$Work = Join-Path $Hdl ".work\v3_h6b3a_ps_board_preflight\$Stamp"
$HostGcc = "C:\AMDDesignTools\2025.2.1\tps\mingw\10.0.0\" +
    "win64.o\nt\bin\gcc.exe"
$ArmGcc = "C:\AMDDesignTools\2025.2.1\Vitis\gnu\aarch32\nt\" +
    "gcc-arm-none-eabi\bin\arm-none-eabi-gcc.exe"
$ArmNm = "C:\AMDDesignTools\2025.2.1\Vitis\gnu\aarch32\nt\" +
    "gcc-arm-none-eabi\bin\arm-none-eabi-nm.exe"
$Vivado = "C:\AMDDesignTools\2025.2.1\Vivado\bin\vivado.bat"
$VdmaDriver = "C:\AMDDesignTools\2025.2.1\data\embeddedsw\" +
    "XilinxProcessorIPLib\drivers\axivdma_v6_16\src"
$VdmaChannelSource = Join-Path $VdmaDriver "xaxivdma_channel.c"
$CsrPackage = Join-Path $Hdl "system_integration\v2\pkg\lidar_csr_map_pkg.vhd"
$V3Top = Join-Path $V3Root "rtl\top\tdc_gpx_lidar_ctrl_v3_top.vhd"
$V3AxisOutput = Join-Path $V3Root (
    "rtl\top\lidar_gpx_hls_axis_output_subsystem.vhd")
$ParentCreate = Join-Path $Hdl (
    "system_integration\v2\parent_l0\create_v2_l0_parent.tcl")
$ParentValidate = Join-Path $Hdl (
    "system_integration\v2\parent_l0\validate_v2_l0_parent.tcl")
$VdmaContractTcl = Join-Path $Hdl (
    "system_integration\v2\parent_l0\check_vdma_frame_irq_contract.tcl")

function Invoke-Checked {
    param([string]$Executable, [string[]]$ToolArguments)

    & $Executable @ToolArguments
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed ($LASTEXITCODE): $Executable " +
            ($ToolArguments -join " ")
    }
}

$TransactionSource = Join-Path $V3Ps "lidar_v3_vdma_transaction.c"
$TransactionHeader = Join-Path $V3Ps "lidar_v3_vdma_transaction.h"
$ControlSource = Join-Path $V3Ps "lidar_v3_control.c"
$ControlHeader = Join-Path $V3Ps "lidar_v3_control.h"
$TransactionTest = Join-Path $V3Ps (
    "tests\test_lidar_v3_vdma_transaction.c")
$AdapterTest = Join-Path $V3Ps (
    "tests\test_lidar_v3_xilinx_vdma_adapter.c")
$ControlTest = Join-Path $V3Ps "tests\test_lidar_v3_control.c"
$StubInclude = Join-Path $V3Ps "tests\stubs"
$V2ControlSource = Join-Path $V2Ps "lidar_v2_ps_control.c"
$V2ControlHeader = Join-Path $V2Ps "lidar_v2_ps_control.h"
$DecoderSource = Join-Path $Hdl (
    "system_integration\v2\sw_reference\lidar_packed17_ps_decoder.c")
$DecoderHeader = Join-Path $Hdl (
    "system_integration\v2\sw_reference\lidar_packed17_ps_decoder.h")
$AdapterSource = Join-Path $V3Ps (
    "xilinx_standalone\lidar_v3_xilinx_vdma_adapter.c")
$AdapterHeader = Join-Path $V3Ps (
    "xilinx_standalone\lidar_v3_xilinx_vdma_adapter.h")
$FrameServiceSource = Join-Path $V3Ps (
    "xilinx_standalone\lidar_v3_xilinx_frame_service.c")
$FrameServiceHeader = Join-Path $V3Ps (
    "xilinx_standalone\lidar_v3_xilinx_frame_service.h")
$VdmaStub = Join-Path $StubInclude "xaxivdma.h"
$CacheStub = Join-Path $StubInclude "xil_cache.h"
$StatusStub = Join-Path $StubInclude "xstatus.h"
$V2VdmaSource = Join-Path $V2Ps (
    "xilinx_standalone\lidar_v2_xilinx_vdma.c")
$V2VdmaHeader = Join-Path $V2Ps (
    "xilinx_standalone\lidar_v2_xilinx_vdma.h")

foreach ($Required in @(
        $HostGcc, $ArmGcc, $ArmNm, $TransactionSource, $TransactionHeader,
        $ControlSource, $ControlHeader, $TransactionTest, $AdapterTest,
        $ControlTest,
        $V2ControlSource, $V2ControlHeader,
        $DecoderSource, $DecoderHeader,
        $AdapterSource, $AdapterHeader,
        $FrameServiceSource, $FrameServiceHeader,
        $V2VdmaSource, $V2VdmaHeader,
        $VdmaStub, $CacheStub, $StatusStub,
        $VdmaChannelSource,
        $CsrPackage, $V3Top, $V3AxisOutput, $ParentCreate,
        $ParentValidate, $VdmaContractTcl)) {
    if (-not (Test-Path -LiteralPath $Required)) {
        throw "Required file is missing: $Required"
    }
}
New-Item -ItemType Directory -Force -Path $Work | Out-Null

$CsrText = Get-Content -Raw -LiteralPath $CsrPackage -Encoding UTF8
$HeaderText = Get-Content -Raw -LiteralPath $V2ControlHeader -Encoding UTF8
$TopText = Get-Content -Raw -LiteralPath $V3Top -Encoding UTF8
$AxisText = Get-Content -Raw -LiteralPath $V3AxisOutput -Encoding UTF8
$FrameServiceText = Get-Content -Raw -LiteralPath $FrameServiceSource `
    -Encoding UTF8
$AdapterText = Get-Content -Raw -LiteralPath $AdapterSource -Encoding UTF8
$AdapterHeaderText = Get-Content -Raw -LiteralPath $AdapterHeader `
    -Encoding UTF8
$ParentCreateText = Get-Content -Raw -LiteralPath $ParentCreate -Encoding UTF8
$ParentValidateText = Get-Content -Raw -LiteralPath $ParentValidate `
    -Encoding UTF8
$V2VdmaText = Get-Content -Raw -LiteralPath $V2VdmaSource -Encoding UTF8
$VdmaChannelText = Get-Content -Raw -LiteralPath $VdmaChannelSource `
    -Encoding UTF8
if ($CsrText -notmatch 'C_LIDAR_CSR_ABI_MAJOR\s*:\s*natural\s*:=\s*2' -or
    $CsrText -notmatch 'C_LIDAR_CSR_ABI_MINOR\s*:\s*natural\s*:=\s*7' -or
    $HeaderText -notmatch '#define LIDAR_V2_CSR_ABI_MAJOR 2U' -or
    $HeaderText -notmatch '#define LIDAR_V2_CSR_ABI_MINOR 7U') {
    throw "RTL and PS CSR ABI 2.7 contract diverged"
}
if ($TopText -notmatch 'G_OUTPUT_WIDTH\s*:\s*positive\s*:=\s*32' -or
    $AxisText -notmatch 'output_width\s*=\s*32\s+or' -or
    $AxisText -notmatch 'output_width\s*=\s*64') {
    throw "V3 32/64-bit synthesis-width contract diverged"
}
if ($FrameServiceText -notmatch 'poll_processing_idle_snapshot' -or
    $FrameServiceText -notmatch 'LIDAR_V3_DIAG_ALL_PROCESSING_IDLE' -or
    $FrameServiceText -notmatch 'LIDAR_V3_DIAG_GPX_CDC_RESET_BUSY') {
    throw "V3 pre-COMMIT processing-idle gate is missing"
}
if ($AdapterText -notmatch 'XAxiVdma_SetFrameCounter' -or
    $AdapterHeaderText -notmatch
        'LIDAR_V3_XILINX_MIN_FRAME_STORES\s+3U' -or
    $AdapterHeaderText -notmatch
        'LIDAR_V3_XILINX_FRAME_IRQ_THRESHOLD\s+1U' -or
    $ParentCreateText -notmatch 'CONFIG\.c_num_fstores\s+3' -or
    $ParentCreateText -notmatch 'CONFIG\.c_include_mm2s\s+0' -or
    $ParentCreateText -notmatch 'CONFIG\.c_include_s2mm\s+1' -or
    $ParentValidateText -notmatch 'CONFIG\.c_num_fstores') {
    throw "S2MM one-Face-frame interrupt contract diverged"
}
if ($V2VdmaText -notmatch
        'setup\.EnableFrameCounter\s*=\s*0\s*;' -or
    $VdmaChannelText -notmatch
        'XAXIVDMA_CR_FRMCNT_EN_MASK\s*\|\s*XAXIVDMA_CR_RD_PTR_MASK' -or
    $VdmaChannelText -notmatch
        'XAXIVDMA_DELAY_MASK\s*\|\s*XAXIVDMA_FRMCNT_MASK') {
    throw "Continuous S2MM/frame-IRQ driver contract diverged"
}

$HostTest = Join-Path $Work "test_lidar_v3_vdma_transaction.exe"
Invoke-Checked $HostGcc @(
    "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
    "-I", $V3Ps, "-I", $V2Ps,
    $V2ControlSource, $TransactionSource, $TransactionTest,
    "-o", $HostTest
)
$HostOutput = & $HostTest 2>&1
$HostOutput | Set-Content -Encoding ASCII -LiteralPath (
    Join-Path $Work "host_transaction_test.log")
if ($LASTEXITCODE -ne 0 -or
    ($HostOutput -join "`n") -notmatch
        "LIDAR_V3_H6B3A_VDMA_TRANSACTION_PASS") {
    throw "V3 VDMA transaction host test failed"
}

$HostControlTest = Join-Path $Work "test_lidar_v3_control.exe"
Invoke-Checked $HostGcc @(
    "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
    "-I", $V3Ps, "-I", $V2Ps,
    $V2ControlSource, $TransactionSource, $ControlSource, $ControlTest,
    "-o", $HostControlTest
)
$ControlOutput = & $HostControlTest 2>&1
$ControlOutput | Set-Content -Encoding ASCII -LiteralPath (
    Join-Path $Work "host_control_recovery_test.log")
if ($LASTEXITCODE -ne 0 -or
    ($ControlOutput -join "`n") -notmatch
        "LIDAR_V3_H6B3A_CONTROL_RECOVERY_PASS") {
    throw "V3 control recovery host test failed"
}

$HostAdapterTest = Join-Path $Work "test_lidar_v3_xilinx_vdma_adapter.exe"
Invoke-Checked $HostGcc @(
    "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
    "-I", $StubInclude,
    "-I", (Join-Path $V3Ps "xilinx_standalone"),
    "-I", $V3Ps,
    "-I", (Join-Path $V2Ps "xilinx_standalone"),
    "-I", $V2Ps,
    "-I", (Join-Path $Hdl "system_integration\v2\sw_reference"),
    $V2ControlSource, $DecoderSource, $TransactionSource,
    $AdapterSource, $FrameServiceSource, $AdapterTest,
    "-o", $HostAdapterTest
)
$AdapterOutput = & $HostAdapterTest 2>&1
$AdapterOutput | Set-Content -Encoding ASCII -LiteralPath (
    Join-Path $Work "host_xilinx_adapter_test.log")
if ($LASTEXITCODE -ne 0 -or
    ($AdapterOutput -join "`n") -notmatch
        "LIDAR_V3_H6B3A_XILINX_ADAPTER_PASS") {
    throw "V3 Xilinx VDMA adapter host test failed"
}

foreach ($Source in @(
        $TransactionSource, $ControlSource, $V2ControlSource,
        $DecoderSource)) {
    $Object = Join-Path $Work (
        ([IO.Path]::GetFileNameWithoutExtension($Source)) + "_cortex_a9.o")
    Invoke-Checked $ArmGcc @(
        "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
        "-mcpu=cortex-a9", "-marm", "-ffreestanding",
        "-I", $V3Ps, "-I", $V2Ps,
        "-c", $Source, "-o", $Object
    )
}

$XilinxAdapterCompile = "SKIPPED_NO_BSP"
if (-not [string]::IsNullOrWhiteSpace($BspInclude)) {
    $BspInclude = (Resolve-Path -LiteralPath $BspInclude).Path
    $IncludeArguments = @(
        "-I", $BspInclude,
        "-I", $VdmaDriver,
        "-I", $V3Ps,
        "-I", (Join-Path $V3Ps "xilinx_standalone"),
        "-I", $V2Ps,
        "-I", (Join-Path $V2Ps "xilinx_standalone"),
        "-I", (Join-Path $Hdl "system_integration\v2\sw_reference")
    )
    foreach ($Source in @(
            $AdapterSource, $FrameServiceSource, $V2VdmaSource)) {
        $Object = Join-Path $Work (
            ([IO.Path]::GetFileNameWithoutExtension($Source)) + ".o")
        Invoke-Checked $ArmGcc (@(
            "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
            "-mcpu=cortex-a9", "-marm", "-ffreestanding") +
            $IncludeArguments + @("-c", $Source, "-o", $Object))
        if ($Source -eq $AdapterSource -or
            $Source -eq $FrameServiceSource) {
            $Undefined = & $ArmNm -u $Object 2>&1
            if ($LASTEXITCODE -ne 0 -or
                ($Undefined -join "`n") -match '__atomic_') {
                throw "Cortex-A9 adapter requires an external atomic helper"
            }
        }
    }
    $XilinxAdapterCompile = "PASS"
}

$VivadoVdmaContract = "SKIPPED"
if ($CheckVivadoVdma) {
    if (-not (Test-Path -LiteralPath $Vivado)) {
        throw "Vivado executable is missing: $Vivado"
    }
    $VivadoContractWork = Join-Path $Work "vivado_vdma_contract"
    $VivadoAppData = Join-Path $Work "vivado_appdata"
    New-Item -ItemType Directory -Force -Path $VivadoContractWork |
        Out-Null
    New-Item -ItemType Directory -Force -Path $VivadoAppData | Out-Null
    $SavedAppData = $env:APPDATA
    $SavedHome = $env:HOME
    $SavedUserProfile = $env:USERPROFILE
    $SavedLocalAppData = $env:LOCALAPPDATA
    $VivadoLocationPushed = $false
    try {
        $env:APPDATA = $VivadoAppData
        $env:HOME = $VivadoAppData
        $env:USERPROFILE = $VivadoAppData
        $env:LOCALAPPDATA = $VivadoAppData
        # Vivado may create a versioned Tcl Store relative to its startup
        # directory before the supplied Tcl script changes directory.
        Push-Location $VivadoContractWork
        $VivadoLocationPushed = $true
        $VivadoOutput = & $Vivado -mode batch -nolog -nojournal -notrace `
            -source $VdmaContractTcl -tclargs $VivadoContractWork 2>&1
    } finally {
        if ($VivadoLocationPushed) {
            Pop-Location
        }
        $env:APPDATA = $SavedAppData
        $env:HOME = $SavedHome
        $env:USERPROFILE = $SavedUserProfile
        $env:LOCALAPPDATA = $SavedLocalAppData
    }
    $VivadoOutput | Set-Content -Encoding ASCII -LiteralPath (
        Join-Path $Work "vivado_vdma_frame_irq_contract.log")
    if ($LASTEXITCODE -ne 0 -or
        ($VivadoOutput -join "`n") -notmatch
            "LIDAR_VDMA_FRAME_IRQ_CONTRACT_PASS" -or
        ($VivadoOutput -join "`n") -match "CRITICAL WARNING:") {
        throw "Vivado AXI VDMA frame interrupt contract failed"
    }
    $VivadoVdmaContract = "PASS"
}

$ManifestFiles = @(
    $PSCommandPath, $TransactionSource, $TransactionHeader, $TransactionTest,
    $ControlSource, $ControlHeader, $ControlTest,
    $AdapterSource, $AdapterHeader, $AdapterTest,
    $FrameServiceSource, $FrameServiceHeader,
    $VdmaStub, $CacheStub, $StatusStub,
    $V2ControlSource, $V2ControlHeader,
    $DecoderSource, $DecoderHeader,
    $V2VdmaSource, $V2VdmaHeader, $CsrPackage, $V3Top, $V3AxisOutput,
    $ParentCreate, $ParentValidate, $VdmaContractTcl,
    $VdmaChannelSource
)
$Manifest = foreach ($File in $ManifestFiles) {
    $Item = Get-Item -LiteralPath $File
    [ordered]@{
        path = $Item.FullName.Replace('\', '/')
        bytes = $Item.Length
        sha256 = (Get-FileHash -LiteralPath $File -Algorithm SHA256).Hash
    }
}
$Manifest | ConvertTo-Json -Depth 3 | Set-Content -Encoding ASCII `
    -LiteralPath (Join-Path $Work "source_manifest.json")

$Summary = [ordered]@{
    checkpoint = "V3 H6-B3A PS board preflight"
    shared_csr_abi = "2.7"
    amd_axivdma_driver = "6.16"
    supported_output_width_bits = @(32, 64)
    host_transaction_test = "PASS"
    host_control_recovery_test = "PASS"
    host_xilinx_adapter_test = "PASS"
    cortex_a9_transaction_compile = "PASS"
    cortex_a9_packed17_ownership_compile = "PASS"
    xilinx_vdma_adapter_compile = $XilinxAdapterCompile
    vivado_vdma_frame_irq_contract = $VivadoVdmaContract
    cortex_a9_lock_free_mask_atomic = $XilinxAdapterCompile
    verified = @(
        "Rise/Fall lane ACK follows each successful hardware apply",
        "one lane retry does not reprogram the acknowledged lane",
        "CDC ACK release latency does not duplicate programming",
        "changed profile snapshots are never acknowledged as the old profile",
        "CTL23/24 processing-idle sequence gates VDMA stop",
        "busy, stale, error, non-idle, and GPX CDC-reset snapshots block stop",
        "optional poll-wait hook prevents consumer-task starvation",
        "final completed frames remain CPU-owned until consumer success",
        "S2MM completion interrupt threshold is one Face Frame",
        "continuous S2MM keeps the one-Face interrupt coalescing threshold",
        "S2MM-only VDMA and at least three Frame Stores are mandatory",
        "32/64-bit geometry, frame capacity, alignment, and DDR overlap guards"
    )
    board_exclusion = @(
        "actual XAxiVdma register effects",
        "physical DDR S2MM and Cortex-A9 cache maintenance",
        "Ethernet MAC/PHY transmission",
        "bitstream and physical TDC-GPX, encoder, laser, and LVDS I/O"
    )
}
$Summary | ConvertTo-Json -Depth 4 | Set-Content -Encoding ASCII `
    -LiteralPath (Join-Path $Work "summary.json")

Write-Output "LIDAR_V3_H6B3A_PS_BOARD_PREFLIGHT_PASS"
Write-Output "Result: $Work"
