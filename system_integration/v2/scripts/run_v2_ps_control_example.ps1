param(
    [string]$Stamp = (Get-Date -Format "yyMMddHHmmss"),
    [string]$BspInclude = ""
)

$ErrorActionPreference = "Stop"
$Hdl = (Resolve-Path (Join-Path $PSScriptRoot "../../..")).Path
$Source = Join-Path $Hdl "system_integration/v2/sw_reference/zynq_ps_example"
$Work = Join-Path $Hdl "tmp/v2_ps_control_example/$Stamp"
$Archive = Join-Path $Hdl (
    "signoff_results/sessions/${Stamp}_v2_ps_control_example")
$HostGcc = "C:/AMDDesignTools/2025.2.1/tps/mingw/10.0.0/win64.o/nt/bin/gcc.exe"
$ArmGcc = "C:/AMDDesignTools/2025.2.1/Vitis/gnu/aarch32/nt/" +
    "gcc-arm-none-eabi/bin/arm-none-eabi-gcc.exe"

function Invoke-Checked {
    param([string]$Exe, [string[]]$ArgList)
    & $Exe @ArgList
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed ($LASTEXITCODE): $Exe $($ArgList -join ' ')"
    }
}

foreach ($Required in @($HostGcc, $ArmGcc)) {
    if (-not (Test-Path -LiteralPath $Required)) {
        throw "Required compiler is missing: $Required"
    }
}
New-Item -ItemType Directory -Force -Path $Work, $Archive | Out-Null

$HostTest = Join-Path $Work "test_lidar_v2_ps_control.exe"
Invoke-Checked $HostGcc @(
    "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
    (Join-Path $Source "lidar_v2_ps_control.c"),
    (Join-Path $Source "test_lidar_v2_ps_control.c"),
    "-o", $HostTest)
$TestOutput = & $HostTest 2>&1
$TestOutput | Set-Content -Encoding ASCII (
    Join-Path $Archive "host_control_test.log")
if ($LASTEXITCODE -ne 0 -or
    $TestOutput -notmatch "LIDAR_V2_PS_CONTROL_TEST_PASS") {
    throw "Portable PS control host test failed."
}

Invoke-Checked $ArmGcc @(
    "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
    "-mcpu=cortex-a9", "-marm", "-ffreestanding", "-c",
    (Join-Path $Source "lidar_v2_ps_control.c"),
    "-o", (Join-Path $Work "lidar_v2_ps_control_cortex_a9.o"))

$XilinxCompile = "SKIPPED_NO_BSP"
$BoardMainCompile = "SKIPPED_NO_BSP"
if (-not [string]::IsNullOrWhiteSpace($BspInclude)) {
    $BspInclude = (Resolve-Path -LiteralPath $BspInclude).Path
    $VdmaDriver = "C:/AMDDesignTools/2025.2/data/embeddedsw/" +
        "XilinxProcessorIPLib/drivers/axivdma_v6_16/src"
    $IncludeArgs = @(
        "-I", $BspInclude,
        "-I", $VdmaDriver,
        "-I", (Join-Path $Hdl "system_integration/v2/sw_reference"),
        "-I", $Source,
        "-I", (Join-Path $Source "xilinx_standalone"))
    foreach ($Name in @(
            "lidar_packed17_ps_decoder.c",
            "zynq_ps_example/lidar_v2_ps_control.c",
            "zynq_ps_example/xilinx_standalone/lidar_v2_xilinx_vdma.c",
            "zynq_ps_example/xilinx_standalone/lidar_v2_lwip_udp.c",
            "zynq_ps_example/xilinx_standalone/lidar_v2_example_app.c")) {
        $InputPath = Join-Path (
            Join-Path $Hdl "system_integration/v2/sw_reference") $Name
        $ObjectName = ([IO.Path]::GetFileNameWithoutExtension($Name)) + ".o"
        $CompileArgs = @(
            "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
            "-mcpu=cortex-a9", "-marm", "-ffreestanding") +
            $IncludeArgs + @("-c", $InputPath, "-o", (Join-Path $Work $ObjectName))
        Invoke-Checked $ArmGcc $CompileArgs
    }
    $MainCompileArgs = @(
        "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
        "-mcpu=cortex-a9", "-marm", "-ffreestanding",
        "-DLIDAR_V2_BUILD_BOARD_MAIN",
        "-DLIDAR_V2_CSR_BASE_ADDRESS=0x40000000U",
        "-DLIDAR_V2_RISE_VDMA_DEVICE_ID=0U",
        "-DLIDAR_V2_FALL_VDMA_DEVICE_ID=1U",
        "-DLIDAR_V2_GIC_DEVICE_ID=0U",
        "-DLIDAR_V2_RISE_VDMA_IRQ_ID=61U",
        "-DLIDAR_V2_FALL_VDMA_IRQ_ID=62U",
        "-DLIDAR_V2_RISE_DDR_BASE=0x10000000U",
        "-DLIDAR_V2_FALL_DDR_BASE=0x11000000U",
        "-DLIDAR_V2_FRAME_CAPACITY_BYTES=1048576U",
        "-DLIDAR_V2_BIN_RESOLUTION_PS=81U") +
        $IncludeArgs + @(
            "-c", (Join-Path $Source (
                "xilinx_standalone/lidar_v2_main_example.c")),
            "-o", (Join-Path $Work "lidar_v2_main_example.o"))
    Invoke-Checked $ArmGcc $MainCompileArgs
    $XilinxCompile = "PASS"
    $BoardMainCompile = "PASS"
}

@(
    "session=$Stamp",
    "host_control_test=PASS",
    "cortex_a9_portable_compile=PASS",
    "cortex_a9_xilinx_adapter_compile=$XilinxCompile",
    "cortex_a9_board_main_compile=$BoardMainCompile",
    "csr_abi=2.7"
) | Set-Content -Encoding ASCII (Join-Path $Archive "summary.txt")
Copy-Item -LiteralPath (Join-Path $Source "test_lidar_v2_ps_control.c") `
    -Destination $Archive
Write-Output "LIDAR_V2_PS_CONTROL_EXAMPLE_PASS archive=$Archive"
