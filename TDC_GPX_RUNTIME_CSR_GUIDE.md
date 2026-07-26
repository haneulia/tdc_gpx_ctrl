# TDC-GPX Runtime CSR 가이드

## 1. 두 개의 독립 주소 공간

`tdc_gpx_top`은 기능 소유권과 기존 software ABI를 보존하기 위해 AXI4-Lite slave를 두 개 제공한다.

| Slave | 주소 폭 | 용도 |
|---|---:|---|
| `s_axi` | 9 bit | TDC chip 설정, register access, scan/falling 정책, chip IRQ |
| `s_axi_pipe` | 7 bit | pipeline/shot 설정, build profile, 오류/진단 status |

두 slave의 base address는 Vivado Address Editor에서 별도로 할당한다. 아래 offset은 각 base에 더한다.

## 2. Pipeline CSR (`s_axi_pipe`)

### Control

| Offset | 이름 | 주요 bit |
|---:|---|---|
| `0x00` | MAIN_CTRL | `[3:0]` active chip, `[4]` packet scope, `[6:5]` hit mode, `[9:7]` distance scale, `[10]` drain mode, `[11]` enable, `[18:15]` stops/chip, `[22:19]` drain cap, `[27:23]` STOPDIS override, `[31:28]` command |
| `0x04` | RANGE_COLS | `[15:0]` max range in 5 ns ticks, `[31:16]` columns/face |
| `0x08` | AUX_CMD | `[0]` force reinit, `[1]` error soft clear |
| `0x0C..0x1C` | reserved | write하지 않는다 |

MAIN_CTRL command:

| Bit | 명령 |
|---:|---|
| 28 | start |
| 29 | stop |
| 30 | soft reset |
| 31 | chip configuration write |

명령은 rising-edge 검출 방식이다. software는 일반 field shadow를 유지하고, 원하는 command bit를 `0→1`, 다음 write에서 `1→0`으로 복귀시킨다. AUX_CMD도 동일하다.

### Status

| Offset | 이름 | 내용 |
|---:|---|---|
| `0x40` | HW_VERSION | build version |
| `0x44` | HW_CONFIG | chip/stops/hits/TDATA/cell/falling/n_faces profile |
| `0x48` | MAX_ROWS | build profile 최대 row |
| `0x4C` | CELL_SIZE | canonical 최대 cell byte |
| `0x50` | MAX_HSIZE | full-mask 최대 packed line byte |
| `0x54` | STATUS | busy, overrun, fatal, chip/drain/sequence 오류 |
| `0x58` | STATUS_EXT | flush, header drain, collision, overflow, per-shot drain 상태 |
| `0x5C` | STATUS_EXT2 | register timeout, stop ID, quarantine, slope mask, face collapse, init coalesce |

`0x20..0x3C` native status alias와 native IRQ register는 wrapper에서 숨겨진다. Pipeline interrupt input은 현재 reserved이므로 `o_irq_pipe`를 운용 완료/오류 interrupt로 사용하지 말고 `0x54..0x5C`를 읽는다.

### 주요 상태 bit

STATUS `0x54`:

- `[0]` busy
- `[1]` pipeline overrun
- `[2]` fatal retry exhaustion
- `[7:4]` live chip error mask
- `[11:8]` drain timeout mask
- `[15:12]` sequence error mask

STATUS_EXT2 `0x5C`:

- `[3:0]` register timeout mask
- `[7:4]` invalid stop ID mask
- `[10:8]` last run timeout cause
- `[14:11]` quarantine escape mask
- `[15]` masked slope drop
- `[19:16]` rising face-start collapse count low nibble
- `[27:24]` falling face-start collapse count low nibble
- `[31:28]` init configuration coalesce mask

sticky clear 정책은 field마다 다르다. `err_soft_clear`로 지워지는 운용 sticky와 hard reset만으로 지워지는 historical evidence를 구분해야 한다. 상세 bit 정의는 [register_map.md](Doc/register_map.md)를 참조한다.

## 3. Chip CSR (`s_axi`)

### Control

| Offset | 이름 | 주요 bit |
|---:|---|---|
| `0x00` | CTL0 | unused |
| `0x04` | BUS_TIMING | divider/ticks, target address/chip/mask, read/write trigger |
| `0x08` | CTL2 | unused |
| `0x0C` | START_OFF1 | `[17:0]` |
| `0x10` | CFG_REG7 | chip register 7 image |
| `0x14..0x50` | CFG_IMAGE[0..15] | 16 x 32-bit TDC-GPX register image |
| `0x54` | SCAN_TIMEOUT | `[15:0]` max scan 5 ns ticks, `[18:16]` max hits, `[19]` falling enable |
| `0x58..0x7C` | reserved | write하지 않는다 |

Runtime 값은 build cap을 넘을 수 없다.

- active mask는 `g_PRESENT_CHIP_MASK`로 제한된다.
- stops/chip은 `2..g_MAX_STOPS_PER_CHIP`으로 clamp된다.
- max hits의 0은 build maximum alias이고, 그 이상은 build maximum으로 clamp된다.
- falling build가 제거된 경우 `falling_enable=0`을 유지한다.

### Status

| Offset | 이름 | 내용 |
|---:|---|---|
| `0x80` | CHIP0_RESULT | `[27:0]` read data, `[31:28]` register address |
| `0x84` | CHIP1_RESULT | 동일 |
| `0x88` | CHIP2_RESULT | 동일 |
| `0x8C` | CHIP3_RESULT | 동일 |
| `0x90..0xFC` | reserved | zero |

물리 chip 수가 4보다 작아도 status ABI는 네 logical slot을 유지한다. absent slot은 사용하지 않는다.

## 4. Chip interrupt

Chip CSR interrupt register:

| Offset | 접근 | 의미 |
|---:|---|---|
| `0x100` | R/W | INTR_EN |
| `0x104` | R/O | synchronized source level |
| `0x108` | R/W1C | manual-mode pending flag |
| `0x10C` | R/W | mode: 0=manual level, 1=one-clock automatic pulse |

Source bit:

- bit 0: chip register transaction done pulse
- bit 1: reserved, 항상 0

권장 manual interrupt 순서:

1. `INTR_EN=0`
2. `INTR_MODE bit0=0`
3. `INTR_FLAG bit0=1`을 써서 stale flag clear
4. `INTR_EN bit0=1`
5. `o_irq=1`이면 transaction result를 읽는다.
6. `INTR_FLAG bit0=1` W1C 후 ISR을 종료한다.

새 event와 W1C가 같은 cycle이면 새 event가 우선하여 유실되지 않는다.

Automatic mode에서 bit 0을 1로 설정하면 `o_irq`는 `s_axi_aclk` 한 cycle pulse이며 INTR_FLAG에는 남지 않는다. PS interrupt controller가 짧은 pulse를 놓칠 가능성이 있으면 manual mode를 사용한다.

## 5. 5 ns tick 적용

두 runtime timeout은 200 MHz reference tick을 사용한다.

| Field | 소비 domain |
|---|---|
| `max_range_5ns_ticks` | TDC + AXIS |
| `max_scan_5ns_ticks` | AXIS |

```text
local_count = ceil(csr_ticks * local_clock_MHz / 200)
```

예를 들어 1000 ticks는 항상 5 us이다. 150 MHz domain에서는 750 clocks, 200 MHz domain에서는 1000 clocks로 사용된다.

## 6. 권장 software transaction

1. reset 후 두 bank의 build/status를 읽는다.
2. Chip CFG image와 timing을 기록한다.
3. Pipeline runtime profile과 range/columns를 기록한다.
4. CFG_WRITE pulse를 보낸다.
5. busy 해제와 오류 mask 0을 확인한다.
6. START pulse를 보낸다.
7. 운용 중 STATUS/EXT/EXT2와 VDMA completion을 함께 확인한다.
8. 종료 시 STOP pulse 후 최종 sticky를 읽는다.
9. 원인 보존이 필요 없어진 뒤 AUX_CMD error soft clear를 pulse한다.
