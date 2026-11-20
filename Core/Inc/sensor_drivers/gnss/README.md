# ACS5 GNSS driver

u-blox MAX-M10S over UART, UBX binary protocol.

## Files

| File | Layer | Depends on | Tested by |
|---|---|---|---|
| `sensor_drivers/gnss/ubx_parse.{h,c}` | UBX codec | libc | `test/host/test_ubx_parse.c` |
| `sensor_drivers/gnss/max_m10s.{h,c}` | hardware: UART, DMA, module setup | STM32 HAL, `ubx_parse` | bench only |
| `tasks/gnss_task.{h,c}` | FreeRTOS task, `gnss_fix_t` (the published fix) | CMSIS-RTOS2, `max_m10s`, `geo_ned` | bench only |
| `utils/geo_ned.{h,c}` | position relative to the pad (NED) | libm | `test/host/test_geo_ned.c` |

Everything that can be wrong about the wire format or the maths is reachable from
a host test; the hardware layer keeps DMA, timing and retries, and the task layer
keeps the RTOS objects and the published data.

## How it works

- **`ubx_parse`** — a pure C state machine. Feed it bytes one at a time with
  `ubx_parse_byte()`; it finds frame boundaries, checks the checksum and keeps
  health counters (`ubx_stats_t`). It also decodes `UBX-NAV-PVT` and
  `UBX-ACK-ACK/NAK`, builds outgoing frames, and says whether a solution has a
  usable position (`ubx_nav_pvt_has_position()`: `fix_ok` and a 3D or GNSS + dead
  reckoning fix; a time-only fix does not count).

- **`max_m10s`** — the HAL boundary. The driver state (`max_m10s_driver_t`) is
  allocated by the caller and tied to one UART:
  - `max_m10s_init(drv, huart)` — remember the UART and start circular
    receive-to-idle DMA into a 512-byte ring (`s_dma_buf`, section `.dma_buf`,
    32-byte aligned, placed in AXI SRAM by the linker script — not DTCM, which DMA
    cannot reach).
  - `max_m10s_configure(drv)` — blocking; brings the link up and configures the
    module (next section). Safe to call again after a failure.
  - `max_m10s_drain(drv, &frame)` — non-blocking; feeds new ring bytes to the
    parser and returns the next complete frame (`class`, `id`, `len`, `payload`),
    call it until it returns `false`. It invalidates the D-cache over just the
    bytes it is about to read.
  - `max_m10s_on_uart_error(drv)` — see "UART errors".
  - `max_m10s_get_stats(drv, &stats)` — parser counters plus UART overruns and
    other receive errors.

  Nothing here calls `HAL_Delay`, `osDelay` or any other RTOS primitive.
  `configure()` busy-polls (up to 200 ms per attempt), so call it from a task.

- **`gnss_task`** — the FreeRTOS boundary (see "Using the fix").

- **`geo_ned`** — turns latitude/longitude/height into metres north, east and down
  of a datum (WGS-84, scale factors computed once at the datum). Down is positive.

### Link set-up and the module's baud rate

The configuration is written to RAM and battery-backed RAM. After an MCU-only
reset, or a power cycle with the backup cell fitted, the module is still at
115200; without the cell it boots at 9600 (MAX-M10S, integration manual 2.1.4).
`max_m10s_configure()` therefore:

1. tries 115200 first (`CFG-UART1-BAUDRATE` = 115200, one attempt); an ACK means
   the module was already there;
2. otherwise, for 9600 and then 38400: switches the STM32 to that speed and sends
   the same command (its ACK is not relied on), waits 100 ms for the module to
   switch, then switches the STM32 to 115200 and requires an ACK there.

Every switch of the STM32 UART aborts the running receive first, re-initialises the
UART and restarts the DMA, checking each return value.


## Using the fix

The task is set up in two steps in `freertos.c`, before the kernel starts:

```c
if (!gnss_init(&huart7))       /* creates the mutex and semaphore, remembers the UART */
    Error_Handler();
gnssTaskHandle = osThreadNew(gnss_task, NULL, &gnssTask_attributes);
```

`gnss_task()` then brings the module up — retrying every 2 s for as long as it
takes — and afterwards publishes one `gnss_fix_t` per NAV-PVT. If no UBX frame
arrives for 3 s it reconfigures the module, which covers a module-only reset. Other code reads it
with:

```c
gnss_fix_t fix;
if (gnss_get_fix(&fix)) {   /* false: not initialised yet, or being written right now */
    /* ... */
}
```

`gnss_get_fix()` never blocks. Read the fields in this order:

| Field | Meaning |
|---|---|
| `fix.link` | `GNSS_LINK_STARTING` (not configured yet), `GNSS_LINK_OK`, or `GNSS_LINK_FAILED` (the task keeps retrying; `fix.link_error` says why: 1 HAL/DMA, 2 NAK, 3 no answer) |
| `fix.fix_valid` | the latest solution has a usable position. `false` together with `GNSS_LINK_OK` just means "no fix yet" |
| `fix.pvt` | the receiver's last NAV-PVT, as decoded: time, `fix_type`/`fix_ok`, position, accuracy, NED velocity, DOP |
| `fix.ned_valid`, `fix.ned_m` | position relative to the pad datum; only meaningful when `ned_valid` |
| `fix.stats` | `parser.frames_ok`, `parser.checksum_errors`, `parser.resyncs`, `parser.oversized`, `uart_overruns`, `uart_rx_errors` |

Conventions to check before integrating: `pvt.vel_ned_mps[2]` and `ned_m[2]` are
**Down, positive downward** (a climb is negative); `pvt.lat_1e7` / `pvt.lon_1e7`
are degrees x 1e7 — use `ned_m` for metres.

`gnss_capture_datum()` only sets a request flag; the datum is taken from the next
NAV-PVT with a usable position, and from then on `ned_valid` is true on every
snapshot that has `fix_valid`.

## Locked configuration

| Setting | Value | Source |
|---|---|---|
| Constellations | GPS + Galileo (+ SBAS + QZSS, which ride along with GPS) | team decision; the module ships with BeiDou on, so BeiDou and GLONASS are disabled explicitly |
| Navigation rate | 10 Hz | datasheet Table 2: GPS+GAL at the default CPU clock; 20 Hz needs an irreversible OTP write |
| UART baud | 115200 | 10 Hz x 100 B x 10 bit = 10 kbit/s, 9% of the line |
| Output message | UBX-NAV-PVT only | one message carries fix, position, velocity, accuracy and time |
| NMEA | disabled unconditionally | the driver parses UBX only |
| Dynamic model | airborne <4 g | the default portable model caps at 310 m/s horizontal, 50 m/s vertical, 12 km |
| Config persistence | RAM + BBR, re-sent every boot | MAX-M10S-00B is ROM SPG 5.10 with no flash; BBR survives while the backup cell holds |

### Configuration sequence sent by `max_m10s_configure()`

After the baud-rate negotiation above, the constellation keys go out as their own
`CFG-VALSET` (a signal change restarts the receiver; the driver waits 500 ms after
the ACK). The five settings below then go out together as **one** `CFG-VALSET`
message (`layers = 0x03`, RAM + BBR), so a single `ACK-ACK` covers them with one
retry budget (3 attempts x 200 ms); a NAK fails the whole group and is not retried.
The module therefore cannot say which of them it disliked.

| Key ID | Type | Value | Meaning |
|---|---|---|---|
| `0x40520001` | U4 | `115200` | CFG-UART1-BAUDRATE — sent first, on its own |
| `0x1031001f` / `0x10310021` | L | `1` / `1` | CFG-SIGNAL-GPS_ENA / GAL_ENA — sent second, with the two below |
| `0x10310022` / `0x10310025` | L | `0` / `0` | CFG-SIGNAL-BDS_ENA / GLO_ENA |
| `0x20110021` | U1 | `8` | CFG-NAVSPG-DYNMODEL — airborne <4 g |
| `0x30210001` | U2 | `100` | CFG-RATE-MEAS — 100 ms measurement period |
| `0x30210002` | U2 | `1` | CFG-RATE-NAV — 1 solution per measurement (with the row above: 10 Hz) |
| `0x20910007` | U1 | `1` | CFG-MSGOUT-UBX_NAV_PVT_UART1 — enable NAV-PVT on UART1 |
| `0x10740002` | L | `0` | CFG-UART1OUTPROT-NMEA — disable NMEA output on UART1 |

Key IDs come from **UBX-21035062** (see "Technical references"). The keys and
values are named constants and a table in `max_m10s.c`.

## Constraints

The receiver is specified to `<= 4 g`, 500 m/s, 80 km (datasheet Table 1). The
vehicle exceeds 4 g under thrust, so **expect the fix to drop during the burn and
for some seconds after** while the receiver reacquires. A NAV-PVT also describes
an epoch already 100-200 ms old when its last byte lands, which is 30-60 m of
position error at 300 m/s from latency alone.

The driver therefore reports quality rather than hiding it, and nothing in it
decides what is good enough to act on: `fix_valid == false` and
`ned_valid == false` are normal, recurring states that consumers must tolerate.


## Technical references

- **u-blox MAX-M10S Data Sheet** (UBX-20035208)
- **u-blox M10 SPG 5.10 Interface Description** (UBX-21035062)
