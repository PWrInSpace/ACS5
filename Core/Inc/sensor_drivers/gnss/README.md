# ACS5 GNSS driver

u-blox MAX-M10S over UART, UBX binary protocol.



## How the driver works

Three files, three jobs

- **`ubx_parse.{h,c}`** — Feed it bytes one at a time

- **`max_m10s.c`** — the HAL boundary. Owns the circular DMA receive buffer so the linker can keep it out of DTCM) and three entry points:

- **`tasks/gnss_task.c`** — the FreeRTOS boundary.

  `gnss_capture_datum()` only sets a request flag; the datum is taken from the
  next NAV-PVT with `fix_ok` and `fix_type >= 3D`.



## Locked configuration

| Setting | Value | Source |
|---|---|---|
| Constellations | GPS + Galileo (+ SBAS + QZSS, which ride along with GPS) | team decision; already the factory default, so no `CFG-SIGNAL` is sent |
| Navigation rate | 20 Hz | datasheet Table 2: GPS+GAL high-performance ceiling is exactly 20 Hz |
| UART baud | 115200 | 20 Hz x 100 B x 10 bit = 20 kbit/s, 17% of the line |
| Output message | UBX-NAV-PVT only | one message carries fix, position, velocity, accuracy and time |
| NMEA | disabled unconditionally | no runtime flag exists yet — see "Still open" |
| Dynamic model | airborne <4 g | the default pedestrian model rejects the trajectory outright |
| Config persistence | re-sent every boot | MAX-M10S-00B is ROM SPG 5.10; there is no flash to save to |


### Configuration sequence actually sent by `max_m10s_configure()`

Six `CFG-VALSET` messages, RAM layer only (`layers = 0x01`), sent in this exact
order and each gated on its own `ACK-ACK` before the next is sent:

| Step | Key ID | Type | Value | Meaning |
|---|---|---|---|---|
| 1 | `0x40520001` | U4 | `115200` | CFG-UART1-BAUDRATE — STM32 UART is switched to 115200 and DMA restarted right after this ACK |
| 2 | `0x20110021` | U1 | `8` | CFG-NAVSPG-DYNMODEL — airborne <4 g |
| 3 | `0x30210001` | U2 | `50` | CFG-RATE-MEAS — 50 ms measurement period |
| 4 | `0x30210002` | U2 | `1` | CFG-RATE-NAV — 1 solution per measurement (together with step 3: 20 Hz) |
| 5 | `0x20910007` | U1 | `1` | CFG-MSGOUT-UBX_NAV_PVT_UART1 — enable NAV-PVT on UART1 |
| 6 | `0x10740002` | L | `0` | CFG-UART1OUTPROT-NMEA — disable NMEA output on UART1 |



## Constraints

The receiver is specified to `<= 4 g`, 500 m/s, 80 km (datasheet Table 1). The
vehicle exceeds 4 g under thrust, so **expect the fix to drop during the burn and
for some seconds after** while the receiver reacquires.



## Still open — before/while transferring into the main ACS5 repo

The wire-format and HAL/RTOS layers are written and match this document, but
none of the following has been done yet:

1. **Schematic facts, still TBD**:
   - **Which UART** — `GNSS_HUART_HANDLE` in `max_m10s.h` is a placeholder
     (`#define GNSS_HUART_HANDLE huart3`). Update that one `#define` once the
     schematic is final; no other file needs to change.
   - `VIO_SEL` level (1.8 V I/O would make a 3.3 V TX line destructive),
     `V_BCKP`, `RESET_N` on a GPIO, `TIMEPULSE` routing, antenna type and its
     placement relative to the airframe (carbon fibre blocks L1 entirely).


## Technical references

- **u-blox MAX-M10S Data Sheet** (UBX-20035208) 
- **u-blox M10 SPG 5.10 Interface Description** (UBX-21035062)
