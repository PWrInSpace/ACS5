# ACS5 (ORC Meteor)

Roll-control flight computer firmware for the ORC Meteor board (STM32H725VGT6,
FreeRTOS): attitude estimation, flight-phase detection, canard roll control, SD
flight log and GNSS.

## Prerequisites

| Tool | Used for |
|---|---|
| `arm-none-eabi-gcc` 13.2 with newlib | firmware |
| CMake ≥ 3.22, Ninja | all builds |
| host `gcc`, Python 3 | host tests, log decoder |
| `dfu-util`, or an ST-Link with OpenOCD / STM32CubeProgrammer | flashing |
| STM32CubeMX 6.17 | only to change `ACS5.ioc` |

## Build

Flight image (`-O2`, LTO):

```bash
cmake --preset Release
cmake --build --preset Release   # -> build/Release/ACS5.elf
```

The `Debug` preset (`-O0 -g3`) is for the debugger only. Project sources build
with `-Wall -Wextra -Werror`.

## Flash

**USB DFU** (no programmer):

1. Hold BOOT0, press reset: the board enumerates as `0483:df11`.
2. Flash:
   ```bash
   arm-none-eabi-objcopy -O binary build/Release/ACS5.elf ACS5.bin
   dfu-util -a 0 -d 0483:df11 -s 0x08000000:leave -D ACS5.bin
   ```
3. Release BOOT0 and press reset. USB does not come up after the `:leave` jump alone.

**ST-Link (SWD)**:

```bash
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg \
        -c "program build/Release/ACS5.elf verify reset exit"
```

## Tests

```bash
bash test/host/run_tests.sh     # = cmake --preset host && build && ctest --preset host
```

Host unit tests for the platform-independent code (estimator, flight state
machine, arming, roll control, parsers, log format, sensor math) plus the roll
simulator scenarios and Monte Carlo gate, built with ASan and UBSan. CI
(`.github/workflows/ci.yml`) runs them and builds Debug, Release and Bench on
every push.

## Architecture

| Task | Rate | Does |
|---|---|---|
| flight (`tasks/flight_task.c`) | 500 Hz | IMU, baro, mag → attitude estimator → flight state machine → arming → roll controller → servos; publishes a log record every 8 ms in flight, 240 ms on the pad |
| SD log (`tasks/sd_log_task.c`) | on records | writes `LOG_NNN.BIN` to the card |
| GNSS (`tasks/gnss_task.c`) | 10 Hz | UART9 DMA → UBX NAV-PVT → `gnss_fix_t` |

| Directory (`Core/Inc`, `Core/Src`) | Contents | HAL |
|---|---|---|
| `filter/` | attitude estimator (Mahony backend) | no |
| `flight/` | flight state machine, arming, estimator input policy, baro datum | no |
| `control/` | roll controller, dynamic pressure, body axes | no |
| `log/` | binary log record | no |
| `utils/` | quaternions, loop rate, geodesy, timestamps, fault record | mostly no |
| `config/` | airframe parameters | no |
| `sensor_drivers/` | IIM-42653, MS5611, MMC5983MA, MAX-M10S, servos, SPI bus | yes (`*_math` files no) |
| `tasks/` | FreeRTOS task bodies | yes |

Design rationale too long for code comments: `docs/design.md`.

## Running it

USB CDC console (any terminal, 115200): boot messages, then telemetry at
12.5 Hz — state, loop rate, overruns, longest loop body (`busy`), Euler angles.
Every 24 s on the pad: gyro-bias and log counters (`[LOG] dropped/errors`).

A HardFault, `Error_Handler` or failed `configASSERT` resets the MCU; the next
boot prints `[FAULT] previous run: ...` with the PC/LR or file:line.

## Flight log

Card: microSD(HC), FAT32, up to 32 GB (exFAT is not enabled). Decode on a PC:

```bash
python3 tools/decode_log.py LOG_001.BIN -o LOG_001.csv
```

Format: `docs/log_format.md`.

## Board (ORC Meteor)

| Function | Pins |
|---|---|
| IMU / baro / mag | SPI2 (PB13–15), CS PD13 / PD10 / PD8 |
| SD card | SDMMC1, detect PD1 |
| LEDs | PA8 / PA4 |
| Buzzer | PE5, TIM15 |
| GNSS (on-board MAX-M10S) | UART9: PD15 TX, PD14 RX; reset PC6 |
| Cameras (SPLIT V3) | PC1 MOSFET, HIGH = on; on at STANDBY → ACCELERATION |
| Servos / lock | TIM3 CH1–4 (PA6/PA7/PB0/PB1 = J4 CH5..CH2) / TIM2_CH1 (PA5 = J4 CH6) |

J4 CH1 has no MCU pin on this board revision. Flight firmware holds the canards
neutral and the lock closed.

The GPS sheet nets `TX4/RX4` (module TXD/RXD) reach PD14/PD15, UART9 AF11.
`TX6/RX6` on PE7/PE8 go to the external `UART6` port and are not configured.

## Servo bench build

Cycles each canard through +10 / −10 / 0 deg (J4 CH2..CH5), then opens and
closes the lock (J4 CH6), printing each step over USB. Never runs the flight
loop — do not fly this build.

```bash
cmake --preset Bench
cmake --build --preset Bench    # -> build/Bench/ACS5_bench.elf
```

Procedure: `docs/bench/servo_bench.md`.

## Changing the CubeMX configuration

Peripheral setup lives in `ACS5.ioc`. Edit it in STM32CubeMX and regenerate;
project code inside generated files sits in `USER CODE` blocks, which CubeMX
keeps. Headless regeneration:

```bash
printf 'config load %s\nproject generate\nexit\n' "$PWD/ACS5.ioc" > gen.txt
~/STM32CubeMX/STM32CubeMX -q gen.txt
```

Check `git diff` afterwards: CubeMX may rewrite the linker script with CRLF line
endings; the project's copy carries the `.dma_buf` and `.noinit` sections.
