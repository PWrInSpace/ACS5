# ACS5 (ORC Meteor)

Port of the ACS4v2 flight stack onto the ORC Meteor board (STM32H725VGT6), plus
the MAX-M10S GNSS driver already developed for ACS5.

## Board deltas vs ACS4v2

| Function | ACS4v2 | ACS5 / Meteor |
|---|---|---|
| IMU CS | PD8 | **PD13** |
| BARO CS | PD11 | **PD10** |
| MAG CS | PB12 | **PD8** |
| SD detect | PC0 | **PD0** |
| LEDs | PA1/2/3 | **PA7 / PA3** |
| Buzzer | PE5 TIM15 | PE5 TIM15 (unchanged) |
| GNSS | — | **UART7 PE8 TX / PE7 RX** |
| Cameras | — | **PC0/PC1/PC2 MOSFET HIGH = on** |
| Servos / lock | TIM4 / TIM1 | **disabled** (pinout conflict) |

Camera (SPLIT V3) turns on at the STANDBY → ACCELERATION transition.

## Build

```bash
cmake --preset Debug
cmake --build --preset Debug
```

## GNSS note

The Meteor schematic labels a board port `TX6/RX6` on PE8/PE7; those pads are
UART7 in the H725 alternate-function table. PD14/PD15 are also labeled
`TX4/RX4` near the GPS sheet but have no UART TX/RX AF — firmware uses UART7.
Confirm on the harness before flight.
