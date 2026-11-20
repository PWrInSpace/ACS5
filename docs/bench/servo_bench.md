# Servo bench test (bench build)

Goal: confirm which J4 socket drives which fin, the deflection directions and the
canard lock. The results feed the 3-axis controller's mixer.

## First power-up of a board

The system clock comes from the 50 MHz HSE crystal (Y1). If the crystal does not
start, the firmware stops in `Error_Handler` right after power-on: no USB
enumeration, no messages, no LED activity. Check the Y1 assembly and C34/C35
(6 pF).

1. Flash the flight build, connect USB.
2. The CDC port appears and the terminal shows `--- ACS5 Flight Task Starting ---`.
3. Telemetry reports a ~500 Hz loop (`[STANDBY] 500 Hz`).

## Setup

1. SRT T75 servos in J4 CH2–CH5, the lock servo in CH6. CH1 has no MCU pin.
2. VIN supply for the servos; USB to the computer, terminal on the CDC port.
3. Bench build (README, "Servo bench build"), flash.

## Procedure

1. After start-up: the lock opens for 3 s and closes; the terminal shows
   `[OK] Servos armed (neutral), canards locked`, then
   `[BENCH] Servo bench build - flight loop disabled`.
2. For each socket CH2–CH5 the terminal prints `[BENCH] J4 CHx +10 deg`, `-10 deg`,
   `+0 deg`, one per second. Only the servo in CHx moves. Record which fin on the
   airframe it is and which way the trailing edge moves at `+10`.
3. `[BENCH] LOCK OPEN` → lock open; after 3 s `[BENCH] LOCK CLOSED` → closed.
4. The cycle repeats every 18 s.

## Flight build

1. Flash the flight build.
2. After start-up: lock sequence (open 3 s → closed), canards neutral, no servo
   movement in STANDBY.
3. LED_1 (PA8) blinks with the telemetry; the servos do not twitch with it.

## Results

Date: — Board: — Tested by: —

| J4 socket | Physical fin (position on the airframe, seen from the nose) | Trailing-edge direction at +10° | Neutral [µs] |
|---|---|---|---|
| CH2 (fin 0) | | | |
| CH3 (fin 1) | | | |
| CH4 (fin 2) | | | |
| CH5 (fin 3) | | | |

| Lock (CH6) | Result |
|---|---|
| 1050 µs | open / closed |
| 1900 µs | open / closed |
