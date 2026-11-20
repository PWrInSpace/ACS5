# Design notes

Rationale behind non-obvious choices in the firmware and host tests, grouped by module. Code comments point here by anchor.

## Airframe parameters

All airframe-dependent defaults live in `Core/Inc/config/airframe.h`. The current values belong to the previous airframe (Joust 1 on a K353 motor) and must be re-derived for a new one. The flight profile they were derived from:

| Quantity | Value |
|---|---|
| Burnout altitude | about 148 m AGL |
| Burnout speed | 77.6 m/s |
| Dynamic pressure at burnout | 3634 Pa |
| Dynamic pressure 5 s after burnout | about 350 Pa |
| Coast, burnout to apogee | about 7.3 s |

- `AIRFRAME_CANARD_LOCK_TRAVEL_S`: time the mechanical canard lock needs to clear after burnout before the servos may push against it.
- `AIRFRAME_ARM_MIN_ALTITUDE_M`: see [Altitude floor](#altitude-floor).
- `AIRFRAME_MAX_COAST_S`: forces FREE_FLIGHT to FALLING if the barometer never reports apogee. 20 s is a placeholder, not derived from a measured burnout-to-apogee time; it must exceed the real coast with margin.
- `AIRFRAME_LAUNCH_CONFIRM_*`: after this long in ACCELERATION, a launch is undone if the barometer climbed less than the baro threshold and the IMU-integrated climb is below the IMU threshold.
- `AIRFRAME_ROLL_PROFILE_MAX_RATE_RADS`, `AIRFRAME_ROLL_PROFILE_MAX_ACCEL_RADS2`: the trapezoid the 90 deg rotation follows; bounded by canard authority at the dynamic pressures of the early coast. The rate-loop clamp `rate_limit_rads` (120 deg/s) must stay above the profile rate.
- `AIRFRAME_ROLL_KP_ANGLE`, `AIRFRAME_ROLL_KP_RATE`, `AIRFRAME_ROLL_KI_RATE`: see [Roll gain tuning](#roll-gain-tuning). The integral time `kp_rate / ki_rate` is 1 s: slow next to the rate loop, fast next to the coast. Retuning `kp_rate` must carry `ki_rate` with it.
- `AIRFRAME_ROLL_RATE_INT_LIMIT_RAD`: must exceed the deflection that cancels the worst credible trim. On Joust 1, half a degree of aft-fin misalignment needs about 4 deg of canard to cancel.
- `AIRFRAME_QBAR_REF_PA`: the dynamic pressure the rate-loop gains were tuned at, here the burnout value.
- `AIRFRAME_QBAR_FLOOR_PA`: about 16 m/s, roughly 1.3 s before apogee. Below it the canards have about a twenty-fourth of their burnout authority and the gain-scale clamp has run out of room to compensate. See [Qbar floor](#qbar-floor).
- `AIRFRAME_MAX_DEFLECTION_RAD`: mechanical canard deflection limit.

## Control loop rate

`CONTROL_TICK_MS` in `Core/Inc/utils/control_rate.h` is the single statement of the loop rate. The flight task derives its tick period from it, the estimator and the roll controller default `dt_s` to it, and the host tests integrate their plants on it. Every rate-dependent count (flight-task loop intervals, tick-denominated debounces in `flight_sm` and `roll_ctrl`) goes through `SEC_TO_TICKS` / `HZ_TO_TICKS`; a bare tick literal would be silently rescaled by a rate change.

The period, not the rate, is primary. The loop wakes on `osDelayUntil` against a 1 kHz FreeRTOS tick, so only whole-millisecond periods are schedulable, and `dt` must be the period actually slept. A rate chosen independently could be one the scheduler rounds away, leaving `dt` describing a loop that does not exist.

## Arming

### Arming trigger

Burnout is the trigger, not altitude. The roll maneuver is only executable while there is dynamic pressure, which means the first seconds of the coast, so any delay in arming comes straight out of the usable window. `FLIGHT_STATE_FREE_FLIGHT` already requires debounced launch detection and debounced burnout detection, a far stronger statement about not being on the pad than any altitude threshold. After that, the canard lock needs `lock_travel_ticks` to clear.

Arming is one-way per flight. Dropping back to disarmed clears the whole controller state, which would discard a latched abort and capture a fresh reference, commanding another 90 deg from wherever the airframe then points. Leaving FREE_FLIGHT (apogee) disarms permanently; altitude is not consulted for disarming, because the arming point sits near the floor, exactly where barometer noise crosses it.

### Altitude floor

Altitude survives only as a ground interlock: without it, a state machine fooled on the ground could unlock and drive four servos against a locked mechanism, where each stalls at about 2.8 A. It is not a trigger and must not be tuned like one.

The input is raw barometric AGL on a vehicle near burnout speed, where port and boundary-layer effects read low and the sensor lags. A delayed arm costs dynamic-pressure budget, so the floor sits well under the burnout altitude minus the worst credible baro error, while staying far above anything the pad can produce.

## Flight state machine

### Baro velocity

`flight_sm` differences barometric altitude over a fixed minimum span (`baro_vel_min_span_s`), not over every publish interval, then low-passes the result with time constant `baro_vel_tau_s`; the per-sample filter coefficient is derived from the measured interval, so the response does not depend on the baro cadence.

Differencing altitude over a span S gives velocity noise sigma_h * sqrt(2) / S, so the span sets the SNR, and the low-pass recovers only sqrt(S / tau) of what a short span throws away. The MS5611 at OSR 4096 resolves about 1.2 Pa RMS (about 0.1 m); over 0.05 s with tau 0.34 s that leaves roughly 1 m/s of noise on vz, already the width of the -1.0 m/s apogee gate. Differencing across every publish would scale that by sqrt(0.05 / dt) and drown the gate; compensating with tau alone would need tau near 2 s, slower than the altitude-drop term the apogee gate is ANDed with. So the span is fixed and the publish rate is free to change.

The low-pass has no path back out of NaN, and `baro_vel_z` feeds the only exits from FREE_FLIGHT and FALLING, so non-finite samples and intervals are rejected before they reach it.

## Roll controller

### Roll gain tuning

The gains were tuned in the host simulator (`test/host/sim_roll.c`) against the previous airframe, with Monte Carlo dispersion of +-50 % on the roll-damping derivative C_lp, the canard roll-control derivative C_ld and the roll inertia I_xx.

They sit at the low edge of the passing region, not its middle. The simulator shows no degradation well past `kp_rate` 0.16, which does not mean those gains are safe: the model has no linkage backlash, no structural compliance and no measurement delay, so it cannot reveal an upper stability bound. That bound must come from bench testing, and every gain is provisional until it does. There is no flight data.

### No derivative term

The inner rate loop is PI. Differentiating the raw gyro at the loop rate multiplies sensor noise by 1/dt before the low-pass sees it, and the airframe supplies its own damping through C_lp; in simulation the phase lead bought nothing measurable and cost continuous servo travel through the hold. If bench testing shows the linkage slower than its datasheet implies, reconsider a D term on a filtered rate, not the raw one.

### Rate integrator

A steady aerodynamic trim torque otherwise leaves a standing rate error, and the DAMP->ROTATE settle gate turns that error into a pass/fail on the whole maneuver: a trim exceeding `kp_rate * settle_rate_rads` of equivalent deflection holds the sequencer in DAMP for the entire coast. On Joust 1 a quarter degree of aft-fin misalignment already exceeds that.

- The integrator sits outside the output low-pass: it is already the slow path, and filtering it only adds lag to the term that cancels the trim.
- Integration is conditional (anti-windup). The term exists only to cancel a standing trim. Entering free flight spinning, or recovering from a dropped gyro sample, presents an error tens of times the steady-state one; integrating it drives the accumulator into its clamp, and unwinding it afterwards throws the rate through zero and rings. Accumulation therefore stops while the command sits against the deflection limit and the error would push it further in.
- It is frozen during ROTATE: there the error is dominated by lag behind the moving profile, which is not a disturbance and reverses sign when the profile decelerates. The trim learned in DAMP stays valid across the maneuver.

### Gain scheduling on dynamic pressure

Canard authority is proportional to dynamic pressure qbar, which falls by more than an order of magnitude across the coast (3634 Pa at burnout to about 350 Pa five seconds later on Joust 1), so a fixed gain is right at one instant and wrong at every other. The rate loop is scaled by `qbar_ref / qbar`, holding the loop gain where it was tuned. Both P and I scale, so the integral time does not change with airspeed; the integrator clamp does not scale, since it is a deflection limit.

The scale clamp is required: the ratio diverges as qbar goes to zero, and an unbounded gain near apogee would chase gyro noise into the stops.

A non-positive qbar is the absence of a measurement, not a measurement of zero: scheduling off, scale 1, floor not applied, so a caller without airspeed degrades to the fixed-gain controller rather than to no controller.

### Qbar floor

Below `qbar_floor_pa` the proportional path is dropped and the canards hold only the integrator's trim correction. A standing trim moment and the canard authority opposing it are both proportional to qbar, so the deflection that cancels the trim is the same at every airspeed; going neutral would hand the airframe back to the disturbance for the rest of the coast (on a 2 deg trim, over ten degrees of drift off the target). The proportional path is what is worth dropping: its scheduled gain would spend the last seconds chasing gyro noise it has no authority to act on. The sequencer keeps running so the log still shows which phase ran out.

### Fault debounce and recovery

Sensor faults are overwhelmingly transient: one NaN, one dropped IMU frame, a momentary estimator re-convergence. At the control rate a single-sample latch would turn a 2 ms blip into permanent loss of the maneuver, so ABORT requires `abort_debounce_ticks` consecutive faulted ticks, still negligible against the maneuver duration for a genuine persistent fault.

A data-fault ABORT is not final: the coast is several times longer than the maneuver, so a blip that has passed is no reason to spend the rest of the flight damping. After `recover_debounce_ticks` of healthy data the sequencer resumes the same maneuver (same `q_ref`, same target): `update_theta` first accounts for the rotation made while the estimate was rejected, then the profile is re-aimed from the angle actually reached. `theta_meas` is what post-flight identification reads, and a silently short angle there is worse than a visibly wrong one.

Recovery is much slower than abort. With one shared threshold an estimator dropping out every few ticks walks the sequencer across ABORT and back indefinitely: each recovery restarts the profile from standstill, each abort brakes to zero, the maneuver makes no progress and the servos pay for all of it.

An over-limit roll rate is different in kind: past that threshold the airframe or the sign of the servo channels is wrong, neither of which heals, and a retry would re-enter the same divergence with a running start. It latches for the rest of the armed window. The overspeed flag is sticky across the whole fault run, because the excursion that trips it drives the gyro into full scale, so the run can end on an invalid sample rather than an over-limit one.

### Angle unwrap across faults

`update_theta` accumulates the twist about the roll axis across +-pi wraps. Callers must skip it on a rejected quaternion: NaN is absorbing there (it reaches `theta_meas` and `theta_prev_wrapped`, and within an armed window only the DAMP->ROTATE edge reassigns either), and `clampf` does not stop it because both comparisons are false on NaN.

Skipping freezes `theta_prev_wrapped` for the duration of the fault. Before the debounce expires the gap is at most `abort_debounce_ticks - 1` ticks; afterwards the sequencer is in ABORT until recovery, which is why the resume path unwraps explicitly before re-aiming the profile. The unwrap mistracks only if more than pi accumulates across the gap, and ABORT spends that whole window damping the rate toward zero.

## Attitude estimator

### Accel corrections are gated by flight phase

During powered flight and coast the accelerometer measures thrust and drag, not gravity. On the previous airframe coast drag reads a few tenths of a g pointing aft; levelling against it drives the estimate toward inverted, and the roll maneuver reads its angle from the same quaternion. A gravity-magnitude window cannot tell gravity from drag (a decelerating airframe can read about 1 g, and the margin moves with mass and drag coefficient), so the flight phase (`corrections_inhibited`) gates both the accel and mag corrections; the magnitude window remains only as a check against a faulty sensor.

### Attitude recovery after an IMU stall

Gravity seeding reads the accelerometer as gravity, so it is gated with the corrections: an IMU stall that clears mid-coast would otherwise rebuild the attitude from the drag vector and call it valid. With nothing to rebuild from in flight, validity instead returns on the quaternion already held. A stall stops the quaternion being advanced; it does not corrupt it, and the cost is the drift accumulated over the stall. The alternative is unrecoverable loss of attitude for the rest of the flight, which the roll controller cannot handle at all.

## Flight task

### Standby drift reset ordering

On the pad the flight task periodically resets the attitude estimator to discard accumulated drift. The reset runs after the log and telemetry blocks of the same tick. `STANDBY_RESET_INTERVAL` is a whole multiple of both cadences, so resetting first would make every reset-tick log row show zeroed velocity/position and an identity attitude, losing the one sample per period that records how much drift was discarded. The reset is safe at that point because `flight_sm` reads only `az_mps2` in STANDBY and the reset leaves `acc_original` untouched.

## SD logging

### SD logging LED codes

| LED | Meaning |
|---|---|
| LD2 on solid | task started, waiting on `f_mount` |
| LD2 off | `f_mount` returned |
| LD1 blinks N times, repeating | `f_mount` failed, N = FRESULT code |
| LD1 blinks 20 times, repeating | `f_open` failed |
| LD3 on solid | mounted, log file open |
| LD3 toggles | log synced to the card |
| LD2 on again | a write or sync failed (counted in `log_errors`) |

## IMU (IIM-42653)

### IMU UI filter bandwidth

The control loop samples the data registers at 500 Hz, so anything above 250 Hz aliases. With the 2nd-order UI filter at ODR 1 kHz (datasheet filter table):

| Setting | 3 dB corner | Noise bandwidth | Group delay |
|---|---|---|---|
| ODR/4 | 230.7 Hz (at Nyquist) | 223.7 Hz | 2.1 ms |
| ODR/8 | 117.5 Hz | 122.7 Hz | 3.2 ms |

ODR/8 is used for both gyro and accel. It adds 1.1 ms of phase lag in the roll loop on top of the 2 ms sample-and-hold; any rate-loop retune must budget for it.

## Servos (SRT T75 HV)

### Servo slew limit

`slew_us_per_ms = 4.17` comes from the datasheet speed of 0.16 s/60 deg at 6 V (the lowest rated supply), i.e. 375 deg/s, times 11.111 us/deg. Two caveats:

- The datasheet figure is no-load. Under hinge moment the horn moves slower, so the limiter stops binding.
- `servo_t75_slew_step` rounds into `uint16_t` on every call. At a 2 ms tick the 8.34 us step truncates to 8 us, so the achieved rate is 4.00 us/ms (360 deg/s), not 4.17.

## Host tests and simulator

### Roll simulator plant model

`test/host/sim_roll.c` is a 1-DOF roll model of the coast phase, linked against the production `roll_ctrl.c`. Its default parameters describe the previous airframe (Joust 1 on a K353 motor): burnout at 3.04 s, 148 m and 77.6 m/s; apogee at 10.32 s. The run covers burnout to apogee (7.28 s).

- **Aero coefficients.** `C_ld` and `C_lp` come from Barrowman, which models each fin set in isolation. The canard wake over the aft fins opposes the commanded roll and is in neither number, so `C_ld` is an upper bound. The Monte Carlo mode perturbs `C_ld`, `C_lp` and `I_xx` by +-50% and the trim by +-2 deg.
- **Airspeed.** Coast deceleration is gravity plus quadratic drag (`k_drag`) fitted to the trajectory. Dynamic pressure, and with it the plant gain, falls by more than an order of magnitude between burnout and apogee.
- **Trim.** A standing roll moment from fin misalignment, expressed as the canard deflection that cancels it; 0.25 deg of aft-fin misalignment is about 2 deg of canard on this airframe. It is applied as a moment on the airframe, independent of canard state.
- **Servo.** The electronics slew-limit an internal setpoint and the horn follows it with first-order lag. Slew is 375 deg/s at the horn (T75 at 6 V), assuming a 1:1 linkage.
- **Hold window.** The hold error is measured from 4.0 s to 5.5 s: after the maneuver completes and before dynamic pressure falls below the controller's floor (about 6 s). Below the floor the airframe coasts at whatever angle it reached.
- **Not modelled.** Linkage backlash, structural compliance and measurement delay. The simulator therefore shows no upper stability bound on the gains.
- **Baro.** `roll_ctrl` has no baro input; a baro failure acts through the arming gate, covered in `test/host/test_arming.c` and `test/host/test_flight_sm.c`.
