# Flight log format

The SD task writes `LOG_NNN.BIN` (next free number, 001–999) to the card root.
Decode it on a PC:

```bash
python3 tools/decode_log.py LOG_001.BIN -o LOG_001.csv
```

The decoder prints the record count, the records missing by sequence number and
any corrupt bytes it skipped.

## File

- A sequence of fixed-size records, 204 bytes each, little-endian, no file header.
- The file is preallocated (32 MB, contiguous) when opened, so its size on the card
  is not the amount logged: the decoder stops at the first never-written region
  (all `0x00` or all `0xFF`).
- A corrupt record is skipped up to the next `magic`.

## Rates

| Flight state | One record every |
|---|---|
| STANDBY | 240 ms |
| any other, until `mission_complete` | 8 ms |

Records travel through a 64-record queue. When the card stalls longer than the
queue covers, records are dropped: `seq` skips and `log_dropped` counts them.

## Record (`log_record_t`, version 1)

Defined in `Core/Inc/log/log_record.h`; `tools/decode_log.py` mirrors it.

| Offset | Type | Field | Unit / meaning |
|---|---|---|---|
| 0 | u32 | `magic` | `0x474C3541` |
| 4 | u16 | `version` | 1 |
| 6 | u16 | `size` | 204 |
| 8 | u32 | `seq` | consecutive per record published, dropped ones included |
| 12 | u32 | `tick_ms` | RTOS tick |
| 16 | u8 | `flight_state` | 0 STANDBY, 1 ACCELERATION, 2 FREE_FLIGHT, 3 FALLING, 4 RECOVERY |
| 17 | u8 | `ctrl_phase` | `roll_ctrl_phase_t` |
| 18 | u8 | `ctrl_arm` | `ctrl_arm_state_t` |
| 19 | u8 | `flags` | bit 0 GNSS fix, 1 accel clipped, 2–4 gyro X/Y/Z clipped (chip axes), 5 rate measured this tick |
| 20 | f32[3] | `pos_m` | world, pad origin, m |
| 32 | f32[3] | `vel_mps` | m/s |
| 44 | f32[3] | `acc_mps2` | world, gravity removed, m/s² |
| 56 | f32[3] | `eul_rad` | ZYX Euler, rad; [2] is about body Z, the airframe roll axis |
| 68 | f32 | `roll_twist_rad` | twist of the attitude about body Z, rad |
| 72 | f32[3] | `ang_vel_rads` | body rates, gyro bias removed, rad/s |
| 84 | f32[3] | `accel_raw_mps2` | chip axes, before the board remap |
| 96 | f32[3] | `gyro_raw_rads` | chip axes |
| 108 | f32[3] | `mag_raw_ut` | chip axes, µT |
| 120 | f32 | `baro_alt_m` | above the pad datum |
| 124 | f32 | `baro_press_pa` | Pa |
| 128 | f32 | `baro_temp_c` | die temperature, °C |
| 132 | f32 | `ctrl_theta_sp_rad` | roll angle setpoint |
| 136 | f32 | `ctrl_rate_sp_rads` | roll rate setpoint |
| 140 | f32 | `ctrl_theta_meas_rad` | unwrapped maneuver angle |
| 144 | f32 | `ctrl_canard_cmd_rad` | commanded deflection, all four fins |
| 148 | u16 | `ctrl_canard_us` | fin 0 pulse after the servo slew limit |
| 150 | u8 | `gnss_num_sv` | satellites used |
| 151 | u8 | `false_launch_count` | launches undone by the baro/IMU check (saturates at 255) |
| 152 | f32[3] | `gnss_ned_m` | from the pad datum, Down positive |
| 164 | f32[3] | `gnss_vel_ned_mps` | Down positive |
| 176 | i32 | `gnss_lat_1e7` | deg × 1e7 |
| 180 | i32 | `gnss_lon_1e7` | deg × 1e7 |
| 184 | i32 | `gnss_h_msl_mm` | mm |
| 188 | u32 | `gyro_bias_n` | pad samples behind the gyro bias |
| 192 | u32 | `log_dropped` | records lost to a full queue so far |
| 196 | u32 | `log_errors` | SD write/sync failures so far |
| 200 | u32 | `crc32` | CRC-32 (zlib) over bytes 0–199 |

The CSV from the decoder converts angles to degrees (`eul_*_deg`, `roll_twist_deg`,
`theta_*_deg`, `rate_sp_dps`, `canard_cmd_deg`); other columns keep the units above.
