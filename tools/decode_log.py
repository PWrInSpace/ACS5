#!/usr/bin/env python3
"""Decode an ACS5 binary flight log (LOG_NNN.BIN) into CSV.

Record layout: Core/Inc/log/log_record.h (version 1, 204 bytes, little-endian),
described in docs/log_format.md.
"""
import argparse
import csv
import math
import struct
import sys
import zlib

MAGIC = 0x474C3541
VERSION = 1
RECORD = struct.Struct("<IHHII4B32fH2B6f3i3I I")
SIZE = 204
assert RECORD.size == SIZE


COLUMNS = (
    ["seq", "tick_ms", "state", "ctrl_phase", "ctrl_arm",
     "gnss_fix", "accel_clipped", "gyro_clipped_x", "gyro_clipped_y", "gyro_clipped_z", "rate_fresh",
     "pos_x", "pos_y", "pos_z", "vel_x", "vel_y", "vel_z", "acc_x", "acc_y", "acc_z",
     "eul_x_deg", "eul_y_deg", "eul_z_deg", "roll_twist_deg",
     "ang_vel_x", "ang_vel_y", "ang_vel_z",
     "accel_raw_x", "accel_raw_y", "accel_raw_z",
     "gyro_raw_x", "gyro_raw_y", "gyro_raw_z",
     "mag_raw_x", "mag_raw_y", "mag_raw_z",
     "baro_alt", "baro_press", "baro_temp",
     "theta_sp_deg", "rate_sp_dps", "theta_meas_deg", "canard_cmd_deg", "canard_us",
     "gnss_sv", "false_launches",
     "gnss_n_m", "gnss_e_m", "gnss_d_m", "gnss_vn", "gnss_ve", "gnss_vd",
     "gnss_lat_1e7", "gnss_lon_1e7", "gnss_h_msl_mm",
     "gyro_bias_n", "log_dropped", "log_errors"]
)

STATES = {0: "STANDBY", 1: "ACCEL", 2: "FREE_FLT", 3: "FALLING", 4: "RECOVERY"}


def valid(buf, off):
    if off + SIZE > len(buf):
        return None
    f = RECORD.unpack_from(buf, off)
    if f[0] != MAGIC or f[1] != VERSION or f[2] != SIZE:
        return None
    if zlib.crc32(buf[off:off + SIZE - 4]) != f[-1]:
        return None
    return f


def row(f):
    seq, tick, state, phase, arm, flags = f[3], f[4], f[5], f[6], f[7], f[8]
    fl = f[9:41]
    deg = [math.degrees(x) for x in fl[9:13]]
    ctrl = fl[28:32]
    rest = f[41:]
    canard_us, sv, false_launches = rest[0], rest[1], rest[2]
    gnss = rest[3:9]
    lat, lon, hmsl = rest[9:12]
    bias_n, dropped, errors = rest[12:15]
    return ([seq, tick, STATES.get(state, state), phase, arm]
            + [(flags >> b) & 1 for b in range(6)]
            + list(fl[0:9]) + deg + list(fl[13:28])
            + [math.degrees(ctrl[0]), math.degrees(ctrl[1]), math.degrees(ctrl[2]),
               math.degrees(ctrl[3]), canard_us, sv, false_launches]
            + list(gnss) + [lat, lon, hmsl, bias_n, dropped, errors])


def next_valid(buf, off):
    """Offset of the next valid record at or after off, or None."""
    magic = struct.pack("<I", MAGIC)
    while True:
        off = buf.find(magic, off)
        if off < 0 or off + SIZE > len(buf):
            return None
        if valid(buf, off) is not None:
            return off
        off += 1


def decode(buf):
    """Yield CSV rows; the StopIteration value is (records, seq_gaps, corrupt_bytes, tail_bytes).

    Bytes between valid records are corrupt. Bytes after the last valid record are
    the preallocated, never-written tail: f_expand leaves whatever the card held.
    """
    off, records, gaps, corrupt, prev_seq = 0, 0, 0, 0, None
    while True:
        f = valid(buf, off)
        if f is None:
            nxt = next_valid(buf, off + 1)
            if nxt is None:
                break
            corrupt += nxt - off
            off = nxt
            continue
        if prev_seq is not None and f[3] != prev_seq + 1:
            gaps += f[3] - prev_seq - 1
        prev_seq = f[3]
        records += 1
        yield row(f)
        off += SIZE
    return records, gaps, corrupt, len(buf) - off


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("log", help="LOG_NNN.BIN from the SD card")
    ap.add_argument("-o", "--output", help="CSV path (default: stdout)")
    args = ap.parse_args(argv)

    with open(args.log, "rb") as fh:
        buf = fh.read()
    out = open(args.output, "w", newline="") if args.output else sys.stdout
    w = csv.writer(out)
    w.writerow(COLUMNS)
    gen = decode(buf)
    while True:
        try:
            w.writerow(next(gen))
        except StopIteration as done:
            records, gaps, corrupt, tail = done.value
            break
    if out is not sys.stdout:
        out.close()
    print(f"{records} records, {gaps} missing by seq, {corrupt} corrupt bytes skipped, "
          f"{tail} unwritten bytes after the last record", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
