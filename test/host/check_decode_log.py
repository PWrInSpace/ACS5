"""Decode the sample written by test_log_record and check it field by field."""
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "tools"))
import decode_log  # noqa: E402

def _drain(gen):
    """Consume a decode() generator; return (rows, final counters)."""
    rows = []
    while True:
        try:
            rows.append(next(gen))
        except StopIteration as done:
            return rows, done.value


buf = pathlib.Path(sys.argv[1]).read_bytes()
gen = decode_log.decode(buf)
rows = []
while True:
    try:
        rows.append(dict(zip(decode_log.COLUMNS, next(gen))))
    except StopIteration as done:
        records, gaps, corrupt, tail = done.value
        break

assert len(decode_log.COLUMNS) == len(set(decode_log.COLUMNS))
assert (records, gaps, corrupt, tail) == (3, 1, 0, 1024), (records, gaps, corrupt, tail)
assert [r["seq"] for r in rows] == [0, 1, 3]
assert [r["pos_z"] for r in rows] == [100.0, 101.0, 102.0]
assert all(r["state"] == "FREE_FLT" and r["gnss_fix"] == 1 and r["rate_fresh"] == 1 for r in rows)
assert all(abs(r["eul_z_deg"] - math.degrees(0.5)) < 1e-4 for r in rows)
assert rows[0]["gnss_lat_1e7"] == 521234567 and rows[0]["gyro_bias_n"] == 12345
assert rows[2]["log_dropped"] == 1 and rows[2]["tick_ms"] == 1016
# A preallocated tail holds whatever the card held; a damaged record mid-file is corrupt.
rec = decode_log.SIZE
dirty = bytearray(buf[:3 * rec]) + bytes((i * 37 + 11) & 0xFF for i in range(4096))
_, (records, gaps, corrupt, tail) = _drain(decode_log.decode(bytes(dirty)))
assert (records, corrupt, tail) == (3, 0, 4096), (records, corrupt, tail)
damaged = bytearray(buf[:3 * rec])
damaged[rec + 50] ^= 0xFF
_, (records, gaps, corrupt, tail) = _drain(decode_log.decode(bytes(damaged)))
assert (records, corrupt, tail) == (2, rec, 0), (records, corrupt, tail)
print("decode_log: OK")
