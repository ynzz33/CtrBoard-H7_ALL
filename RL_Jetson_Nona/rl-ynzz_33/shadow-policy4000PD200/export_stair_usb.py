"""Export H7 USB type-5 telemetry from the Jetson record command's records.bin."""
import argparse
import csv
import struct
import zlib
from pathlib import Path

LOG_HEADER = struct.Struct("<IIQ")
FRAME_HEADER = struct.Struct("<4sBBHQIIQ")
PAYLOAD = struct.Struct("<5I55f")
FRAME_SIZE = FRAME_HEADER.size + PAYLOAD.size + 4
SWING_PAYLOAD = struct.Struct("<6I19f")
SWING_LEGACY_PAYLOAD = struct.Struct("<6I18f")
STAIR_PAYLOAD = struct.Struct("<6I45f")
HIPS = ("lf", "lr", "rf", "rr")
STAIR_COLUMNS = [
    "h7_us", "sequence", "trial", "flags", "board_drops", "fault", "stair_phase",
    "stair_result", "stair_exit_reason", "control_condition",
    *(f"quat_{i}" for i in "wxyz"),
    "pitch_rad", "roll_rad", "yaw_rad",
    *(f"gyro_{axis}_rad_s" for axis in "xyz"),
    *(f"acc_{axis}_g" for axis in "xyz"),
    *(f"{field}_{joint}" for field in ("angle_rad", "velocity_rad_s", "feedback_nm") for joint in HIPS),
    "wheel_velocity_l_rad_s", "wheel_velocity_r_rad_s",
    *(f"request_nm_{joint}" for joint in (*HIPS, "lw", "rw")),
    *(f"{side}_{field}" for side in ("left", "right") for field in
      ("length_m", "angle_rad", "length_rate_m_s", "angle_rate_rad_s")),
    "hip_to_wheel_height_m", "cmd_vx_m_s", "cmd_yaw_rad_s", "cmd_leg_length_m",
]
SWING_COLUMNS = [
    "h7_us", "sequence", "trial", "flags", "board_drops", "trial_config",
    "fault", "front_age_us", "rear_age_us", "swing_nm", "axial_n",
    *(f"{field}_{joint}" for field in
      ("angle_rad", "velocity_rad_s", "feedback_nm", "mapped_nm", "prelimit_nm", "submitted_nm")
      for joint in ("front", "rear")),
    "length_m", "length_rate_m_s", "swing_rad", "swing_rate_rad_s", "swing_ref_rad_s",
]
MOTORS = ("lf", "lr", "rf", "rr", "lw", "rw")
FLAGS = (
    "rl_selected", "rl_engaged", "action_ready", "pd_valid", "dispatched",
    "obs_valid", "warmup", "drive_permit", "dm_submitted", "wheel_submitted",
)
COLUMNS = [
    "h7_us", "sequence", "reset_count", "flags", "fault", "board_drops",
    "obs_sequence", "obs_publish_ms", *FLAGS,
    *(f"obs_{i}" for i in range(25)),
    *(f"{field}_{motor}" for field in
      ("target", "current", "velocity_rad_s", "pd_model_nm", "request_firmware_nm")
      for motor in MOTORS),
]


def rx_chunks(source):
    while header := source.read(LOG_HEADER.size):
        if len(header) != LOG_HEADER.size:
            raise ValueError("truncated log header")
        kind, size, _ = LOG_HEADER.unpack(header)
        data = source.read(size)
        if len(data) != size:
            raise ValueError("truncated log record")
        if kind == 1:
            yield data


def export(source, writer, swing=False, stair=False):
    buffer = bytearray()
    count = errors = missing = resets = 0
    first_us = last_us = last_sequence = None
    writer.writerow(STAIR_COLUMNS if stair else SWING_COLUMNS if swing else COLUMNS)
    for chunk in rx_chunks(source):
        buffer.extend(chunk)
        while len(buffer) >= FRAME_HEADER.size:
            start = buffer.find(b"JRL2")
            if start == -1:
                del buffer[:-3]
                break
            del buffer[:start]
            if len(buffer) < FRAME_HEADER.size:
                break
            _, version, kind, size, _, reset, sequence, timestamp = FRAME_HEADER.unpack_from(buffer)
            if version != 2 or size > 348:
                errors += 1
                del buffer[0]
                continue
            length = FRAME_HEADER.size + size + 4
            if len(buffer) < length:
                break
            expected, = struct.unpack_from("<I", buffer, length - 4)
            if zlib.crc32(buffer[:length - 4]) != expected:
                errors += 1
                del buffer[0]
                continue
            if stair and kind == 14 and size == STAIR_PAYLOAD.size:
                flags, drops, fault, phase, event, condition, *values = STAIR_PAYLOAD.unpack_from(buffer, FRAME_HEADER.size)
                if last_us is not None:
                    if timestamp <= last_us:
                        resets += 1
                    else:
                        delta = (sequence - last_sequence) & 0xffffffff
                        if 1 < delta < 0x80000000:
                            missing += delta - 1
                if first_us is None:
                    first_us = timestamp
                last_us, last_sequence = timestamp, sequence
                writer.writerow([timestamp, sequence, reset, flags, drops, fault, phase, event & 0xff, event >> 8, condition, *values])
                count += 1
            elif swing and kind == 13 and size in (SWING_PAYLOAD.size, SWING_LEGACY_PAYLOAD.size):
                if size == SWING_PAYLOAD.size:
                    values = SWING_PAYLOAD.unpack_from(buffer, FRAME_HEADER.size)
                else:
                    values = (*SWING_LEGACY_PAYLOAD.unpack_from(buffer, FRAME_HEADER.size), "")
                if last_sequence is not None and timestamp > last_us:
                    missing += max(0, sequence - last_sequence - 1)
                if first_us is None:
                    first_us = timestamp
                if last_us is not None and timestamp <= last_us:
                    resets += 1
                last_us, last_sequence = timestamp, sequence
                writer.writerow([timestamp, sequence, reset, *values])
                count += 1
            elif not swing and not stair and kind == 5 and length == FRAME_SIZE:
                flags, fault, drops, obs_seq, obs_ms, *values = PAYLOAD.unpack_from(
                    buffer, FRAME_HEADER.size
                )
                if last_us is not None:
                    if timestamp <= last_us:
                        resets += 1
                    else:
                        delta = (sequence - last_sequence) & 0xffffffff
                        if 1 < delta < 0x80000000:
                            missing += delta - 1
                if first_us is None:
                    first_us = timestamp
                last_us, last_sequence = timestamp, sequence
                writer.writerow([
                    timestamp, sequence, reset, flags, fault, drops, obs_seq, obs_ms,
                    *(int(bool(flags & (1 << i))) for i in range(len(FLAGS))), *values,
                ])
                count += 1
            del buffer[:length]
    print(f"frames={count} crc_or_header_errors={errors} missing={missing} "
          f"clock_resets={resets} trailing_bytes={len(buffer)}")
    if count > 1 and not resets and last_us > first_us:
        print(f"rate_hz={(count - 1) * 1e6 / (last_us - first_us):.2f}")
    return count


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("records", type=Path)
    parser.add_argument("csv", type=Path)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--swing", action="store_true", help="export type-13 joint identification samples")
    mode.add_argument("--stair", action="store_true", help="export type-14 stair samples")
    args = parser.parse_args()
    with args.records.open("rb") as source, args.csv.open("x", newline="") as target:
        if not export(source, csv.writer(target), args.swing, args.stair):
            parser.exit(1, f"No type-{14 if args.stair else 13 if args.swing else 5} telemetry frames found.\n")


if __name__ == "__main__":
    main()
