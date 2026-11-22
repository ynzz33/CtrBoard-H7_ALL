"""Export the actual model input/action records (kind 15) from dual-model logs."""
import argparse
import csv
import struct
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("record", type=Path)
parser.add_argument("output", type=Path)
args = parser.parse_args()
phases = ["walk", "armed", "takeoff", "flight", "landing", "fault"]
with args.record.open("rb") as source, args.output.open("x", newline="", encoding="utf-8-sig") as target:
    writer = csv.writer(target)
    writer.writerow(["jetson_time_us", "sequence", "phase", "model", "flags", "board_time_us",
                     *[f"obs_{i}" for i in range(25)],
                     *[f"history_{i}" for i in range(125)],
                     *[f"action_{i}" for i in range(6)], "support_force_n", "mean_leg_length_m"])
    while header := source.read(16):
        if len(header) != 16:
            raise ValueError("Truncated record header")
        kind, size, time_us = struct.unpack("<IIQ", header)
        if size > 4096:
            raise ValueError("Invalid record length")
        payload = source.read(size)
        if len(payload) != size:
            raise ValueError("Truncated record body")
        if kind != 15:
            continue
        if size != 656:
            raise ValueError("Unknown dual-model record layout")
        sequence, phase, model, flags, board_us = struct.unpack_from("<IIIIQ", payload)
        values = struct.unpack_from("<158f", payload, 24)
        writer.writerow([time_us, sequence, phases[phase], model, flags, board_us, *values])
