"""Report Jetson replies and H7's first valid PD use from an external-action run."""

import argparse
import collections
import math
import struct
from pathlib import Path

from analyze_shadow import frames


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("records", type=Path)
    args = parser.parse_args()
    counts = collections.Counter()
    used = {}
    for frame in frames(args.records):
        kind = frame[5]
        counts[kind] += 1
        if kind == 11 and len(frame) == 52:
            contract, returned, consumed, rejected = struct.unpack_from("<4I", frame, 32)
            if contract == 3000201:
                used[frame[8:32]] = returned, consumed, rejected
    print(f"pd_telemetry={counts[5]} input_first={counts[6]} "
          f"input_second={counts[7]} pd_consumed={len(used)}")
    for column, name in enumerate(("input_to_publish_us", "input_to_first_valid_pd_us")):
        values = sorted(row[column] for row in used.values())
        if values:
            print(f"{name}: p50={values[math.ceil(len(values)*.5)-1]} "
                  f"p99={values[math.ceil(len(values)*.99)-1]} max={values[-1]}")
    if used:
        print(f"h7_rejected_reported_max={max(row[2] for row in used.values())}")
    return int(not used)


if __name__ == "__main__":
    raise SystemExit(main())
