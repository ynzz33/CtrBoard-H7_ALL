"""Read Jetson shadow logs without additional dependencies."""
import argparse
import math
from pathlib import Path
import struct
import zlib

def frames(path):
    buffer = bytearray()
    with path.open('rb') as log:
        while header := log.read(16):
            kind, size, _ = struct.unpack('<IIQ', header)
            data = log.read(size)
            if len(data) != size:
                raise ValueError('truncated log')
            if kind != 1:
                continue
            buffer.extend(data)
            while len(buffer) >= 8:
                pos = buffer.find(b'JRL2')
                if pos < 0:
                    del buffer[:-3]
                    break
                del buffer[:pos]
                if len(buffer) < 8:
                    break
                length = 36 + int.from_bytes(buffer[6:8], 'little')
                if buffer[4] != 2 or length > 384:
                    del buffer[0]
                    continue
                if len(buffer) < length:
                    break
                frame = bytes(buffer[:length])
                if zlib.crc32(frame[:-4]) != int.from_bytes(frame[-4:], 'little'):
                    del buffer[0]
                    continue
                del buffer[:length]
                yield frame

def stats(name, data):
    if not data:
        return
    data = sorted(data)
    def quantile(p):
        return data[math.ceil(len(data) * p) - 1]
    print(f'{name}: n={len(data)} p50={quantile(.5)} p95={quantile(.95)} '
          f'p99={quantile(.99)} max={data[-1]}')

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('records', type=Path)
    args = parser.parse_args()
    requests, matched = set(), set()
    latency, forward, pd = [], [], []
    failed = 0
    max_error = 0.0
    counters = [0] * 4
    previous = None
    for frame in frames(args.records):
        kind = frame[5]
        identity = frame[8:32]
        if kind == 5 and len(frame) == 276:
            _, seq, us = struct.unpack_from('<IIQ', frame, 16)
            if previous and (seq - previous[0]) & 0xffffffff == 1 and us > previous[1]:
                pd.append(us - previous[1])
            previous = seq, us
        if kind == 7 and len(frame) == 340:
            requests.add(identity)
        if kind != 9 or len(frame) != 112 or identity in matched:
            continue
        contract, *values = struct.unpack_from('<I6fI6f5I', frame, 32)
        if contract != 3500301:
            continue
        matched.add(identity)
        remote, local = values[:6], values[7:13]
        forward.append(values[6])
        latency.append(values[13])
        counters = [max(a, b) for a, b in zip(counters, values[14:18])]
        for a, b in zip(remote, local):
            error = abs(a - b)
            if not math.isfinite(error):
                failed += 1
                continue
            max_error = max(max_error, error)
            failed += error > 1e-4 + 1e-4 * abs(b)
    print(f'input_second_frames={len(requests)} matched={len(matched)} '
          f'unmatched={len(requests - matched)} failed_components={failed} '
          f'max_abs_error={max_error:.8g}')
    print('reported_counter_max(snapshot_drops,rx_drops,rx_parser_errors,rejected)=', counters)
    stats('h7_roundtrip_us', latency)
    stats('jetson_forward_us', forward)
    stats('consecutive_pd_sample_interval_us', pd)
    print('roundtrip_ge_10000_us=', sum(us >= 10000 for us in latency))
    return int(not matched or failed != 0)

if __name__ == '__main__':
    raise SystemExit(main())
