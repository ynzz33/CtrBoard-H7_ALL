#!/usr/bin/env python3
"""Decode H7LAT001 timing sideband from a hashed recording; never opens a port.

CAN completion times are IRQ/poll observation upper bounds. Unknown source ages,
unmatched pairs, reset boundaries, packet loss and unfinished CAN stay explicit.
"""
from __future__ import annotations
import argparse
import binascii
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path
import struct

import numpy as np
import timing_session as mirror

MAGIC = b'H7LAT001'
HEADER = struct.Struct('<8s4H6I')
RECORD = struct.Struct('<IHBBQQII8I')
KINDS = {1: 'sample', 2: 'policy', 3: 'pd', 4: 'actuation', 5: 'can'}
assert RECORD.size == 64


class LatencyParser:
    def __init__(self):
        self.buffer = bytearray()
        self.rejected = 0

    def feed(self, chunk):
        self.buffer.extend(chunk)
        packets = []
        while True:
            start = self.buffer.find(MAGIC)
            if start < 0:
                del self.buffer[:max(0, len(self.buffer) - 7)]
                break
            del self.buffer[:start]
            if len(self.buffer) < HEADER.size:
                break
            h = HEADER.unpack_from(self.buffer)
            if (h[1:3] != (1, 64) or h[3] > 128 or h[4] or h[8] < h[3]
                    or h[8] > 1024 or h[9] > 1024 or h[10] > 96):
                self.rejected += 1
                del self.buffer[0]
                continue
            length = 40 + h[3] * 64 + 2
            if len(self.buffer) < length:
                break
            packet = bytes(self.buffer[:length])
            if binascii.crc_hqx(packet[:-2], 0xffff) != int.from_bytes(packet[-2:], 'little'):
                self.rejected += 1
                del self.buffer[0]
                continue
            records = [RECORD.unpack_from(packet, 40 + i * 64) for i in range(h[3])]
            for r in records:
                if r[1] not in KINDS or r[5] < r[4]:
                    raise ValueError('unsupported timing kind or invalid interval')
                if r[1] == 5 and (r[2] not in (1, 2, 3) or (r[3] & 0x7f) not in (1, 2, 3, 4)):
                    raise ValueError('unsupported CAN timing result')
            packets.append(dict(sequence=h[5], produced=h[6], dropped=h[7], queued=h[8],
                                high_water=h[9], pending_can=h[10], records=records))
            del self.buffer[:length]
        return packets


def write_csv(path, rows):
    fields = list(dict.fromkeys(k for row in rows for k in row))
    with path.open('w') as output:
        writer = csv.DictWriter(output, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def decode_session(session, out):
    meta, _, _ = mirror.load_session(session)
    parser = LatencyParser()
    rows, packets, boundaries = [], [], []
    connection = -1
    span = 0
    last_record = last_packet = None
    partial = []
    for event_number, event in enumerate(mirror.iter_session_events(session)):
        if event['type'] != 'data':
            if parser.buffer.startswith(MAGIC) or MAGIC.startswith(parser.buffer):
                if parser.buffer:
                    partial.append(len(parser.buffer))
            parser.buffer.clear()
            if event['type'] == 'connected':
                connection += 1
                span += 1
                last_record = last_packet = None
            continue
        rejected = parser.rejected
        for packet in parser.feed(bytes.fromhex(event['data_hex'])):
            if last_packet and ((packet['sequence'] - last_packet['sequence']) % 2**32 != 1
                                or packet['dropped'] != last_packet['dropped']):
                span += 1
                boundaries.append(dict(event=event_number, reason='packet_sequence_or_drop'))
            if parser.rejected != rejected:
                span += 1
            packets.append({k: v for k, v in packet.items() if k != 'records'} | dict(connection=connection))
            for r in packet['records']:
                if last_record is not None and (r[0] - last_record) % 2**32 != 1:
                    span += 1
                    boundaries.append(dict(event=event_number, reason='record_sequence', after=last_record, before=r[0]))
                row = dict(connection=connection, span=span, event=event_number, receive_ns=event['t_ns'],
                           sequence=r[0], kind=KINDS[r[1]], source=r[2], flags=r[3],
                           begin_ns=r[4], end_ns=r[5], correlation=r[6], reset_count=r[7])
                row.update({f'value_{i}': v for i, v in enumerate(r[8:])})
                rows.append(row)
                last_record = r[0]
            last_packet = packet
    if not packets:
        raise ValueError('no H7LAT001 timing records: firmware does not contain the new probes')
    # Reset count and connection are part of every cross-stage identity.
    samples = {(r['connection'], r['correlation']): r for r in rows if r['kind'] == 'sample'}
    policies = {(r['connection'], r['reset_count'], r['correlation']): r
                for r in rows if r['kind'] == 'policy' and r['flags'] == 4}
    dispatches = {(r['connection'], r['reset_count'], r['value_2']): r
                  for r in rows if r['kind'] == 'actuation' and r['flags'] & 1}
    for row in rows:
        context = None
        if row['kind'] == 'actuation':
            context = row
        elif row['kind'] == 'pd':
            context = dispatches.get((row['connection'], row['reset_count'], row['correlation']))
        elif row['kind'] == 'can' and row['flags'] & 128:
            context = dispatches.get((row['connection'], row['reset_count'], row['value_6']))
        if context:
            row.update(strategy=context['value_4'], fault=context['value_5'], phase=context['value_6'])
        elif row['kind'] == 'policy':
            row.update(strategy=row['value_5'], fault=-1, phase=row['value_6'])
        else:
            row.update(strategy=-1, fault=-1, phase=-1)
    previous = {}
    metrics = []
    missing = Counter()

    def metric(row, name, value):
        metrics.append(dict(sequence=row['sequence'], kind=row['kind'], connection=row['connection'],
                            span=row['span'], reset_count=row['reset_count'], source=row['source'],
                            can_id=row['value_0'] if row['kind'] == 'can' else '',
                            strategy=row['strategy'], fault=row['fault'], phase=row['phase'],
                            result=row['flags'], metric=name, value_us=float(value),
                            begin_ns=row['begin_ns'], event=row['event']))

    for row in rows:
        kind = row['kind']
        if kind != 'can' or (row['flags'] & 0x7f) in (1, 2):
            metric(row, 'duration_observed_us', (row['end_ns'] - row['begin_ns']) / 1000)
        if kind == 'sample':
            for i, name in enumerate(('imu', 'lf0', 'lf00', 'rf0', 'rf00', 'lfwheel', 'rfwheel', 'rc')):
                if row[f'value_{i}'] != 0xffffffff:
                    metric(row, name + '_age_at_cache_us', row[f'value_{i}'])
        elif kind == 'policy':
            if row['flags']:
                metric(row, 'observation_build_us', row['value_2'])
            if row['value_3'] != 0xffffffff:
                metric(row, 'network_call_us', row['value_3'])
            sample = samples.get((row['connection'], row['value_0']))
            read_ns = row['begin_ns'] + row['value_1']
            if (sample and sample['span'] == row['span'] and sample['reset_count'] == row['reset_count']
                    and sample['end_ns'] <= read_ns and row['flags'] != 0 and row['value_1'] != 0xffffffff):
                metric(row, 'cache_to_policy_read_us', (read_ns - sample['end_ns']) / 1000)
                if row['flags'] == 4:
                    metric(row, 'cache_to_action_publish_us', (row['end_ns'] - sample['end_ns']) / 1000)
                for i, name in enumerate(('imu', 'lf0', 'lf00', 'rf0', 'rf00', 'lfwheel', 'rfwheel')):
                    if sample[f'value_{i}'] != 0xffffffff:
                        metric(row, name + '_age_at_policy_read_us', sample[f'value_{i}'] + (read_ns - sample['end_ns']) / 1000)
            elif row['flags']:
                missing['policy_sample_pair'] += 1
        elif kind == 'pd':
            if row['flags'] & 1:
                if row['value_1'] != 0xffffffff:
                    metric(row, 'action_age_at_pd_us', row['value_1'])
                for i, name in enumerate(mirror.MOTORS):
                    if row[f'value_{i + 2}'] != 0xffffffff:
                        metric(row, name + '_age_at_pd_us', row[f'value_{i + 2}'])
                policy = policies.get((row['connection'], row['reset_count'], row['value_0']))
                if (policy and policy['span'] == row['span'] and policy['end_ns'] <= row['begin_ns']):
                    if row['flags'] & 2:
                        metric(row, 'publish_to_first_pd_use_us', (row['begin_ns'] - policy['end_ns']) / 1000)
                else:
                    missing['pd_policy_pair'] += 1
        elif kind == 'can':
            returned = row['value_2'] | row['value_3'] << 32
            if not row['begin_ns'] <= returned <= row['end_ns']:
                raise ValueError('CAN HAL return not inside recorded interval')
            metric(row, 'enqueue_call_us', (returned - row['begin_ns']) / 1000)
            if (row['flags'] & 0x7f) in (1, 2):
                metric(row, 'hal_return_to_completion_observation_us', (row['end_ns'] - returned) / 1000)
        if kind in ('pd', 'actuation', 'policy'):
            key = row['connection'], row['span'], row['reset_count'], kind
            prev = previous.get(key)
            if prev and row['begin_ns'] > prev['begin_ns']:
                same_state = all(row[k] == prev[k] for k in ('strategy', 'fault', 'phase'))
                if same_state and (kind != 'policy' or (row['flags'] == prev['flags'] == 4
                                        and (row['correlation'] - prev['correlation']) % 2**32 == 1)):
                    metric(row, 'entry_interval_us', (row['begin_ns'] - prev['begin_ns']) / 1000)
                if kind == 'actuation':
                    advances = (row['value_0'] - prev['value_0']) % 2**32
                    row['trigger_advances'] = advances
                    # This is not a time unit and is not itself proof of lost releases.
                    if advances != 1:
                        missing['actuation_trigger_advances_not_one'] += 1
                if same_state and kind == 'pd' and row['value_0'] == prev['value_0'] and row['flags'] & prev['flags'] & 1:
                    row['same_action_reuse_count'] = prev.get('same_action_reuse_count', 0) + 1
                    row['observed_action_hold_start_ns'] = prev.get('observed_action_hold_start_ns', prev['begin_ns'])
                    metric(row, 'observed_same_action_hold_us', (row['begin_ns'] - row['observed_action_hold_start_ns']) / 1000)
            previous[key] = row
    stats = []
    dimensions = ('kind', 'source', 'can_id', 'result', 'strategy', 'fault', 'phase', 'metric')
    grouped = {}
    for item in metrics:
        grouped.setdefault(tuple(item[k] for k in dimensions), []).append(item)
    for key in sorted(grouped):
        chosen = grouped[key]
        values = np.asarray([m['value_us'] for m in chosen])
        stats.append(dict(zip(dimensions, key)) | dict(count=len(values),
                          mean_us=float(values.mean()), median_us=float(np.median(values)),
                          std_us=float(values.std()), p95_us=float(np.percentile(values, 95)),
                          p99_us=float(np.percentile(values, 99)), max_us=float(values.max()),
                          min_us=float(values.min()), peak_to_peak_us=float(np.ptp(values)),
                          max_record_sequence=chosen[int(values.argmax())]['sequence']))
    out.mkdir(parents=True, exist_ok=False)
    write_csv(out / 'records.csv', rows)
    write_csv(out / 'measurements.csv', metrics)
    write_csv(out / 'statistics.csv', stats)
    observed_complete = bool(rows) and not (parser.rejected or partial or boundaries or len({p['connection'] for p in packets}) > 1)
    observed_complete = observed_complete and meta.get('capture_complete', False) and packets[-1]['pending_can'] == 0 and packets[-1]['queued'] == len(last_packet['records']) and packets[-1]['dropped'] == packets[0]['dropped']
    report = dict(format='h7-latency-capture-analysis', session=str(session), stream_sha256=meta['stream_sha256'],
                  decoder_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  record_count=len(rows), kinds=dict(Counter(r['kind'] for r in rows)),
                  records_complete_in_observed_span=observed_complete, boundaries=boundaries,
                  rejected_packets=parser.rejected, partial_packets=partial,
                  first_dropped_total=packets[0]['dropped'], last_dropped_total=packets[-1]['dropped'],
                  first_pending_can=packets[0]['pending_can'], last_pending_can=packets[-1]['pending_can'],
                  first_queued=packets[0]['queued'], last_queued=packets[-1]['queued'] - len(last_packet['records']),
                  unavailable_pairs_and_trigger_notes=dict(missing),
                  can_results=dict(Counter(str(r['flags'] & 0x7f) for r in rows if r['kind'] == 'can')),
                  statistics=stats,
                  limitations=['CAN IRQ/poll observation is an upper bound, not electrical or motor application time',
                               'Trigger advances measure releases between entries, not proof of scheduler deadline failure',
                               'Build/SDK/age fields are integer us; read offset is integer ns; no nanosecond accuracy claim',
                               'Failed/uninvoked SDK duration is unavailable, not a stale prior run_us',
                               'Observed hold begins at the first PD record in this continuous span; action age uses its original publication',
                               'Source times are H7 reception times, not internal sensor sampling/filtering',
                               'No physics/observations/actions were modified by this offline decoder',
                               'Pending CAN and initial/final queued records are outside complete-export claims'])
    (out / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    result = decode_session(args.session, args.out)
    print(json.dumps({k: v for k, v in result.items() if k != 'statistics'}, ensure_ascii=False, indent=2))
    if not result['records_complete_in_observed_span']:
        raise SystemExit(2)


if __name__ == '__main__':
    main()
