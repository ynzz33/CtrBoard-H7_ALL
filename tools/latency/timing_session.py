"""Hashed raw timing sessions; no simulation, model inference or device writes."""
import hashlib
import json
from pathlib import Path

MOTORS = ('lf0', 'lf00', 'rf0', 'rf00', 'lfwheel', 'rfwheel')


def iter_session_events(directory):
    last = -1
    stopped = False
    connected = False
    with (Path(directory) / 'stream.jsonl').open('rb') as stream:
        for raw in stream:
            if not raw.endswith(b'\n'):
                raise ValueError('truncated session line')
            event = json.loads(raw)
            if type(event['t_ns']) is not int or event['t_ns'] < last or stopped:
                raise ValueError('invalid session clock/end')
            kind = event['type']
            if kind == 'connected':
                connected = True
            elif kind == 'disconnected':
                connected = False
            elif kind == 'data':
                if not connected or not 0 < len(bytes.fromhex(event['data_hex'])) <= 4096:
                    raise ValueError('invalid session data')
            elif kind == 'stop':
                stopped = True
            else:
                raise ValueError('unsupported session event')
            last = event['t_ns']
            yield event
    if not stopped:
        raise ValueError('missing final stop event')


def load_session(directory):
    directory = Path(directory)
    meta = json.loads((directory / 'meta.json').read_text(encoding='utf-8'))
    if meta.get('format') != 'h7-timing-session' or meta.get('version') != 1 or not meta.get('complete'):
        raise ValueError('unsupported/incomplete recording')
    if hashlib.sha256((directory / 'stream.jsonl').read_bytes()).hexdigest() != meta['stream_sha256']:
        raise ValueError('recording hash mismatch')
    events = list(iter_session_events(directory))
    if len(events) != meta['event_count'] or events[-1]['t_ns'] != meta['duration_ns']:
        raise ValueError('recording count/duration mismatch')
    for key in ('model', 'machine_config', 'firmware_manifest'):
        item = meta[key]
        if hashlib.sha256((directory / item['file']).read_bytes()).hexdigest() != item['sha256']:
            raise ValueError('identity file hash mismatch: ' + key)
    from record_timing import StatusParser
    parser = StatusParser()
    seen_start = False
    seen_end = False
    for event in events:
        if event['type'] == 'data':
            for ack in parser.feed(bytes.fromhex(event['data_hex'])):
                if ack['token'] == meta['token']:
                    seen_start |= ack['state'] == 1
                    seen_end |= seen_start and ack['state'] == 3 and not (ack['queued_output'] or ack['queued_timing'] or ack['pending_can'])
    meta['capture_complete'] = bool(meta.get('capture_complete') and seen_start and seen_end)
    return meta, directory / meta['model']['file'], directory / meta['machine_config']['file']
