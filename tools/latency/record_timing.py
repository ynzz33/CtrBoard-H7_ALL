"""Record the full UART/USB binary timing stream. Writes only trace START/STOP.

USB COM name is an argument. This tool never enables a motor or changes a target.
Raw chunks, firmware identity and start/stop acknowledgements are preserved.
"""
import argparse
import binascii
import hashlib
import json
from pathlib import Path
import secrets
import struct
import time

ROOT = Path(__file__).resolve().parents[2]


def command(op, token):
    data = struct.pack('<4sBBHI', b'H7TC', op, 1, 0, token)
    return data + struct.pack('<H', binascii.crc_hqx(data, 0xffff))


class StatusParser:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        result = []
        while True:
            start = self.buffer.find(b'H7STAT01')
            if start < 0:
                del self.buffer[:max(0, len(self.buffer) - 7)]
                break
            del self.buffer[:start]
            if len(self.buffer) < 34:
                break
            packet = self.buffer[:34]
            if packet[8] == 1 and binascii.crc_hqx(packet[:32], 0xffff) == int.from_bytes(packet[32:], 'little'):
                result.append(dict(state=packet[9], token=int.from_bytes(packet[12:16], 'little'),
                                   queued_output=int.from_bytes(packet[16:20], 'little'),
                                   queued_timing=int.from_bytes(packet[20:24], 'little'),
                                   pending_can=int.from_bytes(packet[24:28], 'little')))
                del self.buffer[:34]
            else:
                del self.buffer[0]
        return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', required=True)
    p.add_argument('--baud', type=int, default=1152000, help='must match firmware UART; USB ignores line coding rate')
    p.add_argument('--seconds', type=float, default=30.)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--firmware-manifest', type=Path, required=True)
    p.add_argument('--model', type=Path, default=ROOT / 'X-CUBE-AI/App/networkzn1_data_params.c')
    p.add_argument('--machine-config', type=Path, default=ROOT / 'imcalib/config/machine_config.c')
    a = p.parse_args()
    if not 0 < a.seconds <= 3600:
        p.error('seconds must be in (0,3600]')
    import serial
    a.out.mkdir(parents=True, exist_ok=False)
    meta = dict(format='h7-timing-session', version=1, complete=False, capture_complete=False,
                port=a.port, baud=a.baud, seconds=a.seconds, token=secrets.randbits(32), acknowledgements=[])
    for key, source in (('model', a.model), ('machine_config', a.machine_config), ('firmware_manifest', a.firmware_manifest)):
        data = source.read_bytes()
        name = key + source.suffix
        (a.out / name).write_bytes(data)
        meta[key] = dict(file=name, source=str(source.resolve()), sha256=hashlib.sha256(data).hexdigest())
    meta_path = a.out / 'meta.json'
    meta_path.write_text(json.dumps(meta, indent=2), encoding='utf-8')
    parser = StatusParser()
    digest = hashlib.sha256()
    count = 0
    origin = time.monotonic_ns()
    final_stamp = 0
    error = None
    with (a.out / 'stream.jsonl').open('wb') as stream:
        def event(kind, **fields):
            nonlocal count, final_stamp
            final_stamp = time.monotonic_ns() - origin
            raw = (json.dumps(dict(type=kind, t_ns=final_stamp, **fields), separators=(',', ':')) + '\n').encode()
            stream.write(raw)
            digest.update(raw)
            count += 1
        try:
            with serial.Serial(a.port, a.baud, timeout=.05, write_timeout=.5) as port:
                try:
                    event('connected')
                    port.write(command(1, meta['token']))
                    started = None
                    stop_sent = None
                    while True:
                        data = port.read(min(max(port.in_waiting, 1), 4096))
                        now = time.monotonic()
                        if data:
                            event('data', data_hex=data.hex())
                            for ack in parser.feed(data):
                                if ack['token'] != meta['token']:
                                    continue
                                meta['acknowledgements'].append(ack)
                                if ack['state'] == 1 and started is None:
                                    started = now
                                if stop_sent is not None and ack['state'] == 3:
                                    meta['capture_complete'] = True
                        if meta['capture_complete']:
                            break
                        if started is None and (time.monotonic_ns() - origin) > 5_000_000_000:
                            raise TimeoutError('no trace START acknowledgement; check firmware/selected port/RX wiring/baud')
                        if started is not None and now - started >= a.seconds and stop_sent is None:
                            port.write(command(2, meta['token']))
                            stop_sent = now
                        if stop_sent is not None and now - stop_sent > 5:
                            raise TimeoutError('STOP not drained; pending CAN/USB or missing tail; keep this session')
                finally:
                    if not meta['capture_complete']:
                        try:
                            port.write(command(2, meta['token']))
                        except Exception:
                            pass  # Recording remains explicitly incomplete; never issue motor commands.
        except BaseException as exc:
            error = repr(exc)
        finally:
            event('stop')
            stream.flush()
    meta.update(complete=True, error=error, event_count=count, duration_ns=final_stamp, stream_sha256=digest.hexdigest())
    meta_path.write_text(json.dumps(meta, indent=2), encoding='utf-8')
    print(json.dumps(dict(directory=str(a.out), capture_complete=meta['capture_complete'], error=error)))
    if error:
        raise SystemExit(2)


if __name__ == '__main__':
    main()
