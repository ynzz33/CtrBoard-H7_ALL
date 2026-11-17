"""Receive H7 JRL2 telemetry and save only RL-engaged frames."""

import argparse
import fcntl
import os
import select
import shutil
import signal
import struct
import termios
import time
import tty
import zlib
from datetime import datetime
from pathlib import Path


FRAME_HEADER = struct.Struct("<4sBBHQIIQ")
LOG_HEADER = struct.Struct("<IIQ")
U32 = struct.Struct("<I")
RL_ENGAGED = 1 << 1
PD_PAYLOAD_BYTES = 240
SEGMENT_SECONDS = 300
IDLE_SECONDS = 2
MIN_FREE_BYTES = 5 * 1024**3


class Telemetry:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        while True:
            start = self.buffer.find(b"JRL2")
            if start < 0:
                del self.buffer[:-3]
                return
            del self.buffer[:start]
            if len(self.buffer) < FRAME_HEADER.size:
                return
            _, version, kind, size, *_ = FRAME_HEADER.unpack_from(self.buffer)
            if version != 2 or size > 348:
                del self.buffer[0]
                continue
            end = FRAME_HEADER.size + size
            if len(self.buffer) < end + U32.size:
                return
            if zlib.crc32(self.buffer[:end]) != U32.unpack_from(self.buffer, end)[0]:
                del self.buffer[0]
                continue
            frame = bytes(self.buffer[:end + U32.size])
            del self.buffer[:end + U32.size]
            if kind == 5 and size == PD_PAYLOAD_BYTES:
                yield frame


class Segments:
    def __init__(self, directory):
        self.directory = directory
        self.file = None
        self.pending = None
        self.deadline = 0
        self.disk_checked_at = float("-inf")

    def save(self):
        if self.file is None:
            return
        file, self.file = self.file, None
        with file:
            file.flush()
            os.fsync(file.fileno())
        target = self.pending.with_suffix(".bin")
        self.pending.rename(target)
        print(f"Saved {target}", flush=True)

    def check_deadline(self, now):
        if self.file is not None and now >= self.deadline:
            self.save()

    def handle(self, frame, received):
        flags, = U32.unpack_from(frame, FRAME_HEADER.size)
        if not flags & RL_ENGAGED:
            self.save()
            return
        self.check_deadline(received)
        if self.file is None:
            if received - self.disk_checked_at < 5:
                return
            if shutil.disk_usage(self.directory).free < MIN_FREE_BYTES:
                self.disk_checked_at = received
                print("Waiting for 5 GiB of free space", flush=True)
                return
            stamp = datetime.now().strftime("%Y%m%d-%H%M%S-%f")
            self.pending = self.directory / f"records-{stamp}-{time.monotonic_ns()}.partial"
            self.file = self.pending.open("xb")
            self.deadline = received + SEGMENT_SECONDS
        self.file.write(LOG_HEADER.pack(1, len(frame), int(received * 1_000_000)) + frame)


def receive(port, segments, is_stopping):
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        fcntl.ioctl(fd, termios.TIOCEXCL)
        tty.setraw(fd)
        config = termios.tcgetattr(fd)
        config[2] = (config[2] & ~termios.CRTSCTS) | termios.CLOCAL | termios.CREAD
        config[4] = config[5] = termios.B1152000
        termios.tcsetattr(fd, termios.TCSANOW, config)
        poll = select.poll()
        poll.register(fd, select.POLLIN)
        parser = Telemetry()
        last_frame = None
        print(f"Listening on {port}", flush=True)
        while not is_stopping():
            events = poll.poll(200)
            now = time.monotonic()
            segments.check_deadline(now)
            if last_frame is not None and now - last_frame >= IDLE_SECONDS:
                segments.save()
                last_frame = None
            for _, events_mask in events:
                if events_mask & (select.POLLERR | select.POLLHUP | select.POLLNVAL):
                    raise OSError(f"USB disconnected: {port}")
                try:
                    data = os.read(fd, 4096)
                except BlockingIOError:
                    continue
                if not data:
                    raise OSError(f"USB closed: {port}")
                for frame in parser.feed(data):
                    last_frame = now
                    segments.handle(frame, now)
    finally:
        os.close(fd)


def main():
    args = argparse.ArgumentParser(description=__doc__)
    args.add_argument("--port", required=True)
    args.add_argument("--log-dir", type=Path, required=True)
    options = args.parse_args()
    options.log_dir.mkdir(parents=True, exist_ok=True)
    segments = Segments(options.log_dir)
    stopping = False

    def stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    previous_error = None
    while not stopping:
        try:
            receive(options.port, segments, lambda: stopping)
        except OSError as error:
            if str(error) != previous_error:
                print(error, flush=True)
                previous_error = str(error)
        finally:
            segments.save()
        if not stopping:
            time.sleep(2)


if __name__ == "__main__":
    main()
