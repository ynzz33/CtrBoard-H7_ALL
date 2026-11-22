"""Exercise both models over a pseudo-terminal, with no motor connection."""
import binascii
import glob
import os
import pty
import select
import struct
import subprocess
import tempfile
import time

root = "/home/imca/rl-ynzz_33"
master, slave = pty.openpty()
directory = tempfile.mkdtemp(prefix="jump-usb-check-")
run = directory + "/records"
process = subprocess.Popen([
    root + "/jump-dual-stage/jetson_rl", "shadow",
    "--port", os.ttyname(slave),
    "--engine", root + "/policy6838-stage/policy6838PD200.engine",
    "--jump-engine", root + "/jump5200/policy5200.engine",
    "--log-dir", run, "--seconds", "0",
], env={**os.environ, "JRL_EXTERNAL_ACTION": "1"})
buffer = bytearray()

def receive(kind):
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        if len(buffer) >= 8:
            size = 36 + struct.unpack_from("<H", buffer, 6)[0]
            if len(buffer) >= size:
                packet = bytes(buffer[:size])
                del buffer[:size]
                assert binascii.crc32(packet[:-4]) == struct.unpack_from("<I", packet, size-4)[0]
                if packet[5] == kind:
                    return packet
                continue
        if select.select([master], [], [], .1)[0]:
            buffer.extend(os.read(master, 4096))
    raise TimeoutError(f"frame {kind}")

try:
    ready = receive(10)
    session = struct.unpack_from("<Q", ready, 8)[0]
    observation = [0.] * 25
    observation[5], observation[8] = -1., 1.5
    inputs = observation * 6
    sequence = 0
    def request(flags, force, contract=3000202):
        global sequence
        sequence += 1
        timestamp = 1000000 + sequence * 10000
        for half in range(2):
            payload = struct.pack("<I75f", contract, *inputs[half*75:(half+1)*75])
            if contract == 3000202:
                payload += struct.pack("<Iff", flags, force, .25)
            header = struct.pack("<4sBBHQIIQ", b"JRL2", 2, 6+half, len(payload), session, 1, sequence, timestamp)
            packet = header + payload
            os.write(master, packet + struct.pack("<I", binascii.crc32(packet)))
        reply = receive(8)
        assert struct.unpack_from("<I", reply, 20)[0] == sequence
        assert struct.unpack_from("<I", reply, 32)[0] == contract
        time.sleep(.01)
    request(0, 80, 3000201)  # Existing firmware retains WALK.
    request(5, 80)
    request(7, 80)
    for _ in range(4): request(7, 0)
    for _ in range(6): request(7, 80)
    for _ in range(9): request(7, 80)
finally:
    process.terminate()
    process.wait(timeout=15)
    os.close(master)
    os.close(slave)

models, phases = [], []
for filename in glob.glob(run + "/*.bin"):
    with open(filename, "rb") as stream:
        while header := stream.read(16):
            kind, size, _ = struct.unpack("<IIQ", header)
            payload = stream.read(size)
            if kind == 15:
                _, phase, model = struct.unpack_from("<III", payload)
                phases.append(phase)
                models.append(model)
                if model == 5200:
                    assert struct.unpack_from("<f", payload, 24+8*4)[0] == 5.0
assert 5200 in models and models[-1] == 6838
assert {1, 2, 3, 4}.issubset(phases)
print("USB pseudo-terminal passed:", directory, "models", sorted(set(models)))
