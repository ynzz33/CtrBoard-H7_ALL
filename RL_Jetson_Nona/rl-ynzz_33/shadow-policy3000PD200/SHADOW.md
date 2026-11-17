# 3500v3 USB shadow test

Use a fresh extract directory and the matched H7 shadow HEX. The previous
3500v3 TensorRT engine and its .meta file can be reused on the same Jetson.
Stop other readers of the CDC port before starting. First test on a bench
with motor power disconnected. H7 CubeAI remains the action source.

    cmake -S deployment/jetson_rl -B build -DCMAKE_BUILD_TYPE=Release
    cmake --build build -j2
    mkdir -p logs
    run="logs/shadow-$(date +%s)"
    ./build/jetson_rl shadow --port /dev/ttyACM0 --engine /path/to/policy3500v3.engine --log-dir "$run" --seconds 30
    python3 analyze_shadow.py "$run/records.bin"
    cat "$run/summary.txt"

The shadow input is produced only when the existing control path calls the
network successfully. A bench with unavailable motor feedback may yield
zero requests. Check the existing RL gates and telemetry in that case.
Do not bypass them. This is not an external-control executable.

The analyzer reports H7 input-ready to background reply-validation time.
That includes local CubeAI while preparing the sample, USB scheduling, and
Jetson forward. It does not measure time to a PD consumption. The first
and last samples can be partial at capture boundaries. For an A/B compare,
use the same firmware sources and workload with the build switch off/on.
The independent legacy latency protocol is a separate USB mode.
