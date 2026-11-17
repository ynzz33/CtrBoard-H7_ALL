set -euo pipefail
cd /home/imca/rl-ynzz_33/shadow-policy3000PD200
shopt -s nullglob
ports=(/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_*-if00)
if (( ${#ports[@]} != 1 )); then
    echo "Expected one H7 USB port; found ${#ports[@]}" >&2
    exit 1
fi
port=${ports[0]}
test -c "$port" && test -r "$port" && test -w "$port"
unit=$(systemd-escape --template=jetson-rl-usb-record@.service "${port##*/}")
sudo -n true
logs=/home/imca/rl-ynzz_33/logs/shadow
mkdir -p "$logs"
run="$logs/policy3000PD200-$(date +%Y%m%d-%H%M%S)-$$"
restore=0
cleanup() {
    if (( restore )); then sudo -n systemctl start "$unit"; fi
}
trap cleanup EXIT
if systemctl is-active --quiet "$unit"; then
    restore=1
    sudo -n systemctl stop "$unit"
fi
if sudo -n fuser -s "$port"; then
    echo "USB port already in use: $port" >&2
    exit 1
fi
printf 'Opening %s; run=%s\n' "$port" "$run"
status=0
./build/jetson_rl shadow --port "$port" --engine policy3000PD200.engine --log-dir "$run" --seconds "${1:-90}" || status=$?
cleanup
restore=0
if [[ -s "$run/records.bin" ]]; then
    cp policy3000PD200.engine.meta "$run/engine.meta"
    python3 analyze_shadow.py "$run/records.bin" | tee "$run/shadow-summary.txt"
fi
exit "$status"