#!/usr/bin/env bash
set -euo pipefail
cd /home/imca/rl-ynzz_33/shadow-policy4000PD200
port="/dev/serial/by-id/${1:?pass the STM32 USB by-id name}"
unit=$(systemd-escape --template=jetson-rl-usb-record@.service "${port##*/}")
logs=/home/imca/rl-ynzz_33/logs/shadow
prefix=policy6838PD200
analyzer=analyze_shadow.py
if [[ ${JRL_EXTERNAL_ACTION:-0} == 1 ]]; then
    logs=/home/imca/rl-ynzz_33/logs/external
    prefix=policy6838PD200-external
    analyzer=analyze_external.py
fi
mkdir -p "$logs"
restore=0
child=
cleanup() {
    if [[ -n "$child" ]]; then
        kill -TERM "$child" 2>/dev/null || true
        wait "$child" || true
    fi
    if (( restore )); then sudo -n systemctl start "$unit"; fi
}
trap cleanup EXIT
trap 'exit 0' TERM INT
if systemctl is-active --quiet "$unit"; then
    restore=1
    sudo -n systemctl stop "$unit"
fi
echo "Waiting for H7 USB: $port"
while true; do
    if [[ ! -c "$port" || ! -r "$port" || ! -w "$port" ]]; then
        sleep 2
        continue
    fi
    if sudo -n fuser -s "$port"; then
        echo "USB busy: $port"
        sleep 5
        continue
    fi
    run="$logs/$prefix-$(date +%Y%m%d-%H%M%S)-$(cat /proc/sys/kernel/random/uuid)"
    echo "Opening $port; run=$run"
    /home/imca/rl-ynzz_33/jump-dual-stage/jetson_rl shadow --port "$port" --engine /home/imca/rl-ynzz_33/policy6838-stage/policy6838PD200.engine --jump-engine /home/imca/rl-ynzz_33/jump5200/policy5200.engine --log-dir "$run" --seconds 0 &
    child=$!
    status=0
    wait "$child" || status=$?
    child=
    echo "Shadow connection ended ($status); waiting to reopen USB"
    shopt -s nullglob
    segments=("$run"/records-*.bin)
    if (( ${#segments[@]} )); then
        cp /home/imca/rl-ynzz_33/policy6838-stage/policy6838PD200.engine.meta "$run/engine.meta"
        cp /home/imca/rl-ynzz_33/jump5200/policy5200.engine.meta "$run/jump-engine.meta"
        cp /home/imca/rl-ynzz_33/jump5200/interface.json "$run/jump-interface.json"
        for segment in "${segments[@]}"; do
            python3 "$analyzer" "$segment" > "${segment%.bin}-summary.txt" || true
        done
    fi
    sleep 2
done
