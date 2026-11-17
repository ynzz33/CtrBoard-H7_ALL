#!/usr/bin/env bash
set -euo pipefail

root=/home/imca/rl-ynzz_33
src="$root/rl-usb-recorder"
if [[ $(id -un) != imca ]]; then
    echo 'Run this script as imca without sudo.' >&2
    exit 1
fi

if (( $# > 1 )); then
    echo 'Usage: bash install.sh [/dev/serial/by-id/<STM32 device>]' >&2
    exit 1
fi
if (( $# == 1 )); then
    port=$1
else
    shopt -s nullglob
    ports=(/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_*-if00)
    if (( ${#ports[@]} != 1 )); then
        echo 'Connect the H7, or pass its exact /dev/serial/by-id/ path.' >&2
        ls -l /dev/serial/by-id/ >&2 || true
        exit 1
    fi
    port=${ports[0]}
fi
if [[ $(dirname "$port") != /dev/serial/by-id || ! -c "$port" || ! -r "$port" || ! -w "$port" ]]; then
    echo "Cannot access the USB serial device as imca: $port" >&2
    exit 1
fi

name=$(systemd-escape --template=jetson-rl-usb-record@.service "${port##*/}")
if systemctl is-active --quiet "$name"; then
    echo "Already running: $name"
    exit 0
fi
if systemctl is-enabled --quiet jetson-usb-record.service ||
   systemctl is-active --quiet jetson-usb-record.service; then
    echo 'Disable the old jetson-usb-record.service before installing.' >&2
    exit 1
fi
command -v fuser >/dev/null
sudo -v
if sudo fuser -s "$port"; then
    echo "Stop the process currently using $port before installing." >&2
    sudo fuser -v "$port" || true
    exit 1
fi

sudo install -m 644 "$src/jetson-rl-usb-record@.service" /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now "$name"
systemctl status "$name" --no-pager
