#!/usr/bin/env bash
set -euo pipefail
# H7失能后执行；服务启动只建立通信，实际出力仍由H7许可。
root=/home/imca/rl-ynzz_33
test -x "$root/jump-dual-stage/jetson_rl"
test -f "$root/jump5200/policy5200.engine.meta"
test -f "$root/jump-dual-stage/run-jump-dual.sh"
temp=$(mktemp)
trap 'rm -f "$temp"' EXIT
cat > "$temp" <<'UNIT'
[Service]
ExecStart=
ExecStart=/bin/bash /home/imca/rl-ynzz_33/jump-dual-stage/run-jump-dual.sh usb-STMicroelectronics_STM32_Virtual_ComPort_386F33613134-if00
UNIT
sudo -n install -m 644 "$temp" /etc/systemd/system/jetson-rl-external-4000.service.d/jump-model.conf
sudo -n systemctl daemon-reload
sudo -n systemctl restart jetson-rl-external-4000.service
systemctl is-active jetson-rl-external-4000.service
