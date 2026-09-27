#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
sudo install -m 0644 "${SCRIPT_DIR}/99-meow-usb2fdcan.rules" /etc/udev/rules.d/99-meow-usb2fdcan.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG dialout "${USER}"
echo "udev rule installed. Unplug/replug the adapter, then log out and back in once."

