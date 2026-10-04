#!/bin/sh
# install_terrad.sh - run once on the board:  sudo sh install_terrad.sh
#
# Installs terrad as a service that starts at boot. Starting it ends the
# login session on the serial port (terrad needs that port); that is expected.
set -e
cd "$(dirname "$0")"
[ -x ./terrad ] && [ -x ./mc ] || { echo "terrad and mc must be next to this script"; exit 1; }
install -m 644 terrad.service /etc/systemd/system/terrad.service
systemctl daemon-reload
systemctl enable terrad.service
echo "terrad installed and enabled at boot."
echo "Starting it now: this serial session will close. Type 'console' + Enter"
echo "in minicom to get a login shell back."
sleep 2
systemctl start terrad.service
