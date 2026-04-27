#!/usr/bin/bash
set -euo pipefail

# install.sh - install and enable the rover systemd unit
# Usage: run from the repository (or from anywhere) as a user with sudo privileges

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_SRC="$SCRIPT_DIR/rover.service"
TARGET="/etc/systemd/system/rover.service"

if [ ! -f "$SERVICE_SRC" ]; then
	echo "Service file not found: $SERVICE_SRC" >&2
	exit 1
TARGET="/etc/systemd/system/rover.service"

if [ ! -f "$SERVICE_SRC" ]; then
	echo "Service file not found: $SERVICE_SRC" >&2
	exit 1
fi

fi

# --- New: install Docker & docker-compose on Raspberry Pi OS if missing ---
# Determine real user (when run via sudo)
# Install Docker & docker-compose (moved to separate script for clarity)
bash "$SCRIPT_DIR/docker-install.sh" "$SERVICE_SRC"
# --- End new content ---

echo "Copying $SERVICE_SRC to $TARGET (sudo may be required)"
sudo cp "$SERVICE_SRC" "$TARGET"
sudo chmod 644 "$TARGET"

echo "Reloading systemd daemon"
sudo systemctl daemon-reload

echo "Enabling rover.service"
sudo systemctl enable rover.service

echo "Starting (or restarting) rover.service"
sudo systemctl restart rover.service

echo "Service status (last lines)"
sudo systemctl status rover.service --no-pager || true

echo "install.sh: complete"