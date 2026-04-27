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
fi

# --- New: install Docker & docker-compose on Raspberry Pi OS if missing ---
# Determine real user (when run via sudo)
REAL_USER="${SUDO_USER:-$USER}"

if ! command -v docker >/dev/null 2>&1; then
	echo "Installing Docker Engine (using Docker apt repo)..."
	sudo apt-get update
	sudo apt-get install -y ca-certificates curl gnupg lsb-release

	# Add Docker GPG key and repo
	sudo mkdir -p /etc/apt/keyrings
	curl -fsSL https://download.docker.com/linux/debian/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
	ARCH="$(dpkg --print-architecture)"
	CODENAME="$(lsb_release -cs)"
	echo "deb [arch=$ARCH signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/debian $CODENAME stable" | \
		sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

	# Install Docker Engine + compose plugin
	sudo apt-get update
	sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin

	# Enable and start docker
	sudo systemctl enable --now docker

	# Add real user to docker group so they can run docker without sudo
	if [ -n "$REAL_USER" ] && [ "$REAL_USER" != "root" ]; then
		echo "Adding $REAL_USER to docker group (you may need to log out/in)"
		sudo usermod -aG docker "$REAL_USER" || true
	fi
else
	echo "Docker already installed"
fi

# Ensure docker-compose CLI is available (try plugin symlink, otherwise download standalone)
if ! command -v docker-compose >/dev/null 2>&1; then
	# Common plugin locations
	if [ -f /usr/libexec/docker/cli-plugins/docker-compose ]; then
		sudo ln -sf /usr/libexec/docker/cli-plugins/docker-compose /usr/local/bin/docker-compose || true
	elif [ -f /usr/lib/docker/cli-plugins/docker-compose ]; then
		sudo ln -sf /usr/lib/docker/cli-plugins/docker-compose /usr/local/bin/docker-compose || true
	else
		echo "Installing standalone docker-compose v2 binary for this architecture..."
		ARCH_NAME="$(uname -m)"
		case "$ARCH_NAME" in
			aarch64) ASSET=linux-aarch64 ;;
			armv7l)  ASSET=linux-armv7 ;;
			armv6l)  ASSET=linux-armv6 ;;
			x86_64)  ASSET=linux-x86_64 ;;
			*)       ASSET=linux-x86_64 ;;
		esac

		# Try to get latest tag from GitHub; fall back to a known tag if unavailable
		TAG="$(curl -sSL https://api.github.com/repos/docker/compose/releases/latest | grep -Po '"tag_name": "\K.*?(?=")' || true)"
		if [ -z "$TAG" ]; then
			TAG="v2.20.2"
		fi

		DOWNLOAD_URL="https://github.com/docker/compose/releases/download/${TAG}/docker-compose-${ASSET}"
		sudo curl -fsSL -o /usr/local/bin/docker-compose "$DOWNLOAD_URL"
		sudo chmod +x /usr/local/bin/docker-compose || true
	fi
else
	echo "docker-compose already available"
fi
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