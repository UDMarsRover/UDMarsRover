#!/bin/bash

# Exit on error
set -e

echo "--- Starting UDMRT Camera Pi Installation Script ---"
echo  "This script can take a while to complete. "
echo  "This script will also fail if not connected to the network."

echo  "Confirming Architecture..."

# 1. Determine Architecture
ARCH=$(uname -m)
if [ "$ARCH" = "x86_64" ]; then
    PLATFORM="linux_amd64"
elif [ "$ARCH" = "aarch64" ]; then
    PLATFORM="linux_arm64v8"
elif [[ "$ARCH" == armv* ]]; then
    PLATFORM="linux_armv7"
else
    echo "Unsupported architecture: $ARCH"
    exit 1
fi

echo "Detected architecture: $PLATFORM"



echo "--- Starting MediaMTX Installation ---"

echo  "Installing Required Libraries"

sudo apt-get install libfreetype6 libcamera0.6

echo  "Required Libraries Installed. Proceeding with downloading MediaMTX v1.11.3."

sudo mkdir /home/udmrt/mediamtx
cd /home/udmrt/mediamtx

wget https://github.com/bluenviron/mediamtx/releases/download/v1.11.3/mediamtx_v1.11.3_linux_arm64v8.tar.gz


echo  "Extracting MediaMTX..."

tar xzvf mediamtx_v1.11.3_linux_arm64v8.tar.gz

# 4. Install Binaries and Config
#echo "Installing files..."
#sudo mv /tmp/mediamtx/mediamtx /usr/local/bin/
#sudo mkdir -p /usr/local/etc/mediamtx/
# Move config only if it doesn't exist to avoid overwriting your settings later
#if [ ! -f /usr/local/etc/mediamtx/mediamtx.yml ]; then
 #   sudo mv /tmp/mediamtx/mediamtx.yml /usr/local/etc/mediamtx/
#fi

CONF_FILE=mediamtx.yml

echo  "Updating Configuration for UDMRT RPi Cameras..."
# 2. Use sed to delete everything from the "paths:" line to the end of the file
# This clears out the example paths you mentioned
sudo sed -i '/^paths:/,$d' "$CONF_FILE"

# 3. Append your specific RPi Camera configuration
sudo bash -c cat <<EOF >> $CONF_FILE
paths:
  cam:
    source: rpiCamera
    rpiCameraWidth: 1280
    rpiCameraHeight: 720
    rpiCameraVFlip: true
    rpiCameraHFlip: true
    rpiCameraBitrate: 1500000
EOF


# 5. Create Systemd Service
echo "Creating systemd service..."
sudo bash -c 'cat <<EOF > /etc/systemd/system/mediamtx.service
[Unit]
Description=MediaMTX (Media Server)
After=network.target

[Service]
Type=simple
ExecStart=/home/udmrt/mediamtx
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF'

# 6. Reload and Start
sudo systemctl daemon-reload
sudo systemctl enable mediamtx
sudo systemctl start mediamtx

echo "--- MediaMTX Installation Complete! ---"
echo "Service is running. Edit config at: /home/udmrt/mediamtx/mediamtx.yml"
echo "Check status with: sudo systemctl status mediamtx"


echo "########################################################"

echo "--- Beginning setup of Docker! ---"

sudo mkdir /home/udmrt/docker
cd /home/udmrt/docker

sudo apt-get install ca-certificates curl gnupg lsb-release -y


sudo mkdir -p /etc/apt/keyrings

echo "Downloading Docker GPG key and adding to keyrings..."

curl -fsSL https://download.docker.com/linux/debian/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg

sudo chmod a+r /etc/apt/keyrings/docker.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/debian $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

echo "Downloading Docker."
curl -fsSL https://get.docker.com -o get-docker.sh

echo "Installing Docker..."
sudo sh get-docker.sh


echo "Enabling and starting Docker service..."
sudo systemctl enable docker
sudo systemctl start docker

