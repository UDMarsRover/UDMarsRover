# Cameras must be connected *before* boot, or they will not configure properly.


# ffmpeg must be installed for the usb cameras to work.



## COMMANDS TO RUN TO SETUP AUTO START

sudo nano /etc/systemd/system/mediamtx.service

# Then in that file paste:

[Unit]
Description=MediaMTX Media Server
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
ExecStart=/home/udmrt/mediamtx/mediamtx
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target

# Then run:
sudo systemctl daemon-reload
sudo systemctl enable mediamtx.service
sudo systemctl start mediamtx.service



