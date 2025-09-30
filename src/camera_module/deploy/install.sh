#!/bin/bash
# install the bt service

sudo cp camera-server.service /etc/systemd/system/camera-server.service
sudo cp servo.service /etc/systemd/system/servo.service
sudo systemctl daemon-reload
sudo systemctl enable camera-server.service
sudo systemctl enable servo.service
sudo systemctl start camera-server.service
sudo systemctl start servo.service