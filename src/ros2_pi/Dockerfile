FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Added libraspberrypi-bin for vcgencmd support
RUN apt-get update && apt-get install -y \
    python3-pip \
    bluez \
    bluetooth \
    dbus \
    libraspberrypi-bin \
    && rm -rf /var/lib/apt/lists/*

ENV PIP_BREAK_SYSTEM_PACKAGES=1

# Added psutil to the pip installs
RUN pip3 install bleak gpiozero rpi-lgpio psutil

WORKDIR /app

COPY canbat_node.py .
COPY gpio_node.py .
COPY sys_monitor_node.py .
COPY start.sh .

RUN chmod +x start.sh

CMD ["/app/start.sh"]
