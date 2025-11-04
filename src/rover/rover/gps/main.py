"""
This script is written for use on a Raspberry Pi Pico. It is written in MicroPython, and utilizes the machine package, which is a core package
within MicroPython. The purpose of this script is to parse data from the NEO-7M GPS module and output a string containing coordinate information
over the Pico's COM port.

Because we are using the Raspberry Pi Pico, this file MUST be named "main.py". By default, when the Pico is connected to a power source, it will
run the python script saved in it's EEPROM that is titled "main.py", and there is no way to command the Pico to run any other script on startup.
"""

def nmea_to_decimal(coord_str, direction):
    # coord_str like '4807.038' or '01131.000'
    if not coord_str or coord_str == '':
        return None
    if '.' not in coord_str:
        return None
    deg_len = 2 if direction in ['N', 'S'] else 3
    degrees = float(coord_str[:deg_len])
    minutes = float(coord_str[deg_len:])
    decimal = degrees + minutes / 60
    if direction in ['S', 'W']:
        decimal *= -1
    return decimal

from machine import UART, Pin
import sys
import time

uart = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))

while True:
    line = uart.readline()
    if line:
        try:
            s = line.decode('ascii').strip()
        except:
            continue
        if s.startswith('$GPGGA') or s.startswith('$GPRMC'):
            parts = s.split(',')
            # GPGGA: lat/lon are parts[2], parts[3], parts[4], parts[5]
            lat = nmea_to_decimal(parts[2], parts[3])
            lon = nmea_to_decimal(parts[4], parts[5])
            if lat is not None and lon is not None:
                # Outputs coordinate data in (latitude, longitude) format
                sys.stdout.buffer.write(f"{lat:.6f}, {lon:.6f}")
        else:
            sys.stdout.buffer.write("Couldn't get a fix\n")
            time.sleep(1)

