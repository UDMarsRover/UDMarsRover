from machine import UART, Pin
import machine
import time
import sys

def nmea_to_decimal(coord_str, direction):
    # coord_str like '4807.038' or '01131.000'
    if not coord_str or coord_str == '':
        return None
    if '.' not in coord_str:
        return None
    deg_len = 2 if direction in ['N', 'S'] else 3
    try:
        degrees = float(coord_str[:deg_len])
        minutes = float(coord_str[deg_len:])
    except:
        return None
    decimal = degrees + minutes / 60
    if direction in ['S', 'W']:
        decimal *= -1
    return decimal

# SetGPS UART
gps_uart = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))

# Geiger UART
geiger_uart = UART(1, baudrate=115200, tx=Pin(8), rx=Pin(9))

###Setup voltage reader###
# Define the GPIO pin connected to your analog signal.
adc_pin = machine.ADC(27)

# Define the maximum voltage for your scaled output
MAX_SCALED_VOLTAGE = 48.0

# The Pico's ADC has a 16-bit resolution (0-65535)
ADC_MAX_VALUE = 65535

#geiger = "None"

waiting = False

while True:
#----------#### Get and Send GPS Data ####----------#
    gps_data = gps_uart.readline()
    if gps_data:
        try:
            s = gps_data.decode('ascii').strip()
            print(f"GPS: Satelites: " + parts[50])
        except:
            continue
        if s.startswith('$GPGGA') or s.startswith('$GPRMC'):
            parts = s.split(',')
            # GPGGA: lat/lon are parts[2], parts[3], parts[4], parts[5]
            lat = nmea_to_decimal(parts[2], parts[3])
            lon = nmea_to_decimal(parts[4], parts[5])
            if lat is not None and lon is not None:
                sys.stdout.buffer.write(f"GPS: {lat:.6f}, {lon:.6f}")
                waiting = False
        else:
            if waiting == False:
                sys.stdout.buffer.write("GPS: Waiting for a fix\n")
                waiting = True
            time.sleep(0.01)
            
    else:
        sys.stdout.buffer.write("GPS: No data\n")

#----------#### Get and Send Voltage Data ####----------#
    # Read the raw 16-bit integer value from the ADC pin
    raw_value = adc_pin.read_u16()
    
    # Convert the raw ADC value to the scaled 48V range using a ratio.
    scaled_voltage = ((raw_value / ADC_MAX_VALUE) * MAX_SCALED_VOLTAGE) - 0.45
    
    # Print the result to the console, formatted to 2 decimal places
    sys.stdout.buffer.write(f"Volt: {scaled_voltage:.2f}V\n")
    
    
    # Wait for 0.01 seconds before taking the next reading
    time.sleep(0.01)


#----------#### Get and Send Geiger Data ####----------#
    # Get geiger data
    geiger_data = geiger_uart.readline()
    #sys.stdout.buffer.write(f"Raw: {geiger_data}\n")
    if geiger_data:
        try:
            g = geiger_data.decode('ascii').strip()
            #g = str(g)
            sys.stdout.buffer.write(f"GEIGER: {g}\n")
        except:
            # Handle bad data
            sys.stdout.buffer.write("GEIGER: Received invalid data\n")
            continue
    time.sleep(0.1)

