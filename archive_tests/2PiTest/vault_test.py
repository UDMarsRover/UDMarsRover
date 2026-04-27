import serial # Toolbox for the pins
import time   # Toolbox for the clock

ser = serial.Serial('/dev/serial0', 9600, timeout=1) 

print("VAULT: Waiting...")
try:
    while True:
        if ser.in_waiting > 0:
            # Read and clean the incoming data
            data = ser.readline().decode('utf-8').strip()
            print(f"ALARM: {data}")
            # Send a reply back to the Rover
            ser.write(b"ACCESS DENIED\n")
        
        # CPU breather - MUST be inside while, but outside if
        time.sleep(0.1) 
except KeyboardInterrupt:
    print("\nShutting down Vault...")
    ser.close()

