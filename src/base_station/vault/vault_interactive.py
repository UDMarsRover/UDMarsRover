import serial
import time

ser = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)

def universal_read():
    if ser.in_waiting > 0:
        raw_payload = ser.read(ser.in_waiting)
        try:
            decoded_str = raw_payload.decode('utf-8').strip()
            return ("STRING", decoded_str)
        except UnicodeDecodeError:
            return ("HEX", raw_payload.hex().upper())
    return (None, None)

# Set the "Secret" here
SECRET_CODE = "1234"

print(f"VAULT: Guarding secret '{SECRET_CODE}'. System Live.")

while True:
    dtype, data = universal_read()
    
    if dtype == "STRING":
        if data == "REQUEST_ACCESS":
            print("LOG: Rover is at the door. Prompting for pass...")
            ser.write(b"PROMPT_PASSCODE\n")
            
        elif data == SECRET_CODE:
            print(f"LOG: Correct code '{data}' received!")
            ser.write(b"SUCCESS_UNLOCKED\n")
            
        else:
            print(f"LOG: Unauthorized entry attempt with code: {data}")
            ser.write(b"INVALID_CREDENTIALS\n")

    time.sleep(0.1)
