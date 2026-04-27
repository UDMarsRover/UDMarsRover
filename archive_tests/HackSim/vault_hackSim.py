import serial
import time

# Pi 5 Hardware UART
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

print("VAULT: Monitoring Active. Waiting for Rover...")

while True:
    dtype, data = universal_read()
    
    if dtype == "STRING":
        print(f"VAULT LOG: Received -> {data}")
        if data == "REQUEST_ACCESS":
            ser.write(b"PROMPT_PASSCODE\n")
        elif data == "1234":
            ser.write(b"SUCCESS_UNLOCKED\n")
        else:
            ser.write(b"INVALID_CREDENTIALS\n")
            time.sleep(1)
            ser.write(b"PROMPT_PASSCODE\n") # Prompt again immediately

    elif dtype == "HEX":
        print(f"VAULT LOG: Received Raw/Mangled: 0x{data}")
        ser.write(b"ERROR_SIGNAL_UNREADABLE\n")

    time.sleep(0.1)



