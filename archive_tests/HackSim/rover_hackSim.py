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

print("ROVER: Starting Heist Sequence...")
ser.write(b"REQUEST_ACCESS\n")

while True:
    dtype, data = universal_read()
    
    if dtype == "STRING":
        print(f"ROVER LOG: Vault said -> {data}")
        if data == "PROMPT_PASSCODE":
            print("Action: Sending Code 1234")
            ser.write(b"1234\n")
        elif data == "SUCCESS_UNLOCKED":
            print("MISSION SUCCESS: Vault is open!")
            break
        elif data == "INVALID_CREDENTIALS":
            print("Action: Retrying request...")
            ser.write(b"REQUEST_ACCESS\n")

    elif dtype == "HEX":
        print(f"ROVER LOG: Vault sent Raw Bytes -> 0x{data}")

    time.sleep(0.1)





