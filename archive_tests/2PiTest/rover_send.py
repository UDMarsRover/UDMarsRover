import serial
import time

# Open the port at 9600 speed
ser = serial.Serial('/dev/serial0', 9600, timeout=1)

print("ROVER: Sending heist code...")
ser.write(b"OPEN_SESAME\n") # The 'b' is for bytes

# Logic to wait for a reply for 3 seconds
start_time = time.time()
while (time.time() - start_time) < 3:
    if ser.in_waiting > 0:
        reply = ser.readline().decode('utf-8').strip()
        print(f"VAULT REPLIED: {reply}")
        break

    # CPU breather
    time.sleep(0.01)
else:
    print("TIMEOUT: No response from Vault.")

ser.close()





