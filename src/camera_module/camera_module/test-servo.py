import time
import gpiod

# BCM pin 12 corresponds to GPIO chip 0, line 12 on most Raspberry Pi models.
# You can verify this with the command `gpioinfo`.
# If you are using a different pin, you'll need to change this line.
LINE_PIN = 12

# Find the GPIO chip, which is usually `gpiochip0` on a Raspberry Pi
# and get the specific line we want to use.
try:
    chip = gpiod.Chip('gpiochip0')
    line = chip.get_line(LINE_PIN)
except FileNotFoundError:
    print("Error: Could not find GPIO chip. Is this running on a Raspberry Pi?")
    exit(1)
except Exception as e:
    print(f"An error occurred: {e}")
    exit(1)

# Request the line for output with an initial value of 0.
# The consumer is a descriptive name for the application using the line.
line.request(consumer='servo-control', type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])

# Unlike RPi.GPIO, gpiod doesn't have a direct PWM function in its Python library.
# The easiest way to achieve software PWM is to manually toggle the pin.
# This approach is suitable for servo control, which doesn't require high-frequency PWM.
def set_angle(angle):
    # Map the angle (0-180) to a duty cycle pulse width in microseconds (500us to 2500us).
    pulse_width_us = 500 + (angle * 2000 / 180)
    
    # Calculate the number of cycles for the 50Hz (20ms) period.
    # The servo pulse requires a high period and a low period.
    period_us = 20000
    high_time_us = pulse_width_us
    low_time_us = period_us - high_time_us
    
    # Perform the software PWM pulse
    line.set_value(1)
    time.sleep(high_time_us / 1000000.0)
    line.set_value(0)
    time.sleep(low_time_us / 1000000.0)

try:
    while True:
        # Move to 0 degrees
        set_angle(0)
        time.sleep(1)
        # Move to 180 degrees
        set_angle(180)
        time.sleep(1)
except KeyboardInterrupt:
    pass
finally:
    # Release the line and chip.
    line.release()
    chip.close()