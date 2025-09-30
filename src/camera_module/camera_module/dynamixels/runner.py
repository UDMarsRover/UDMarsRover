from control import DynamixelMX
import time

motor = DynamixelMX('/dev/ttyAMA0', 1, 57600)
while True:
    motor.write_goal_position(int(motor.get_max_position()))
    time.sleep(1.5)
    motor.write_goal_position(int(motor.get_min_position()))
    # time.sleep(1.5)
    # motor.write_goal_position(motor.get_min_position())
    time.sleep(1.5)