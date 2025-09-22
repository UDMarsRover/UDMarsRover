# serial_protocol.py

from enum import Enum

class CommandType(Enum):
    LED = "L"
    MOTOR = "M"
    TELEMETRY = "T"
    STATUS = "S"

def construct_message(command, data=None):
    # Construct a message to be sent to the motor controller
    if data:
        return f"{command}{data}".encode()
    else:
        return command.encode()


def read_message(serial_connection):
    # Read a message from the motor controller
    if serial_connection and serial_connection.is_open:
        response = serial_connection.readline().decode().strip()
        return response
    else:
        return None
