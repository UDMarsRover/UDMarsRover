from enum import Enum
import serial
import numpy as np

class ExamplsStatusMessage(Enum):
    VOLTAGE = 12.0
    CURRENT = 3.0
    TEMPERATURE = 25.0
    VELOCITY = 100.0

class Connection(Enum):
    PORT = '/dev/ttyUSB0'
    BAUDRATE = 115200

class ControlMessage(Enum):
    VELOCITY_SET = 0x01
    IDLE_MODE = 0x02
    CURRENT_LIMIT = 0x03
    CLEAR_FAULTS = 0x04

class StatusMessage(Enum):
    VELOCITY = 0x85
    OTHER = 0x86

class IdleMode(Enum):
    COAST = 0x00
    BRAKE = 0x01

class UDMRTMotorSerial:
    def __init__(self, port = Connection.PORT.value, baudrate = Connection.BAUDRATE.value, connected = True, motor_count = 6):
        self.port = port
        self.baudrate = baudrate
        self.motor_count = motor_count
        self.connected = connected
        self.serial_conn = None

    def connect(self):
        try:
            self.serial_conn = serial.Serial(self.port, self.baudrate)
            self.serial_conn.flushInput()
            self.serial_conn.flushOutput()
            self.connected = True
        except serial.SerialException:
            return False
        return True
    
    @staticmethod
    def construct_velocity_set_packet(velocities: np.ndarray) -> bytearray:
        if len(velocities) != 6:
            raise ValueError("Velocities array must contain exactly 6 floats")
        packet = bytearray()
        packet.append(ControlMessage.VELOCITY_SET.value)
        for velocity in velocities:
            packet.extend(np.float32(velocity).tobytes())
        return packet
    
    @staticmethod
    def deconstruct_velocity_set_packet(packet: bytearray) -> np.ndarray:
        if len(packet) != 25:
            raise ValueError("Packet must contain exactly 25 bytes")
        velocities = np.zeros(6)
        for i in range(6):
            velocities[i] = np.frombuffer(packet, dtype=np.float32, count=1, offset=1 + 4*i)[0]
        return velocities
    
    @staticmethod
    def get_message_type(message: bytearray) -> Enum:
        return ControlMessage(message[0])
    
    def deconstruct_control_message(self, message: bytearray) -> np.ndarray:
        message_type = self.get_message_type(message)
        if message_type == ControlMessage.VELOCITY_SET:
            return self.deconstruct_velocity_set_packet(message)
        else:
            raise ValueError("Message type not recognized")
    
    @staticmethod
    def construct_idle_mode_packet(mode) -> bytearray:
        packet = bytearray()
        packet.append(ControlMessage.IDLE_MODE.value)
        packet.append(mode)
        while len(packet) < 25:
            packet.append(0x00)
        return packet
    
    def construct_brake_packet(self):
        packet = self.construct_idle_mode_packet(IdleMode.BRAKE.value)
        return packet

    def construct_idle_packet(self):
        packet = self.construct_idle_mode_packet(IdleMode.COAST.value)
        return packet
    
    @staticmethod
    def construct_velocity_status_frame():
        frame = bytearray()
        for _ in range(6):
            frame.extend(np.float32(ExamplsStatusMessage.VELOCITY.value).tobytes())
        return frame
    
    def construct_velocity_status_packet(self):
        packet = bytearray()
        packet.append(StatusMessage.VELOCITY.value)
        packet.extend(self.construct_velocity_status_frame())
        return packet
    
    @staticmethod
    def construct_other_status_frame():
        frame = bytearray()
        for _ in range(6):
            frame.extend(np.int8(ExamplsStatusMessage.TEMPERATURE.value).tobytes())
            voltage_fixed_point = int(ExamplsStatusMessage.VOLTAGE.value * (2**12 - 1) / 12.0)
            frame.extend(voltage_fixed_point.to_bytes(2, byteorder='big'))
            current_fixed_point = int(ExamplsStatusMessage.CURRENT.value * (2**12 - 1) / 3.0)
            frame.extend(current_fixed_point.to_bytes(2, byteorder='big'))
        return frame
    
    def construct_other_status_packet(self):
        packet = bytearray()
        packet.append(StatusMessage.OTHER.value)
        packet.extend(self.construct_other_status_frame())
        return packet
    
    def deconstruct_velocity_status_frame(self, frame: bytearray):
        velocities = np.zeros(6)
        for i in range(6):
            velocities[i] = np.frombuffer(frame, dtype=np.float32, count=1, offset=4*i)[0]
        return velocities
    
    def deconstruct_other_status_frame(self, frame: bytearray):
        stats = []
        for i in range(6):
            temperature = np.frombuffer(frame, dtype=np.int8, count=1, offset=4*i)[0]
            
            voltage_current_fixed_point = frame[1+4*i:4 + 4*i]
            # print(' '.join(f'{byte:02X}' for byte in voltage_current_fixed_point))
            voltage_fixed_point = ((voltage_current_fixed_point[1] & 0x0F) << 8) | voltage_current_fixed_point[0]
            
            # print(f"Voltage fixed point: {voltage_fixed_point:04X}")
            current_fixed_point = (voltage_current_fixed_point[2] << 4) | (voltage_current_fixed_point[1] >> 4)
            voltage = voltage_fixed_point * 32.0 / 4095
            voltage -= 0.02 * voltage
            
            current = current_fixed_point * 32.0 / 4095
            stats.append((temperature, voltage, current))
        return stats
    
    def deconstruct_velocity_status_packet(self, packet: bytearray):
        return self.deconstruct_velocity_status_frame(packet[1:])
    
    def deconstruct_other_status_packet(self, packet: bytearray):
        return self.deconstruct_other_status_frame(packet[1:])
    
    def parse_status_packet(self, packet: bytearray):
        try:
            status_type = StatusMessage(packet[0])
            if status_type == StatusMessage.VELOCITY:
                return self.deconstruct_velocity_status_packet(packet)
            elif status_type == StatusMessage.OTHER:
                return self.deconstruct_other_status_packet(packet)
            else:
                print(f"Received unknown status code: {packet[0]}")
                return None
        except ValueError:
            print("Invalid status packet received")

    def send_packet(self, packet: bytearray):
        if self.connected:
            self.serial_conn.write(packet)
        else:
            raise ValueError("Serial connection not open")
        
    def send_velocity_set(self, velocities: np.ndarray):
        packet = self.construct_velocity_set_packet(velocities)
        self.send_packet(packet)

    def send_brake(self):
        packet = self.construct_brake_packet()
        self.send_packet(packet)

    def send_idle(self):
        packet = self.construct_idle_packet()
        self.send_packet(packet)

    def read_packet(self):
        if self.connected:
            packet = self.serial_conn.read(25)
            return packet
        
    def spin_once(self):
        packet = self.read_packet()
        # print("Raw packet:", ' '.join(f'{byte:02X}' for byte in packet))
        if packet:
            parsed_data = self.parse_status_packet(packet)
            return parsed_data
        else:
            return -1, 0, 0

    def close(self):
        if self.connected:
            self.serial_conn.close()
            self.connected = False

def main():
    motor_controller = UDMRTMotorSerial(connected=False)
    packet = motor_controller.construct_velocity_status_packet()
    packet = motor_controller.construct_other_status_packet()
    parsed_data = motor_controller.parse_status_packet(packet)
    if isinstance(parsed_data[0], float):
        print("velocities: ", parsed_data)
    else:
        for i, (temperature, voltage, current) in enumerate(parsed_data):
            print(f"Motor {i+1} - temperature: {temperature}, voltage: {voltage}, current: {current}")

if __name__ == '__main__':
    main()