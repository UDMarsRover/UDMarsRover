"""
This script is designed to pull coordinate data from the COM port a Raspberry Pi Pico or other device is connected to and push it to ROS. The
script assumes that data is pushed in (latitude, longitude) format
"""

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import String

class GPS_Publisher(Node):

    def __init__(self, serial):
        super().__init__('gps_node')
        self.publisher_ = self.create_publisher(String, 'gps_pub', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.serial = serial

    def timer_callback(self):
        msg = String()
        lat, lon = None, None

        line = self.serial.readline()
        if line:
            line_d = line.decode('ascii')
            try:
                lat, lon = line_d.split(",")
            except:
                pass
            if lat is not None and lon is not None:
                msg.data = str(lat) + ", " + str(lon)
                self.publisher_.publish(msg)
                self.get_logger().info('Lat, long: %s' % msg.data)
            else:
                msg.data = "No fix"
                self.publisher_.publish(msg)
        else:
            msg.data = "No data"
            self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    ser = serial.Serial()

    ser.port = '/dev/ttyACM0'
    ser.baudrate = 9600
    try:
        ser.open()
    except:
        print("Device not found; Check port in terminal and validate that device is connected properly.")
        
    gps_pub = GPS_Publisher(ser)

    while True:
        rclpy.spin(gps_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gps_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()