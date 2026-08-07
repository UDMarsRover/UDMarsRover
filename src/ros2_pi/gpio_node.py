import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from gpiozero import DigitalOutputDevice

class GpioSubscriber(Node):
    def __init__(self):
        super().__init__('gpio_subscriber')
        
        # BCM Pin number (change 17 to whatever pin you are using)
        self.pin_number = 17
        self.device = DigitalOutputDevice(self.pin_number)
        
        # Create a subscription to the /pin_cmd topic
        self.subscription = self.create_subscription(
            Bool,
            'headlights',
            self.listener_callback,
            10
        )
        self.get_logger().info(f'Listening on /headlights to control GPIO {self.pin_number}')

    def listener_callback(self, msg: Bool):
        if msg.data:
            self.device.on()
            self.get_logger().info('Pin turned ON')
        else:
            self.device.off()
            self.get_logger().info('Pin turned OFF')

def main(args=None):
    rclpy.init(args=args)
    node = GpioSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure the pin is turned off when the node shuts down
        node.device.off()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
