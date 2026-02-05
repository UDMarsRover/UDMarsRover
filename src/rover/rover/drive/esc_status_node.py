#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32, UInt16, Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_srvs.srv import SetBool

class ESCStatusNode(Node):
    def __init__(self):
        super().__init__('esc_status_node')
        
        # Declare parameters
        self.declare_parameter('num_motors', 6)
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('motor_names', ['motor_1', 'motor_2', 'motor_3', 'motor_4', 'motor_5', 'motor_6'])
        
        # Get parameters
        self.num_motors = self.get_parameter('num_motors').value
        self.motor_names = self.get_parameter('motor_names').value
        
        # Storage for latest status
        self.latest_status = None
        
        # Flag for individual telemetry publishing
        self.publish_individual_telemetry = False
        
        # Subscriber to wheel status
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'wheel_status',
            self.status_callback,
            10)
        
        # Publishers for individual motor diagnostics
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            '/drive/out/diagnostics',
            10)
        
        # Publishers for individual motor telemetry (created but only publish when enabled)
        self.velocity_pubs = []
        self.current_pubs = []
        self.voltage_pubs = []
        self.temp_pubs = []
        self.fault_pubs = []
        
        for i in range(self.num_motors):
            motor_name = self.motor_names[i] if i < len(self.motor_names) else f'motor_{i+1}'
            
            self.velocity_pubs.append(
                self.create_publisher(Float32, f'esc/{motor_name}/velocity', 10))
            self.current_pubs.append(
                self.create_publisher(Float32, f'esc/{motor_name}/current', 10))
            self.voltage_pubs.append(
                self.create_publisher(Float32, f'esc/{motor_name}/voltage', 10))
            self.temp_pubs.append(
                self.create_publisher(Float32, f'esc/{motor_name}/temperature', 10))
            self.fault_pubs.append(
                self.create_publisher(UInt16, f'esc/{motor_name}/faults', 10))
        
        # Service to enable/disable individual telemetry publishing
        self.enable_telemetry_service = self.create_service(
            SetBool,
            'esc/enable_individual_telemetry',
            self.handle_enable_telemetry)
        
        # Timer for publishing diagnostics
        publish_period = 1.0 / self.get_parameter('publish_rate').value
        self.timer = self.create_timer(publish_period, self.publish_diagnostics)
        
        self.get_logger().info(f'ESC Status node started with {self.num_motors} motors')
        self.get_logger().info('Individual telemetry publishing is DISABLED by default')
        self.get_logger().info('Call service: ros2 service call /esc/enable_individual_telemetry std_srvs/srv/SetBool "{data: true}"')
    
    def handle_enable_telemetry(self, request, response):
        """Service callback to enable/disable individual telemetry publishing"""
        self.publish_individual_telemetry = request.data
        response.success = True
        if request.data:
            response.message = 'Individual telemetry publishing enabled'
            self.get_logger().info('Individual telemetry publishing ENABLED')
        else:
            response.message = 'Individual telemetry publishing disabled'
            self.get_logger().info('Individual telemetry publishing DISABLED')
        return response
    
    def status_callback(self, msg: Float32MultiArray):
        """Store the latest wheel status data"""
        self.latest_status = list(msg.data)
    
    def publish_diagnostics(self):
        """Publish diagnostic messages and individual telemetry"""
        if self.latest_status is None:
            return
        
        # Expected format per motor: [Velocity, Position, Current, Voltage, Temp, Faults]
        values_per_motor = 6
        
        if len(self.latest_status) < self.num_motors * values_per_motor:
            self.get_logger().warn(f'Incomplete status data: expected {self.num_motors * values_per_motor}, got {len(self.latest_status)}')
            return
        
        # Create diagnostic array
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        for i in range(self.num_motors):
            base_idx = i * values_per_motor
            
            # Extract values
            velocity = self.latest_status[base_idx + 0]
            position = self.latest_status[base_idx + 1]
            current = self.latest_status[base_idx + 2]
            voltage = self.latest_status[base_idx + 3]
            temp = self.latest_status[base_idx + 4]
            faults = int(self.latest_status[base_idx + 5])
            
            # Publish individual telemetry topics only if enabled
            if self.publish_individual_telemetry:
                self.velocity_pubs[i].publish(Float32(data=velocity))
                self.current_pubs[i].publish(Float32(data=current))
                self.voltage_pubs[i].publish(Float32(data=voltage))
                self.temp_pubs[i].publish(Float32(data=temp))
                self.fault_pubs[i].publish(UInt16(data=faults))
            
            # Create diagnostic status
            motor_name = self.motor_names[i] if i < len(self.motor_names) else f'motor_{i+1}'
            diag_status = DiagnosticStatus()
            diag_status.name = f'ESC: {motor_name}'
            diag_status.hardware_id = f'motor_{i+1}'
            
            # Determine status level based on faults and temperature
            if faults != 0:
                diag_status.level = DiagnosticStatus.ERROR
                diag_status.message = f'Fault code: 0x{faults:04X}'
            elif temp > 80:
                diag_status.level = DiagnosticStatus.WARN
                diag_status.message = f'High temperature: {temp}°C'
            elif voltage < 10.0:
                diag_status.level = DiagnosticStatus.WARN
                diag_status.message = f'Low voltage: {voltage:.2f}V'
            else:
                diag_status.level = DiagnosticStatus.OK
                diag_status.message = 'Normal operation'
            
            # Add key-value pairs
            diag_status.values = [
                KeyValue(key='Velocity (RPM)', value=f'{velocity:.2f}'),
                KeyValue(key='Position (rot)', value=f'{position:.2f}'),
                KeyValue(key='Current (A)', value=f'{current:.2f}'),
                KeyValue(key='Voltage (V)', value=f'{voltage:.2f}'),
                KeyValue(key='Temperature (°C)', value=f'{temp:.0f}'),
                KeyValue(key='Fault Code', value=f'0x{faults:04X}'),
            ]
            
            diag_array.status.append(diag_status)
        
        # Publish diagnostics
        self.diagnostics_pub.publish(diag_array)

def main(args=None):
    rclpy.init(args=args)
    node = ESCStatusNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
