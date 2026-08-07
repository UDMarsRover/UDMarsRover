import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import psutil
import subprocess
import json
import time

class PiMonitorNode(Node):
    def __init__(self):
        super().__init__('pi_sys_monitor')
        self.pub = self.create_publisher(String, 'pi_system_metrics', 10)
        
        # Publish every 2 seconds
        self.timer = self.create_timer(2.0, self.publish_metrics)

        # Baseline for network bandwidth calculations
        self.last_net_io = psutil.net_io_counters()
        self.last_time = time.time()
        
        self.get_logger().info('System Monitor Node started. Publishing to /pi_system_metrics')

    def get_vcgencmd_value(self, cmd):
        """Helper to get GPU and Voltage data securely."""
        try:
            # Requires libraspberrypi-bin and hardware privileges
            out = subprocess.check_output(['vcgencmd', cmd]).decode('utf-8').strip()
            # Example parses: "temp=45.0'C" -> "45.0" | "volt=0.8500V" -> "0.8500"
            return float(out.split('=')[1].replace('\'C', '').replace('V', ''))
        except Exception:
            return -1.0 # Return -1 if reading fails

    def get_cpu_temp(self):
        """Reads CPU temp directly from hardware files."""
        try:
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                return float(f.read().strip()) / 1000.0
        except Exception:
            return -1.0

    def publish_metrics(self):
        current_time = time.time()
        dt = current_time - self.last_time
        current_net_io = psutil.net_io_counters()

        # Calculate network speed in Megabytes per second (MB/s)
        rx_mbs = ((current_net_io.bytes_recv - self.last_net_io.bytes_recv) / dt) / (1024 * 1024)
        tx_mbs = ((current_net_io.bytes_sent - self.last_net_io.bytes_sent) / dt) / (1024 * 1024)

        self.last_net_io = current_net_io
        self.last_time = current_time

        # Gather all metrics
        metrics = {
            "cpu_usage_percent": psutil.cpu_percent(),
            "system_load_avg": psutil.getloadavg(), # Returns tuple: (1m, 5m, 15m)
            "ram_usage_percent": psutil.virtual_memory().percent,
            "cpu_temp_c": self.get_cpu_temp(),
            "gpu_temp_c": self.get_vcgencmd_value('measure_temp'),
            "cpu_freq_mhz": psutil.cpu_freq().current if psutil.cpu_freq() else 0.0,
            "voltage_v": self.get_vcgencmd_value('measure_volts core'),
            "net_rx_mbs": round(rx_mbs, 3),
            "net_tx_mbs": round(tx_mbs, 3)
        }

        # Publish as JSON string
        msg = String()
        msg.data = json.dumps(metrics)
        self.pub.publish(msg)

        # Log a summarized version to the terminal UI
        self.get_logger().info(
            f"Sys | CPU: {metrics['cpu_usage_percent']}% "
            f"RAM: {metrics['ram_usage_percent']}% "
            f"Temp: {metrics['cpu_temp_c']:.1f}C "
            f"Net: {metrics['net_rx_mbs']:.2f}MB/s ?"
        )

def main(args=None):
    rclpy.init(args=args)
    node = PiMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
