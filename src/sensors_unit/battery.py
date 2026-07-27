import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import asyncio
import threading
from bleak import BleakClient

MAC_ADDRESS = "30:55:44:3A:99:2E"
NOTIFY_CHAR_UUID = "0000ffe4-0000-1000-8000-00805f9b34fb"
WRITE_CHAR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"

# Request ping to keep the BLE connection awake
REQUEST_HEX = bytearray.fromhex("DDA50300FFFD77")

class CanbatNode(Node):
    def __init__(self):
        super().__init__('canbat_battery')
        self.pub = self.create_publisher(BatteryState, 'battery_state', 10)
        
        self.state = BatteryState()
        self.state.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIFE
        
        # String buffer to stitch the ASCII stream together
        self.text_buffer = ""
        
        self.create_timer(1.0, self.publish_state)
        threading.Thread(target=self.run_ble_loop, daemon=True).start()

    def publish_state(self):
        self.state.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.state)

    def ble_notification_handler(self, sender, data: bytearray):
        # Decode and clean raw bytes
        text_chunk = data.decode('ascii', errors='ignore').replace('\x00', '')
        self.text_buffer += text_chunk
        
        # Wait for the '^' start character
        if '^' in self.text_buffer:
            start_idx = self.text_buffer.find('^')
            self.text_buffer = self.text_buffer[start_idx:]
            
            # The payload we care about is 61 chars long
            if len(self.text_buffer) >= 61:
                packet = self.text_buffer[:61]
                
                try:
                    def parse_32bit_le(hex_str):
                        rev_hex = hex_str[6:8] + hex_str[4:6] + hex_str[2:4] + hex_str[0:2]
                        return int(rev_hex, 16)
                        
                    def parse_16bit_le(hex_str):
                        rev_hex = hex_str[2:4] + hex_str[0:2]
                        return int(rev_hex, 16)
                    
                    # 1. Parse Core Specs
                    voltage_mv = parse_32bit_le(packet[1:9])
                    rem_cap_mah = parse_32bit_le(packet[9:17])
                    total_cap_mah = parse_32bit_le(packet[17:25])
                    
                    # 2. Parse Temperature (0.1 Kelvin format -> 273.1 K = 0°C)
                    temp_raw_k = parse_16bit_le(packet[33:37])
                    temp_c = (temp_raw_k - 2731) / 10.0
                    
                    # 3. Parse Cell Voltages
                    cell1_mv = parse_16bit_le(packet[45:49])
                    cell2_mv = parse_16bit_le(packet[49:53])
                    cell3_mv = parse_16bit_le(packet[53:57])
                    cell4_mv = parse_16bit_le(packet[57:61])
                    
                    # 4. Populate ROS 2 Message Fields natively
                    self.state.voltage = voltage_mv / 1000.0
                    self.state.capacity = rem_cap_mah / 1000.0
                    self.state.design_capacity = total_cap_mah / 1000.0
                    self.state.temperature = float(temp_c)
                    
                    # Add to the official ROS 2 cell_voltage array
                    self.state.cell_voltage = [
                        cell1_mv / 1000.0, 
                        cell2_mv / 1000.0, 
                        cell3_mv / 1000.0, 
                        cell4_mv / 1000.0
                    ]
                    
                    if total_cap_mah > 0:
                        self.state.percentage = float(rem_cap_mah) / float(total_cap_mah)
                        
                    # Terminal UI
                    self.get_logger().info(
                        f"--> PARSED: {self.state.voltage:.2f}V | "
                        f"Cap: {self.state.capacity:.2f}Ah | "
                        f"Temp: {self.state.temperature:.1f}°C | "
                        f"SoC: {self.state.percentage * 100:.1f}%"
                    )
                    
                except ValueError as e:
                    # Downgraded from ERROR to WARN to handle RF noise gracefully
                    self.get_logger().warn(f"Dropped corrupted BLE frame (Normal RF interference): {e}")
                
                # Clear processed packet to catch the next one
                self.text_buffer = self.text_buffer[61:]

    def run_ble_loop(self):
        async def ble_task():
            while rclpy.ok():
                try:
                    async with BleakClient(MAC_ADDRESS) as client:
                        self.get_logger().info("Connected to Canbat Battery!")
                        await client.start_notify(NOTIFY_CHAR_UUID, self.ble_notification_handler)
                        
                        while rclpy.ok():
                            await client.write_gatt_char(WRITE_CHAR_UUID, REQUEST_HEX, response=False)
                            await asyncio.sleep(1.0)
                            
                except Exception as e:
                    self.get_logger().error(f"BLE connection dropped: {e}. Retrying in 5s...")
                    await asyncio.sleep(5.0)

        asyncio.run(ble_task())

def main(args=None):
    rclpy.init(args=args)
    node = CanbatNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
