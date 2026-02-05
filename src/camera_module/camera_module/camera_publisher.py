#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from std_srvs.srv import SetBool
import cv2
from cv_bridge import CvBridge
import threading
import time

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        # Declare parameters
        self.declare_parameter('camera_devices', ['/dev/video0'])
        self.declare_parameter('camera_names', ['camera_0'])
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('jpeg_quality', 80)
        
        # Get parameters
        self.camera_devices = self.get_parameter('camera_devices').value
        self.camera_names = self.get_parameter('camera_names').value
        self.frame_rate = self.get_parameter('frame_rate').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        
        # Validate configuration
        if len(self.camera_names) != len(self.camera_devices):
            if len(self.camera_names) < len(self.camera_devices):
                # Auto-generate missing names
                for i in range(len(self.camera_names), len(self.camera_devices)):
                    self.camera_names.append(f'camera_{i}')
            else:
                self.get_logger().warn(f'More names than devices, using first {len(self.camera_devices)} names')
                self.camera_names = self.camera_names[:len(self.camera_devices)]
        
        self.bridge = CvBridge()
        
        # Camera state tracking
        self.cameras = {}
        
        for device, name in zip(self.camera_devices, self.camera_names):
            self.cameras[name] = {
                'device': device,
                'capture': None,
                'enabled': False,
                'thread': None,
                'stop_flag': threading.Event(),
                'raw_pub': self.create_publisher(Image, f'camera/{name}/raw', 10),
                'compressed_pub': self.create_publisher(CompressedImage, f'camera/{name}/compressed', 10),
                'info_pub': self.create_publisher(CameraInfo, f'camera/{name}/camera_info', 10),
                'service': self.create_service(
                    SetBool,
                    f'camera/{name}/enable',
                    lambda req, resp, cam_name=name: self.handle_enable_camera(req, resp, cam_name)
                )
            }
        
        self.get_logger().info(f'Camera publisher initialized with {len(self.cameras)} cameras')
        for name, cam in self.cameras.items():
            self.get_logger().info(f'  - {name}: {cam["device"]} (disabled by default)')
            self.get_logger().info(f'    Enable: ros2 service call /camera/{name}/enable std_srvs/srv/SetBool "{{data: true}}"')
    
    def handle_enable_camera(self, request, response, camera_name):
        """Service callback to enable/disable camera streaming"""
        camera = self.cameras[camera_name]
        
        if request.data:
            # Enable camera
            if camera['enabled']:
                response.success = True
                response.message = f'Camera {camera_name} already enabled'
                return response
            
            # Open camera device - try different methods
            device = camera['device']
            cap = None
            
            # Try as device path first
            if isinstance(device, str) and device.startswith('/dev/'):
                self.get_logger().info(f'Attempting to open {device}')
                cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
                
                if not cap.isOpened():
                    # Try without V4L2
                    cap.release()
                    cap = cv2.VideoCapture(device)
            # Try as integer index
            elif isinstance(device, int) or (isinstance(device, str) and device.isdigit()):
                idx = int(device) if isinstance(device, str) else device
                self.get_logger().info(f'Attempting to open camera index {idx}')
                cap = cv2.VideoCapture(idx)
            else:
                cap = cv2.VideoCapture(device)
            
            if cap is None or not cap.isOpened():
                if cap is not None:
                    cap.release()
                response.success = False
                response.message = f'Failed to open camera device {device}. Check permissions (sudo usermod -a -G video $USER)'
                self.get_logger().error(response.message)
                return response
            
            # Configure camera
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_FPS, self.frame_rate)
            
            # Verify camera is working by reading one frame
            ret, test_frame = cap.read()
            if not ret:
                cap.release()
                response.success = False
                response.message = f'Camera opened but failed to read test frame'
                self.get_logger().error(response.message)
                return response
            
            camera['capture'] = cap
            camera['enabled'] = True
            camera['stop_flag'].clear()
            
            # Start capture thread
            camera['thread'] = threading.Thread(
                target=self.capture_loop,
                args=(camera_name,),
                daemon=True
            )
            camera['thread'].start()
            
            response.success = True
            response.message = f'Camera {camera_name} enabled'
            self.get_logger().info(response.message)
        
        else:
            # Disable camera
            if not camera['enabled']:
                response.success = True
                response.message = f'Camera {camera_name} already disabled'
                return response
            
            # Stop capture thread
            camera['stop_flag'].set()
            if camera['thread'] is not None:
                camera['thread'].join(timeout=2.0)
            
            # Release camera
            if camera['capture'] is not None:
                camera['capture'].release()
                camera['capture'] = None
            
            camera['enabled'] = False
            
            response.success = True
            response.message = f'Camera {camera_name} disabled'
            self.get_logger().info(response.message)
        
        return response
    
    def capture_loop(self, camera_name):
        """Capture and publish frames from camera"""
        camera = self.cameras[camera_name]
        cap = camera['capture']
        
        frame_period = 1.0 / self.frame_rate
        
        while not camera['stop_flag'].is_set():
            loop_start = time.time()
            
            ret, frame = cap.read()
            if not ret:
                self.get_logger().error(f'Failed to read frame from {camera_name}')
                time.sleep(0.1)
                continue
            
            timestamp = self.get_clock().now().to_msg()
            
            # Publish raw image
            if camera['raw_pub'].get_subscription_count() > 0:
                try:
                    raw_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                    raw_msg.header.stamp = timestamp
                    raw_msg.header.frame_id = camera_name
                    camera['raw_pub'].publish(raw_msg)
                except Exception as e:
                    self.get_logger().error(f'Failed to publish raw image: {e}')
            
            # Publish compressed image
            if camera['compressed_pub'].get_subscription_count() > 0:
                try:
                    compressed_msg = CompressedImage()
                    compressed_msg.header.stamp = timestamp
                    compressed_msg.header.frame_id = camera_name
                    compressed_msg.format = 'jpeg'
                    
                    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
                    _, buffer = cv2.imencode('.jpg', frame, encode_param)
                    compressed_msg.data = buffer.tobytes()
                    
                    camera['compressed_pub'].publish(compressed_msg)
                except Exception as e:
                    self.get_logger().error(f'Failed to publish compressed image: {e}')
            
            # Publish camera info
            if camera['info_pub'].get_subscription_count() > 0:
                info_msg = CameraInfo()
                info_msg.header.stamp = timestamp
                info_msg.header.frame_id = camera_name
                info_msg.width = self.width
                info_msg.height = self.height
                camera['info_pub'].publish(info_msg)
            
            # Maintain frame rate
            elapsed = time.time() - loop_start
            sleep_time = frame_period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    def cleanup(self):
        """Clean up all cameras"""
        for name, camera in self.cameras.items():
            if camera['enabled']:
                camera['stop_flag'].set()
                if camera['thread'] is not None:
                    camera['thread'].join(timeout=2.0)
                if camera['capture'] is not None:
                    camera['capture'].release()
                self.get_logger().info(f'Released camera {name}')

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
