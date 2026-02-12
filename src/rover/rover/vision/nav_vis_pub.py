import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

#!/usr/bin/env python3

CAMERA_PATH = "/dev/video10"  # Adjust this if your camera is at a different path

class NavVisPub(Node):
    def __init__(self):
        super().__init__('nav_vis_pub')
        
        # Publisher for the image topic
        self.publisher_ = self.create_publisher(Image, 'nav_camera/image_raw', 10)
        
        # Timer to trigger the callback periodically
        # Assuming 30 FPS, period is roughly 0.033 seconds
        self.timer = self.create_timer(0.033, self.timer_callback)
        
        # OpenCV bridge to convert CV2 images to ROS messages
        self.bridge = CvBridge()
        
        # Initialize video capture (0 is usually the default USB camera)
        # You might need to change the index if you have multiple cameras
        self.cap = cv2.VideoCapture(CAMERA_PATH, cv2.CAP_V4L2)
        
        # Set camera properties if needed (optional)
        # 3: CAP_PROP_FRAME_WIDTH, 4: CAP_PROP_FRAME_HEIGHT
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        if not self.cap.isOpened():
            self.get_logger().error('Could not open video device')

    def timer_callback(self):
        ret, frame = self.cap.read()
        
        if ret:
            # Convert the OpenCV image to a ROS Image message
            # 'bgr8' is the standard color encoding for OpenCV images
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'nav_camera_optical_frame'
            
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn('Failed to capture frame')

    def __del__(self):
        # Release the camera resource on exit
        if self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    nav_vis_pub = NavVisPub()
    
    try:
        rclpy.spin(nav_vis_pub)
    except KeyboardInterrupt:
        pass
    finally:
        nav_vis_pub.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()