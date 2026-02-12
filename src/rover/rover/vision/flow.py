import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy.spatial import cKDTree

class LightweightOpticalFlow(Node):
    def __init__(self):
        super().__init__('optical_flow_node')

        # Subscriber for the global shutter camera
        self.subscription = self.create_subscription(
            Image,
            'nav_camera/image_raw',
            self.image_callback,
            10
        )

        # Publisher for the debug/segmentation view
        self.debug_publisher = self.create_publisher(Image, 'nav_camera/debug_flow', 10)

        self.bridge = CvBridge()

        # State variables
        self.prev_gray = None
        self.prev_points = None

        # Performance Settings for Raspberry Pi
        self.width, self.height = 320, 240
        self.min_features = 40

        # Lucas-Kanade and Feature Parameters
        self.feature_params = dict(maxCorners=200, qualityLevel=0.01, minDistance=7, blockSize=7)
        self.lk_params = dict(winSize=(21, 21), maxLevel=3, 
                              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))

        self.get_logger().info("Lightweight Optical Flow Node Initialized.")

    def image_callback(self, msg):
        try:
            # 1. Pre-processing
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame_small = cv2.resize(frame, (self.width, self.height))
            gray = cv2.cvtColor(frame_small, cv2.COLOR_BGR2GRAY)

            # 2. Masking (Focusing on the ground to ignore the horizon/sky)
            mask = np.zeros_like(gray)
            mask[int(self.height * 0.4):, :] = 255  # Bottom 60% of image

            # 3. Handle Feature Points
            if self.prev_points is None or len(self.prev_points) < self.min_features:
                self.prev_points = cv2.goodFeaturesToTrack(gray, mask=mask, **self.feature_params)
                self.prev_gray = gray
                return

            # 4. Calculate Optical Flow
            next_points, status, _ = cv2.calcOpticalFlowPyrLK(
                self.prev_gray, gray, self.prev_points, None, **self.lk_params
            )

            if next_points is not None and status is not None:
                good_new = next_points[status == 1]
                good_old = self.prev_points[status == 1]

                # 5. Global Motion Compensation (Subtracting the "Turn" effect)
                dx_all = good_new[:, 0] - good_old[:, 0]
                dy_all = good_new[:, 1] - good_old[:, 1]

                median_dx = np.median(dx_all)
                median_dy = np.median(dy_all)

                # 6. Isolate Residual Motion (Relative to the background)
                residual_data = []
                for p_new, p_old in zip(good_new, good_old):
                    rdx = (p_new[0] - p_old[0]) - median_dx
                    rdy = (p_new[1] - p_old[1]) - median_dy
                    residual_data.append([p_new[0], p_new[1], rdx, rdy])

                # 7. Lightweight Clustering (Segmentation)
                self.process_clusters(np.array(residual_data), frame_small)

                # Update State
                self.prev_gray = gray.copy()
                self.prev_points = good_new.reshape(-1, 1, 2)

            # 8. Publish Debug View
            debug_msg = self.bridge.cv2_to_imgmsg(frame_small, encoding='bgr8')
            self.debug_publisher.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {e}")

    def process_clusters(self, data, debug_frame):
        """Group points into obstacles based on proximity."""
        if len(data) == 0:
            return

        # Use cKDTree for efficient clustering
        points = data[:, :2]
        tree = cKDTree(points)
        clusters = tree.query_ball_tree(tree, r=35.0)  # Group points within 35 pixels

        # Analyze and draw each cluster
        for cluster_indices in clusters:
            cluster = data[cluster_indices]
            if len(cluster) < 6:  # Minimum points to be considered an obstacle
                continue

            # Magnitude of motion NOT explained by robot turning
            avg_res_mag = np.mean(np.sqrt(cluster[:, 2]**2 + cluster[:, 3]**2))

            # If residual motion is high, it's a 'looming' obstacle
            if avg_res_mag > 1.3:
                x_min, y_min = np.min(cluster[:, :2], axis=0)
                x_max, y_max = np.max(cluster[:, :2], axis=0)

                # Draw Obstacle Box
                cv2.rectangle(debug_frame, (int(x_min), int(y_min)), 
                              (int(x_max), int(y_max)), (0, 0, 255), 2)
                cv2.putText(debug_frame, "OBSTACLE", (int(x_min), int(y_min)-5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

def main(args=None):
    rclpy.init(args=args)
    node = LightweightOpticalFlow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()