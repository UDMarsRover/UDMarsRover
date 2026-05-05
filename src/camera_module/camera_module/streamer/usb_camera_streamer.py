import glob
import re
import threading
import time
from typing import Dict, Optional
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

def _video_device_sort_key(path: str):
    match = re.search(r"(\d+)$", path)
    return int(match.group(1)) if match else path


class CameraWorker:
    def __init__(
        self,
        node: Node,
        device_path: str,
        camera_index: int,
        width: int,
        height: int,
        fps: float,
        jpeg_quality: int,
    ) -> None:
        self.node = node
        self.device_path = device_path
        self.camera_index = camera_index
        self.width = width
        self.height = height
        self.fps = fps
        self.jpeg_quality = jpeg_quality

        self.bridge = CvBridge()
        self.capture: Optional[cv2.VideoCapture] = None
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

        base_topic = f"video_{self.camera_index}"
        self.raw_pub = self.node.create_publisher(
            Image,
            f"{base_topic}/raw",
            qos_profile_sensor_data,
        )
        self.compressed_pub = self.node.create_publisher(
            CompressedImage,
            f"{base_topic}/compressed",
            qos_profile_sensor_data,
        )

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        if self.thread.is_alive():
            self.thread.join(timeout=2.0)
        self._close_capture()

    def _open_capture(self) -> bool:
        self._close_capture()

        self.capture = cv2.VideoCapture(self.device_path, cv2.CAP_V4L2)
        if not self.capture.isOpened():
            self._close_capture()
            return False

        if self.width > 0:
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        if self.height > 0:
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        if self.fps > 0:
            self.capture.set(cv2.CAP_PROP_FPS, self.fps)

        return True

    def _close_capture(self) -> None:
        if self.capture is not None:
            self.capture.release()
            self.capture = None

    def _publish_frame(self, frame) -> None:
        stamp = self.node.get_clock().now().to_msg()
        frame_id = f"video_{self.camera_index}"

        raw_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        raw_msg.header.stamp = stamp
        raw_msg.header.frame_id = frame_id
        self.raw_pub.publish(raw_msg)

        ok, encoded = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), int(self.jpeg_quality)],
        )
        if ok:
            compressed_msg = CompressedImage()
            compressed_msg.header.stamp = stamp
            compressed_msg.header.frame_id = frame_id
            compressed_msg.format = "jpeg"
            compressed_msg.data = encoded.tobytes()
            self.compressed_pub.publish(compressed_msg)

    def _run(self) -> None:
        frame_period = 1.0 / self.fps if self.fps > 0 else 0.0

        while not self.stop_event.is_set():
            if self.capture is None or not self.capture.isOpened():
                if not self._open_capture():
                    time.sleep(1.0)
                    continue

            loop_start = time.monotonic()
            ret, frame = self.capture.read()

            if not ret or frame is None:
                self.node.get_logger().warning(
                    f"Read failed for {self.device_path}, retrying..."
                )
                self._close_capture()
                time.sleep(0.5)
                continue

            self._publish_frame(frame)

            if frame_period > 0:
                elapsed = time.monotonic() - loop_start
                sleep_time = frame_period - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)


class UsbCameraStreamer(Node):
    def __init__(self) -> None:
        super().__init__("usb_camera_streamer")

        self.declare_parameter("scan_period", 5.0)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 15.0)
        self.declare_parameter("jpeg_quality", 80)

        self.scan_period = float(self.get_parameter("scan_period").value)
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.fps = float(self.get_parameter("fps").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)

        self.cameras: Dict[str, CameraWorker] = {}
        self.next_camera_index = 1

        self.scan_timer = self.create_timer(self.scan_period, self.scan_devices)
        self.scan_devices()

    def _is_usable_device(self, device_path: str) -> bool:
        cap = cv2.VideoCapture(device_path, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            return False

        ret, frame = cap.read()
        cap.release()
        return bool(ret and frame is not None)

    def scan_devices(self) -> None:
        found_devices = sorted(
            glob.glob("/dev/video*"),
            key=_video_device_sort_key,
        )
        found_set = set(found_devices)
        active_set = set(self.cameras.keys())

        removed_devices = active_set - found_set
        for device_path in removed_devices:
            self.get_logger().info(f"Camera removed: {device_path}")
            self.cameras[device_path].stop()
            del self.cameras[device_path]

        new_devices = [d for d in found_devices if d not in self.cameras]
        for device_path in new_devices:
            if not self._is_usable_device(device_path):
                continue

            camera_index = self.next_camera_index
            self.next_camera_index += 1

            worker = CameraWorker(
                node=self,
                device_path=device_path,
                camera_index=camera_index,
                width=self.width,
                height=self.height,
                fps=self.fps,
                jpeg_quality=self.jpeg_quality,
            )
            self.cameras[device_path] = worker
            worker.start()

            self.get_logger().info(
                f"Streaming {device_path} on topics "
                f"'video_{camera_index}/raw' and 'video_{camera_index}/compressed'"
            )

    def destroy_node(self):
        for worker in list(self.cameras.values()):
            worker.stop()
        self.cameras.clear()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UsbCameraStreamer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()