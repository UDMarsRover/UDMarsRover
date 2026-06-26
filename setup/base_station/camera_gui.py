#!/usr/bin/env python3
"""
ROS2 Humble Camera Stream GUI
-----------------------------
A PyQt5 GUI that subscribes to a sensor_msgs/Image topic and displays the
live stream. Includes topic switching, FPS counter, snapshot saving,
and start/stop controls.


Run inside (or alongside) your ROS2 Humble Docker container.


Usage:
    python3 camera_gui.py
    python3 camera_gui.py --topic /camera/image_raw
"""


import sys
import os
import argparse
import time
from threading import Thread, Lock


import cv2
import numpy as np


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge


from PyQt5 import QtCore, QtGui, QtWidgets




# ---------------------------------------------------------------------------
# ROS2 Node
# ---------------------------------------------------------------------------
class CameraSubscriber(Node):
    """ROS2 node that subscribes to a camera topic and forwards frames."""


    def __init__(self, topic: str = "/camera/image_raw", compressed: bool = False):
        super().__init__("camera_gui_subscriber")
        self.bridge = CvBridge()
        self.topic = topic
        self.compressed = compressed
        self._frame = None
        self._frame_lock = Lock()
        self._sub = None


        # QoS suitable for camera streams (best-effort, keep last)
        self.qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.subscribe(self.topic, self.compressed)


    def subscribe(self, topic: str, compressed: bool):
        """(Re)subscribe to a new topic."""
        if self._sub is not None:
            self.destroy_subscription(self._sub)
            self._sub = None


        self.topic = topic
        self.compressed = compressed
        msg_type = CompressedImage if compressed else Image
        cb = self._compressed_cb if compressed else self._raw_cb
        self._sub = self.create_subscription(msg_type, topic, cb, self.qos)
        self.get_logger().info(
            f"Subscribed to {topic} ({'CompressedImage' if compressed else 'Image'})"
        )


    def _raw_cb(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self._frame_lock:
                self._frame = cv_img
        except Exception as e:
            self.get_logger().error(f"cv_bridge raw error: {e}")


    def _compressed_cb(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            with self._frame_lock:
                self._frame = cv_img
        except Exception as e:
            self.get_logger().error(f"compressed decode error: {e}")


    def get_frame(self):
        with self._frame_lock:
            return None if self._frame is None else self._frame.copy()




# ---------------------------------------------------------------------------
# ROS2 spinner thread
# ---------------------------------------------------------------------------
class RosSpinThread(Thread):
    def __init__(self, node: Node):
        super().__init__(daemon=True)
        self.node = node
        self._stop = False


    def run(self):
        while not self._stop and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)


    def stop(self):
        self._stop = True




# ---------------------------------------------------------------------------
# PyQt5 GUI
# ---------------------------------------------------------------------------
class CameraGUI(QtWidgets.QMainWindow):
    def __init__(self, node: CameraSubscriber):
        super().__init__()
        self.node = node
        self.setWindowTitle("ROS2 Camera Stream Viewer")
        self.resize(1000, 720)


        # ---- Central image display ----
        self.image_label = QtWidgets.QLabel("Waiting for frames...")
        self.image_label.setAlignment(QtCore.Qt.AlignCenter)
        self.image_label.setStyleSheet(
            "background-color: #1e1e1e; color: #aaa; font-size: 16px;"
        )
        self.image_label.setMinimumSize(640, 480)


        # ---- Controls ----
        self.topic_edit = QtWidgets.QLineEdit(self.node.topic)
        self.compressed_check = QtWidgets.QCheckBox("Compressed")
        self.compressed_check.setChecked(self.node.compressed)
        self.subscribe_btn = QtWidgets.QPushButton("Subscribe")
        self.snapshot_btn = QtWidgets.QPushButton("Snapshot")
        self.pause_btn = QtWidgets.QPushButton("Pause")
        self.pause_btn.setCheckable(True)


        controls = QtWidgets.QHBoxLayout()
        controls.addWidget(QtWidgets.QLabel("Topic:"))
        controls.addWidget(self.topic_edit, 1)
        controls.addWidget(self.compressed_check)
        controls.addWidget(self.subscribe_btn)
        controls.addWidget(self.pause_btn)
        controls.addWidget(self.snapshot_btn)


        # ---- Status bar ----
        self.status = self.statusBar()
        self.fps_label = QtWidgets.QLabel("FPS: 0.0")
        self.res_label = QtWidgets.QLabel("Resolution: -")
        self.status.addPermanentWidget(self.res_label)
        self.status.addPermanentWidget(self.fps_label)


        # ---- Layout ----
        central = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(central)
        layout.addLayout(controls)
        layout.addWidget(self.image_label, 1)
        self.setCentralWidget(central)


        # ---- Signals ----
        self.subscribe_btn.clicked.connect(self.on_subscribe)
        self.snapshot_btn.clicked.connect(self.on_snapshot)


        # ---- Refresh timer ----
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(33)  # ~30 Hz UI refresh


        self._last_time = time.time()
        self._fps = 0.0
        self._last_frame = None


    # -----------------------------------------------------------------------
    def on_subscribe(self):
        topic = self.topic_edit.text().strip()
        if not topic:
            return
        compressed = self.compressed_check.isChecked()
        self.node.subscribe(topic, compressed)
        self.status.showMessage(f"Subscribed to {topic}", 3000)


    def on_snapshot(self):
        if self._last_frame is None:
            self.status.showMessage("No frame available", 2000)
            return
        out_dir = os.path.expanduser("~/ros2_snapshots")
        os.makedirs(out_dir, exist_ok=True)
        fname = os.path.join(out_dir, f"snapshot_{int(time.time())}.png")
        cv2.imwrite(fname, self._last_frame)
        self.status.showMessage(f"Saved {fname}", 3000)


    # -----------------------------------------------------------------------
    def update_frame(self):
        if self.pause_btn.isChecked():
            return


        frame = self.node.get_frame()
        if frame is None:
            return


        self._last_frame = frame


        # FPS calculation (EMA)
        now = time.time()
        dt = now - self._last_time
        self._last_time = now
        if dt > 0:
            inst = 1.0 / dt
            self._fps = 0.9 * self._fps + 0.1 * inst if self._fps > 0 else inst


        h, w = frame.shape[:2]
        self.res_label.setText(f"Resolution: {w}x{h}")
        self.fps_label.setText(f"FPS: {self._fps:0.1f}")


        # Convert BGR -> RGB QImage
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        qimg = QtGui.QImage(rgb.data, w, h, 3 * w, QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap.fromImage(qimg)


        # Scale to label while preserving aspect ratio
        scaled = pix.scaled(
            self.image_label.size(),
            QtCore.Qt.KeepAspectRatio,
            QtCore.Qt.SmoothTransformation,
        )
        self.image_label.setPixmap(scaled)


    # -----------------------------------------------------------------------
    def closeEvent(self, event):
        self.timer.stop()
        event.accept()




# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/camera/image_raw",
                        help="Image topic to subscribe to")
    parser.add_argument("--compressed", action="store_true",
                        help="Subscribe as sensor_msgs/CompressedImage")
    args, ros_args = parser.parse_known_args()


    rclpy.init(args=ros_args)
    node = CameraSubscriber(topic=args.topic, compressed=args.compressed)


    spin_thread = RosSpinThread(node)
    spin_thread.start()


    app = QtWidgets.QApplication(sys.argv)
    gui = CameraGUI(node)
    gui.show()


    try:
        exit_code = app.exec_()
    finally:
        spin_thread.stop()
        spin_thread.join(timeout=1.0)
        node.destroy_node()
        rclpy.shutdown()


    sys.exit(exit_code)




if __name__ == "__main__":
    main()



