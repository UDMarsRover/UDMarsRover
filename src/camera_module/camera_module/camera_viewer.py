#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_srvs.srv import SetBool
import tkinter as tk
from tkinter import ttk
from PIL import Image as PILImage, ImageTk
import cv2
from cv_bridge import CvBridge
import numpy as np
import threading

class CameraViewer(Node):
    def __init__(self, gui_root):
        super().__init__('camera_viewer')
        self.gui_root = gui_root
        
        # Declare parameters
        self.declare_parameter('camera_names', ['camera_0'])
        self.camera_names = self.get_parameter('camera_names').value
        
        self.bridge = CvBridge()
        self.current_camera = None
        self.current_mode = 'compressed'  # 'raw' or 'compressed'
        self.current_subscription = None
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        
        # Setup GUI
        self.setup_ui()
        
        # Start update timer
        self.gui_root.after(33, self.update_display)  # ~30 FPS
        
        self.get_logger().info('Camera viewer initialized')
    
    def setup_ui(self):
        self.gui_root.title("Camera Viewer")
        
        # Main layout
        main_frame = tk.Frame(self.gui_root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Control panel
        control_frame = tk.LabelFrame(main_frame, text="Camera Control", font=("Arial", 10, "bold"))
        control_frame.pack(side=tk.TOP, fill=tk.X, pady=(0, 10))
        
        # Camera selection
        cam_frame = tk.Frame(control_frame)
        cam_frame.pack(side=tk.LEFT, padx=10, pady=5)
        
        tk.Label(cam_frame, text="Camera:").pack(side=tk.LEFT, padx=5)
        self.camera_var = tk.StringVar()
        if self.camera_names and len(self.camera_names) > 0:
            self.camera_var.set(self.camera_names[0])
        else:
            # Default camera name if none provided
            self.camera_names = ['camera_0']
            self.camera_var.set('camera_0')
        
        camera_menu = ttk.Combobox(cam_frame, textvariable=self.camera_var, 
                                    values=self.camera_names, state='readonly', width=15)
        camera_menu.pack(side=tk.LEFT, padx=5)
        
        # Mode selection
        mode_frame = tk.Frame(control_frame)
        mode_frame.pack(side=tk.LEFT, padx=10, pady=5)
        
        tk.Label(mode_frame, text="Mode:").pack(side=tk.LEFT, padx=5)
        self.mode_var = tk.StringVar(value='compressed')
        
        tk.Radiobutton(mode_frame, text="Raw", variable=self.mode_var, 
                      value='raw').pack(side=tk.LEFT, padx=5)
        tk.Radiobutton(mode_frame, text="Compressed", variable=self.mode_var, 
                      value='compressed').pack(side=tk.LEFT, padx=5)
        
        # Buttons
        btn_frame = tk.Frame(control_frame)
        btn_frame.pack(side=tk.LEFT, padx=10, pady=5)
        
        tk.Button(btn_frame, text="Enable & View", command=self.enable_camera,
                 bg="#ccffcc", font=("Arial", 9, "bold")).pack(side=tk.LEFT, padx=5)
        tk.Button(btn_frame, text="Disable", command=self.disable_camera,
                 bg="#ffcccc", font=("Arial", 9, "bold")).pack(side=tk.LEFT, padx=5)
        
        # Status label
        status_frame = tk.Frame(control_frame)
        status_frame.pack(side=tk.LEFT, padx=10, pady=5, fill=tk.X, expand=True)
        
        self.status_label = tk.Label(status_frame, text="Status: No camera selected", 
                                     font=("Arial", 9), anchor='w')
        self.status_label.pack(fill=tk.X)
        
        # Video display
        video_frame = tk.LabelFrame(main_frame, text="Camera Feed", font=("Arial", 10, "bold"))
        video_frame.pack(fill=tk.BOTH, expand=True)
        
        self.video_label = tk.Label(video_frame, bg='black')
        self.video_label.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Info panel
        info_frame = tk.Frame(main_frame)
        info_frame.pack(fill=tk.X, pady=(5, 0))
        
        self.info_label = tk.Label(info_frame, text="", font=("Courier", 8), anchor='w')
        self.info_label.pack(fill=tk.X)
    
    def enable_camera(self):
        camera_name = self.camera_var.get()
        if not camera_name:
            self.update_status("No camera selected", "red")
            return
        
        # Call service to enable camera
        service_name = f'/camera/{camera_name}/enable'
        client = self.create_client(SetBool, service_name)
        
        if not client.wait_for_service(timeout_sec=2.0):
            self.update_status(f"Service {service_name} not available", "red")
            self.get_logger().error(f"Service {service_name} not available")
            return
        
        request = SetBool.Request()
        request.data = True
        
        future = client.call_async(request)
        future.add_done_callback(lambda f: self.handle_enable_response(f, camera_name))
    
    def handle_enable_response(self, future, camera_name):
        try:
            response = future.result()
            if response.success:
                self.gui_root.after(0, self.start_viewing, camera_name)
                self.update_status(f"Enabled {camera_name}", "green")
            else:
                self.update_status(f"Failed to enable: {response.message}", "red")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            self.update_status(f"Error: {str(e)}", "red")
    
    def start_viewing(self, camera_name):
        # Unsubscribe from previous camera
        if self.current_subscription is not None:
            self.destroy_subscription(self.current_subscription)
            self.current_subscription = None
        
        self.current_camera = camera_name
        self.current_mode = self.mode_var.get()
        
        # Subscribe to appropriate topic
        if self.current_mode == 'raw':
            topic = f'/camera/{camera_name}/raw'
            self.current_subscription = self.create_subscription(
                Image, topic, self.raw_callback, 10)
            self.get_logger().info(f"Subscribed to {topic}")
        else:
            topic = f'/camera/{camera_name}/compressed'
            self.current_subscription = self.create_subscription(
                CompressedImage, topic, self.compressed_callback, 10)
            self.get_logger().info(f"Subscribed to {topic}")
        
        self.update_status(f"Viewing {camera_name} ({self.current_mode})", "green")
    
    def disable_camera(self):
        camera_name = self.camera_var.get()
        if not camera_name:
            return
        
        # Unsubscribe first
        if self.current_subscription is not None:
            self.destroy_subscription(self.current_subscription)
            self.current_subscription = None
        
        self.current_camera = None
        with self.frame_lock:
            self.latest_frame = None
        
        # Call service to disable camera
        service_name = f'/camera/{camera_name}/enable'
        client = self.create_client(SetBool, service_name)
        
        if not client.wait_for_service(timeout_sec=2.0):
            self.update_status("Service not available", "orange")
            return
        
        request = SetBool.Request()
        request.data = False
        
        future = client.call_async(request)
        future.add_done_callback(lambda f: self.handle_disable_response(f, camera_name))
    
    def handle_disable_response(self, future, camera_name):
        try:
            response = future.result()
            self.update_status(f"Disabled {camera_name}", "blue")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
    
    def raw_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.frame_lock:
                self.latest_frame = cv_image
        except Exception as e:
            self.get_logger().error(f"Failed to convert raw image: {e}")
    
    def compressed_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            with self.frame_lock:
                self.latest_frame = cv_image
        except Exception as e:
            self.get_logger().error(f"Failed to decode compressed image: {e}")
    
    def update_display(self):
        with self.frame_lock:
            if self.latest_frame is not None:
                frame = self.latest_frame.copy()
            else:
                frame = None
        
        if frame is not None:
            # Frame is already in BGR from OpenCV, convert to RGB for display
            # frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame_rgb = frame
            
            # Resize to fit display
            label_width = self.video_label.winfo_width()
            label_height = self.video_label.winfo_height()
            
            if label_width > 1 and label_height > 1:
                h, w = frame_rgb.shape[:2]
                aspect = w / h
                
                if label_width / label_height > aspect:
                    new_height = label_height
                    new_width = int(new_height * aspect)
                else:
                    new_width = label_width
                    new_height = int(new_width / aspect)
                
                frame_resized = cv2.resize(frame_rgb, (new_width, new_height))
            else:
                frame_resized = frame_rgb
            
            # Convert to PIL Image and display
            pil_image = PILImage.fromarray(frame_resized)
            photo = ImageTk.PhotoImage(image=pil_image)
            self.video_label.config(image=photo)
            self.video_label.image = photo  # Keep reference
            
            # Update info
            h, w = frame.shape[:2]
            self.info_label.config(text=f"Resolution: {w}x{h} | Mode: {self.current_mode}")
        
        # Schedule next update
        self.gui_root.after(33, self.update_display)
    
    def update_status(self, message, color="black"):
        self.gui_root.after(0, lambda: self.status_label.config(text=f"Status: {message}", fg=color))

def main(args=None):
    rclpy.init(args=args)
    
    root = tk.Tk()
    root.geometry("800x600")
    
    viewer = CameraViewer(root)
    
    # Run ROS spin in separate thread
    ros_thread = threading.Thread(target=lambda: rclpy.spin(viewer), daemon=True)
    ros_thread.start()
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            viewer.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
