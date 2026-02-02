#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
import tkinter as tk
from tkinter import ttk
import threading
import sys

class MotorGui(Node):
    def __init__(self, gui_root):
        super().__init__('motor_gui_node')
        self.gui_root = gui_root
        
        # Publisher for wheel velocities
        # Topic: /wheel_velocities_cmd
        self.publisher_ = self.create_publisher(Float32MultiArray, 'wheel_velocities_cmd', 10)

        # Publisher for idle mode
        # Topic: /motor_idle_mode
        self.idle_publisher_ = self.create_publisher(Bool, 'motor_idle_mode', 10)
        
        # Subscriber for wheel status
        # Topic: /wheel_status
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'wheel_status',
            self.listener_callback,
            10)

        # GUI Setup
        self.setup_ui()
        
        # Timer for publishing
        self.create_timer(0.1, self.publish_velocities)

    def setup_ui(self):
        self.gui_root.title("Motor Control GUI")
        
        # Main Layout
        main_frame = tk.Frame(self.gui_root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # --- Left Side: Sliders ---
        sliders_frame = tk.LabelFrame(main_frame, text="Command Velocities", font=("Arial", 10, "bold"))
        sliders_frame.pack(side=tk.LEFT, padx=10, pady=10, fill=tk.Y)
        
        self.sliders = []
        for i in range(6):
            frame = tk.Frame(sliders_frame)
            frame.pack(pady=5, fill=tk.X)
            
            label = tk.Label(frame, text=f"M{i+1}", width=4)
            label.pack(side=tk.LEFT)
            
            # Range -300 to 300
            slider = tk.Scale(frame, from_=-300, to=300, orient=tk.HORIZONTAL, length=150)
            slider.set(0)
            slider.pack(side=tk.RIGHT)
            self.sliders.append(slider)
            
        # Stop Button
        stop_btn = tk.Button(sliders_frame, text="STOP ALL", command=self.stop_all, bg="#ffcccc", fg="red", font=("Arial", 10, "bold"))
        stop_btn.pack(pady=10, fill=tk.X)

        # Idle Mode Control
        mode_frame = tk.LabelFrame(sliders_frame, text="Idle Mode", font=("Arial", 10))
        mode_frame.pack(pady=10, fill=tk.X)
        self.idle_var = tk.StringVar(value="Coast")
        
        tk.Radiobutton(mode_frame, text="Coast", variable=self.idle_var, value="Coast", command=self.update_idle_mode).pack(anchor="w")
        tk.Radiobutton(mode_frame, text="Brake", variable=self.idle_var, value="Brake", command=self.update_idle_mode).pack(anchor="w")

        # --- Right Side: Status Table ---
        status_frame = tk.LabelFrame(main_frame, text="Motor Feedback", font=("Arial", 10, "bold"))
        status_frame.pack(side=tk.RIGHT, padx=10, pady=10, fill=tk.BOTH, expand=True)

        # Define Grid Headers
        headers = ["ID", "Velocity", "Position", "Current (A)", "Voltage (V)", "Temp (C)", "Faults"]
        for col, text in enumerate(headers):
            lbl = tk.Label(status_frame, text=text, font=("Arial", 9, "bold"), borderwidth=1, relief="groove", width=10, padx=5, pady=2)
            lbl.grid(row=0, column=col, sticky="nsew")

        # Create Grid Rows for 6 Motors
        self.stat_vars = [] # List of lists of StringVars [velocity, pos, curr, volt, temp, faults]
        for row in range(6):
            # Motor ID Label
            tk.Label(status_frame, text=f"Motor {row+1}", borderwidth=1, relief="ridge").grid(row=row+1, column=0, sticky="nsew")
            
            row_data = []
            for col in range(6): # 6 data fields
                var = tk.StringVar(value="--")
                lbl = tk.Label(status_frame, textvariable=var, borderwidth=1, relief="sunken", bg="white")
                lbl.grid(row=row+1, column=col+1, sticky="nsew")
                row_data.append(var)
            self.stat_vars.append(row_data)

        # Configure grid weights
        for i in range(7):
            status_frame.columnconfigure(i, weight=1)

    def stop_all(self):
        for slider in self.sliders:
            slider.set(0)
        self.publish_velocities()

    def update_idle_mode(self):
        msg = Bool()
        # True = Brake, False = Coast
        msg.data = (self.idle_var.get() == "Brake")
        self.idle_publisher_.publish(msg)
        self.get_logger().info(f"Published Idle Mode: {self.idle_var.get()}")

    def publish_velocities(self):
        msg = Float32MultiArray()
        # Get values from sliders
        msg.data = [float(s.get()) for s in self.sliders]
        self.publisher_.publish(msg)

    def listener_callback(self, msg):
        try:
            data = list(msg.data)
            self.gui_root.after(0, self.update_stats, data)
        except Exception as e:
            self.get_logger().error(f"Callback error: {e}")

    def update_stats(self, data):
        # Expected format per motor: [Velocity, Position, Current, Voltage, Temp, Faults]
        # Total size should be 6 * 6 = 36
        if len(data) < 36:
            return

        for i in range(6):
            base = i * 6
            # Extract values
            vel   = data[base + 0]
            pos   = data[base + 1]
            curr  = data[base + 2]
            volt  = data[base + 3]
            temp  = data[base + 4]
            fault = data[base + 5]

            # Update StringVars
            self.stat_vars[i][0].set(f"{vel:.2f}")
            self.stat_vars[i][1].set(f"{pos:.2f}")
            self.stat_vars[i][2].set(f"{curr:.2f}")
            self.stat_vars[i][3].set(f"{volt:.2f}")
            self.stat_vars[i][4].set(f"{int(temp)}")
            self.stat_vars[i][5].set(f"0x{int(fault):X}")

def main(args=None):
    rclpy.init(args=args)
    
    root = tk.Tk()
    motor_gui = MotorGui(root)
    
    # Run ROS spin in a separate thread so it doesn't block GUI
    ros_thread = threading.Thread(target=lambda: rclpy.spin(motor_gui), daemon=True)
    ros_thread.start()
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        if rclpy.ok():
            motor_gui.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
