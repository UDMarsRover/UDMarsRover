import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
import numpy as np
from scipy.optimize import minimize

class IKController(Node):
    def __init__(self):
        super().__init__('ik_controller')

        self.x = 0.3
        self.y = 0.0
        self.z = 0.2

        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.pub = self.create_publisher(JointState, '/joint_targets', 10)

        self.timer = self.create_timer(0.05, self.loop)

    def joy_callback(self, msg):
        self.x += msg.axes[1] * 0.1
        self.y += msg.axes[0] * 0.1
        self.z += msg.axes[3] * 0.1

    def forward_kinematics(self, q):
        # Simple 3-link planar approximation
        l1 = 0.4572
        l2 = 0.4572

        t1, t2, t3 = q

        x = np.cos(t1) * (l1*np.cos(t2) + l2*np.cos(t2+t3))
        y = np.sin(t1) * (l1*np.cos(t2) + l2*np.cos(t2+t3))
        z = l1*np.sin(t2) + l2*np.sin(t2+t3)

        return np.array([x, y, z])

    def ik(self, target):
        def cost(q):
            pos = self.forward_kinematics(q)
            return np.linalg.norm(pos - target)

        q0 = np.array([0.0, 0.0, 0.0])

        res = minimize(cost, q0, method='BFGS')

        if res.success:
            return res.x
        return None

    def loop(self):
        target = np.array([self.x, self.y, self.z])
        sol = self.ik(target)

        if sol is not None:
            msg = JointState()
            msg.name = ["turret_joint", "shoulder_joint", "elbow_joint"]
            msg.position = sol.tolist()
            self.pub.publish(msg)
        else:
            print("No IK solution")

def main():
    rclpy.init()
    node = IKController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
