#!/usr/bin/env python3
"""Bridge: /lowstate (unitree_go/msg/LowState) -> /joint_states (sensor_msgs/JointState).
Maps the 12 Unitree motor angles to the Go2 URDF joint names so robot_state_publisher
can produce TF for Foxglove's 3D panel."""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from unitree_go.msg import LowState

# Unitree SDK motor order (0..11) -> URDF joint names
JOINT_NAMES = [
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
]

class Bridge(Node):
    def __init__(self):
        super().__init__("lowstate_to_jointstate")
        self.sub = self.create_subscription(
            LowState, "lowstate", self.cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(JointState, "joint_states", 10)
        self.get_logger().info("lowstate -> joint_states bridge running")

    def cb(self, msg):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = JOINT_NAMES
        js.position = [float(msg.motor_state[i].q) for i in range(12)]
        js.velocity = [float(msg.motor_state[i].dq) for i in range(12)]
        self.pub.publish(js)

def main():
    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
