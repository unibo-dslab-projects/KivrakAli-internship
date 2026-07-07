#!/usr/bin/env python3
"""Publish an odom -> base transform from /sportmodestate so the robot's world
position and orientation appear in TF. In Foxglove set the 3D panel 'Display
frame' to 'odom' to watch the robot move across the ground (instead of the
camera being locked to base). This makes the direction of travel visible."""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from unitree_go.msg import SportModeState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class OdomPub(Node):
    def __init__(self):
        super().__init__('sportmode_to_odom')
        self.sub = self.create_subscription(
            SportModeState, 'sportmodestate', self.cb, qos_profile_sensor_data)
        self.br = TransformBroadcaster(self)
        self.get_logger().info("sportmodestate -> odom->base TF running")

    def cb(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base'
        t.transform.translation.x = float(msg.position[0])
        t.transform.translation.y = float(msg.position[1])
        t.transform.translation.z = float(msg.position[2])
        q = msg.imu_state.quaternion
        t.transform.rotation.w = float(q[0])
        t.transform.rotation.x = float(q[1])
        t.transform.rotation.y = float(q[2])
        t.transform.rotation.z = float(q[3])
        self.br.sendTransform(t)

def main():
    rclpy.init()
    node = OdomPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
