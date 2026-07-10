#!/usr/bin/env python3
"""Publish the MuJoCo scene obstacle boxes as markers so they appear in the 3D
view alongside the robot. Boxes are read from the Go2 scene (scene.xml). MuJoCo
'size' is a half-extent; RViz marker 'scale' is full size, so we multiply by 2.
Markers are published in the 'odom' frame (same world frame as sportmode_to_odom)."""
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

BOXES = [
    {'pos': [1.2, 0, 0.04], 'size': [0.1, 2, 0.04],  'quat': [1, 0, 0, 0]},
    {'pos': [1.6, 0, 0.04], 'size': [0.1, 2, 0.04],  'quat': [1, 0, 0, 0]},
    {'pos': [2.3, 0, 0.02], 'size': [0.2, 2, 0.15],  'quat': [1, 0, 0, 0]},
    {'pos': [2.6, 0, 0.02], 'size': [0.22, 2, 0.3],  'quat': [1, 0, 0, 0]},
    {'pos': [2.8, 0, 0.02], 'size': [0.23, 2, 0.45], 'quat': [1, 0, 0, 0]},
    {'pos': [3.0, 0, 0.02], 'size': [0.24, 2, 0.6],  'quat': [1, 0, 0, 0]},
    {'pos': [3.2, 0, 0.02], 'size': [0.25, 2, 0.75], 'quat': [1, 0, 0, 0]},
    {'pos': [3.4, 0, 0.02], 'size': [0.26, 2, 0.9],  'quat': [1, 0, 0, 0]},
]
FRAME = "odom"

class SceneMarkers(Node):
    def __init__(self):
        super().__init__('scene_markers')
        self.pub = self.create_publisher(MarkerArray, 'scene_markers', 1)
        self.timer = self.create_timer(1.0, self.publish)
        self.get_logger().info(f"publishing {len(BOXES)} scene boxes as markers in '{FRAME}'")

    def publish(self):
        arr = MarkerArray()
        for i, b in enumerate(BOXES):
            m = Marker()
            m.header.frame_id = FRAME
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "scene_boxes"
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = float(b['pos'][0])
            m.pose.position.y = float(b['pos'][1])
            m.pose.position.z = float(b['pos'][2])
            q = b['quat']
            m.pose.orientation.w = float(q[0])
            m.pose.orientation.x = float(q[1])
            m.pose.orientation.y = float(q[2])
            m.pose.orientation.z = float(q[3])
            m.scale.x = float(b['size'][0]) * 2.0
            m.scale.y = float(b['size'][1]) * 2.0
            m.scale.z = float(b['size'][2]) * 2.0
            m.color.r = 0.6; m.color.g = 0.6; m.color.b = 0.65; m.color.a = 0.8
            arr.markers.append(m)
        self.pub.publish(arr)

def main():
    rclpy.init()
    node = SceneMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
