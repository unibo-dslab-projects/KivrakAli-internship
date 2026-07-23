import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Robot namespace -> spawn pose (must match the launch file) and goal, both in
# WORLD coordinates. Gazebo's DiffDrive publishes odometry relative to each
# robot's spawn pose, so the offset is added back to recover the world pose.
# NOTE: valid only because every robot spawns with yaw=0; a nonzero spawn yaw
# would also require rotating the odom frame into world axes.
ROBOTS = {
    'robot1': {'spawn': (0.0,  0.0), 'goal': (4.0,  0.0)},
    'robot2': {'spawn': (0.0,  1.5), 'goal': (4.0,  1.5)},
    'robot3': {'spawn': (0.0, -1.5), 'goal': (4.0, -1.5)},
}
K_LIN, K_ANG = 0.6, 1.5
MAX_LIN, MAX_ANG = 0.8, 1.2
GOAL_TOL = 0.15

def clamp(v, lo, hi): return max(lo, min(hi, v))

def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)

class RobotController:
    """One go-to-goal controller per robot, operating in world coordinates.
    M2 (formation / leader-follower) and M3 (LLM supervisor) will drive this
    unit instead of the hardcoded goals above -- this is the extension point."""
    def __init__(self, node, ns, spawn, goal):
        self.ns, self.spawn, self.goal = ns, spawn, goal
        self.pose = None
        self.reached = self._logged = False
        self.cmd_pub = node.create_publisher(Twist, f'/{ns}/cmd_vel', 10)
        node.create_subscription(Odometry, f'/{ns}/odom', self._odom_cb, 10)

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        # shift odom (spawn-relative) into the shared world frame
        self.pose = (p.x + self.spawn[0], p.y + self.spawn[1],
                     yaw_from_quat(msg.pose.pose.orientation))

    def step(self):
        cmd = Twist()
        if self.pose is None or self.reached:
            self.cmd_pub.publish(cmd)  # zero twist -> hold still
            return
        x, y, yaw = self.pose
        dx, dy = self.goal[0] - x, self.goal[1] - y
        dist = math.hypot(dx, dy)
        if dist < GOAL_TOL:
            self.reached = True
            self.cmd_pub.publish(cmd)
            return
        err = math.atan2(dy, dx) - yaw
        err = math.atan2(math.sin(err), math.cos(err))  # normalize to [-pi, pi]
        cmd.angular.z = clamp(K_ANG * err, -MAX_ANG, MAX_ANG)
        # only drive forward once roughly aligned (avoids arcing off course)
        cmd.linear.x = clamp(K_LIN * dist * max(0.0, math.cos(err)), 0.0, MAX_LIN)
        self.cmd_pub.publish(cmd)

class SwarmCoordinator(Node):
    def __init__(self):
        super().__init__('swarm_coordinator')
        self.ctl = {ns: RobotController(self, ns, c['spawn'], c['goal'])
                    for ns, c in ROBOTS.items()}
        self._all_logged = False
        self.create_timer(0.1, self._tick)  # 10 Hz
        self.get_logger().info(f'Coordinating: {list(self.ctl)}')

    def _tick(self):
        for ns, c in self.ctl.items():
            c.step()
            if c.reached and not c._logged:
                c._logged = True
                x, y, _ = c.pose
                self.get_logger().info(f'{ns} reached goal at ({x:.2f}, {y:.2f}).')
        if not self._all_logged and all(c.reached for c in self.ctl.values()):
            self._all_logged = True
            self.get_logger().info('All robots reached their goals.')

def main():
    rclpy.init()
    node = SwarmCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
