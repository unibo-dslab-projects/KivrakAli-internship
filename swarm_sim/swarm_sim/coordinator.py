import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

LEADER = 'robot1'

# Spawn poses, must match the launch file (world frame).
SPAWN = {
    'robot1': (0.0,  0.0),
    'robot2': (0.0,  1.5),
    'robot3': (0.0, -1.5),
}

# Waypoints the leader visits in order. The 90-degree turn is the interesting
# part: it forces the formation to rotate rather than just translate.
LEADER_WAYPOINTS = [(4.0, 0.0), (4.0, 4.0), (0.0, 4.0)]

# Follower offsets in the LEADER'S BODY FRAME: +x forward, +y left.
# Negative x puts the follower behind the leader.
FORMATION = {
    'robot2': (-1.2,  1.2),   # behind-left
    'robot3': (-1.2, -1.2),   # behind-right
}

K_LIN, K_ANG = 0.6, 1.5
MAX_LIN, MAX_ANG = 0.8, 1.2
FOLLOWER_MAX_LIN = 1.4        # followers need headroom to catch up on turns
WAYPOINT_TOL = 0.15           # leader switches waypoint inside this radius
HOLD_TOL = 0.12               # followers idle inside this band (kills jitter)

def clamp(v, lo, hi): return max(lo, min(hi, v))

def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class Robot:
    """Odometry subscriber + velocity publisher for a single robot.

    Gazebo's DiffDrive reports odometry relative to each robot's spawn pose,
    so the spawn offset is added back to recover a shared world pose. Valid
    only because every robot spawns with yaw=0.
    """
    def __init__(self, node, ns, spawn):
        self.ns, self.spawn = ns, spawn
        self.pose = None
        self.cmd_pub = node.create_publisher(Twist, f'/{ns}/cmd_vel', 10)
        node.create_subscription(Odometry, f'/{ns}/odom', self._odom_cb, 10)

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x + self.spawn[0], p.y + self.spawn[1],
                     yaw_from_quat(msg.pose.pose.orientation))

    def stop(self):
        self.cmd_pub.publish(Twist())

    def drive_to(self, target, tol, max_lin):
        """Proportional go-to-point. Returns True once within tol."""
        if self.pose is None:
            return False
        x, y, yaw = self.pose
        dx, dy = target[0] - x, target[1] - y
        dist = math.hypot(dx, dy)
        if dist < tol:
            self.stop()
            return True
        err = math.atan2(dy, dx) - yaw
        err = math.atan2(math.sin(err), math.cos(err))  # normalize to [-pi, pi]
        cmd = Twist()
        cmd.angular.z = clamp(K_ANG * err, -MAX_ANG, MAX_ANG)
        # throttle forward speed by heading error: turn first, then drive
        cmd.linear.x = clamp(K_LIN * dist * max(0.0, math.cos(err)), 0.0, max_lin)
        self.cmd_pub.publish(cmd)
        return False


class SwarmCoordinator(Node):
    """Leader-follower coordination. The leader walks a fixed waypoint route;
    followers continuously chase a point rigidly attached to the leader's body
    frame. In M3 the LLM supervisor replaces LEADER_WAYPOINTS and FORMATION
    with values derived from natural-language intent."""

    def __init__(self):
        super().__init__('swarm_coordinator')
        self.robots = {ns: Robot(self, ns, sp) for ns, sp in SPAWN.items()}
        self.leader = self.robots[LEADER]
        self.wp_index = 0
        self.route_done = False
        self.create_timer(0.1, self._tick)  # 10 Hz
        self.get_logger().info(
            f'Leader: {LEADER} | followers: {list(FORMATION)}')

    def _follower_target(self, offset):
        """Rotate a body-frame offset into world coordinates using the leader's
        current heading, so the formation turns with the leader."""
        lx, ly, lyaw = self.leader.pose
        ox, oy = offset
        return (lx + ox * math.cos(lyaw) - oy * math.sin(lyaw),
                ly + ox * math.sin(lyaw) + oy * math.cos(lyaw))

    def _tick(self):
        # followers cannot compute a target until the leader pose is known
        if self.leader.pose is None:
            return

        # --- leader: follow its waypoint route ---
        if self.route_done:
            self.leader.stop()
        else:
            wp = LEADER_WAYPOINTS[self.wp_index]
            if self.leader.drive_to(wp, WAYPOINT_TOL, MAX_LIN):
                self.get_logger().info(
                    f'{LEADER} reached waypoint {self.wp_index + 1}/'
                    f'{len(LEADER_WAYPOINTS)} at ({wp[0]:.1f}, {wp[1]:.1f})')
                self.wp_index += 1
                if self.wp_index >= len(LEADER_WAYPOINTS):
                    self.route_done = True
                    self.get_logger().info('Leader finished its route.')

        # --- followers: chase their slot in the leader's body frame ---
        for ns, offset in FORMATION.items():
            self.robots[ns].drive_to(self._follower_target(offset),
                                     HOLD_TOL, FOLLOWER_MAX_LIN)


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
