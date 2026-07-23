# swarm_sim

Multi-robot simulation package for the UGV swarm coordination work: several
namespaced differential-drive robots in Gazebo, driven by a ROS 2 coordinator
node. Built incrementally, one milestone per coordinator.

## Environment

| Component | Version |
|---|---|
| ROS 2 | Humble |
| Gazebo | Fortress (`ignition-gazebo6`, 6.16.0) |
| Bridge | `ros_gz_sim`, `ros_gz_bridge` |

Two working setups:

- **macOS (M2, arm64)** — Docker container `ros2-go2`, headless only. Gazebo's
  GUI cannot get an OpenGL context through XQuartz, so run with `-s` and verify
  through topics.
- **`disi-pau` (Linux, amd64)** — ROS 2 via RoboStack (conda, no root needed):
  `conda create -n ros_env -c conda-forge -c robostack-humble ros-humble-desktop`,
  then `conda install ros-humble-ros-gz ros-humble-xacro colcon-common-extensions`.
  The GUI works over a TigerVNC session (software rendering).

## Layout

```
swarm_sim/
├── launch/multi_robot.launch.py   # world, spawns, bridges
├── models/diff_robot.urdf.xacro   # diff-drive robot + DiffDrive plugin
└── swarm_sim/
    ├── independent_goals.py       # M1 coordinator
    └── coordinator.py             # M2 coordinator
```

## Architecture

Each robot lives under its own namespace (`robot1`, `robot2`, `robot3`), giving
`/<ns>/cmd_vel` and `/<ns>/odom`. The launch file starts one Gazebo server, one
`robot_state_publisher` and one `parameter_bridge` per robot, and staggers the
spawns ~2 s apart so each robot description has time to latch.

Coordination is centralised: a single coordinator node subscribes to every
robot's odometry and publishes every robot's velocity command. This mirrors the
dual-layer pattern in the reference literature — a slow supervisory layer above
fast per-robot execution — and is the seam where the LLM supervisor will attach.

**Odometry frame.** Gazebo's DiffDrive plugin reports odometry relative to each
robot's *spawn pose*, not the world origin. Both coordinators add the spawn
offset back to recover a shared world frame. This is only valid because every
robot spawns with `yaw=0`; a nonzero spawn yaw would also require rotating the
odom frame into world axes.

## Milestones

### M1 — `independent_goals`

Three robots, three fixed goals, no coupling between them. Each robot runs an
independent proportional go-to-goal controller and stops on arrival. Proves the
plumbing end to end: namespaced spawning, bidirectional bridging, and a control
loop that addresses several robots at once.

Goals are set in the `GOALS` dict.

### M2 — `coordinator`

Leader-follower formation. `robot1` walks a fixed waypoint route; the other two
continuously chase a point rigidly attached to the leader's body.

The offsets in `FORMATION` are expressed in the **leader's body frame**
(+x forward, +y left) and rotated into world coordinates every tick using the
leader's heading. The formation therefore rotates with the leader — a world-frame
offset would leave the followers lined up the same way no matter which direction
the leader turned. The 90-degree turn in the default route exists to exercise
this.

Configuration: `LEADER`, `LEADER_WAYPOINTS`, `FORMATION`.

## Running

```bash
colcon build --packages-select swarm_sim
source install/setup.bash

# terminal A — simulation (headless by default)
ros2 launch swarm_sim multi_robot.launch.py

# with the Gazebo GUI (needs a working display)
ros2 launch swarm_sim multi_robot.launch.py gz_args:='-r -v4 empty.sdf'

# terminal B — pick one coordinator
ros2 run swarm_sim independent_goals   # M1
ros2 run swarm_sim coordinator         # M2
```

Verifying without a GUI:

```bash
ros2 topic list | grep robot
ros2 topic echo /robot1/odom --field pose.pose.position
```

To restart a run, stop both terminals and relaunch. Killing only Gazebo is not
enough: the coordinators latch arrival state, and stale `ign gazebo`,
`parameter_bridge` and `robot_state_publisher` processes often survive `Ctrl-C`.

Adding a robot means adding an entry to `ROBOTS` in the launch file and a
matching entry in the coordinator's spawn/goal tables.

## Known limitations

- **No collision avoidance.** Nothing prevents robots from driving into each
  other; tight formation offsets will cause contact.
- **Followers do not align their heading** with the leader once parked in their
  slot, so they must rotate before moving again.
- **Spawn yaw must be zero** for the odometry offset correction to hold.
- **No reset service** — a full relaunch is the only clean reset.
- **`colcon build --symlink-install` fails** with recent setuptools
  (`option --editable not recognized`), since `setup.py develop` was removed.
  Use a plain `colcon build`, or pin `setuptools<70`.

## Next

M3 — an LLM supervisory layer that derives `LEADER_WAYPOINTS` and `FORMATION`
from natural-language intent instead of hardcoding them, running asynchronously
above the real-time control loop.
