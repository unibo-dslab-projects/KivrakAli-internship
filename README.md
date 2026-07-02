# Unitree Go2 — Simulation, ROS 2 Control & Foxglove Visualization

This repository documents my work on setting up a simulation, control, and visualization
pipeline for the **Unitree Go2** quadruped, carried out during my Erasmus+ research
internship (University of Bologna, Cesena campus). It is part of the broader project on
distributed coordination of unmanned ground vehicles (UGVs) with an LLM-based supervisory
layer; this phase focused on building hands-on competency with the robot communication
stack (DDS / ROS 2), a working simulation-control loop, and 3D visualization.

The end result is a **ROS 2 (rclpy) node that controls the simulated Go2's body pose in
real time** (publishing `unitree_go/msg/LowCmd` to `/lowcmd`, reading `/lowstate`, using
the official `unitree_ros2` message interface), together with a **Foxglove Studio setup
that shows the moving robot as a live 3D model**.

> **Note for review:** Section [Known limitations & open issues](#6-known-limitations--open-issues)
> is the most important part if you are looking for where things are fragile or non-standard.

---

## 1. What works today

- **macOS-native MuJoCo simulation** of the Go2 with an interactive keyboard teleop
  (built on the Unitree Python SDK). A graphical viewer window opens and the robot
  responds to keyboard input.
- **A single Docker container** (ROS 2 Humble, Ubuntu 22.04, arm64) that holds the
  simulator, the Unitree SDK, **and** the `unitree_ros2` packages together, so a ROS 2
  node and the sim share the same DDS domain.
- **A ROS 2 (rclpy) teleop node** that drives the simulated robot through `/lowcmd`.
  Verified: `ros2 topic info /lowcmd` shows `Publisher count: 1, Subscription count: 1`
  while the node runs, and `/lowstate` reflects the commanded changes.
- **Foxglove Studio visualization**: the headless sim is shown on the host as live plots
  (joint angles, body height) and as a **3D robot model with meshes** that moves as the
  robot is driven. See [section 4](#4-foxglove-visualization-3d-robot-view).

What the robot can do in both setups: **body-pose control** — raise/lower the body,
roll, pitch, and a few preset stances (stand / sit / bow). All four feet stay planted,
which keeps the open-loop position controller stable.

**The robot does not walk.** See [issue 2](#2-locomotion-walking-is-not-implemented).

---

## 2. Environments

### 2.1 macOS-native simulation (Apple Silicon, M2)

Runs the MuJoCo sim natively on macOS. Because macOS differs from the Linux target the
SDK assumes, a few patches were required:

- `unitree_sdk2py/utils/timerfd.py` — guarded the `timerfd_create/settime/gettime`
  calls behind a `sys.platform.startswith("linux")` check (macOS has no `timerfd_create`).
- `unitree_sdk2py/utils/thread.py` — added a `time.monotonic()` / `time.sleep` fallback
  loop in `RecurrentThread` for when `timerfd` is unavailable.
- CycloneDDS built from source (`releases/0.10.x`), `CYCLONEDDS_HOME` exported.
- `simulate_python/config.py`: `INTERFACE = "lo0"` (macOS loopback), `USE_JOYSTICK = 0`
  (pygame's joystick init crashes on a background thread on macOS), `DOMAIN_ID = 1`,
  `SIMULATE_DT = 0.002`.
- The sim must be launched with `mjpython` (not `python3`) on macOS.

Run (two terminals):

```bash
# Terminal 1 — simulator (opens a viewer window)
cd ~/unitree_mujoco/simulate_python && mjpython ./unitree_mujoco.py

# Terminal 2 — keyboard teleop (SDK-based)
cd ~/unitree_mujoco/example/python && python3 key_teleop_go2.py
```

### 2.2 Docker ROS 2 container (primary environment)

Base image: `althack/ros2:humble-full` (Ubuntu 22.04, ROS 2 Humble, arm64).
Everything below was installed *inside* the container and then committed to an image.

Installed in the container:

- `mujoco` (3.9.0) via pip.
- `unitree_sdk2_python` via `pip install -e .`. On Linux **no** `timerfd` patch is needed.
- CycloneDDS for the SDK, built from source to `~/cyclonedds/install`
  (`CYCLONEDDS_HOME`).
- `unitree_ros2` (ROS 2 message packages `unitree_go`, `unitree_api`, `unitree_hg`,
  plus examples), built against its own CycloneDDS in `cyclonedds_ws/`.
- `ros-humble-foxglove-bridge` and `ros-humble-robot-state-publisher` (for the Foxglove
  3D visualization — see section 4).

Because the sim's MuJoCo viewer cannot open inside the container (see
[issue 1](#1-no-gui-inside-the-container)), the sim runs **headless** via a small
launcher (`sim_headless.py`) that steps the physics and runs the DDS bridge without
a window.

---

## 3. How the ROS 2 control loop works

The key fact (from the Unitree docs): Unitree's robots and the SDK communicate over
CycloneDDS, and ROS 2 also uses DDS. The DDS-level topic name for a ROS 2 topic
`lowcmd` is `rt/lowcmd` — which is exactly what the SDK/sim uses. So a ROS 2 node using
`rmw_cyclonedds_cpp`, the `unitree_go` message types, and the matching DDS domain can
talk to the simulator directly, without wrapping the SDK.

```
rclpy node  --(unitree_go/msg/LowCmd on /lowcmd)-->  CycloneDDS (rt/lowcmd)  -->  sim bridge  -->  PD control on 12 joints
sim bridge  --(unitree_go/msg/LowState on /lowstate)-->  CycloneDDS  -->  rclpy node (reads joint angles, IMU)
```

Control model: the sim only accepts **low-level** commands (`LowCmd` = per-joint
position/velocity/torque targets with `kp`/`kd` gains). The bridge applies
`tau = kp*(q_des - q) + kd*(dq_des - dq)`. There is **no** high-level "walk" service.

**CRC detail (important):** the sim silently ignores a `LowCmd` whose `crc` field is
wrong. The ROS 2 `unitree_go/msg/LowCmd` has the same field layout as the SDK's
`LowCmd_`, so the teleop node fills the ROS message, copies the fields into an SDK
`LowCmd_`, computes the CRC with the SDK's `CRC().Crc()`, and writes it back into the
ROS message before publishing.

Body-pose kinematics: keyboard input adjusts three scalars — height `H`, pitch `P`,
roll `R` — which are mapped to per-leg "crouch" amounts and then to thigh/calf joint
angles, keeping all feet on the ground. The mapping is clamped so that every reachable
combination stays inside the Go2 joint limits.

---

## 4. Foxglove visualization (3D robot view)

Because the MuJoCo viewer cannot open inside the container (see
[issue 1](#1-no-gui-inside-the-container)), the simulated robot is visualized in
**Foxglove Studio** running on the host (macOS), connected to the container's ROS 2
stack over a WebSocket bridge. This shows the robot both as live plots (joint angles,
body height) and as a **3D model that moves with the simulation**.

Pipeline for the 3D model:

```
/lowstate (unitree_go/msg/LowState)
   -> [lowstate_to_jointstate.py]          # 12 motor angles -> URDF joint names
/joint_states (sensor_msgs/JointState)
   -> [robot_state_publisher + Go2 URDF]
/tf  ->  Foxglove 3D panel (loads the URDF, poses each link from /tf)
```

Components:

- `foxglove_bridge` (installed in the container) exposes the ROS 2 topics over
  `ws://localhost:8765`. The container is run with `-p 8765:8765` so the host can reach
  it.
- `lowstate_to_jointstate.py` subscribes to `/lowstate` with **best-effort QoS**
  (`qos_profile_sensor_data`) and republishes the 12 joint angles as
  `sensor_msgs/JointState`, mapping the Unitree motor order (FR, FL, RR, RL) to the Go2
  URDF joint names (`FR_hip_joint`, `FR_thigh_joint`, ...). The best-effort QoS matters:
  Unitree publishes `/lowstate` as best-effort, so a default (reliable) subscription
  receives no messages — this was also why a naive teleop/CLI subscriber would hang on
  "waiting for /lowstate" while `foxglove_bridge` (which adapts QoS) worked fine.
- `robot_state_publisher` loads the Go2 URDF (from the `unitree_ros` description package,
  cloned separately) and publishes `/tf` + `/robot_description`.

Running it (in addition to the sim), each in its own sourced terminal:

```bash
# Foxglove bridge
ros2 launch foxglove_bridge foxglove_bridge_launch.xml

# LowState -> JointState bridge
cd ~/ros2_ws && python3 lowstate_to_jointstate.py

# robot_state_publisher with the Go2 URDF
ros2 run robot_state_publisher robot_state_publisher \
  ~/unitree_ros/robots/go2_description/urdf/go2_description.urdf
```

Then in Foxglove: **Open connection → Foxglove WebSocket → `ws://localhost:8765`**, add
a **3D panel**, and set the **display frame** to `base`. Add **Plot** panels for
`/lowstate.motor_state[i].q` (joint angles) and `/sportmodestate.position[2]` (body
height) to see the values change as the robot is driven.

**Mesh loading caveat:** the Go2 URDF references meshes as `package://go2_description/...`.
The Foxglove **web** app cannot resolve `package://` (or `file://`) paths, so it shows
only the TF axes, not the meshes. The **desktop** app is required, and even there
`ROS_PACKAGE_PATH` resolution was unreliable. The meshes were finally loaded by copying
the description to the host and rewriting the mesh paths to absolute `file://` paths,
then adding that URDF as a **"File path" custom layer** in the 3D panel:

```bash
# on the host: copy the description out of the container
docker cp ros2-go2:/root/unitree_ros/robots/go2_description ~/go2_foxglove/

# rewrite package:// -> absolute file:// paths
sed 's|package://go2_description/|file:///Users/aliyagiz/go2_foxglove/go2_description/|g' \
  ~/go2_foxglove/go2_description/urdf/go2_description.urdf \
  > ~/go2_foxglove/go2_fixed.urdf
# then in Foxglove: 3D panel -> Custom layers -> + -> URDF -> File path -> go2_fixed.urdf
```

Result: the Go2 appears in the 3D panel with meshes, and driving it via the ROS 2
teleop moves the legs live.

---

## 5. Setup / reproduction (container)

This assumes the committed image already contains MuJoCo, the SDK, CycloneDDS,
`unitree_ros2`, `foxglove_bridge`, and `robot_state_publisher`. The two non-obvious
requirements are the **`NET_ADMIN` capability** and **enabling multicast on the loopback
interface** (see [issue 3](#3-dds-discovery-on-loopback-needs-net_admin--multicast)).

```bash
# Start the container WITH NET_ADMIN, the Foxglove port, a display + workspace mount
docker run -it --name ros2-go2 \
  --cap-add NET_ADMIN \
  -p 8765:8765 \
  -e DISPLAY=host.docker.internal:0 \
  -v /Users/aliyagiz/ros2_tutorials:/root/ros2_ws \
  ros2-go2-img zsh

# Inside the container: enable loopback multicast (needed for DDS discovery)
ip link set lo multicast on
```

`setup_sim.sh` configures the ROS 2 + Unitree + CycloneDDS environment for **loopback
/ simulation** use (real-robot use would point the interface at the Ethernet adapter
instead of `lo`):

```bash
source /opt/ros/humble/setup.zsh
source $HOME/unitree_ros2/cyclonedds_ws/install/setup.zsh
source $HOME/unitree_ros2/install/setup.zsh
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=1
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" priority="default" multicast="true"/></Interfaces></General></Domain></CycloneDDS>'
```

Run — each terminal is a `docker exec -it ros2-go2 zsh` that first does
`source ~/unitree_ros2/setup_sim.sh`. The full 3D-visualization setup uses five
terminals:

```bash
# 1 — headless simulator
cd ~/unitree_mujoco/simulate_python && python3 ./sim_headless.py

# 2 — Foxglove bridge
ros2 launch foxglove_bridge foxglove_bridge_launch.xml

# 3 — LowState -> JointState bridge
cd ~/ros2_ws && python3 lowstate_to_jointstate.py

# 4 — robot_state_publisher (Go2 URDF -> /tf, /robot_description)
ros2 run robot_state_publisher robot_state_publisher \
  ~/unitree_ros/robots/go2_description/urdf/go2_description.urdf

# 5 — ROS 2 (rclpy) teleop
cd ~/ros2_ws && python3 ros_teleop_go2.py
```

Quick sanity check (any sourced terminal): `ros2 topic list` should show
`/lowcmd /lowstate /sportmodestate ...`, and `ros2 topic info /lowcmd` should show
`Publisher count: 1` while the teleop runs.

**Every consumer (sim, bridge, teleop, robot_state_publisher, foxglove_bridge) must
source the same `setup_sim.sh` (multicast = true), and multicast must be enabled on
`lo`.** A mismatch here is the single most common cause of "topics visible but no data".

### `unitree_ros2` build notes (gotchas hit during setup)

- CycloneDDS must be built in a shell where the **ROS 2 environment is NOT sourced**,
  otherwise the build picks up the wrong dependencies. A bare `zsh -f` plus
  `unset CYCLONEDDS_HOME AMENT_PREFIX_PATH CMAKE_PREFIX_PATH ...` was used.
- The CycloneDDS build failed on the `ddsperf` tool (an iceoryx/shared-memory
  dependency). Building with `--cmake-args -DBUILD_DDSPERF=OFF -DENABLE_SHM=OFF`
  avoids it; the DDS library itself is all that's needed.
- A bare `colcon build` (no args) re-builds CycloneDDS and re-triggers the `ddsperf`
  failure. Use `--packages-skip cyclonedds` once CycloneDDS is built, so only the
  message packages are compiled.
- Use the **`humble`** branch of `rmw_cyclonedds` (the upstream Unitree README shows
  `foxy`).

---

## 6. Known limitations & open issues

### 1. No GUI inside the container
The MuJoCo viewer fails to open under XQuartz from the container
(`libGL: failed to load driver: swrast`, `could not create window`). This is the
known macOS → Docker → XQuartz → OpenGL limitation. The container therefore runs the
sim **headless**; visualization is done through Foxglove (section 4) or by inspecting
`/lowstate` / `/sportmodestate` topics.

### 2. Locomotion (walking) is not implemented
The simulator is **low-level only** — it accepts joint commands and applies a PD law,
with no balance feedback and no built-in gait/sport service. Open-loop position
commands are only stable for quasi-static poses with all feet planted. Walking would
require either a hand-written gait + balance controller or, more realistically, a
**reinforcement-learning locomotion policy** (e.g. trained in MuJoCo Playground / Isaac
and exported for inference). The current teleop is intentionally limited to body-pose
control. (This is also why the IMU roll/pitch barely changes when commanded — the robot
keeps all feet planted and stays balanced rather than tilting its body.)

### 3. DDS discovery on loopback needs `NET_ADMIN` + multicast
ROS 2 could not discover the sim's topics over `lo` until multicast was enabled on the
loopback interface (`ip link set lo multicast on`), which requires running the
container with `--cap-add NET_ADMIN`. Without this, `ros2 topic list` only shows
`/parameter_events` and `/rosout`. **This is the most non-standard part of the setup**
and a likely point to review — there may be a cleaner CycloneDDS discovery
configuration for single-host / container use. Note also that `ip link set lo multicast
on` is a runtime setting and must be re-applied each time the container is (re)started.

### 4. Two CycloneDDS installations coexist
The SDK uses a source-built CycloneDDS (`CYCLONEDDS_HOME=~/cyclonedds/install`), while
ROS 2's `rmw_cyclonedds_cpp` uses the one built inside `unitree_ros2/cyclonedds_ws`.
They coexist and both work, but this is a known source of confusion (and was the cause
of several earlier failures).

### 5. macOS-native sim and the container are network-isolated
The Mac-native sim and the container's ROS 2 cannot communicate (Docker Desktop network
boundary). Everything in the ROS 2 path must run **inside the same container**. The
macOS-native setup is only useful for the visual/SDK demo, not for ROS 2.

### 6. Foxglove mesh rendering needs the desktop app + `file://` paths
The Go2 URDF uses `package://` mesh paths, which the Foxglove **web** app cannot
resolve — it then shows only TF axes. Meshes render only in the **desktop** app, and
only after rewriting the paths to absolute `file://` paths and loading the URDF as a
"File path" custom layer (see section 4).

---

## 7. File inventory

The repository layout:

```
go2_sim/
  mac/                      # macOS-native (Unitree SDK based)
    pose_go2.py
    demo_go2.py
    key_teleop_go2.py
    config_mac.py
  container/                # Docker ROS 2 environment
    sim_headless.py
    ros_teleop_go2.py
    lowstate_to_jointstate.py
    setup_sim.sh
    config_container.py
README.md
```

**`go2_sim/mac/` — macOS-native (Unitree SDK based):**
- `pose_go2.py` — move the robot to a named static pose (interpolated).
- `demo_go2.py` — scripted pose sequence.
- `key_teleop_go2.py` — interactive keyboard teleop (terminal input, SDK pub/sub).
- `config_mac.py` — sim config for macOS (`INTERFACE="lo0"`, `USE_JOYSTICK=0`,
  `DOMAIN_ID=1`, `SIMULATE_DT=0.002`).

**`go2_sim/container/` — simulation & ROS 2:**
- `sim_headless.py` — runs the MuJoCo physics + DDS bridge with no viewer window.
- `config_container.py` — sim config for the container (`INTERFACE="lo"`,
  `USE_JOYSTICK=0`, `DOMAIN_ID=1`).
- `ros_teleop_go2.py` — **the main control deliverable**: rclpy node publishing
  `unitree_go/msg/LowCmd` to `/lowcmd`, subscribing `/lowstate`, with SDK-computed CRC.
- `lowstate_to_jointstate.py` — bridges `/lowstate` to `/joint_states` (best-effort QoS,
  motor-order → URDF joint-name mapping) so `robot_state_publisher` can drive the
  Foxglove 3D model.
- `setup_sim.sh` — sources ROS 2 + `unitree_ros2` + CycloneDDS and sets the
  loopback/simulation DDS configuration.

Not committed (cloned / rebuilt via the instructions above): `unitree_ros2`,
`unitree_mujoco`, `unitree_sdk2_python`, `cyclonedds`, and `unitree_ros` (the Go2 URDF /
mesh description used for visualization).

---

## 8. Key technical notes (lessons learned)

- **Leg tremor** in the sim is an underdamped PD oscillation. It is fixed by reducing
  the simulation timestep (`SIMULATE_DT` 0.005 → 0.002), **not** by raising `kd` —
  raising `kd` at a coarse timestep makes the oscillation worse.
- Final gains used in teleop: `KP = 50`, `KD = 3.5`, with a smooth target-tracking
  factor so commanded positions ease toward targets rather than snapping.
- The Unitree SDK API (`ChannelPublisher`/`ChannelSubscriber`) and ROS 2
  (`rclpy` publisher/subscriber) are two different APIs over the **same** DDS layer;
  the pub/sub concepts map directly between them.
- **QoS matters for Unitree topics:** `/lowstate` (and the other state topics) are
  published best-effort. A default reliable subscription silently receives nothing;
  use `qos_profile_sensor_data`. `foxglove_bridge` adapts automatically, which is why it
  could read `/lowstate` when a naive rclpy subscriber could not.
