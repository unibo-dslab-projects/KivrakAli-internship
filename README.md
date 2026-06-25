# Unitree Go2 — Simulation & ROS 2 Low-Level Control

This repository documents my work on setting up a simulation and control pipeline for the
**Unitree Go2** quadruped, carried out during my Erasmus+ research internship
(University of Bologna, Cesena campus). It is part of the broader project on
distributed coordination of unmanned ground vehicles (UGVs) with an LLM-based
supervisory layer; this phase focused on building hands-on competency with the
robot communication stack (DDS / ROS 2) and a working simulation-control loop.

The end result is a **ROS 2 (rclpy) node that controls the simulated Go2's body
pose in real time** by publishing `unitree_go/msg/LowCmd` to `/lowcmd` and reading
`/lowstate`, using the official `unitree_ros2` message interface.

> **Note for review:** Section [Known limitations & open issues](#5-known-limitations--open-issues)
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

## 4. Setup / reproduction (container)

This assumes the committed image already contains MuJoCo, the SDK, CycloneDDS, and
`unitree_ros2`. The two non-obvious requirements are the **`NET_ADMIN` capability**
and **enabling multicast on the loopback interface** (see
[issue 3](#3-dds-discovery-on-loopback-needs-net_admin--multicast)).

```bash
# Start the container WITH NET_ADMIN and a display + workspace mount
docker run -it --name ros2-go2 \
  --cap-add NET_ADMIN \
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

Run (each terminal is a `docker exec -it ros2-go2 zsh` that first does
`source ~/unitree_ros2/setup_sim.sh`):

```bash
# Terminal A — headless simulator
cd ~/unitree_mujoco/simulate_python && python3 ./sim_headless.py

# Terminal B — ROS 2 (rclpy) teleop
cd ~/ros2_ws && python3 ros_teleop_go2.py

# Terminal C (optional) — verify the bridge
ros2 topic list                 # should list /lowcmd /lowstate /sportmodestate ...
ros2 topic info /lowcmd         # Publisher count: 1, Subscription count: 1 while teleop runs
ros2 topic echo /lowstate --once
```

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

## 5. Known limitations & open issues

### 1. No GUI inside the container
The MuJoCo viewer fails to open under XQuartz from the container
(`libGL: failed to load driver: swrast`, `could not create window`). This is the
known macOS → Docker → XQuartz → OpenGL limitation. The container therefore runs the
sim **headless**; visualization is only available either on the macOS-native sim or by
inspecting `/lowstate` / `/sportmodestate` topics.

### 2. Locomotion (walking) is not implemented
The simulator is **low-level only** — it accepts joint commands and applies a PD law,
with no balance feedback and no built-in gait/sport service. Open-loop position
commands are only stable for quasi-static poses with all feet planted. Walking would
require either a hand-written gait + balance controller or, more realistically, a
**reinforcement-learning locomotion policy** (e.g. trained in MuJoCo Playground / Isaac
and exported for inference). The current teleop is intentionally limited to body-pose
control.

### 3. DDS discovery on loopback needs `NET_ADMIN` + multicast
ROS 2 could not discover the sim's topics over `lo` until multicast was enabled on the
loopback interface (`ip link set lo multicast on`), which requires running the
container with `--cap-add NET_ADMIN`. Without this, `ros2 topic list` only shows
`/parameter_events` and `/rosout`. **This is the most non-standard part of the setup**
and a likely point to review — there may be a cleaner CycloneDDS discovery
configuration for single-host / container use.

### 4. Two CycloneDDS installations coexist
The SDK uses a source-built CycloneDDS (`CYCLONEDDS_HOME=~/cyclonedds/install`), while
ROS 2's `rmw_cyclonedds_cpp` uses the one built inside `unitree_ros2/cyclonedds_ws`.
They coexist and both work, but this is a known source of confusion (and was the cause
of several earlier failures).

### 5. macOS-native sim and the container are network-isolated
The Mac-native sim and the container's ROS 2 cannot communicate (Docker Desktop network
boundary). Everything in the ROS 2 path must run **inside the same container**. The
macOS-native setup is only useful for the visual/SDK demo, not for ROS 2.

---

## 6. File inventory

**`go2_sim/mac/` — macOS-native (Unitree SDK based):**
- `pose_go2.py` — move the robot to a named static pose (interpolated).
- `demo_go2.py` — scripted pose sequence.
- `key_teleop_go2.py` — interactive keyboard teleop (terminal input, SDK pub/sub).
- `config_mac.py` — sim config for macOS (`INTERFACE="lo0"`, `USE_JOYSTICK=0`, `DOMAIN_ID=1`, `SIMULATE_DT=0.002`).

**`go2_sim/container/` — simulation:**
- `sim_headless.py` — runs the MuJoCo physics + DDS bridge with no viewer window.
- `config_container.py` — sim config for the container (`INTERFACE="lo"`, `USE_JOYSTICK=0`, `DOMAIN_ID=1`).

**`go2_sim/container/` — ROS 2:**
- `setup_sim.sh` — sources ROS 2 + `unitree_ros2` + CycloneDDS and sets the
  loopback/simulation DDS configuration.
- `ros_teleop_go2.py` — **the main deliverable**: rclpy node publishing
  `unitree_go/msg/LowCmd` to `/lowcmd`, subscribing `/lowstate`, with SDK-computed CRC.
---

## 7. Key technical notes (lessons learned)

- **Leg tremor** in the sim is an underdamped PD oscillation. It is fixed by reducing
  the simulation timestep (`SIMULATE_DT` 0.005 → 0.002), **not** by raising `kd` —
  raising `kd` at a coarse timestep makes the oscillation worse.
- Final gains used in teleop: `KP = 50`, `KD = 3.5`, with a smooth target-tracking
  factor so commanded positions ease toward targets rather than snapping.
- The Unitree SDK API (`ChannelPublisher`/`ChannelSubscriber`) and ROS 2
  (`rclpy` publisher/subscriber) are two different APIs over the **same** DDS layer;
  the pub/sub concepts map directly between them.
