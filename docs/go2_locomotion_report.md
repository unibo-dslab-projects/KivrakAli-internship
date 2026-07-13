# Go2 Locomotion — Progress Report

**Author:** Ali Yağız Kıvrak
**Context:** Erasmus+ research internship, University of Bologna (Cesena campus)
**Topic:** Forward/backward locomotion of the Unitree Go2 in simulation

---

## 1. Objective

The goal for this step was to make the simulated Go2 walk forward and backward on flat
ground, building on the earlier phase (Go2 MuJoCo simulation + ROS 2 low-level control +
Foxglove 3D visualization), which is already working and committed to the repository.

---

## 2. What I did

Everything below runs on top of the existing stack (headless MuJoCo sim, `unitree_ros2`
messages over CycloneDDS, `foxglove_bridge`, `robot_state_publisher`, Foxglove Studio),
and drives the robot through the same low-level `/lowcmd` interface used by the pose
teleop.

- **Closed-form 3D leg inverse kinematics.** A per-leg IK that places the foot at a
  target `(x_forward, y_lateral, z_up)` relative to the hip, using hip abduction plus a
  2-link (thigh/calf) planar solve. Derived from the URDF link geometry
  (`hip_offset = 0.0955`, `thigh = calf = 0.213`) and verified numerically against
  forward kinematics (round-trip error < 1e-4) and live in the sim.

- **Static crawl gait** (`gait_go2.py`). A statically-stable gait where one leg swings at
  a time (duty factor 0.75, so three feet are always in contact), including:
  - a leg swing sequence (FR → RL → FL → RR),
  - body weight-shift toward the support triangle synchronized with each swing,
  - propulsion by moving the planted stance feet backward in the body frame,
  - a smooth "settle-then-ramp" start so the robot eases into the stand before walking,
  - a cosine-eased swing trajectory, so the swing foot has near-zero horizontal velocity
    at touchdown (added to reduce foot sliding on landing).
  All commanded joint angles stay within the Go2 joint limits across the full cycle.

- **Keyboard-teleop gait** (`gait_teleop.py`). The same gait under live keyboard control:
  direction (forward / backward / stop) plus live adjustment of stride length, swing
  height, weight-shift, and cycle time. This was the decisive tool — it replaced the
  edit-code → restart-sim → observe loop with real-time tuning, and is how a working
  parameter set was actually found.

- **Contact/parameter tuning in the MuJoCo model.** The default foot friction in the Go2
  model is low (`0.4`); I raised it (up to `2.0`) and added floor friction. The
  simulation timestep was also tuned (smaller steps remove a PD tremor but slow the sim
  on this machine, so there is a trade-off).

- **World-frame visualization.** An `odom → base` TF publisher (`sportmode_to_odom.py`)
  that republishes the robot's world position and IMU orientation from
  `/sportmodestate`, so the 3D view can use a fixed world frame and the robot is seen
  travelling across the ground (rather than the camera being locked to the robot) — this
  is what made the direction of travel observable at all. Plus a marker publisher
  (`scene_markers.py`) that draws the MuJoCo scene's obstacle boxes in Foxglove, so the
  robot is shown walking in the same scene the physics uses.

---

## 3. Results

**The robot walks.** With the teleop gait and a calibrated parameter set
(stride ≈ 0.08 m, swing height ≈ 0.08 m, weight-shift ≈ 0.04 m, cycle ≈ 5 s), the Go2
produces a **clean, sustained, statically-stable walk with a steady net displacement**:
`/sportmodestate.position` shows a continuous trend, body height holds around 0.31–0.35 m
(no collapse), and the robot is visibly seen crossing the ground plane in the 3D view —
far enough, in one run, to reach the obstacle boxes in the scene.

Also verified along the way:
- the legs lift and step in the correct sequence, and the weight-shift moves the body
  toward the correct support triangle for each swing (checked in the joint/position plots
  and in the 3D view);
- joint commands stay within limits; no invalid configurations are commanded;
- the robot stands stably via IK before and after walking.

**The key finding for the gait itself:** the failure mode was not friction, and not the
weight-shift — it was **swing height vs. stride length**. With a low swing (~0.02 m) and a
long stride (~0.15 m) the foot drags along the ground and the robot shuffles in place;
raising the swing to ~0.08 m and reducing the stride to ~0.08 m gives clean foot clearance
and real propulsion. Raising the model's foot friction helped but was not by itself
sufficient.

---

## 4. Limitations

- **Directional asymmetry.** The clean walk is only obtained in **one** direction — and
  that direction is *backward* relative to the robot's heading. Commanding the gait in the
  **forward** direction (the way the robot faces) does not produce clean locomotion: the
  robot takes an initial step or two, then shuffles in place or drifts weakly and needs
  constant re-acceleration. Watching the feet, the swing foot visibly slides as it plants
  in that direction (it gets "pulled back" on touchdown) instead of gripping.

- The gait's fore-aft foot trajectory is **symmetric in code** (verified: reversing the
  stride sign mirrors the swing/stance trajectory exactly), so the asymmetry is a
  foot–ground **contact** effect in the simulation rather than a sign error in the gait.

- Extensive live tuning of stride length, swing height, cycle time, weight-shift, and
  foot/floor friction did not remove the asymmetry. A cosine-eased swing (zero horizontal
  foot velocity at touchdown) reduced sliding but did not fix the forward direction.

- The gait is **open-loop**: there is no balance feedback. At aggressive stride/height/
  speed settings the robot can tip over, and it does not recover.

- The walk is **slow**, which is inherent to a statically-stable crawl (only one leg
  moves at a time, three always planted); speed would require a dynamic gait.

**Contributing factors:**
- The simulator exposes only a low-level interface (per-joint position/torque with PD
  gains) and has no built-in gait or balance controller, so all locomotion has to be
  provided externally.
- An open-loop, hand-tuned static gait is sensitive to foot-contact timing, friction,
  swing trajectory, and center-of-mass placement.
- The current dev machine (M2 MacBook Air running Docker + MuJoCo + ROS 2 + Foxglove) is
  near its performance limit, which makes the tuning loop slow.

All of the above (IK, crawl gait, teleop gait, `odom` publisher, scene markers) is
committed and documented in the repository.
