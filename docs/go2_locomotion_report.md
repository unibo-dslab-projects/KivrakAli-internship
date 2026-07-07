# Go2 Locomotion — Progress Report

**Author:** Ali Yağız Kıvrak
**Context:** Erasmus+ research internship, University of Bologna (Cesena campus)
**Topic:** Forward/backward locomotion of the Unitree Go2 in simulation

---

## 1. Objective

The goal for this step was to make the simulated Go2 walk forward and backward on flat
ground, building on the earlier phase (Go2 MuJoCo simulation + ROS 2 low-level control
+ Foxglove 3D visualization), which is already working and committed to the repository.

---

## 2. What I did

Everything below runs on top of the existing stack (headless MuJoCo sim, `unitree_ros2`
messages over CycloneDDS, `foxglove_bridge`, `robot_state_publisher`, Foxglove Studio),
and drives the robot through the same low-level `/lowcmd` interface used by the teleop.

- **Closed-form 3D leg inverse kinematics.** A per-leg IK that places the foot at a
  target `(x_forward, y_lateral, z_up)` relative to the hip, using hip abduction plus a
  2-link (thigh/calf) planar solve. Derived from the URDF link geometry
  (`hip_offset = 0.0955`, `thigh = calf = 0.213`) and verified numerically against
  forward kinematics (round-trip error < 1e-4) and live in the sim.

- **Static crawl gait.** A statically-stable gait where one leg swings at a time
  (duty factor 0.75, so three feet are always in contact), including:
  - a leg swing sequence (FR → RL → FL → RR),
  - body weight-shift toward the support triangle synchronized with each swing,
  - forward propulsion by moving the planted stance feet backward in the body frame,
  - a smooth "settle-then-ramp" start so the robot eases into the stand before walking.
  All commanded joint angles stay within the Go2 joint limits across the full cycle.

- **Contact/parameter tuning in the MuJoCo model.** The default foot friction in the
  Go2 model is low (`0.4`); I raised it (up to `2.0`) and added floor friction to reduce
  foot slipping, and adjusted the simulation timestep for stability/performance.

- **`odom → base` TF publisher.** A small node that republishes the robot's world
  position and IMU orientation from `/sportmodestate` as a TF transform, so the robot
  can be viewed moving across the ground in Foxglove (rather than the camera being
  locked to the robot) to see the direction of travel.

---

## 3. Results and limitations

**What works:**
- The robot stands stably using IK (body height holds around 0.35 m).
- The legs lift and step in the correct sequence, and the weight-shift behaves correctly
  — verified both in the joint/position plots and in the 3D view.
- Joint commands stay within limits; no invalid configurations are commanded.

**Limitations — clean forward locomotion was not achieved:**
- Across many parameter settings, instead of walking forward the robot showed one of:
  in-place shuffling with near-zero net displacement, slow backward drift, an alternating
  forward/backward shuffle with a small net backward motion, or occasional falls at more
  aggressive stride/height settings.
- There is a direction asymmetry: a negative stride parameter produced net motion, while
  a positive stride of the same magnitude produced only in-place shuffling, indicating
  the open-loop foot trajectory does not robustly propel the body in a controlled
  direction.
- Extensive tuning of stride length, cycle time, swing height, weight-shift magnitude,
  foot/floor friction, and simulation timestep did not converge to a stable forward walk.

**Contributing factors:**
- The simulator exposes only a low-level interface (per-joint position/torque with PD
  gains) and has no built-in gait or balance controller, so all locomotion has to be
  provided externally.
- An open-loop, hand-tuned static gait is sensitive to foot-contact timing, friction,
  swing trajectory, and center-of-mass placement; small parameter changes flip the
  behavior between the failure modes above.
- The current dev machine (M2 MacBook Air running Docker + MuJoCo + ROS 2 + Foxglove) is
  near its performance limit, which makes the tuning loop slow.

The manual-gait code (IK, crawl gait, `odom` publisher) is committed and documented, so
this phase is captured even though it did not produce a stable forward walk.
