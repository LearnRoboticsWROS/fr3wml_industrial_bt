# 🦾 fr3wml_industrial_bt

> **FR3WML cobot pick-and-place application** — blind + vision-driven — built on the `industrial_bt_framework` Behavior Tree stack.

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-Apache%202.0-green)](LICENSE)
[![Robot: FR3WML](https://img.shields.io/badge/Robot-Fairino%20FR3WML-red)](https://www.fair-innovation.com/)
[![Camera: D455](https://img.shields.io/badge/Camera-RealSense%20D455-purple)](https://www.intelrealsense.com/depth-camera-d455/)

---

## What it is

This package is the **application layer** of a three-tier industrial robotics architecture. It takes the **robot-agnostic** [`industrial_bt_framework`](https://github.com/LearnRoboticsWROS/industrial_bt_framework) and turns it into a **fully working pick-and-place cell** for:

- 🤖 Fairino **FR3WML** 6-DoF cobot
- 🎯 RealSense **D455** depth camera (base-mounted)
- 🌀 Soft gripper (DH AG-95-style) + suction cup
- 💊 Bottle + capsule assembly task

The end goal of the cycle: **pick a bottle with the soft gripper → place it on the table → pick a capsule with the suction cup using vision → place the capsule on top of the bottle**.

Two pipelines are supported out of the box:

| Pipeline | BT XML | Capsule pose source |
|---|---|---|
| 🧠 **Blind** | `bottle_capsule_task.xml` | Hardcoded YAML coordinates |
| 👁️ **Vision** | `bottle_capsule_task_vision.xml` | YOLO + 6D pose estimator on the Jetson |

---

## Where it sits in the stack

```
┌─────────────────────────────────────────────────────────┐
│ APPLICATION   fr3wml_industrial_bt  ← YOU ARE HERE      │
│   • BT XML recipes (blind + vision)                     │
│   • YAML config (poses, tools, profiles, scene)         │
│   • FairinoMoveLBackend pluginlib                       │
│   • URDF assembly (FR3WML + camera + gripper)           │
│   • ArUco calibration tool                              │
└─────────────────────────────────────────────────────────┘
                            ▼
┌─────────────────────────────────────────────────────────┐
│ FRAMEWORK     industrial_bt_framework                   │
│   • BT executors, reusable bricks, scene manager        │
└─────────────────────────────────────────────────────────┘
                            ▼
┌─────────────────────────────────────────────────────────┐
│ DRIVERS       fairino_bridge + fairino_gripper          │
└─────────────────────────────────────────────────────────┘
```

---

## Repository layout

```
fr3wml_industrial_bt/
├── bt_trees/                  ← Behavior Tree recipes
│   ├── bottle_capsule_task.xml             # blind pipeline
│   ├── bottle_capsule_task_visual.xml      # blind + Groot2 ports
│   ├── bottle_capsule_task_vision.xml      # vision pipeline
│   └── bottle_capsule_task_vision_visual.xml
├── config/
│   ├── bt_runner.yaml                      # base config (tools, scene, params)
│   ├── bt_runner_vision.yaml               # vision-specific overlay
│   ├── motion_profiles.yaml                # per-segment speed profiles
│   ├── scene_bottle_capsule.yaml           # planning scene
│   └── fairino_movel_backend.yaml          # vendor backend tuning
├── launch/
│   ├── industrial_bt_demo.launch.py        # blind, headless
│   ├── industrial_bt_visual_demo.launch.py # blind + Groot2 + RViz
│   ├── industrial_bt_vision_demo.launch.py # vision, headless
│   ├── industrial_bt_visual_vision_demo.launch.py  # vision + Groot2
│   ├── calibration_bringup.launch.py       # minimal MoveIt for ArUco
│   └── bringup_fr3wml.launch.py            # MoveIt-only sandbox
├── plugins/
│   └── fairino_movel_backend.cpp           # vendor CartesianBackend impl
├── urdf/
│   └── fr_camera_gripper.urdf.xacro        # full robot+cam+gripper assembly
├── meshes/                                  # gripper + suction cup STLs
├── rviz/                                    # RViz configs
└── tools/
    └── aruco_calibration_node.py           # hand-eye calibration
```

---

## Installation

Prerequisites:
- ROS 2 Humble
- MoveIt 2
- `industrial_bt_framework` (see [related repos](#related-repositories))
- `fairino_bridge` + `fairino_gripper` (the driver layer)
- `fr3wml_fr5_camera_gripper_moveit_config` (the MoveIt config)

```bash
cd ~/fr3wml_ws/src
git clone https://github.com/LearnRoboticsWROS/fr3wml_industrial_bt.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select fr3wml_industrial_bt --symlink-install
source install/setup.bash
```

---

## Usage

### Blind demo (PC only, no camera required)

```bash
ros2 launch fr3wml_industrial_bt industrial_bt_visual_demo.launch.py
```

Brings up RViz + Groot2 publisher + the BT runner. The robot performs the full bottle + capsule cycle using **hardcoded** coordinates from `bt_runner.yaml`.

### Vision demo (requires Jetson YOLO + 6D pose nodes streaming)

```bash
ros2 launch fr3wml_industrial_bt industrial_bt_visual_vision_demo.launch.py
```

The capsule pose comes from the live `PoseArray` on `/detected_objects/poses_6d` — published by [`inference_running_jetson`](https://github.com/LearnRoboticsWROS/inference_running_jetson) + [`sixd_pose_pcl`](https://github.com/LearnRoboticsWROS/sixd_pose_pcl).

> Make sure `ROS_DOMAIN_ID` matches between the PC and the Jetson container.

### Calibration

To calibrate the base→camera transform after physically moving the camera:

```bash
# 1. Drag-teach mode on the robot
ros2 service call /fairino_remote_command_service \
  fairino_msgs/srv/RemoteCmdInterface "{cmd_str: 'Mode(1)'}"

# 2. Minimal MoveIt
ros2 launch fr3wml_industrial_bt calibration_bringup.launch.py

# 3. Run the ArUco calibration
python3 src/fr3wml_industrial_bt/tools/aruco_calibration_node.py --ros-args \
  -p marker_size:=0.090 -p marker_id:=0 \
  -p tool_z_offset:=0.190 -p num_samples:=50
```

The tool prints a `<joint name="camera_joint">` snippet → paste it into `urdf/fr_camera_gripper.urdf.xacro` and rebuild.

---

## Key design choices

### 🌳 Behavior Tree everywhere

Every motion is a BT node — no hardcoded sequences in C++. To change the cycle you edit XML, not code.

### 🔌 Vendor IK via the SDK, not MoveIt

The `FairinoMoveLBackend` pluginlib implementation routes every `ExecuteCartesianSegment` brick to the vendor's `/fairino/movel_pose` service. This avoids the **j6 spin / IK-branch mismatch** problem that arises when MoveIt and the SDK pick different IK solutions for the same Cartesian target.

### 🎯 All-MoveL pipeline (vision pick)

The capsule pick path is **entirely Cartesian** (`SDK MoveL`) from pre-pick to retreat — no PTP between waypoints — to guarantee that all four poses lie on the same IK branch.

### 🧭 Empirical bias correction

The detected capsule pose is corrected by a configurable `capsule_pick_correction_xyz` (in `bt_runner_vision.yaml`) BEFORE `ComputeSuctionPickPose`. This compensates the small systematic offset between the detected centroid and the real one.

### 📐 Joint-config similarity rule

Consecutive PTP waypoints are taught in similar joint configurations to avoid pathological PTP paths (long swings, near-singularities). See `pre_pick_bottle_joints_deg` ↔ `pre_place_bottle_joints_deg` in `bt_runner.yaml`.

---

## Configuration cheat-sheet

| Key | File | Purpose |
|---|---|---|
| `tools:` | `bt_runner.yaml` | Trigger service registry (gripper open/close, suction on/off) |
| `task_parameters:` | `bt_runner.yaml` | Poses, joint angles, retreats — everything the BT XML reads with `{key}` |
| `motion_profiles:` | `motion_profiles.yaml` | Per-segment vel/accel + vendor MoveL speed |
| `capsule_pose_topic` | `bt_runner_vision.yaml` | PoseArray topic from the Jetson |
| `capsule_pick_correction_xyz` | `bt_runner_vision.yaml` | Empirical XYZ bias compensation |
| `capsule_suction_tool_offset_z` | `bt_runner_vision.yaml` | wrist3 → suction-cup tip Z offset |

---

## Related repositories

| Layer | Repository |
|---|---|
| Framework | [industrial_bt_framework](https://github.com/LearnRoboticsWROS/industrial_bt_framework) |
| Robot driver | [fairino_bridge](https://github.com/LearnRoboticsWROS/fairino_bridge_master_class) |
| Gripper driver | [fairino_gripper](https://github.com/LearnRoboticsWROS/fairino_bridge_gripper_master_class) |
| MoveIt config | [fr3wml_fr5_camera_gripper_moveit_config](https://github.com/LearnRoboticsWROS/fr3wml_camera_gripper_moveit_config_master_class) |
| Vendor SDK | [frcobot_ros2](https://github.com/FAIR-INNOVATION/frcobot_ros2) |
| Jetson YOLO | [inference_running_jetson](https://github.com/LearnRoboticsWROS/inference_running_jetson) |
| Jetson 6D pose | [sixd_pose_pcl](https://github.com/LearnRoboticsWROS/sixd_pose_pcl) |
| Monolithic reference | [fr3wml_camera_gripper](https://github.com/LearnRoboticsWROS/fr3wml_camera_gripper_master_class) |

---

## License

Apache 2.0. See [LICENSE](LICENSE).

---

Built with ❤️ for Learn Robotics with ROS.
