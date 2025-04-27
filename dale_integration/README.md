# dale_integration

## Overview

`dale_integration` is a ROS 2 package designed to coordinate and integrate all core subsystems of the **DalESelfieBot** system. It provides a centralized launch file to simplify the deployment of multiple components required for operation.

This package integrates the following subsystems:
- `img_prc` (Image Processing)
- `tool_path_planning` (Path Generation)
- `ur3_control` (Motion Execution)
- `ur_moveit_config` (Motion Planning)

---

## System Startup Instructions

### 1. Perform Calibration
Use the following command, replacing `<>` with your robot’s IP address and `"path"` with your calibration file:
```bash
ros2 launch ur_calibration calibration_correction.launch.py \
  robot_ip:=<ROBOT_IP> target:="path/to/your_config.yaml"
```

### 2. Connect to the Robot
Use the following command, replacing `<>` with your robot’s IP address and `"path"` with your calibration file:
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=<ROBOT_IP> launch_rviz:=true kinematics_config:="path/to/your_config.yaml"
```

### 3. Perform Localisation
Run the localisation node:
```bash
ros2 run ur3_localisation localisation_node
```

Save each position by running (as needed):
```bash
ros2 topic pub /save_position std_msgs/msg/Empty "{}" -1
```

### 4. Launch the Full System
```bash
ros2 launch dale_integration dale_launch.py
```

### 5. Launch the GUI
Command to be added here when finalized.

### 6. Follow the User Manual
Refer to the official DalESelfieBot user manual for operation guidance.

---

## Launch Files

This package provides the following main launch file:

### `dale_launch.py`
This file launches the following components:
- **Motion Planning:**
  ```bash
  ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e launch_rviz:=false
  ```
- **Motion Execution:**
  ```bash
  ros2 launch ur3_control moveit_stack.launch.py ur_type:=ur3e launch_rviz:=false
  ```
- **Image Processing:**
  _Launch command to be added (TBA)_

- **Tool Path Planning:**
  _Launch command to be added (TBA)_

---

## Maintainer
Jarred Deluca  
[jarred.g.deluca@student.uts.edu.au](mailto:jarred.g.deluca@student.uts.edu.au)