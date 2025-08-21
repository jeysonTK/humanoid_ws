# Humanoid Project

This project provides a humanoid robot simulation using **ROS 2** and **Gazebo**.

---

## Requirements

- ROS 2 (Humble/Foxy or compatible)
- colcon
- Gazebo

---

## Build

Navigate to your workspace and build the packages:

```bash
cd ~/humanoid_ws
colcon build --symlink-install

## Run

Before running any commands, source the workspace:

```bash
source install/setup.bash
ros2 launch humanoid_gazebo spawn_humanoid.launch.py
