# my_simulation

A complete ROS Noetic simulation package for PX4 + VINS-Fusion + Ego Planner with depth camera obstacle avoidance.

## Prerequisites

- Ubuntu 20.04
- ROS Noetic
- PX4-Autopilot (SITL setup)
- MAVROS

## 1. Installation

### 1.1 Install Dependencies
```bash
sudo apt update
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-depth-image-proc ros-noetic-pcl-ros ros-noetic-tf2-sensor-msgs ros-noetic-realtime-tools ros-noetic-joy
sudo apt install libceres-dev libgoogle-glog-dev libatlas-base-dev
```

### 1.2 Clone and Compile VINS-Fusion and Ego Planner
It is recommended to use tested versions or standard repos.

**VINS-Fusion:**
```bash
cd ~/catkin_ws/src
git clone https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git
```

**Ego Planner:**
```bash
cd ~/catkin_ws/src
git clone https://github.com/ZJU-FAST-Lab/ego-planner.git
# Note: ego-planner might need its own submodules or specific branch (usually 'master' is fine).
```

### 1.3 Compile Workspace
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 2. Configuration

### 2.1 PX4 Setup
Ensure `PX4-Autopilot` is installed. The launch files assume `mavros_posix_sitl.launch` is available (usually via `px4` package or if you sourced PX4 environment).
If you built PX4 from source, source its setup first:
```bash
cd <PX4-Autopilot_path>
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic
```

### 2.2 VINS Config Check
Verify `config/vins_config.yaml` points to `cam0_pinhole.yaml` correctly.

## 3. Running the Simulation

You can launch everything with one command:

```bash
roslaunch my_simulation sim_all.launch
```

### What happens:
1.  **Gazebo** starts with `iris_with_depth` in `obstacle.world`.
2.  **MAVROS** connects to PX4 SITL.
3.  **VINS-Fusion** estimates pose from Mono Camera + IMU.
4.  **Ego Planner** generates trajectory avoiding obstacles.
5.  **Trajectory Tracker** commands the drone to follow the path.
6.  **RViz** visualizes the depth, pointcloud, and path.

### Manual Control / Testing
To start the mission (if not automatic):
1.  Arm the drone (The tracker script attempts to OFFBOARD and ARM automatically).
2.  Send a goal in RViz (2D Nav Goal) if Ego-Planner supports it, or use the default target in the planner config.

## 4. Troubleshooting
-   **No Pointcloud?** Check `rostopic hz /iris/depth_camera/points`. If 0, check `depth_image_proc` node in `ego_planner.launch`.
-   **VINS Drifting?** Simulation should be perfect, but ensure IMU noise params match PX4 SITL IMU (usually low noise).
-   **Drone not taking off?** Check `mavros/state` to see if connected and armed.

