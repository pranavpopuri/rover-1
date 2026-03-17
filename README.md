# Rover2 ROS2

ROS2 Humble packages for the Viam Rover 2, enabling simulation and (eventually) real hardware control.

## Project Status

| Component | Status |
|-----------|--------|
| **Simulation (Gazebo)** | Ready to test |
| **RViz Visualization** | Ready to test |
| **Real Hardware** | Phase 2 (not yet implemented) |

## Quick Start (Simulation)

### Prerequisites

#### Option 1: Docker (Recommended for Mac)
```bash
# Pull ROS2 Humble desktop image
docker pull osrf/ros:humble-desktop

# Run container with GUI support (Mac)
docker run -it --rm \
  -e DISPLAY=host.docker.internal:0 \
  -v $(pwd):/ws \
  -w /ws \
  osrf/ros:humble-desktop \
  bash
```

#### Option 2: Robostack (Native on Mac)
```bash
# Create conda environment
conda create -n ros2 python=3.10
conda activate ros2

# Install ROS2 Humble
conda install -c conda-forge -c robostack-staging ros-humble-desktop ros-humble-gazebo-ros-pkgs
```

### Build

```bash
# Source ROS2 (if not using Docker)
source /opt/ros/humble/setup.bash

# Build the workspace
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

### Run Simulation

```bash
# Launch Gazebo with rover2 + RViz
ros2 launch rover2_bringup simulation.launch.py

# In another terminal, control with keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### View Robot Only (No Gazebo)

```bash
# View URDF in RViz with joint sliders
ros2 launch rover2_description view_robot.launch.py
```

## Package Structure

```
src/
├── rover2_description/    # URDF, meshes, RViz config
│   ├── urdf/             # Robot description files
│   ├── rviz/             # RViz configuration
│   └── launch/           # Visualization launch files
│
├── rover2_gazebo/         # Simulation support
│   ├── worlds/           # Gazebo world files
│   └── launch/           # Simulation launch files
│
└── rover2_bringup/        # Main launch files
    ├── launch/           # Robot bringup launch files
    └── config/           # Configuration files
```

## ROS2 Topics (Simulation)

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | geometry_msgs/Twist | Velocity commands |
| `/odom` | nav_msgs/Odometry | Odometry from wheel encoders |
| `/imu/data` | sensor_msgs/Imu | IMU data |
| `/camera/image_raw` | sensor_msgs/Image | Camera image |
| `/camera/camera_info` | sensor_msgs/CameraInfo | Camera calibration |
| `/joint_states` | sensor_msgs/JointState | Wheel joint states |
| `/tf` | tf2_msgs/TFMessage | Transform tree |

## TF Frames

```
odom
└── base_link
    ├── base_footprint
    ├── chassis
    │   ├── imu_link
    │   ├── camera_link
    │   │   └── camera_optical_frame
    │   └── caster_mount
    │       └── caster_wheel
    ├── left_wheel
    └── right_wheel
```

## Robot Specifications

| Parameter | Value |
|-----------|-------|
| Wheel Diameter | 121.3 mm |
| Wheel Separation | 356 mm |
| Max Linear Velocity | ~0.76 m/s |
| Max Angular Velocity | ~2.0 rad/s |
| Encoder Ticks/Rotation | 1992 |

## Teleop Controls

When using `teleop_twist_keyboard`:

```
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
```

## Troubleshooting

### Gazebo doesn't start
- Ensure you have `gazebo_ros_pkgs` installed
- Try: `ros2 pkg list | grep gazebo`

### Robot falls through ground
- The spawn height might be too low
- Try: `ros2 launch rover2_bringup simulation.launch.py z_pose:=0.2`

### No camera image
- Check if Gazebo camera plugin is loaded
- Try: `ros2 topic list | grep camera`

### TF errors in RViz
- Wait for Gazebo to fully load
- Check: `ros2 run tf2_tools view_frames`

## Phase 2: Real Hardware (Coming Soon)

The following will be added for Raspberry Pi 5 hardware:

- `rover2_base/` - ros2_control hardware interface for GPIO motors/encoders
- `rover2_sensors/` - INA219 power monitor node
- Hardware test scripts
- Pi 5 setup script

## License

MIT

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Run validation: `python3 scripts/validate.py`
5. Submit a pull request
