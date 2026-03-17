# Viam Rover 2 to ROS2 Migration Plan

## Overview

Migrate Viam Rover 2 from Viam middleware to ROS2 Humble, creating a complete ros2_control-based differential drive platform with IMU, camera, power monitoring, and Gazebo simulation support.

**Target System**: Raspberry Pi 5 + ROS2 Humble + Gazebo

---

## Hardware Specifications

| Component | Details |
|-----------|---------|
| **Motors** | 2x Brushed DC motors with gearbox (Max 120 RPM) |
| **Encoders** | Single-phase, 1992 ticks/rotation (post-gearbox) |
| **Motor Driver** | GPIO-based H-Bridge (PWM + A/B direction pins) |
| **IMU** | MPU6050 6-axis (I2C bus 1, addr 0x68) |
| **Power Monitor** | INA219 (I2C bus 1, addr 0x40) |
| **Camera** | USB webcam (Jieli Technology) |
| **Compute** | Raspberry Pi 5 |
| **Chassis** | Differential drive (2 powered wheels + caster) |

### Wheel Specifications

| Parameter | Value |
|-----------|-------|
| **Wheel Diameter** | 121.3 mm |
| **Wheel Radius** | 60.65 mm (0.06065 m) |
| **Wheel Circumference** | 381 mm |
| **Wheel Separation** | 356 mm (0.356 m) |
| **Ticks per Rotation** | 1992 |
| **Max RPM** | 120 |
| **Spin Slip Factor** | 1.0 |

### GPIO Pin Mapping (Physical → BCM)

| Function | Physical Pin | BCM GPIO |
|----------|-------------|----------|
| **Left Motor PWM** | 37 | GPIO 26 |
| **Left Motor A** | 35 | GPIO 19 |
| **Left Motor B** | 33 | GPIO 13 |
| **Right Motor PWM** | 15 | GPIO 22 |
| **Right Motor A** | 31 | GPIO 6 |
| **Right Motor B** | 29 | GPIO 5 |
| **Left Encoder** | 38 | GPIO 20 |
| **Right Encoder** | 40 | GPIO 21 |

*Note: Single-phase encoders (not quadrature). Direction inferred from motor commands.*

---

## Responsibilities

### What Claude Will Create

| Deliverable | Description |
|-------------|-------------|
| **rover2_base/** | C++ ros2_control hardware interface for motors/encoders |
| **rover2_description/** | Complete URDF/Xacro with collision, inertia, TF |
| **rover2_bringup/** | Launch files and config for real robot |
| **rover2_gazebo/** | Simulation launch, world file, Gazebo plugins |
| **rover2_sensors/** | Python node for INA219 power monitoring |
| **rover2_msgs/** | Custom message definitions (if needed) |
| **scripts/hardware_tests/** | Standalone Python scripts to test GPIO, encoders, I2C |
| **scripts/setup_pi5.sh** | Automated ROS2 Humble installation script for Pi 5 |
| **Documentation** | README with build/run instructions |

### What You Need To Do

#### On Mac (One-Time Setup)
| Task | How |
|------|-----|
| Install ROS2 Humble | Docker: `docker pull osrf/ros:humble-desktop` OR Robostack conda |
| Clone this repo | Already done |

#### On Mac (Each Development Session)
| Task | How |
|------|-----|
| Build packages | `colcon build --symlink-install && source install/setup.bash` |
| Run simulation | `ros2 launch rover2_bringup simulation.launch.py` |
| Test with teleop | `ros2 run teleop_twist_keyboard teleop_twist_keyboard` |

#### On Pi 5 (One-Time Setup)
| Task | How |
|------|-----|
| Install Ubuntu 22.04 | Raspberry Pi Imager → Ubuntu Server 22.04 (64-bit) |
| Run setup script | `./scripts/setup_pi5.sh` (installs ROS2 + dependencies) |
| Enable I2C | `sudo raspi-config` → Interface Options → I2C → Enable |
| Add GPIO permissions | `sudo usermod -aG gpio $USER` then logout/login |
| Clone this repo | `git clone <repo-url>` |

#### On Pi 5 (Hardware Verification - Before ROS2)
| Task | How | What to Check |
|------|-----|---------------|
| Test motors | `python3 scripts/hardware_tests/test_motors.py` | Wheels spin correct direction |
| Test encoders | `python3 scripts/hardware_tests/test_encoders.py` | ~1992 ticks per wheel rotation |
| Test I2C | `i2cdetect -y 1` | Shows 0x40 and 0x68 |
| Test IMU | `python3 scripts/hardware_tests/test_imu.py` | Reasonable accel/gyro values |

#### On Pi 5 (Running the Robot)
| Task | How |
|------|-----|
| Build packages | `colcon build --symlink-install && source install/setup.bash` |
| Launch robot | `ros2 launch rover2_bringup robot.launch.py` |
| Drive with teleop | `ros2 run teleop_twist_keyboard teleop_twist_keyboard` |

#### Physical Tasks (Cannot Be Automated)
| Task | When | Notes |
|------|------|-------|
| Confirm motor direction | During `test_motors.py` | If wrong, swap A/B pins in config |
| Rotate wheels for encoder test | During `test_encoders.py` | Count should reach ~1992 per rotation |
| Odometry calibration | After full integration | Drive 1m, measure actual, adjust wheel_radius |
| IMU calibration | At robot startup | Keep robot stationary for 5 seconds |

---

## Development Workflow

**Two-Machine Setup:**
- **Mac (Development)**: Build packages, run Gazebo simulation, iterate on code
- **Pi 5 (Hardware)**: Run real robot, hardware validation tests

**Incremental Verification Strategy:**
1. Simulation-first development on Mac
2. Hardware test scripts to verify GPIO, encoders, I2C before full integration
3. Same ROS2 topics in sim and real for seamless transition

---

## Package Structure

```
rover-1/  # This repo
├── rover2_base/                 # C++ hardware interface (ros2_control)
│   ├── include/rover2_base/
│   │   ├── l298n_driver.hpp     # L298N motor abstraction
│   │   ├── gpio_encoder.hpp     # Encoder reader via lgpio
│   │   └── rover2_hardware.hpp  # ros2_control SystemInterface
│   ├── src/
│   │   ├── l298n_driver.cpp
│   │   ├── gpio_encoder.cpp
│   │   └── rover2_hardware.cpp
│   ├── config/
│   │   └── rover2_controllers.yaml
│   └── rover2_base.xml          # Plugin export
│
├── rover2_description/          # URDF, meshes, visualization
│   ├── urdf/
│   │   ├── rover2.urdf.xacro
│   │   ├── rover2.ros2_control.xacro
│   │   └── rover2.gazebo.xacro
│   ├── meshes/                  # STL files (from Viam CAD)
│   ├── rviz/rover2.rviz
│   └── launch/view_robot.launch.py
│
├── rover2_bringup/              # Launch files and configs
│   ├── launch/
│   │   ├── robot.launch.py      # Full robot bringup
│   │   ├── simulation.launch.py # Gazebo simulation
│   │   └── sensors.launch.py    # IMU + camera
│   └── config/
│       ├── mpu6050.yaml
│       ├── camera.yaml
│       └── ina219.yaml
│
├── rover2_gazebo/               # Simulation support
│   ├── worlds/
│   │   └── rover2_world.sdf
│   └── launch/
│       └── gazebo.launch.py
│
├── rover2_sensors/              # Python sensor nodes
│   ├── rover2_sensors/
│   │   └── ina219_node.py       # Power monitor
│   └── setup.py
│
└── rover2_msgs/                 # Custom messages (optional)
    └── msg/BatteryStatus.msg
```

---

## Implementation Phases

### Phase 1: Project Setup & URDF (Days 1-2)

**Tasks:**
1. Create ROS2 workspace and all package skeletons
2. Build complete URDF with:
   - Base link, chassis, wheels, caster
   - IMU link, camera link with optical frame
   - Collision and inertia properties
3. Create ros2_control hardware tag in URDF
4. Test visualization in RViz2

**Deliverables:**
- All packages compile
- Robot visible in RViz with correct TF tree

**Critical Files:**
- `rover2_description/urdf/rover2.urdf.xacro`
- `rover2_description/urdf/rover2.ros2_control.xacro`

---

### Phase 2: ros2_control Hardware Interface (Days 3-6)

**Tasks:**
1. Implement L298N driver class using lgpio (Pi 5 compatible)
2. Implement GPIO encoder class with interrupt-based counting
3. Create ros2_control SystemInterface:
   - `on_init()`: Parse URDF parameters
   - `on_configure()`: Initialize GPIO
   - `on_activate()`: Start encoder counting
   - `read()`: Get encoder positions/velocities
   - `write()`: Send PWM to motors
4. Configure diff_drive_controller
5. Test with teleop_twist_keyboard

**Key Configuration** (`rover2_controllers.yaml`):
```yaml
diff_drive_controller:
  ros__parameters:
    left_wheel_names: ["left_wheel_joint"]
    right_wheel_names: ["right_wheel_joint"]
    wheel_separation: 0.356    # 356 mm from Viam spec
    wheel_radius: 0.06065      # 60.65 mm from Viam spec
    publish_rate: 50.0
    enable_odom_tf: true
    linear.x.max_velocity: 0.76  # ~120 RPM * 0.381m circumference / 60
    angular.z.max_velocity: 2.0
```

**Deliverables:**
- Motors respond to /cmd_vel
- Odometry publishes on /odom
- TF: odom → base_link

**Critical Files:**
- `rover2_base/src/rover2_hardware.cpp`
- `rover2_base/include/rover2_base/l298n_driver.hpp`
- `rover2_base/config/rover2_controllers.yaml`

---

### Phase 3: Sensor Integration (Days 7-8)

**IMU (MPU6050):**
- Use existing package: [ros2_mpu6050_driver](https://github.com/hiwad-aziz/ros2_mpu6050_driver)
- Publishes: `/imu/data` (sensor_msgs/Imu)
- Run calibration on startup

**Camera:**
- Use `usb_cam` package
- Publishes: `/camera/image_raw`, `/camera/camera_info`
- 720p @ 30fps

**Power Monitor (Optional):**
- Custom Python node for INA219
- Publishes: `/battery_state` (sensor_msgs/BatteryState)

**Deliverables:**
- IMU data in RViz
- Camera image streaming
- Battery voltage monitoring

**Critical Files:**
- `rover2_bringup/config/mpu6050.yaml`
- `rover2_bringup/config/camera.yaml`
- `rover2_sensors/rover2_sensors/ina219_node.py`

---

### Phase 4: Gazebo Simulation (Days 9-10)

**Tasks:**
1. Create Gazebo URDF plugins file
2. Configure `gazebo_ros2_control` plugin
3. Add differential drive plugin for simulation
4. Create test world with obstacles
5. Verify simulation matches real hardware behavior

**Gazebo Plugins** (`rover2.gazebo.xacro`):
```xml
<gazebo>
  <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
    <parameters>$(find rover2_base)/config/rover2_controllers.yaml</parameters>
  </plugin>
</gazebo>

<!-- IMU sensor plugin -->
<gazebo reference="imu_link">
  <sensor name="imu" type="imu">
    <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
      <ros><namespace>/</namespace></ros>
      <topicName>/imu/data</topicName>
    </plugin>
  </sensor>
</gazebo>

<!-- Camera plugin -->
<gazebo reference="camera_link">
  <sensor type="camera" name="camera">
    <camera><horizontal_fov>1.047</horizontal_fov>...</camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <cameraName>camera</cameraName>
    </plugin>
  </sensor>
</gazebo>
```

**Deliverables:**
- Working Gazebo simulation
- Same ROS2 topics as real robot
- Test environment for SLAM/nav development

**Critical Files:**
- `rover2_description/urdf/rover2.gazebo.xacro`
- `rover2_gazebo/worlds/rover2_world.sdf`
- `rover2_gazebo/launch/gazebo.launch.py`

---

### Phase 5: Launch Files & Integration (Days 11-12)

**Tasks:**
1. Create modular launch files
2. Add parameter files for all nodes
3. Test full system integration
4. Document startup procedures

**Main Launch File** (`robot.launch.py`):
- Loads robot_description
- Starts controller_manager with hardware interface
- Spawns diff_drive_controller and joint_state_broadcaster
- Launches IMU, camera, power monitor nodes

**Deliverables:**
- Single command to launch entire robot
- Separate launch for simulation vs hardware

**Critical Files:**
- `rover2_bringup/launch/robot.launch.py`
- `rover2_bringup/launch/simulation.launch.py`

---

### Phase 5.5: Hardware Verification Scripts (Pi 5 Only)

**Purpose:** Verify hardware works BEFORE full ROS2 integration. Run these standalone Python scripts to validate GPIO, encoders, and I2C.

**Scripts to Create** (`scripts/hardware_tests/`):

1. **`test_motors.py`** - Spin each motor forward/backward
   ```python
   # Test left motor: PWM=26, A=19, B=13
   # Test right motor: PWM=22, A=6, B=5
   # Visual confirmation: wheels spin correct direction
   ```

2. **`test_encoders.py`** - Count encoder pulses while manually rotating wheels
   ```python
   # Left encoder: GPIO 20
   # Right encoder: GPIO 21
   # Expected: ~1992 ticks per full rotation
   ```

3. **`test_i2c.py`** - Verify IMU and power sensor respond
   ```bash
   i2cdetect -y 1  # Should show 0x40 (INA219) and 0x68 (MPU6050)
   ```

4. **`test_imu.py`** - Read MPU6050 acceleration/gyro values

5. **`test_power.py`** - Read battery voltage/current from INA219

**Verification Checkpoints:**
- [ ] Motors spin in correct direction
- [ ] Encoder ticks match expected count (~1992/rotation)
- [ ] I2C devices detected at correct addresses
- [ ] IMU returns reasonable accel/gyro values
- [ ] Power sensor shows battery voltage

---

### Phase 6: Testing & Calibration (Days 13-14)

**Hardware Tests:**
1. Verify each GPIO pin function
2. Test encoder accuracy (drive 1m, measure actual distance)
3. Calibrate wheel_separation and wheel_radius
4. Run IMU calibration routine
5. Verify camera intrinsics

**Integration Tests:**
1. Drive square pattern, measure odometry drift
2. Compare simulation vs real behavior
3. Test emergency stop behavior
4. Verify all TF frames correct

**Odometry Calibration Procedure:**
```bash
# Mark 1 meter on floor
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}}"
# Measure actual distance traveled
# Adjust wheel_radius = wheel_radius * (commanded / actual)
```

---

## TF Tree

```
odom
└── base_link
    ├── base_footprint
    ├── chassis
    │   ├── imu_link
    │   ├── camera_link
    │   │   └── camera_optical_frame
    │   └── caster_link
    ├── left_wheel
    └── right_wheel
```

---

## Dependencies

### Mac (Simulation Development)
```bash
# Install ROS2 Humble via Homebrew or Docker
# Option 1: Docker (recommended for Mac)
docker pull osrf/ros:humble-desktop

# Option 2: Robostack (native conda environment)
conda create -n ros_env python=3.10
conda activate ros_env
conda install -c conda-forge -c robostack-staging ros-humble-desktop
```

### Pi 5 (Hardware)

**ROS2 Humble Installation:**
```bash
# Ubuntu 22.04 on Pi 5
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl

# Add ROS2 repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt install -y ros-humble-ros-base ros-humble-ros2-control ros-humble-ros2-controllers
```

**System Packages (Pi 5):**
```bash
sudo apt install -y \
  ros-humble-xacro \
  ros-humble-usb-cam \
  ros-humble-teleop-twist-keyboard \
  i2c-tools \
  python3-lgpio \
  python3-smbus2
```

**Enable I2C:**
```bash
sudo raspi-config  # Interface Options → I2C → Enable
```

**GPIO Permissions:**
```bash
sudo usermod -aG gpio $USER
# Logout and login for changes to take effect
```

### Clone from Source
```bash
# MPU6050 driver (in this repo)
cd /path/to/rover-1
git clone https://github.com/hiwad-aziz/ros2_mpu6050_driver src/ros2_mpu6050_driver
```

---

## Pi 5 GPIO Notes

The Raspberry Pi 5 uses a different GPIO library than Pi 4:
- **Use `lgpio`** instead of `pigpio` (not compatible with Pi 5)
- Hardware PWM available on GPIO 12, 13, 18, 19
- May need to run with elevated permissions for GPIO access

```bash
# Add user to gpio group
sudo usermod -aG gpio $USER
```

---

## Future Enhancements (Post-Migration)

1. **SLAM**: Add `slam_toolbox` with RPLiDAR
2. **Navigation**: Integrate `nav2` stack
3. **Sensor Fusion**: Use `robot_localization` for IMU+odometry EKF
4. **Computer Vision**: Add object detection nodes
5. **Web Interface**: Create web dashboard for monitoring

---

## Verification Checklist

- [ ] All packages compile without errors
- [ ] URDF loads correctly in RViz
- [ ] TF tree shows expected frames
- [ ] Motors respond to /cmd_vel commands
- [ ] Odometry updates on /odom
- [ ] IMU data on /imu/data
- [ ] Camera image on /camera/image_raw
- [ ] Simulation matches hardware behavior
- [ ] Robot drives straight (no drift)
- [ ] Rotation matches commanded angle

---

## Quick Start Commands

### Mac (Simulation)
```bash
# Build workspace
cd /path/to/rover-1
colcon build --symlink-install
source install/setup.bash

# Launch simulation
ros2 launch rover2_bringup simulation.launch.py

# Teleoperation (in another terminal)
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# View in RViz
ros2 launch rover2_description view_robot.launch.py
```

### Pi 5 (Hardware)
```bash
# Run hardware verification first
cd /path/to/rover-1/scripts/hardware_tests
python3 test_motors.py
python3 test_encoders.py
python3 test_i2c.py

# Build workspace
cd /path/to/rover-1
colcon build --symlink-install
source install/setup.bash

# Launch real robot
ros2 launch rover2_bringup robot.launch.py

# Teleoperation (from Mac or Pi)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Sources

- [Viam Rover 2 Docs](https://docs.viam.com/dev/reference/try-viam/rover-resources/rover-tutorial/)
- [Viam Rover 2 GitHub](https://github.com/viamrobotics/Viam-Rover-2)
- [ros2_control Humble Docs](https://control.ros.org/humble/)
- [diff_drive_controller](https://control.ros.org/humble/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html)
- [ros2_mpu6050_driver](https://github.com/hiwad-aziz/ros2_mpu6050_driver)
- [lgpio Library (Pi 5)](https://abyz.me.uk/lg/py_lgpio.html)
