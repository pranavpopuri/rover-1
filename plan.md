# Viam Rover 2 to ROS2 Migration Plan

## Overview

Migrate Viam Rover 2 from Viam middleware to ROS2 Jazzy, running entirely on the Raspberry Pi 5. Development proceeds in two stages: first get simulation working on the Pi, then deploy to real hardware.

**Target System**: Raspberry Pi 5 + Ubuntu 24.04 + ROS2 Jazzy + Gazebo

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

## What Exists Already

The following packages have been written and need to be built and tested:

| Package | Contents | Status |
|---------|----------|--------|
| **rover2_description/** | URDF/Xacro (base, mock, hardware variants), materials, inertia macros, RViz config, view_robot launch | Written, untested |
| **rover2_base/** | C++ ros2_control hardware interface (rover2_hardware, gpio_motor, gpio_encoder), controller config, plugin export | Written, untested |
| **rover2_bringup/** | Launch files (robot, mock hardware, teleop), teleop config | Written, untested |
| **rover2_sensors/** | INA219 power monitor Python node, sensor config, sensor launch | Written, untested |
| **scripts/hardware_tests/** | Standalone test scripts for motors, encoders, I2C, IMU, power | Written, untested |
| **scripts/setup_pi5.sh** | Automated Pi 5 setup (installs ROS2 Jazzy + deps) | Written, untested |

---

## Development Workflow

**Single-Machine Setup — everything runs on the Raspberry Pi 5.**

1. Get mock hardware mode working on the Pi (tests control pipeline without GPIO or a simulator)
2. Validate all ROS2 topics and TF frames with mock hardware
3. Run hardware test scripts to verify GPIO, encoders, I2C
4. Switch from mock hardware to real hardware
5. Calibrate odometry and sensors on real hardware

---

## Safety Precautions

**Read this before starting any phase involving hardware.**

### Power Sources

The Pi has two mutually exclusive power sources:

| Source | How | When to Use |
|--------|-----|-------------|
| **USB-C wall adapter** | Plugged into Pi's USB-C port | Phases 1-2 (setup, simulation) |
| **Robot battery** | Via GPIO ribbon cable from motor driver board | Phase 3 onward (hardware testing, driving) |

**CRITICAL: Never connect both at the same time. Having the USB-C adapter plugged in while the ribbon cable is connected will brick the robot.**

- The ribbon cable carries both power (battery → Pi) and GPIO signals (motors, encoders, I2C sensors)
- When on battery power, the Pi will brown out if voltage drops too low — always check voltage before running motors
- When switching power sources, **always shut down the Pi first** (`sudo shutdown now`), then disconnect the old source, connect the new source, and power on

### SD Card Protection

- **Back up the SD card** before starting Phase 1. If a brownout corrupts the filesystem, you can re-flash instead of starting from scratch:
  ```bash
  # From another machine, with the SD card inserted:
  sudo dd if=/dev/sdX of=rover2-backup.img bs=4M status=progress
  ```
- **Shut down cleanly** (`sudo shutdown now`) before disconnecting power. Never just pull the plug.

### GPIO & Wiring Safety

- **Visually verify all wiring matches the GPIO pin table** before connecting the ribbon cable for the first time. A miswired GPIO pin can short the Pi's 3.3V rail and permanently damage it.
- If anything smells hot or a motor doesn't spin, **disconnect power immediately** — don't keep retrying.

### Motor Safety

- **Keep your hand on the power switch** during first motor tests (Phase 3b and 4b). The software emergency stop (`cmd_vel_timeout`) only works if the ROS2 node is running — if the node crashes, motors may keep spinning.
- **Always test on blocks first** (wheels off the ground) before driving on the floor.
- Do not stall motors (hold wheels still while powered) for more than a few seconds — this can burn out the motor driver.

---

## Implementation Phases

### Phase 1: Pi 5 Setup & Build

**Power: USB-C wall adapter. Ribbon cable disconnected.**

**Goal:** Get the ROS2 workspace compiling on the Pi.

**Prerequisites:**
- Back up the SD card (see Safety Precautions above)

**Tasks:**
1. Run `scripts/setup_pi5.sh` to install ROS2 Jazzy and dependencies
2. Reboot, verify ROS2 is sourced (`ros2 --help`)
3. Build the workspace:
   ```bash
   cd ~/github/rover-1
   colcon build --symlink-install
   source install/setup.bash
   ```
4. Fix any compile errors (missing deps, API changes between Humble→Jazzy)

**Deliverables:**
- All packages compile cleanly on the Pi
- `ros2 pkg list | grep rover2` shows all packages

**Potential Issues:**
- C++ code may need minor API adjustments for Jazzy
- `liblgpio` C library may not be in Ubuntu 24.04 repos — the build will fall back to mock GPIO mode, which is fine for Phase 2

---

### Phase 2: Mock Hardware Testing

**Power: USB-C wall adapter. Ribbon cable disconnected.**

**Goal:** Verify the entire ros2_control pipeline works using mock hardware — no GPIO, no simulator, no GPU needed. Mock hardware mirrors velocity commands to state feedback, so the controllers, odometry, and TF all function as if a real robot were moving.

**Tasks:**
1. Launch mock hardware:
   ```bash
   ros2 launch rover2_bringup simulation.launch.py
   ```
2. Verify controllers loaded:
   ```bash
   ros2 control list_controllers
   # Should show diff_drive_controller [active] and joint_state_broadcaster [active]
   ```
3. Verify ROS2 topics are publishing:
   ```bash
   ros2 topic list
   ros2 topic echo /diff_drive_controller/odom          # Should show zeros until cmd_vel is sent
   ros2 topic echo /joint_states  # Should show wheel positions
   ```
4. Drive with teleop in another terminal:
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel
   ```
5. Confirm odometry updates when driving:
   ```bash
   ros2 topic echo /diff_drive_controller/odom
   # Position should change as you send cmd_vel commands
   ```
6. Verify TF tree:
   ```bash
   ros2 run tf2_tools view_frames
   ```
7. Optionally view in RViz (if display available):
   ```bash
   ros2 launch rover2_bringup simulation.launch.py rviz:=true
   ```

**Deliverables:**
- [ ] `diff_drive_controller` and `joint_state_broadcaster` both active
- [ ] `/diff_drive_controller/cmd_vel` commands produce odometry changes on `/diff_drive_controller/odom`
- [ ] `/joint_states` shows wheel positions changing
- [ ] TF tree matches expected frame hierarchy
- [ ] RViz shows robot with wheels spinning (if display available)

**Expected TF Tree:**
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

---

### Phase 3: Hardware Verification (Pre-ROS2)

**Power: Robot battery via ribbon cable. USB-C wall adapter DISCONNECTED.**

**Goal:** Verify all hardware works using standalone scripts and physically confirm the software's assumptions match the real robot.

#### Switching to robot power (one-time)

This is the first phase that requires the ribbon cable. Follow this procedure exactly:

1. `sudo shutdown now` — wait for Pi to fully power off
2. Unplug the USB-C wall adapter
3. Visually verify the ribbon cable wiring matches the GPIO pin table (see Safety Precautions)
4. Connect the ribbon cable to the Pi's GPIO header
5. Turn on the robot's battery power
6. Pi should boot from battery power via the ribbon cable
7. SSH back into the Pi

**From this point forward, the Pi is always powered by the robot battery via the ribbon cable. Do not reconnect the USB-C adapter without first shutting down and disconnecting the ribbon cable.**

#### 3a: Connectivity & Sensor Check

| Step | Command | What to Check |
|------|---------|---------------|
| 1. Verify I2C | `python3 scripts/hardware_tests/test_i2c.py` | Devices at 0x40 (INA219) and 0x68 (MPU6050) |
| 2. Test IMU | `python3 scripts/hardware_tests/test_imu.py` | Reasonable accel/gyro values |
| 3. Test power sensor | `python3 scripts/hardware_tests/test_power.py` | Battery voltage reads correctly |

**Physical verification:**
- [ ] Tilt robot forward — IMU X-axis accel increases
- [ ] Tilt robot left — IMU Y-axis accel increases
- [ ] IMU Z-axis reads ~9.8 m/s² when stationary (gravity)
- [ ] Battery voltage matches multimeter reading (within 0.2V)

#### 3b: Motor & Encoder Check

**Before running any motor test:**
1. Visually verify all motor and encoder wiring matches the GPIO pin table (see Safety Precautions)
2. Confirm battery voltage is above 13.0V (from step 3 in 3a)
3. Place the robot on blocks so wheels are off the ground
4. Keep your hand on the power switch

| Step | Command | What to Check |
|------|---------|---------------|
| 4. Test motors | `python3 scripts/hardware_tests/test_motors.py` | Wheels spin |
| 5. Test encoders | `python3 scripts/hardware_tests/test_encoders.py` | Ticks count up |

**Physical verification (robot on blocks, wheels off ground):**
- [ ] "Forward" command spins both wheels so the robot would move forward — if not, swap A/B pins in config and re-test
- [ ] Left and right motors spin at roughly the same speed at the same PWM
- [ ] Encoder ticks per full wheel rotation: record actual count (expected ~1992). If significantly different, update `encoder_ticks_per_rotation` in URDF and controller config

#### 3c: Measure & Reconcile Physical Dimensions

The URDF and controller config assume dimensions from the Viam spec sheet. Measure the actual robot and update configs if they differ.

| Parameter | Spec Value | How to Measure | Config to Update |
|-----------|-----------|----------------|------------------|
| Wheel diameter | 121.3 mm | Calipers on wheel | `wheel_radius` in `rover2_controllers.yaml` and URDF |
| Wheel separation | 356 mm | Center-to-center distance between left and right wheel contact patches | `wheel_separation` in `rover2_controllers.yaml` and URDF |
| Encoder ticks/rotation | 1992 | Recorded from step 3b | `encoder_ticks_per_rotation` in `rover2.ros2_control.xacro` |

**Deliverables:**
- [ ] All I2C devices respond
- [ ] IMU axes orientation confirmed physically
- [ ] Motors spin in correct direction
- [ ] Encoder ticks/rotation recorded and config updated if needed
- [ ] Wheel diameter and separation measured and config updated if needed

---

### Phase 4: Real Hardware Bringup

**Power: Robot battery via ribbon cable.**

**Goal:** Run the rover on real hardware using ros2_control and verify basic motion.

#### 4a: Launch & Verify Controllers

1. Launch the real robot:
   ```bash
   ros2 launch rover2_bringup robot.launch.py
   ```
2. Verify controller_manager loads the hardware interface:
   ```bash
   ros2 control list_hardware_interfaces
   ros2 control list_controllers
   ```
3. Confirm `diff_drive_controller` is in `active` state and `joint_state_broadcaster` is running

#### 4b: Stationary Checks (robot on blocks)

**Before proceeding:** Confirm battery voltage is above 13.0V. Keep your hand on the power switch.

Test with wheels off the ground before driving on the floor:

1. Send a low-speed forward command:
   ```bash
   ros2 topic pub --once /diff_drive_controller/cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}}"
   ```
2. Verify:
   - [ ] Both wheels spin forward
   - [ ] `/diff_drive_controller/odom` position increases in X
   - [ ] `/joint_states` shows both wheel positions changing
3. Send a rotation command:
   ```bash
   ros2 topic pub --once /diff_drive_controller/cmd_vel geometry_msgs/Twist "{angular: {z: 0.5}}"
   ```
4. Verify:
   - [ ] Wheels spin in opposite directions
   - [ ] `/diff_drive_controller/odom` yaw changes

#### 4c: First Drive (on the ground)

1. Confirm battery voltage is above 13.0V
2. Place robot on flat floor with plenty of space to move, away from edges/stairs
3. Keep your hand near the power switch for the first few commands
4. Drive with teleop at low speed:
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel
   ```
5. Verify:
   - [ ] Robot drives forward when expected
   - [ ] Robot turns left/right correctly
   - [ ] Robot stops when key is released (within `cmd_vel_timeout`)
   - [ ] No erratic behavior or unexpected jerking

**Deliverables:**
- [ ] Controllers load and activate without errors
- [ ] Wheels respond correctly to `/diff_drive_controller/cmd_vel` (verified on blocks first)
- [ ] Odometry direction matches physical motion
- [ ] Robot drives safely on the ground with teleop

---

### Phase 5: Sensor Integration

**Power: Robot battery via ribbon cable.**

**Goal:** Add IMU, camera, and power monitor as ROS2 nodes and verify each produces correct data.

#### 5a: IMU (MPU6050)

- Clone and build `ros2_mpu6050_driver`:
  ```bash
  git clone https://github.com/hiwad-aziz/ros2_mpu6050_driver src/ros2_mpu6050_driver
  colcon build --symlink-install
  ```
- Publishes: `/imu/data` (sensor_msgs/Imu)

**Physical verification:**
- [ ] `/imu/data` publishes at expected rate (~100 Hz)
- [ ] With robot stationary, linear_acceleration.z ≈ 9.8
- [ ] Tilt robot — acceleration values change in correct axis
- [ ] Rotate robot by hand — angular velocity values respond

#### 5b: Camera

- Use `usb_cam` package (installed by setup script)
- Publishes: `/camera/image_raw`, `/camera/camera_info`

**Physical verification:**
- [ ] Image topic publishes at expected rate
- [ ] View image (e.g., `ros2 run rqt_image_view rqt_image_view` or save a frame) — image is right-side up and not mirrored
- [ ] Camera is pointed in the direction the URDF `camera_link` frame indicates (forward)

#### 5c: Power Monitor

- Already implemented in `rover2_sensors`
- Publishes: `/battery_state` (sensor_msgs/BatteryState)

**Physical verification:**
- [ ] Voltage reading matches multimeter (within 0.2V)
- [ ] Current reading increases when motors are running
- [ ] Low voltage warning triggers at correct threshold

**Deliverables:**
- [ ] All three sensor nodes run simultaneously without errors
- [ ] Data from each sensor is physically verified as correct
- [ ] Sensor data is visible in RViz

---

### Phase 6: Camera Integration

**Power: Robot battery via ribbon cable.**

**Goal:** Get USB camera streaming as a ROS2 topic.

**Tasks:**
1. Connect USB camera to the Pi
2. Verify the camera is detected:
   ```bash
   v4l2-ctl --list-devices
   ls /dev/video*
   ```
3. Launch the camera node:
   ```bash
   ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0
   ```
4. Verify image topic:
   ```bash
   ros2 topic hz /image_raw
   ```
5. Save a test frame to verify orientation:
   ```bash
   ros2 run usb_cam usb_cam_node_exe &
   ros2 topic echo /image_raw --once | head -5  # or save with cv2
   ```

**Deliverables:**
- [ ] Camera detected at `/dev/video*`
- [ ] `/image_raw` publishes at expected rate
- [ ] Image is right-side up and not mirrored

---

### Phase 7: Sensor Fusion (IMU + Odometry EKF)

**Power: Robot battery via ribbon cable.**

**Goal:** Use `robot_localization` EKF to fuse IMU heading with wheel odometry. This corrects the heading drift caused by open-loop motor control and asymmetric wheel speeds.

**Why this is needed:** On-ground testing showed ~10° heading drift over 1m of forward travel. Without heading correction, odometry calibration is unreliable and the robot cannot drive straight.

**Tasks:**
1. Install robot_localization:
   ```bash
   sudo apt install -y ros-jazzy-robot-localization
   ```
2. Create EKF config file (`rover2_bringup/config/ekf.yaml`):
   - Fuse `/diff_drive_controller/odom` (x, y velocity from wheels)
   - Fuse `/imu` (yaw angle/velocity from IMU)
   - Output: `/odometry/filtered` with corrected heading
3. Add EKF node to `robot.launch.py`
4. Verify filtered odometry corrects heading drift:
   - Drive forward 1m — heading should stay near 0°
   - Compare `/diff_drive_controller/odom` (drifts) vs `/odometry/filtered` (corrected)

**Deliverables:**
- [ ] EKF node running and publishing `/odometry/filtered`
- [ ] Heading drift significantly reduced when driving straight
- [ ] TF tree: `odom → base_link` now comes from EKF, not raw wheel odometry

---

### Phase 8: Calibration & Validation

**Power: Robot battery via ribbon cable.**

**Goal:** With sensor fusion correcting heading, tune odometry parameters and run integration tests.

#### 8a: Linear Calibration

1. Mark a 1m line on the floor
2. Drive forward using the teleop script until odometry reports 1m
3. Measure actual distance with tape measure
4. Adjust: `wheel_radius = wheel_radius * (odometry_distance / actual_distance)`
5. Rebuild, repeat until error < 2%

- [ ] Robot reports 1m, actually travels 1m (±2 cm)

#### 8b: Rotational Calibration

1. Mark robot's heading on the floor
2. Command a 360° rotation using teleop
3. Measure actual rotation
4. Adjust: `wheel_separation = wheel_separation * (actual_angle / commanded_angle)`
5. Rebuild, repeat until error < 5°

- [ ] Robot reports 360°, actually rotates 360° (±5°)

#### 8c: Integration Tests

| Test | Procedure | Pass Criteria |
|------|-----------|---------------|
| Drive straight 2m | Teleop forward, stop at 2m odometry | Lateral drift < 5 cm |
| Drive square (1m sides) | Four 1m straights with 90° turns | Returns within 10 cm of start |
| Emergency stop | Drive forward, release keys | Stops within `cmd_vel_timeout` (0.5s) |
| IMU + odometry agreement | Drive forward, check filtered odom heading stays ~0° | Heading drift < 2° per meter |

**Deliverables:**
- [ ] Linear odometry accurate to ±2%
- [ ] Rotational odometry accurate to ±5°
- [ ] Square test returns within 10 cm of start
- [ ] Emergency stop works reliably
- [ ] Heading drift corrected by sensor fusion

---

## Package Structure

```
rover-1/
├── src/
│   ├── rover2_base/                 # C++ hardware interface (ros2_control)
│   │   ├── include/rover2_base/
│   │   │   ├── rover2_hardware.hpp
│   │   │   ├── gpio_motor.hpp
│   │   │   ├── gpio_encoder.hpp
│   │   │   └── visibility_control.h
│   │   ├── src/
│   │   │   ├── rover2_hardware.cpp
│   │   │   ├── gpio_motor.cpp
│   │   │   └── gpio_encoder.cpp
│   │   ├── config/
│   │   │   └── rover2_controllers.yaml
│   │   └── rover2_base.xml
│   │
│   ├── rover2_description/          # URDF and visualization
│   │   ├── urdf/
│   │   │   ├── rover2.urdf.xacro             # Base robot geometry
│   │   │   ├── rover2_hardware.urdf.xacro    # Hardware variant (real GPIO)
│   │   │   ├── rover2_mock.urdf.xacro        # Mock variant (no hardware)
│   │   │   ├── rover2.ros2_control.xacro     # Real hardware ros2_control config
│   │   │   ├── rover2_mock.ros2_control.xacro # Mock hardware ros2_control config
│   │   │   ├── materials.xacro
│   │   │   └── inertial_macros.xacro
│   │   ├── rviz/rover2.rviz
│   │   └── launch/view_robot.launch.py
│   │
│   ├── rover2_bringup/              # Launch files and configs
│   │   ├── launch/
│   │   │   ├── robot.launch.py      # Real hardware launch
│   │   │   ├── simulation.launch.py # Mock hardware launch
│   │   │   └── teleop.launch.py
│   │   └── config/
│   │       └── teleop.yaml
│   │
│   └── rover2_sensors/              # Python sensor nodes
│       ├── rover2_sensors/
│       │   ├── __init__.py
│       │   └── ina219_node.py
│       ├── config/sensors.yaml
│       ├── launch/sensors.launch.py
│       └── setup.py
│
├── scripts/
│   ├── setup_pi5.sh
│   └── hardware_tests/
│       ├── README.md
│       ├── test_motors.py
│       ├── test_encoders.py
│       ├── test_i2c.py
│       ├── test_imu.py
│       └── test_power.py
│
└── plan.md
```

---

## Key Configuration

**diff_drive_controller** (`rover2_controllers.yaml`):
```yaml
diff_drive_controller:
  ros__parameters:
    left_wheel_names: ["left_wheel_joint"]
    right_wheel_names: ["right_wheel_joint"]
    wheel_separation: 0.356
    wheel_radius: 0.06065
    publish_rate: 50.0
    enable_odom_tf: true
    linear.x.max_velocity: 0.76
    angular.z.max_velocity: 2.0
```

---

## ROS2 Topics

| Topic | Type | Source |
|-------|------|--------|
| `/diff_drive_controller/cmd_vel` | geometry_msgs/Twist | Teleop / nav2 |
| `/diff_drive_controller/odom` | nav_msgs/Odometry | diff_drive_controller |
| `/imu/data` | sensor_msgs/Imu | MPU6050 driver |
| `/camera/image_raw` | sensor_msgs/Image | usb_cam |
| `/camera/camera_info` | sensor_msgs/CameraInfo | usb_cam |
| `/joint_states` | sensor_msgs/JointState | joint_state_broadcaster |
| `/battery_state` | sensor_msgs/BatteryState | ina219_node |
| `/tf` | tf2_msgs/TFMessage | robot_state_publisher |

---

## Dependencies

### Pi 5 Setup (handled by `scripts/setup_pi5.sh`)

**OS:** Ubuntu 24.04 (Noble)

**ROS2 Jazzy:**
```bash
ros-jazzy-ros-base
ros-jazzy-ros2-control
ros-jazzy-ros2-controllers
ros-jazzy-xacro
ros-jazzy-robot-state-publisher
ros-jazzy-joint-state-publisher
ros-jazzy-usb-cam
ros-jazzy-teleop-twist-keyboard
python3-colcon-common-extensions
python3-rosdep
```

**Optional (for RViz visualization):**
```bash
ros-jazzy-rviz2
ros-jazzy-joint-state-publisher-gui
```

**Hardware:**
```bash
python3-lgpio
python3-smbus2
i2c-tools
v4l-utils
```

**From Source:**
```bash
# MPU6050 driver
git clone https://github.com/hiwad-aziz/ros2_mpu6050_driver src/ros2_mpu6050_driver
```

---

## Pi 5 GPIO Notes

- **Use `lgpio`** — `pigpio` is not compatible with Pi 5
- Hardware PWM available on GPIO 12, 13, 18, 19
- GPIO access requires `gpio` group membership:
  ```bash
  sudo usermod -aG gpio $USER
  ```
- The hardware interface has a mock mode (`HAS_LGPIO` compile flag) for testing without GPIO

---

## Quick Start

```bash
# 1. Setup (one-time, USB-C power)
./scripts/setup_pi5.sh
sudo reboot

# 2. Build
cd ~/github/rover-1
colcon build --symlink-install
source install/setup.bash

# 3. Test with mock hardware (USB-C power, no ribbon cable)
ros2 launch rover2_bringup simulation.launch.py

# 4. Teleop (another terminal)
source ~/github/rover-1/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel

# 5. Switch to battery power (shutdown, unplug USB-C, connect ribbon cable, power on)

# 6. Hardware tests
python3 scripts/hardware_tests/test_power.py    # Check voltage first!
python3 scripts/hardware_tests/test_i2c.py
python3 scripts/hardware_tests/test_motors.py
python3 scripts/hardware_tests/test_encoders.py

# 7. Real hardware
ros2 launch rover2_bringup robot.launch.py
```

---

## Future Enhancements

1. **SLAM**: Add `slam_toolbox` with RPLiDAR
2. **Navigation**: Integrate `nav2` stack
3. **Computer Vision**: Add object detection nodes
4. **Web Interface**: Create web dashboard for monitoring

---

## Sources

- [Viam Rover 2 Docs](https://docs.viam.com/dev/reference/try-viam/rover-resources/rover-tutorial/)
- [Viam Rover 2 GitHub](https://github.com/viamrobotics/Viam-Rover-2)
- [ros2_control Jazzy Docs](https://control.ros.org/jazzy/)
- [diff_drive_controller](https://control.ros.org/jazzy/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html)
- [ros2_mpu6050_driver](https://github.com/hiwad-aziz/ros2_mpu6050_driver)
- [lgpio Library (Pi 5)](https://abyz.me.uk/lg/py_lgpio.html)
