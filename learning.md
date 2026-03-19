# Learning Guide: Building a ROS2 Robot from Scratch

This document teaches how this repository was built, step by step, decision by decision. It's written for someone who has never worked with ROS2, robotics, or embedded systems. By the end, you'll understand not just *what* was done, but *why* every choice was made.

---

## Table of Contents

1. [The Starting Point](#1-the-starting-point)
2. [Understanding the Hardware](#2-understanding-the-hardware)
3. [Choosing the Software Stack](#3-choosing-the-software-stack)
4. [Repository Structure: Why So Many Packages?](#4-repository-structure-why-so-many-packages)
5. [Phase 1: Describing the Robot (URDF)](#5-phase-1-describing-the-robot-urdf)
6. [Phase 2: Controlling Motors (ros2_control)](#6-phase-2-controlling-motors-ros2_control)
7. [Phase 3: Testing Without Hardware (Mock Mode)](#7-phase-3-testing-without-hardware-mock-mode)
8. [Phase 4: Real Hardware Bringup](#8-phase-4-real-hardware-bringup)
9. [Phase 5: Sensors (IMU and Power Monitor)](#9-phase-5-sensors-imu-and-power-monitor)
10. [Phase 6: Sensor Fusion (EKF)](#10-phase-6-sensor-fusion-ekf)
11. [Phase 7: Vision and Autonomous Navigation](#11-phase-7-vision-and-autonomous-navigation)
12. [Every Bug and What It Taught Us](#12-every-bug-and-what-it-taught-us)
13. [Key Decisions Summary](#13-key-decisions-summary)
14. [Glossary](#14-glossary)

---

## 1. The Starting Point

### What we had

A **Viam Rover 2** — a small differential-drive robot originally designed to run on Viam's proprietary middleware. The robot was functional but locked into Viam's ecosystem. The goal was to migrate it to **ROS2**, the open-source industry standard for robotics.

### Why migrate?

Viam is a hosted service — your robot talks to their cloud. ROS2 is open, runs locally, has thousands of community packages, and is what the robotics industry uses. If you want to add SLAM, navigation, manipulation, or anything advanced, ROS2 has packages for it. Viam doesn't.

### The computer

Everything runs on a **Raspberry Pi 5 (8GB RAM)** mounted directly on the robot. No separate laptop needed for computation. Ubuntu 24.04 (Noble) is the operating system — this matters because ROS2 versions are tied to specific Ubuntu versions.

---

## 2. Understanding the Hardware

Before writing any code, you need to understand every piece of hardware on the robot and how it communicates with the Pi.

### The motors (DC motors with H-Bridge)

The rover has **two DC motors**, one per side. This is called **differential drive** — to go forward, both wheels spin forward. To turn left, the right wheel spins faster than the left (or the left stops/reverses).

DC motors are controlled by **PWM (Pulse Width Modulation)** — you rapidly switch the power on and off. The percentage of time the power is "on" (the **duty cycle**) controls the speed. 50% duty = roughly 50% speed.

Each motor connects to the Pi through an **H-Bridge** — a circuit that lets you control both speed and direction. The H-Bridge has three inputs per motor:
- **PWM pin**: Controls speed (0-100% duty cycle)
- **A pin**: Direction control 1
- **B pin**: Direction control 2

The direction logic:
| A | B | Result |
|---|---|--------|
| 1 | 0 | Forward |
| 0 | 1 | Backward |
| 0 | 0 | Coast (free-spin) |
| 1 | 1 | Brake (active stop) |

**Decision**: We use **1000 Hz PWM frequency**. Too low and you hear audible whining. Too high and the H-Bridge can't keep up. 1000 Hz is the sweet spot.

### The encoders (measuring wheel rotation)

Each wheel has an **encoder** — a sensor that generates electrical pulses as the wheel rotates. The Viam Rover 2 encoders produce **1992 pulses per full rotation**. By counting pulses over time, you can calculate:
- **Position**: How far the wheel has turned (in radians)
- **Velocity**: How fast it's turning (in radians/second)

**Critical discovery**: These encoders are **powered by the motor driver**, not independently. This means they only generate signals when the motors are receiving power. You cannot manually spin the wheel and see encoder output. This was discovered during testing and fundamentally blocked closed-loop control (more on this in Phase 4).

### The IMU (Inertial Measurement Unit — MPU6050)

The **MPU6050** is a chip that contains:
- **Accelerometer**: Measures acceleration in X, Y, Z (including gravity)
- **Gyroscope**: Measures rotational velocity around X, Y, Z

For a ground robot, the most useful reading is **gyro Z** — the rate of rotation around the vertical axis (yaw). This tells you how fast the robot is turning, which helps correct heading drift from wheel slippage.

The MPU6050 communicates over **I2C** (Inter-Integrated Circuit) — a two-wire protocol at address `0x68`.

### The power monitor (INA219)

The **INA219** measures the robot's battery voltage and current draw. It also communicates over I2C at address `0x40`. This lets us:
- Monitor battery level
- Detect low battery conditions
- Estimate remaining runtime

### The camera (USB webcam)

A standard USB webcam mounted on the front of the robot. Used for object detection. Communicates over USB as a V4L2 (Video4Linux2) device at `/dev/video0`.

### GPIO pin mapping

**GPIO** (General Purpose Input/Output) pins are the Pi's way of talking to hardware. Each pin has a **BCM number** (Broadcom's numbering scheme). The specific assignments:

```
Left Motor:   PWM=26, A=19, B=13
Right Motor:  PWM=22, A=6,  B=5
Left Encoder:  GPIO 20
Right Encoder: GPIO 21
```

**Decision**: We use **BCM numbering** everywhere, not physical pin numbers. BCM is what the software libraries expect and what's printed on pinout diagrams.

---

## 3. Choosing the Software Stack

### Why ROS2 Jazzy?

ROS2 has multiple **distributions** (like Ubuntu versions). Each distribution targets a specific Ubuntu version:

| ROS2 Distribution | Ubuntu Version |
|-------------------|---------------|
| Humble (2022) | Ubuntu 22.04 |
| Iron (2023) | Ubuntu 23.04 |
| **Jazzy (2024)** | **Ubuntu 24.04** |

Since the Pi runs **Ubuntu 24.04**, we must use **Jazzy**. This is not a choice — ROS2 distributions are tightly coupled to OS versions.

**Important Jazzy quirk**: The `diff_drive_controller` in Jazzy requires **`TwistStamped`** messages (velocity commands with timestamps), not plain `Twist`. This is different from older ROS2 versions. If you send plain `Twist`, the controller silently ignores it — no error, no warning, just nothing happens. This was one of the hardest bugs to find.

### Why ros2_control?

**ros2_control** is a framework for hardware abstraction. Instead of writing motor commands directly, you:

1. Write a **hardware interface** (a C++ plugin that talks to your specific motors/encoders)
2. Use standard **controllers** (like `diff_drive_controller`) that convert velocity commands into wheel speeds
3. Everything is connected through standardized interfaces

The benefit: if you change your motors, you only change the hardware interface. The controllers, navigation stack, and everything else stays the same.

**Decision**: We use `diff_drive_controller` (not a custom controller) because differential drive is the most common robot type and the standard controller handles all the kinematics (converting linear/angular velocity to left/right wheel speeds).

### Why lgpio (not pigpio)?

The Pi 5 uses a **completely different GPIO architecture** than Pi 4. The popular `pigpio` library does not work on Pi 5. The replacement is **lgpio** (Linux GPIO), which uses the kernel's GPIO character device interface (`/dev/gpiochipN`).

**Critical discovery**: On Pi 5, the GPIO chip for user-accessible pins is `gpiochip4` (not `gpiochip0` like on Pi 4). Our code tries `gpiochip4` first, then falls back to `gpiochip0`.

**Another critical discovery**: lgpio's **callback functions don't work on Pi 5**. Callbacks are a way to be notified when a GPIO pin changes state. Since encoders generate thousands of pulses per second, callbacks would be ideal. But since they don't work, we use a **polling thread** that checks the pin state 20,000 times per second (every 50 microseconds). This is less elegant but works reliably.

### Why not Gazebo?

**Gazebo** is the standard physics simulator for ROS2. It lets you test your robot in a virtual world before deploying to hardware. However:

1. **No GPU on Pi 5**: Gazebo needs a GPU for rendering. The Pi 5 has no dedicated GPU.
2. **No arm64 packages**: Gazebo Classic doesn't have arm64 (Pi's architecture) packages. Gazebo Harmonic crashes on Pi 5.
3. **No X display over SSH**: Even if it ran, you'd need a display server.

**Decision**: Instead of Gazebo, we use **ros2_control's mock hardware mode**. This is a feature where the controller framework simulates motor behavior — commands sent to the motors are immediately reflected back as encoder readings. It's not physics simulation, but it lets us test the full software stack without real hardware.

---

## 4. Repository Structure: Why So Many Packages?

ROS2 projects are organized into **packages** — self-contained units of code. Each package has a clear responsibility. This is the ROS2 convention:

```
src/
  rover2_description/   ← What the robot looks like (URDF model)
  rover2_base/          ← How to talk to the hardware (motors, encoders)
  rover2_bringup/       ← How to start everything (launch files)
  rover2_sensors/       ← Sensor drivers (power monitor)
  rover2_vision/        ← Camera and object detection
  ros2_mpu6050_driver/  ← IMU driver (external, git submodule)
```

### Why separate packages?

1. **rover2_description** can be used on any computer (for visualization) — it has no hardware dependencies
2. **rover2_base** only compiles on the Pi (needs lgpio) — but in mock mode, it works anywhere
3. **rover2_bringup** ties everything together — it depends on all other packages
4. **rover2_sensors** is Python-only — different build system than C++ packages
5. **rover2_vision** is also Python-only — can be developed independently

### C++ vs Python packages

ROS2 supports two build systems:
- **ament_cmake** (C++): Used for performance-critical code like hardware interfaces. Compiled.
- **ament_python** (Python): Used for higher-level logic like sensor processing. Interpreted.

`rover2_base` is C++ because the hardware interface runs at 50 Hz and needs to be fast. `rover2_sensors` and `rover2_vision` are Python because they're simpler and speed isn't critical.

### The build system

```bash
colcon build                          # Build everything
colcon build --packages-select pkg    # Build one package
source install/setup.bash             # Load built packages into your shell
```

**`colcon`** is ROS2's build tool. It finds all packages in `src/`, determines dependencies, and builds them in the right order. After building, you must `source install/setup.bash` to make the new code available.

---

## 5. Phase 1: Describing the Robot (URDF)

### What is URDF?

**URDF (Unified Robot Description Format)** is an XML format that describes your robot's physical structure — every link (rigid body), every joint (connection between links), their dimensions, masses, and visual appearance.

ROS2 tools use the URDF to:
- Visualize the robot in RViz
- Compute transformations between frames (e.g., "where is the camera relative to the wheels?")
- Configure controllers (wheel radius, wheel separation)

### Our robot's frame hierarchy

```
base_footprint          ← On the ground, directly below the wheel axis
    └── base_link       ← At the wheel axis center (main reference frame)
        ├── chassis     ← The robot body (offset forward and up from base_link)
        │   ├── imu_link    ← Where the IMU is mounted
        │   ├── camera_link ← Where the camera is mounted
        │   └── caster_wheel ← The front caster (passive wheel)
        ├── left_wheel  ← Continuous joint (rotates forever)
        └── right_wheel ← Continuous joint
```

**Decision**: `base_link` is at the wheel axis center, not the chassis center. This is the ROS convention for differential drive robots — it makes the kinematics simpler because wheel rotations directly translate to base_link motion.

### Why Xacro instead of raw URDF?

**Xacro** is a macro language for URDF. It adds:
- **Properties** (variables): Define wheel radius once, use it everywhere
- **Macros**: Define a wheel template, instantiate it for left and right
- **Includes**: Split the robot into multiple files
- **Conditionals**: Different configs for real vs. mock hardware

For example, instead of duplicating the wheel definition:
```xml
<!-- Xacro macro — define once -->
<xacro:macro name="wheel" params="prefix side_sign">
  <link name="${prefix}_wheel"> ... </link>
  <joint name="${prefix}_wheel_joint">
    <origin xyz="0 ${side_sign * wheel_separation/2} 0"/>
  </joint>
</xacro:macro>

<!-- Use twice -->
<xacro:wheel prefix="left" side_sign="1"/>
<xacro:wheel prefix="right" side_sign="-1"/>
```

### The two URDF variants

We maintain **two versions** of the robot description:
1. **`rover2_hardware.urdf.xacro`** — References the real hardware plugin (`rover2_base/Rover2Hardware`)
2. **`rover2_mock.urdf.xacro`** — References mock hardware (`mock_components/GenericSystem`)

Both include the same base `rover2.urdf.xacro` (same physical dimensions) but differ in their `<ros2_control>` section. This lets us test the full stack without real motors.

### Inertia: why does it matter?

Every link in the URDF needs an **inertia tensor** — a 3x3 matrix describing how the mass is distributed. Physics simulators and some controllers use this. Getting it wrong doesn't break anything on real hardware, but it would make simulated behavior unrealistic.

We use simplified shapes (boxes and cylinders) to calculate inertia using standard formulas defined in `inertial_macros.xacro`.

---

## 6. Phase 2: Controlling Motors (ros2_control)

### The architecture

```
Your code (teleop/navigation)
    │
    │ publishes TwistStamped to /diff_drive_controller/cmd_vel
    ▼
diff_drive_controller
    │
    │ converts (linear_x, angular_z) to (left_wheel_vel, right_wheel_vel)
    │ using: left  = (linear - angular * separation/2) / radius
    │        right = (linear + angular * separation/2) / radius
    ▼
Hardware Interface (rover2_hardware.cpp)
    │
    │ write(): sends velocity commands to GPIO pins
    │ read():  reads encoder ticks from GPIO pins
    ▼
Physical motors and encoders
```

### The hardware interface plugin

`rover2_hardware.cpp` implements `hardware_interface::SystemInterface` — a standard ROS2 interface with lifecycle methods:

1. **`on_init()`** — Parse parameters from URDF (pin numbers, wheel specs)
2. **`on_configure()`** — Open GPIO chip, create motor and encoder objects
3. **`on_activate()`** — Reset encoders, ready to run
4. **`read()`** — Called every cycle (50 Hz): read encoder positions and velocities
5. **`write()`** — Called every cycle: send motor commands to GPIO
6. **`on_deactivate()`** — Stop motors
7. **`on_cleanup()`** — Release GPIO resources

**Decision**: The hardware interface is a **pluginlib plugin**, not a standalone node. This means it runs inside the `controller_manager` process. It's loaded dynamically based on the URDF's `<ros2_control>` section. This is the ros2_control convention — it avoids inter-process communication overhead for the tight control loop.

### The plugin descriptor

`rover2_base.xml` tells pluginlib how to find our hardware interface:
```xml
<class name="rover2_base/Rover2Hardware"
       type="rover2_base::Rover2Hardware"
       base_class_type="hardware_interface::SystemInterface">
</class>
```

This file is referenced in `CMakeLists.txt` via `pluginlib_export_plugin_description_file()`.

### Open-loop vs closed-loop control

**Closed-loop**: The controller reads encoder feedback and adjusts motor commands to match the desired velocity. If the left wheel is going too slow, increase its PWM.

**Open-loop**: The controller sends motor commands based on the desired velocity but doesn't correct using feedback. If the left wheel is slower due to friction, it stays slower.

**Decision**: We use **open-loop** (`open_loop: true` in the controller config). Here's why:

We attempted closed-loop control but hit a chicken-and-egg problem:
1. The encoders only generate signals when the motors are powered
2. Closed-loop control won't power the motors until it sees encoder feedback
3. The robot never moves

Additionally, the encoder polling (at 20 kHz) produces noisy velocity readings. A PI controller amplified this noise, causing the motors to oscillate wildly. Open-loop is stable and good enough for basic navigation — SLAM will correct any drift later.

### Controller configuration

`rover2_controllers.yaml` defines two controllers:

```yaml
joint_state_broadcaster:
  type: joint_state_broadcaster/JointStateBroadcaster
  # Publishes wheel positions/velocities to /joint_states

diff_drive_controller:
  type: diff_drive_controller/DiffDriveController
  # Converts velocity commands to wheel commands
  wheel_separation: 0.356      # meters between wheel centers
  wheel_radius: 0.06065        # meters (121.3mm diameter / 2)
  open_loop: true              # don't use encoder feedback for control
  enable_odom_tf: true         # publish odom → base_link transform
  use_stamped_vel: false       # accept TwistStamped input
```

**Why `use_stamped_vel: false`?** Confusingly, this doesn't mean "use unstamped Twist." In Jazzy, the controller always expects `TwistStamped` on the `/diff_drive_controller/cmd_vel` topic. This parameter controls a different internal behavior. The actual requirement for stamped messages was discovered through painful debugging (see [Bugs](#12-every-bug-and-what-it-taught-us)).

---

## 7. Phase 3: Testing Without Hardware (Mock Mode)

### Why mock first?

If your first test is on real hardware and something is wrong, you don't know if it's:
- A wiring problem
- A software configuration problem
- A controller parameter problem
- A URDF problem

Mock mode eliminates hardware from the equation. If it works in mock mode, the software is correct and any problems on real hardware are hardware-related.

### How mock hardware works

The `mock_components/GenericSystem` plugin has `calculate_dynamics: true`, which means:
- Velocity commands sent to the motors are immediately reflected as encoder readings
- The diff_drive_controller sees the expected velocities and computes correct odometry
- From the software's perspective, the robot is moving perfectly

### The simulation launch file

`simulation.launch.py` starts:
1. **robot_state_publisher** — Loads the mock URDF, publishes transforms
2. **controller_manager** — Runs with mock hardware
3. **Spawners** — Start the controllers (joint_state_broadcaster, diff_drive_controller)
4. **RViz** (optional) — 3D visualization

### What we verified in mock mode

- Controllers load and activate correctly
- `ros2 topic pub` can send velocity commands
- Odometry is published on `/diff_drive_controller/odom`
- TF tree is correct (odom → base_link → wheels)
- The teleop script works

---

## 8. Phase 4: Real Hardware Bringup

### The GPIO library (lgpio)

```cpp
// Open the GPIO chip (Pi 5 uses gpiochip4)
int chip = lgGpioClaimOutput(gpio_chip_, 0, pin, 0);

// Set PWM on a pin (1000 Hz, variable duty cycle)
lgTxPwm(gpio_chip_, pwm_pin_, 1000, duty, 0, 0);

// Read a GPIO pin state
int state = lgGpioRead(gpio_chip_, pin_);
```

**`lgGpioClaimOutput`** claims a pin for output. **`lgTxPwm`** starts hardware PWM. **`lgGpioRead`** reads a pin's state (0 or 1).

### The motor driver (gpio_motor.cpp)

```cpp
void GPIOMotor::setVelocity(double velocity, double max_velocity) {
    double fraction = std::abs(velocity) / max_velocity;
    int duty = static_cast<int>(fraction * 100.0);
    duty = std::clamp(duty, 0, 100);

    if (velocity >= 0) {
        lgGpioWrite(gpio_chip_, a_pin_, 1);  // Forward
        lgGpioWrite(gpio_chip_, b_pin_, 0);
    } else {
        lgGpioWrite(gpio_chip_, a_pin_, 0);  // Backward
        lgGpioWrite(gpio_chip_, b_pin_, 1);
    }
    lgTxPwm(gpio_chip_, pwm_pin_, 1000, duty, 0, 0);
}
```

The controller sends a velocity in rad/s. We convert it to a 0-100% duty cycle by dividing by `max_velocity`.

### The encoder reader (gpio_encoder.cpp)

```cpp
void GPIOEncoder::pollLoop() {
    int last_state = lgGpioRead(gpio_chip_, pin_);
    while (running_) {
        int state = lgGpioRead(gpio_chip_, pin_);
        if (state == 1 && last_state == 0) {  // Rising edge
            tick_count_.fetch_add(direction_.load());
        }
        last_state = state;
        std::this_thread::sleep_for(std::chrono::microseconds(50));
    }
}
```

This runs in a dedicated thread, checking the encoder pin every 50 microseconds. When it detects a transition from 0→1 (rising edge), it increments the tick counter. The direction is set by the motor commands — since this is a single-phase encoder, we can't determine direction from the encoder signal alone.

**Why `std::atomic`?** The tick counter is accessed from two threads — the polling thread writes to it, and the ros2_control thread reads from it. `std::atomic` ensures thread-safe access without a mutex (which would be too slow at 20 kHz).

### The conditional compilation pattern

```cmake
find_library(LGPIO_LIBRARY lgpio
    PATHS /usr/lib /usr/lib/aarch64-linux-gnu)

if(LGPIO_LIBRARY)
    target_compile_definitions(rover2_hardware PRIVATE HAS_LGPIO)
    target_link_libraries(rover2_hardware ${LGPIO_LIBRARY})
endif()
```

If lgpio isn't installed (e.g., on a Mac for development), the code compiles without it. All GPIO calls are wrapped in `#ifdef HAS_LGPIO` blocks, so the hardware interface becomes a no-op. This lets you build and test the software architecture on any platform.

### The teleop script

`teleop_stamped.py` is a keyboard teleop that sends **TwistStamped** messages:

```python
msg = TwistStamped()
msg.header.stamp = node.get_clock().now().to_msg()  # Real timestamp!
msg.header.frame_id = 'base_link'
msg.twist.linear.x = linear
msg.twist.angular.z = angular
publisher.publish(msg)
```

**Why not use `teleop_twist_keyboard`?** The standard ROS2 teleop package sends plain `Twist` messages. Jazzy's diff_drive_controller silently ignores these. We wrote a custom teleop that sends `TwistStamped` with real timestamps.

### GPIO busy errors

When a ROS2 process crashes or is killed with SIGKILL, it may not release GPIO pins. The next launch fails with "GPIO busy" errors. The fix:

```python
import lgpio
h = lgpio.gpiochip_open(4)
lgpio.gpiochip_close(h)
```

This opens and closes the GPIO chip, releasing any stale claims.

---

## 9. Phase 5: Sensors (IMU and Power Monitor)

### MPU6050 IMU driver

We use an **external ROS2 package** (`ros2_mpu6050_driver`) as a **git submodule**. A git submodule is a way to include another repository inside yours without copying its code. The submodule is stored at `src/ros2_mpu6050_driver/`.

The driver:
1. Opens I2C bus 1 (`/dev/i2c-1`)
2. Configures the MPU6050 (range, filter settings)
3. Calibrates by averaging readings at startup (assumes the robot is stationary)
4. Publishes `sensor_msgs/Imu` messages with acceleration and angular velocity

**Fix required**: The upstream package was missing `#include <array>` which GCC 13 (used in Ubuntu 24.04) requires but GCC 12 didn't.

### INA219 power monitor

`ina219_node.py` is a custom Python node that reads battery voltage and current:

```python
class INA219Driver:
    def __init__(self, bus=1, address=0x40):
        self.bus = smbus2.SMBus(bus)
        # Configure: 32V range, 320mV shunt, 12-bit ADC
        self.bus.write_word_data(address, 0x00, config_value)
        # Write calibration register
        self.bus.write_word_data(address, 0x05, 4096)
```

The INA219 measures voltage across a **shunt resistor** (a very small known resistance in series with the battery). From Ohm's law: `current = voltage / resistance`. The chip does this calculation internally and provides both bus voltage and current readings.

**Decision**: The node publishes both a `BatteryState` message (standard ROS2 type) and simple Float32 values. The BatteryState includes percentage estimation based on a 4S LiPo discharge curve (12.0V empty → 16.8V full).

### Graceful degradation

Both sensor nodes handle missing hardware gracefully:
- If `/dev/i2c-1` can't be opened, the IMU driver exits with an error message
- If `smbus2` isn't installed, the INA219 node publishes dummy data with a warning

This means you can run the launch file even if sensors aren't connected — the rest of the system still works.

---

## 10. Phase 6: Sensor Fusion (EKF)

### Why fuse sensors?

Each sensor has weaknesses:
- **Wheel odometry** drifts over time — wheels slip, especially when turning
- **IMU** provides rotational velocity but drifts due to bias
- **Neither alone** gives accurate position over long distances

An **Extended Kalman Filter (EKF)** combines multiple sensor readings, weighting each by its expected accuracy. The `robot_localization` package provides a standard EKF for ROS2.

### Our EKF configuration

```yaml
odom0: /diff_drive_controller/odom
odom0_config: [false, false, false,    # x, y, z position — don't fuse
               false, false, false,    # roll, pitch, yaw — don't fuse
               true,  true,  false,    # vx, vy, vz — fuse velocity
               false, false, false,    # roll_rate, pitch_rate, yaw_rate
               false, false, false]    # ax, ay, az

# IMU — DISABLED due to motor vibration
# imu0: /imu
```

**Decision**: We only fuse wheel odometry **velocity** (not position). Why? Position from wheel odometry is an integration of velocity — errors accumulate. The EKF does its own integration with better math. Feeding it raw velocity lets it compute a better position estimate.

### Why the IMU is disabled

During testing, we discovered that **motor vibration corrupts the gyroscope readings**:
- Stationary: gyro Z noise = ~0.65 deg/s (acceptable)
- Driving: gyro Z noise = **~68 deg/s** (100x worse, completely unusable)

The vibration from the motors transfers through the chassis to the IMU. The gyroscope interprets this vibration as rotation, producing enormous noise.

**Fix (not yet implemented)**: Mount the IMU on rubber/foam standoffs to dampen vibration. This is a common problem in robotics and the standard solution is mechanical isolation.

---

## 11. Phase 7: Vision and Autonomous Navigation

### The goal

The robot should:
1. Look for a specific object (e.g., a cup)
2. Drive toward it
3. Stop when it's close enough

### YOLOv8n: why this model?

**YOLO (You Only Look Once)** is a family of object detection models. We use **YOLOv8n** (the "nano" variant) because:
- It's the smallest/fastest YOLOv8 model
- It runs on CPU (the Pi 5 has no GPU)
- It detects 80 object classes from the COCO dataset (person, car, bottle, cup, etc.)
- At 320px input, it achieves **~5 FPS** on the Pi 5

**Benchmark results on Pi 5 CPU:**
| Input size | Inference time | FPS |
|-----------|---------------|-----|
| 160px | 78ms | 12.8 |
| 224px | 103ms | 9.7 |
| **320px** | **186ms** | **5.4** |
| 480px | 367ms | 2.7 |

**Decision**: 320px is the sweet spot — fast enough for real-time tracking, large enough for reliable detection.

### The detection pipeline

```
Camera (15 FPS)
    │
    │  /image_raw (sensor_msgs/Image)
    ▼
detector_node (5 FPS inference)
    │
    │  /detections (std_msgs/String, JSON)
    │  Contains: class_name, confidence, bounding box, center point
    ▼
follower_node (10 Hz control)
    │
    │  /diff_drive_controller/cmd_vel (TwistStamped)
    ▼
Robot moves
```

### The detector node

`detector_node.py` subscribes to camera images and runs YOLO:

```python
results = self.model.predict(frame, conf=0.15, imgsz=320, verbose=False)
for box in results[0].boxes:
    class_name = self.model.names[int(box.cls[0])]
    if class_name not in self.accepted_classes:
        continue
    # Extract bounding box, confidence, center point
```

**Class aliases**: YOLO frequently confuses similar objects. A ceramic mug is often classified as "vase" instead of "cup." We map aliases:
```python
self._class_aliases = {
    'cup': {'cup', 'vase'},
    'bottle': {'bottle', 'vase'},
}
```

**Confidence threshold**: Set to 0.15 (very low). YOLO on 320px input with small objects produces low confidence scores. A threshold of 0.5 (the default) misses most detections. 0.15 catches more true positives at the cost of occasional false positives.

### The follower node

`follower_node.py` implements a **state machine**:

```
SEARCHING ─── target detected ───→ APPROACHING
    ▲                                     │
    │                                     │
    └── target lost (3s timeout) ────────┘
                                          │
                               bbox height ≥ 300px
                                          │
                                          ▼
                                      ARRIVED
```

**Steering (proportional controller)**:
```python
error_x = detection_center_x - (image_width / 2)  # Positive = object is right of center
angular_z = -angular_gain * error_x                 # Negative = turn right
```

If the object is to the right of frame center, `error_x` is positive, so `angular_z` is negative (turn right toward the object). The **gain** (0.003) determines how aggressively the robot steers.

**Distance estimation**: We don't have depth information (no stereo camera or LIDAR). Instead, we use the **bounding box height** as a proxy for distance — a larger bounding box means the object is closer.

```python
approach_ratio = bbox_height / stop_bbox_height  # 0 = far, 1 = close enough
linear_x = max_speed * (1.0 - approach_ratio)    # Slow down as you approach
```

When `bbox_height >= 300px`, the robot considers itself "arrived."

**Spin-out prevention**: At ~5 FPS inference, there's a 200ms gap between detections. If the robot is turning toward an off-center object, it might overshoot and lose the object from frame. To prevent endless spinning:

```python
if det_age > 0.3:  # Detection older than 300ms
    angular_z = 0.0  # Stop turning, just drive straight
```

This means the robot only steers on **fresh** detections. If it hasn't seen the object recently, it drives straight and waits for the next detection.

### The dashboard node

`dashboard_node.py` runs an HTTP server on port 8080, providing a web interface to monitor the robot. It subscribes to `/image_raw`, `/detections`, and `/diff_drive_controller/cmd_vel`, then serves:

- **Camera feed**: JPEG-encoded frames with detection bounding boxes overlaid
- **State display**: Current follower state (SEARCHING/APPROACHING/ARRIVED)
- **Detection info**: Confidence, bounding box size, class name
- **Motor commands**: Linear and angular velocity with visual bars

The page auto-refreshes every 500ms using JavaScript `fetch()` calls.

---

## 12. Every Bug and What It Taught Us

These are listed in the order we encountered them. Each taught something important about the platform.

### 1. No C++ compiler
**Error**: `cmake` couldn't find a C++ compiler
**Fix**: `sudo apt install build-essential`
**Lesson**: Ubuntu Server minimal install doesn't include a compiler. Always install `build-essential` first.

### 2. Empty YAML section crashes controller_manager
**Error**: Controller manager crashes at startup
**Fix**: Removed empty `joint_state_broadcaster.ros__parameters` section from YAML
**Lesson**: ROS2 YAML parsing doesn't handle empty parameter blocks. If a controller has no parameters, don't create the section.

### 3. Controller spawner flags don't work in Jazzy
**Error**: `--controller-type` flag not recognized by spawner
**Fix**: Define controller types in `rover2_controllers.yaml` instead of command-line flags
**Lesson**: Jazzy changed how controller types are specified. YAML-based type definitions are the correct approach.

### 4. Twist messages silently ignored
**Error**: Publishing `Twist` to `/diff_drive_controller/cmd_vel` shows subscriber count = 1, but motors don't move. No error messages anywhere.
**Fix**: Send `TwistStamped` with real timestamps instead of `Twist`
**Lesson**: This was the hardest bug. Jazzy's diff_drive_controller subscribes to both message types but only acts on `TwistStamped`. There's no warning when it receives the wrong type. Always check the controller's documentation for your specific ROS2 version.

### 5. lgpio callbacks don't work on Pi 5
**Error**: Encoder callbacks never fire, tick count stays at zero
**Fix**: Replace callbacks with a polling thread at 20 kHz
**Lesson**: Pi 5's GPIO subsystem is fundamentally different from Pi 4. Don't assume library features work across Pi versions. Test basic GPIO operations before building on top of them.

### 6. Encoders only work when motors are powered
**Error**: Manually rotating wheels produces zero encoder output
**Fix**: N/A — this is a hardware characteristic of the Viam Rover 2
**Lesson**: Always test sensors in the actual operating conditions. Reading the encoder datasheet would not have revealed this — it's specific to how Viam wired the encoders to the motor driver's power supply.

### 7. GPIO busy errors after crash
**Error**: `can not open gpiochip` or `GPIO busy` errors when restarting
**Fix**: `python3 -c "import lgpio; h=lgpio.gpiochip_open(4); lgpio.gpiochip_close(h)"`
**Lesson**: Always handle GPIO cleanup in your deactivation code. When debugging, kill processes with SIGINT (not SIGKILL) to allow cleanup handlers to run.

### 8. Closed-loop control oscillation
**Error**: PI controller makes motors oscillate wildly
**Fix**: Reverted to open-loop control
**Lesson**: Closing a control loop requires clean sensor feedback. Noisy encoder readings (from polling at variable rates) make the derivative term unstable. Hardware filtering (capacitors on encoder lines) or a higher-quality encoder would be needed.

### 9. Motor vibration corrupts IMU
**Error**: Gyro Z reads ~68 deg/s while driving (vs 0.65 stationary)
**Fix**: Disabled IMU in EKF config. Future fix: rubber/foam standoffs.
**Lesson**: Sensor placement matters enormously. An IMU rigidly mounted to a vibrating chassis is useless. This is why commercial robots use vibration-damped IMU mounts.

### 10. Heading drift at low speeds
**Error**: Robot drifts ~30 degrees per meter at 0.2 m/s
**Fix**: Increased speed to 0.5 m/s where drift is ~3 deg/m (acceptable)
**Lesson**: Motor asymmetry (slightly different friction in each gearbox) causes one wheel to spin slightly faster. At low speeds, the difference is proportionally larger. SLAM will correct this in the long term.

### 11. NumPy version conflict
**Error**: `cv_bridge` (compiled with NumPy 1.x) crashes with NumPy 2.x installed by `ultralytics`
**Fix**: `pip3 install "numpy<2" --break-system-packages`
**Lesson**: ROS2 system packages are compiled against specific library versions. Installing pip packages that upgrade system libraries can break ROS2. Always check for conflicts.

### 12. YOLO misclassifies cups as vases
**Error**: A ceramic mug is consistently classified as "vase" with ~20% confidence
**Fix**: Added class alias mapping (cup ↔ vase, bottle ↔ vase)
**Lesson**: YOLO's COCO training data has specific definitions of "cup" (typically a paper/disposable cup). Real-world objects don't always match training data categories. Build flexibility into your detection pipeline.

### 13. Camera crashes on second launch
**Error**: `usb_cam_node_exe` crashes with `terminate called after throwing an instance of 'char*'`
**Fix**: Ensure previous camera process is fully killed before relaunching. Kill stale processes holding `/dev/video0`.
**Lesson**: USB cameras can only be opened by one process at a time. Zombie processes from crashed launches can hold the device open. Always clean up before relaunching.

### 14. Follower spins in circles
**Error**: Robot detects object off-center, starts turning, overshoots, loses object, keeps turning on stale detection
**Fix**: Zero out angular velocity when detection is older than 300ms
**Lesson**: At low inference rates (~5 FPS), the robot moves significantly between detections. Control strategies designed for high-frequency feedback don't work at low frequencies. Always consider the sensor update rate in your control design.

---

## 13. Key Decisions Summary

| Decision | Choice | Why |
|----------|--------|-----|
| OS | Ubuntu 24.04 | Required for ROS2 Jazzy |
| ROS2 version | Jazzy | Matches Ubuntu 24.04 |
| Simulation | Mock hardware (not Gazebo) | No GPU on Pi 5, no arm64 Gazebo packages |
| GPIO library | lgpio (not pigpio) | Pi 5 compatibility |
| Encoder reading | Polling thread (not callbacks) | lgpio callbacks broken on Pi 5 |
| Motor control | Open-loop (not closed-loop) | Encoder noise blocks PID control |
| Command type | TwistStamped (not Twist) | Jazzy diff_drive_controller requirement |
| IMU driver | Git submodule (external package) | Existing driver with minor fix needed |
| IMU fusion | Disabled in EKF | Motor vibration corrupts gyro readings |
| Object detection | YOLOv8n at 320px | Best speed/accuracy tradeoff on Pi 5 CPU |
| Detection confidence | 0.15 (low) | Small objects at 320px produce low confidence |
| Follower control | Proportional with angular decay | Prevents spin-out at low inference FPS |
| Packages | 5 separate packages | Separation of concerns, independent development |
| Language | C++ for hardware, Python for vision/sensors | Performance where needed, simplicity elsewhere |

---

## 14. Glossary

**ament**: ROS2's build system (replaces catkin from ROS1). Comes in two flavors: `ament_cmake` for C++ and `ament_python` for Python.

**BCM**: Broadcom pin numbering scheme used by Raspberry Pi GPIO.

**colcon**: The build tool that orchestrates building multiple ROS2 packages. Reads `package.xml` for dependencies and builds in the correct order.

**COCO**: Common Objects in Context — a dataset of 80 object classes used to train YOLO and other detection models.

**controller_manager**: The ROS2 node that loads hardware interfaces and controllers, running the real-time control loop.

**diff_drive_controller**: A standard ROS2 controller for differential-drive robots. Converts linear/angular velocity commands into individual wheel velocities.

**duty cycle**: The percentage of time a PWM signal is "high." 0% = off, 100% = full power.

**EKF**: Extended Kalman Filter — a mathematical algorithm that combines multiple noisy sensor readings into a single, more accurate estimate of the robot's state.

**GPIO**: General Purpose Input/Output — digital pins on the Raspberry Pi that can be set high/low (output) or read (input).

**H-Bridge**: A circuit that controls the direction and speed of a DC motor using four switches arranged in an "H" pattern.

**I2C**: Inter-Integrated Circuit — a two-wire communication protocol used by sensors like the IMU and power monitor.

**IMU**: Inertial Measurement Unit — a sensor combining accelerometers and gyroscopes to measure motion and orientation.

**lgpio**: Linux GPIO library for Raspberry Pi 5. Replaces the older pigpio library.

**mock hardware**: A ros2_control feature that simulates motor behavior by mirroring commands as state feedback, allowing software testing without physical hardware.

**odometry**: Estimating a robot's position by integrating wheel rotations over time. Subject to drift from wheel slippage.

**open-loop control**: Sending commands without feedback correction. The opposite of closed-loop (PID) control.

**pluginlib**: A ROS2 library for dynamically loading C++ plugins at runtime, used by ros2_control to load hardware interfaces.

**PWM**: Pulse Width Modulation — rapidly switching a signal on/off to control average power delivery.

**ros2_control**: A framework for abstracting robot hardware behind standardized interfaces, allowing controllers to be hardware-agnostic.

**SLAM**: Simultaneous Localization and Mapping — building a map of the environment while tracking the robot's position within it.

**spawner**: A ROS2 utility that loads and activates controllers in the controller_manager.

**TF (Transform)**: ROS2's system for tracking the spatial relationships between coordinate frames (e.g., where is the camera relative to the base?).

**TwistStamped**: A ROS2 message containing linear and angular velocity with a timestamp header. Required by Jazzy's diff_drive_controller.

**URDF**: Unified Robot Description Format — XML format describing a robot's physical structure, joints, sensors, and visual appearance.

**V4L2**: Video4Linux2 — the Linux kernel's interface for video capture devices (cameras).

**Xacro**: XML macro language for URDF files. Adds variables, macros, includes, and conditionals to make robot descriptions reusable.

**YOLO**: You Only Look Once — a family of real-time object detection neural networks.
