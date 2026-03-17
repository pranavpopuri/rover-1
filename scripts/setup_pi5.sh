#!/bin/bash
#
# Rover2 Pi 5 Setup Script
# Installs ROS2 Humble and all dependencies for Viam Rover 2
#
# Usage:
#   chmod +x setup_pi5.sh
#   ./setup_pi5.sh
#
# Requirements:
#   - Raspberry Pi 5
#   - Ubuntu 22.04 (Jammy)
#   - Internet connection
#   - ~30 minutes for full installation
#

set -e

echo "============================================================"
echo "ROVER2 RASPBERRY PI 5 SETUP"
echo "============================================================"
echo ""

# Check if running on Pi
if [ ! -f /proc/device-tree/model ]; then
    echo "Warning: Not running on a Raspberry Pi"
    read -p "Continue anyway? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
else
    echo "Device: $(cat /proc/device-tree/model)"
fi

# Check Ubuntu version
if [ -f /etc/os-release ]; then
    . /etc/os-release
    echo "OS: $PRETTY_NAME"
    if [ "$VERSION_CODENAME" != "jammy" ]; then
        echo "Warning: Expected Ubuntu 22.04 (jammy), got $VERSION_CODENAME"
        read -p "Continue anyway? (y/N) " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi
fi

echo ""
echo "This script will install:"
echo "  - ROS2 Humble (ros-humble-ros-base)"
echo "  - ros2_control and ros2_controllers"
echo "  - Hardware dependencies (lgpio, smbus2, i2c-tools)"
echo "  - USB camera support"
echo ""
read -p "Press ENTER to continue (Ctrl+C to cancel)..."

# ============================================================
# STEP 1: System Update
# ============================================================
echo ""
echo "============================================================"
echo "STEP 1/6: Updating system packages..."
echo "============================================================"

sudo apt update
sudo apt upgrade -y

# ============================================================
# STEP 2: Enable I2C
# ============================================================
echo ""
echo "============================================================"
echo "STEP 2/6: Enabling I2C..."
echo "============================================================"

# Check if I2C is already enabled
if [ -e /dev/i2c-1 ]; then
    echo "I2C already enabled"
else
    echo "Enabling I2C via raspi-config..."
    sudo raspi-config nonint do_i2c 0
    echo "I2C enabled (may require reboot)"
fi

# ============================================================
# STEP 3: Install ROS2 Humble
# ============================================================
echo ""
echo "============================================================"
echo "STEP 3/6: Installing ROS2 Humble..."
echo "============================================================"

# Check if ROS2 is already installed
if [ -d /opt/ros/humble ]; then
    echo "ROS2 Humble already installed"
else
    # Install prerequisites
    sudo apt install -y software-properties-common curl

    # Add ROS2 repository
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
        sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    # Update and install ROS2
    sudo apt update
    sudo apt install -y ros-humble-ros-base
fi

# ============================================================
# STEP 4: Install ROS2 Packages
# ============================================================
echo ""
echo "============================================================"
echo "STEP 4/6: Installing ROS2 packages..."
echo "============================================================"

sudo apt install -y \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-usb-cam \
    ros-humble-teleop-twist-keyboard \
    python3-colcon-common-extensions \
    python3-rosdep

# Initialize rosdep if needed
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init || true
fi
rosdep update || true

# ============================================================
# STEP 5: Install Hardware Dependencies
# ============================================================
echo ""
echo "============================================================"
echo "STEP 5/6: Installing hardware dependencies..."
echo "============================================================"

sudo apt install -y \
    python3-lgpio \
    python3-smbus2 \
    i2c-tools \
    v4l-utils

# ============================================================
# STEP 6: Configure User Permissions
# ============================================================
echo ""
echo "============================================================"
echo "STEP 6/6: Configuring user permissions..."
echo "============================================================"

# Add user to required groups
sudo usermod -aG gpio $USER 2>/dev/null || true
sudo usermod -aG i2c $USER 2>/dev/null || true
sudo usermod -aG video $USER 2>/dev/null || true
sudo usermod -aG dialout $USER 2>/dev/null || true

# ============================================================
# STEP 7: Setup Shell Environment
# ============================================================
echo ""
echo "============================================================"
echo "Setting up shell environment..."
echo "============================================================"

# Add ROS2 sourcing to bashrc if not already there
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# ROS2 Humble" >> ~/.bashrc
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    echo "Added ROS2 sourcing to ~/.bashrc"
else
    echo "ROS2 sourcing already in ~/.bashrc"
fi

# ============================================================
# DONE
# ============================================================
echo ""
echo "============================================================"
echo "SETUP COMPLETE!"
echo "============================================================"
echo ""
echo "Next steps:"
echo ""
echo "1. REBOOT to apply all changes:"
echo "   sudo reboot"
echo ""
echo "2. After reboot, verify I2C devices:"
echo "   i2cdetect -y 1"
echo "   (should show 0x40 and 0x68)"
echo ""
echo "3. Run hardware tests:"
echo "   cd ~/rover-1/scripts/hardware_tests"
echo "   python3 test_i2c.py"
echo "   python3 test_imu.py"
echo "   python3 test_motors.py"
echo ""
echo "4. Build the ROS2 workspace:"
echo "   cd ~/rover-1"
echo "   colcon build --symlink-install"
echo "   source install/setup.bash"
echo ""
echo "5. Launch the robot:"
echo "   ros2 launch rover2_bringup robot.launch.py"
echo ""
echo "============================================================"
