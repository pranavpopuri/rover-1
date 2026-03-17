#!/bin/bash
# Helper script to run Rover2 simulation in Docker
# Usage: ./run_simulation.sh

set -e

echo "=== Rover2 Simulation Launcher ==="

# Check Docker
if ! docker info >/dev/null 2>&1; then
    echo "Error: Docker is not running. Start Docker Desktop first."
    exit 1
fi

# Enable X11 forwarding
xhost +localhost 2>/dev/null || true

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "Starting Docker container..."
echo "Once inside, run:"
echo "  colcon build --symlink-install"
echo "  source install/setup.bash"
echo "  ros2 launch rover2_bringup simulation.launch.py"
echo ""
echo "To control the robot, open another terminal and run:"
echo "  docker exec -it rover2_sim bash"
echo "  source /ros2_ws/install/setup.bash"
echo "  ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo ""
echo "Press Ctrl+D or type 'exit' to quit the container."
echo "=================================================="
echo ""

docker run -it --rm \
    --name rover2_sim \
    -e DISPLAY=host.docker.internal:0 \
    -v "$SCRIPT_DIR":/ros2_ws \
    -w /ros2_ws \
    osrf/ros:humble-desktop \
    bash
