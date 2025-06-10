#!/bin/bash

# Exit on error
set -e

# Define workspace and source directories
WORKSPACE_DIR="$HOME/tello_ws"
SRC_DIR="$WORKSPACE_DIR/src"

# Create workspace and source directories
echo "Creating ROS 2 workspace at $WORKSPACE_DIR"
mkdir -p "$SRC_DIR"

# Navigate to source directory
cd "$SRC_DIR"

# Initialize ROS 2 packages
declare -a packages=("tello_controller_node" "yolov11_detector_node" "apriltag_detector_node" "navigation_node" "mission_manager_node")

for pkg in "${packages[@]}"; do
  echo "Creating package: $pkg"
  ros2 pkg create --build-type ament_cmake "$pkg" --dependencies rclcpp std_msgs geometry_msgs sensor_msgs
done

# Create basic README for workspace
cat << EOF > "$WORKSPACE_DIR/README.md"
# Tello Drone ROS 2 Workspace

This workspace contains ROS 2 nodes for controlling a Tello drone and performing various tasks.

## Structure
- **tello_controller_node**: Controls Tello drone flight.
- **yolov11_detector_node**: Performs object detection using YOLOv11.
- **apriltag_detector_node**: Detects and returns AprilTag poses.
- **navigation_node**: Handles path planning and execution.
- **mission_manager_node**: Manages the main control flow logic.

## Setup Instructions
1. Source ROS 2 Foxy: \`source /opt/ros/foxy/setup.bash\`
2. Build the workspace: \`cd $WORKSPACE_DIR && colcon build\`
3. Source the workspace: \`source $WORKSPACE_DIR/install/setup.bash\`

## Dependencies
- ROS 2 Foxy
- Additional dependencies (e.g., YOLOv11, AprilTag libraries) to be installed separately.
EOF

# Build the workspace
cd "$WORKSPACE_DIR"
colcon build

# Add workspace sourcing to .bashrc if not already present
if ! grep -Fx "source $WORKSPACE_DIR/install/setup.bash" ~/.bashrc; then
  echo "source $WORKSPACE_DIR/install/setup.bash" >> ~/.bashrc
  echo "Added workspace sourcing to ~/.bashrc"
fi

echo "ROS 2 workspace setup complete at $WORKSPACE_DIR"
echo "To use the workspace, source it with: source $WORKSPACE_DIR/install/setup.bash"
