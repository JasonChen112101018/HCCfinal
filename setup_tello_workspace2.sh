#!/bin/bash

# Set workspace name
WS_NAME="final"
SRC_DIR="$WS_NAME/src"

# Node package names
NODES=(
  "tello_controller_node"
  "yolov11_detector_node"
  "apriltag_detector_node"
  "navigation_node"
  "mission_manager_node"
)

# Create workspace and src folder
echo "Creating ROS 2 workspace: $WS_NAME"
mkdir -p "$SRC_DIR"
cd "$WS_NAME" || exit

# Initialize workspace
colcon build 2>/dev/null

# Create each node package
cd src || exit
for NODE in "${NODES[@]}"; do
  echo "Creating package: $NODE"
  ros2 pkg create --build-type ament_python --dependencies rclpy "$NODE"
done

# Add __init__.py to each package
for NODE in "${NODES[@]}"; do
  touch "$NODE/$NODE/__init__.py"
done

# Create a .gitignore and README.md
cd ..
echo -e "build/\ninstall/\nlog/" > .gitignore
echo "# $WS_NAME ROS 2 Workspace" > README.md

# Final message
echo "ROS 2 workspace '$WS_NAME' setup complete."
echo "Next steps:"
echo "  1. source /opt/ros/foxy/setup.bash"
echo "  2. cd $WS_NAME && colcon build"
echo "  3. source install/setup.bash"
