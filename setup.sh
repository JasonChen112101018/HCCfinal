#!/bin/bash

# Exit on error
set -e

echo "=== Installing ROS 2 Foxy ==="

# Setup locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 repo
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS 2
sudo apt update
sudo apt install -y ros-foxy-desktop python3-colcon-common-extensions python3-rosdep python3-argcomplete

# Source ROS 2 in bashrc
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Initialize rosdep
sudo rosdep init || true
rosdep update

echo "=== Creating Workspace Structure ==="

# Create workspace
mkdir -p ~/workspace/src
cd ~/workspace

# Clone workspace nodes
cd src
# Replace with your actual repositories or initialize with template
for pkg in tello_controller_node yolov11_detector_node apriltag_detector_node navigation_node mission_manager_node; do
    mkdir -p "$pkg"/src
    touch "$pkg"/src/__init__.py  # Placeholder
    echo "Created package: $pkg"
done

cd ~/workspace
colcon build --symlink-install

echo "=== Installing Python dependencies ==="

# Install YOLOv11 and other dependencies in venv (recommended)
sudo apt install -y python3-pip python3-venv python3-opencv

python3 -m pip install --upgrade pip
pip install numpy opencv-python torch torchvision
pip install filterpy # For Kalman filter
pip install yolov5  # temporary fallback if YOLOv11 not pip-installable
pip install apriltag # Pure python wrapper (alternative: use ROS C++ apriltag_ros)

echo "=== Setup Complete ==="
echo "Don't forget to source your workspace: source ~/workspace/install/setup.bash"
