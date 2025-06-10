#!/bin/bash

echo "🔧 Setting up ROS 2 Foxy workspace for Tello..."

# 1. 建立 workspace 資料夾
mkdir -p ~/final/src
cd ~/final

# 2. 套件名稱清單
NODES=(
  "tello_controller_node"
  "yolov11_detector_node"
  "apriltag_detector_node"
  "navigation_node"
  "mission_manager_node"
)

# 3. 建立每個套件結構
for NODE in "${NODES[@]}"; do
  echo "📁 Creating package: $NODE"
  PACKAGE_DIR="src/$NODE"
  mkdir -p $PACKAGE_DIR/$NODE

  # setup.py
  cat > $PACKAGE_DIR/setup.py <<EOF
from setuptools import setup

package_name = '$NODE'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    py_modules=['$NODE'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    description='$NODE for ROS 2 Tello Project',
    entry_points={
        'console_scripts': [
            '$NODE = $NODE:main',
        ],
    },
)
EOF

  # package.xml
  cat > $PACKAGE_DIR/package.xml <<EOF
<package format="3">
  <name>$NODE</name>
  <version>0.1.0</version>
  <description>Auto-generated $NODE package</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>MIT</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
</package>
EOF

  # Python module init
  touch $PACKAGE_DIR/$NODE/__init__.py

  # 建立主程式入口檔
  cat > $PACKAGE_DIR/$NODE/$NODE.py <<EOF
def main():
    print("✅ $NODE started (stub). Please implement the logic.")
EOF
done

# 4. 編譯工作區
echo "🔨 Building workspace..."
source /opt/ros/foxy/setup.bash
colcon build

# 5. 提示使用者
echo "✅ Workspace setup complete!"
echo "👉 Don't forget to source it each time:"
echo "   source ~/ros2_ws/install/setup.bash"
