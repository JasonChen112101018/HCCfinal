#!/bin/bash

# === 1. 建立 workspace ===
WORKSPACE=~/workspace
SRC=$WORKSPACE/src
mkdir -p $SRC
cd $SRC

# === 2. 建立五個 Python package ===
declare -a packages=(
  tello_controller_node
  yolov11_detector_node
  apriltag_detector_node
  navigation_node
  mission_manager_node
)

for pkg in "${packages[@]}"; do
  ros2 pkg create --build-type ament_python $pkg
done

# === 3. 初始化每個 Python package ===
for pkg in "${packages[@]}"; do
  cd "$SRC/$pkg"

  # === package.xml: 添加依賴 + build type ===
  sed -i '/<\/package>/i\
  <build_type>ament_python</build_type>\n\
  <exec_depend>rclpy</exec_depend>\n\
  <exec_depend>std_msgs</exec_depend>\n\
  <exec_depend>geometry_msgs</exec_depend>\n\
  <exec_depend>sensor_msgs</exec_depend>\n\
  <exec_depend>nav_msgs</exec_depend>' package.xml

  # === 建立 Python 模組 ===
  mkdir -p $pkg
  touch $pkg/__init__.py

  cat <<EOF > $pkg/main.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalNode(Node):
    def __init__(self):
        super().__init__('${pkg}_node')
        self.publisher_ = self.create_publisher(String, 'status', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello from ${pkg}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
EOF

  # === setup.py: 設定 console_scripts + data_files ===
  cat <<EOF > setup.py
from setuptools import setup

package_name = '$pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Auto-generated package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            '${pkg}_main = $pkg.main:main',
        ],
    },
)
EOF

  # === setup.cfg ===
  echo -e "[develop]\nscript_dir=\n[install]\ninstall_scripts=" > setup.cfg

  # === 建立 resource index 檔案 ===
  mkdir -p resource
  touch resource/$pkg
done

# === 4. 回到 workspace 並 build ===
cd $WORKSPACE
colcon build
