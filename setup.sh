#!/bin/bash

set -e

echo "=== 初始化無人機任務專案 Workspace ==="

WORKSPACE=~/workspace
SRC_DIR=$WORKSPACE/src

mkdir -p $SRC_DIR
cd $WORKSPACE

# 初始化各 Node
for pkg in tello_controller_node yolov11_detector_node apriltag_detector_node navigation_node mission_manager_node; do
    echo "建立套件: $pkg"
    ros2 pkg create $pkg --build-type ament_python --dependencies rclpy std_msgs sensor_msgs geometry_msgs nav_msgs
done

# 建立虛擬環境（可選）
# python3 -m venv $WORKSPACE/venv
# source $WORKSPACE/venv/bin/activate

echo "=== 安裝 Python 套件 ==="
pip install --upgrade pip
pip install opencv-python torch torchvision numpy filterpy apriltag

echo "=== 建立初始執行檔範例 ==="
for pkg in tello_controller_node yolov11_detector_node apriltag_detector_node navigation_node mission_manager_node; do
    ENTRY_FILE=$SRC_DIR/$pkg/$pkg/$pkg.py
    mkdir -p $(dirname $ENTRY_FILE)
    cat <<EOF > $ENTRY_FILE
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class ${pkg^}Node(Node):
    def __init__(self):
        super().__init__('${pkg}_node')
        self.get_logger().info("Node ${pkg} 啟動中...")

def main(args=None):
    rclpy.init(args=args)
    node = ${pkg^}Node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
    chmod +x $ENTRY_FILE
done

echo "=== 建立 setup.py & package.xml 調整 ==="
for pkg in tello_controller_node yolov11_detector_node apriltag_detector_node navigation_node mission_manager_node; do
    SETUP_FILE=$SRC_DIR/$pkg/setup.py
    PACKAGE_NAME=$pkg

    cat <<EOF > $SETUP_FILE
from setuptools import setup

package_name = '${PACKAGE_NAME}'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YOUR_NAME',
    maintainer_email='YOUR_EMAIL@example.com',
    description='${PACKAGE_NAME} node for UAV mission',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            '${PACKAGE_NAME}_node = ${PACKAGE_NAME}.${PACKAGE_NAME}:main',
        ],
    },
)
EOF
done

echo "=== 建構 Workspace ==="
cd $WORKSPACE
colcon build --symlink-install

echo "=== 完成！請加入你的功能在每個 Node 中 ==="
echo "啟用 workspace: source ~/workspace/install/setup.bash"
