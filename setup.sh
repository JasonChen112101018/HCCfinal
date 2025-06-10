#!/bin/bash

# 設定工作區名稱
WORKSPACE_NAME=final
SRC_DIR=$WORKSPACE_NAME/src

# 建立工作區與src資料夾
mkdir -p $SRC_DIR
cd $WORKSPACE_NAME

# 初始化 colcon 工作區
colcon build
source install/setup.bash

# 定義每個 package 與其程式語言
declare -A packages=(
  ["tello_controller_node"]="py"
  ["yolov11_detector_node"]="py"
  ["apriltag_detector_node"]="py"
  ["navigation_node"]="py"
  ["mission_manager_node"]="py"
)

# 建立每個 package
cd src
for pkg in "${!packages[@]}"; do
  lang=${packages[$pkg]}
  if [ "$lang" == "py" ]; then
    ros2 pkg create --build-type ament_python --dependencies rclpy std_msgs $pkg
    mkdir -p $pkg/$pkg
    touch $pkg/$pkg/__init__.py
    cat <<EOF > $pkg/$pkg/main.py
import rclpy
from rclpy.node import Node

class ${pkg^}Node(Node):
    def __init__(self):
        super().__init__('${pkg}_node')
        self.get_logger().info('${pkg} node started')

def main(args=None):
    rclpy.init(args=args)
    node = ${pkg^}Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
    # 更新 setup.py 入口點
    sed -i "/entry_points=\{/a \        'console_scripts': ['${pkg}_node = ${pkg}.main:main']," $pkg/setup.py
  else
    ros2 pkg create --build-type ament_cmake --dependencies rclcpp std_msgs $pkg
    mkdir -p $pkg/src
    cat <<EOF > $pkg/src/main.cpp
#include "rclcpp/rclcpp.hpp"

class ${pkg^}Node : public rclcpp::Node {
public:
  ${pkg^}Node() : Node("${pkg}_node") {
    RCLCPP_INFO(this->get_logger(), "${pkg} node started");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<${pkg^}Node>());
  rclcpp::shutdown();
  return 0;
}
EOF
    # 修改 CMakeLists.txt 添加執行檔
    echo -e "\nadd_executable(${pkg}_node src/main.cpp)" >> $pkg/CMakeLists.txt
    echo "ament_target_dependencies(${pkg}_node rclcpp std_msgs)" >> $pkg/CMakeLists.txt
    echo "install(TARGETS ${pkg}_node DESTINATION lib/\${PROJECT_NAME})" >> $pkg/CMakeLists.txt
  fi
done

echo "ROS 2 workspace and packages created under $WORKSPACE_NAME/"
