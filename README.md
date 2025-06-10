建議的 ROS /foxy架構實作

workspace/
├── src/
│   ├── tello_controller_node/    # 控制無人機飛行
│   ├── yolov5_detector_node/     # 執行辨識任務
│   ├── apriltag_detector_node/   # 偵測並回傳 Tag Pose
│   ├── navigation_node/          # 路徑規劃與執行
│   └── mission_manager_node/     # 主控流程邏輯
