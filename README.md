現在有個撰寫python控制無人機(tello)的任務，目標是這樣的，起飛之後無人機鏡頭會先偵測某個圖片，然後使用我自己預先訓練好的yolo模型去辨識圖片物體的類別，按照類別等一下會有不同的要求降落地點，實驗的場地是一個3*3的九宮格，在九宮格四個邊上各有兩個apriltag,它們各自的座標是已知的，你必須使用無人機與apriltag的相對座標去計算出自己的方位（x,y, 不用高度），接著，場地內有三個不知道位置為何的需偵測apriltag，你需要偵測此apriltag, 並利用無人機與此的相對位置算出此apriltag的位置（x,y, 不用高度），最後，降落到一開始辨識物體時規定的降落地點上（九宮格的其中一格），事先不會知道圖片是什麼、待測apriltag會擺放在哪裡，飛行計畫是：takeoff、yolov11偵測圖片、往前飛特定距離、偵測已知和未知apriltag、降落到指定位置。

建議的 ROS /foxy架構實作

workspace/
├── src/
│   ├── tello_controller_node/    # 控制無人機飛行
│   ├── yolov11_detector_node/     # 執行辨識任務
│   ├── apriltag_detector_node/   # 偵測並回傳 Tag Pose
│   ├── navigation_node/          # 路徑規劃與執行
│   └── mission_manager_node/     # 主控流程邏輯
