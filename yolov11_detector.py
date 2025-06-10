import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import torch
import numpy as np
import cv2
import time
from pathlib import Path

class YoloV11DetectorNode(Node):
    def __init__(self):
        super().__init__('yolov11_detector')

        # 模型與設備
        model_path = str(Path(__file__).parent / 'yolov11_weights.pt')
        self.model = torch.load(model_path, map_location='cpu')
        self.model.eval()
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model.to(self.device)

        # 模型前處理參數
        self.input_size = 640
        self.conf_threshold = 0.6

        # 類別名稱（請依你的模型調整）
        self.class_names = ['class1', 'class2', 'class3', 'class4']

        # 穩定偵測狀態
        self.should_detect = False
        self.last_seen_class = None
        self.detection_start_time = None

        # ROS 工具
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/tello/image', self.image_callback, 10)
        self.detect_trigger_sub = self.create_subscription(String, '/yolo/detect_trigger', self.trigger_callback, 10)
        self.detection_pub = self.create_publisher(String, '/yolo/detection', 10)
        self.annotated_image_pub = self.create_publisher(Image, '/yolo/annotated_image', 10)

        self.get_logger().info("✅ YOLOv11 Detector Node with stability filter initialized")

    def trigger_callback(self, msg: String):
        if msg.data == "start":
            self.should_detect = True
            self.last_seen_class = None
            self.detection_start_time = None
            self.get_logger().info("🟢 YOLO detection started")
        elif msg.data == "stop":
            self.should_detect = False
            self.get_logger().info("🔴 YOLO detection stopped")

    def image_callback(self, msg: Image):
        if not self.should_detect:
            return

        try:
            # ROS Image → OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            orig_h, orig_w = cv_image.shape[:2]

            # 預處理
            resized = cv2.resize(cv_image, (self.input_size, self.input_size))
            img_tensor = torch.from_numpy(resized).float().permute(2, 0, 1) / 255.0
            img_tensor = img_tensor.unsqueeze(0).to(self.device)

            # 推論
            with torch.no_grad():
                outputs = self.model(img_tensor)[0]

            detections = outputs.cpu().numpy()

            best_class = None
            best_conf = 0
            best_box = None

            for det in detections:
                conf = det[4]
                class_id = int(det[5])
                if conf >= self.conf_threshold:
                    if conf > best_conf:
                        best_class = self.class_names[class_id]
                        best_conf = conf
                        best_box = det[:4]

            now = time.time()

            if best_class:
                if best_class == self.last_seen_class:
                    if self.detection_start_time is None:
                        self.detection_start_time = now
                    elif now - self.detection_start_time >= 3.0:
                        # 發布穩定偵測結果
                        msg_out = String()
                        msg_out.data = f"{best_class}:{best_conf:.2f}"
                        self.detection_pub.publish(msg_out)
                        self.get_logger().info(f"✅ Confirmed detection: {best_class} ({best_conf:.2f})")
                        self.should_detect = False
                else:
                    self.last_seen_class = best_class
                    self.detection_start_time = now
            else:
                self.last_seen_class = None
                self.detection_start_time = None

            # 可視化框框
            if best_box is not None:
                x1, y1, x2, y2 = [int(coord * orig_w / self.input_size) if i % 2 == 0 else int(coord * orig_h / self.input_size)
                                  for i, coord in enumerate(best_box)]
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(cv_image, f"{best_class} {best_conf:.2f}", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            annotated_msg.header = msg.header
            self.annotated_image_pub.publish(annotated_msg)

        except Exception as e:
            self.get_logger().error(f"[YOLOv11] Error: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloV11DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("YOLOv11 node interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
