import rclpy
from rclpy.node import Node
from djitellopy import Tello
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Point, PoseStamped
from std_msgs.msg import String, Bool, Float32MultiArray
from cv_bridge import CvBridge
import threading
import time

class TelloControllerNode(Node):
    def __init__(self):
        super().__init__('tello_controller')
        
        # 建立 Tello 實例並連線
        self.tello = Tello()
        self.tello.connect()
        self.tello.enable_mission_pads()
        self.tello.set_mission_pad_detection_direction(0)  # 往下偵測

        # 發布者
        self.image_pub = self.create_publisher(Image, '/tello/image', 10)
        self.battery_pub = self.create_publisher(Float32MultiArray, '/tello/battery', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/tello/pose', 10)

        # 訂閱者
        self.cmd_vel_sub = self.create_subscription(Twist, '/tello/cmd_vel', self.cmd_vel_callback, 10)
        self.takeoff_sub = self.create_subscription(Bool, '/tello/takeoff', self.takeoff_callback, 10)
        self.land_sub = self.create_subscription(Bool, '/tello/land', self.land_callback, 10)
        self.move_to_sub = self.create_subscription(Point, '/tello/move_to', self.move_to_callback, 10)

        # 影像轉換工具
        self.bridge = CvBridge()

        # 狀態標記
        self.is_flying = False

        # 啟動影像串流
        self.tello.streamon()

        # 計時器每 0.1 秒發佈資料
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("✅ Tello Controller Node initialized")

    def timer_callback(self):
        # 發布攝影機影像
        frame = self.tello.get_frame_read().frame
        if frame is not None:
            img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            img_msg.header.stamp = self.get_clock().now().to_msg()
            self.image_pub.publish(img_msg)

        # 發布電池電量
        battery = self.tello.get_battery()
        battery_msg = Float32MultiArray()
        battery_msg.data = [float(battery)]
        self.battery_pub.publish(battery_msg)

        # 發布相對座標（若偵測到 Mission Pad / AprilTag）
        try:
            pad_id = self.tello.get_mission_pad_id()
            if pad_id != -1:
                x = self.tello.get_mission_pad_distance_x()
                y = self.tello.get_mission_pad_distance_y()
                z = self.tello.get_mission_pad_distance_z()

                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = f"apriltag_{pad_id}"
                pose_msg.pose.position.x = float(x)
                pose_msg.pose.position.y = float(y)
                pose_msg.pose.position.z = float(z)
                self.pose_pub.publish(pose_msg)
        except:
            pass

    def takeoff_callback(self, msg: Bool):
        if msg.data and not self.is_flying:
            self.tello.takeoff()
            self.is_flying = True
            self.get_logger().info("🚁 Takeoff initiated")

    def land_callback(self, msg: Bool):
        if msg.data and self.is_flying:
            self.tello.land()
            self.is_flying = False
            self.get_logger().info("🛬 Landing initiated")

    def cmd_vel_callback(self, msg: Twist):
        if self.is_flying:
            # 將 ROS Twist 轉換為 Tello 指令
            lr = int(msg.linear.y * 100)  # 左右
            fb = int(msg.linear.x * 100)  # 前後
            ud = int(msg.linear.z * 100)  # 上下
            yaw = int(msg.angular.z * 100)  # 旋轉

            self.tello.send_rc_control(lr, fb, ud, yaw)

    def move_to_callback(self, msg: Point):
        if self.is_flying:
            # 讓 Tello 飛往指定座標（單位：cm）
            x = int(msg.x)
            y = int(msg.y)
            z = int(msg.z)
            self.tello.go_xyz_speed(x, y, z, 50)
            self.get_logger().info(f"🎯 Moving to: x={x} y={y} z={z}")

    def destroy_node(self):
        # 結束時關閉串流與降落
        self.tello.streamoff()
        if self.is_flying:
            self.tello.land()
        self.tello.end()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TelloControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Tello Controller...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
