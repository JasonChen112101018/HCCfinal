import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32MultiArray
from geometry_msgs.msg import Point
import time

class MissionManagerNode(Node):
    def __init__(self):
        super().__init__('mission_manager')

        # Publisher
        self.takeoff_pub = self.create_publisher(Bool, '/tello/takeoff', 10)
        self.land_pub = self.create_publisher(Bool, '/tello/land', 10)
        self.cmd_pub = self.create_publisher(String, '/navigation/command', 10)
        self.yolo_trigger_pub = self.create_publisher(String, '/yolo/detect_trigger', 10)
        self.move_to_pub = self.create_publisher(Point, '/tello/move_to', 10)
        self.rotate_pub = self.create_publisher(String, '/tello/rotate', 10)  # Optional custom topic

        # Subscriber
        self.yolo_result_sub = self.create_subscription(String, '/yolo/detection', self.yolo_callback, 10)
        self.position_sub = self.create_subscription(Point, '/navigation/drone_position', self.position_callback, 10)
        self.tag_pos_sub = self.create_subscription(Point, '/navigation/unknown_tag_position', self.unknown_tag_callback, 10)
        self.battery_sub = self.create_subscription(Float32MultiArray, '/tello/battery', self.battery_callback, 10)

        # 狀態
        self.state = "IDLE"
        self.start_time = time.time()
        self.class_result = None
        self.drone_position = None
        self.unknown_tags_found = set()
        self.battery_level = 100

        # 類別對應格子 ID
        self.class_to_grid = {
            "class1": 0,
            "class2": 2,
            "class3": 6,
            "class4": 8
        }

        self.timer = self.create_timer(1.0, self.update)
        self.get_logger().info("✅ Mission Manager initialized")
        time.sleep(2)
        self.start_mission()

    def start_mission(self):
        self.state = "TAKEOFF"
        self.start_time = time.time()
        self.get_logger().info("🚁 Mission started")

    def update(self):
        now = time.time()
        elapsed = now - self.start_time

        if self.state == "TAKEOFF":
            self.get_logger().info("🚀 Taking off")
            self.takeoff_pub.publish(Bool(data=True))
            if elapsed > 5:
                self.state = "DETECT_OBJECT"
                self.start_time = now

        elif self.state == "DETECT_OBJECT":
            self.get_logger().info("🎯 Starting object detection")
            self.yolo_trigger_pub.publish(String(data="start"))
            if self.class_result:
                self.get_logger().info(f"✅ Object Detected: {self.class_result}")
                self.state = "MOVE_FORWARD"
                self.start_time = now
            elif elapsed > 10:
                self.get_logger().warn("❌ YOLO timeout, retrying")
                self.start_time = now

        elif self.state == "MOVE_FORWARD":
            self.get_logger().info("➡️ Moving forward (assumed center)")
            forward = Point(x=100, y=0, z=0)  # 飛行 100cm 向前
            self.move_to_pub.publish(forward)
            if elapsed > 5:
                self.state = "ROTATE_AND_SCAN"
                self.rotation_step = 0
                self.start_time = now

        elif self.state == "ROTATE_AND_SCAN":
            if self.rotation_step < 8:
                angle = self.rotation_step * 45
                self.get_logger().info(f"🔄 Rotating {angle}°")
                rotate_msg = String()
                rotate_msg.data = f"rotate:{angle}"
                self.cmd_pub.publish(rotate_msg)
                self.rotation_step += 1
                self.start_time = now
            else:
                self.state = "WAIT_FOR_LOCALIZATION"
                self.start_time = now

        elif self.state == "WAIT_FOR_LOCALIZATION":
            if self.drone_position:
                self.get_logger().info(f"📍 Localized at ({self.drone_position.x:.1f}, {self.drone_position.y:.1f})")
                self.state = "NAVIGATE_TO_LANDING"
                self.start_time = now
            elif elapsed > 10:
                self.get_logger().warn("⚠️ Localization timeout, proceeding anyway")
                self.state = "NAVIGATE_TO_LANDING"
                self.start_time = now

        elif self.state == "NAVIGATE_TO_LANDING":
            if self.class_result in self.class_to_grid:
                grid_id = self.class_to_grid[self.class_result]
                cmd = String()
                cmd.data = f"move_to_grid:{grid_id}"
                self.cmd_pub.publish(cmd)
                self.get_logger().info(f"🧭 Navigating to grid {grid_id}")
                self.state = "LAND"
                self.start_time = now
            else:
                self.get_logger().error("❌ Unknown class result")
                self.state = "LAND"
                self.start_time = now

        elif self.state == "LAND":
            if elapsed > 5:
                self.get_logger().info("🛬 Landing")
                self.land_pub.publish(Bool(data=True))
                self.state = "DONE"

        elif self.state == "DONE":
            self.get_logger().info("✅ Mission complete")

    def yolo_callback(self, msg: String):
        parts = msg.data.split(":")
        self.class_result = parts[0]

    def position_callback(self, msg: Point):
        self.drone_position = msg

    def unknown_tag_callback(self, msg: Point):
        self.unknown_tags_found.add(int(msg.z))

    def battery_callback(self, msg: Float32MultiArray):
        if msg.data:
            self.battery_level = msg.data[0]
            if self.battery_level < 15:
                self.get_logger().error("🔋 Battery critical! Landing")
                self.land_pub.publish(Bool(data=True))
                self.state = "DONE"

def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
